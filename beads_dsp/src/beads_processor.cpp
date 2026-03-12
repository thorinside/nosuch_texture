#include "beads_processor.h"
#include "util/dsp_utils.h"
#include <cstdint>
#include <cstring>
#include <cmath>
#include <new>
#include <algorithm>

namespace beads {

static constexpr size_t kImplAlignment = 16;

static size_t AlignUp(size_t size, size_t alignment = kImplAlignment) {
    return (size + alignment - 1) & ~(alignment - 1);
}

BeadsProcessor::MemoryRequirements BeadsProcessor::GetMemoryRequirements(float sample_rate) {
    MemoryRequirements req;

    size_t impl_bytes = AlignUp(sizeof(Impl));
    size_t recording_bytes = RecordingBuffer::RequiredBytes(
        sample_rate, kDefaultBufferDuration, 2);
    size_t reverb_bytes = kReverbBufferSize * sizeof(float);

    // Add alignment padding so Init() can align the base pointer internally
    // even if the caller provides unaligned memory.
    req.total_bytes = (kImplAlignment - 1) + impl_bytes + AlignUp(recording_bytes) + AlignUp(reverb_bytes);
    req.alignment = kImplAlignment;
    return req;
}

void BeadsProcessor::Init(void* memory, size_t memory_size, float sample_rate) {
    if (!memory || memory_size == 0) return;

    // Verify the caller provided enough memory.
    auto req = GetMemoryRequirements(sample_rate);
    if (memory_size < req.total_bytes) return;

    // Align the base pointer to the required alignment before placement-new.
    uintptr_t addr = reinterpret_cast<uintptr_t>(memory);
    addr = (addr + kImplAlignment - 1) & ~(kImplAlignment - 1);
    uint8_t* ptr = reinterpret_cast<uint8_t*>(addr);

    // Placement-new the Impl at the aligned start of the memory block
    impl_ = new (ptr) Impl();
    ptr += AlignUp(sizeof(Impl));

    impl_->sample_rate = sample_rate;
    impl_->params = BeadsParameters{};
    impl_->feedback_sample = {0.0f, 0.0f};
    impl_->prev_freeze = false;
    impl_->delay_mode = false;

    // Allocate recording buffer
    size_t recording_bytes = RecordingBuffer::RequiredBytes(
        sample_rate, kDefaultBufferDuration, 2);
    size_t num_frames = static_cast<size_t>(sample_rate * kDefaultBufferDuration);
    impl_->recording_buffer.Init(reinterpret_cast<float*>(ptr), num_frames, 2);
    ptr += AlignUp(recording_bytes);

    // Allocate reverb delay memory
    impl_->reverb.Init(reinterpret_cast<float*>(ptr), kReverbBufferSize, sample_rate);
    ptr += AlignUp(kReverbBufferSize * sizeof(float));

    // Initialize sub-processors
    impl_->grain_engine.Init(sample_rate, &impl_->recording_buffer);
    impl_->delay_engine.Init(sample_rate, &impl_->recording_buffer);
    impl_->saturation.Init();
    impl_->quality_processor.Init(sample_rate);
    impl_->auto_gain.Init(sample_rate);
    impl_->wavetable_osc.Init(sample_rate);

    // Feedback HP filter at ~20Hz to remove DC
    impl_->feedback_hp_l.Init();
    impl_->feedback_hp_l.SetFrequencyHz(20.0f, sample_rate);
    impl_->feedback_hp_l.SetQ(0.707f);
    impl_->feedback_hp_r.Init();
    impl_->feedback_hp_r.SetFrequencyHz(20.0f, sample_rate);
    impl_->feedback_hp_r.SetQ(0.707f);
}

void BeadsProcessor::SetWavetableProvider(WavetableProvider* provider) {
    if (!impl_) return;
    impl_->wavetable_osc.SetProvider(provider);
}

void BeadsProcessor::SetParameters(const BeadsParameters& params) {
    if (!impl_) return;
    impl_->params = params;

    bool new_delay_mode = (params.size >= 1.0f);
    if (new_delay_mode != impl_->delay_mode) {
        impl_->prev_delay_mode = impl_->delay_mode;
        impl_->delay_mode = new_delay_mode;
        impl_->mode_xfade_counter = Impl::kModeXfadeSamples;
    }

    // Configure reverb from parameters
    impl_->reverb.SetAmount(params.reverb);
    impl_->reverb.SetDecay(0.3f + params.reverb * 0.65f);
    impl_->reverb.SetDiffusion(0.7f);

    // Quality mode affects reverb LP: Tape=warmest, HiFi=brightest
    float reverb_lp;
    switch (params.quality_mode) {
        case QualityMode::kTape:      reverb_lp = 0.3f; break;
        case QualityMode::kCleanLoFi: reverb_lp = 0.5f; break;
        case QualityMode::kClouds:    reverb_lp = 0.6f; break;
        default:                      reverb_lp = 0.7f; break;
    }
    impl_->reverb.SetLpCutoff(reverb_lp);
}

void BeadsProcessor::Process(const StereoFrame* input, StereoFrame* output,
                              size_t num_frames) {
    if (!impl_) {
        for (size_t i = 0; i < num_frames; ++i) {
            output[i] = {0.0f, 0.0f};
        }
        return;
    }
    auto& s = *impl_;  // shorthand

    // Detect freeze transitions for crossfade
    if (s.params.freeze != s.prev_freeze) {
        s.recording_buffer.StartFreezeCrossfade();
        s.prev_freeze = s.params.freeze;
    }

    // Precompute wavetable fade increment for this block
    float wt_fade_inc = 1.0f / static_cast<float>(Impl::kWavetableXfadeSamples);

    // --- Per-sample input processing (steps 1-4) ---
    for (size_t i = 0; i < num_frames; ++i) {
        StereoFrame in = input[i];

        // Check for wavetable mode activation (silence detection)
        // Uses a smooth fade to avoid clicks on wavetable transitions
        bool wt_wants_active = s.wavetable_osc.ShouldActivate(&in, 1);
        if (wt_wants_active) {
            // Fade wavetable in
            s.wavetable_fade = std::min(s.wavetable_fade + wt_fade_inc, 1.0f);
            StereoFrame wt_out;
            s.wavetable_osc.Process(s.params.pitch, s.params.feedback, &wt_out, 1);
            // Crossfade between input and wavetable
            in.l = in.l * (1.0f - s.wavetable_fade) + wt_out.l * s.wavetable_fade;
            in.r = in.r * (1.0f - s.wavetable_fade) + wt_out.r * s.wavetable_fade;
        } else {
            if (s.wavetable_fade > 0.0f) {
                // Fade wavetable out before deactivating
                s.wavetable_fade = std::max(s.wavetable_fade - wt_fade_inc, 0.0f);
                StereoFrame wt_out;
                s.wavetable_osc.Process(s.params.pitch, s.params.feedback, &wt_out, 1);
                in.l = in.l * (1.0f - s.wavetable_fade) + wt_out.l * s.wavetable_fade;
                in.r = in.r * (1.0f - s.wavetable_fade) + wt_out.r * s.wavetable_fade;
                if (s.wavetable_fade <= 0.0f) {
                    s.wavetable_osc.Deactivate();
                }
            }
        }

        // 1. Auto-gain
        in = s.auto_gain.Process(in, s.params.manual_gain_db);

        // 2. Quality input processing
        in = s.quality_processor.ProcessInput(in, s.params.quality_mode);

        // 3. Feedback mix (smoothed to prevent zipper noise)
        ONE_POLE(s.smoothed_feedback, s.params.feedback, 0.002f);
        float feedback_gain = s.smoothed_feedback * s.smoothed_feedback;
        StereoFrame fb = {
            s.feedback_hp_l.ProcessHP(s.feedback_sample.l),
            s.feedback_hp_r.ProcessHP(s.feedback_sample.r)
        };
        in += fb * feedback_gain;
        in = s.saturation.LimitFeedback(in, s.params.quality_mode);

        // 4. Record to buffer (unless frozen)
        if (!s.params.freeze) {
            s.recording_buffer.Write(in);
        }
        if (s.recording_buffer.crossfading()) {
            s.recording_buffer.ProcessFreezeCrossfade();
        }

        output[i] = {0.0f, 0.0f};
    }

    // --- Block-based wet signal generation + output processing (steps 5-10) ---
    // Process in blocks of kMaxBlockSize to avoid stack overflow when
    // num_frames > kMaxBlockSize.
    StereoFrame wet[kMaxBlockSize];
    StereoFrame wet_alt[kMaxBlockSize];  // For mode crossfade: the outgoing engine
    size_t remaining = num_frames;
    size_t offset = 0;

    // If we are in a mode crossfade, we need both engines' output
    bool crossfading_modes = (s.mode_xfade_counter > 0);

    while (remaining > 0) {
        size_t block = std::min(remaining, kMaxBlockSize);

        // Tape mode wow/flutter: compute pitch modulation for this block.
        // The modulation is very slow (0.5Hz wow) so one value per block is fine.
        float pitch_mod = s.quality_processor.GetPitchModulation(s.params.quality_mode, block);
        s.grain_engine.SetPitchModulation(pitch_mod);
        s.delay_engine.SetPitchModulation(pitch_mod);

        if (crossfading_modes) {
            // Run both engines and crossfade between them
            s.delay_engine.Process(s.params, s.delay_mode ? wet : wet_alt, block);
            s.grain_engine.Process(s.params, s.delay_mode ? wet_alt : wet, block);
            // wet = new (current) engine, wet_alt = old (outgoing) engine
        } else {
            if (s.delay_mode) {
                s.delay_engine.Process(s.params, wet, block);
            } else {
                s.grain_engine.Process(s.params, wet, block);
            }
        }

        // Per-sample output processing for this block
        for (size_t i = 0; i < block; ++i) {
            StereoFrame wet_frame;

            if (s.mode_xfade_counter > 0) {
                // Crossfade from old engine (wet_alt) to new engine (wet)
                float xfade = static_cast<float>(s.mode_xfade_counter)
                            / static_cast<float>(Impl::kModeXfadeSamples);
                // xfade goes from 1.0 (all old) down to 0.0 (all new)
                wet_frame.l = wet_alt[i].l * xfade + wet[i].l * (1.0f - xfade);
                wet_frame.r = wet_alt[i].r * xfade + wet[i].r * (1.0f - xfade);
                s.mode_xfade_counter--;
                if (s.mode_xfade_counter == 0) {
                    crossfading_modes = false;
                }
            } else {
                wet_frame = wet[i];
            }

            // 6. Quality output processing
            wet_frame = s.quality_processor.ProcessOutput(wet_frame, s.params.quality_mode);

            // 7. Capture feedback sample (before reverb)
            s.feedback_sample = wet_frame;

            // 8. Dry/wet crossfade (equal-power) -- smoothed to avoid zipper noise
            ONE_POLE(s.smoothed_dry_wet, s.params.dry_wet, 0.002f);
            StereoFrame mixed = EqualPowerCrossfade(input[offset + i], wet_frame, s.smoothed_dry_wet);

            // 9. Reverb
            float rev_l, rev_r;
            s.reverb.Process(mixed.l, mixed.r, &rev_l, &rev_r);
            mixed = {rev_l, rev_r};

            // 10. Output
            output[offset + i] = mixed;
        }

        offset += block;
        remaining -= block;
    }
}

bool BeadsProcessor::IsDelayMode() const {
    return impl_ ? impl_->delay_mode : false;
}

bool BeadsProcessor::IsWavetableMode() const {
    return impl_ ? impl_->wavetable_osc.IsActive() : false;
}

int BeadsProcessor::ActiveGrainCount() const {
    return impl_ ? impl_->grain_engine.ActiveGrainCount() : 0;
}

float BeadsProcessor::InputLevel() const {
    return impl_ ? impl_->auto_gain.InputLevel() : 0.0f;
}

} // namespace beads
