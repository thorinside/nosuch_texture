#include "grain_engine.h"
#include "../buffer/recording_buffer.h"
#include "../util/dsp_utils.h"

#include <cmath>
#include <algorithm>

namespace beads {

void GrainEngine::Init(float sample_rate, RecordingBuffer* buffer) {
    sample_rate_ = sample_rate;
    buffer_ = buffer;
    overlap_count_lp_ = 0.0f;
    gain_normalization_ = 1.0f;

    for (int i = 0; i < kMaxGrains; ++i) {
        grains_[i].Init();
    }

    scheduler_.Init(sample_rate);
    random_.Init(0x68A14CED);

    ar_time_.Init(&random_);
    ar_size_.Init(&random_);
    ar_shape_.Init(&random_);
    ar_pitch_.Init(&random_);
}

int GrainEngine::ActiveGrainCount() const {
    int count = 0;
    for (int i = 0; i < kMaxGrains; ++i) {
        if (grains_[i].active()) ++count;
    }
    return count;
}

Grain* GrainEngine::AllocateGrain() {
    // First pass: find an inactive grain.
    for (int i = 0; i < kMaxGrains; ++i) {
        if (!grains_[i].active()) {
            return &grains_[i];
        }
    }

    // Pool is full — mark the oldest grain for zero-crossing kill.
    // The grain will be killed at the next zero crossing (or after a
    // short fallback fade), avoiding clicks without smearing transients.
    // We do NOT return the grain for immediate reuse; the new trigger
    // is simply dropped.
    for (int i = 0; i < kMaxGrains; ++i) {
        if (!grains_[i].pending_kill()) {
            grains_[i].StartPendingKill();
            break;
        }
    }
    return nullptr;
}

Grain::GrainParameters GrainEngine::ComputeGrainParams(
        const BeadsParameters& params, int pre_delay) {
    Grain::GrainParameters gp;

    // --- SIZE → grain duration + direction ---
    // Negative size = reverse grain playback.
    //   -1.0 → long reverse grains, approaching 0 → short grains.
    // Positive size = forward grain playback.
    //   0.0 → short grains, ~0.95 → long grains, 1.0 → delay mode.
    // (At SIZE >= 1.0 the processor switches to delay mode, so
    // we don't need to handle that case here.)
    float mod_size = ar_size_.Process(params.size, params.size_ar,
                                       params.size_cv, params.size_cv_connected);
    bool reverse = (mod_size < 0.0f);
    float abs_size = std::fabs(mod_size);
    abs_size = Clamp(abs_size, 0.0f, 0.999f);

    // Exponential mapping from 0..1 to 10ms..max duration
    // Decimation extends effective buffer duration
    int df = buffer_->decimation_factor();
    float df_f = static_cast<float>(df);
    float min_dur = 0.01f;   // 10ms
    float max_dur = static_cast<float>(buffer_->size()) * df_f / sample_rate_;
    float duration = min_dur * std::pow(max_dur / min_dur, abs_size);
    gp.size = duration * sample_rate_;

    // --- TIME → buffer read position ---
    // Matches original Beads manual:
    //   0.0 (fully CCW) → most recent audio (near write head)
    //   1.0 (fully CW)  → oldest audio (far from write head)
    float mod_time = ar_time_.Process(params.time, params.time_ar,
                                       params.time_cv, params.time_cv_connected);
    mod_time = Clamp(mod_time, 0.0f, 1.0f);

    // --- PITCH → playback rate ---
    // (computed before position so we can calculate grain span for headroom)
    float mod_pitch = ar_pitch_.Process(params.pitch, params.pitch_ar,
                                         params.pitch_cv, params.pitch_cv_connected);
    mod_pitch += params.midi_pitch_offset;
    gp.pitch_ratio = SemitonesToRatio(mod_pitch) * pitch_mod_ratio_ / df_f;
    if (reverse) {
        gp.pitch_ratio = -gp.pitch_ratio;
    }

    // Convert to an absolute position in the recording buffer.
    // time=0.0 means read from write_head (newest), time=1.0 means read
    // from one full buffer behind the write_head (oldest).
    float buf_size_f = static_cast<float>(buffer_->size());
    float offset_frames = mod_time * buf_size_f;

    // Ensure the grain has enough headroom to play without reading past the
    // write head (forward) or before the oldest valid data (reverse).
    // Without this, TIME near 0 + large SIZE reads stale/unwritten data.
    float span = gp.size * std::fabs(gp.pitch_ratio);
    // Also account for write head advancing during grain lifetime
    float write_advance = gp.size / df_f;
    float min_offset = span + write_advance;
    min_offset = std::min(min_offset, buf_size_f - 1.0f);
    offset_frames = std::max(offset_frames, min_offset);

    float pos = static_cast<float>(buffer_->write_head()) - offset_frames;
    if (pos < 0.0f) pos += buf_size_f;
    gp.position = pos;

    if (reverse) {
        // Offset start position to the END of the segment a forward grain
        // would play.  The reverse grain then reads backwards through the
        // same audio, producing true reversed playback of the intended
        // segment rather than reading into unrelated older audio.
        gp.position += span;
        while (gp.position >= buf_size_f) gp.position -= buf_size_f;
    }

    // --- SHAPE → envelope shape ---
    float mod_shape = ar_shape_.Process(params.shape, params.shape_ar,
                                         params.shape_cv, params.shape_cv_connected);
    gp.shape = Clamp(mod_shape, 0.0f, 1.0f);

    // --- PAN ---
    // Slight random panning for stereo width.
    gp.pan = random_.NextBipolar() * 0.5f;

    // --- GAIN (MIDI velocity) ---
    gp.gain = params.midi_velocity_gain;

    // --- PRE-DELAY ---
    gp.pre_delay = pre_delay;

    return gp;
}

void GrainEngine::Process(const BeadsParameters& params,
                          StereoFrame* output, size_t num_frames) {
    if (!buffer_ || buffer_->size() == 0) {
        for (size_t i = 0; i < num_frames; ++i) {
            output[i] = {0.0f, 0.0f};
        }
        return;
    }

    // --- Schedule triggers ---
    static constexpr int kMaxTriggers = 32;
    int trigger_samples[kMaxTriggers];
    int num_triggers = scheduler_.Process(params, num_frames,
                                          trigger_samples, kMaxTriggers);

    // Compute a dynamic grain cap based on grain size.
    // Long grains don't benefit from heavy overlap — they play the same
    // audio and just waste CPU.  Cap = buffer_duration / grain_duration,
    // scaled by 2 for texture, clamped to [2, kMaxGrains].
    int df = buffer_->decimation_factor();
    float buf_dur = static_cast<float>(buffer_->size()) * static_cast<float>(df)
                  / sample_rate_;
    float abs_size = std::fabs(params.size);
    abs_size = Clamp(abs_size, 0.0f, 0.999f);
    float min_dur = 0.01f;
    float grain_dur = min_dur * std::pow(buf_dur / min_dur, abs_size);
    int max_active = static_cast<int>(buf_dur / grain_dur * 2.0f);
    max_active = std::max(max_active, 2);
    max_active = std::min(max_active, kMaxGrains);

    int active_before = ActiveGrainCount();

    // Start new grains at their trigger points.
    for (int t = 0; t < num_triggers; ++t) {
        if (active_before >= max_active) break;
        Grain* g = AllocateGrain();
        if (g) {
            auto gp = ComputeGrainParams(params, trigger_samples[t]);
            g->Start(gp);
            ++active_before;
        }
    }

    // --- Zero output buffer ---
    for (size_t i = 0; i < num_frames; ++i) {
        output[i] = {0.0f, 0.0f};
    }

    // --- Render grains: grain-major order for cache locality ---
    // Processing each grain across the full block keeps buffer reads
    // sequential (consecutive samples read adjacent memory), which is
    // much faster than sample-major order where each sample touches
    // all active grain positions and thrashes L1 cache.
    int active_count = 0;
    float buf_size_f = static_cast<float>(buffer_->size());
    for (int g = 0; g < kMaxGrains; ++g) {
        if (!grains_[g].active()) continue;
        ++active_count;

        grains_[g].ProcessBlock(*buffer_, buf_size_f, output, num_frames);
    }

    // --- Overlap normalization (matches Clouds approach) ---
    // Asymmetric tracking: fast rise (when grains pile up, reduce gain
    // quickly to prevent clipping), slow fall (when grains end, restore
    // gain slowly to prevent pumping).
    float count_f = static_cast<float>(active_count);
    float slope_coeff = (count_f > overlap_count_lp_) ? 0.9f : 0.2f;
    for (size_t i = 0; i < num_frames; ++i) {
        ONE_POLE(overlap_count_lp_, count_f, slope_coeff);
    }

    // 1/sqrt(n-1) for n > 2, unity gain for 1-2 grains.
    float gain_norm = (overlap_count_lp_ > 2.0f)
        ? 1.0f / std::sqrt(overlap_count_lp_ - 1.0f)
        : 1.0f;

    // Per-sample smooth the gain itself to avoid clicks on sudden changes.
    for (size_t i = 0; i < num_frames; ++i) {
        ONE_POLE(gain_normalization_, gain_norm, 0.01f);
        output[i].l *= gain_normalization_;
        output[i].r *= gain_normalization_;
    }
}

} // namespace beads
