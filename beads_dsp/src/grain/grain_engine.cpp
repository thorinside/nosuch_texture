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

    // Pool is full — start fading out the oldest grain that isn't already
    // fading.  This ensures the stolen grain's output ramps to zero
    // (over ~32 samples) instead of cutting abruptly, which would click.
    // We do NOT return the fading grain for immediate reuse; the new
    // trigger is simply dropped.  The grain will become free once the
    // fade completes.
    for (int i = 0; i < kMaxGrains; ++i) {
        if (!grains_[i].fading_out()) {
            grains_[i].StartFadeOut();
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

    // Exponential mapping from 0..1 to 10ms..4s
    float min_dur = 0.01f;   // 10ms
    float max_dur = kDefaultBufferDuration;  // 4s
    float duration = min_dur * std::pow(max_dur / min_dur, abs_size);
    gp.size = duration * sample_rate_;

    // --- TIME → buffer read position ---
    // Matches original Beads manual:
    //   0.0 (fully CCW) → most recent audio (near write head)
    //   1.0 (fully CW)  → oldest audio (far from write head)
    float mod_time = ar_time_.Process(params.time, params.time_ar,
                                       params.time_cv, params.time_cv_connected);
    mod_time = Clamp(mod_time, 0.0f, 1.0f);

    // Convert to an absolute position in the recording buffer.
    // time=0.0 means read from write_head (newest), time=1.0 means read
    // from one full buffer behind the write_head (oldest).
    float buf_size_f = static_cast<float>(buffer_->size());
    float offset_frames = mod_time * buf_size_f;
    float pos = static_cast<float>(buffer_->write_head()) - offset_frames;
    if (pos < 0.0f) pos += buf_size_f;
    gp.position = pos;

    // --- PITCH → playback rate ---
    float mod_pitch = ar_pitch_.Process(params.pitch, params.pitch_ar,
                                         params.pitch_cv, params.pitch_cv_connected);
    gp.pitch_ratio = SemitonesToRatio(mod_pitch) * pitch_mod_ratio_;
    if (reverse) gp.pitch_ratio = -gp.pitch_ratio;

    // --- SHAPE → envelope shape ---
    float mod_shape = ar_shape_.Process(params.shape, params.shape_ar,
                                         params.shape_cv, params.shape_cv_connected);
    gp.shape = Clamp(mod_shape, 0.0f, 1.0f);

    // --- PAN ---
    // Slight random panning for stereo width.
    gp.pan = random_.NextBipolar() * 0.5f;

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

    // Start new grains at their trigger points.
    for (int t = 0; t < num_triggers; ++t) {
        Grain* g = AllocateGrain();
        if (g) {
            auto gp = ComputeGrainParams(params, trigger_samples[t]);
            g->Start(gp);
        }
    }

    // --- Render all active grains, overlap-add ---
    for (size_t i = 0; i < num_frames; ++i) {
        float sum_l = 0.0f;
        float sum_r = 0.0f;
        int active_count = 0;

        for (int g = 0; g < kMaxGrains; ++g) {
            if (!grains_[g].active()) continue;

            float gl = 0.0f;
            float gr = 0.0f;
            bool still_active = grains_[g].Process(*buffer_, &gl, &gr);
            (void)still_active;

            sum_l += gl;
            sum_r += gr;
            ++active_count;
        }

        // --- Overlap normalization ---
        // Smooth the active grain count with a one-pole filter.
        float count_f = static_cast<float>(active_count);
        ONE_POLE(overlap_count_lp_, count_f, 0.01f);

        // Scale output by 1 / max(1, sqrt(overlap)) to prevent volume spikes.
        float norm = 1.0f;
        if (overlap_count_lp_ > 1.0f) {
            norm = 1.0f / std::sqrt(overlap_count_lp_);
        }

        output[i].l = sum_l * norm;
        output[i].r = sum_r * norm;
    }
}

} // namespace beads
