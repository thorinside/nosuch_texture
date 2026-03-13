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
    gp.pitch_ratio = SemitonesToRatio(mod_pitch) * pitch_mod_ratio_ / df_f;
    if (reverse) {
        gp.pitch_ratio = -gp.pitch_ratio;
        // Offset start position to the END of the segment a forward grain
        // would play.  The reverse grain then reads backwards through the
        // same audio, producing true reversed playback of the intended
        // segment rather than reading into unrelated older audio.
        float span = gp.size * std::fabs(gp.pitch_ratio);
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

    // --- Overlap normalization ---
    // Advance the smoothed count for the full block, then apply the
    // normalization factor once. The ONE_POLE with 0.01 coefficient
    // changes < 0.6% across 64 samples, so per-block is inaudible
    // vs per-sample, and saves 64 sqrt calls.
    float count_f = static_cast<float>(active_count);
    for (size_t i = 0; i < num_frames; ++i) {
        ONE_POLE(overlap_count_lp_, count_f, 0.01f);
    }

    if (overlap_count_lp_ > 1.0f) {
        float norm = 1.0f / std::sqrt(overlap_count_lp_);
        for (size_t i = 0; i < num_frames; ++i) {
            output[i].l *= norm;
            output[i].r *= norm;
        }
    }
}

} // namespace beads
