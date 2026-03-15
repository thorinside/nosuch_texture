#pragma once

#include <cstddef>
#include "../../include/beads/types.h"
#include "../../include/beads/parameters.h"
#include "../random/random.h"
#include "../random/attenurandomizer.h"
#include "../pitch/pitch_quantizer.h"
#include "grain.h"
#include "grain_scheduler.h"

namespace beads {

class RecordingBuffer;

class GrainEngine {
public:
    void Init(float sample_rate, RecordingBuffer* buffer);

    // Process one block of audio
    void Process(const BeadsParameters& params, StereoFrame* output,
                 size_t num_frames);

    // Set pitch modulation ratio (tape mode wow/flutter). 1.0 = no modulation.
    void SetPitchModulation(float ratio) { pitch_mod_ratio_ = ratio; }

    int ActiveGrainCount() const;

    // Scale quantization
    void LoadScale(const double* ratios, uint32_t num_notes) { pitch_quantizer_.loadRatios(ratios, num_notes); }
    void ClearScale() { pitch_quantizer_.clear(); }
    void SetScaleRoot(int midi_note) { pitch_quantizer_.set_root(midi_note); }

private:
    Grain grains_[kMaxGrains];
    GrainScheduler scheduler_;
    Attenurandomizer ar_time_;
    Attenurandomizer ar_size_;
    Attenurandomizer ar_shape_;
    Attenurandomizer ar_pitch_;
    Random random_;
    PitchQuantizer pitch_quantizer_;

    RecordingBuffer* buffer_ = nullptr;
    float sample_rate_ = 48000.0f;

    // Overlap normalization
    float overlap_count_lp_ = 0.0f;   // Smoothed active grain count
    float gain_normalization_ = 1.0f;  // Smoothed gain factor

    // Tape mode wow/flutter pitch modulation (ratio, 1.0 = none)
    float pitch_mod_ratio_ = 1.0f;

    // Allocate a grain from the pool (returns nullptr if full after stealing)
    Grain* AllocateGrain();

    // Compute grain parameters from BeadsParameters + attenurandomizers
    Grain::GrainParameters ComputeGrainParams(const BeadsParameters& params,
                                               int pre_delay);
};

} // namespace beads
