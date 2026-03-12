#pragma once

#include "../../include/beads/types.h"
#include "../../include/beads/parameters.h"

namespace beads {

class RecordingBuffer;

class DelayEngine {
public:
    void Init(float sample_rate, RecordingBuffer* buffer);

    void Process(const BeadsParameters& params, StereoFrame* output, size_t num_frames);

    // Set pitch modulation ratio (tape mode wow/flutter). 1.0 = no modulation.
    void SetPitchModulation(float ratio) { pitch_mod_ratio_ = ratio; }

private:
    float sample_rate_ = 48000.0f;
    RecordingBuffer* buffer_ = nullptr;

    // Primary delay read position
    float read_position_ = 0.0f;
    float delay_time_target_ = 0.0f;

    // Secondary tap (golden ratio of primary)
    float secondary_tap_ratio_ = 1.6180339887f;  // Golden ratio

    // Pitch shifter (2 overlapping read-heads with triangular crossfade)
    float pitch_shift_phase_[2] = {0.0f, 0.5f};
    float pitch_shift_increment_ = 0.0f;

    // Tempo-synced amplitude envelope (SHAPE as tremolo/slicer)
    float envelope_phase_ = 0.0f;

    // Freeze loop state
    bool frozen_ = false;
    float loop_start_ = 0.0f;
    float loop_length_ = 0.0f;

    // Smoothing
    float smoothed_delay_time_ = 0.0f;
    float smoothed_envelope_gain_ = 1.0f;  // Smoothed slicer envelope to avoid hard edges
    float smoothed_secondary_mix_ = 0.0f;  // Smoothed secondary tap mix level

    // Tape mode wow/flutter pitch modulation (ratio, 1.0 = none)
    float pitch_mod_ratio_ = 1.0f;

    // Loop crossfade length (samples) to avoid click at loop boundary
    static constexpr int kLoopXfadeSamples = 64;
};

} // namespace beads
