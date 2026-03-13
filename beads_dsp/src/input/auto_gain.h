#pragma once

#include "../../include/beads/types.h"

namespace beads {

class AutoGain {
public:
    void Init(float sample_rate);

    // Process a stereo frame, returns the gain-adjusted frame
    StereoFrame Process(StereoFrame input, float manual_gain_db);

    // Get current detected input level (0-1)
    float InputLevel() const;

private:
    float sample_rate_ = 48000.0f;
    float envelope_ = 0.0f;        // Peak envelope follower
    float gain_ = 1.0f;            // Current applied gain
    float target_gain_ = 1.0f;
    float last_gain_db_ = 0.0f;   // Cache to skip DbToGain when unchanged

    // Timing
    float attack_coeff_ = 0.0f;     // Fast attack
    float release_coeff_ = 0.0f;    // 5s release

    static constexpr float kMinGainDb = -60.0f;
    static constexpr float kMaxGainDb = 32.0f;
};

} // namespace beads
