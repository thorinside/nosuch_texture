#pragma once

#include "../../include/beads/types.h"

namespace beads {

class AutoGain {
public:
    void Init(float sample_rate);

    // Process a stereo frame, returns the gain-adjusted frame.
    // auto_gain_on: true = calibrate-and-lock mode, false = manual gain.
    StereoFrame Process(StereoFrame input, float manual_gain_db, bool auto_gain_on);

    // Signal that auto-gain was just toggled on — start fresh calibration.
    void StartCalibration();

    // Get current detected input level (0-1)
    float InputLevel() const;

    // Calibration progress: 0.0 → 1.0 while calibrating, 1.0 once locked or disabled.
    float CalibrationProgress() const;

private:
    enum class State { kDisabled, kCalibrating, kLocked };

    float sample_rate_ = 48000.0f;
    float envelope_ = 0.0f;        // Peak envelope follower
    float gain_ = 1.0f;            // Current applied gain
    float target_gain_ = 1.0f;     // Locked value once calibration completes
    float last_gain_db_ = 0.0f;    // Cache to skip DbToGain when unchanged

    State state_ = State::kDisabled;
    int calibration_counter_ = 0;

    // Timing
    float release_coeff_ = 0.0f;    // ~500ms release for calibration
    int calibration_samples_ = 0;   // ~5 seconds of calibration

    static constexpr float kMinGainDb = -60.0f;
    static constexpr float kMaxGainDb = 32.0f;
    // Headroom reserved for downstream peaks (granular reconstruction overlap,
    // reverb tank gain, LP filter ringing). Calibration targets -kHeadroomDb
    // instead of 0 dBFS so the signal leaves room before clipping at the host.
    // Bumped from 6 → 10 dB on 2026-05-01 to fix audible distortion when many
    // grains stack constructively or reverb adds tank gain.
    static constexpr float kHeadroomDb = 10.0f;
};

} // namespace beads
