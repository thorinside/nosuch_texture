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
    float target_gain_ = 1.0f;
    float locked_gain_ = 1.0f;     // Snapshot when entering kLocked
    float last_gain_db_ = 0.0f;    // Cache to skip DbToGain when unchanged

    State state_ = State::kDisabled;
    int calibration_counter_ = 0;

    // Timing
    float release_coeff_ = 0.0f;    // ~500ms release for calibration
    int calibration_samples_ = 0;   // ~5 seconds of calibration

    static constexpr float kMinGainDb = -60.0f;
    static constexpr float kMaxGainDb = 32.0f;
    // Headroom reserved for downstream peaks (LP filter ringing, mu-law expansion,
    // granular reconstruction overlap). Calibration targets -kHeadroomDb instead
    // of 0 dBFS so the signal leaves room before clipping at the host.
    static constexpr float kHeadroomDb = 6.0f;
    // Asymmetric ratchet ceiling in locked mode: if output_peak (envelope * gain)
    // exceeds this threshold we ratchet target_gain_ down (and never raise it back).
    // -1 dBFS leaves ~5 dB of natural fluctuation room above the -6 dBFS calibration
    // target before intervening, but stops short of letting downstream peaks clip.
    static constexpr float kRatchetCeilingDb = -1.0f;
};

} // namespace beads
