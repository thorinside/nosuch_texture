#include "auto_gain.h"
#include "../util/dsp_utils.h"

#include <cmath>

namespace beads {

// Fast dB-to-linear approximation using exp2.
// pow(10, db/20) = exp2(db * log2(10)/20) = exp2(db * 0.16609640474)
static inline float FastDbToGain(float db) {
    float x = db * 0.16609640474f;
    if (x < -20.0f) return 0.0f;
    if (x > 20.0f) x = 20.0f;
    return std::exp2(x);
}

// Fast linear-to-dB using logf.
// 20 * log10(x) = 20 * ln(x) / ln(10) = ln(x) * 8.6858896381
static inline float FastGainToDb(float gain) {
    if (gain < 1e-10f) return -200.0f;
    return logf(gain) * 8.6858896381f;
}

void AutoGain::Init(float sample_rate) {
    sample_rate_ = sample_rate;
    envelope_ = 0.0f;
    gain_ = 1.0f;
    target_gain_ = 1.0f;
    last_gain_db_ = 0.0f;
    state_ = State::kDisabled;
    calibration_counter_ = 0;

    // ~500ms release for calibration measurement.  Attack is instantaneous
    // (true peak follower) — see Process().
    release_coeff_ = 1.0f - std::exp(-1.0f / (0.5f * sample_rate_));

    // ~5 second calibration window
    calibration_samples_ = static_cast<int>(sample_rate_ * 5.0f);
}

void AutoGain::StartCalibration() {
    state_ = State::kCalibrating;
    calibration_counter_ = calibration_samples_;
    // Don't reset envelope — keep whatever level info we already have
}

StereoFrame AutoGain::Process(StereoFrame input, float manual_gain_db, bool auto_gain_on) {
    // Peak detection (always runs for the input level meter)
    float abs_l = std::abs(input.l);
    float abs_r = std::abs(input.r);
    if (std::isnan(abs_l)) abs_l = 0.0f;
    if (std::isnan(abs_r)) abs_r = 0.0f;
    float peak = std::max(abs_l, abs_r);

    // True peak follower: instantaneous attack, slow release.  An attack
    // low-pass (even at 1 ms) under-tracks low-frequency signals — a 50 Hz
    // sine only spends a few samples near peak per 20 ms cycle, so the
    // ONE_POLE attack would only reach ~60% of the true peak.  That
    // under-reading made the calibration set the gain too high and the
    // downstream chain clipped.  Snap the envelope to the peak on attack;
    // release at ~500 ms so the calibration sample-counter sees a stable
    // estimate of the loudest peak in the calibration window.
    if (peak > envelope_) {
        envelope_ = peak;
    } else {
        ONE_POLE(envelope_, peak, release_coeff_);
    }

    if (!auto_gain_on) {
        // Manual gain mode
        state_ = State::kDisabled;
        float gain_db = Clamp(manual_gain_db, kMinGainDb, kMaxGainDb);
        if (gain_db != last_gain_db_) {
            last_gain_db_ = gain_db;
            target_gain_ = FastDbToGain(gain_db);
        }
        ONE_POLE(gain_, target_gain_, 0.0001f);
        return input * gain_;
    }

    // Auto-gain mode
    if (state_ == State::kCalibrating) {
        // Track input level and compute gain in real-time during calibration.
        // Target -kHeadroomDb (not 0 dBFS) so downstream stages have peak room.
        float input_db = FastGainToDb(envelope_);
        float gain_db = Clamp(-input_db - kHeadroomDb, kMinGainDb, kMaxGainDb);
        if (gain_db != last_gain_db_) {
            last_gain_db_ = gain_db;
            target_gain_ = FastDbToGain(gain_db);
        }
        ONE_POLE(gain_, target_gain_, 0.001f);

        calibration_counter_--;
        if (calibration_counter_ <= 0) {
            // Lock the current gain.  target_gain_ holds the locked value;
            // gain_ smoothly converges to it during locked-state ticks.
            target_gain_ = gain_;
            last_gain_db_ = FastGainToDb(target_gain_);
            state_ = State::kLocked;
        }
    } else if (state_ == State::kLocked) {
        // Locked: hold the calibrated gain.  The envelope follower keeps
        // updating for the input level meter, but the gain does not change
        // until the next StartCalibration() / TriggerAutoGainCalibration() —
        // no ratcheting on hot transients (the prior ratchet measured
        // AutoGain's local output, not the post-DSP-chain bus, so it gave
        // a false sense of clip protection while still allowing audible
        // distortion downstream).
        ONE_POLE(gain_, target_gain_, 0.01f);
    } else {
        // State::kDisabled but auto_gain_on — first enable, start calibrating
        StartCalibration();
        ONE_POLE(gain_, target_gain_, 0.001f);
    }

    return input * gain_;
}

float AutoGain::InputLevel() const {
    return envelope_;
}

float AutoGain::CalibrationProgress() const {
    if (state_ == State::kCalibrating && calibration_samples_ > 0) {
        return 1.0f - static_cast<float>(calibration_counter_) /
                          static_cast<float>(calibration_samples_);
    }
    return 1.0f;
}

} // namespace beads
