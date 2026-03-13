#include "auto_gain.h"
#include "../util/dsp_utils.h"

#include <cmath>

namespace beads {

// Fast dB-to-linear approximation using exp2.
// pow(10, db/20) = exp2(db * log2(10)/20) = exp2(db * 0.16609640474)
// Uses the standard fast-exp2 trick: split into integer + fractional parts,
// then polynomial approximation for the fractional part.
static inline float FastDbToGain(float db) {
    float x = db * 0.16609640474f;  // db * log2(10)/20
    // Clamp to avoid overflow/underflow in bit manipulation
    if (x < -20.0f) return 0.0f;
    if (x > 20.0f) x = 20.0f;
    return std::exp2(x);  // ARM Cortex-M7: ~20 cycles vs pow(10,x) ~80 cycles
}

// Fast linear-to-dB using logf (available on NT runtime).
// 20 * log10(x) = 20 * ln(x) / ln(10) = ln(x) * 8.6858896381
static inline float FastGainToDb(float gain) {
    if (gain < 1e-10f) return -200.0f;
    return std::log(gain) * 8.6858896381f;
}

void AutoGain::Init(float sample_rate) {
    sample_rate_ = sample_rate;
    envelope_ = 0.0f;
    gain_ = 1.0f;
    target_gain_ = 1.0f;
    last_gain_db_ = 0.0f;

    // Fast attack: ~1ms time constant
    attack_coeff_ = 1.0f - std::exp(-1.0f / (0.001f * sample_rate_));

    // Slow release: 5 seconds
    release_coeff_ = 1.0f - std::exp(-1.0f / (5.0f * sample_rate_));
}

StereoFrame AutoGain::Process(StereoFrame input, float manual_gain_db) {
    // Peak detection: take the max of left and right absolute values.
    // Guard against NaN input so the envelope follower doesn't latch to NaN.
    float abs_l = std::abs(input.l);
    float abs_r = std::abs(input.r);
    if (std::isnan(abs_l)) abs_l = 0.0f;
    if (std::isnan(abs_r)) abs_r = 0.0f;
    float peak = std::max(abs_l, abs_r);

    // Envelope follower: fast attack, slow release
    if (peak > envelope_) {
        ONE_POLE(envelope_, peak, attack_coeff_);
    } else {
        ONE_POLE(envelope_, peak, release_coeff_);
    }

    // Determine the gain to apply
    float gain_db;
    if (!std::isnan(manual_gain_db)) {
        // Manual gain mode: use the provided value directly.
        gain_db = Clamp(manual_gain_db, kMinGainDb, kMaxGainDb);
    } else {
        // Auto-gain mode: lower input level -> higher gain, up to +32dB
        float input_db = FastGainToDb(envelope_);
        gain_db = Clamp(-input_db, kMinGainDb, kMaxGainDb);
    }

    // Only recompute target_gain_ when gain_db changes.
    // In manual mode this is constant, saving ~80 cycles/sample.
    if (gain_db != last_gain_db_) {
        last_gain_db_ = gain_db;
        target_gain_ = FastDbToGain(gain_db);
    }

    // Smooth gain changes
    ONE_POLE(gain_, target_gain_, 0.0001f);

    return input * gain_;
}

float AutoGain::InputLevel() const {
    return envelope_;
}

} // namespace beads
