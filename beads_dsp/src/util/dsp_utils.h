#pragma once

#include <cmath>
#include <algorithm>
#include "../../include/beads/types.h"
#include "cosine_table.h"

namespace beads {

static constexpr float kPi = 3.14159265358979323846f;
static constexpr float kTwoPi = 2.0f * kPi;

// One-pole low-pass filter macro
// state += coefficient * (input - state)
#define ONE_POLE(state, input, coefficient) \
    state += (coefficient) * ((input) - (state))

inline float Clamp(float value, float min_val, float max_val) {
    return std::min(std::max(value, min_val), max_val);
}

inline float Crossfade(float a, float b, float mix) {
    return a + (b - a) * mix;
}

// Equal-power crossfade (stereo)
// Uses CosLookup: mix 0..1 maps to phase 0..0.25 (quarter cycle).
inline StereoFrame EqualPowerCrossfade(StereoFrame dry, StereoFrame wet, float mix) {
    float phase = mix * 0.25f;  // 0..0.25 = 0..pi/2
    float dry_gain = CosLookup(phase);
    float wet_gain = CosLookup(phase - 0.25f);  // sin = cos(x - pi/2)
    return {
        dry.l * dry_gain + wet.l * wet_gain,
        dry.r * dry_gain + wet.r * wet_gain
    };
}

inline float SemitonesToRatio(float semitones) {
    return std::exp2(semitones / 12.0f);
}

// Soft clipping using tanh approximation
inline float SoftClip(float x) {
    if (x < -3.0f) return -1.0f;
    if (x > 3.0f) return 1.0f;
    return x * (27.0f + x * x) / (27.0f + 9.0f * x * x);
}

// Hard clip
inline float HardClip(float x, float limit = 1.0f) {
    return Clamp(x, -limit, limit);
}

// dB to linear gain
inline float DbToGain(float db) {
    return std::pow(10.0f, db / 20.0f);
}

// Linear gain to dB
inline float GainToDb(float gain) {
    return 20.0f * std::log10(std::max(gain, 1e-10f));
}

// Fast approximation of tanh
inline float FastTanh(float x) {
    float x2 = x * x;
    return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}

// Mu-law compression (for tape quality mode)
// Note: output exceeds [-1, 1] when |x| > 1. Call sites that need
// bounded output (e.g. feedback limiters) must clamp separately.
// Optimized: precomputed reciprocal of log(1+mu) for mu=64 (tape mode).
inline float MuLawCompress(float x, float mu = 255.0f) {
    float abs_x = std::min(std::abs(x), 100.0f);
    float sign = x >= 0.0f ? 1.0f : -1.0f;
    // For mu=64 (tape mode): 1/log(65) = 0.23964..
    // Use precomputed value when mu=64 to avoid per-sample log()
    float inv_log_1_plus_mu = (mu == 64.0f) ? 0.23964147646f : (1.0f / std::log(1.0f + mu));
    return sign * std::log(1.0f + mu * abs_x) * inv_log_1_plus_mu;
}

inline float MuLawExpand(float x, float mu = 255.0f) {
    float abs_x = std::min(std::abs(x), 1.0f);
    float sign = x >= 0.0f ? 1.0f : -1.0f;
    // pow(1+mu, abs_x) = exp(abs_x * log(1+mu))
    // For mu=64: log(65) = 4.17438..
    float log_1_plus_mu = (mu == 64.0f) ? 4.17438726989f : std::log(1.0f + mu);
    return sign * (std::exp(abs_x * log_1_plus_mu) - 1.0f) / mu;
}

} // namespace beads
