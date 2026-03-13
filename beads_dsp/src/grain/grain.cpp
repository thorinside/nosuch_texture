#include "grain.h"

#include <cmath>
#include <algorithm>

namespace beads {

void Grain::Init() {
    active_ = false;
    fading_out_ = false;
    steal_fade_counter_ = 0;
    read_position_ = 0.0f;
    phase_increment_ = 0.0f;
    envelope_phase_ = 0.0f;
    envelope_increment_ = 0.0f;
    smoothness_ = 1.0f;
    slope_ = 0.5f;
    inv_slope_ = 2.0f;
    inv_one_minus_slope_ = 2.0f;
    slope_mode_ = SlopeMode::kNormal;
    pan_l_ = 1.0f;
    pan_r_ = 1.0f;
    pre_delay_ = 0;
}

void Grain::StartFadeOut() {
    if (active_ && !fading_out_) {
        fading_out_ = true;
        steal_fade_counter_ = kStealFadeSamples;
    }
}

void Grain::Start(const GrainParameters& params) {
    active_ = true;
    fading_out_ = false;
    steal_fade_counter_ = 0;

    read_position_ = params.position;
    phase_increment_ = params.pitch_ratio;

    // Envelope increments once per sample over the grain's duration.
    // Phase goes from 0 to 1 over params.size samples.
    if (params.size > 0.0f) {
        envelope_increment_ = 1.0f / params.size;
    } else {
        envelope_increment_ = 1.0f;
    }
    envelope_phase_ = 0.0f;

    // Precompute envelope shape parameters (constant for grain lifetime).
    // Avoids per-sample branching and division in ComputeEnvelope().
    float shape = Clamp(params.shape, 0.0f, 1.0f);
    if (shape < 0.5f) {
        smoothness_ = shape * 2.0f;
        slope_ = 0.5f;
    } else {
        smoothness_ = 1.0f;
        slope_ = 1.0f - (shape - 0.5f) * 2.0f;
    }

    if (slope_ < 0.001f) {
        slope_mode_ = SlopeMode::kDegenLow;
        inv_slope_ = 0.0f;
        inv_one_minus_slope_ = 0.0f;
    } else if (slope_ > 0.999f) {
        slope_mode_ = SlopeMode::kDegenHigh;
        inv_slope_ = 0.0f;
        inv_one_minus_slope_ = 0.0f;
    } else {
        slope_mode_ = SlopeMode::kNormal;
        inv_slope_ = 1.0f / slope_;
        inv_one_minus_slope_ = 1.0f / (1.0f - slope_);
    }

    // Equal-power panning.
    // pan: -1 (full left) to +1 (full right), 0 = center.
    float p = Clamp(params.pan * 0.5f + 0.5f, 0.0f, 1.0f);  // map to 0..1
    pan_l_ = std::cos(p * kPi * 0.5f);
    pan_r_ = std::sin(p * kPi * 0.5f);

    pre_delay_ = std::max(params.pre_delay, 0);
}

} // namespace beads
