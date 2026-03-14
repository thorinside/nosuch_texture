#include "grain_scheduler.h"
#include "../util/dsp_utils.h"

#include <cmath>
#include <algorithm>

namespace beads {

void GrainScheduler::Init(float sample_rate) {
    sample_rate_ = sample_rate;
    latched_phase_ = 0.0f;
    prev_gate_ = false;
    gate_phase_ = 0.0f;
    prev_clock_ = false;
    clock_period_ = 0.0f;
    samples_since_clock_ = 0;
    random_.Init(0xBEAD5EED);
}

float GrainScheduler::DensityToRate(float density) {
    // density 0.5 → 0 Hz (silence)
    // density 0.0 → ~100 Hz (fast, regular)
    // density 1.0 → ~100 Hz average (fast, random)
    //
    // Both halves map the distance from center to a rate.
    // The mapping is exponential for a musically useful range.
    float distance = std::abs(density - 0.5f) * 2.0f;  // 0..1
    if (distance < 0.001f) return 0.0f;

    // Exponential mapping: 0 → ~0.25 Hz, 1 → ~100 Hz
    return 0.25f * std::exp2(distance * 8.64f);  // 2^8.64 ≈ 400, 0.25*400=100
}

int GrainScheduler::Process(const BeadsParameters& params, size_t block_size,
                            int* trigger_samples, int max_triggers) {
    int trigger_count = 0;

    switch (params.trigger_mode) {
    case TriggerMode::kLatched: {
        // Internal phasor mode.
        float rate = DensityToRate(params.density);
        if (rate <= 0.0f) break;

        float phase_inc = rate / sample_rate_;
        bool use_random = (params.density > 0.5f);

        for (size_t i = 0; i < block_size && trigger_count < max_triggers; ++i) {
            latched_phase_ += phase_inc;

            if (latched_phase_ >= 1.0f) {
                latched_phase_ -= 1.0f;
                trigger_samples[trigger_count++] = static_cast<int>(i);

                if (use_random) {
                    // For density > 0.5, use exponential distribution to
                    // randomize the interval.  Set the phasor so the next
                    // wrap happens after a random interval whose *average*
                    // equals the deterministic period (1 / rate).
                    //
                    // NextExponential() returns -ln(U) with mean 1.0.
                    // We want the next trigger after (1/rate) * random_factor
                    // samples on average.  Expressing that in phase units:
                    // phase_remaining = phase_inc * (1/rate) * exp_random
                    //                 = 1.0 * exp_random
                    // So we set latched_phase_ = 1.0 - exp_random, clamped
                    // so it stays in a reasonable range (don't wait forever).
                    float exp_rand = random_.NextExponential();
                    latched_phase_ = Clamp(1.0f - exp_rand, -2.0f, 0.99f);
                }
            }
        }
        break;
    }

    case TriggerMode::kGated: {
        bool gate = params.gate;
        bool rising_edge = gate && !prev_gate_;

        if (rising_edge) {
            // Always trigger on rising edge.
            if (trigger_count < max_triggers) {
                trigger_samples[trigger_count++] = 0;
            }

            // Reset gate-phase for repetition tracking.
            gate_phase_ = 0.0f;
        }

        if (gate && params.density != 0.5f) {
            if (params.density > 0.5f) {
                // CW from noon: repeat while gate is held.
                // Higher density → higher repeat rate.
                float repeat_rate = DensityToRate(params.density);
                if (repeat_rate > 0.0f) {
                    float phase_inc = repeat_rate / sample_rate_;
                    // Skip the first trigger if we already emitted one on
                    // the rising edge.
                    bool skip_first = rising_edge;
                    for (size_t i = 0; i < block_size && trigger_count < max_triggers; ++i) {
                        gate_phase_ += phase_inc;
                        if (gate_phase_ >= 1.0f) {
                            gate_phase_ -= 1.0f;
                            if (skip_first) {
                                skip_first = false;
                            } else {
                                trigger_samples[trigger_count++] = static_cast<int>(i);
                            }
                        }
                    }
                }
            } else if (rising_edge && params.density < 0.5f) {
                // CCW from noon: burst of grains at gate start.
                // Lower density → more grains in the burst.
                float burst_amount = (0.5f - params.density) * 2.0f;  // 0..1
                int burst_count = static_cast<int>(burst_amount * 15.0f);  // up to 15 extra
                // Spread the burst across the block.
                for (int b = 0; b < burst_count && trigger_count < max_triggers; ++b) {
                    int offset = static_cast<int>(
                        (static_cast<float>(b + 1) / static_cast<float>(burst_count + 1))
                        * static_cast<float>(block_size));
                    offset = std::min(offset, static_cast<int>(block_size) - 1);
                    trigger_samples[trigger_count++] = offset;
                }
            }
        }

        prev_gate_ = gate;
        break;
    }

    case TriggerMode::kClocked: {
        bool clock = params.gate;
        bool rising_edge = clock && !prev_clock_;
        // Cap the counter to prevent uint32_t overflow after very long
        // periods without a clock edge (~24h at 48kHz).  10 seconds of
        // samples is already far beyond any musical clock period.
        uint32_t max_samples = static_cast<uint32_t>(sample_rate_ * 10.0f);
        if (samples_since_clock_ < max_samples) {
            samples_since_clock_ += static_cast<uint32_t>(block_size);
        }

        if (rising_edge) {
            // Measure period between clock edges.
            if (clock_period_ > 0.0f) {
                // Smooth the period estimate.
                ONE_POLE(clock_period_,
                         static_cast<float>(samples_since_clock_), 0.5f);
            } else {
                clock_period_ = static_cast<float>(samples_since_clock_);
            }
            samples_since_clock_ = 0;

            if (params.density < 0.5f) {
                // CCW from noon: clock division.
                // Map 0.0 → /16, 0.5 → /1
                float div_amount = (0.5f - params.density) * 2.0f;  // 0..1
                // Exponential mapping to division ratios: 1, 2, 4, 8, 16
                int division = 1 << static_cast<int>(div_amount * 4.0f);
                division = std::min(division, 16);

                // Use a simple counter to divide.
                // We repurpose gate_phase_ as a clock-division counter.
                // Increment first, then check: the first clock starts at 1
                // and triggers when counter reaches the division value.
                gate_phase_ += 1.0f;
                if (static_cast<int>(gate_phase_) >= division) {
                    gate_phase_ = 0.0f;
                    if (trigger_count < max_triggers) {
                        trigger_samples[trigger_count++] = 0;
                    }
                }
                // Note: for division=1, every clock triggers (1 >= 1).
                // For division=2, every other clock triggers (1 < 2, 2 >= 2).
                // This means the first clock after init (gate_phase_ starts
                // at 0) triggers for division=1 but is delayed by one for
                // division>=2.  This matches typical clock divider behavior
                // where the first output aligns with the Nth input clock.
            } else if (params.density > 0.5f) {
                // CW from noon: probability trigger.
                // Map 0.5 → 0%, 1.0 → 100%
                float probability = (params.density - 0.5f) * 2.0f;
                if (random_.NextFloat() < probability) {
                    if (trigger_count < max_triggers) {
                        trigger_samples[trigger_count++] = 0;
                    }
                }
            } else {
                // Exactly 0.5: trigger on every clock.
                if (trigger_count < max_triggers) {
                    trigger_samples[trigger_count++] = 0;
                }
            }
        }

        prev_clock_ = clock;
        break;
    }
    case TriggerMode::kMidi:
        // Falls through to identical gated logic — MIDI host sets gate/pitch/velocity.
        goto midi_gated;
    } // switch

    goto scheduler_done;

    midi_gated: {
        bool gate = params.gate;
        bool rising_edge = gate && !prev_gate_;

        if (rising_edge) {
            if (trigger_count < max_triggers) {
                trigger_samples[trigger_count++] = 0;
            }
            gate_phase_ = 0.0f;
        }

        if (gate && params.density != 0.5f) {
            if (params.density > 0.5f) {
                float repeat_rate = DensityToRate(params.density);
                if (repeat_rate > 0.0f) {
                    float phase_inc = repeat_rate / sample_rate_;
                    bool skip_first = rising_edge;
                    for (size_t i = 0; i < block_size && trigger_count < max_triggers; ++i) {
                        gate_phase_ += phase_inc;
                        if (gate_phase_ >= 1.0f) {
                            gate_phase_ -= 1.0f;
                            if (skip_first) {
                                skip_first = false;
                            } else {
                                trigger_samples[trigger_count++] = static_cast<int>(i);
                            }
                        }
                    }
                }
            } else if (rising_edge && params.density < 0.5f) {
                float burst_amount = (0.5f - params.density) * 2.0f;
                int burst_count = static_cast<int>(burst_amount * 15.0f);
                for (int b = 0; b < burst_count && trigger_count < max_triggers; ++b) {
                    int offset = static_cast<int>(
                        (static_cast<float>(b + 1) / static_cast<float>(burst_count + 1))
                        * static_cast<float>(block_size));
                    offset = std::min(offset, static_cast<int>(block_size) - 1);
                    trigger_samples[trigger_count++] = offset;
                }
            }
        }

        prev_gate_ = gate;
    }

    scheduler_done:

    return trigger_count;
}

} // namespace beads
