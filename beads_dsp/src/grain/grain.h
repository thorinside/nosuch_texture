#pragma once

#include "../../include/beads/types.h"
#include "../buffer/recording_buffer.h"
#include "../util/cosine_table.h"
#include "../util/dsp_utils.h"

#include <cmath>

namespace beads {

class Grain {
public:
    struct GrainParameters {
        float position;         // Buffer position in frames (where to read)
        float size;             // Duration in samples
        float pitch_ratio;      // Playback rate (1.0 = normal, 2.0 = octave up)
        float shape;            // 0-1 envelope shape
        float pan;              // -1 to +1 stereo pan
        int pre_delay;          // Sub-block start offset in samples
    };

    void Init();

    // Trigger a new grain
    void Start(const GrainParameters& params);

    // Begin a short fade-out for grain stealing (avoids click)
    void StartFadeOut();

    // Process one sample, reading from the recording buffer.
    // Returns true if grain is still active.
    // INLINED for the grain-major hot loop — must stay in the header.
    inline bool Process(const RecordingBuffer& buffer,
                        float buf_size, float* out_l, float* out_r) {
        if (!active_) {
            *out_l = 0.0f;
            *out_r = 0.0f;
            return false;
        }

        // Pre-delay: output silence until the grain's sub-block offset is reached.
        if (pre_delay_ > 0) {
            --pre_delay_;
            *out_l = 0.0f;
            *out_r = 0.0f;
            return true;
        }

        // Check if envelope has completed before computing this sample.
        // (Checking *before* the increment avoids discarding the last
        // computed sample, which would cause a small discontinuity.)
        // Note: NaN fails all ordered comparisons, so we must also
        // explicitly check for it to avoid passing NaN into ComputeEnvelope.
        if (envelope_phase_ >= 1.0f || !std::isfinite(envelope_phase_)) {
            active_ = false;
            *out_l = 0.0f;
            *out_r = 0.0f;
            return false;
        }

        // Envelope (smoothness/slope precomputed in Start())
        float env = ComputeEnvelope(envelope_phase_);
        envelope_phase_ += envelope_increment_;

        // Read from recording buffer with linear interpolation.
        float sample_l, sample_r;
        buffer.ReadLinearStereoFast(read_position_, &sample_l, &sample_r);

        // Advance read position, wrapping around buffer size.
        read_position_ += phase_increment_;
        if (buf_size > 0.0f) {
            while (read_position_ >= buf_size) read_position_ -= buf_size;
            while (read_position_ < 0.0f) read_position_ += buf_size;
        }

        // Apply steal fade-out if active.
        if (fading_out_) {
            float fade = static_cast<float>(steal_fade_counter_)
                       * steal_fade_inv_;
            env *= fade;
            steal_fade_counter_--;
            if (steal_fade_counter_ < 0) {
                active_ = false;
                fading_out_ = false;
                *out_l = 0.0f;
                *out_r = 0.0f;
                return false;
            }
        }

        // Apply envelope and panning.
        *out_l = sample_l * env * pan_l_;
        *out_r = sample_r * env * pan_r_;

        return true;
    }

    // Process a full block. Checks for NaN once at the end instead of per-sample.
    // buf_size_f should be static_cast<float>(buffer.size()).
    inline void ProcessBlock(const RecordingBuffer& buffer, float buf_size_f,
                             StereoFrame* output, size_t num_frames) {
        for (size_t i = 0; i < num_frames; ++i) {
            float gl = 0.0f, gr = 0.0f;
            Process(buffer, buf_size_f, &gl, &gr);
            output[i].l += gl;
            output[i].r += gr;
        }
        // One-time NaN guard per grain per block instead of per sample.
        // If the grain produced NaN, zero the contribution and kill the grain.
        if (!std::isfinite(output[0].l + output[num_frames - 1].l +
                           output[0].r + output[num_frames - 1].r)) {
            // Scan and zero any non-finite samples
            for (size_t i = 0; i < num_frames; ++i) {
                if (!std::isfinite(output[i].l)) output[i].l = 0.0f;
                if (!std::isfinite(output[i].r)) output[i].r = 0.0f;
            }
            active_ = false;
        }
    }

    bool active() const { return active_; }
    bool fading_out() const { return fading_out_; }

private:
    bool active_ = false;
    bool fading_out_ = false;

    // Fade-out for grain stealing
    static constexpr int kStealFadeSamples = 32;
    static constexpr float steal_fade_inv_ = 1.0f / static_cast<float>(kStealFadeSamples);
    int steal_fade_counter_ = 0;

    // Read position (fractional for sub-sample accuracy)
    float read_position_ = 0.0f;
    float phase_increment_ = 0.0f;

    // Envelope
    float envelope_phase_ = 0.0f;
    float envelope_increment_ = 0.0f;

    // Precomputed envelope parameters (derived from shape in Start())
    float smoothness_ = 1.0f;
    float slope_ = 0.5f;
    float inv_slope_ = 2.0f;         // 1.0f / slope_
    float inv_one_minus_slope_ = 2.0f; // 1.0f / (1.0f - slope_)
    enum class SlopeMode : uint8_t { kNormal, kDegenLow, kDegenHigh };
    SlopeMode slope_mode_ = SlopeMode::kNormal;

    // Stereo
    float pan_l_ = 1.0f;
    float pan_r_ = 1.0f;

    // Pre-delay counter
    int pre_delay_ = 0;

    // Compute envelope value from phase using precomputed smoothness/slope.
    // INLINED — called per-sample per-grain in the hot loop.
    inline float ComputeEnvelope(float phase) {
        // Asymmetric triangle envelope using precomputed reciprocals
        // (avoids per-sample division).
        float trap;
        switch (slope_mode_) {
        case SlopeMode::kDegenLow:
            trap = 1.0f - phase;
            break;
        case SlopeMode::kDegenHigh:
            trap = phase;
            break;
        default:
            if (phase < slope_) {
                trap = phase * inv_slope_;
            } else {
                trap = (1.0f - phase) * inv_one_minus_slope_;
            }
            break;
        }
        trap = Clamp(trap, 0.0f, 1.0f);

        // Hann window: 0.5 - 0.5 * cos(2*pi*phase)
        float hann = 0.5f - 0.5f * CosLookup(phase);

        // Blend between trapezoidal and Hann based on smoothness.
        return Crossfade(trap, hann, smoothness_);
    }
};

} // namespace beads
