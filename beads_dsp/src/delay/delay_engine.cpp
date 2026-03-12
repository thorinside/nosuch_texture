#include "delay_engine.h"
#include "../buffer/recording_buffer.h"
#include "../util/dsp_utils.h"
#include "../util/interpolation.h"

#include <cmath>

namespace beads {

void DelayEngine::Init(float sample_rate, RecordingBuffer* buffer) {
    sample_rate_ = sample_rate;
    buffer_ = buffer;

    read_position_ = 0.0f;
    delay_time_target_ = 0.0f;
    smoothed_delay_time_ = 0.0f;
    smoothed_envelope_gain_ = 1.0f;
    smoothed_secondary_mix_ = 0.0f;

    pitch_shift_phase_[0] = 0.0f;
    pitch_shift_phase_[1] = 0.5f;
    pitch_shift_increment_ = 0.0f;

    envelope_phase_ = 0.0f;

    frozen_ = false;
    loop_start_ = 0.0f;
    loop_length_ = 0.0f;
}

void DelayEngine::Process(const BeadsParameters& params,
                          StereoFrame* output,
                          size_t num_frames) {
    if (!buffer_ || buffer_->size() == 0) {
        for (size_t i = 0; i < num_frames; ++i) {
            output[i] = {0.0f, 0.0f};
        }
        return;
    }

    const float buffer_size = static_cast<float>(buffer_->size());

    // ---------------------------------------------------------------
    // DENSITY -> delay time (exponential mapping)
    // density=0 -> full buffer length, density=1.0 -> ~1ms (audio rate)
    // ---------------------------------------------------------------
    const float min_delay_samples = sample_rate_ * 0.001f;  // 1ms
    const float max_delay_samples = buffer_size;

    // Exponential mapping: delay = max * exp(-k * density)
    // At density=0: delay = max
    // At density=1: delay ~= min
    // k = ln(max / min)
    // Guard: if buffer is too small, k could be zero or negative.
    float ratio = max_delay_samples / std::max(min_delay_samples, 1.0f);
    if (ratio < 1.0f) ratio = 1.0f;
    float k = std::log(ratio);
    delay_time_target_ = max_delay_samples * std::exp(-k * params.density);
    delay_time_target_ = Clamp(delay_time_target_, 1.0f, max_delay_samples);

    // ---------------------------------------------------------------
    // Secondary tap: golden ratio of primary, fades in when density > 0.5
    // Target is computed immediately; smoothing happens per-sample below.
    // ---------------------------------------------------------------
    float secondary_mix_target = 0.0f;
    if (params.density > 0.5f) {
        secondary_mix_target = (params.density - 0.5f) * 2.0f;  // 0 at 0.5, 1 at 1.0
    }

    // ---------------------------------------------------------------
    // PITCH -> pitch shifter ratio
    // ---------------------------------------------------------------
    float pitch_ratio = SemitonesToRatio(params.pitch) * pitch_mod_ratio_;
    pitch_shift_increment_ = pitch_ratio;

    // ---------------------------------------------------------------
    // SHAPE -> tremolo / slicer envelope
    // 0.0 = no modulation, 0.5 = sine tremolo, 1.0 = hard slicer
    // Rate derived from delay time for tempo-synced feel
    // ---------------------------------------------------------------
    float envelope_rate = 0.0f;
    if (smoothed_delay_time_ > 0.0f) {
        // One full tremolo cycle per delay time period
        envelope_rate = 1.0f / smoothed_delay_time_;
    }

    // ---------------------------------------------------------------
    // FREEZE handling
    // ---------------------------------------------------------------
    if (params.freeze && !frozen_) {
        // Entering freeze: capture loop slice
        frozen_ = true;
        loop_start_ = static_cast<float>(buffer_->write_head());
        // SIZE selects loop duration (scaled from minimum to buffer length)
        float min_loop = sample_rate_ * 0.01f;  // 10ms minimum loop
        loop_length_ = min_loop + params.size * std::max(0.0f, buffer_size - min_loop);
        loop_length_ = Clamp(loop_length_, min_loop, buffer_size);

        // Set read_position_ to match where we were reading before freeze,
        // so there's no position discontinuity at the freeze boundary.
        // Before freeze: primary_pos = write_pos - delay.
        // After freeze: primary_pos = loop_start_ + read_position_.
        // For continuity: read_position_ = -smoothed_delay_time_ (mod loop_length_).
        if (loop_length_ > 0.0f) {
            float d = std::fmod(smoothed_delay_time_, loop_length_);
            read_position_ = (d > 0.0f) ? (loop_length_ - d) : 0.0f;
        } else {
            read_position_ = 0.0f;
        }

        // Reset pitch shifter phases to avoid timbral discontinuity
        // from stale phase values in the new context.
        pitch_shift_phase_[0] = 0.0f;
        pitch_shift_phase_[1] = 0.5f;
    } else if (!params.freeze && frozen_) {
        // Exiting freeze: reset pitch shifter phases
        frozen_ = false;
        pitch_shift_phase_[0] = 0.0f;
        pitch_shift_phase_[1] = 0.5f;
    }

    // If frozen, TIME selects which portion of the buffer to loop
    if (frozen_) {
        float min_loop = sample_rate_ * 0.01f;
        loop_length_ = min_loop + params.size * std::max(0.0f, buffer_size - min_loop);
        loop_length_ = Clamp(loop_length_, min_loop, buffer_size);
        float scan_range = std::max(0.0f, buffer_size - loop_length_);
        loop_start_ = params.time * scan_range;
    }

    // ---------------------------------------------------------------
    // Per-sample processing
    // ---------------------------------------------------------------
    for (size_t i = 0; i < num_frames; ++i) {
        // Smooth delay time and secondary mix changes
        ONE_POLE(smoothed_delay_time_, delay_time_target_, 0.001f);
        ONE_POLE(smoothed_secondary_mix_, secondary_mix_target, 0.002f);

        float current_delay = smoothed_delay_time_;
        float secondary_mix = smoothed_secondary_mix_;

        // ---------------------------------------------------------------
        // Compute primary read position
        // ---------------------------------------------------------------
        float primary_pos;
        float loop_xfade_gain = 1.0f;  // Used to crossfade at loop boundary
        if (frozen_) {
            // In freeze mode, read from the captured loop
            read_position_ += pitch_ratio;
            // Wrap within loop using fmod for NaN safety
            if (loop_length_ > 0.0f) {
                read_position_ = std::fmod(read_position_, loop_length_);
                if (read_position_ < 0.0f) read_position_ += loop_length_;
            } else {
                read_position_ = 0.0f;
            }

            // Crossfade at loop boundaries to avoid click.
            // When read_position_ is near 0 or near loop_length_, we
            // blend with the sample from the opposite end of the loop.
            float xfade_len = static_cast<float>(kLoopXfadeSamples);
            if (xfade_len > loop_length_ * 0.25f) {
                xfade_len = loop_length_ * 0.25f;
            }
            if (xfade_len >= 1.0f && read_position_ < xfade_len) {
                loop_xfade_gain = read_position_ / xfade_len;
            }

            primary_pos = loop_start_ + read_position_;
            // Wrap within buffer using fmod
            if (buffer_size > 0.0f) {
                primary_pos = std::fmod(primary_pos, buffer_size);
                if (primary_pos < 0.0f) primary_pos += buffer_size;
            }
        } else {
            // Normal delay mode: read position relative to write head
            float write_pos = static_cast<float>(buffer_->write_head());
            primary_pos = write_pos - current_delay;
            if (buffer_size > 0.0f) {
                primary_pos = std::fmod(primary_pos, buffer_size);
                if (primary_pos < 0.0f) primary_pos += buffer_size;
            }
        }

        // ---------------------------------------------------------------
        // Pitch-shifted read using rotary-head technique
        // Two overlapping read-heads, 180 degrees apart in phase
        // Triangular crossfade: one fading in while the other fades out
        // ---------------------------------------------------------------
        StereoFrame pitched_sample = {0.0f, 0.0f};
        float window_size = current_delay * 0.5f;
        if (window_size < 64.0f) window_size = 64.0f;

        for (int head = 0; head < 2; ++head) {
            // Advance phase for this head
            float phase = pitch_shift_phase_[head];

            // Triangular crossfade envelope based on phase (0-1)
            // Phase 0..0.5 -> ramp up, 0.5..1 -> ramp down
            float tri_env;
            if (phase < 0.5f) {
                tri_env = phase * 2.0f;
            } else {
                tri_env = 2.0f - phase * 2.0f;
            }

            // Offset from primary position based on phase
            float head_offset = phase * window_size;
            float head_pos = primary_pos + head_offset;
            if (buffer_size > 0.0f) {
                head_pos = std::fmod(head_pos, buffer_size);
                if (head_pos < 0.0f) head_pos += buffer_size;
            }

            // Read from buffer with Hermite interpolation
            float left = buffer_->ReadHermite(0, head_pos);
            float right = buffer_->ReadHermite(1, head_pos);

            pitched_sample.l += left * tri_env;
            pitched_sample.r += right * tri_env;

            // Advance the phase: increment determines pitch shift
            // pitch_ratio > 1 = faster read = higher pitch
            // The phase accumulator wraps at 1.0 (one window traversal)
            pitch_shift_phase_[head] += (pitch_shift_increment_ - 1.0f) / window_size;
            if (pitch_shift_phase_[head] >= 1.0f) {
                pitch_shift_phase_[head] -= 1.0f;
            }
            if (pitch_shift_phase_[head] < 0.0f) {
                pitch_shift_phase_[head] += 1.0f;
            }
        }

        // ---------------------------------------------------------------
        // Loop boundary crossfade (frozen mode only)
        // When near the loop start, blend with audio from the loop end
        // to create a seamless loop without clicks.
        // ---------------------------------------------------------------
        if (frozen_ && loop_xfade_gain < 1.0f) {
            // Read from near the end of the loop (mirror position)
            float xfade_len = static_cast<float>(kLoopXfadeSamples);
            if (xfade_len > loop_length_ * 0.25f) {
                xfade_len = loop_length_ * 0.25f;
            }
            float mirror_pos_in_loop = loop_length_ - xfade_len + read_position_;
            float mirror_pos = loop_start_ + mirror_pos_in_loop;
            if (buffer_size > 0.0f) {
                mirror_pos = std::fmod(mirror_pos, buffer_size);
                if (mirror_pos < 0.0f) mirror_pos += buffer_size;
            }
            float mirror_l = buffer_->ReadHermite(0, mirror_pos);
            float mirror_r = buffer_->ReadHermite(1, mirror_pos);
            // Blend: at read_position_=0, 100% mirror; at xfade boundary, 100% primary
            pitched_sample.l = pitched_sample.l * loop_xfade_gain + mirror_l * (1.0f - loop_xfade_gain);
            pitched_sample.r = pitched_sample.r * loop_xfade_gain + mirror_r * (1.0f - loop_xfade_gain);
        }

        // ---------------------------------------------------------------
        // Secondary tap (golden ratio of primary delay time)
        // ---------------------------------------------------------------
        StereoFrame mixed_sample = pitched_sample;

        if (secondary_mix > 0.0f) {
            float secondary_delay = current_delay * secondary_tap_ratio_;
            if (secondary_delay > buffer_size) {
                secondary_delay = buffer_size;
            }

            float secondary_pos;
            if (frozen_) {
                // In freeze mode, secondary tap also reads from the loop
                float sec_offset = secondary_delay;
                if (loop_length_ > 0.0f) {
                    sec_offset = std::fmod(sec_offset, loop_length_);
                    if (sec_offset < 0.0f) sec_offset += loop_length_;
                }
                secondary_pos = loop_start_ + sec_offset;
                if (buffer_size > 0.0f) {
                    secondary_pos = std::fmod(secondary_pos, buffer_size);
                    if (secondary_pos < 0.0f) secondary_pos += buffer_size;
                }
            } else {
                float write_pos = static_cast<float>(buffer_->write_head());
                secondary_pos = write_pos - secondary_delay;
                if (buffer_size > 0.0f) {
                    secondary_pos = std::fmod(secondary_pos, buffer_size);
                    if (secondary_pos < 0.0f) secondary_pos += buffer_size;
                }
            }

            float sec_l = buffer_->ReadHermite(0, secondary_pos);
            float sec_r = buffer_->ReadHermite(1, secondary_pos);

            // Mix both taps with equal gain
            float primary_gain = 1.0f - secondary_mix * 0.5f;
            float secondary_gain = secondary_mix * 0.5f;

            mixed_sample.l = pitched_sample.l * primary_gain + sec_l * secondary_gain;
            mixed_sample.r = pitched_sample.r * primary_gain + sec_r * secondary_gain;
        }

        // ---------------------------------------------------------------
        // SHAPE -> tremolo / slicer amplitude envelope
        // ---------------------------------------------------------------
        float env_gain = 1.0f;
        if (params.shape > 0.001f) {
            // Advance envelope phase
            envelope_phase_ += envelope_rate;
            if (envelope_phase_ >= 1.0f) {
                envelope_phase_ -= 1.0f;
            }

            // Generate envelope waveform based on shape
            float sine_env = 0.5f + 0.5f * std::sin(envelope_phase_ * kTwoPi);

            if (params.shape <= 0.5f) {
                // 0.0 -> 0.5: blend from steady (1.0) to sine tremolo
                float tremolo_depth = params.shape * 2.0f;  // 0 at shape=0, 1 at shape=0.5
                env_gain = 1.0f - tremolo_depth * (1.0f - sine_env);
            } else {
                // 0.5 -> 1.0: morph from sine tremolo to hard slicer (square gating)
                // Instead of a hard square, use a raised-cosine pulse that
                // sharpens as slicer_blend increases.  This avoids the
                // instant 0->1 and 1->0 transitions of a true square wave
                // which produce audible clicks.
                float slicer_blend = (params.shape - 0.5f) * 2.0f;  // 0 at 0.5, 1 at 1.0

                // Steepness increases from 1 (sine) to 8 (near-square with
                // smooth edges).  The raised-cosine pulse: take the sine
                // envelope and apply a power curve to steepen it.
                float steepness = 1.0f + slicer_blend * 7.0f;
                float steep_env = std::pow(sine_env, steepness);
                // Normalize so the peak stays at 1.0
                // (pow(1.0, n) == 1.0, pow(0.0, n) == 0.0, so extremes are fine)
                env_gain = steep_env;
            }
        }

        // Smooth the envelope gain to catch any remaining abrupt transitions
        // (e.g. when shape parameter itself changes suddenly)
        ONE_POLE(smoothed_envelope_gain_, env_gain, 0.05f);

        mixed_sample.l *= smoothed_envelope_gain_;
        mixed_sample.r *= smoothed_envelope_gain_;

        output[i] = mixed_sample;
    }
}

} // namespace beads
