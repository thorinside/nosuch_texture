#include "quality_processor.h"
#include <cmath>

namespace beads {

void QualityProcessor::Init(float sample_rate) {
    sample_rate_ = sample_rate;

    input_lp_l_.Init();
    input_lp_r_.Init();
    output_lp_l_.Init();
    output_lp_r_.Init();

    // Set default cutoff frequencies — they'll be overridden per-mode in
    // Process*, but a sane default avoids uninitialized filter state.
    input_lp_l_.SetFrequencyHz(kCloudsInputLpHz, sample_rate_);
    input_lp_r_.SetFrequencyHz(kCloudsInputLpHz, sample_rate_);
    output_lp_l_.SetFrequencyHz(kCleanLoFiLpHz, sample_rate_);
    output_lp_r_.SetFrequencyHz(kCleanLoFiLpHz, sample_rate_);

    // Gentle resonance (Butterworth-ish)
    input_lp_l_.SetQ(0.707f);
    input_lp_r_.SetQ(0.707f);
    output_lp_l_.SetQ(0.707f);
    output_lp_r_.SetQ(0.707f);

    // LFO state
    wow_phase_ = 0.0f;
    flutter_phase_ = 0.0f;
    wow_increment_ = kWowHz / sample_rate_;
    flutter_increment_ = kFlutterHz / sample_rate_;

    noise_gen_.Init(0xBEAD5EED);
}

// ---------------------------------------------------------------------------
// ProcessInput: quality-mode coloring applied *before* recording into the
// grain buffer.
// ---------------------------------------------------------------------------
StereoFrame QualityProcessor::ProcessInput(StereoFrame input, QualityMode mode) {
    // Detect mode change and start crossfade
    if (mode != prev_input_mode_) {
        input_xfade_counter_ = kModeXfadeSamples;
        prev_input_mode_ = mode;
    }

    // Always run the input LP filters to keep their state fresh.
    // Set the cutoff based on the current mode (or a reasonable default
    // for modes that don't use the input LP).
    float cutoff_hz;
    switch (mode) {
        case QualityMode::kClouds:    cutoff_hz = kCloudsInputLpHz; break;
        case QualityMode::kTape:      cutoff_hz = kTapeLpHz; break;
        default:                      cutoff_hz = kCloudsInputLpHz; break;  // Nyquist-ish passthrough
    }
    input_lp_l_.SetFrequencyHz(cutoff_hz, sample_rate_);
    input_lp_r_.SetFrequencyHz(cutoff_hz, sample_rate_);
    float filtered_l = input_lp_l_.ProcessLP(input.l);
    float filtered_r = input_lp_r_.ProcessLP(input.r);

    StereoFrame result;
    switch (mode) {
        case QualityMode::kHiFi:
        case QualityMode::kCleanLoFi:
            // No input degradation — use unfiltered signal
            result = input;
            break;

        case QualityMode::kClouds:
            result = { filtered_l, filtered_r };
            break;

        case QualityMode::kTape: {
            // Mono sum
            float mono = (filtered_l + filtered_r) * 0.5f;
            // Add subtle tape hiss
            float hiss = noise_gen_.NextBipolar() * kTapeHissLevel;
            mono += hiss;
            // Mu-law compression
            mono = MuLawCompress(mono, 64.0f);
            result = { mono, mono };
            break;
        }
    }

    // Crossfade from the unprocessed input to the new mode's output
    // when a mode switch just happened.  This avoids the abrupt timbral
    // jump (especially into/out of tape mode's mono sum + mu-law).
    if (input_xfade_counter_ > 0) {
        float mix = static_cast<float>(input_xfade_counter_) / static_cast<float>(kModeXfadeSamples);
        // mix goes from 1 (all old = raw input) to 0 (all new mode)
        result.l = input.l * mix + result.l * (1.0f - mix);
        result.r = input.r * mix + result.r * (1.0f - mix);
        input_xfade_counter_--;
    }

    return result;
}

// ---------------------------------------------------------------------------
// ProcessOutput: quality-mode coloring applied *after* grain / delay readout.
// ---------------------------------------------------------------------------
StereoFrame QualityProcessor::ProcessOutput(StereoFrame input, QualityMode mode) {
    // Detect mode change and start crossfade
    if (mode != prev_output_mode_) {
        output_xfade_counter_ = kModeXfadeSamples;
        prev_output_mode_ = mode;
    }

    // Always run the output LP filters to keep their state fresh.
    float cutoff_hz;
    switch (mode) {
        case QualityMode::kCleanLoFi: cutoff_hz = kCleanLoFiLpHz; break;
        case QualityMode::kTape:      cutoff_hz = kTapeLpHz; break;
        default:                      cutoff_hz = kCleanLoFiLpHz; break;
    }
    output_lp_l_.SetFrequencyHz(cutoff_hz, sample_rate_);
    output_lp_r_.SetFrequencyHz(cutoff_hz, sample_rate_);

    // Feed-through: always tick the filters so their state tracks the signal.
    // For modes that use the LP output, we use it below; for modes that
    // don't, the filter still processes the sample to stay in sync.
    float lp_l = output_lp_l_.ProcessLP(input.l);
    float lp_r = output_lp_r_.ProcessLP(input.r);

    StereoFrame result;
    switch (mode) {
        case QualityMode::kHiFi:
            result = input;
            break;

        case QualityMode::kClouds: {
            float l = std::round(input.l * kQuantScale) / kQuantScale;
            float r = std::round(input.r * kQuantScale) / kQuantScale;
            result = { l, r };
            break;
        }

        case QualityMode::kCleanLoFi:
            result = { lp_l, lp_r };
            break;

        case QualityMode::kTape: {
            // Mu-law expansion.  The output LP was already ticked above
            // with the raw (compressed) input, which keeps the filter
            // state tracking the signal.  The LP result (lp_l/lp_r)
            // applied to the compressed signal is a reasonable
            // approximation; applying MuLawExpand *before* LP and
            // re-filtering would double-tick the SVF.  Instead, expand
            // the already-filtered result.
            result = {
                MuLawExpand(lp_l, 64.0f),
                MuLawExpand(lp_r, 64.0f)
            };
            break;
        }
    }

    // Crossfade from unprocessed input to new mode output on mode change
    if (output_xfade_counter_ > 0) {
        float mix = static_cast<float>(output_xfade_counter_) / static_cast<float>(kModeXfadeSamples);
        result.l = input.l * mix + result.l * (1.0f - mix);
        result.r = input.r * mix + result.r * (1.0f - mix);
        output_xfade_counter_--;
    }

    return result;
}

// ---------------------------------------------------------------------------
// GetPitchModulation: returns a pitch *ratio* multiplier for the current
// sample.  Only tape mode produces modulation; all others return 1.0.
//
// Wow  = slow (~0.5 Hz), +/- 0.075 semitones
// Flutter = fast (~6 Hz), +/- 0.012 semitones
// ---------------------------------------------------------------------------
float QualityProcessor::GetPitchModulation(QualityMode mode, size_t num_samples) {
    if (mode != QualityMode::kTape) {
        return 1.0f;
    }

    // Advance LFO phases by the number of samples in this block.
    // The LFO increments are per-sample, so multiply by block size.
    float advance = static_cast<float>(num_samples);
    wow_phase_ += wow_increment_ * advance;
    while (wow_phase_ >= 1.0f) wow_phase_ -= 1.0f;

    flutter_phase_ += flutter_increment_ * advance;
    while (flutter_phase_ >= 1.0f) flutter_phase_ -= 1.0f;

    // Combined pitch deviation in semitones
    float wow_st     = kWowSemitones     * std::sin(wow_phase_     * kTwoPi);
    float flutter_st = kFlutterSemitones * std::sin(flutter_phase_ * kTwoPi);

    // Convert semitones offset to ratio
    return SemitonesToRatio(wow_st + flutter_st);
}

} // namespace beads
