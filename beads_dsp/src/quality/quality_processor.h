#pragma once

#include "../../include/beads/types.h"
#include "../util/dsp_utils.h"
#include "../util/svf.h"
#include "../random/random.h"

namespace beads {

// Simulates the 4 quality mode characters via DSP.
//
// Quality mode processing:
//   HiFi:      No degradation, pass-through.
//   Clouds:    LP at ~14kHz input.  12-bit quantization on output
//              (multiply by 2048, round, divide by 2048).
//   CleanLoFi: LP at ~10kHz on wet path only (dry bypasses).  No quantization.
//   Tape:      LP at ~10kHz.  Mono sum.  Gentle mu-law transfer curve (mu=64).
//              Wow LFO (~0.5 Hz, +/-0.075 semitones) +
//              flutter LFO (~6 Hz, +/-0.012 semitones).
class QualityProcessor {
public:
    void Init(float sample_rate);

    // Process input through quality mode character (called before recording)
    StereoFrame ProcessInput(StereoFrame input, QualityMode mode);

    // Process output through quality mode character (called after grain/delay)
    StereoFrame ProcessOutput(StereoFrame input, QualityMode mode);

    // Get wow/flutter pitch modulation (for tape mode grain read position).
    // Returns a pitch ratio multiplier (1.0 = no modulation).
    // num_samples: number of samples to advance the LFO by (typically block size).
    float GetPitchModulation(QualityMode mode, size_t num_samples = 1);

private:
    float sample_rate_ = 48000.0f;

    // LP filters for frequency band limiting per mode
    StateVariableFilter input_lp_l_;
    StateVariableFilter input_lp_r_;
    StateVariableFilter output_lp_l_;
    StateVariableFilter output_lp_r_;

    // Wow/flutter LFOs (tape mode)
    float wow_phase_ = 0.0f;
    float flutter_phase_ = 0.0f;

    // Noise generator for tape hiss
    Random noise_gen_;

    // Pre-computed per-sample LFO increments
    float wow_increment_ = 0.0f;
    float flutter_increment_ = 0.0f;

    // -- Constants --
    static constexpr float kWowHz = 0.5f;
    static constexpr float kFlutterHz = 6.0f;
    static constexpr float kWowSemitones = 0.075f;
    static constexpr float kFlutterSemitones = 0.012f;

    static constexpr float kCloudsInputLpHz  = 14000.0f;
    static constexpr float kCleanLoFiLpHz    = 10000.0f;
    static constexpr float kTapeLpHz         = 10000.0f;

    // 12-bit quantization scale (2^11 = 2048)
    static constexpr float kQuantScale = 2048.0f;

    // Tape hiss level (subtle)
    static constexpr float kTapeHissLevel = 0.00025f;

    // Quality mode transition: crossfade between old and new mode output
    QualityMode prev_input_mode_ = QualityMode::kHiFi;
    QualityMode prev_output_mode_ = QualityMode::kHiFi;
    static constexpr int kModeXfadeSamples = 64;
    int input_xfade_counter_ = 0;
    int output_xfade_counter_ = 0;
};

} // namespace beads
