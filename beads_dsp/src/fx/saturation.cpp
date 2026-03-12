#include "saturation.h"
#include <cmath>

namespace beads {

void Saturation::Init() {
    // No persistent state needed
}

// ---------------------------------------------------------------------------
// Asymmetric soft clip for tape character.
// Positive peaks are clipped a little harder (drive * 1.1) to mimic
// magnetic bias asymmetry.
// ---------------------------------------------------------------------------
float Saturation::AsymmetricSoftClip(float x) {
    if (x >= 0.0f) {
        return FastTanh(x * 1.1f);
    } else {
        return FastTanh(x * 0.9f);
    }
}

// ---------------------------------------------------------------------------
// Process: apply saturation curve matching the current quality mode.
// ---------------------------------------------------------------------------
float Saturation::Process(float input, QualityMode mode) {
    switch (mode) {
        case QualityMode::kHiFi:
            // Clean hard clip — transparent brickwall
            return HardClip(input, 1.0f);

        case QualityMode::kClouds:
            // Medium-drive soft clip (tanh-like)
            return SoftClip(input * 1.5f);

        case QualityMode::kCleanLoFi:
            // Asymmetric tape-style soft clip, moderate drive
            return AsymmetricSoftClip(input);

        case QualityMode::kTape:
            // Mu-law compression for warm tape character
            return MuLawCompress(input, 64.0f);
    }
    return input;
}

StereoFrame Saturation::Process(StereoFrame input, QualityMode mode) {
    return {
        Process(input.l, mode),
        Process(input.r, mode)
    };
}

// ---------------------------------------------------------------------------
// LimitFeedback: keep feedback gain under control per quality mode.
//
// HiFi is a hard wall; the others allow softer, more musical limiting
// so that high feedback sounds characterful rather than brittle.
// ---------------------------------------------------------------------------
float Saturation::LimitFeedback(float input, QualityMode mode) {
    switch (mode) {
        case QualityMode::kHiFi:
            // Brickwall at +/-1
            return HardClip(input, 1.0f);

        case QualityMode::kClouds:
            // Soft clip with moderate headroom
            return SoftClip(input);

        case QualityMode::kCleanLoFi:
            // Slightly compressed feedback
            return AsymmetricSoftClip(input * 0.9f);

        case QualityMode::kTape:
            // Mu-law keeps feedback warm and saturated.
            // Pre-clamp input so output stays in [-1, 1] (mu-law
            // output exceeds unity for |input| > 1).
            return MuLawCompress(Clamp(input, -1.0f, 1.0f), 32.0f);
    }
    return input;
}

StereoFrame Saturation::LimitFeedback(StereoFrame input, QualityMode mode) {
    return {
        LimitFeedback(input.l, mode),
        LimitFeedback(input.r, mode)
    };
}

} // namespace beads
