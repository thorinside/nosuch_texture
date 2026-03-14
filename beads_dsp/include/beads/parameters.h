#pragma once

#include "types.h"

namespace beads {

struct BeadsParameters {
    // Primary grain controls (0-1 normalized)
    float time = 0.5f;
    float size = 0.5f;
    float shape = 0.5f;
    float pitch = 0.0f;       // Semitones, -24 to +24
    float density = 0.5f;

    // Mix controls (0-1)
    float feedback = 0.0f;
    float dry_wet = 0.5f;
    float reverb = 0.0f;

    // Attenurandomizer knobs (-1 to +1, 0=noon)
    float time_ar = 0.0f;
    float size_ar = 0.0f;
    float shape_ar = 0.0f;
    float pitch_ar = 0.0f;

    // CV inputs (volts, 0 if unpatched)
    float time_cv = 0.0f;
    float size_cv = 0.0f;
    float shape_cv = 0.0f;
    float pitch_cv = 0.0f;
    bool time_cv_connected = false;
    bool size_cv_connected = false;
    bool shape_cv_connected = false;
    bool pitch_cv_connected = false;

    // Gate/clock/freeze
    bool gate = false;
    bool freeze = false;
    TriggerMode trigger_mode = TriggerMode::kLatched;
    QualityMode quality_mode = QualityMode::kHiFi;

    // Input config
    float manual_gain_db = NAN;  // NaN = auto-gain
    bool auto_gain = true;       // true = calibrate-and-lock auto-gain
    bool stereo_input = true;
};

} // namespace beads
