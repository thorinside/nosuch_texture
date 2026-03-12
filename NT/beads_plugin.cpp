/*
 * Beads — Disting NT Plugin
 * Granular texture synthesizer based on Mutable Instruments Beads
 *
 * Unity build: #includes all beads_dsp source files for cross-TU inlining.
 */

#include <new>
#include <cstring>
#include <cmath>
#include <distingnt/api.h>

// ============================================================================
// Unity build — include all beads_dsp source files
// ============================================================================

#include "random/random.cpp"
#include "random/attenurandomizer.cpp"
#include "buffer/recording_buffer.cpp"
#include "fx/saturation.cpp"
#include "fx/reverb.cpp"
#include "grain/grain.cpp"
#include "grain/grain_engine.cpp"
#include "grain/grain_scheduler.cpp"
#include "delay/delay_engine.cpp"
#include "input/auto_gain.cpp"
#include "quality/quality_processor.cpp"
#include "wavetable/wavetable_oscillator.cpp"
#include "beads_processor.cpp"

// ============================================================================
// Soft takeover (reuse proven pattern from nt_303)
// ============================================================================

struct SoftTakeoverState {
    float lastPotPos[3];
    float target[3];
};

static void initSoftTakeover(SoftTakeoverState* state) {
    for (int i = 0; i < 3; i++) {
        state->lastPotPos[i] = 0.5f;
        state->target[i] = 0.5f;
    }
}

static float processPotDelta(SoftTakeoverState* state, int potIndex, float potPos) {
    float delta = potPos - state->lastPotPos[potIndex];
    state->target[potIndex] += delta;
    if (state->target[potIndex] < 0.0f) state->target[potIndex] = 0.0f;
    if (state->target[potIndex] > 1.0f) state->target[potIndex] = 1.0f;

    // Snap when physical position is within 2% of target or at endpoints
    if (fabsf(potPos - state->target[potIndex]) < 0.02f ||
        potPos <= 0.01f || potPos >= 0.99f) {
        state->target[potIndex] = potPos;
    }

    state->lastPotPos[potIndex] = potPos;
    return state->target[potIndex];
}

// ============================================================================
// Parameter enums
// ============================================================================

enum {
    // I/O Routing
    kParamInputL,
    kParamInputR,
    kParamOutputL,
    kParamOutputLMode,
    kParamOutputR,
    kParamOutputRMode,

    // CV Input buses
    kParamTimeCvIn,
    kParamSizeCvIn,
    kParamShapeCvIn,
    kParamPitchCvIn,
    kParamDensityCvIn,
    kParamFreezeCvIn,
    kParamGateCvIn,
    kParamMacroCvIn,
    kParamMacroCvTarget,

    // Primary controls
    kParamTime,
    kParamSize,
    kParamShape,
    kParamPitch,
    kParamDensity,

    // Mix controls
    kParamFeedback,
    kParamDryWet,
    kParamReverb,

    // Attenurandomizers
    kParamTimeAR,
    kParamSizeAR,
    kParamShapeAR,
    kParamPitchAR,

    // Mode/Config
    kParamFreeze,
    kParamTriggerMode,
    kParamQualityMode,
    kParamInputGain,
    kParamStereoInput,

    kNumParams
};

// ============================================================================
// Enum strings
// ============================================================================

static const char* const freezeStrings[] = { "Off", "On", NULL };
static const char* const triggerStrings[] = { "Latched", "Gated", "Clocked", NULL };
static const char* const qualityStrings[] = { "HiFi", "Clouds", "Clean LoFi", "Tape", NULL };
static const char* const macroCvTargetStrings[] = { "Feedback", "Dry/Wet", "Reverb", NULL };
static const char* const stereoInputStrings[] = { "Mono", "Stereo", NULL };

// ============================================================================
// Parameter definitions
// ============================================================================

static const _NT_parameter parameters[] = {
    // I/O Routing
    NT_PARAMETER_AUDIO_INPUT( "Input L", 1, 1 )
    NT_PARAMETER_AUDIO_INPUT( "Input R", 0, 2 )
    NT_PARAMETER_AUDIO_OUTPUT_WITH_MODE( "Output L", 1, 13 )
    NT_PARAMETER_AUDIO_OUTPUT_WITH_MODE( "Output R", 1, 14 )

    // CV Input buses
    NT_PARAMETER_CV_INPUT( "Time CV", 0, 0 )
    NT_PARAMETER_CV_INPUT( "Size CV", 0, 0 )
    NT_PARAMETER_CV_INPUT( "Shape CV", 0, 0 )
    NT_PARAMETER_CV_INPUT( "Pitch CV", 0, 0 )
    NT_PARAMETER_CV_INPUT( "Density CV", 0, 0 )
    NT_PARAMETER_CV_INPUT( "Freeze gate", 0, 0 )
    NT_PARAMETER_CV_INPUT( "Seed gate", 0, 0 )
    NT_PARAMETER_CV_INPUT( "Macro CV", 0, 0 )
    { .name = "Macro target", .min = 0, .max = 2, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = macroCvTargetStrings },

    // Primary controls
    { .name = "Time",    .min = 0, .max = 1000, .def = 500, .unit = kNT_unitNone, .scaling = kNT_scaling1000, .enumStrings = NULL },
    { .name = "Size",    .min = 0, .max = 1000, .def = 500, .unit = kNT_unitNone, .scaling = kNT_scaling1000, .enumStrings = NULL },
    { .name = "Shape",   .min = 0, .max = 1000, .def = 500, .unit = kNT_unitNone, .scaling = kNT_scaling1000, .enumStrings = NULL },
    { .name = "Pitch",   .min = -2400, .max = 2400, .def = 0, .unit = kNT_unitCents, .scaling = 0, .enumStrings = NULL },
    { .name = "Density", .min = 0, .max = 1000, .def = 500, .unit = kNT_unitNone, .scaling = kNT_scaling1000, .enumStrings = NULL },

    // Mix controls
    { .name = "Feedback", .min = 0, .max = 100, .def = 0,  .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
    { .name = "Dry/Wet",  .min = 0, .max = 100, .def = 50, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
    { .name = "Reverb",   .min = 0, .max = 100, .def = 0,  .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },

    // Attenurandomizers
    { .name = "Time AR",  .min = -100, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
    { .name = "Size AR",  .min = -100, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
    { .name = "Shape AR", .min = -100, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
    { .name = "Pitch AR", .min = -100, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },

    // Mode/Config
    { .name = "Freeze",       .min = 0, .max = 1, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = freezeStrings },
    { .name = "Trigger mode", .min = 0, .max = 2, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = triggerStrings },
    { .name = "Quality",      .min = 0, .max = 3, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = qualityStrings },
    { .name = "Input gain",   .min = -60, .max = 20, .def = -60, .unit = kNT_unitDb_minInf, .scaling = 0, .enumStrings = NULL },
    { .name = "Stereo input", .min = 0, .max = 1, .def = 1, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = stereoInputStrings },
};

// ============================================================================
// Parameter pages
// ============================================================================

static const uint8_t pageGrain[] = { kParamTime, kParamSize, kParamShape, kParamPitch, kParamDensity };
static const uint8_t pageMix[] = { kParamFeedback, kParamDryWet, kParamReverb };
static const uint8_t pageAR[] = { kParamTimeAR, kParamSizeAR, kParamShapeAR, kParamPitchAR };
static const uint8_t pageMode[] = { kParamFreeze, kParamTriggerMode, kParamQualityMode, kParamInputGain, kParamStereoInput };
static const uint8_t pageRouting[] = {
    kParamInputL, kParamInputR, kParamOutputL, kParamOutputLMode,
    kParamOutputR, kParamOutputRMode,
    kParamTimeCvIn, kParamSizeCvIn, kParamShapeCvIn, kParamPitchCvIn,
    kParamDensityCvIn, kParamFreezeCvIn, kParamGateCvIn,
    kParamMacroCvIn, kParamMacroCvTarget
};

static const _NT_parameterPage pages[] = {
    { .name = "GRAIN",   .numParams = ARRAY_SIZE(pageGrain),   .group = 1, .unused = {}, .params = pageGrain },
    { .name = "MIX",     .numParams = ARRAY_SIZE(pageMix),     .group = 1, .unused = {}, .params = pageMix },
    { .name = "AR",      .numParams = ARRAY_SIZE(pageAR),      .group = 1, .unused = {}, .params = pageAR },
    { .name = "MODE",    .numParams = ARRAY_SIZE(pageMode),     .group = 2, .unused = {}, .params = pageMode },
    { .name = "ROUTING", .numParams = ARRAY_SIZE(pageRouting), .group = 3, .unused = {}, .params = pageRouting },
};

static const _NT_parameterPages parameterPages = {
    .numPages = ARRAY_SIZE(pages),
    .pages = pages,
};

// ============================================================================
// Algorithm struct
// ============================================================================

struct _beadsAlgorithm : public _NT_algorithm {
    _beadsAlgorithm() {}
    ~_beadsAlgorithm() {}

    beads::BeadsProcessor processor;
    beads::BeadsParameters cachedParams;

    // UI state
    SoftTakeoverState potState;

    // Encoder-controlled parameter accumulators
    int densityAccum;    // 0-1000
    int pitchAccum;      // -2400 to +2400 cents

    // Gate edge detection
    bool gateHigh;
    bool freezeHigh;

    // Button press state for SEED (momentary)
    bool seedButtonHeld;

    // Display cache (updated from audio thread, read by UI thread)
    volatile int displayGrainCount;
    volatile float displayInputLevel;
};

// ============================================================================
// Helper: clamp
// ============================================================================

static inline float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline int clampi(int x, int lo, int hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// ============================================================================
// Factory functions
// ============================================================================

static void calculateRequirements(_NT_algorithmRequirements& req, const int32_t* specifications) {
    req.numParameters = kNumParams;
    req.sram = sizeof(_beadsAlgorithm);
    // DRAM for beads processor buffers (~1.6MB at 48kHz, ~3.1MB at 96kHz)
    auto memReq = beads::BeadsProcessor::GetMemoryRequirements((float)NT_globals.sampleRate);
    req.dram = (uint32_t)memReq.total_bytes;
    req.dtc = 0;
    req.itc = 0;
}

static _NT_algorithm* construct(const _NT_algorithmMemoryPtrs& ptrs,
                                 const _NT_algorithmRequirements& req,
                                 const int32_t* specifications) {
    _beadsAlgorithm* alg = new (ptrs.sram) _beadsAlgorithm();
    alg->parameters = parameters;
    alg->parameterPages = &parameterPages;

    // Initialize the beads processor with DRAM
    alg->processor.Init(ptrs.dram, req.dram, (float)NT_globals.sampleRate);

    // Initialize cached params to defaults
    alg->cachedParams = beads::BeadsParameters{};

    // UI state
    initSoftTakeover(&alg->potState);
    alg->densityAccum = 500;
    alg->pitchAccum = 0;

    // Gate state
    alg->gateHigh = false;
    alg->freezeHigh = false;
    alg->seedButtonHeld = false;

    // Display
    alg->displayGrainCount = 0;
    alg->displayInputLevel = 0.0f;

    return alg;
}

// ============================================================================
// parameterChanged
// ============================================================================

static void parameterChanged(_NT_algorithm* self, int p) {
    // Most parameter processing happens in step() since we need to combine
    // knob values with CV. Nothing to cache here.
    (void)self;
    (void)p;
}

// ============================================================================
// step — primary audio processing
// ============================================================================

static void step(_NT_algorithm* self, float* busFrames, int numFramesBy4) {
    _beadsAlgorithm* alg = (_beadsAlgorithm*)self;
    const int16_t* v = alg->v;
    int numFrames = numFramesBy4 * 4;

    // --- Read audio input ---
    beads::StereoFrame input[16];  // NT max is typically 4-16 frames
    int inputBusL = v[kParamInputL] - 1;
    int inputBusR = v[kParamInputR] - 1;
    bool stereoIn = (inputBusR >= 0);

    for (int i = 0; i < numFrames; ++i) {
        float l = (inputBusL >= 0) ? busFrames[inputBusL * numFrames + i] : 0.0f;
        float r = stereoIn ? busFrames[inputBusR * numFrames + i] : l;
        // Scale ±5V bus voltage to ±1.0 internal range
        input[i].l = l * 0.2f;
        input[i].r = r * 0.2f;
    }

    // --- Read CV buses ---
    float timeCv = 0.0f, sizeCv = 0.0f, shapeCv = 0.0f, pitchCv = 0.0f;
    float densityCv = 0.0f;
    bool timeCvConnected = false, sizeCvConnected = false;
    bool shapeCvConnected = false, pitchCvConnected = false;

    // Read CV values (average across block for stability)
    if (v[kParamTimeCvIn] != 0) {
        timeCvConnected = true;
        const float* bus = busFrames + (v[kParamTimeCvIn] - 1) * numFrames;
        for (int i = 0; i < numFrames; i++) timeCv += bus[i];
        timeCv = (timeCv / numFrames) / 5.0f;  // ±5V → ±1.0
    }
    if (v[kParamSizeCvIn] != 0) {
        sizeCvConnected = true;
        const float* bus = busFrames + (v[kParamSizeCvIn] - 1) * numFrames;
        for (int i = 0; i < numFrames; i++) sizeCv += bus[i];
        sizeCv = (sizeCv / numFrames) / 5.0f;
    }
    if (v[kParamShapeCvIn] != 0) {
        shapeCvConnected = true;
        const float* bus = busFrames + (v[kParamShapeCvIn] - 1) * numFrames;
        for (int i = 0; i < numFrames; i++) shapeCv += bus[i];
        shapeCv = (shapeCv / numFrames) / 5.0f;
    }
    if (v[kParamPitchCvIn] != 0) {
        pitchCvConnected = true;
        const float* bus = busFrames + (v[kParamPitchCvIn] - 1) * numFrames;
        for (int i = 0; i < numFrames; i++) pitchCv += bus[i];
        pitchCv = (pitchCv / numFrames) * 12.0f;  // 1V/oct → semitones
    }
    if (v[kParamDensityCvIn] != 0) {
        const float* bus = busFrames + (v[kParamDensityCvIn] - 1) * numFrames;
        for (int i = 0; i < numFrames; i++) densityCv += bus[i];
        densityCv = (densityCv / numFrames) / 5.0f;
    }

    // --- Gate detection with hysteresis ---
    bool gateFromCv = alg->gateHigh;
    if (v[kParamGateCvIn] != 0) {
        const float* bus = busFrames + (v[kParamGateCvIn] - 1) * numFrames;
        for (int i = 0; i < numFrames; i++) {
            if (!alg->gateHigh && bus[i] > 1.0f) alg->gateHigh = true;
            else if (alg->gateHigh && bus[i] < 0.5f) alg->gateHigh = false;
        }
        gateFromCv = alg->gateHigh;
    } else {
        gateFromCv = false;
    }

    bool freezeFromCv = alg->freezeHigh;
    if (v[kParamFreezeCvIn] != 0) {
        const float* bus = busFrames + (v[kParamFreezeCvIn] - 1) * numFrames;
        for (int i = 0; i < numFrames; i++) {
            if (!alg->freezeHigh && bus[i] > 1.0f) alg->freezeHigh = true;
            else if (alg->freezeHigh && bus[i] < 0.5f) alg->freezeHigh = false;
        }
        freezeFromCv = alg->freezeHigh;
    } else {
        freezeFromCv = false;
    }

    // --- Macro CV ---
    float macroCv = 0.0f;
    if (v[kParamMacroCvIn] != 0) {
        const float* bus = busFrames + (v[kParamMacroCvIn] - 1) * numFrames;
        for (int i = 0; i < numFrames; i++) macroCv += bus[i];
        macroCv = clampf((macroCv / numFrames) / 5.0f, 0.0f, 1.0f);
    }

    // --- Build BeadsParameters ---
    beads::BeadsParameters& p = alg->cachedParams;

    p.time = v[kParamTime] / 1000.0f;
    p.size = v[kParamSize] / 1000.0f;
    p.shape = v[kParamShape] / 1000.0f;
    p.pitch = v[kParamPitch] / 100.0f;  // cents → semitones
    p.density = v[kParamDensity] / 1000.0f;

    p.feedback = v[kParamFeedback] / 100.0f;
    p.dry_wet = v[kParamDryWet] / 100.0f;
    p.reverb = v[kParamReverb] / 100.0f;

    // Apply macro CV to target
    int macroTarget = v[kParamMacroCvTarget];
    if (macroTarget == 0) p.feedback = clampf(p.feedback + macroCv, 0.0f, 1.0f);
    else if (macroTarget == 1) p.dry_wet = clampf(p.dry_wet + macroCv, 0.0f, 1.0f);
    else p.reverb = clampf(p.reverb + macroCv, 0.0f, 1.0f);

    p.time_ar = v[kParamTimeAR] / 100.0f;
    p.size_ar = v[kParamSizeAR] / 100.0f;
    p.shape_ar = v[kParamShapeAR] / 100.0f;
    p.pitch_ar = v[kParamPitchAR] / 100.0f;

    p.time_cv = timeCv;
    p.size_cv = sizeCv;
    p.shape_cv = shapeCv;
    p.pitch_cv = pitchCv;
    p.time_cv_connected = timeCvConnected;
    p.size_cv_connected = sizeCvConnected;
    p.shape_cv_connected = shapeCvConnected;
    p.pitch_cv_connected = pitchCvConnected;

    // Gate: OR of button press and CV
    p.gate = alg->seedButtonHeld || gateFromCv;

    // Freeze: OR of parameter toggle and CV
    p.freeze = (v[kParamFreeze] != 0) || freezeFromCv;

    p.trigger_mode = (beads::TriggerMode)v[kParamTriggerMode];
    p.quality_mode = (beads::QualityMode)v[kParamQualityMode];

    // Input gain: -60 = auto (NaN), otherwise dB
    int gainParam = v[kParamInputGain];
    if (gainParam <= -60)
        p.manual_gain_db = NAN;
    else
        p.manual_gain_db = (float)gainParam;

    p.stereo_input = (v[kParamStereoInput] != 0);

    // --- Process ---
    alg->processor.SetParameters(p);

    beads::StereoFrame output[16];
    alg->processor.Process(input, output, numFrames);

    // --- Write output ---
    int outBusL = v[kParamOutputL] - 1;
    int outBusR = v[kParamOutputR] - 1;
    bool replaceL = v[kParamOutputLMode];
    bool replaceR = v[kParamOutputRMode];

    if (outBusL >= 0) {
        float* out = busFrames + outBusL * numFrames;
        if (replaceL) {
            for (int i = 0; i < numFrames; ++i) out[i] = output[i].l * 5.0f;
        } else {
            for (int i = 0; i < numFrames; ++i) out[i] += output[i].l * 5.0f;
        }
    }
    if (outBusR >= 0) {
        float* out = busFrames + outBusR * numFrames;
        if (replaceR) {
            for (int i = 0; i < numFrames; ++i) out[i] = output[i].r * 5.0f;
        } else {
            for (int i = 0; i < numFrames; ++i) out[i] += output[i].r * 5.0f;
        }
    }

    // --- Update display cache ---
    alg->displayGrainCount = alg->processor.ActiveGrainCount();
    alg->displayInputLevel = alg->processor.InputLevel();
}

// ============================================================================
// Custom UI
// ============================================================================

static uint32_t hasCustomUi(_NT_algorithm* self) {
    (void)self;
    // Claim all 3 pots + pot buttons, both encoders + encoder buttons.
    // Pots always map to Time/Size/Shape (GRAIN controls).
    // Enc L = Density, Enc R = Pitch. Buttons: freeze + seed.
    return kNT_potL | kNT_potC | kNT_potR
         | kNT_potButtonL | kNT_potButtonC | kNT_potButtonR
         | kNT_encoderL | kNT_encoderR
         | kNT_encoderButtonL | kNT_encoderButtonR;
}

static void customUi(_NT_algorithm* self, const _NT_uiData& data) {
    _beadsAlgorithm* alg = (_beadsAlgorithm*)self;
    int algIdx = NT_algorithmIndex(self);
    uint32_t paramOff = NT_parameterOffset();

    // --- Encoder L press: toggle FREEZE (all pages) ---
    if ((data.controls & kNT_encoderButtonL) && !(data.lastButtons & kNT_encoderButtonL)) {
        int newFreeze = alg->v[kParamFreeze] ? 0 : 1;
        NT_setParameterFromUi(algIdx, kParamFreeze + paramOff, newFreeze);
    }

    // --- Encoder R press: momentary SEED (all pages) ---
    bool encRPressed = (data.controls & kNT_encoderButtonR) != 0;
    alg->seedButtonHeld = encRPressed;

    // --- Pots always map to Time/Size/Shape (GRAIN controls) ---
    if (data.controls & kNT_potL) {
        float t = processPotDelta(&alg->potState, 0, data.pots[0]);
        NT_setParameterFromUi(algIdx, kParamTime + paramOff, (int)(t * 1000.0f + 0.5f));
    }
    if (data.controls & kNT_potC) {
        float t = processPotDelta(&alg->potState, 1, data.pots[1]);
        NT_setParameterFromUi(algIdx, kParamSize + paramOff, (int)(t * 1000.0f + 0.5f));
    }
    if (data.controls & kNT_potR) {
        float t = processPotDelta(&alg->potState, 2, data.pots[2]);
        NT_setParameterFromUi(algIdx, kParamShape + paramOff, (int)(t * 1000.0f + 0.5f));
    }

    // Enc L: Density (±10 per click, range 0-1000)
    if (data.encoders[0] != 0) {
        alg->densityAccum = clampi(alg->densityAccum + data.encoders[0] * 10, 0, 1000);
        NT_setParameterFromUi(algIdx, kParamDensity + paramOff, alg->densityAccum);
    }

    // Enc R: Pitch (±100 cents = ±1 semitone per click)
    if (data.encoders[1] != 0) {
        alg->pitchAccum = clampi(alg->pitchAccum + data.encoders[1] * 100, -2400, 2400);
        NT_setParameterFromUi(algIdx, kParamPitch + paramOff, alg->pitchAccum);
    }

    // --- Pot button presses: reset Time/Size/Shape to center ---
    if ((data.controls & kNT_potButtonL) && !(data.lastButtons & kNT_potButtonL)) {
        NT_setParameterFromUi(algIdx, kParamTime + paramOff, 500);
        alg->potState.target[0] = 0.5f;
    }
    if ((data.controls & kNT_potButtonC) && !(data.lastButtons & kNT_potButtonC)) {
        NT_setParameterFromUi(algIdx, kParamSize + paramOff, 500);
        alg->potState.target[1] = 0.5f;
    }
    if ((data.controls & kNT_potButtonR) && !(data.lastButtons & kNT_potButtonR)) {
        NT_setParameterFromUi(algIdx, kParamShape + paramOff, 500);
        alg->potState.target[2] = 0.5f;
    }
}

// ============================================================================
// setupUi — sync pot positions when UI becomes active or page changes
// ============================================================================

static void setupUi(_NT_algorithm* self, _NT_float3& pots) {
    _beadsAlgorithm* alg = (_beadsAlgorithm*)self;

    // Pots always map to Time/Size/Shape
    pots[0] = alg->v[kParamTime] / 1000.0f;
    pots[1] = alg->v[kParamSize] / 1000.0f;
    pots[2] = alg->v[kParamShape] / 1000.0f;

    // Initialize soft takeover targets from current pot positions
    for (int i = 0; i < 3; i++) {
        alg->potState.target[i] = pots[i];
        alg->potState.lastPotPos[i] = pots[i];
    }

    // Sync encoder accumulators from current param values
    alg->densityAccum = alg->v[kParamDensity];
    alg->pitchAccum = alg->v[kParamPitch];
}

// ============================================================================
// draw — custom OLED display
// ============================================================================

static const char* qualityLabel(int q) {
    switch (q) {
    case 0: return "HiFi";
    case 1: return "Clouds";
    case 2: return "LoFi";
    case 3: return "Tape";
    default: return "?";
    }
}

static const char* triggerLabel(int t) {
    switch (t) {
    case 0: return "Latch";
    case 1: return "Gate";
    case 2: return "Clock";
    default: return "?";
    }
}

static bool draw(_NT_algorithm* self) {
    _beadsAlgorithm* alg = (_beadsAlgorithm*)self;
    const int16_t* v = alg->v;

    // Row 12-23: Mode info + encoder params
    char buf[64];

    // Quality and trigger mode labels
    NT_drawText(0, 14, qualityLabel(v[kParamQualityMode]), 15, kNT_textLeft, kNT_textTiny);
    NT_drawText(30, 14, triggerLabel(v[kParamTriggerMode]), 15, kNT_textLeft, kNT_textTiny);

    // Density value
    int len = 0;
    buf[len++] = 'D'; buf[len++] = ':';
    len += NT_floatToString(buf + len, v[kParamDensity] / 1000.0f, 2);
    buf[len] = 0;
    NT_drawText(80, 14, buf, 15, kNT_textLeft, kNT_textTiny);

    // Pitch value
    len = 0;
    buf[len++] = 'P'; buf[len++] = ':';
    if (v[kParamPitch] >= 0) buf[len++] = '+';
    len += NT_intToString(buf + len, v[kParamPitch] / 100);
    buf[len++] = 's'; buf[len++] = 't';
    buf[len] = 0;
    NT_drawText(130, 14, buf, 15, kNT_textLeft, kNT_textTiny);

    // Row 24-31: pot bar graphs (page-dependent)
    int barY = 24;
    int barH = 6;
    int barW = 70;
    int barSpacing = 86;

    // Always show Time/Size/Shape bar graphs (pot-controlled params)
    float potValues[3] = {
        v[kParamTime] / 1000.0f,
        v[kParamSize] / 1000.0f,
        v[kParamShape] / 1000.0f,
    };

    for (int pot = 0; pot < 3; pot++) {
        int x0 = pot * barSpacing + 5;
        // Background
        NT_drawShapeI(kNT_box, x0, barY, x0 + barW, barY + barH, 4);
        // Filled portion
        int fillW = (int)(potValues[pot] * barW);
        if (fillW > 0) {
            NT_drawShapeI(kNT_rectangle, x0, barY, x0 + fillW, barY + barH, 10);
        }
    }

    // Row 32-47: grain activity visualization
    int grainCount = alg->displayGrainCount;
    int dotY = 40;
    int maxDots = 30;
    int dotSpacing = 8;
    int dotStartX = (256 - maxDots * dotSpacing) / 2;
    for (int i = 0; i < maxDots; i++) {
        int x = dotStartX + i * dotSpacing;
        int brightness = (i < grainCount) ? 15 : 3;
        NT_drawShapeI(kNT_rectangle, x, dotY, x + 4, dotY + 4, brightness);
    }

    // Row 48-63: Status footer
    // Grain count
    len = NT_intToString(buf, grainCount);
    buf[len] = 0;
    NT_drawText(0, 52, buf, 15, kNT_textLeft, kNT_textTiny);
    NT_drawText(12, 52, "grains", 8, kNT_textLeft, kNT_textTiny);

    // Input level meter
    float level = alg->displayInputLevel;
    int meterX = 80;
    int meterW = 60;
    NT_drawText(meterX - 12, 52, "IN", 8, kNT_textLeft, kNT_textTiny);
    NT_drawShapeI(kNT_box, meterX, 52, meterX + meterW, 58, 4);
    int meterFill = (int)(clampf(level, 0.0f, 1.0f) * meterW);
    if (meterFill > 0) {
        int meterColor = (level > 0.9f) ? 15 : 10;
        NT_drawShapeI(kNT_rectangle, meterX, 52, meterX + meterFill, 58, meterColor);
    }

    // Freeze indicator
    if (v[kParamFreeze] != 0) {
        NT_drawText(200, 52, "FROZEN", 15, kNT_textLeft, kNT_textTiny);
    }

    // Return false to keep standard parameter display at top
    return false;
}

// ============================================================================
// Factory definition
// ============================================================================

static const _NT_factory factory = {
    .guid = NT_MULTICHAR('T', 'h', 'B', 'e'),
    .name = "Beads",
    .description = "Granular texture synthesizer",
    .numSpecifications = 0,
    .specifications = NULL,
    .calculateStaticRequirements = NULL,
    .initialise = NULL,
    .calculateRequirements = calculateRequirements,
    .construct = construct,
    .parameterChanged = parameterChanged,
    .step = step,
    .draw = draw,
    .midiRealtime = NULL,
    .midiMessage = NULL,
    .tags = kNT_tagEffect | kNT_tagDelay,
    .hasCustomUi = hasCustomUi,
    .customUi = customUi,
    .setupUi = setupUi,
    .serialise = NULL,
    .deserialise = NULL,
    .midiSysEx = NULL,
    .parameterUiPrefix = NULL,
    .parameterString = NULL,
};

// ============================================================================
// Plugin entry point
// ============================================================================

extern "C" uintptr_t pluginEntry(_NT_selector selector, uint32_t data) {
    switch (selector) {
    case kNT_selector_version:
        return kNT_apiVersionCurrent;
    case kNT_selector_numFactories:
        return 1;
    case kNT_selector_factoryInfo:
        return (uintptr_t)((data == 0) ? &factory : NULL);
    }
    return 0;
}
