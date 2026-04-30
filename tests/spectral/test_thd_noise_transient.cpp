// Spectral / measurement tests for THD+N, noise floor, transient response and
// mu-law roundtrip.
//
// Spec artifact: f43b6fa3-b457-4872-990e-e71bb0baf78a
//   §3.5 Frozen-buffer trick
//   §4.3 THD methodology
//   §4.5 Transient response
//   §4.6 Noise floor
//   §5   Per-mode targets
//
// All BeadsProcessor-driven tests configure the processor for a *dry path*
// pass-through (`dry_wet=0.0`) unless explicitly noted.  This is the only
// signal path that is approximately LTI: at `dry_wet=0` the output is the dry
// chain `input -> AutoGain -> Reverb(amount=0) -> output`.  The wet chain
// includes grain windowing which is inherently non-linear vs. an LTI pipeline,
// and is exercised separately for Clouds (where its 12-bit quantization is the
// characteristic distortion) via a dedicated SECTION.
//
// All tests disable auto-gain calibration ramps by setting
// `auto_gain=false` and `manual_gain_db=0.0f` so that the input/output gain is
// flat and deterministic.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "beads/beads.h"
#include "quality/quality_processor.h"

#include "spectral_helpers.h"

using namespace beads_spec;
using beads::BeadsParameters;
using beads::BeadsProcessor;
using beads::QualityMode;
using beads::QualityProcessor;
using beads::StereoFrame;
using beads::TriggerMode;

namespace {

constexpr float kSr = 48000.0f;

// Default common parameters for the BeadsProcessor in measurement tests.
BeadsParameters MakeBaseParams(QualityMode mode) {
    BeadsParameters p;
    p.size = 0.5f;
    p.shape = 0.5f;
    p.pitch = 0.0f;
    p.density = 0.5f;
    p.feedback = 0.0f;
    p.dry_wet = 0.0f;     // overridden in many tests
    p.reverb = 0.0f;
    p.gate = false;
    p.freeze = false;
    p.trigger_mode = TriggerMode::kLatched;
    p.quality_mode = mode;
    p.auto_gain = false;
    p.manual_gain_db = 0.0f;
    p.time = 0.5f;
    return p;
}

// Allocate and initialise a BeadsProcessor with the standard sample rate.
struct Proc {
    std::vector<uint8_t> memory;
    BeadsProcessor processor;

    Proc() {
        auto req = BeadsProcessor::GetMemoryRequirements(kSr);
        memory.assign(req.total_bytes, 0u);
        processor.Init(memory.data(), memory.size(), kSr);
    }
};

// Run `n` frames of mono input (broadcast to stereo) through the processor and
// capture the left channel of the output.
void RunMono(BeadsProcessor& proc, const float* input_mono, float* output_left,
             std::size_t n) {
    constexpr std::size_t kChunk = 256;
    std::vector<StereoFrame> in_buf(kChunk);
    std::vector<StereoFrame> out_buf(kChunk);
    std::size_t i = 0;
    while (i < n) {
        std::size_t block = std::min(kChunk, n - i);
        for (std::size_t j = 0; j < block; ++j) {
            in_buf[j] = { input_mono[i + j], input_mono[i + j] };
        }
        proc.Process(in_buf.data(), out_buf.data(), block);
        for (std::size_t j = 0; j < block; ++j) {
            output_left[i + j] = out_buf[j].l;
        }
        i += block;
    }
}

// Run `n` frames of silence through the processor and capture the left channel.
void RunSilence(BeadsProcessor& proc, float* output_left, std::size_t n) {
    constexpr std::size_t kChunk = 256;
    std::vector<StereoFrame> in_buf(kChunk, {0.0f, 0.0f});
    std::vector<StereoFrame> out_buf(kChunk);
    std::size_t i = 0;
    while (i < n) {
        std::size_t block = std::min(kChunk, n - i);
        proc.Process(in_buf.data(), out_buf.data(), block);
        for (std::size_t j = 0; j < block; ++j) {
            output_left[i + j] = out_buf[j].l;
        }
        i += block;
    }
}

// Pick the FFT bin nearest 1 kHz given the frame count.
float NearestBinHz(float target_hz, std::size_t fft_n, float sr) {
    const float bin_w = sr / static_cast<float>(fft_n);
    long bin = static_cast<long>(std::lround(target_hz / bin_w));
    if (bin < 0) bin = 0;
    return static_cast<float>(bin) * bin_w;
}

// Common THD+N measurement on the dry path: drive 1 kHz at amp 0.3 and measure
// the residual after settling.  Returns measured THD+N in percent.
float MeasureDryPathThdPct(QualityMode mode, const std::string& mode_tag) {
    constexpr std::size_t kSettle = 4096;
    constexpr std::size_t kFftN = 8192;
    constexpr std::size_t kTotal = kSettle + kFftN;

    Proc proc;
    BeadsParameters params = MakeBaseParams(mode);
    params.dry_wet = 0.0f;  // dry path
    proc.processor.SetParameters(params);

    // Bin-align 1 kHz to the FFT length so the fundamental sits cleanly on a
    // single bin.
    const float f0 = NearestBinHz(1000.0f, kFftN, kSr);

    std::vector<float> input(kTotal);
    std::vector<float> output(kTotal);
    GenSine(input.data(), kTotal, f0, kSr, 0.3f);

    RunMono(proc.processor, input.data(), output.data(), kTotal);

    WriteSamplesCsv(OutputPath("thd_" + mode_tag + "_input.csv"),
                    input.data(), kTotal);
    WriteSamplesCsv(OutputPath("thd_" + mode_tag + "_output.csv"),
                    output.data(), kTotal);

    std::vector<std::complex<float>> spec(kFftN);
    Fft(output.data() + kSettle, kFftN, spec.data());
    return ThdPlusNPct(spec.data(), kFftN, kSr, f0);
}

// Wet-path THD+N measurement (used for Clouds): fill the buffer with a sine
// for ~1 second, then capture another stretch of `kFftN` samples and FFT.
float MeasureWetPathThdPct(QualityMode mode, const std::string& mode_tag) {
    constexpr std::size_t kFill = 48000;     // 1 s
    constexpr std::size_t kFftN = 8192;
    constexpr std::size_t kTotal = kFill + kFftN;

    Proc proc;
    BeadsParameters params = MakeBaseParams(mode);
    params.dry_wet = 1.0f;          // wet path
    params.density = 0.2f;          // ensure grains are firing densely
    params.size = 0.5f;
    params.time = 0.0f;
    params.trigger_mode = TriggerMode::kLatched;
    proc.processor.SetParameters(params);

    const float f0 = NearestBinHz(1000.0f, kFftN, kSr);

    std::vector<float> input(kTotal);
    std::vector<float> output(kTotal);
    GenSine(input.data(), kTotal, f0, kSr, 0.3f);

    RunMono(proc.processor, input.data(), output.data(), kTotal);

    WriteSamplesCsv(OutputPath("thd_" + mode_tag + "_wet_input.csv"),
                    input.data(), kTotal);
    WriteSamplesCsv(OutputPath("thd_" + mode_tag + "_wet_output.csv"),
                    output.data(), kTotal);

    std::vector<std::complex<float>> spec(kFftN);
    Fft(output.data() + kFill, kFftN, spec.data());
    return ThdPlusNPct(spec.data(), kFftN, kSr, f0);
}

// ---------------------------------------------------------------------------
// Helpers for transient/IR response.
// ---------------------------------------------------------------------------

struct TransientStats {
    float peak;
    std::size_t peak_idx;
    float rise_us;
    float settle_us;
};

TransientStats AnalyseTransient(const float* x, std::size_t len, float sr) {
    TransientStats s{};
    s.peak = 0.0f;
    s.peak_idx = 0;
    for (std::size_t i = 0; i < len; ++i) {
        const float v = std::fabs(x[i]);
        if (v > s.peak) {
            s.peak = v;
            s.peak_idx = i;
        }
    }
    if (s.peak <= 0.0f) {
        s.rise_us = 0.0f;
        s.settle_us = 0.0f;
        return s;
    }
    const float p10 = 0.10f * s.peak;
    const float p90 = 0.90f * s.peak;
    const float p05 = 0.05f * s.peak;

    // Find earliest index where envelope crosses 10% (going up).
    std::size_t i10 = 0;
    bool found10 = false;
    for (std::size_t i = 0; i <= s.peak_idx; ++i) {
        if (std::fabs(x[i]) >= p10) { i10 = i; found10 = true; break; }
    }
    // Find earliest index where envelope crosses 90% (going up).
    std::size_t i90 = s.peak_idx;
    bool found90 = false;
    for (std::size_t i = found10 ? i10 : 0; i <= s.peak_idx; ++i) {
        if (std::fabs(x[i]) >= p90) { i90 = i; found90 = true; break; }
    }
    if (!found10 || !found90 || i90 < i10) {
        s.rise_us = 0.0f;
    } else {
        s.rise_us = static_cast<float>(i90 - i10) * 1e6f / sr;
    }

    // Settle: from peak_idx, find first index where |x| < 5% of peak AND stays
    // below 5% for the remainder of the buffer.
    std::size_t settle_idx = len - 1;
    for (std::size_t i = s.peak_idx; i < len; ++i) {
        if (std::fabs(x[i]) < p05) {
            bool stays_below = true;
            for (std::size_t j = i; j < len; ++j) {
                if (std::fabs(x[j]) >= p05) { stays_below = false; break; }
            }
            if (stays_below) { settle_idx = i; break; }
        }
    }
    s.settle_us = (settle_idx >= s.peak_idx)
        ? static_cast<float>(settle_idx - s.peak_idx) * 1e6f / sr
        : 0.0f;
    return s;
}

}  // namespace

// ===========================================================================
// T-THD-* (4 tests)
// ===========================================================================

TEST_CASE("T-THD-HiFi: THD+N < 0.1 %",
          "[spectral][thd][hifi]") {
    const float thd = MeasureDryPathThdPct(QualityMode::kHiFi, "hifi");
    INFO("T-THD-HiFi measured THD+N = " << thd << " %");
    REQUIRE(thd < 0.1f);
}

TEST_CASE("T-THD-Clouds: THD+N < 1 %",
          "[spectral][thd][clouds]") {
    // Clouds has dry-path passthrough (no quantization on dry), so we measure
    // both paths and report the worst case.  The wet path is where the 12-bit
    // ProcessOutput quantization shows up; that's the characteristic Clouds
    // distortion.
    float thd_dry = 0.0f;
    float thd_wet = 0.0f;
    SECTION("dry path") {
        thd_dry = MeasureDryPathThdPct(QualityMode::kClouds, "clouds");
        INFO("T-THD-Clouds dry THD+N = " << thd_dry << " %");
        REQUIRE(thd_dry < 1.0f);
    }
    SECTION("wet path") {
        thd_wet = MeasureWetPathThdPct(QualityMode::kClouds, "clouds");
        INFO("T-THD-Clouds wet THD+N = " << thd_wet << " %");
        REQUIRE(thd_wet < 1.0f);
    }
}

TEST_CASE("T-THD-LoFi: THD+N < 0.5 %",
          "[spectral][thd][lofi]") {
    const float thd = MeasureDryPathThdPct(QualityMode::kCleanLoFi, "lofi");
    INFO("T-THD-LoFi measured THD+N = " << thd << " %");
    REQUIRE(thd < 0.5f);
}

TEST_CASE("T-THD-Tape: THD+N < 5 %",
          "[spectral][thd][tape]") {
    const float thd = MeasureDryPathThdPct(QualityMode::kTape, "tape");
    INFO("T-THD-Tape measured THD+N = " << thd << " %");
    REQUIRE(thd < 5.0f);
}

// ===========================================================================
// T-NF-* (4 tests)
// ===========================================================================
//
// Methodology (corrected per spec leading comment): the spec's frozen-buffer
// recipe describes recording a sine into the buffer, freezing, then measuring
// — but with continuous grains, that captures grain-modulated sine, not a
// noise floor.  We instead run silence end-to-end through the wet path for 5 s
// (skipping the 1 s settle) and compute the residual RMS in dBFS.  This
// captures the engine's true noise floor — mu-law hiss in tape mode,
// quantization in Clouds, numerical / SVF residual elsewhere.
//

namespace {

float MeasureNoiseFloorDbfs(QualityMode mode, const std::string& mode_tag) {
    constexpr std::size_t kSettle = 48000;        // 1 s skip
    constexpr std::size_t kMeasure = 4 * 48000;   // 4 s measure
    constexpr std::size_t kTotal = kSettle + kMeasure;

    Proc proc;
    BeadsParameters params = MakeBaseParams(mode);
    params.dry_wet = 1.0f;
    params.density = 1.0f;       // continuous grain output
    params.size = 0.5f;
    params.shape = 0.5f;
    params.pitch = 0.0f;
    params.feedback = 0.0f;
    params.reverb = 0.0f;
    params.freeze = false;
    params.trigger_mode = TriggerMode::kLatched;
    proc.processor.SetParameters(params);

    std::vector<float> output(kTotal);
    RunSilence(proc.processor, output.data(), kTotal);

    WriteSamplesCsv(OutputPath("nf_" + mode_tag + ".csv"),
                    output.data(), kTotal);

    const float rms = Rms(output.data() + kSettle, kMeasure);
    const float dbfs = (rms > 1e-20f)
        ? 20.0f * std::log10(rms)
        : -200.0f;
    return dbfs;
}

}  // namespace

TEST_CASE("T-NF-HiFi: noise floor < -90 dBFS",
          "[spectral][noisefloor][hifi]") {
    const float dbfs = MeasureNoiseFloorDbfs(QualityMode::kHiFi, "hifi");
    INFO("T-NF-HiFi measured noise = " << dbfs << " dBFS");
    REQUIRE(dbfs < -90.0f);
}

TEST_CASE("T-NF-Clouds: noise floor < -65 dBFS",
          "[spectral][noisefloor][clouds]") {
    const float dbfs = MeasureNoiseFloorDbfs(QualityMode::kClouds, "clouds");
    INFO("T-NF-Clouds measured noise = " << dbfs << " dBFS");
    REQUIRE(dbfs < -65.0f);
}

TEST_CASE("T-NF-LoFi: noise floor < -85 dBFS",
          "[spectral][noisefloor][lofi]") {
    const float dbfs = MeasureNoiseFloorDbfs(QualityMode::kCleanLoFi, "lofi");
    INFO("T-NF-LoFi measured noise = " << dbfs << " dBFS");
    REQUIRE(dbfs < -85.0f);
}

TEST_CASE("T-NF-Tape: noise floor < -70 dBFS",
          "[spectral][noisefloor][tape]") {
    const float dbfs = MeasureNoiseFloorDbfs(QualityMode::kTape, "tape");
    INFO("T-NF-Tape measured noise = " << dbfs << " dBFS");
    REQUIRE(dbfs < -70.0f);
}

// ===========================================================================
// T-IR-* (4 tests) — frozen-buffer transient (spec §3.5)
// ===========================================================================
//
// The grain engine's effective IR is dominated by the Hann/Tukey grain envelope
// and the chosen `size` parameter.  These targets characterise the END-TO-END
// rise/fall behaviour including grain windowing.
//
// Spec note: the methodology section describes a "single-sample Dirac" input,
// but the peak thresholds (e.g. 0.5012 = -6 dB, 0.7079 = -3 dB) only make sense
// for a sustained input — a single-sample Dirac smeared by a ~10000-sample
// grain envelope produces a peak of ~1/10000, far below any of the thresholds.
// The spec considered DC vs sine vs Dirac and the peak thresholds clearly track
// "0.5 * mode_passthrough_gain" — i.e. DC = 0.5.  We fill with DC = 0.5 so the
// post-freeze readback is a sequence of grain envelopes scaled by 0.5, and the
// peak threshold checks "the engine preserves 0.5 with the expected per-mode
// passthrough gain".

namespace {

void RunIrTest(QualityMode mode,
               const std::string& mode_tag,
               float rise_us_max,
               float settle_us_max,
               float peak_min) {
    constexpr std::size_t kFill = 48000;        // 1 s buffer fill
    constexpr std::size_t kCapture = 4096;      // capture window after freeze

    Proc proc;
    BeadsParameters params = MakeBaseParams(mode);
    params.dry_wet = 1.0f;
    params.density = 1.0f;
    params.size = 0.5f;
    params.shape = 0.5f;
    params.pitch = 0.0f;
    params.feedback = 0.0f;
    params.reverb = 0.0f;
    params.freeze = false;
    params.trigger_mode = TriggerMode::kLatched;
    // Read close to the write head so grains pull from the freshly-recorded
    // DC during phase 1, not from the still-silent older portion of the
    // buffer (default `time=0.5` would land on uninitialised zeros).
    params.time = 0.0f;
    proc.processor.SetParameters(params);

    // Phase 1: drive 1 s of constant DC = 0.5 so every grain has signal to read
    // wherever it looks in the buffer.  The grain envelope shape then cleanly
    // shows up at the output.
    std::vector<float> input(kFill, 0.5f);
    std::vector<float> warmup_out(kFill);
    RunMono(proc.processor, input.data(), warmup_out.data(), kFill);

    // Phase 2: freeze and drive zeros while capturing.
    params.freeze = true;
    proc.processor.SetParameters(params);

    std::vector<float> capture(kCapture);
    RunSilence(proc.processor, capture.data(), kCapture);

    WriteSamplesCsv(OutputPath("tir_" + mode_tag + ".csv"),
                    capture.data(), kCapture);

    const TransientStats st = AnalyseTransient(capture.data(), kCapture, kSr);
    INFO("T-IR-" << mode_tag
         << " peak=" << st.peak
         << " rise_us=" << st.rise_us
         << " settle_us=" << st.settle_us);

    REQUIRE(st.peak >= peak_min);
    REQUIRE(st.rise_us < rise_us_max);
    REQUIRE(st.settle_us < settle_us_max);
}

}  // namespace

TEST_CASE("T-IR-HiFi: rise < 50 us, peak >= -3 dB",
          "[spectral][transient][hifi]") {
    RunIrTest(QualityMode::kHiFi, "hifi", 50.0f, 200.0f, 0.7079f);
}

TEST_CASE("T-IR-Clouds: rise < 100 us, peak >= -6 dB",
          "[spectral][transient][clouds]") {
    RunIrTest(QualityMode::kClouds, "clouds", 100.0f, 400.0f, 0.5012f);
}

TEST_CASE("T-IR-LoFi: rise < 400 us, peak >= -10 dB",
          "[spectral][transient][lofi]") {
    RunIrTest(QualityMode::kCleanLoFi, "lofi", 400.0f, 2000.0f, 0.3162f);
}

TEST_CASE("T-IR-Tape: rise < 200 us, peak >= -8 dB",
          "[spectral][transient][tape]") {
    RunIrTest(QualityMode::kTape, "tape", 200.0f, 1000.0f, 0.3981f);
}

// ===========================================================================
// T-MULAW-Round
// ===========================================================================

TEST_CASE("T-MULAW-Round: input ~ output through compress->expand",
          "[spectral][mulaw][tape]") {
    constexpr std::size_t kSettle = 4096;
    constexpr std::size_t kMeasure = 8192;
    constexpr std::size_t kTotal = kSettle + kMeasure;
    constexpr float kAmp = 0.5f;

    QualityProcessor q;
    q.Init(kSr);

    // Bin-align 1 kHz to the FFT length so the post-LP attenuation is sampled
    // cleanly.  (We don't FFT here, but the alignment also gives us a clean
    // peak.)
    const float f0 = NearestBinHz(1000.0f, kMeasure, kSr);

    std::vector<float> sine(kTotal);
    GenSine(sine.data(), kTotal, f0, kSr, kAmp);

    std::vector<float> roundtrip(kTotal);
    for (std::size_t i = 0; i < kTotal; ++i) {
        StereoFrame in = { sine[i], sine[i] };
        StereoFrame mid = q.ProcessInput(in, QualityMode::kTape);
        StereoFrame out = q.ProcessOutput(mid, QualityMode::kTape);
        roundtrip[i] = out.l;
    }

    WriteSamplesCsv(OutputPath("mulaw_round_input.csv"),
                    sine.data(), kTotal);
    WriteSamplesCsv(OutputPath("mulaw_round_output.csv"),
                    roundtrip.data(), kTotal);

    const float peak = PeakAbs(roundtrip.data() + kSettle, kMeasure);
    INFO("T-MULAW-Round peak = " << peak
         << " (target band " << 0.95f * kAmp << " .. " << 1.05f * kAmp << ")");

    REQUIRE(peak >= 0.95f * kAmp);
    REQUIRE(peak <= 1.05f * kAmp);
}
