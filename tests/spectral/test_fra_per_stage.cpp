// T-FRA-* per-stage frequency response tests for nosuch_texture DSP.
//
// These tests measure the magnitude response of individual LTI subsystems
// (QualityProcessor input/output stages, RecordingBuffer decimation, Reverb,
// and the StateVariableFilter HP). For each test, we either drive a Dirac
// through the subsystem and FFT the impulse response (`IrFra`), or use a
// stepped-sine sweep when the system has -infty dB at DC (`SteppedSineFra`).
//
// CSV outputs are written to BEADS_SPECTRAL_OUTPUT_DIR (set by CMake) for
// post-mortem plotting.
//
// References:
//   - Spec artifact §4.1 (test list) and §5 (numerical targets)
//   - tests/spectral/spectral_helpers.h (harness API)

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <limits>
#include <vector>

#include "spectral_helpers.h"

#include "beads/types.h"
#include "buffer/recording_buffer.h"
#include "fx/reverb.h"
#include "quality/quality_processor.h"
#include "util/svf.h"

using Catch::Approx;
using namespace beads;
using namespace beads_spec;

namespace {

// ----------------------------------------------------------------------------
// Helpers — test-local; intentionally NOT added to spectral_helpers.h to keep
// the public harness narrow.
// ----------------------------------------------------------------------------

// Maximum |db| over a sub-grid satisfying lo_hz <= hz <= hi_hz.
// Used for unity-gain reference checks (HiFi passthrough).
float MaxAbsDbInBand(const std::vector<FraPoint>& pts, float lo_hz, float hi_hz) {
    float worst = 0.0f;
    for (const auto& p : pts) {
        if (p.hz < lo_hz || p.hz > hi_hz) continue;
        const float a = std::fabs(p.db);
        if (a > worst) worst = a;
    }
    return worst;
}

// Deviation from band mean: max |db - mean_db| over [lo, hi]. Used for
// systems that may have non-unity passband gain (e.g. mu-law in Tape mode).
// "Flat ±X dB" in DSP typically refers to this measure.
float FlatnessDevInBand(const std::vector<FraPoint>& pts,
                        float lo_hz, float hi_hz) {
    double sum = 0.0;
    int count = 0;
    for (const auto& p : pts) {
        if (p.hz < lo_hz || p.hz > hi_hz) continue;
        sum += static_cast<double>(p.db);
        ++count;
    }
    if (count == 0) return 0.0f;
    const float mean = static_cast<float>(sum / count);
    float worst = 0.0f;
    for (const auto& p : pts) {
        if (p.hz < lo_hz || p.hz > hi_hz) continue;
        const float a = std::fabs(p.db - mean);
        if (a > worst) worst = a;
    }
    return worst;
}

// Find the first frequency >= lo_hz where the response crosses below
// `threshold_db` (absolute dB value, e.g. -3 dB for a unity-gain system).
// Returns -1 if no such crossing exists in the grid.
float FindCornerHz(const std::vector<FraPoint>& pts, float threshold_db,
                   float lo_hz = 0.0f) {
    float prev_hz = -1.0f;
    float prev_db = 0.0f;
    bool first = true;
    for (const auto& p : pts) {
        if (p.hz < lo_hz) {
            prev_hz = p.hz;
            prev_db = p.db;
            first = false;
            continue;
        }
        if (!first && prev_db >= threshold_db && p.db <= threshold_db) {
            // Linear interpolation in log-frequency / linear-dB.
            const float lp = std::log10(prev_hz);
            const float lh = std::log10(p.hz);
            const float t = (threshold_db - prev_db) / (p.db - prev_db);
            return std::pow(10.0f, lp + t * (lh - lp));
        }
        prev_hz = p.hz;
        prev_db = p.db;
        first = false;
    }
    return -1.0f;
}

// Find the first frequency >= lo_hz where the response drops by `drop_db`
// (e.g. 3 dB) below the band-mean of the passband [pass_lo, pass_hi].
// Useful for systems with non-unity passband gain.
float FindRelativeCornerHz(const std::vector<FraPoint>& pts,
                           float drop_db,
                           float pass_lo, float pass_hi) {
    double sum = 0.0;
    int count = 0;
    for (const auto& p : pts) {
        if (p.hz < pass_lo || p.hz > pass_hi) continue;
        sum += static_cast<double>(p.db);
        ++count;
    }
    if (count == 0) return -1.0f;
    const float ref = static_cast<float>(sum / count);
    return FindCornerHz(pts, ref - drop_db, pass_lo);
}

// Smooth a (hz, db) FRA curve by averaging over a sliding window of width
// `window` points (centered). Useful for reverb-style responses dominated by
// comb-filter ripple where the underlying trend is what we want to measure.
std::vector<FraPoint> SmoothFra(const std::vector<FraPoint>& pts, int window) {
    if (window < 1) window = 1;
    const int half = window / 2;
    std::vector<FraPoint> out;
    out.reserve(pts.size());
    for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
        double sum = 0.0;
        int count = 0;
        for (int k = std::max(0, i - half);
             k <= std::min(static_cast<int>(pts.size()) - 1, i + half);
             ++k) {
            sum += static_cast<double>(pts[k].db);
            ++count;
        }
        out.push_back({pts[i].hz,
                       static_cast<float>(sum / std::max(1, count))});
    }
    return out;
}

// dB at the grid point closest to `target_hz`.
float DbAtHz(const std::vector<FraPoint>& pts, float target_hz) {
    float best_db = 0.0f;
    float best_dist = std::numeric_limits<float>::infinity();
    for (const auto& p : pts) {
        const float d = std::fabs(std::log10(p.hz) - std::log10(target_hz));
        if (d < best_dist) {
            best_dist = d;
            best_db = p.db;
        }
    }
    return best_db;
}

// Run `n_zero_frames` of zero input through `qp.ProcessInput(_, mode)` to
// flush the mode-change crossfade (kModeXfadeSamples = 64) and let the SVF
// reach steady state.
void PreRollProcessInput(QualityProcessor& qp, QualityMode mode,
                         int n_zero_frames = 256) {
    StereoFrame z = {0.0f, 0.0f};
    for (int i = 0; i < n_zero_frames; ++i) {
        (void)qp.ProcessInput(z, mode);
    }
}

void PreRollProcessOutput(QualityProcessor& qp, QualityMode mode,
                          int n_zero_frames = 256) {
    StereoFrame z = {0.0f, 0.0f};
    for (int i = 0; i < n_zero_frames; ++i) {
        (void)qp.ProcessOutput(z, mode);
    }
}

} // namespace

// ============================================================================
// QualityProcessor::ProcessInput — per-mode FRA
// ============================================================================

TEST_CASE("T-FRA-QPInput-HiFi: passband flat ±0.05 dB", "[fra][qp][input][hifi]") {
    QualityProcessor qp;
    qp.Init(kSampleRate);
    PreRollProcessInput(qp, QualityMode::kHiFi);

    auto grid = StandardFraGrid();
    auto pts = IrFra([&](float x) {
        StereoFrame in = {x, x};
        StereoFrame out = qp.ProcessInput(in, QualityMode::kHiFi);
        return out.l;
    }, grid, /*n=*/8192, kSampleRate, /*impulse_amp=*/1.0f);
    WriteFraCsv(OutputPath("fra_qpinput_hifi.csv"), pts);

    REQUIRE(MaxAbsDbInBand(pts, 20.0f, 22050.0f) <= 0.05f);
}

TEST_CASE("T-FRA-QPInput-Clouds: -3 dB at 9.5–10.5 kHz", "[fra][qp][input][clouds]") {
    QualityProcessor qp;
    qp.Init(kSampleRate);
    PreRollProcessInput(qp, QualityMode::kClouds);

    auto grid = StandardFraGrid();
    auto pts = IrFra([&](float x) {
        StereoFrame in = {x, x};
        StereoFrame out = qp.ProcessInput(in, QualityMode::kClouds);
        return out.l;
    }, grid, /*n=*/8192, kSampleRate, /*impulse_amp=*/1.0f);
    WriteFraCsv(OutputPath("fra_qpinput_clouds.csv"), pts);

    // Passband flat ±1.5 dB up to 8 kHz
    REQUIRE(MaxAbsDbInBand(pts, 20.0f, 8000.0f) <= 1.5f);
    // -3 dB corner ∈ [9.5k, 10.5k]
    const float corner = FindCornerHz(pts, -3.0f);
    REQUIRE(corner >= 9500.0f);
    REQUIRE(corner <= 10500.0f);
}

TEST_CASE("T-FRA-QPInput-LoFi: -3 dB at 2.4–2.6 kHz", "[fra][qp][input][lofi]") {
    QualityProcessor qp;
    qp.Init(kSampleRate);
    PreRollProcessInput(qp, QualityMode::kCleanLoFi);

    auto grid = StandardFraGrid();
    auto pts = IrFra([&](float x) {
        StereoFrame in = {x, x};
        StereoFrame out = qp.ProcessInput(in, QualityMode::kCleanLoFi);
        return out.l;
    }, grid, /*n=*/8192, kSampleRate, /*impulse_amp=*/1.0f);
    WriteFraCsv(OutputPath("fra_qpinput_lofi.csv"), pts);

    // Passband flat ±1.0 dB to 2 kHz
    REQUIRE(MaxAbsDbInBand(pts, 20.0f, 2000.0f) <= 1.0f);
    // -3 dB corner ∈ [2.4k, 2.6k]
    const float corner = FindCornerHz(pts, -3.0f);
    REQUIRE(corner >= 2400.0f);
    REQUIRE(corner <= 2600.0f);
    // -40 dB by 6 kHz (a 2nd-order SVF may not reach this — implement as written)
    REQUIRE(DbAtHz(pts, 6000.0f) <= -40.0f);
}

TEST_CASE("T-FRA-QPInput-Tape: -3 dB at 4.8–5.2 kHz", "[fra][qp][input][tape]") {
    QualityProcessor qp;
    qp.Init(kSampleRate);
    PreRollProcessInput(qp, QualityMode::kTape);

    auto grid = StandardFraGrid();
    // Tape mode goes through mu-law compression — keep impulse small so we
    // stay near the linear region of the curve. Mu-law has non-unity DC
    // gain (slope ~15.3x at zero for mu=64), so flatness and the -3 dB
    // corner are measured RELATIVE to the passband mean.
    auto pts = IrFra([&](float x) {
        StereoFrame in = {x, x};
        StereoFrame out = qp.ProcessInput(in, QualityMode::kTape);
        return out.l;
    }, grid, /*n=*/8192, kSampleRate, /*impulse_amp=*/0.05f);
    WriteFraCsv(OutputPath("fra_qpinput_tape.csv"), pts);

    // Passband flat ±2.0 dB 100 Hz–4 kHz (deviation from band mean)
    REQUIRE(FlatnessDevInBand(pts, 100.0f, 4000.0f) <= 2.0f);
    // -3 dB corner ∈ [4.8k, 5.2k] (3 dB below passband mean)
    const float corner = FindRelativeCornerHz(pts, 3.0f, 100.0f, 4000.0f);
    REQUIRE(corner >= 4800.0f);
    REQUIRE(corner <= 5200.0f);
}

// ============================================================================
// QualityProcessor::ProcessOutput — per-mode FRA
// ============================================================================

TEST_CASE("T-FRA-QPOutput-HiFi: passband flat ±0.05 dB", "[fra][qp][output][hifi]") {
    QualityProcessor qp;
    qp.Init(kSampleRate);
    PreRollProcessOutput(qp, QualityMode::kHiFi);

    auto grid = StandardFraGrid();
    auto pts = IrFra([&](float x) {
        StereoFrame in = {x, x};
        StereoFrame out = qp.ProcessOutput(in, QualityMode::kHiFi);
        return out.l;
    }, grid, /*n=*/8192, kSampleRate, /*impulse_amp=*/1.0f);
    WriteFraCsv(OutputPath("fra_qpoutput_hifi.csv"), pts);

    REQUIRE(MaxAbsDbInBand(pts, 20.0f, 22050.0f) <= 0.05f);
}

TEST_CASE("T-FRA-QPOutput-Clouds: passthrough except quantization",
          "[fra][qp][output][clouds]") {
    // Clouds output applies 12-bit quantization but no LP; with a 0.5
    // amplitude impulse the quantization step (~4.88e-4) is tiny relative to
    // the impulse, so the magnitude response is essentially flat.
    QualityProcessor qp;
    qp.Init(kSampleRate);
    PreRollProcessOutput(qp, QualityMode::kClouds);

    auto grid = StandardFraGrid();
    auto pts = IrFra([&](float x) {
        StereoFrame in = {x, x};
        StereoFrame out = qp.ProcessOutput(in, QualityMode::kClouds);
        return out.l;
    }, grid, /*n=*/8192, kSampleRate, /*impulse_amp=*/0.5f);
    WriteFraCsv(OutputPath("fra_qpoutput_clouds.csv"), pts);

    REQUIRE(MaxAbsDbInBand(pts, 20.0f, 22050.0f) <= 0.5f);
}

TEST_CASE("T-FRA-QPOutput-LoFi: -3 dB at ~10 kHz", "[fra][qp][output][lofi]") {
    QualityProcessor qp;
    qp.Init(kSampleRate);
    PreRollProcessOutput(qp, QualityMode::kCleanLoFi);

    auto grid = StandardFraGrid();
    auto pts = IrFra([&](float x) {
        StereoFrame in = {x, x};
        StereoFrame out = qp.ProcessOutput(in, QualityMode::kCleanLoFi);
        return out.l;
    }, grid, /*n=*/8192, kSampleRate, /*impulse_amp=*/1.0f);
    WriteFraCsv(OutputPath("fra_qpoutput_lofi.csv"), pts);

    // Passband flat ±1.0 dB to 8 kHz
    REQUIRE(MaxAbsDbInBand(pts, 20.0f, 8000.0f) <= 1.0f);
    // -3 dB corner ∈ [9.5k, 10.5k]
    const float corner = FindCornerHz(pts, -3.0f);
    REQUIRE(corner >= 9500.0f);
    REQUIRE(corner <= 10500.0f);
}

TEST_CASE("T-FRA-QPOutput-Tape: -3 dB at ~5 kHz", "[fra][qp][output][tape]") {
    QualityProcessor qp;
    qp.Init(kSampleRate);
    PreRollProcessOutput(qp, QualityMode::kTape);

    auto grid = StandardFraGrid();
    // ProcessOutput's Tape branch applies LP-then-mu-law-expand. Mu-law
    // expand also has non-unity DC slope, so we measure flatness and the
    // -3 dB corner RELATIVE to the passband mean.
    auto pts = IrFra([&](float x) {
        StereoFrame in = {x, x};
        StereoFrame out = qp.ProcessOutput(in, QualityMode::kTape);
        return out.l;
    }, grid, /*n=*/8192, kSampleRate, /*impulse_amp=*/0.05f);
    WriteFraCsv(OutputPath("fra_qpoutput_tape.csv"), pts);

    // Passband flat ±2.0 dB 100 Hz–4 kHz (deviation from band mean)
    REQUIRE(FlatnessDevInBand(pts, 100.0f, 4000.0f) <= 2.0f);
    // -3 dB corner ∈ [4.8k, 5.2k] (3 dB below passband mean)
    const float corner = FindRelativeCornerHz(pts, 3.0f, 100.0f, 4000.0f);
    REQUIRE(corner >= 4800.0f);
    REQUIRE(corner <= 5200.0f);
}

// ============================================================================
// RecordingBuffer decimation FRA
//
// We isolate the decimation behavior by writing a unit Dirac into the buffer,
// then writing zeros for `n-1` more frames. Because decimation is sample-and-
// hold (only every D-th write is kept), the captured response represents the
// ZOH frequency response. Reading back via `ReadHermite(channel, float(i))`
// at integer positions gives us the buffer contents directly.
// ============================================================================

namespace {

std::vector<FraPoint> MeasureBufferDecFra(int decimation, std::size_t n) {
    // Allocate a fresh buffer for each test. Write a Dirac at sample 0 then
    // zeros, per the brief's methodology. Read back via ReadHermite at
    // positions 0..n-1 (where each position is a fractional-domain index
    // tracking input time, advancing by 1/decimation per output frame).
    //
    // This models the typical playback path: the decimator stores 1 sample
    // per D inputs, and grain playback reads back at the full output rate
    // via Hermite interpolation. The resulting response is the combined
    // decimation+upsampling-by-Hermite-interp characteristic.
    const std::size_t frames = n + static_cast<std::size_t>(kInterpolationTail) + 16;
    std::vector<float> mem(frames * 2 + kInterpolationTail * 2, 0.0f);

    RecordingBuffer rb;
    rb.Init(mem.data(), frames, /*num_channels=*/2);
    rb.SetDecimationFactor(decimation);

    // The decimation_counter starts at 0 and pre-increments on each Write.
    // For D=1, the first Write stores. For D>=2, the first D-1 writes are
    // dropped before the first Write is kept. To make sure the impulse is
    // captured at buffer position 0, we drive D-1 leading zeros (dropped),
    // then the impulse (stored), then zeros.
    const int leading_zeros = decimation - 1;
    for (int i = 0; i < leading_zeros; ++i) {
        rb.Write(0.0f, 0.0f);
    }
    rb.Write(1.0f, 1.0f);
    const std::size_t total_writes = n * static_cast<std::size_t>(decimation);
    for (std::size_t i = static_cast<std::size_t>(leading_zeros) + 1;
         i < total_writes; ++i) {
        rb.Write(0.0f, 0.0f);
    }

    // Read back at fractional positions advancing by 1/D per output sample.
    std::vector<float> ir(n, 0.0f);
    for (std::size_t i = 0; i < n; ++i) {
        const float pos = static_cast<float>(i) /
                          static_cast<float>(decimation);
        ir[i] = rb.ReadHermite(0, pos);
    }

    std::vector<std::complex<float>> spec(n);
    Fft(ir.data(), n, spec.data());

    std::vector<FraPoint> out;
    auto grid = StandardFraGrid();
    out.reserve(grid.size());
    // Normalize so unity-gain (D=1) yields 0 dB at DC.
    const float dc_mag = MagAtHz(spec.data(), n, kSampleRate, 0.0f);
    const float ref = (dc_mag > 1e-10f) ? dc_mag : 1.0f;
    for (float f : grid) {
        const float mag = MagAtHz(spec.data(), n, kSampleRate, f) / ref;
        const float db = (mag > 1e-10f) ? 20.0f * std::log10(mag) : -200.0f;
        out.push_back({f, db});
    }
    return out;
}

}  // namespace

TEST_CASE("T-FRA-Buffer-Dec1: flat", "[fra][buffer][dec1]") {
    auto pts = MeasureBufferDecFra(1, 8192);
    WriteFraCsv(OutputPath("fra_buffer_dec1.csv"), pts);
    REQUIRE(MaxAbsDbInBand(pts, 20.0f, 22050.0f) <= 0.1f);
}

TEST_CASE("T-FRA-Buffer-Dec2: ZOH droop -0.91 dB at 12 kHz", "[fra][buffer][dec2]") {
    auto pts = MeasureBufferDecFra(2, 8192);
    WriteFraCsv(OutputPath("fra_buffer_dec2.csv"), pts);
    // ZOH at decimation D=2 has |sinc(pi f / Fs * D)| droop.
    // At 12 kHz of 48 kHz, the argument is pi/2 * 0.5 ≈ 0.785;
    // sin(0.785)/0.785 ≈ 0.9 ≈ -0.91 dB.
    REQUIRE(DbAtHz(pts, 12000.0f) == Approx(-0.91f).margin(0.5f));
}

TEST_CASE("T-FRA-Buffer-Dec4: ZOH droop -3.92 dB at 6 kHz", "[fra][buffer][dec4]") {
    auto pts = MeasureBufferDecFra(4, 8192);
    WriteFraCsv(OutputPath("fra_buffer_dec4.csv"), pts);
    // At 6 kHz of 48 kHz with D=4, sample-and-hold first null is at Fs/D = 12 kHz;
    // 6 kHz is half-Nyquist of the new band → 2/π ≈ 0.637 ≈ -3.92 dB.
    REQUIRE(DbAtHz(pts, 6000.0f) == Approx(-3.92f).margin(0.5f));
}

TEST_CASE("T-FRA-Buffer-Dec8: ZOH droop -3.92 dB at 3 kHz", "[fra][buffer][dec8]") {
    auto pts = MeasureBufferDecFra(8, 8192);
    WriteFraCsv(OutputPath("fra_buffer_dec8.csv"), pts);
    REQUIRE(DbAtHz(pts, 3000.0f) == Approx(-3.92f).margin(0.5f));
}

// ============================================================================
// Reverb FRA — measure how the reverb LP setting attenuates the magnitude
// response of the wet signal.
// ============================================================================

namespace {

std::vector<FraPoint> MeasureReverbFra(float lp, std::size_t n = 32768) {
    std::vector<float> mem(kReverbBufferSize, 0.0f);
    Reverb rev;
    rev.Init(mem.data(), kReverbBufferSize, kSampleRate);
    rev.SetAmount(1.0f);
    rev.SetDecay(0.5f);
    rev.SetDiffusion(0.7f);
    rev.SetLpCutoff(lp);

    auto grid = StandardFraGrid();
    return IrFra([&](float x) {
        float ol, oR;
        rev.Process(x, x, &ol, &oR);
        return ol;
    }, grid, n, kSampleRate, /*impulse_amp=*/1.0f);
}

}  // namespace

TEST_CASE("T-FRA-Reverb-HiFi: -3 dB ≥ 6 kHz", "[fra][reverb][hifi]") {
    auto pts_raw = MeasureReverbFra(0.7f);
    WriteFraCsv(OutputPath("fra_reverb_hifi.csv"), pts_raw);

    // Reverb's magnitude response is comb-filter-modulated (lots of ripple
    // from the feedback delay lines). Smooth in log-frequency to reveal the
    // underlying low-pass character imposed by the LP in the feedback path.
    auto pts = SmoothFra(pts_raw, /*window=*/9);
    const float corner =
        FindRelativeCornerHz(pts, 3.0f, /*pass_lo=*/100.0f, /*pass_hi=*/1000.0f);
    REQUIRE(corner >= 6000.0f);
    REQUIRE(corner <= 16000.0f);
}

// Helper: smoothed reverb -3 dB corner relative to passband mean (100-1000 Hz).
// Reverbs have noisy comb-filter responses; smoothing reveals trend.
namespace {
float ReverbCornerHz(const std::vector<FraPoint>& pts) {
    auto sm = SmoothFra(pts, /*window=*/9);
    return FindRelativeCornerHz(sm, 3.0f, 100.0f, 1000.0f);
}
}  // namespace

TEST_CASE("T-FRA-Reverb-Clouds: warmer than HiFi", "[fra][reverb][clouds]") {
    auto pts_hifi = MeasureReverbFra(0.7f);
    auto pts = MeasureReverbFra(0.6f);
    WriteFraCsv(OutputPath("fra_reverb_clouds.csv"), pts);

    const float corner_hifi = ReverbCornerHz(pts_hifi);
    const float corner_clouds = ReverbCornerHz(pts);
    REQUIRE(corner_clouds > 0.0f);
    REQUIRE(corner_hifi > 0.0f);
    REQUIRE(corner_clouds < corner_hifi);
}

TEST_CASE("T-FRA-Reverb-LoFi: warmer still", "[fra][reverb][lofi]") {
    auto pts_clouds = MeasureReverbFra(0.6f);
    auto pts = MeasureReverbFra(0.5f);
    WriteFraCsv(OutputPath("fra_reverb_lofi.csv"), pts);

    const float corner_clouds = ReverbCornerHz(pts_clouds);
    const float corner_lofi = ReverbCornerHz(pts);
    REQUIRE(corner_lofi > 0.0f);
    REQUIRE(corner_clouds > 0.0f);
    REQUIRE(corner_lofi < corner_clouds);
}

TEST_CASE("T-FRA-Reverb-Tape: warmest", "[fra][reverb][tape]") {
    auto pts_lofi = MeasureReverbFra(0.5f);
    auto pts = MeasureReverbFra(0.3f);
    WriteFraCsv(OutputPath("fra_reverb_tape.csv"), pts);

    const float corner_lofi = ReverbCornerHz(pts_lofi);
    const float corner_tape = ReverbCornerHz(pts);
    REQUIRE(corner_tape > 0.0f);
    REQUIRE(corner_lofi > 0.0f);
    REQUIRE(corner_tape < corner_lofi);
}

// ============================================================================
// Feedback HP filter (StateVariableFilter HP @ 30 Hz, Q=0.707)
// IrFra is unsuitable because |H(f=0)| = -inf dB; use SteppedSineFra instead.
// ============================================================================

TEST_CASE("T-FRA-FbHP: 30 Hz HP flat above 100 Hz", "[fra][svf][hp]") {
    StateVariableFilter hp;
    hp.Init();
    hp.SetFrequencyHz(30.0f, kSampleRate);
    hp.SetQ(0.707f);

    // Coarse log-spaced grid: 30 points spanning 10 Hz to 22 kHz.
    std::vector<float> grid;
    constexpr int N = 30;
    const float lo = 10.0f, hi = 22050.0f;
    const float lr = std::log10(hi / lo) / static_cast<float>(N - 1);
    for (int i = 0; i < N; ++i) {
        grid.push_back(lo * std::pow(10.0f, lr * i));
    }

    auto pts = SteppedSineFra(
        [&](float x) { return hp.ProcessHP(x); },
        grid,
        kSampleRate,
        /*settle_seconds=*/0.2f,
        /*measure_seconds=*/0.3f,
        /*input_amp=*/0.25f);
    WriteFraCsv(OutputPath("fra_fbhp.csv"), pts);

    // Above 100 Hz: ±0.5 dB
    REQUIRE(MaxAbsDbInBand(pts, 100.0f, 22050.0f) <= 0.5f);
    // At 10 Hz: at least -10 dB
    REQUIRE(DbAtHz(pts, 10.0f) <= -10.0f);
}
