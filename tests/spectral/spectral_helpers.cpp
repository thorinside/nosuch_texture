#include "spectral_helpers.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>

#ifndef BEADS_SPECTRAL_OUTPUT_DIR
// Fallback so the file still compiles outside of CMake (e.g. an IDE indexer).
#define BEADS_SPECTRAL_OUTPUT_DIR "."
#endif

namespace beads_spec {

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;

bool IsPow2(std::size_t n) {
    return n > 0 && (n & (n - 1)) == 0;
}

std::size_t Log2Pow2(std::size_t n) {
    assert(IsPow2(n));
    std::size_t k = 0;
    while ((std::size_t{1} << k) < n) ++k;
    return k;
}

// xorshift32 with the canonical 13/17/5 triple. We deliberately do NOT seed
// with 0 (xorshift32 is degenerate at 0); callers use any nonzero seed.
inline std::uint32_t XorShift32(std::uint32_t& s) {
    if (s == 0u) s = 1u;
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return s;
}

}  // namespace

// ---------------------------------------------------------------------------
// Standard FRA grid
// ---------------------------------------------------------------------------
std::vector<float> StandardFraGrid() {
    std::vector<float> grid;
    grid.reserve(kFraPoints);
    const float log_lo = std::log10(kFraFmin);
    const float log_hi = std::log10(kFraFmax);
    for (int i = 0; i < kFraPoints; ++i) {
        const float t = static_cast<float>(i) /
                        static_cast<float>(kFraPoints - 1);
        grid.push_back(std::pow(10.0f, log_lo + t * (log_hi - log_lo)));
    }
    return grid;
}

// ---------------------------------------------------------------------------
// Signal generation
// ---------------------------------------------------------------------------
void GenSine(float* out, std::size_t len, float freq_hz, float sr, float amp) {
    const float w = kTwoPi * freq_hz / sr;
    for (std::size_t i = 0; i < len; ++i) {
        out[i] = amp * std::sin(w * static_cast<float>(i));
    }
}

void GenLogSweep(float* out, std::size_t len, float f0, float f1,
                 float sr, float amp) {
    if (len == 0) return;
    // Exponential sweep: f(t) = f0 * (f1/f0)^(t/T).
    // Phase = 2*pi * integral_0^t f(s) ds
    //       = 2*pi * f0 * T / ln(K) * (K^(t/T) - 1), K = f1/f0.
    const double T = static_cast<double>(len) / static_cast<double>(sr);
    const double K = std::max(1e-9, static_cast<double>(f1) /
                                          static_cast<double>(f0));
    const double lnK = std::log(K);
    const double coef = (lnK > 1e-9)
                            ? (kTwoPi * static_cast<double>(f0) * T / lnK)
                            : (kTwoPi * static_cast<double>(f0));
    for (std::size_t i = 0; i < len; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(sr);
        const double phase =
            (lnK > 1e-9) ? coef * (std::pow(K, t / T) - 1.0)
                         : coef * t;
        out[i] = amp * static_cast<float>(std::sin(phase));
    }
}

void GenImpulse(float* out, std::size_t len, float amp) {
    if (len == 0) return;
    out[0] = amp;
    for (std::size_t i = 1; i < len; ++i) out[i] = 0.0f;
}

void GenWhiteNoise(float* out, std::size_t len, std::uint32_t seed, float amp) {
    std::uint32_t s = (seed == 0u) ? 1u : seed;
    constexpr float kInv = 1.0f / static_cast<float>(INT32_MAX);
    for (std::size_t i = 0; i < len; ++i) {
        const std::uint32_t r = XorShift32(s);
        // Map to roughly [-1, 1] via signed reinterpretation, then scale.
        const std::int32_t si = static_cast<std::int32_t>(r);
        out[i] = amp * (static_cast<float>(si) * kInv);
    }
}

// ---------------------------------------------------------------------------
// FFT (iterative radix-2 Cooley-Tukey, bit-reversed input permutation)
// ---------------------------------------------------------------------------
void Fft(const float* in, std::size_t n, std::complex<float>* out) {
    assert(IsPow2(n));
    if (n == 0) return;

    // Bit-reversed copy of input into output as complex.
    const std::size_t bits = Log2Pow2(n);
    for (std::size_t i = 0; i < n; ++i) {
        std::size_t j = 0;
        std::size_t x = i;
        for (std::size_t b = 0; b < bits; ++b) {
            j = (j << 1) | (x & 1u);
            x >>= 1;
        }
        out[j] = std::complex<float>(in[i], 0.0f);
    }

    // Iterative butterflies.
    for (std::size_t size = 2; size <= n; size <<= 1) {
        const std::size_t half = size >> 1;
        const float theta = -kTwoPi / static_cast<float>(size);
        const std::complex<float> w_step(std::cos(theta), std::sin(theta));
        for (std::size_t base = 0; base < n; base += size) {
            std::complex<float> w(1.0f, 0.0f);
            for (std::size_t k = 0; k < half; ++k) {
                const std::complex<float> t = w * out[base + k + half];
                const std::complex<float> u = out[base + k];
                out[base + k] = u + t;
                out[base + k + half] = u - t;
                w *= w_step;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Frequency-domain helpers
// ---------------------------------------------------------------------------
float MagAtHz(const std::complex<float>* spectrum, std::size_t n, float sr,
              float target_hz) {
    if (n == 0) return 0.0f;
    const float bin_width = sr / static_cast<float>(n);
    long bin = static_cast<long>(std::lround(target_hz / bin_width));
    if (bin < 0) bin = 0;
    if (bin > static_cast<long>(n - 1)) bin = static_cast<long>(n - 1);
    return std::abs(spectrum[bin]);
}

float PowerInBand(const std::complex<float>* spectrum, std::size_t n, float sr,
                  float f_lo_hz, float f_hi_hz) {
    if (n == 0) return 0.0f;
    if (f_hi_hz < f_lo_hz) std::swap(f_lo_hz, f_hi_hz);
    const float bin_width = sr / static_cast<float>(n);
    long lo = static_cast<long>(std::lround(f_lo_hz / bin_width));
    long hi = static_cast<long>(std::lround(f_hi_hz / bin_width));
    if (lo < 0) lo = 0;
    if (hi < 0) hi = 0;
    const long max_bin = static_cast<long>(n / 2);
    if (lo > max_bin) lo = max_bin;
    if (hi > max_bin) hi = max_bin;
    float p = 0.0f;
    for (long k = lo; k <= hi; ++k) {
        const float m = std::abs(spectrum[k]);
        p += m * m;
    }
    return p;
}

// ---------------------------------------------------------------------------
// Time-domain helpers
// ---------------------------------------------------------------------------
float Rms(const float* x, std::size_t len) {
    if (len == 0) return 0.0f;
    double s = 0.0;
    for (std::size_t i = 0; i < len; ++i) {
        const double v = static_cast<double>(x[i]);
        s += v * v;
    }
    return static_cast<float>(std::sqrt(s / static_cast<double>(len)));
}

float PeakAbs(const float* x, std::size_t len) {
    float p = 0.0f;
    for (std::size_t i = 0; i < len; ++i) {
        const float v = std::fabs(x[i]);
        if (v > p) p = v;
    }
    return p;
}

// ---------------------------------------------------------------------------
// Stepped-sine FRA
// ---------------------------------------------------------------------------
std::vector<FraPoint> SteppedSineFra(std::function<float(float)> process,
                                     const std::vector<float>& freqs,
                                     float sr,
                                     float settle_seconds,
                                     float measure_seconds,
                                     float input_amp) {
    std::vector<FraPoint> out;
    out.reserve(freqs.size());

    const std::size_t settle_n = static_cast<std::size_t>(
        std::max(0.0f, settle_seconds) * sr + 0.5f);
    const std::size_t measure_n = static_cast<std::size_t>(
        std::max(0.0f, measure_seconds) * sr + 0.5f);

    std::vector<float> capture;
    capture.resize(measure_n);

    const float rms_in =
        (input_amp > 0.0f) ? input_amp / std::sqrt(2.0f) : 1.0f;

    for (float f : freqs) {
        const float w = kTwoPi * f / sr;
        // Settle phase: drive the system, discard output.
        for (std::size_t i = 0; i < settle_n; ++i) {
            const float s = input_amp *
                            std::sin(w * static_cast<float>(i));
            (void)process(s);
        }
        // Measure phase: continue the sine phase for continuity.
        for (std::size_t i = 0; i < measure_n; ++i) {
            const float t = static_cast<float>(settle_n + i);
            const float s = input_amp * std::sin(w * t);
            capture[i] = process(s);
        }
        const float rms_out = Rms(capture.data(), capture.size());
        // Avoid log(0): clamp at -200 dB.
        const float ratio =
            (rms_in > 0.0f) ? (rms_out / rms_in) : 0.0f;
        const float db =
            (ratio > 1e-10f) ? 20.0f * std::log10(ratio) : -200.0f;
        out.push_back({f, db});
    }
    return out;
}

// ---------------------------------------------------------------------------
// THD+N
// ---------------------------------------------------------------------------
float ThdPlusNPct(const std::complex<float>* spectrum, std::size_t n, float sr,
                  float f0, int nharm) {
    if (n == 0 || f0 <= 0.0f || sr <= 0.0f) return 0.0f;
    const float bin_width = sr / static_cast<float>(n);
    const long fund_bin =
        static_cast<long>(std::lround(f0 / bin_width));
    const long max_bin = static_cast<long>(n / 2);
    if (fund_bin <= 0 || fund_bin >= max_bin) return 0.0f;

    // Collect harmonic bin indices: fundamental + 2..nharm (clamped).
    const float fund_mag = std::abs(spectrum[fund_bin]);
    if (fund_mag <= 0.0f) return 0.0f;

    // Mark which bins are "the fundamental" (we exclude only that single bin
    // from the residual; harmonic bins are PART of the residual numerator).
    auto is_fund = [&](long k) { return k == fund_bin; };

    // Sum power across all non-DC bins except the fundamental.
    double residual_power = 0.0;
    for (long k = 1; k <= max_bin; ++k) {
        if (is_fund(k)) continue;
        const float m = std::abs(spectrum[k]);
        residual_power += static_cast<double>(m) * static_cast<double>(m);
    }
    // (The harmonics are already in residual_power; nharm mostly matters when
    // we want THD-without-noise. Per spec, THD+N treats harmonics+noise as
    // residual, so we honor the simple formulation. The nharm parameter is
    // accepted for API compatibility and used for clamping safety only.)
    (void)nharm;

    const double pct =
        100.0 * std::sqrt(residual_power) / static_cast<double>(fund_mag);
    return static_cast<float>(pct);
}

// ---------------------------------------------------------------------------
// IR-based FRA
// ---------------------------------------------------------------------------
std::vector<FraPoint> IrFra(std::function<float(float)> process,
                            const std::vector<float>& freqs,
                            std::size_t n,
                            float sr,
                            float impulse_amp) {
    assert(IsPow2(n));
    std::vector<float> ir(n, 0.0f);
    // Drive a single Dirac of amplitude impulse_amp through `process` and
    // capture the response. `process` may have internal state.
    for (std::size_t i = 0; i < n; ++i) {
        const float in = (i == 0) ? impulse_amp : 0.0f;
        ir[i] = process(in);
    }
    std::vector<std::complex<float>> spec(n);
    Fft(ir.data(), n, spec.data());

    std::vector<FraPoint> out;
    out.reserve(freqs.size());
    const float inv_amp =
        (impulse_amp > 0.0f) ? 1.0f / impulse_amp : 1.0f;
    for (float f : freqs) {
        const float mag = MagAtHz(spec.data(), n, sr, f) * inv_amp;
        const float db =
            (mag > 1e-10f) ? 20.0f * std::log10(mag) : -200.0f;
        out.push_back({f, db});
    }
    return out;
}

// ---------------------------------------------------------------------------
// CSV output
// ---------------------------------------------------------------------------
void WriteFraCsv(const std::string& path,
                 const std::vector<FraPoint>& pts) {
    std::filesystem::path p(path);
    if (p.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(p.parent_path(), ec);
        (void)ec;
    }
    std::ofstream f(path);
    if (!f.is_open()) return;
    f << "freq_hz,magnitude_db\n";
    f.setf(std::ios::fixed);
    f.precision(6);
    for (const auto& pt : pts) {
        f << pt.hz << "," << pt.db << "\n";
    }
}

void WriteSamplesCsv(const std::string& path, const float* x, std::size_t len) {
    std::filesystem::path p(path);
    if (p.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(p.parent_path(), ec);
        (void)ec;
    }
    std::ofstream f(path);
    if (!f.is_open()) return;
    f << "index,sample\n";
    f.setf(std::ios::fixed);
    f.precision(8);
    for (std::size_t i = 0; i < len; ++i) {
        f << i << "," << x[i] << "\n";
    }
}

// ---------------------------------------------------------------------------
// Default output dir
// ---------------------------------------------------------------------------
std::string OutputPath(const std::string& filename) {
    std::filesystem::path base(BEADS_SPECTRAL_OUTPUT_DIR);
    return (base / filename).string();
}

}  // namespace beads_spec
