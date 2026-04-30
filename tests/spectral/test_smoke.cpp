// Smoke test for the spectral helpers. These checks exist primarily to
// guarantee that every public helper compiles and links correctly, and that
// the FFT / FRA paths produce the values their downstream tests will rely on.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <complex>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "spectral_helpers.h"

using Catch::Approx;
using namespace beads_spec;

TEST_CASE("FFT of a unit cosine matches expected magnitude", "[spectral][smoke][fft]") {
    constexpr std::size_t N = 1024;
    constexpr int kBin = 100;
    std::vector<float> x(N);
    const float w = 2.0f * 3.14159265358979323846f *
                    static_cast<float>(kBin) / static_cast<float>(N);
    for (std::size_t i = 0; i < N; ++i) {
        x[i] = std::cos(w * static_cast<float>(i));
    }
    std::vector<std::complex<float>> X(N);
    Fft(x.data(), N, X.data());

    const float bin_hz = (static_cast<float>(kBin) / static_cast<float>(N)) *
                         kSampleRate;
    const float mag = MagAtHz(X.data(), N, kSampleRate, bin_hz);
    REQUIRE(mag == Approx(static_cast<float>(N) / 2.0f).margin(1e-2f));

    // Bins far from kBin should be ~0.
    REQUIRE(std::abs(X[10]) < 1e-2f);
    REQUIRE(std::abs(X[N / 2 - 1]) < 1e-2f);
}

TEST_CASE("FFT of an impulse is flat", "[spectral][smoke][fft]") {
    constexpr std::size_t N = 256;
    std::vector<float> x(N, 0.0f);
    GenImpulse(x.data(), N, 1.0f);
    std::vector<std::complex<float>> X(N);
    Fft(x.data(), N, X.data());
    for (std::size_t k = 0; k < N; ++k) {
        REQUIRE(std::abs(X[k]) == Approx(1.0f).margin(1e-4f));
    }
}

TEST_CASE("RMS of a unit sine equals 1/sqrt(2)", "[spectral][smoke][rms]") {
    constexpr std::size_t N = 48000;  // 1 s of sine at 1 kHz
    std::vector<float> x(N);
    GenSine(x.data(), N, 1000.0f, kSampleRate, 1.0f);
    const float r = Rms(x.data(), N);
    REQUIRE(r == Approx(1.0f / std::sqrt(2.0f)).margin(1e-3f));
    REQUIRE(PeakAbs(x.data(), N) == Approx(1.0f).margin(1e-3f));
}

TEST_CASE("GenWhiteNoise is reproducible for the same seed",
          "[spectral][smoke][noise]") {
    constexpr std::size_t N = 4096;
    std::vector<float> a(N), b(N), c(N);
    GenWhiteNoise(a.data(), N, 0xDEADBEEFu);
    GenWhiteNoise(b.data(), N, 0xDEADBEEFu);
    GenWhiteNoise(c.data(), N, 0xCAFEBABEu);
    for (std::size_t i = 0; i < N; ++i) {
        REQUIRE(a[i] == b[i]);
    }
    // Different seed should produce different output (at least somewhere).
    bool any_diff = false;
    for (std::size_t i = 0; i < N && !any_diff; ++i) {
        if (a[i] != c[i]) any_diff = true;
    }
    REQUIRE(any_diff);
}

TEST_CASE("IrFra of identity passthrough is flat",
          "[spectral][smoke][irfra]") {
    auto identity = [](float x) { return x; };
    const auto pts = IrFra(identity, StandardFraGrid(), 4096);
    REQUIRE(pts.size() == static_cast<std::size_t>(kFraPoints));
    for (const auto& p : pts) {
        REQUIRE(p.db == Approx(0.0f).margin(0.05f));
    }
}

TEST_CASE("ThdPlusNPct of a pure sine is ~0", "[spectral][smoke][thd]") {
    constexpr std::size_t N = 8192;
    // Choose 1 kHz so that f0 lands exactly on a bin (1000 Hz @ 48 kHz
    // and N=8192 is non-integer, so we use a bin-aligned freq instead).
    const float bin_width = kSampleRate / static_cast<float>(N);
    const long target_bin =
        static_cast<long>(std::lround(1000.0f / bin_width));
    const float f0 = static_cast<float>(target_bin) * bin_width;
    std::vector<float> x(N);
    GenSine(x.data(), N, f0, kSampleRate, 1.0f);
    std::vector<std::complex<float>> X(N);
    Fft(x.data(), N, X.data());
    const float thd = ThdPlusNPct(X.data(), N, kSampleRate, f0, 8);
    REQUIRE(thd < 0.1f);
}

TEST_CASE("SteppedSineFra of a one-pole LP at 1 kHz cutoff is -3 dB at fc",
          "[spectral][smoke][fra]") {
    // y[n] = a*x[n] + b*y[n-1], with cutoff fc.
    constexpr float fc = 1000.0f;
    const float wc = 2.0f * 3.14159265358979323846f * fc / kSampleRate;
    const float b = std::exp(-wc);
    const float a = 1.0f - b;

    float state = 0.0f;
    auto lp = [&state, a, b](float x) {
        state = a * x + b * state;
        return state;
    };

    std::vector<float> freqs = {fc};
    auto pts = SteppedSineFra(lp, freqs, kSampleRate, 0.2f, 0.3f, 0.25f);
    REQUIRE(pts.size() == 1);
    REQUIRE(pts[0].db == Approx(-3.01f).margin(0.5f));
}

TEST_CASE("WriteFraCsv produces a non-empty file with the expected header",
          "[spectral][smoke][csv]") {
    auto identity = [](float x) { return x; };
    auto pts = IrFra(identity, StandardFraGrid(), 4096);
    const std::string path = OutputPath("smoke_fra.csv");
    WriteFraCsv(path, pts);

    std::ifstream f(path);
    REQUIRE(f.is_open());
    std::stringstream buf;
    buf << f.rdbuf();
    const std::string contents = buf.str();
    REQUIRE_FALSE(contents.empty());
    REQUIRE(contents.find("freq_hz,magnitude_db") != std::string::npos);
}
