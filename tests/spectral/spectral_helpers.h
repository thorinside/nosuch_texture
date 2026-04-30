#pragma once
//
// Spectral / frequency-domain test helpers for the beads_dsp project.
//
// Phase 1 of the numerical correction plan: an offline analysis toolkit used
// by tests under tests/spectral/. None of these helpers are real-time-safe;
// they allocate freely and are used only from Catch2 tests.
//

#include <complex>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace beads_spec {

// --- Constants ---
constexpr float kSampleRate = 48000.0f;
constexpr int kFraPoints = 60;
constexpr float kFraFmin = 20.0f;
constexpr float kFraFmax = 22050.0f;

// 60 log-spaced frequencies between 20 Hz and 22050 Hz inclusive.
std::vector<float> StandardFraGrid();

// --- Signal generation ---
// All write `len` samples to `out`. Phase 0 at t=0.
void GenSine(float* out, std::size_t len, float freq_hz,
             float sr = kSampleRate, float amp = 1.0f);

// Logarithmic sweep f0 -> f1 over `len` samples (instantaneous frequency
// follows a log law). Useful for chirp-based analysis; not used for stepped
// FRA which is constructed manually.
void GenLogSweep(float* out, std::size_t len, float f0, float f1,
                 float sr = kSampleRate, float amp = 1.0f);

// Dirac at sample 0, zeros elsewhere.
void GenImpulse(float* out, std::size_t len, float amp = 1.0f);

// Reproducible white noise (xorshift32).
void GenWhiteNoise(float* out, std::size_t len, std::uint32_t seed,
                   float amp = 0.25f);

// --- FFT ---
// Real-input radix-2 Cooley-Tukey FFT.
//
// `n` must be a power of two and equal to the length of `in`. Output `out`
// must hold `n` complex bins (the full spectrum). For real-input magnitude
// analysis the caller typically uses bins [0, n/2].
//
// The implementation is the textbook iterative bit-reversed Cooley-Tukey;
// correctness and clarity matter more than throughput here.
void Fft(const float* in, std::size_t n, std::complex<float>* out);

// --- Frequency-domain helpers ---
// Linear magnitude at the FFT bin nearest target_hz. Returns 0 if `n` is 0.
float MagAtHz(const std::complex<float>* spectrum, std::size_t n, float sr,
              float target_hz);

// Sum of |X[k]|^2 across [f_lo_hz, f_hi_hz]. Both edges are inclusive in the
// nearest-bin sense.
float PowerInBand(const std::complex<float>* spectrum, std::size_t n, float sr,
                  float f_lo_hz, float f_hi_hz);

// --- Time-domain helpers ---
float Rms(const float* x, std::size_t len);
float PeakAbs(const float* x, std::size_t len);

// --- FRA via stepped sine ---
struct FraPoint {
    float hz;
    float db;  // 20*log10(rms_out / amp_in)
};

// Stepped-sine frequency response. For each frequency in `freqs` we run
// `settle_seconds` of sine through `process` (output discarded), then capture
// `measure_seconds` of output and compute the RMS. The dB value is referenced
// to the input amplitude (i.e. unity-gain identity -> 0 dB - 3.01 dB? no:
//
//   For a sine of amplitude A, RMS_in = A / sqrt(2). For a unity-gain
//   passthrough, RMS_out = A / sqrt(2). We report 20*log10(RMS_out / A) so
//   that the unity-gain case yields -3.01 dB? That would be confusing.
//
// We therefore report 20*log10(RMS_out / RMS_in) where RMS_in = amp/sqrt(2),
// so unity-gain identity yields 0 dB. The doc comment in the spec says
// "20*log10(rms_out / amp_in)"; we interpret this in the dB-referred-to-input
// sense (i.e. unity gain -> 0 dB), since that is what every test downstream
// expects.
std::vector<FraPoint> SteppedSineFra(
    std::function<float(float)> process,
    const std::vector<float>& freqs,
    float sr = kSampleRate,
    float settle_seconds = 0.2f,
    float measure_seconds = 0.3f,
    float input_amp = 0.25f);

// --- THD ---
// Total harmonic distortion + noise as a percentage. Bins at +/- k*f0 for
// k = 2..nharm are treated as harmonics; remaining non-DC, non-fundamental
// bins are noise. Returns 100 * sqrt(noise_power + harmonic_power) / fund.
float ThdPlusNPct(const std::complex<float>* spectrum, std::size_t n, float sr,
                  float f0, int nharm = 8);

// --- IR-based magnitude response ---
// Feed a Dirac of amplitude `impulse_amp` through `process`, capture `n`
// samples (n must be power of two), FFT, and return magnitude (in dB) at each
// frequency in `freqs`.
//
// For a Dirac of amplitude A the FFT magnitude IS A * |H(f)|. We therefore
// convert to dB as 20*log10(|X[k]| / impulse_amp), so unity-gain identity
// yields 0 dB across the whole band.
std::vector<FraPoint> IrFra(std::function<float(float)> process,
                            const std::vector<float>& freqs,
                            std::size_t n,
                            float sr = kSampleRate,
                            float impulse_amp = 1.0f);

// --- CSV output ---
// Writes "freq_hz,magnitude_db\n" with a header row. Creates parent dir if
// needed.
void WriteFraCsv(const std::string& path,
                 const std::vector<FraPoint>& pts);

// Writes raw samples as "index,sample\n".
void WriteSamplesCsv(const std::string& path, const float* x, std::size_t len);

// --- Default output dir ---
// Joins BEADS_SPECTRAL_OUTPUT_DIR (set by CMake) with `filename`.
std::string OutputPath(const std::string& filename);

}  // namespace beads_spec
