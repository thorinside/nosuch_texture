#pragma once

#include <cstddef>
#include <cstdint>
#include <cmath>

namespace beads {

struct StereoFrame {
    float l, r;

    StereoFrame operator+(const StereoFrame& other) const {
        return {l + other.l, r + other.r};
    }
    StereoFrame operator*(float gain) const {
        return {l * gain, r * gain};
    }
    StereoFrame& operator+=(const StereoFrame& other) {
        l += other.l;
        r += other.r;
        return *this;
    }
    StereoFrame& operator*=(float gain) {
        l *= gain;
        r *= gain;
        return *this;
    }
};

enum class QualityMode : uint8_t {
    kHiFi = 0,
    kClouds = 1,
    kCleanLoFi = 2,
    kTape = 3
};

enum class TriggerMode : uint8_t {
    kLatched = 0,
    kGated = 1,
    kClocked = 2
};

// Maximum number of simultaneous grains
static constexpr int kMaxGrains = 30;

// Default recording buffer duration in seconds
static constexpr float kDefaultBufferDuration = 4.0f;

// Reverb delay memory size (12 partitioned delay lines, ~12K samples needed)
static constexpr size_t kReverbBufferSize = 16384;

// Processing block size (kept small to limit stack usage; the Process() loop
// handles arbitrary num_frames by iterating in chunks of this size)
static constexpr size_t kMaxBlockSize = 64;

// Hermite interpolation requires 4 samples
static constexpr int kInterpolationTail = 4;

// Wavetable size
static constexpr int kWavetableSize = 256;

// Silence threshold for wavetable mode activation (10 seconds)
static constexpr float kSilenceThresholdSeconds = 10.0f;

// Wavetable data provider — implemented by host wrapper
struct WavetableProvider {
    virtual ~WavetableProvider() = default;
    // Return pointer to waveform data for given bank and index within bank
    // Data is kWavetableSize samples, normalized float [-1, 1]
    virtual const float* GetWaveform(int bank, int index) const = 0;
    virtual int NumBanksAvailable() const = 0;
    virtual int WaveformsPerBank() const = 0;
};

} // namespace beads
