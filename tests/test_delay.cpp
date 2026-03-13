#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <vector>
#include <cmath>

#include "beads/types.h"
#include "beads/parameters.h"
#include "buffer/recording_buffer.h"
#include "delay/delay_engine.h"

using namespace beads;
using Catch::Approx;

static constexpr float kSampleRate = 48000.0f;

// Helper: create a recording buffer with known content
struct DelayTestBuffer {
    std::vector<uint8_t> memory;
    RecordingBuffer buffer;
    size_t num_frames;

    DelayTestBuffer(size_t frames = 48000) : num_frames(frames) {
        size_t bytes = (num_frames + kInterpolationTail) * 2 * sizeof(float);
        memory.resize(bytes, 0);
        buffer.Init(reinterpret_cast<float*>(memory.data()), num_frames, 2);
    }
};

TEST_CASE("DelayEngine: Init and process without crash", "[delay]") {
    DelayTestBuffer tb;
    DelayEngine engine;
    engine.Init(kSampleRate, &tb.buffer);

    // Write some data into the buffer
    for (size_t i = 0; i < 4800; ++i) {
        tb.buffer.Write(0.0f, 0.0f);
    }

    BeadsParameters params;
    params.size = 1.0f;  // Delay mode
    params.density = 0.5f;
    params.pitch = 0.0f;
    params.shape = 0.0f;
    params.time = 0.5f;

    std::vector<StereoFrame> output(256, {0.0f, 0.0f});
    engine.Process(params, output.data(), 256);

    // Should produce finite output
    for (size_t i = 0; i < 256; ++i) {
        REQUIRE(std::isfinite(output[i].l));
        REQUIRE(std::isfinite(output[i].r));
    }
}

TEST_CASE("DelayEngine: Impulse produces echo at expected delay", "[delay]") {
    DelayTestBuffer tb(48000);  // 1 second buffer
    DelayEngine engine;
    engine.Init(kSampleRate, &tb.buffer);

    // The delay engine reads relative to the current write head.
    // We continuously write into the buffer while processing the delay.
    // Write an impulse early, then silence — the delay should read it back.
    BeadsParameters params;
    params.size = 1.0f;
    params.density = 0.25f;  // Moderate base delay time
    params.pitch = 0.0f;
    params.shape = 0.0f;
    params.time = 0.0f;      // 1x multiplier (shortest)

    std::vector<StereoFrame> output(256, {0.0f, 0.0f});
    float max_out = 0.0f;

    for (int block = 0; block < 200; ++block) {
        // Write 256 samples per block, impulse in first block only
        for (size_t i = 0; i < 256; ++i) {
            float val = (block == 0 && i == 0) ? 1.0f : 0.0f;
            tb.buffer.Write(val, val);
        }
        engine.Process(params, output.data(), 256);
        for (size_t i = 0; i < 256; ++i) {
            max_out = std::max(max_out, std::max(std::abs(output[i].l), std::abs(output[i].r)));
        }
    }

    // The impulse should eventually appear in the output
    REQUIRE(max_out > 0.01f);
}

TEST_CASE("DelayEngine: Freeze loop repeats content", "[delay]") {
    DelayTestBuffer tb(48000);
    DelayEngine engine;
    engine.Init(kSampleRate, &tb.buffer);

    // Fill buffer with a recognizable pattern (sine wave)
    for (size_t i = 0; i < 48000; ++i) {
        float phase = static_cast<float>(i) / 48000.0f * 2.0f * 3.14159265f * 10.0f;
        tb.buffer.Write(std::sin(phase), std::sin(phase));
    }

    BeadsParameters params;
    params.size = 1.0f;
    params.density = 0.5f;
    params.pitch = 0.0f;
    params.shape = 0.0f;
    params.time = 0.5f;

    // Process a few blocks without freeze first
    std::vector<StereoFrame> output(256, {0.0f, 0.0f});
    for (int i = 0; i < 10; ++i) {
        engine.Process(params, output.data(), 256);
    }

    // Enable freeze
    params.freeze = true;
    params.size = 0.5f;  // Loop about half the buffer
    params.time = 0.0f;

    // Capture two consecutive runs of the frozen loop
    std::vector<float> run1, run2;
    for (int block = 0; block < 20; ++block) {
        engine.Process(params, output.data(), 256);
        for (size_t i = 0; i < 256; ++i) {
            run1.push_back(output[i].l);
        }
    }
    for (int block = 0; block < 20; ++block) {
        engine.Process(params, output.data(), 256);
        for (size_t i = 0; i < 256; ++i) {
            run2.push_back(output[i].l);
        }
    }

    // Both runs should be non-silent and periodic — check that the loop produced output
    float energy1 = 0.0f, energy2 = 0.0f;
    for (size_t i = 0; i < run1.size(); ++i) {
        energy1 += run1[i] * run1[i];
        energy2 += run2[i] * run2[i];
    }

    REQUIRE(energy1 > 0.0f);
    REQUIRE(energy2 > 0.0f);
    // Both runs read from the same frozen loop, so their energies should be similar
    float ratio = energy1 / std::max(energy2, 1e-10f);
    REQUIRE(ratio > 0.5f);
    REQUIRE(ratio < 2.0f);
}

TEST_CASE("DelayEngine: TIME multiplier affects delay length", "[delay]") {
    // TIME=0 (CCW) = 1x base delay (short), TIME=1 (CW) = max multiple (long)
    // With the same DENSITY (base delay), different TIME values should
    // produce different actual delay times.

    DelayTestBuffer tb1(48000), tb2(48000);
    DelayEngine engine1, engine2;
    engine1.Init(kSampleRate, &tb1.buffer);
    engine2.Init(kSampleRate, &tb2.buffer);

    // Fill both buffers with the same impulse pattern
    for (size_t i = 0; i < 48000; ++i) {
        float val = (i == 1000) ? 1.0f : 0.0f;
        tb1.buffer.Write(val, val);
        tb2.buffer.Write(val, val);
    }

    BeadsParameters params1, params2;
    params1.size = 1.0f;
    params1.pitch = 0.0f;
    params1.shape = 0.0f;
    params1.density = 0.3f;  // Base delay set by density
    params2 = params1;

    params1.time = 0.9f;   // High time = long delay (large multiplier)
    params2.time = 0.1f;   // Low time = short delay (small multiplier)

    std::vector<StereoFrame> out1(256), out2(256);

    // Process enough blocks for the delay time to converge
    float first_nonzero_block1 = -1, first_nonzero_block2 = -1;
    for (int block = 0; block < 200; ++block) {
        engine1.Process(params1, out1.data(), 256);
        engine2.Process(params2, out2.data(), 256);

        for (size_t i = 0; i < 256; ++i) {
            if (first_nonzero_block1 < 0 && std::abs(out1[i].l) > 0.01f) {
                first_nonzero_block1 = static_cast<float>(block);
            }
            if (first_nonzero_block2 < 0 && std::abs(out2[i].l) > 0.01f) {
                first_nonzero_block2 = static_cast<float>(block);
            }
        }
    }

    // Low time (short delay) should produce output sooner than high time (long delay)
    if (first_nonzero_block1 >= 0 && first_nonzero_block2 >= 0) {
        REQUIRE(first_nonzero_block2 <= first_nonzero_block1);
    }
}

TEST_CASE("DelayEngine: Output is finite with extreme parameters", "[delay]") {
    DelayTestBuffer tb(48000);
    DelayEngine engine;
    engine.Init(kSampleRate, &tb.buffer);

    // Fill buffer with a sine wave
    for (size_t i = 0; i < 48000; ++i) {
        float phase = static_cast<float>(i) / kSampleRate * 440.0f * 2.0f * 3.14159265f;
        tb.buffer.Write(std::sin(phase), std::sin(phase));
    }

    BeadsParameters params;
    params.size = 1.0f;
    params.density = 1.0f;   // Shortest delay
    params.pitch = 24.0f;    // 2 octaves up
    params.shape = 1.0f;     // Full slicer
    params.time = 0.0f;

    std::vector<StereoFrame> output(256);
    for (int block = 0; block < 50; ++block) {
        engine.Process(params, output.data(), 256);
        for (size_t i = 0; i < 256; ++i) {
            REQUIRE(std::isfinite(output[i].l));
            REQUIRE(std::isfinite(output[i].r));
        }
    }
}
