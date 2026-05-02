#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <vector>
#include <cmath>
#include <cstring>

#include "beads/beads.h"

using namespace beads;
using Catch::Approx;

static constexpr float kSampleRate = 48000.0f;
static constexpr size_t kBlockSize = 256;

// Helper to create and init a processor
struct TestProcessor {
    std::vector<uint8_t> memory;
    BeadsProcessor processor;

    TestProcessor() {
        auto req = BeadsProcessor::GetMemoryRequirements(kSampleRate);
        memory.resize(req.total_bytes, 0);
        processor.Init(memory.data(), memory.size(), kSampleRate);
    }
};

TEST_CASE("BeadsProcessor: GetMemoryRequirements returns sensible values", "[processor]") {
    auto req = BeadsProcessor::GetMemoryRequirements(kSampleRate);
    REQUIRE(req.total_bytes > 0);
    REQUIRE(req.alignment > 0);
    // Should be roughly 1.5MB with 192K-frame buffer (4s at 48kHz)
    REQUIRE(req.total_bytes > 1000000);
    REQUIRE(req.total_bytes < 10000000);
}

TEST_CASE("BeadsProcessor: Init does not crash", "[processor]") {
    TestProcessor tp;
    // If we got here, Init succeeded
    REQUIRE(tp.processor.IsDelayMode() == false);
    REQUIRE(tp.processor.IsWavetableMode() == false);
    REQUIRE(tp.processor.ActiveGrainCount() == 0);
}

TEST_CASE("BeadsProcessor: Process with silence input", "[processor]") {
    TestProcessor tp;

    BeadsParameters params;
    tp.processor.SetParameters(params);

    std::vector<StereoFrame> input(kBlockSize, {0.0f, 0.0f});
    std::vector<StereoFrame> output(kBlockSize);

    tp.processor.Process(input.data(), output.data(), kBlockSize);

    // With silence in and default params, output should be near silence
    for (size_t i = 0; i < kBlockSize; ++i) {
        REQUIRE(std::isfinite(output[i].l));
        REQUIRE(std::isfinite(output[i].r));
    }
}

TEST_CASE("BeadsProcessor: Process with sine input produces output", "[processor]") {
    TestProcessor tp;

    BeadsParameters params;
    params.density = 0.1f;  // Far left of noon = fast grain trigger rate
    params.dry_wet = 1.0f;  // Full wet
    params.time = 0.95f;    // Read near write head (where data has been written)
    params.size = 0.3f;     // Short grains
    params.shape = 0.5f;
    params.pitch = 0.0f;
    params.manual_gain_db = 0.0f;  // Bypass auto-gain ramping
    params.trigger_mode = TriggerMode::kLatched;
    tp.processor.SetParameters(params);

    // Generate a sine wave input
    std::vector<StereoFrame> input(kBlockSize);
    for (size_t i = 0; i < kBlockSize; ++i) {
        float phase = static_cast<float>(i) / kSampleRate * 440.0f * 2.0f * 3.14159265f;
        input[i] = {std::sin(phase), std::sin(phase)};
    }

    std::vector<StereoFrame> output(kBlockSize);

    // Process enough blocks to fill buffer, trigger grains, and render output
    float max_level = 0.0f;
    for (int block = 0; block < 200; ++block) {
        tp.processor.Process(input.data(), output.data(), kBlockSize);
        for (size_t i = 0; i < kBlockSize; ++i) {
            max_level = std::max(max_level, std::max(std::abs(output[i].l), std::abs(output[i].r)));
            REQUIRE(std::isfinite(output[i].l));
            REQUIRE(std::isfinite(output[i].r));
        }
    }

    // After many blocks, we should have some output from grains
    REQUIRE(max_level > 0.001f);
}

TEST_CASE("BeadsProcessor: Delay mode activates at size=1.0", "[processor]") {
    TestProcessor tp;

    BeadsParameters params;
    params.size = 1.0f;
    tp.processor.SetParameters(params);
    REQUIRE(tp.processor.IsDelayMode() == true);

    params.size = 0.5f;
    tp.processor.SetParameters(params);
    REQUIRE(tp.processor.IsDelayMode() == false);
}

TEST_CASE("BeadsProcessor: No NaN in output with extreme parameters", "[processor]") {
    TestProcessor tp;

    BeadsParameters params;
    params.density = 1.0f;
    params.feedback = 0.99f;
    params.dry_wet = 1.0f;
    params.reverb = 1.0f;
    params.size = 0.5f;
    params.pitch = 24.0f;  // 2 octaves up
    params.quality_mode = QualityMode::kTape;
    params.manual_gain_db = 0.0f;  // Fixed gain to avoid auto-gain runaway
    tp.processor.SetParameters(params);

    std::vector<StereoFrame> input(kBlockSize);
    for (size_t i = 0; i < kBlockSize; ++i) {
        float phase = static_cast<float>(i) / kSampleRate * 440.0f * 2.0f * 3.14159265f;
        input[i] = {std::sin(phase), std::sin(phase)};
    }

    std::vector<StereoFrame> output(kBlockSize);

    for (int block = 0; block < 100; ++block) {
        tp.processor.Process(input.data(), output.data(), kBlockSize);
        for (size_t i = 0; i < kBlockSize; ++i) {
            REQUIRE(std::isfinite(output[i].l));
            REQUIRE(std::isfinite(output[i].r));
        }
    }
}

TEST_CASE("BeadsProcessor: Freeze stops recording", "[processor]") {
    TestProcessor tp;

    BeadsParameters params;
    params.density = 0.3f;
    params.size = 0.5f;
    params.dry_wet = 0.5f;
    tp.processor.SetParameters(params);

    std::vector<StereoFrame> input(kBlockSize);
    for (size_t i = 0; i < kBlockSize; ++i) {
        float phase = static_cast<float>(i) / kSampleRate * 440.0f * 2.0f * 3.14159265f;
        input[i] = {std::sin(phase), std::sin(phase)};
    }
    std::vector<StereoFrame> output(kBlockSize);

    // Process a few blocks
    for (int i = 0; i < 10; ++i) {
        tp.processor.Process(input.data(), output.data(), kBlockSize);
    }

    // Freeze
    params.freeze = true;
    tp.processor.SetParameters(params);
    tp.processor.Process(input.data(), output.data(), kBlockSize);

    // Process should still work without crash
    for (int i = 0; i < 10; ++i) {
        tp.processor.Process(input.data(), output.data(), kBlockSize);
        for (size_t j = 0; j < kBlockSize; ++j) {
            REQUIRE(std::isfinite(output[j].l));
            REQUIRE(std::isfinite(output[j].r));
        }
    }
}

TEST_CASE("BeadsProcessor: All quality modes work without NaN", "[processor]") {
    for (int mode = 0; mode < 4; ++mode) {
        TestProcessor tp;

        BeadsParameters params;
        params.quality_mode = static_cast<QualityMode>(mode);
        params.density = 0.3f;
        params.size = 0.5f;
        params.dry_wet = 1.0f;
        tp.processor.SetParameters(params);

        std::vector<StereoFrame> input(kBlockSize);
        for (size_t i = 0; i < kBlockSize; ++i) {
            float phase = static_cast<float>(i) / kSampleRate * 440.0f * 2.0f * 3.14159265f;
            input[i] = {std::sin(phase), std::sin(phase)};
        }

        std::vector<StereoFrame> output(kBlockSize);
        for (int block = 0; block < 20; ++block) {
            tp.processor.Process(input.data(), output.data(), kBlockSize);
            for (size_t i = 0; i < kBlockSize; ++i) {
                REQUIRE(std::isfinite(output[i].l));
                REQUIRE(std::isfinite(output[i].r));
            }
        }
    }
}

TEST_CASE("BeadsProcessor: Mode transitions produce no NaN", "[processor][decimation]") {
    TestProcessor tp;

    std::vector<StereoFrame> input(kBlockSize);
    for (size_t i = 0; i < kBlockSize; ++i) {
        float phase = static_cast<float>(i) / kSampleRate * 440.0f * 2.0f * 3.14159265f;
        input[i] = {std::sin(phase), std::sin(phase)};
    }
    std::vector<StereoFrame> output(kBlockSize);

    // Cycle through all quality modes rapidly
    QualityMode modes[] = {
        QualityMode::kHiFi, QualityMode::kClouds,
        QualityMode::kCleanLoFi, QualityMode::kTape,
        QualityMode::kHiFi
    };

    for (auto mode : modes) {
        BeadsParameters params;
        params.quality_mode = mode;
        params.density = 0.3f;
        params.size = 0.5f;
        params.dry_wet = 1.0f;
        params.manual_gain_db = 0.0f;
        tp.processor.SetParameters(params);

        for (int block = 0; block < 30; ++block) {
            tp.processor.Process(input.data(), output.data(), kBlockSize);
            for (size_t i = 0; i < kBlockSize; ++i) {
                REQUIRE(std::isfinite(output[i].l));
                REQUIRE(std::isfinite(output[i].r));
            }
        }
    }
}

TEST_CASE("BeadsProcessor: LoFi delay mode produces output", "[processor][decimation]") {
    // Verify LoFi delay mode (8x decimation) works correctly end-to-end.
    // The buffer-level "Effective duration scales with decimation" test
    // verifies the >2s retention property directly.
    TestProcessor tp;

    BeadsParameters params;
    params.quality_mode = QualityMode::kCleanLoFi;
    params.size = 1.0f;      // Delay mode
    params.density = 0.3f;   // Moderate base delay
    params.time = 0.0f;      // Short delay (read near write head)
    params.dry_wet = 1.0f;
    params.pitch = 0.0f;
    params.shape = 0.0f;
    params.manual_gain_db = 0.0f;
    tp.processor.SetParameters(params);

    std::vector<StereoFrame> input(kBlockSize);
    std::vector<StereoFrame> output(kBlockSize);

    // Feed continuous audio and check for delay output
    float max_level = 0.0f;
    for (int block = 0; block < 200; ++block) {
        for (size_t i = 0; i < kBlockSize; ++i) {
            float t = static_cast<float>(block * kBlockSize + i);
            float phase = t / kSampleRate * 440.0f * 2.0f * 3.14159265f;
            input[i] = {std::sin(phase) * 0.5f, std::sin(phase) * 0.5f};
        }
        tp.processor.Process(input.data(), output.data(), kBlockSize);

        // Check later blocks after delay has converged
        if (block > 50) {
            for (size_t i = 0; i < kBlockSize; ++i) {
                max_level = std::max(max_level,
                    std::max(std::abs(output[i].l), std::abs(output[i].r)));
                REQUIRE(std::isfinite(output[i].l));
                REQUIRE(std::isfinite(output[i].r));
            }
        }
    }

    REQUIRE(max_level > 0.001f);
}

TEST_CASE("BeadsProcessor: Memory requirements unchanged with decimation", "[processor][decimation]") {
    // Decimation doesn't change the physical buffer size — verify requirements
    // are the same regardless of what mode we'll use
    auto req = BeadsProcessor::GetMemoryRequirements(kSampleRate);

    // Fixed 192K-frame buffer (~1.5MB stereo float + overhead)
    REQUIRE(req.total_bytes > 1000000);
    REQUIRE(req.total_bytes < 10000000);
}

TEST_CASE("BeadsProcessor: Feedback produces sustained echo in delay mode", "[processor]") {
    // Run the same scenario with and without feedback.
    // With feedback, output during silence should be significantly louder
    // because the signal is re-recorded into the buffer on each delay cycle.
    float echo_level_with_fb = 0.0f;
    float echo_level_without_fb = 0.0f;

    for (int use_feedback = 0; use_feedback < 2; ++use_feedback) {
        TestProcessor tp;

        BeadsParameters params;
        params.size = 1.0f;             // Delay mode
        params.density = 0.3f;          // Base delay time (moderate)
        params.time = 0.3f;             // Multiplier (moderate)
        params.feedback = use_feedback ? 0.9f : 0.0f;
        params.dry_wet = 1.0f;          // Full wet
        params.pitch = 0.0f;
        params.shape = 0.0f;            // No tremolo
        params.manual_gain_db = 0.0f;   // Unity gain
        tp.processor.SetParameters(params);

        std::vector<StereoFrame> input(kBlockSize);
        std::vector<StereoFrame> output(kBlockSize);

        // Phase 1: Feed a sine burst for ~50 blocks
        for (int block = 0; block < 50; ++block) {
            for (size_t i = 0; i < kBlockSize; ++i) {
                float t = static_cast<float>(block * kBlockSize + i);
                float phase = t / kSampleRate * 440.0f * 2.0f * 3.14159265f;
                input[i] = {std::sin(phase) * 0.5f, std::sin(phase) * 0.5f};
            }
            tp.processor.Process(input.data(), output.data(), kBlockSize);
        }

        // Phase 2: Feed silence for many blocks — measure LATE output
        // Skip early blocks where buffer still has original content
        // Focus on blocks 100-200 where only feedback-sustained content remains
        float max_late_level = 0.0f;
        for (int block = 0; block < 200; ++block) {
            for (size_t i = 0; i < kBlockSize; ++i) {
                input[i] = {0.0f, 0.0f};
            }
            tp.processor.Process(input.data(), output.data(), kBlockSize);

            if (block >= 100) {
                for (size_t i = 0; i < kBlockSize; ++i) {
                    max_late_level = std::max(max_late_level,
                        std::max(std::abs(output[i].l), std::abs(output[i].r)));
                }
            }
        }

        if (use_feedback)
            echo_level_with_fb = max_late_level;
        else
            echo_level_without_fb = max_late_level;
    }

    // Without feedback, the buffer gets overwritten with silence and output drops to ~0
    // With 90% feedback, signal should still be audible after 200 blocks
    REQUIRE(echo_level_with_fb > echo_level_without_fb * 10.0f);
    REQUIRE(echo_level_with_fb > 0.01f);
}

TEST_CASE("BeadsProcessor: Output levels match Eurorack input levels", "[processor][levels]") {
    // Eurorack audio is ±5V. The plugin scales input ×0.2 (→ ±1.0 internal)
    // and output ×5.0 (→ ±5V). The DSP chain should maintain roughly unity
    // gain so that output amplitudes match input amplitudes.
    //
    // Test: send a ±1.0 sine (= ±5V Eurorack) through the full processor
    // in grain mode and delay mode, then verify the wet output peak is
    // within a reasonable range (not drastically attenuated or amplified).

    static constexpr float kSineFreq = 440.0f;
    static constexpr float kSineAmplitude = 1.0f;  // ±5V Eurorack level
    static constexpr int kWarmupBlocks = 100;  // Let grains fill the buffer
    static constexpr int kMeasureBlocks = 100;

    auto make_sine_block = [](StereoFrame* buf, size_t n, int block_idx) {
        for (size_t i = 0; i < n; ++i) {
            float t = static_cast<float>(block_idx * static_cast<int>(n) + static_cast<int>(i));
            float phase = t / kSampleRate * kSineFreq * 2.0f * 3.14159265f;
            float s = std::sin(phase) * kSineAmplitude;
            buf[i] = {s, s};
        }
    };

    // -- Grain mode: 100% wet, 0dB gain, no reverb, no feedback --
    SECTION("Grain mode at 100% wet") {
        TestProcessor tp;
        BeadsParameters params;
        params.density = 0.2f;            // Regular grain triggers
        params.size = 0.5f;               // Medium grains
        params.time = 0.0f;               // Read from most recent audio
        params.shape = 0.5f;
        params.pitch = 0.0f;              // Unity pitch
        params.dry_wet = 1.0f;            // Full wet
        params.feedback = 0.0f;
        params.reverb = 0.0f;
        params.manual_gain_db = 0.0f;     // 0dB = unity
        params.auto_gain = false;         // Test the DSP chain's unity gain, not AutoGain calibration
        params.trigger_mode = TriggerMode::kLatched;
        tp.processor.SetParameters(params);

        std::vector<StereoFrame> input(kBlockSize);
        std::vector<StereoFrame> output(kBlockSize);

        // Warm up: fill buffer and let grains stabilize
        for (int b = 0; b < kWarmupBlocks; ++b) {
            make_sine_block(input.data(), kBlockSize, b);
            tp.processor.Process(input.data(), output.data(), kBlockSize);
        }

        // Measure peak output level
        float peak = 0.0f;
        for (int b = 0; b < kMeasureBlocks; ++b) {
            make_sine_block(input.data(), kBlockSize, kWarmupBlocks + b);
            tp.processor.Process(input.data(), output.data(), kBlockSize);
            for (size_t i = 0; i < kBlockSize; ++i) {
                peak = std::max(peak,
                    std::max(std::abs(output[i].l), std::abs(output[i].r)));
            }
        }

        // Wet output should be within -6dB to +3dB of input level.
        // (±1.0 input → output peak should be between 0.5 and 1.4)
        INFO("Grain mode wet peak = " << peak);
        REQUIRE(peak > 0.5f);
        REQUIRE(peak < 1.4f);
    }

    // -- Delay mode: 100% wet, 0dB gain --
    SECTION("Delay mode at 100% wet") {
        TestProcessor tp;
        BeadsParameters params;
        params.size = 1.0f;               // Delay mode
        params.density = 0.3f;            // Moderate delay time
        params.time = 0.1f;               // Short delay multiplier
        params.pitch = 0.0f;
        params.shape = 0.0f;              // No tremolo
        params.dry_wet = 1.0f;
        params.feedback = 0.0f;
        params.reverb = 0.0f;
        params.manual_gain_db = 0.0f;
        params.auto_gain = false;         // Test the DSP chain's unity gain, not AutoGain calibration
        tp.processor.SetParameters(params);

        std::vector<StereoFrame> input(kBlockSize);
        std::vector<StereoFrame> output(kBlockSize);

        for (int b = 0; b < kWarmupBlocks; ++b) {
            make_sine_block(input.data(), kBlockSize, b);
            tp.processor.Process(input.data(), output.data(), kBlockSize);
        }

        float peak = 0.0f;
        for (int b = 0; b < kMeasureBlocks; ++b) {
            make_sine_block(input.data(), kBlockSize, kWarmupBlocks + b);
            tp.processor.Process(input.data(), output.data(), kBlockSize);
            for (size_t i = 0; i < kBlockSize; ++i) {
                peak = std::max(peak,
                    std::max(std::abs(output[i].l), std::abs(output[i].r)));
            }
        }

        INFO("Delay mode wet peak = " << peak);
        REQUIRE(peak > 0.5f);
        REQUIRE(peak < 1.4f);
    }

    // -- Dry pass-through: should be unity --
    SECTION("Dry pass-through") {
        TestProcessor tp;
        BeadsParameters params;
        params.dry_wet = 0.0f;            // Full dry
        params.reverb = 0.0f;
        params.manual_gain_db = 0.0f;
        tp.processor.SetParameters(params);

        std::vector<StereoFrame> input(kBlockSize);
        std::vector<StereoFrame> output(kBlockSize);

        // A few blocks to settle
        for (int b = 0; b < 10; ++b) {
            make_sine_block(input.data(), kBlockSize, b);
            tp.processor.Process(input.data(), output.data(), kBlockSize);
        }

        float peak = 0.0f;
        for (int b = 0; b < 10; ++b) {
            make_sine_block(input.data(), kBlockSize, 10 + b);
            tp.processor.Process(input.data(), output.data(), kBlockSize);
            for (size_t i = 0; i < kBlockSize; ++i) {
                peak = std::max(peak,
                    std::max(std::abs(output[i].l), std::abs(output[i].r)));
            }
        }

        // Dry pass-through should be near unity
        INFO("Dry pass-through peak = " << peak);
        REQUIRE(peak > 0.9f);
        REQUIRE(peak < 1.1f);
    }
}

TEST_CASE("BeadsProcessor: High density + large size produces continuous audio", "[processor][stress]") {
    TestProcessor tp;

    BeadsParameters params;
    params.density = 0.1f;    // Fast grain triggers
    params.size = 0.9f;       // Long grains (many active)
    params.dry_wet = 1.0f;
    params.time = 0.5f;
    params.shape = 0.5f;
    params.pitch = 0.0f;
    params.manual_gain_db = 0.0f;
    params.trigger_mode = TriggerMode::kLatched;
    tp.processor.SetParameters(params);

    std::vector<StereoFrame> input(kBlockSize);
    for (size_t i = 0; i < kBlockSize; ++i) {
        float phase = static_cast<float>(i) / kSampleRate * 440.0f * 2.0f * 3.14159265f;
        input[i] = {std::sin(phase), std::sin(phase)};
    }

    std::vector<StereoFrame> output(kBlockSize);

    bool all_finite = true;
    bool had_output = false;

    for (int block = 0; block < 200; ++block) {
        tp.processor.Process(input.data(), output.data(), kBlockSize);

        for (size_t i = 0; i < kBlockSize; ++i) {
            if (!std::isfinite(output[i].l) || !std::isfinite(output[i].r)) {
                all_finite = false;
            }
            float level = std::max(std::abs(output[i].l), std::abs(output[i].r));
            if (level > 0.001f) had_output = true;
        }
    }

    REQUIRE(all_finite);
    REQUIRE(had_output);
}
