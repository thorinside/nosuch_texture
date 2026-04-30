include(FetchContent)

FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.5.2
)
FetchContent_MakeAvailable(Catch2)

add_executable(beads_tests
    test_buffer.cpp
    test_grain.cpp
    test_reverb.cpp
    test_processor.cpp
    test_click_prevention.cpp
    test_delay.cpp
    test_quality_modes.cpp
    test_auto_gain.cpp
    test_attenurandomizer.cpp
    test_silence.cpp
)

target_link_libraries(beads_tests PRIVATE beads_dsp Catch2::Catch2WithMain)

# Allow tests to include internal headers
target_include_directories(beads_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/beads_dsp/src
)

include(CTest)
include(Catch)
catch_discover_tests(beads_tests)

# Spectral analysis test harness (Phase 1 numerical correction).
add_subdirectory(spectral)
