# Spectral analysis test harness for beads_dsp.
#
# Builds a separate executable so a failing/expensive spectral test does not
# block the main beads_tests suite. Catch2 is fetched by the parent
# tests/CMakeLists.txt and is available here as Catch2::Catch2WithMain.

set(BEADS_SPECTRAL_OUTPUT_DIR "${CMAKE_BINARY_DIR}/spectral-output")
file(MAKE_DIRECTORY "${BEADS_SPECTRAL_OUTPUT_DIR}")

add_executable(beads_spectral_tests
    spectral_helpers.cpp
    test_smoke.cpp
    test_aliasing.cpp
    test_fra_per_stage.cpp
    test_thd_noise_transient.cpp
    test_end_to_end.cpp
)

target_link_libraries(beads_spectral_tests PRIVATE
    beads_dsp
    Catch2::Catch2WithMain
)

# Allow tests to include internal DSP headers under beads_dsp/src as well as
# the public headers.
target_include_directories(beads_spectral_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/beads_dsp/src
)

target_compile_definitions(beads_spectral_tests PRIVATE
    BEADS_SPECTRAL_OUTPUT_DIR="${BEADS_SPECTRAL_OUTPUT_DIR}"
)

catch_discover_tests(beads_spectral_tests)
