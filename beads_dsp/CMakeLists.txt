cmake_minimum_required(VERSION 3.16)
project(beads_dsp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(beads_dsp STATIC
    src/buffer/recording_buffer.cpp
    src/grain/grain.cpp
    src/grain/grain_scheduler.cpp
    src/grain/grain_engine.cpp
    src/delay/delay_engine.cpp
    src/fx/reverb.cpp
    src/fx/saturation.cpp
    src/quality/quality_processor.cpp
    src/pitch/pitch_quantizer.cpp
    src/random/random.cpp
    src/random/attenurandomizer.cpp
    src/input/auto_gain.cpp
    src/wavetable/wavetable_oscillator.cpp
    src/beads_processor.cpp
)

target_include_directories(beads_dsp
    PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_options(beads_dsp PRIVATE
    -Wall -Wextra -Wpedantic
)
