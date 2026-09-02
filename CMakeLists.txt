# DSO-746 demo oscilloscope / spectrum analyser / generator
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dso746)

target_sources(app PRIVATE
  src/main.c
  src/ui.c
  src/acq.c
  src/acq_algo.c
  src/gen.c
  src/dsp.c
  src/dsp_math.c
  src/sim.c
)
target_include_directories(app PRIVATE src)
