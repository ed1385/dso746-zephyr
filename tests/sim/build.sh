#!/bin/sh
# builds the UI simulation with LVGL 9 from the Zephyr workspace
set -e
LV=${LV:-/home/claude/zephyrproject/modules/lib/gui/lvgl}
CC=${CC:-gcc}
FLAGS="-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer -DLV_CONF_INCLUDE_SIMPLE -I. -I$LV -Iinc -I../../src -Wno-unused-function"
mkdir -p build
if [ ! -f build/liblvgl.a ]; then
  echo "compiling LVGL (once)..."
  find $LV/src -name '*.c' | grep -v -E '/(demos|examples|libs/(thorvg|freetype|lodepng|libpng|libjpeg_turbo|tjpgd|bmp|gif|rlottie|ffmpeg|barcode|qrcode|svg|expat|fsdrv|tiny_ttf|rle|lz4))/' > build/lv_files.txt
  n=0
  while read f; do o=build/lv_$(echo $f | md5sum | cut -c1-10).o; $CC $FLAGS -c $f -o $o 2>>build/lv_warn.txt || { echo "FAILED: $f"; exit 1; }; n=$((n+1)); done < build/lv_files.txt
  ar rcs build/liblvgl.a build/lv_*.o
  echo "LVGL: $n files"
fi
$CC $FLAGS sim_main.c hw_stubs.c ../../src/ui.c ../../src/sim.c ../../src/acq_algo.c ../../src/dsp_math.c build/liblvgl.a -lm -o build/uisim
echo "built build/uisim"
