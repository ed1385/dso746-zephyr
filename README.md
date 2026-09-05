# Digital Oscilloscope, FFT Spectrum Analyzer and Signal Generator

Three instruments on a single development board, one 480×272 touchscreen, no
external hardware required.

*Русская версия: [README.ru.md](README.ru.md).*

**Target: STM32F746G-DISCO** (STM32F746NG, Cortex-M7 @ 216 MHz, 4.3" 480×272
LCD, FT5336 capacitive touch, 8 MB SDRAM). **Firmware: Zephyr RTOS + LVGL 9.**

> **Work in progress.** Demo mode is fully functional and is what the
> screenshots below show. Live mode — real acquisition from the ADC — is
> implemented but not fully validated on hardware; some parts are verified by
> calculation and host tests only. See [Project status](#project-status).

---

## User interface

### Oscilloscope
![Oscilloscope](docs/images/ui-scope.png)

Two channels, simultaneous sampling, persistence, automatic measurements, XY
mode, cursors, trigger-level and channel-ground markers, 12×8 graticule.

### FFT spectrum analyzer
![Spectrum analyzer](docs/images/ui-fft.png)

FFT up to 4096 points, five window functions, selectable span, peak search,
waterfall with three color maps.

### Signal generator
![Generator](docs/images/ui-gen.png)

Waveform preview, numeric frequency entry, amplitude, offset and duty; output
armed by a long press; separate calibrator/square output.

### Phosphor key highlight
![Pressed key](docs/images/ui-pressed.png)

A pressed key ignites instantly and decays over 260 ms — a native LVGL style
transition, not a per-frame animation. The glow is drawn only on the active
key and RUN: shadows are computed in software and are not DMA2D-accelerated.

Screenshots are renders of the interactive mockup `docs/ui-mockup.html`, built
pixel-for-pixel against the firmware: same geometry, same palette already
quantized to RGB565. Open it in any browser — it is clickable.

---

## Operating modes

- **Oscilloscope** — two channels, simultaneous sampling, persistence,
  automatic measurements, XY mode, cursors, level markers.
- **FFT spectrum analyzer** — up to 4096-point FFT, five windows, selectable
  span, peak search, waterfall with three palettes.
- **Signal generator** — eight waveforms, numeric frequency entry, amplitude,
  offset, duty; output enabled by press-and-hold; separate calibrator/square.
- **Demo** and **live** — identical logic and interface; the *only* difference
  is the sample source (the simulator vs the ADC). Toggled by the blue **B1
  USER** button on the back of the board (pin PI11). The instrument always
  boots into demo mode.

---

## Technical characteristics

### Oscilloscope
| Parameter | Value |
|---|---|
| Channels | 2, simultaneous sampling (ADC1/PA0, ADC3/PF10, common TIM2 trigger) |
| ADC resolution | 12-bit |
| Max sample rate | 1.80 MSa/s per channel (5.40 MSa/s single-channel — planned) |
| Max measurable frequency | 360 kHz (at the ≥ 5× oversampling rule) |
| Input range | 0…3.3 V (±5 V, and ±50 V with a ×10 probe — with the future front end) |
| Vertical scale | 20 mV…2 V/div, 1-2-5 sequence |
| Voltage resolution | 0.81 mV/LSB |
| Timebase | 20 µs…100 ms/div, 1-2-5 sequence, 13 steps |
| Record length per screen | 336…1344 samples (min/max per column) |
| Pre-trigger | 0…100 % of the record (H-position ±6 div) |
| Display | persistence 0.1 s…∞, XY, graticule |

### Trigger
| Parameter | Value |
|---|---|
| Type | software, edge, with hysteresis and re-arm |
| Source | CH1, CH2 (EXT on PG6 — planned) |
| Slope / mode | Rising/Falling/Either · Auto/Normal/Single |
| Hysteresis | 0…160 mV in 8 mV steps |
| Jitter | ±1 sample (0.56 µs at 1.8 MSa/s) |

### Measurements
Vpp, Vmax, Vmin, Vavg, Vrms, frequency, duty cycle.

### FFT spectrum analyzer
| Parameter | Value |
|---|---|
| FFT length | 256…4096 points |
| Windows | Rectangular, Hann, Hamming, Blackman-Harris, Flat-Top |
| Span | 8 / 32 / 128 / 900 kHz (sample rate 16.4k…1.8M) |
| Frequency resolution | 4 Hz…440 Hz |
| FFT time, 2048 pts | ≈ 0.3 ms (Cortex-M7 @ 216 MHz with FPU) |
| Dynamic range | ≈ 70 dB |
| Waterfall | 84 lines, three palettes |

### Signal generator (PWM-DAC on PB4)
| Parameter | Value |
|---|---|
| Waveforms | sine, square, triangle, ramp up/down, pulse, sinc, noise |
| Sine / triangle / ramp | 1 Hz…42 kHz |
| Square, pulse | up to ≈ 1 MHz |
| Amplitude | 0.05…3.3 Vpp, 50 mV steps |
| Offset | 0…3.3 V (offset ± amplitude/2 always kept within 0…3.3 V) |
| PWM carrier | 8-bit → 422 kHz, 10-bit → 105 kHz (selected automatically) |
| Frequency entry | numeric keypad; the unit key (Hz/kHz/MHz) multiplies and applies |
| Calibrator / square | PH6 (D6), 2 Hz…2 MHz, duty 0…100 % |

### Interface and platform
| Parameter | Value |
|---|---|
| Display | 4.3", 480×272, RGB565, double-buffered LTDC |
| Trace rendering | 20 fps, 8-bit intensity map per channel |
| Touch latency | input polled every 10 ms |
| Graphics acceleration | DMA2D (Chrom-ART) |
| Touch targets | ≥ 46 px (9 mm), gaps ≥ 11 px |
| Resource use | FLASH 44 % of 1 MB, SRAM 47 % of 256 KB, SDRAM 1.2 MB of 8 |

---

## Building

    west build -b stm32f746g_disco -p always .

The board overlay in `boards/` is picked up automatically by board name.

## Flashing

The board carries an on-board ST-LINK/V2-1; no separate programmer is needed.

1. Connect a cable to **CN14 (ST-LINK)**.
2. A **DIS_F746NG** mass-storage drive appears.
3. Copy `build/zephyr/zephyr.bin` onto it — the board flashes and restarts.

Alternatively `west flash` (STM32CubeProgrammer in PATH) or the prebuilt
`.hex` from [Releases](../../releases). The Zephyr console is on the same
cable: virtual COM port, 115200 8N1.

If the drag-and-drop copy silently fails, update the ST-LINK firmware itself
and pick the **Debug + Mass Storage + VCP** variant — without Mass Storage the
drive does not appear. Internal flash is 1 MB; the image uses 44 %.

## Controls

The instrument **always boots into demo mode**: signals are synthesized,
nothing needs to be connected. The status line shows **DEMO** and SIM SIGNAL.
The blue **B1 USER** button on the back of the board (pin PI11, next to the
black B2 RESET) toggles demo and live in both directions.

| Control | Action |
|---|---|
| **B1 USER** (button on the back) | demo ↔ live |
| **MODE** | cycle SCOPE → FFT → GEN |
| **RUN / STOP** | run and stop; in generator mode, enable the output by press-and-hold |
| Group keys | open the parameter panel over the trace |
| Parameter tile | a list opens an options page (applied on tap); a stepped value collapses the panel so − / + and drag on the trace adjust it live; a frequency opens the keypad |
| **−** / **+** | change the selected parameter; auto-repeat on hold |
| Drag on the trace | channel offset, trigger level, horizontal position |

The stepper rule is single and absolute: **plus always makes what you see on
screen larger** — for scale parameters that steps the ladder down. Keys and
tiles that would do nothing in the current state are hidden.

## Inputs and outputs

| Signal | Pin | Connector |
|---|---|---|
| Channel 1 | PA0 | Arduino A0 |
| Channel 2 | PF10 | Arduino A1 |
| Generator output (PWM-DAC) | PB4 | Arduino D3 |
| Calibrator / square | PH6 | Arduino D6 |
| External trigger (planned) | PG6 | Arduino D2 |

The input accepts **0…3.3 V** — there is no analog front end on the board. The
firmware is written so that adding an external divider with offset changes only
the calibration constants.

---

## How it works

Full hardware-to-GUI architecture is in [docs/architecture.md](docs/architecture.md);
the UI logic specification is in [docs/UI-SPEC.md](docs/UI-SPEC.md).

**Acquisition.** TIM2 TRGO triggers ADC1 (CH1) and ADC3 (CH2) at the same
event, so the channels are synchronous to within one ADCCLK cycle. Samples are
streamed by circular DMA with half- and full-transfer interrupts. The ring
lives in internal SRAM and outside the D-cache: LTDC reads the framebuffer from
SDRAM continuously, a second stream into SDRAM would starve it of FMC
bandwidth, and the Cortex-M7 data cache is invisible to DMA.

**Trigger** is software, edge, with hysteresis and re-arm: it re-arms only
after the signal has left the hysteresis band, so a single noisy edge cannot
produce a burst of triggers. Hysteresis, hold-off and sample rate are exposed
in the menu.

**Trace rendering** bypasses widgets. Samples are reduced to 336 min/max pairs,
one per pixel column, so a single spike survives decimation. The pairs are
written into an 8-bit-per-channel intensity map; persistence is a constant
subtracted from the whole map once per frame, then the maps are composed into
an RGB565 canvas through a per-channel color ramp.

**Load split.** LVGL is serviced every 10 ms — about 40 ms touch latency — and
the waveform is recomputed every 50 ms. No widget is rewritten unless its value
actually changed, so a quiet panel only redraws the trace rectangle. Fills and
blits go to the Chrom-ART (DMA2D) accelerator.

---

## Software stack

| Layer | Used |
|---|---|
| OS | **Zephyr RTOS** (main, 4.4.99): threads, memory slab, queues, semaphores, mutexes, MPU-tagged non-cacheable region |
| Build | west, CMake, Kconfig, devicetree; **Zephyr SDK 1.0.1**, arm-zephyr-eabi GCC 14.3.0, picolibc |
| Graphics | **LVGL 9** via the Zephyr module; **DMA2D (Chrom-ART)** draw unit |
| DSP | **CMSIS-DSP**: `arm_rfft_fast_f32`, magnitude on the hardware FPU |
| Drivers | LTDC (display), FT5336 via the input subsystem (touch), I2C, gpio-keys (B1), logging |
| Peripherals | **STM32Cube LL** directly for the ADC, timers, DMA and pins |

Using LL is a deliberate exception: the Zephyr ADC driver offers only one-shot
reads and cannot do continuous circular DMA, which an oscilloscope needs. The
matching devicetree nodes are disabled so the drivers do not fight over the DMA
streams and interrupts.

### Host tests

The algorithm core — trigger search, pre-trigger extraction, min/max
reduction, measurements, the sweep planner — is in files with no hardware
access and builds with a normal compiler on the host:

    cd tests/host
    cc -I../../src -O2 -Wall -Wextra test_algo.c ../../src/acq_algo.c \
       ../../src/dsp_math.c -lm -o test_algo && ./test_algo

The tests check that 500 µs/div shows exactly 6 periods of a 1 kHz signal, that
a single spike survives decimation, that the fastest sweep is physically
reachable, and that hysteresis actually cures false triggers: without it a slow
noisy sine yields 33 triggers where there should be 4.

---

## Repository layout

```
src/acq.c        TIM2 + ADC1/ADC3 + circular DMA2, trigger-search thread
src/acq_algo.c   trigger, pre-trigger, min/max, sweep planner, ladders
src/dsp.c        windows and FFT via CMSIS-DSP
src/dsp_math.c   automatic measurements (Vpp, Vrms, frequency, duty)
src/gen.c        PWM-DAC on PB4 and the calibrator on PH6
src/sim.c        demo-mode signal simulator
src/ui.c         LVGL: key panel, settings popups, three render modes
src/main.c       init, two-rate UI loop
boards/          board overlay
docs/            architecture, UI spec, interactive mockup
tests/host/      algorithm tests, built on the host
```

---

## Project status

**Working and verified on hardware:** demo mode in all three screens, touch
control, phosphor highlight, settings popups, demo/live switching via B1,
flashing over the ST-LINK drive.

**Verified by calculation and host tests:** the acquisition and measurement
algorithms, the sweep planner, the pin map (UM1907), the DMA stream numbers
(RM0385).

**Not finished:**

- Live mode is not fully validated on real signals.
- ADC interleaving is not implemented, so the timebase stops at 20 µs/div;
  single-channel 5.40 MSa/s remains a placeholder.
- The PWM-DAC spectrum with an external RC filter has not been measured.
- LTDC + DMA contention on the FMC bus at fast sweeps has not been measured.
- Large-value field widths were computed for a monospace font, while large
  values are drawn in Montserrat with wider digits.
- The analog front end (protection, divider, offset, AC/DC) was not built; the
  input works over 0…3.3 V.

---

## License

MIT, see [LICENSE](LICENSE).
