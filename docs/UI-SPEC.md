# DSO-746 — Instrument UI and Logic Specification

This document is the prompt that defines how the instrument behaves. Code is
written to satisfy it; when code and this document disagree, the document wins
and the code is fixed. It distils the conventions of tinySA, Rigol DHO/DS1000Z,
Siglent SSA/SDG and Scopy into rules that fit a 480×272 touch-only panel.

---

## 1. Principles

1. **Live feedback is mandatory.** While a value is being adjusted the trace
   must stay visible and must reflect the new value on the next frame. It is
   not necessary to see the other controls while adjusting a value.
2. **One control type per parameter.** Every parameter is exactly one of:
   `LIST` (discrete choice from a fixed set, chosen from a list of large
   tiles, applied on tap), `STEP` (continuous or ladder value changed with
   − / + and drag, trace visible), `TOGGLE` (ON/OFF, flips on tap),
   `ACTION` (does something once, e.g. AUTOSET), `ARMED` (dangerous output,
   long-press to enable, tap to disable).
3. **No free numeric entry.** Users choose from lists or step a value; every
   step is clamped to the parameter's range and to the physical range of the
   hardware. An invalid combination cannot be entered.
4. **No undefined state.** Every panel has a guaranteed exit; any mode change,
   the demo/live switch, RUN/STOP and a 10 s inactivity timeout all return the
   screen to the trace. A settings panel never survives into another mode.
5. **The UI thread owns LVGL.** No `lv_*` call is ever made from an
   interrupt, from the input callback thread, or from the acquisition thread.
   Other threads set flags or post messages; the UI thread applies them.
6. **The UI never waits for data.** If acquisition delivers nothing, the UI
   keeps running and says so ("NO DATA" / "WAIT TRIG"); it never blocks.
7. **Plus always makes what you see bigger.** For scale parameters this steps
   the ladder downwards.
8. **Touch targets ≥ 46 px with ≥ 11 px gaps** everywhere, including inside
   popups and lists. Panel density is 5.05 px/mm, so 46 px ≈ 9 mm.
9. **What is shown is what acts.** A key or tile that would change nothing
   in the current state is hidden, in both demo and live: TIME and TRIG
   vanish in XY mode; a switched-off channel shows only its ON/OFF tile;
   DUTY exists only for square and pulse; PWM FREQ and PWM DUTY only while
   PWM OUTPUT is on; PALETTE only with the waterfall; − / + hide while a
   number is being typed. FFT has no channel groups, GEN has no TRIG/TIME.
   The selection always moves to something visible.

---

## 2. Screen layout (fixed)

| Area | Position | Purpose |
|---|---|---|
| Status strip | 0,0 480×22 | state, mode, trigger, sample rate, one readout — information only |
| Trace | 0,22 336×192 | waveform / spectrum / generator preview; accepts drag |
| Rail | 336,22 144×250 | 2×4 keys 61×52: RUN, MODE, six group keys |
| Bar | 0,214 336×58 | − (56×48), value block (194×48), + (56×48) |
| Popup | over the trace, 336×192 | 3×3 tiles 102×46, gaps 11 |

RUN and MODE never move. The six group keys change with the mode.

---

## 3. Modes and groups

### SCOPE
| Group | Parameters (type) |
|---|---|
| CH1 / CH2 | VOLTS/DIV (STEP, ladder 20 mV…2 V, plus enlarges), OFFSET (STEP ±4 div, drag), COUPLING (LIST DC/AC/GND), PROBE (LIST ×1/×10), BANDWIDTH (TOGGLE), INVERT (TOGGLE), CHANNEL (TOGGLE) |
| TIME | TIME/DIV (STEP, ladder 20 µs…100 ms, plus enlarges), H.POSITION (STEP ±6 div, drag), ACQUIRE (LIST NORMAL/PEAK/AVERAGE/HI-RES/ENVELOPE), AUTOSET (ACTION) |
| TRIG | TRIG LEVEL (STEP, drag), SLOPE (LIST RISE/FALL/BOTH), TRIG MODE (LIST AUTO/NORMAL/SINGLE), SOURCE (LIST CH1/CH2), HYSTERESIS (STEP 0…160 mV) |
| MEAS | MEASURE (TOGGLE) |
| DISP | PERSISTENCE (LIST OFF…INFINITE), XY MODE (TOGGLE), GRID (LIST FULL/AXES/OFF) |

### FFT
| Group | Parameters |
|---|---|
| SRC | FFT SOURCE (LIST CH1/CH2), SPAN (LIST 8/32/128/900 kHz), FFT SIZE (LIST 256…4096), WINDOW (LIST RECT/HANN/HAMMING/BLACK-H/FLATTOP) |
| SCALE | REF LEVEL (STEP −80…+20 dB, drag), dB/DIV (LIST 1/2/5/10/20, plus enlarges) |
| MARK | PEAK SEARCH (TOGGLE), SPEC AVG (LIST OFF/2/4/8/16/32) |
| WFALL | WATERFALL (TOGGLE), PALETTE (LIST) |
| CH1 / CH2 | as in SCOPE |

### GEN
| Group | Parameters |
|---|---|
| WAVE | WAVEFORM (LIST SINE/SQUARE/TRIANGLE/RAMP UP/RAMP DN/PULSE/SINC/NOISE), DUTY (STEP 1…99 %) |
| FREQ | FREQUENCY (STEP, logarithmic ×1.2, 1 Hz…42 kHz for synthesised shapes, 1 MHz for square) |
| AMPL | AMPLITUDE (STEP 0.05…3.3 Vpp), DC OFFSET (STEP 0…3.3 V) — the pair is clamped so that offset ± amplitude/2 stays inside 0…3.3 V; the edited value yields |
| PWM | PWM OUTPUT (TOGGLE), PWM FREQ (STEP 2 Hz…2 MHz), PWM DUTY (STEP) |
| TRIG, TIME | as in SCOPE (the trace area shows the live input, so the generator can be observed) |

RUN key in GEN is the ARMED output control: long-press ≥ 600 ms to enable,
tap to disable, output is OFF at power-up and after any mode change.

---

## 4. Interaction model

### 4.1 Demo mode (showcase)
Group key selects the group; the value block cycles the group's parameters;
− / + step. Nothing else. This mode is frozen by decision and is not changed.

### 4.2 Live mode (instrument)
1. **Group key** opens the group's popup: one tile per parameter with its
   current value, last tile BACK. Tapping the same group key again closes it.
2. **Tapping a tile**:
   - `LIST` → the popup switches to the option page: one tile per allowed
     value, the current one highlighted. Tapping an option applies it at once
     and returns to the parameter page. BACK returns without change.
   - `STEP` → the popup **collapses** so the trace is visible; the value
     block shows the parameter; − / + and drag on the trace adjust it with
     the trace updating live. The group key stays lit; tapping it reopens the
     parameter page.
   - `STEP` with a frequency (FREQUENCY, PWM FREQ) → a **numeric keypad**
     page: 5×3 tiles 56×46 — digits, decimal point, DEL, and the unit keys
     Hz / kHz / MHz. The typed number shows in the value block. A unit key is
     the multiplier and the apply: the value is validated against the
     parameter's range and the hardware limits (clamped, never refused),
     applied, and the panel folds back to the trace with − / + fine-tuning.
     Nothing typed → the unit key does nothing.
   - `TOGGLE` → flips immediately, popup stays.
   - `ACTION` → runs immediately, popup closes so the result is visible.
3. **Drag on the trace** (live mode, any popup closed): vertical drag moves
   the OFFSET of the channel whose group is selected, or the TRIG LEVEL when
   the TRIG group is selected, or the REF LEVEL in FFT; horizontal drag
   moves H.POSITION. Drag is direct manipulation of the same values the
   steppers change.
4. **Exit rules**: BACK, the same group key, RUN, MODE, the demo/live switch
   or 10 s without a touch close any popup. A mode change resets the
   selection to the first group.
5. **− / +** repeat while held (long-press repeat), one step per tap.

---

## 5. Validation (applied after every change, in one place)

- Ladder indices clamped to their tables; the time/div floor is the dual-
  channel floor until ADC interleaving exists.
- Offsets ±4 div, H.POSITION ±6 div, trigger level ±1.65 V, hysteresis
  0…200 LSB.
- Generator: amplitude 0.05…3.3 Vpp, offset 0…3.3 V, and
  `offset + ampl/2 ≤ 3.3`, `offset − ampl/2 ≥ 0`; the parameter being
  edited is reduced until both hold. Synthesised shapes above 42 kHz are
  clamped (10 DAC updates per period); square/pulse may go to 1 MHz.
- Enumerations never wrap: stepping past the end stays at the end.
- FFT size ≤ FRAME_MAX; span index < FFT_SPAN_CNT.

---

## 6. State machine

```
BOOT ─► DEMO/SCOPE (defaults for 1 kHz 1 V) 
B1 USER ─► toggle demo/live (UI thread applies: clear display, close popup,
           scope defaults, acquisition keeps running in both)
MODE ─► SCOPE→FFT→GEN→SCOPE: close popup, sel_group=0, clear display,
        generator output OFF, scope defaults when entering SCOPE
RUN/STOP ─► STOP freezes the last complete frame; controls stay live and
            re-render the frozen frame; RUN resumes. SINGLE stops after
            one triggered frame.
No frames for 500 ms in live ─► status "NO DATA" (AUTO) / "WAIT TRIG"
            (NORMAL/SINGLE); UI keeps its 10 ms cadence.
```

---

## 7. Threads and ownership

| Thread | Owns | Talks to UI via |
|---|---|---|
| DMA ISR | ring half flag | k_sem |
| acq (−2) | trigger search, frame slots | k_msgq of frame pointers |
| input | FT5336 / B1 events | LVGL indev (touch); atomic flag (B1) |
| ui/main | **all of LVGL**, cfg under mutex | — |

The UI loop runs `lv_timer_handler()` every 10 ms and renders a frame every
50 ms; it never sleeps on a queue.

---

## 8. Peripheral rules (STM32F7)

- ADC ring buffers in internal SRAM, `__nocache`, 32-byte aligned.
- LTDC framebuffers and LVGL buffers in SDRAM; ADC DMA never targets SDRAM.
- DMA2 and TIM3 nodes disabled in devicetree; LL owns them.
- DMA2D draw unit enabled with its interrupt; dma2d node declared.
- Only the CH1 DMA stream raises interrupts; CH2 runs in lockstep.
