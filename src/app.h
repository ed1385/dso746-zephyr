/*
 * Author:  Eduard
 * Project: DSO-746 - oscilloscope, FFT analyzer and signal generator
 * Repo:    https://github.com/ed1385/dso746-zephyr
 * License: MIT
 *
 * DSO-746 - demo oscilloscope / spectrum analyser / generator
 * Shared types. Everything the UI can change lives in dso_cfg_t; the
 * acquisition and generator modules read it under one mutex.
 */
#ifndef DSO_APP_H
#define DSO_APP_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ---- panel geometry (must match the approved mockup) ------------------- */
#define SCR_W        480
#define SCR_H        272
#define WAVE_X       0
#define WAVE_Y       22
#define WAVE_W       336          /* 12 divisions x 28 px */
#define WAVE_H       192          /*  8 divisions x 24 px */
#define DIV_X        28
#define DIV_Y        24
#define WAVE_COLS    WAVE_W

/* ---- acquisition sizes ------------------------------------------------- */
#define ADC_RING_LEN   8192       /* samples per channel, two halves.        */
/* Kept at 8192: the non-cached MPU region must be power-of-two aligned, so a
 * 16384-sample ring would reserve 128 KB of the 256 KB internal SRAM. */
#define FRAME_MAX      4096       /* samples handed to the UI/DSP per frame */
#define FRAME_SLOTS    3
#define N_CH           2

#define ADC_FULL_SCALE 4095.0f
#define ADC_VREF       3.3f

/*
 * FFT span ladder. The span is set by the sample rate: Nyquist = rate / 2.
 * 8 kHz is the demo default (1 kHz sits at 1/8 of the width); the top step
 * is the full 1.8 MSa/s of the ADC, i.e. a 900 kHz span.
 */
#define FFT_SPAN_CNT 4
extern const float fft_rate_tab[FFT_SPAN_CNT];
#define FFT_SAMPLE_RATE fft_rate_tab[0]

/* ---- modes and groups -------------------------------------------------- */
enum dso_mode { MODE_SCOPE = 0, MODE_FFT, MODE_GEN, MODE_COUNT };

/* ---- user settings ----------------------------------------------------- */
struct dso_chan {
	bool     on;
	uint8_t  vdiv_idx;      /* index into vdiv_tab[]                      */
	float    offset_div;    /* -4 .. +4 divisions                          */
	uint8_t  coupling;      /* 0 DC, 1 AC, 2 GND (GPIO on the future AFE)  */
	bool     probe_x10;
	bool     bw_limit;
	bool     invert;
};

struct dso_trig {
	float    level_v;
	uint8_t  slope;         /* 0 rising, 1 falling, 2 both                 */
	uint8_t  mode;          /* 0 auto, 1 normal, 2 single                  */
	uint8_t  source;        /* 0 CH1, 1 CH2, 2 EXT                         */
	uint8_t  hyst_lsb;      /* hysteresis in ADC LSB                       */
	uint32_t holdoff_us;
};

struct dso_fft {
	uint8_t  source;
	uint8_t  size_idx;      /* index into fft_size_tab[]                   */
	uint8_t  window;        /* 0 rect, 1 hann, 2 hamming, 3 blackman-h, 4 flat-top */
	int8_t   ref_db;
	uint8_t  db_div_idx;
	uint8_t  avg_idx;
	bool     waterfall;
	uint8_t  palette;
	bool     peak_search;
	uint8_t  span_idx;      /* index into fft_rate_tab[]                   */
};

struct dso_gen {
	uint8_t  wave;          /* 0 sine .. 8 arb                             */
	float    freq_hz;
	float    ampl_vpp;
	float    offset_v;
	float    duty_pct;
	uint16_t phase_deg;
	bool     out_on;
	bool     pwm_on;        /* the hard square on the same pin group       */
	float    pwm_freq_hz;
	float    pwm_duty_pct;
};

struct dso_cfg {
	uint8_t  mode;
	uint8_t  tdiv_idx;      /* index into tdiv_tab[]                       */
	float    hpos_div;
	uint8_t  acq_mode;      /* 0 normal, 1 peak, 2 average, 3 hi-res, 4 envelope */
	uint8_t  avg_idx;
	uint8_t  mem_idx;
	bool     running;
	bool     demo;          /* B1 USER key toggles; true at boot          */
	uint8_t  persist_idx;
	bool     xy_mode;
	uint8_t  grid_mode;
	bool     meas_on;
	struct dso_chan ch[N_CH];
	struct dso_trig trig;
	struct dso_fft  fft;
	struct dso_gen  gen;
};

/* ladders shared by the UI and the acquisition */
extern const float    vdiv_tab[];      /* volts per division      */
extern const size_t   vdiv_cnt;
extern const float    tdiv_tab[];      /* seconds per division    */
extern const size_t   tdiv_cnt;
#define TDIV_MIN_DUAL 1                /* 20 us/div: slowest index legal at 1.8 MSa/s */
size_t tdiv_min_idx(bool interleaved); /* first index the hardware can really fill    */
extern const uint16_t fft_size_tab[];
extern const size_t   fft_size_cnt;

/* ---- one captured frame ------------------------------------------------ */
struct dso_frame {
	uint16_t n;                    /* valid samples per channel            */
	uint8_t  nch;
	bool     triggered;
	float    sample_rate;
	uint16_t s[N_CH][FRAME_MAX];
};

/* ---- measurements ------------------------------------------------------ */
struct dso_meas {
	float vpp, vmax, vmin, vavg, vrms;
	float freq_hz, duty_pct;
	bool  valid;
};

/* =======================================================================
 * Hardware-free algorithms. These are compiled both into the firmware and
 * into tests/host, so they can be verified without a board.
 * ======================================================================= */

/*
 * Search a rising/falling crossing of `level` inside ring[from..to) with
 * hysteresis. `armed` carries the arming state between calls exactly like
 * the reference project does: we only re-arm after the signal has gone
 * past level -/+ hyst, so a single noisy edge cannot produce a burst of
 * triggers. Returns the ring index of the crossing or -1.
 */
int  acq_find_trigger(const uint16_t *ring, size_t ring_len,
		      size_t from, size_t to,
		      uint16_t level, uint16_t hyst, uint8_t slope,
		      bool *armed);

/*
 * Sweep planner shared by the hardware, the simulator and the tests.
 * Returns the record length for one screen (12 divisions) and the sample
 * rate to run at. The record is WAVE_COLS x k samples, k = 1..4, so that the
 * min/max reduction to one column per pixel keeps spikes at slow sweeps while
 * fast sweeps still fit the 1.8 MSa/s ceiling at one sample per column.
 */
size_t acq_plan(uint8_t tdiv_idx, float *sample_rate);

/* Pre-trigger samples for a record of n samples and a horizontal position in
 * divisions: 25 % of the record at hpos 0, shifted by hpos divisions. */
size_t acq_pretrigger(size_t n, float hpos_div);

/* Copy `n` samples starting `pre` samples before `trig`, wrapping the ring. */
void acq_extract(const uint16_t *ring, size_t ring_len,
		 size_t trig, size_t pre, uint16_t *out, size_t n);

/*
 * Reduce n samples to `cols` min/max pairs - what a real DSO does so that
 * spikes survive decimation. ymin/ymax are ADC codes.
 */
void acq_reduce_minmax(const uint16_t *src, size_t n,
		       uint16_t *ymin, uint16_t *ymax, size_t cols);

/* Measurements over one frame. lsb_volts converts an ADC code to volts. */
void dsp_measure(const uint16_t *s, size_t n, float sample_rate,
		 float lsb_volts, float zero_code, struct dso_meas *m);

/* ---- module entry points ---------------------------------------------- */
int  acq_init(void);
void acq_apply_cfg(const struct dso_cfg *cfg);   /* reprograms TIM2/ADC     */
void acq_start(void);
void acq_stop(void);
struct dso_frame *acq_take_frame(int timeout_ms); /* NULL if none           */
uint32_t acq_overruns(void);                       /* ISR ran ahead of acq   */
void acq_release_frame(struct dso_frame *f);

int  gen_init(void);
void gen_apply(const struct dso_gen *g);

int  dsp_init(void);
/* Fills `out_db` with `cols` magnitude values in dB, from the frame. */
void dsp_spectrum(const uint16_t *s, size_t n, uint8_t window,
		  float *out_db, size_t cols);

struct dso_frame *sim_frame(const struct dso_cfg *cfg);

int  ui_init(void);
void ui_on_frame(struct dso_frame *f, const struct dso_meas *m,
		 const float *spec_db);
void ui_sync(const struct dso_meas *m, float sample_rate);
void ui_clear_display(void);
void ui_request_demo_toggle(void);   /* safe from any thread            */
void ui_service(void);               /* UI thread: applies requests,    *
				      * popup timeout, no-data watchdog */
struct dso_cfg *ui_cfg(void);            /* the single settings instance    */
void ui_lock(void);
void ui_unlock(void);

#endif /* DSO_APP_H */
