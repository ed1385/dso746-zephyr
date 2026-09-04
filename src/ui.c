/*
 * DSO-746 user interface, LVGL 9.
 *
 * Geometry is the one that was approved in the HTML mockup and it is fixed:
 *   status  0,0   480x22      (information only, no touch targets)
 *   wave    0,22  336x192     (12 x 8 divisions of 28 x 24 px)
 *   rail  336,22  144x250     (2 x 4 keys of 61x52, gaps 11 px)
 *   bar     0,214 336x58      (minus 56x48, value 194x48, plus 56x48)
 * At 5.05 px/mm every key is at least 46 px and every gap at least 11 px.
 * Do not "tidy" these numbers: they are the touch specification.
 *
 * The trace is not drawn with widgets. Each channel has an 8-bit intensity
 * map; the map decays every frame (that is the phosphor), and the two maps
 * are composed into one RGB565 canvas through a colour ramp per channel.
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/linker/devicetree_regions.h>
#include <zephyr/logging/log.h>
#include <lvgl.h>
#include <string.h>
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "app.h"

LOG_MODULE_REGISTER(ui, LOG_LEVEL_INF);

#define SDRAM_SECTION __attribute__((section(LINKER_DT_NODE_REGION_NAME(DT_NODELABEL(sdram1)))))

/* ---- palette ----------------------------------------------------------
 * These are the RGB565-quantised values, so the mockup, this file and the
 * panel all show literally the same colour. Do not re-pick them by eye.
 */
#define C_GND    0x080C10
#define C_GND2   0x101821
#define C_BTN    0x182431
#define C_BTN_ON 0x21344A
#define C_LINE   0x2A3642
#define C_GRID   0x1E2833
#define C_DIM    0x94A6B5
#define C_TXT    0xEFEFF7
#define C_CH1    0xFFB200
#define C_CH2    0x00DBE7
#define C_RUN    0x31D37B
#define C_ALARM  0xFF3831

/* ---- parameter model --------------------------------------------------
 * Data driven on purpose: one table describes every control, so adding a
 * parameter cannot desynchronise the label, the ladder and the stepper.
 */
enum ptype { P_LADDER, P_ENUM, P_BOOL, P_FLOAT, P_IDX, P_ACTION };
/* how the live-mode popup treats a parameter (spec §1.2) */
enum ctl { CTL_STEP, CTL_LIST, CTL_TOGGLE, CTL_ACTION };

enum pfmt { F_RAW, F_VOLT, F_SEC, F_HZ, F_PCT, F_DB, F_DIV };

/*
 * STEPPER DIRECTION - one rule, no exceptions:
 *
 *     PLUS always makes what you see on the screen BIGGER.
 *
 * For most parameters that is the same as increasing the number: offset,
 * trigger level, amplitude, frequency. For the scale parameters it is the
 * opposite - a larger volts-per-division squeezes the trace, so the plus key
 * has to step the ladder DOWN. Those carry zoom = true. Getting this wrong is
 * what made the trace grow on minus.
 */
struct param {
	const char *label;
	uint8_t type;
	uint8_t fmt;
	void *field;                 /* uint8_t*, bool* or float*            */
	const void *tab;             /* const float* or const char* const*   */
	uint8_t cnt;
	float min, max, step;
	bool zoom;                   /* plus steps this parameter downwards  */
	void (*action)(void);        /* P_ACTION only                        */
};

static enum ctl param_ctl(const struct param *p)
{
	switch (p->type) {
	case P_ENUM:   return CTL_LIST;
	case P_BOOL:   return CTL_TOGGLE;
	case P_ACTION: return CTL_ACTION;
	default:       return CTL_STEP;      /* ladders and floats: live trace */
	}
}

struct group {
	const char *name;
	uint32_t color;
	const struct param *p;
	uint8_t np;
};

static struct dso_cfg cfg;
static K_MUTEX_DEFINE(cfg_mtx);

struct dso_cfg *ui_cfg(void) { return &cfg; }
void ui_lock(void)   { k_mutex_lock(&cfg_mtx, K_FOREVER); }
void ui_unlock(void) { k_mutex_unlock(&cfg_mtx); }

static const char *const nm_coupling[] = { "DC", "AC", "GND" };
static const char *const nm_slope[]    = { "RISE", "FALL", "BOTH" };
static const char *const nm_tmode[]    = { "AUTO", "NORMAL", "SINGLE" };
static const char *const nm_tsrc[]     = { "CH1", "CH2", "EXT" };
static const char *const nm_acq[]      = { "NORMAL", "PEAK DET", "AVERAGE",
					   "HI-RES", "ENVELOPE" };
static const char *const nm_pers[]     = { "OFF", "100 ms", "500 ms", "1 s",
					   "5 s", "INFINITE" };
static const char *const nm_grid[]     = { "FULL", "AXES", "OFF" };
static const char *const nm_win[]      = { "RECT", "HANN", "HAMMING",
					   "BLACK-H", "FLATTOP" };
static const char *const nm_pal[]      = { "INFERNO", "VIRIDIS", "GRAY" };
static const char *const nm_fftsz[]    = { "256 pts", "512 pts", "1024pts",
					   "2048pts", "4096pts" };
static const char *const nm_dbdiv[]    = { "1 dB", "2 dB", "5 dB", "10 dB",
					   "20 dB" };
static const char *const nm_avg[]      = { "OFF", "2", "4", "8", "16", "32" };
static const char *const nm_span[]     = { "8 kHz", "32 kHz", "128 kHz", "900 kHz" };
static const char *const nm_wave[]     = { "SINE", "SQUARE", "TRIANGLE",
					   "RAMP UP", "RAMP DN", "PULSE",
					   "SINC", "NOISE", "ARB" };
static const char *const nm_wave_s[]   = { "SIN", "SQR", "TRI", "RMP+", "RMP-",
					   "PULSE", "SINC", "NOISE", "ARB" };

#define CH_PARAMS(i) \
	{ "VOLTS/DIV", P_LADDER, F_VOLT, &cfg.ch[i].vdiv_idx, vdiv_tab, 0, 0, 0, 0, true }, \
	{ "OFFSET",    P_FLOAT,  F_DIV,  &cfg.ch[i].offset_div, NULL, 0, -4, 4, 0.1f }, \
	{ "COUPLING",  P_ENUM,   F_RAW,  &cfg.ch[i].coupling, nm_coupling, 3, 0, 0, 0 }, \
	{ "PROBE",     P_BOOL,   F_RAW,  &cfg.ch[i].probe_x10, NULL, 0, 0, 0, 0 }, \
	{ "BANDWIDTH", P_BOOL,   F_RAW,  &cfg.ch[i].bw_limit, NULL, 0, 0, 0, 0 }, \
	{ "INVERT",    P_BOOL,   F_RAW,  &cfg.ch[i].invert, NULL, 0, 0, 0, 0 }, \
	{ "CHANNEL",   P_BOOL,   F_RAW,  &cfg.ch[i].on, NULL, 0, 0, 0, 0 }

static const struct param p_ch1[] = { CH_PARAMS(0) };
static const struct param p_ch2[] = { CH_PARAMS(1) };

static void autoset_action(void);
static const struct param p_time[] = {
	{ "AUTOSET",   P_ACTION, F_RAW, NULL, NULL, 0, 0, 0, 0, false, autoset_action },
	{ "TIME/DIV",  P_LADDER, F_SEC, &cfg.tdiv_idx, tdiv_tab, 0, 0, 0, 0, true },
	{ "H.POSITION", P_FLOAT, F_DIV, &cfg.hpos_div, NULL, 0, -6, 6, 0.2f },
	{ "ACQUIRE",   P_ENUM,   F_RAW, &cfg.acq_mode, nm_acq, 5, 0, 0, 0 },
};
static const struct param p_trig[] = {
	{ "TRIG LEVEL", P_FLOAT, F_VOLT, &cfg.trig.level_v, NULL, 0, -1.65f, 1.65f, 0.05f },
	{ "SLOPE",      P_ENUM,  F_RAW,  &cfg.trig.slope, nm_slope, 3, 0, 0, 0 },
	{ "TRIG MODE",  P_ENUM,  F_RAW,  &cfg.trig.mode, nm_tmode, 3, 0, 0, 0 },
	{ "SOURCE",     P_ENUM,  F_RAW,  &cfg.trig.source, nm_tsrc, 3, 0, 0, 0 },
	{ "HYSTERESIS", P_IDX,   F_VOLT, &cfg.trig.hyst_lsb, NULL, 0, 0, 200, 10 },
};
static const struct param p_meas[] = {
	{ "MEASURE",   P_BOOL, F_RAW, &cfg.meas_on, NULL, 0, 0, 0, 0 },
};
static const struct param p_disp[] = {
	{ "PERSISTENCE", P_ENUM, F_RAW, &cfg.persist_idx, nm_pers, 6, 0, 0, 0 },
	{ "XY MODE",     P_BOOL, F_RAW, &cfg.xy_mode, NULL, 0, 0, 0, 0 },
	{ "GRID",        P_ENUM, F_RAW, &cfg.grid_mode, nm_grid, 3, 0, 0, 0 },
};

static const struct param p_fsrc[] = {
	{ "FFT SOURCE", P_ENUM, F_RAW, &cfg.fft.source, nm_tsrc, 2, 0, 0, 0 },
	{ "SPAN",       P_ENUM, F_RAW, &cfg.fft.span_idx, nm_span, FFT_SPAN_CNT, 0, 0, 0 },
	{ "FFT SIZE",   P_ENUM, F_RAW, &cfg.fft.size_idx, nm_fftsz, 5, 0, 0, 0 },
	{ "WINDOW",     P_ENUM, F_RAW, &cfg.fft.window, nm_win, 5, 0, 0, 0 },
};
static const struct param p_fscale[] = {
	{ "REF LEVEL", P_FLOAT, F_DB, NULL, NULL, 0, -80, 20, 5 },
	{ "dB / DIV",  P_ENUM,  F_RAW, &cfg.fft.db_div_idx, nm_dbdiv, 5, 0, 0, 0, true },
};
static const struct param p_fmark[] = {
	{ "PEAK SEARCH", P_BOOL, F_RAW, &cfg.fft.peak_search, NULL, 0, 0, 0, 0 },
	{ "SPEC AVG",    P_ENUM, F_RAW, &cfg.fft.avg_idx, nm_avg, 6, 0, 0, 0 },
};
static const struct param p_fwf[] = {
	{ "WATERFALL", P_BOOL, F_RAW, &cfg.fft.waterfall, NULL, 0, 0, 0, 0 },
	{ "PALETTE",   P_ENUM, F_RAW, &cfg.fft.palette, nm_pal, 3, 0, 0, 0 },
};

static const struct param p_gwave[] = {
	{ "WAVEFORM", P_ENUM,  F_RAW, &cfg.gen.wave, nm_wave, 8, 0, 0, 0 },
	{ "DUTY",     P_FLOAT, F_PCT, &cfg.gen.duty_pct, NULL, 0, 1, 99, 0.5f },
};
static const struct param p_gfreq[] = {
	{ "FREQUENCY", P_FLOAT, F_HZ, &cfg.gen.freq_hz, NULL, 0, 1, 200000, 0 },
};
static const struct param p_gampl[] = {
	{ "AMPLITUDE", P_FLOAT, F_VOLT, &cfg.gen.ampl_vpp, NULL, 0, 0.05f, 3.0f, 0.05f },
	{ "DC OFFSET", P_FLOAT, F_VOLT, &cfg.gen.offset_v, NULL, 0, 0.0f, 3.0f, 0.05f },
};
static const struct param p_gpwm[] = {
	{ "PWM OUTPUT", P_BOOL,  F_RAW, &cfg.gen.pwm_on, NULL, 0, 0, 0, 0 },
	{ "PWM FREQ",   P_FLOAT, F_HZ,  &cfg.gen.pwm_freq_hz, NULL, 0, 1, 2000000, 0 },
	{ "PWM DUTY",   P_FLOAT, F_PCT, &cfg.gen.pwm_duty_pct, NULL, 0, 0, 100, 0.5f },
};

#define G(n, c, arr) { n, c, arr, sizeof(arr) / sizeof(arr[0]) }
static const struct group groups[MODE_COUNT][6] = {
	{ G("CH1", C_CH1, p_ch1), G("CH2", C_CH2, p_ch2),
	  G("TIME", C_LINE, p_time), G("TRIG", C_CH1, p_trig),
	  G("MEAS", C_LINE, p_meas), G("DISP", C_LINE, p_disp) },
	{ G("SRC", C_CH2, p_fsrc), G("SCALE", C_LINE, p_fscale),
	  G("MARK", C_CH1, p_fmark), G("WFALL", C_LINE, p_fwf),
	  G("CH1", C_CH1, p_ch1), G("CH2", C_CH2, p_ch2) },
	{ G("WAVE", C_CH2, p_gwave), G("FREQ", C_CH2, p_gfreq),
	  G("AMPL", C_CH2, p_gampl), G("PWM", C_CH1, p_gpwm),
	  G("TRIG", C_CH1, p_trig), G("TIME", C_LINE, p_time) },
};
/* demo shows the showcase set; live exposes every group of the instrument */
static const uint8_t group_cnt_demo[MODE_COUNT] = { 6, 4, 4 };
static const uint8_t group_cnt_live[MODE_COUNT] = { 6, 6, 4 };
#define group_cnt (cfg.demo ? group_cnt_demo : group_cnt_live)

static uint8_t sel_group, sel_param;

/*
 * THE FREEZE THIS FIXES
 * ---------------------
 * Every widget used to be rewritten 20 times a second. lv_label_set_text()
 * frees and re-allocates the label's text buffer on every call, so ~24 labels
 * at 20 Hz meant ~500 allocations per second out of a 64 KB LVGL pool. The
 * pool fragmented, an allocation eventually failed, and LVGL's malloc assert
 * spins forever - the UI thread dies, touch stops responding, while the
 * acquisition thread happily keeps running. That is exactly the symptom:
 * press a couple of keys, everything locks up.
 *
 * From here on nothing is written to a widget unless its value actually
 * changed. Static controls are only touched when the user changes something;
 * the waveform area, which really does change every frame, is a raw canvas
 * buffer we fill ourselves and invalidate once.
 */
struct ui_cache {
	char key_name[8][12];
	char key_val[8][12];
	char sf[5][20];
	char par[20], val[20], sub[36];
	uint8_t mode, group, param, np;
	bool hot, demo, meas, banner;
	uint32_t dot_color;
};
static struct ui_cache uc;

/* returns true if the widget was actually rewritten */
static bool set_text(lv_obj_t *o, char *cache, size_t cap, const char *txt)
{
	if (strncmp(cache, txt, cap - 1) == 0) {
		return false;
	}
	strncpy(cache, txt, cap - 1);
	cache[cap - 1] = '\0';
	lv_label_set_text(o, txt);
	return true;
}

/* ---- widgets ----------------------------------------------------------- */
static lv_obj_t *lbl_state, *lbl_a, *lbl_b, *lbl_c, *lbl_d, *dot;
static lv_obj_t *canvas, *key[8], *key_name[8], *key_val[8];
static lv_obj_t *vb, *lbl_par, *lbl_val, *lbl_sub, *dots[8];
static lv_obj_t *banner;

/*
 * LIVE-MODE SETTINGS POPUP
 * ------------------------
 * In demo mode the rail keys select a group and the value block cycles its
 * parameters - that is the showcase face and it stays exactly as it is.
 *
 * A working instrument has too many settings for that. In live mode a group
 * key opens a panel that covers the waveform area: a 3 x 3 grid of tiles,
 * 102 x 46 px each with 11 px gaps (both above the touch minimum), one tile
 * per parameter, the last tile is DONE. Tapping a tile selects it - the value
 * block and the -/+ keys then act on it; tapping an ON/OFF tile toggles it
 * directly. Tapping the same group key again, or DONE, closes the panel and
 * the trace comes back. Acquisition never stops underneath.
 */
#define TILE_COLS 3
#define TILE_ROWS 3
#define TILE_W    102
#define TILE_H    46
#define TILE_GAP  11
#define TILE_N    (TILE_COLS * TILE_ROWS)
#define TILE_DONE (TILE_N - 1)
static lv_obj_t *popup, *tile[TILE_N], *tile_lab[TILE_N], *tile_val[TILE_N];
static bool popup_open;
static char tile_cache[TILE_N][16];
static uint32_t last_frame_ms;
static lv_style_t st_key, st_key_on, st_key_pr, st_vb;
static lv_style_transition_dsc_t tr_ignite, tr_decay;

/* ---- drawing buffers (SDRAM: 129 + 126 = 255 KB of the 8 MB) ------------ */
static uint16_t cbuf[WAVE_W * WAVE_H] SDRAM_SECTION __aligned(32);
static uint8_t  phos[N_CH][WAVE_W * WAVE_H] SDRAM_SECTION;
static uint8_t  wfall[WAVE_H / 2][WAVE_W] SDRAM_SECTION;
static uint16_t ramp[N_CH][256];
static float    spec_db[WAVE_COLS];

static inline uint16_t rgb565(uint32_t c)
{
	return (uint16_t)(((c >> 19) & 0x1F) << 11 | ((c >> 10) & 0x3F) << 5 |
			  ((c >> 3) & 0x1F));
}

static void build_ramps(void)
{
	static const uint32_t base[N_CH] = { C_CH1, C_CH2 };

	for (int ch = 0; ch < N_CH; ch++) {
		for (int i = 0; i < 256; i++) {
			uint32_t r = ((base[ch] >> 16) & 0xFF) * i / 255;
			uint32_t g = ((base[ch] >> 8) & 0xFF) * i / 255;
			uint32_t b = (base[ch] & 0xFF) * i / 255;

			ramp[ch][i] = rgb565(r << 16 | g << 8 | b);
		}
	}
}

/* ---- value formatting -------------------------------------------------- */
static void fmt_eng(char *out, size_t n, float v, const char *unit)
{
	if (fabsf(v) >= 1e6f) {
		snprintf(out, n, "%.3f M%s", (double)(v / 1e6f), unit);
	} else if (fabsf(v) >= 1e3f) {
		snprintf(out, n, "%.3f k%s", (double)(v / 1e3f), unit);
	} else if (fabsf(v) >= 1.0f || v == 0.0f) {
		snprintf(out, n, "%.2f %s", (double)v, unit);
	} else {
		snprintf(out, n, "%.0f m%s", (double)(v * 1e3f), unit);
	}
}

static void fmt_sec(char *out, size_t n, float t)
{
	if (t < 1e-3f) {
		snprintf(out, n, "%.0f us", (double)(t * 1e6f));
	} else if (t < 1.0f) {
		snprintf(out, n, "%.0f ms", (double)(t * 1e3f));
	} else {
		snprintf(out, n, "%.1f s", (double)t);
	}
}

static void param_text(const struct param *p, char *out, size_t n)
{
	switch (p->type) {
	case P_LADDER: {
		uint8_t i = *(uint8_t *)p->field;
		float v = ((const float *)p->tab)[i];

		if (p->fmt == F_SEC) {
			fmt_sec(out, n, v);
		} else {
			fmt_eng(out, n, v, "V");
		}
		break;
	}
	case P_ENUM:
		snprintf(out, n, "%s", ((const char *const *)p->tab)[*(uint8_t *)p->field]);
		break;
	case P_BOOL:
		snprintf(out, n, "%s", *(bool *)p->field ? "ON" : "OFF");
		break;
	case P_ACTION:
		snprintf(out, n, "RUN");
		break;
	case P_IDX:
		if (!p->field) {
			snprintf(out, n, "-");
			break;
		}
		if (p->fmt == F_VOLT) {      /* hysteresis: LSB shown as mV */
			snprintf(out, n, "%.0f mV",
				 (double)(*(uint8_t *)p->field *
					  (ADC_VREF / ADC_FULL_SCALE) * 1000.0f));
		} else {
			snprintf(out, n, "%u", (unsigned)*(uint8_t *)p->field);
		}
		break;
	case P_FLOAT: {
		float v = p->field ? *(float *)p->field : 0.0f;

		if (p->fmt == F_VOLT) {
			fmt_eng(out, n, v, "V");
		} else if (p->fmt == F_HZ) {
			fmt_eng(out, n, v, "Hz");
		} else if (p->fmt == F_PCT) {
			snprintf(out, n, "%.1f %%", (double)v);
		} else if (p->fmt == F_DB) {
			snprintf(out, n, "%d dB", (int)cfg.fft.ref_db);
		} else {
			snprintf(out, n, "%.2f div", (double)v);
		}
		break;
	}
	default:
		snprintf(out, n, "-");
	}
}

/*
 * Every change goes through here afterwards. Nothing else clamps; if a rule
 * is missing it is added here, not in a handler.
 */
static void validate_cfg(const struct param *edited)
{
	if (cfg.tdiv_idx >= tdiv_cnt) cfg.tdiv_idx = (uint8_t)(tdiv_cnt - 1);
	if (cfg.tdiv_idx < TDIV_MIN_DUAL) cfg.tdiv_idx = TDIV_MIN_DUAL;
	if (cfg.hpos_div < -6.0f) cfg.hpos_div = -6.0f;
	if (cfg.hpos_div > 6.0f) cfg.hpos_div = 6.0f;
	for (int i = 0; i < N_CH; i++) {
		if (cfg.ch[i].vdiv_idx >= vdiv_cnt) cfg.ch[i].vdiv_idx = (uint8_t)(vdiv_cnt - 1);
		if (cfg.ch[i].offset_div < -4.0f) cfg.ch[i].offset_div = -4.0f;
		if (cfg.ch[i].offset_div > 4.0f) cfg.ch[i].offset_div = 4.0f;
		if (cfg.ch[i].coupling > 2) cfg.ch[i].coupling = 2;
	}
	if (cfg.trig.level_v < -1.65f) cfg.trig.level_v = -1.65f;
	if (cfg.trig.level_v > 1.65f) cfg.trig.level_v = 1.65f;
	if (cfg.trig.hyst_lsb > 200) cfg.trig.hyst_lsb = 200;
	if (cfg.trig.source > 1) cfg.trig.source = 1;        /* EXT not wired yet */
	if (cfg.trig.slope > 2) cfg.trig.slope = 2;
	if (cfg.trig.mode > 2) cfg.trig.mode = 2;
	if (cfg.fft.size_idx >= fft_size_cnt) cfg.fft.size_idx = (uint8_t)(fft_size_cnt - 1);
	if (cfg.fft.span_idx >= FFT_SPAN_CNT) cfg.fft.span_idx = FFT_SPAN_CNT - 1;
	if (cfg.fft.window > 4) cfg.fft.window = 4;
	if (cfg.fft.db_div_idx > 4) cfg.fft.db_div_idx = 4;
	if (cfg.fft.ref_db < -80) cfg.fft.ref_db = -80;
	if (cfg.fft.ref_db > 20) cfg.fft.ref_db = 20;
	if (cfg.fft.source > 1) cfg.fft.source = 1;

	/* generator: the DAC window is 0..3.3 V and the pair must fit in it;
	 * the value being edited is the one that yields */
	if (cfg.gen.ampl_vpp < 0.05f) cfg.gen.ampl_vpp = 0.05f;
	if (cfg.gen.ampl_vpp > ADC_VREF) cfg.gen.ampl_vpp = ADC_VREF;
	if (cfg.gen.offset_v < 0.0f) cfg.gen.offset_v = 0.0f;
	if (cfg.gen.offset_v > ADC_VREF) cfg.gen.offset_v = ADC_VREF;
	bool editing_offset = edited && edited->field == &cfg.gen.offset_v;

	if (cfg.gen.offset_v + cfg.gen.ampl_vpp / 2.0f > ADC_VREF) {
		if (editing_offset) cfg.gen.offset_v = ADC_VREF - cfg.gen.ampl_vpp / 2.0f;
		else cfg.gen.ampl_vpp = 2.0f * (ADC_VREF - cfg.gen.offset_v);
	}
	if (cfg.gen.offset_v - cfg.gen.ampl_vpp / 2.0f < 0.0f) {
		if (editing_offset) cfg.gen.offset_v = cfg.gen.ampl_vpp / 2.0f;
		else cfg.gen.ampl_vpp = 2.0f * cfg.gen.offset_v;
	}
	if (cfg.gen.ampl_vpp < 0.05f) cfg.gen.ampl_vpp = 0.05f;

	/* synthesised shapes need 10 DAC updates per period: 42 kHz ceiling;
	 * square and pulse are plain PWM and may go to 1 MHz */
	float fmax = (cfg.gen.wave == 1 || cfg.gen.wave == 5) ? 1.0e6f : 42000.0f;

	if (cfg.gen.freq_hz > fmax) cfg.gen.freq_hz = fmax;
	if (cfg.gen.freq_hz < 1.0f) cfg.gen.freq_hz = 1.0f;
	if (cfg.gen.duty_pct < 1.0f) cfg.gen.duty_pct = 1.0f;
	if (cfg.gen.duty_pct > 99.0f) cfg.gen.duty_pct = 99.0f;
	if (cfg.gen.pwm_freq_hz < 2.0f) cfg.gen.pwm_freq_hz = 2.0f;
	if (cfg.gen.pwm_freq_hz > 2.0e6f) cfg.gen.pwm_freq_hz = 2.0e6f;
	if (cfg.gen.pwm_duty_pct < 0.0f) cfg.gen.pwm_duty_pct = 0.0f;
	if (cfg.gen.pwm_duty_pct > 100.0f) cfg.gen.pwm_duty_pct = 100.0f;
	if (cfg.gen.wave > 7) cfg.gen.wave = 7;               /* ARB not implemented */
}

static void param_step(const struct param *p, int d)
{
	switch (p->type) {
	case P_LADDER: {
		uint8_t *i = (uint8_t *)p->field;
		uint8_t cnt = (p->tab == (const void *)tdiv_tab) ? (uint8_t)tdiv_cnt
								: (uint8_t)vdiv_cnt;
		uint8_t lo = 0;

		if (p->tab == (const void *)tdiv_tab) {
			/* no ADC interleaving yet: a faster sweep than the dual
			 * floor would be drawn with a silently wrong time scale */
			lo = TDIV_MIN_DUAL;
		}
		int v = *i + d;

		*i = (uint8_t)(v < lo ? lo : (v >= cnt ? cnt - 1 : v));
		break;
	}
	case P_ENUM: {
		uint8_t *v = (uint8_t *)p->field;
		int nv = *v + d;

		/* clamp, never wrap: a stepper that jumps from the last item
		 * back to the first looks like a misfire */
		*v = (uint8_t)(nv < 0 ? 0 : (nv >= p->cnt ? p->cnt - 1 : nv));
		break;
	}
	case P_BOOL: {
		bool *b = (bool *)p->field;

		*b = !*b;
		break;
	}
	case P_IDX: {
		if (!p->field) {
			break;
		}
		uint8_t *v = (uint8_t *)p->field;
		int nv = *v + d * (int)p->step;

		*v = (uint8_t)(nv < (int)p->min ? (int)p->min
					       : (nv > (int)p->max ? (int)p->max : nv));
		break;
	}
	case P_FLOAT: {
		if (p->fmt == F_DB) {
			int nv = cfg.fft.ref_db + d * 5;

			cfg.fft.ref_db = (int8_t)(nv < -80 ? -80 : (nv > 20 ? 20 : nv));
			break;
		}
		if (!p->field) {
			break;
		}
		float *f = (float *)p->field;
		float nv;

		if (p->fmt == F_HZ) {
			nv = d > 0 ? *f * 1.2f : *f / 1.2f;   /* logarithmic */
		} else {
			nv = *f + d * p->step;
		}
		*f = nv < p->min ? p->min : (nv > p->max ? p->max : nv);
		break;
	}
	case P_ACTION:
		if (p->action) {
			p->action();
		}
		break;
	default:
		break;
	}
	validate_cfg(p);
}

/* set an enum directly from a list tile */
static void param_set_enum(const struct param *p, uint8_t v)
{
	if (p->type == P_ENUM && p->field && v < p->cnt) {
		*(uint8_t *)p->field = v;
	}
	validate_cfg(p);
}

/* ---- canvas rendering -------------------------------------------------- */
static void canvas_grid(void)
{
	uint16_t bg = rgb565(C_GND), gl = rgb565(C_GRID), ax = rgb565(C_LINE);

	for (int i = 0; i < WAVE_W * WAVE_H; i++) {
		cbuf[i] = bg;
	}
	if (cfg.grid_mode == 2) {
		return;
	}
	if (cfg.grid_mode == 0) {
		for (int x = DIV_X; x < WAVE_W; x += DIV_X) {
			for (int y = 0; y < WAVE_H; y++) {
				cbuf[y * WAVE_W + x] = gl;
			}
		}
		for (int y = DIV_Y; y < WAVE_H; y += DIV_Y) {
			for (int x = 0; x < WAVE_W; x++) {
				cbuf[y * WAVE_W + x] = gl;
			}
		}
	}
	for (int x = 0; x < WAVE_W; x++) {
		cbuf[(WAVE_H / 2) * WAVE_W + x] = ax;
	}
	for (int y = 0; y < WAVE_H; y++) {
		cbuf[y * WAVE_W + WAVE_W / 2] = ax;
	}
}

/* decay factor per frame for each persistence setting, applied at 20 Hz */
static const uint8_t decay_tab[6] = { 255, 120, 60, 32, 8, 0 };

static void phos_decay(void)
{
	uint8_t d = decay_tab[cfg.persist_idx < 6 ? cfg.persist_idx : 0];

	if (d == 0) {
		return;                        /* infinite persistence */
	}
	for (int ch = 0; ch < N_CH; ch++) {
		uint8_t *p = phos[ch];

		for (int i = 0; i < WAVE_W * WAVE_H; i++) {
			p[i] = (p[i] > d) ? (uint8_t)(p[i] - d) : 0;
		}
	}
}

static void phos_column(uint8_t ch, int x, int y0, int y1)
{
	if (y0 > y1) {
		int t = y0; y0 = y1; y1 = t;
	}
	if (y1 < 0 || y0 >= WAVE_H) {
		return;
	}
	if (y0 < 0) {
		y0 = 0;
	}
	if (y1 >= WAVE_H) {
		y1 = WAVE_H - 1;
	}
	uint8_t *p = phos[ch];

	for (int y = y0; y <= y1; y++) {
		p[y * WAVE_W + x] = 255;
	}
}

static void phos_compose(void)
{
	for (int i = 0; i < WAVE_W * WAVE_H; i++) {
		uint8_t a = phos[0][i], b = phos[1][i];

		if (a | b) {
			cbuf[i] = (a >= b) ? ramp[0][a] : ramp[1][b];
		}
	}
}

/*
 * Every drawing buffer lives in SDRAM, in a NOLOAD section: after power-on it
 * holds whatever the chip felt like, and the waterfall scrolls that garbage
 * across the screen for four seconds before real lines push it out. So the
 * buffers are cleared at start-up and whenever a mode is entered.
 */
static void clear_buffers(void)
{
	memset(phos, 0, sizeof(phos));
	memset(wfall, 0, sizeof(wfall));
	for (int i = 0; i < WAVE_COLS; i++) {
		spec_db[i] = -140.0f;      /* an empty spectrum is a noise floor,
					    * not a full-scale bar */
	}
	canvas_grid();
}

static int code_to_y(uint16_t code, const struct dso_chan *c)
{
	float lsb = ADC_VREF / ADC_FULL_SCALE;
	float volts = ((float)code - 2048.0f) * lsb * (c->probe_x10 ? 10.0f : 1.0f);

	if (c->invert) {
		volts = -volts;
	}
	float div = volts / vdiv_tab[c->vdiv_idx] + c->offset_div;

	return WAVE_H / 2 - (int)(div * DIV_Y);
}

static void render_scope(struct dso_frame *f)
{
	static uint16_t mn[WAVE_COLS], mx[WAVE_COLS];

	phos_decay();
	for (uint8_t ch = 0; ch < f->nch; ch++) {
		if (!cfg.ch[ch].on) {
			continue;
		}
		acq_reduce_minmax(f->s[ch], f->n, mn, mx, WAVE_COLS);
		int prev = code_to_y(mx[0], &cfg.ch[ch]);

		for (int x = 0; x < WAVE_COLS; x++) {
			int ytop = code_to_y(mx[x], &cfg.ch[ch]);
			int ybot = code_to_y(mn[x], &cfg.ch[ch]);

			/* join to the previous column, otherwise a fast edge
			 * becomes two dots with a gap between them */
			if (prev < ytop) {
				ytop = prev;
			}
			if (prev > ybot) {
				ybot = prev;
			}
			phos_column(ch, x, ytop, ybot);
			prev = code_to_y((uint16_t)((mn[x] + mx[x]) / 2), &cfg.ch[ch]);
		}
	}
	canvas_grid();
	phos_compose();

	/* trigger level marker on the right edge, ground marker on the left */
	int ty = code_to_y((uint16_t)(2048.0f + cfg.trig.level_v /
				      (ADC_VREF / ADC_FULL_SCALE)),
			   &cfg.ch[0]);

	if (ty >= 2 && ty < WAVE_H - 2) {
		for (int i = 0; i < 7; i++) {
			cbuf[ty * WAVE_W + WAVE_W - 1 - i] = rgb565(C_CH1);
		}
	}
	for (uint8_t ch = 0; ch < N_CH; ch++) {
		int gy = WAVE_H / 2 - (int)(cfg.ch[ch].offset_div * DIV_Y);

		if (gy >= 0 && gy < WAVE_H) {
			for (int i = 0; i < 6; i++) {
				cbuf[gy * WAVE_W + i] = rgb565(ch ? C_CH2 : C_CH1);
			}
		}
	}
}

/* XY: CH1 is the horizontal deflection, CH2 the vertical one */
static void render_xy(struct dso_frame *f)
{
	phos_decay();
	if (f->nch >= 2) {
		for (uint16_t i = 0; i < f->n; i++) {
			int x = (WAVE_W / 2) +
				(int)(((float)f->s[0][i] - 2048.0f) * WAVE_W / 4096.0f * 0.9f);
			int y = code_to_y(f->s[1][i], &cfg.ch[1]);

			if (x >= 0 && x < WAVE_W && y >= 0 && y < WAVE_H) {
				phos[0][y * WAVE_W + x] = 255;
			}
		}
	}
	canvas_grid();
	phos_compose();
}

static uint16_t heat(uint8_t v, uint8_t pal)
{
	uint32_t r, g, b;

	switch (pal) {
	case 2: r = g = b = v; break;
	case 1: r = v / 4; g = 40 + v * 200 / 255; b = 90 + (255 - v) * 60 / 255; break;
	default:
		r = v < 160 ? v * 255 / 160 : 255;
		g = v > 90 ? (v - 90) * 200 / 165 : 0;
		b = v > 190 ? (v - 190) * 120 / 65 : 0;
		break;
	}
	return rgb565(r << 16 | g << 8 | b);
}

static void render_fft(void)
{
	int sh = cfg.fft.waterfall ? 104 : WAVE_H;
	float top = (float)cfg.fft.ref_db;
	static const float dbdiv_tab[5] = { 1, 2, 5, 10, 20 };
	float span = dbdiv_tab[cfg.fft.db_div_idx] * 8.0f;

	canvas_grid();

	uint16_t col = rgb565(C_CH2);

	for (int x = 0; x < WAVE_COLS; x++) {
		float rel = (top - spec_db[x]) / span;

		if (rel < 0.0f) {
			rel = 0.0f;
		}
		if (rel > 1.0f) {
			rel = 1.0f;
		}
		int y = (int)(rel * (sh - 2));

		for (int yy = y; yy < sh; yy++) {
			cbuf[yy * WAVE_W + x] = (yy < y + 2) ? col : rgb565(0x003038);
		}
		if (cfg.fft.waterfall) {
			int v = (int)((1.0f - rel) * 255.0f);

			wfall[0][x] = (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v));
		}
	}

	if (cfg.fft.waterfall) {
		/* scroll one line down, newest on top */
		memmove(&wfall[1][0], &wfall[0][0],
			(size_t)(WAVE_H / 2 - 1) * WAVE_W);
		for (int y = 0; y < WAVE_H / 2 - 1; y++) {
			int dy = sh + 4 + y;

			if (dy >= WAVE_H) {
				break;
			}
			for (int x = 0; x < WAVE_W; x++) {
				cbuf[dy * WAVE_W + x] = heat(wfall[y][x], cfg.fft.palette);
			}
		}
	}
}

static void render_gen(void)
{
	canvas_grid();

	int h = 104;
	float half = cfg.gen.ampl_vpp / 2.0f;
	uint16_t col = rgb565(cfg.gen.out_on ? C_CH2 : C_DIM);
	int prev = -1;

	for (int x = 0; x < WAVE_W; x++) {
		float ph = fmodf((float)x * 3.0f / WAVE_W, 1.0f);
		float s;

		switch (cfg.gen.wave) {
		case 1: s = ph < cfg.gen.duty_pct / 100.0f ? 1.0f : -1.0f; break;
		case 2: s = 2.0f * fabsf(2.0f * ph - 1.0f) - 1.0f; break;
		case 3: s = 2.0f * ph - 1.0f; break;
		case 4: s = 1.0f - 2.0f * ph; break;
		default: s = sinf(2.0f * (float)M_PI * ph); break;
		}
		float v = cfg.gen.offset_v + half * s;
		int y = (int)((1.0f - v / ADC_VREF) * h);

		if (y < 0) {
			y = 0;
		}
		if (y >= h) {
			y = h - 1;
		}
		int a = prev < 0 ? y : (prev < y ? prev : y);
		int b = prev < 0 ? y : (prev < y ? y : prev);

		for (int yy = a; yy <= b; yy++) {
			cbuf[yy * WAVE_W + x] = col;
		}
		prev = y;
	}
}

/* ---- status strip ------------------------------------------------------ */
static void set_dot(uint32_t c)
{
	if (uc.dot_color != c) {
		uc.dot_color = c;
		lv_obj_set_style_bg_color(dot, lv_color_hex(c), 0);
	}
}

static void update_status(const struct dso_meas *m, float sr)
{
	char t[40];

	if (cfg.mode == MODE_GEN) {
		set_text(lbl_state, uc.sf[0], 20, cfg.demo ? "DEMO"
				: (cfg.gen.out_on ? "OUT" : "IDLE"));
		set_dot(cfg.demo ? C_CH1 : (cfg.gen.out_on ? C_CH2 : C_DIM));
		snprintf(t, sizeof(t), "%s", nm_wave[cfg.gen.wave]);
		set_text(lbl_a, uc.sf[1], 20, t);
		fmt_eng(t, sizeof(t), cfg.gen.freq_hz, "Hz");
		set_text(lbl_b, uc.sf[2], 20, t);
		snprintf(t, sizeof(t), "%.2f Vpp", (double)cfg.gen.ampl_vpp);
		set_text(lbl_c, uc.sf[3], 20, t);
		set_text(lbl_d, uc.sf[4], 20, cfg.gen.pwm_on ? "PWM ON" : "PWM OFF");
		return;
	}

	set_text(lbl_state, uc.sf[0], 20, cfg.demo ? "DEMO"
			: (cfg.running ? "RUN" : "STOP"));
	set_dot(cfg.demo ? C_CH1 : (cfg.running ? C_RUN : C_ALARM));

	if (cfg.mode == MODE_FFT) {
		snprintf(t, sizeof(t), "FFT %s %s", nm_tsrc[cfg.fft.source],
			 nm_win[cfg.fft.window]);
		set_text(lbl_a, uc.sf[1], 20, t);
		snprintf(t, sizeof(t), "%u pts", fft_size_tab[cfg.fft.size_idx]);
		set_text(lbl_b, uc.sf[2], 20, t);
	} else {
		uint32_t age = k_uptime_get_32() - last_frame_ms;

		if (cfg.demo) {
			snprintf(t, sizeof(t), "SIM SIGNAL 1kHz");   /* 15 chars, field limit */
		} else if (cfg.running && age > 500U) {
			snprintf(t, sizeof(t), cfg.trig.mode ? "WAIT TRIG %s"
							     : "NO DATA %s",
				 nm_tsrc[cfg.trig.source]);
		} else {
			snprintf(t, sizeof(t), "%s %s %s",
				 cfg.running ? "TRIG" : "STOP",
				 nm_tsrc[cfg.trig.source], nm_slope[cfg.trig.slope]);
		}
		set_text(lbl_a, uc.sf[1], 20, t);
		fmt_sec(t, sizeof(t), tdiv_tab[cfg.tdiv_idx]);
		strncat(t, "/div", sizeof(t) - strlen(t) - 1);
		set_text(lbl_b, uc.sf[2], 20, t);
	}
	fmt_eng(t, sizeof(t), sr, "Sa/s");
	set_text(lbl_c, uc.sf[3], 20, t);

	if (cfg.meas_on && m && m->valid) {
		snprintf(t, sizeof(t), "%.2f Vpp", (double)m->vpp);
	} else {
		snprintf(t, sizeof(t), "%u pts", (unsigned)WAVE_COLS * 4);
	}
	set_text(lbl_d, uc.sf[4], 20, t);
}

/* ---- rail and bar refresh ---------------------------------------------- */
static void refresh_keys(void)
{
	char t[24];
	uint8_t n = group_cnt[cfg.mode];
	bool mode_changed = (uc.mode != cfg.mode);
	bool sel_changed = (uc.group != sel_group) || mode_changed || (uc.demo != cfg.demo);

	/* key 0 = RUN / OUTPUT, key 1 = MODE, keys 2..7 = groups */
	bool hot = (cfg.mode == MODE_GEN) ? cfg.gen.out_on : cfg.running;

	if (cfg.mode == MODE_GEN) {
		set_text(key_name[0], uc.key_name[0], 12, "OUT");
		set_text(key_val[0], uc.key_val[0], 12, cfg.gen.out_on ? "OFF" : "ON");
	} else {
		set_text(key_name[0], uc.key_name[0], 12, cfg.running ? "STOP" : "RUN");
		set_text(key_val[0], uc.key_val[0], 12, "");
	}
	/* local styles overwrite; adding a style object here would grow the
	 * key's style list on every refresh */
	if (uc.hot != hot || mode_changed) {
		uc.hot = hot;
		lv_obj_set_style_bg_color(key[0], lv_color_hex(hot ? 0x4A1A16 : 0x17452C), 0);
		lv_obj_set_style_bg_grad_color(key[0], lv_color_hex(hot ? 0x2A0E0C : 0x0D2317), 0);
		lv_obj_set_style_border_color(key[0], lv_color_hex(hot ? 0x7A2620 : 0x1E5C3C), 0);
		lv_obj_set_style_shadow_color(key[0], lv_color_hex(hot ? C_ALARM : C_RUN), 0);
		lv_obj_set_style_shadow_width(key[0], 8, 0);
		lv_obj_set_style_shadow_ofs_y(key[0], 0, 0);
		lv_obj_set_style_shadow_opa(key[0], LV_OPA_50, 0);
		lv_obj_set_style_text_color(key_name[0], lv_color_hex(hot ? C_ALARM : C_RUN), 0);
		lv_obj_set_style_text_color(key_val[0], lv_color_hex(hot ? C_ALARM : C_RUN), 0);
	}
	set_text(key_val[1], uc.key_val[1], 12,
		 (const char *[]){ "SCOPE", "FFT", "GEN" }[cfg.mode]);

	for (uint8_t i = 0; i < 6; i++) {
		lv_obj_t *k = key[i + 2];

		if (i >= n) {
			lv_obj_add_flag(k, LV_OBJ_FLAG_HIDDEN);
			continue;
		}
		lv_obj_remove_flag(k, LV_OBJ_FLAG_HIDDEN);

		const struct group *g = &groups[cfg.mode][i];

		set_text(key_name[i + 2], uc.key_name[i + 2], 12, g->name);
		param_text(&g->p[0], t, sizeof(t));
		t[7] = '\0';                        /* the field holds 7 chars */
		set_text(key_val[i + 2], uc.key_val[i + 2], 12, t);

		if (!sel_changed) {
			continue;      /* halo styles only move when the selection does */
		}

		if (i == sel_group) {
			lv_obj_add_state(k, LV_STATE_CHECKED);
			/* phosphor halo in the group colour. Only the active key
			 * and RUN get one - see the redraw budget in the plan */
			lv_obj_set_style_shadow_width(k, 8, 0);
			lv_obj_set_style_shadow_spread(k, 1, 0);
			lv_obj_set_style_shadow_ofs_y(k, 0, 0);
			lv_obj_set_style_shadow_color(k, lv_color_hex(g->color), 0);
			lv_obj_set_style_shadow_opa(k, LV_OPA_60, 0);
			lv_obj_set_style_outline_color(k, lv_color_hex(g->color), 0);
			lv_obj_set_style_outline_opa(k, LV_OPA_80, 0);
		} else {
			lv_obj_remove_state(k, LV_STATE_CHECKED);
			lv_obj_set_style_shadow_width(k, 6, 0);
			lv_obj_set_style_shadow_spread(k, 0, 0);
			lv_obj_set_style_shadow_ofs_y(k, 3, 0);
			lv_obj_set_style_shadow_color(k, lv_color_black(), 0);
			lv_obj_set_style_shadow_opa(k, LV_OPA_50, 0);
			lv_obj_set_style_outline_opa(k, LV_OPA_TRANSP, 0);
		}
	}
	uc.mode = cfg.mode;
	uc.group = sel_group;
	uc.demo = cfg.demo;
}

static void refresh_bar(void)
{
	const struct group *g = &groups[cfg.mode][sel_group];
	const struct param *p = &g->p[sel_param];
	char t[32];

	set_text(lbl_par, uc.par, sizeof(uc.par), p->label);
	param_text(p, t, sizeof(t));
	set_text(lbl_val, uc.val, sizeof(uc.val), t);

	if (cfg.mode == MODE_SCOPE && sel_group < 2) {
		snprintf(t, sizeof(t), "CH%u  %s  x%s  BW %s", sel_group + 1U,
			 nm_coupling[cfg.ch[sel_group].coupling],
			 cfg.ch[sel_group].probe_x10 ? "10" : "1",
			 cfg.ch[sel_group].bw_limit ? "100k" : "FULL");
	} else if (cfg.mode == MODE_GEN) {
		snprintf(t, sizeof(t), "PWM-DAC PB4  %s", nm_wave_s[cfg.gen.wave]);
	} else {
		snprintf(t, sizeof(t), "%s GROUP", g->name);
	}
	set_text(lbl_sub, uc.sub, sizeof(uc.sub), t);

	/* the dots only move when the selected parameter or the group does */
	if (uc.param != sel_param || uc.np != g->np) {
		uc.param = sel_param;
		uc.np = g->np;
		for (int i = 0; i < 8; i++) {
			if (i < g->np) {
				lv_obj_remove_flag(dots[i], LV_OBJ_FLAG_HIDDEN);
				lv_obj_set_style_bg_color(dots[i],
					lv_color_hex(i == sel_param ? C_TXT : C_LINE), 0);
			} else {
				lv_obj_add_flag(dots[i], LV_OBJ_FLAG_HIDDEN);
			}
		}
	}
}

static void apply_now(void);
static void scope_defaults(void);

/* ---- settings popup (spec §4.2) ----------------------------------------- */
enum popup_page { PG_NONE, PG_PARAMS, PG_LIST, PG_ADJUST };
static enum popup_page page = PG_NONE;
static uint32_t last_touch_ms;
static bool demo_toggle_req;
static bool frozen_valid;
static struct dso_frame frozen SDRAM_SECTION;   /* last frame, re-rendered on STOP */
static struct dso_meas last_meas;

static void tile_style(int i, bool on, uint32_t color)
{
	lv_obj_set_style_shadow_width(tile[i], on ? 8 : 4, 0);
	lv_obj_set_style_shadow_ofs_y(tile[i], on ? 0 : 2, 0);
	lv_obj_set_style_shadow_color(tile[i], on ? lv_color_hex(color) : lv_color_black(), 0);
	lv_obj_set_style_shadow_opa(tile[i], on ? LV_OPA_60 : LV_OPA_50, 0);
	lv_obj_set_style_outline_color(tile[i], lv_color_hex(color), 0);
	lv_obj_set_style_outline_opa(tile[i], on ? LV_OPA_80 : LV_OPA_TRANSP, 0);
	if (on) {
		lv_obj_add_state(tile[i], LV_STATE_CHECKED);
	} else {
		lv_obj_remove_state(tile[i], LV_STATE_CHECKED);
	}
}

static void tile_set(int i, const char *lab, const char *val)
{
	lv_obj_remove_flag(tile[i], LV_OBJ_FLAG_HIDDEN);
	lv_label_set_text_static(tile_lab[i], lab);
	if (strncmp(tile_cache[i], val, 15) != 0) {
		strncpy(tile_cache[i], val, 15);
		tile_cache[i][15] = '\0';
		lv_label_set_text(tile_val[i], val);
	}
}

/* page of parameters: one tile each, BACK last */
static void popup_refresh(void)
{
	const struct group *g = &groups[cfg.mode][sel_group];
	char t[24];

	if (page == PG_PARAMS) {
		for (int i = 0; i < TILE_DONE; i++) {
			if (i >= g->np) {
				lv_obj_add_flag(tile[i], LV_OBJ_FLAG_HIDDEN);
				continue;
			}
			param_text(&g->p[i], t, sizeof(t));
			tile_set(i, g->p[i].label, t);
			tile_style(i, i == sel_param, g->color);
		}
		lv_label_set_text_static(tile_val[TILE_DONE], "BACK");
	} else if (page == PG_LIST) {
		const struct param *p = &g->p[sel_param];
		uint8_t cur = p->field ? *(uint8_t *)p->field : 0;

		for (int i = 0; i < TILE_DONE; i++) {
			if (i >= p->cnt) {
				lv_obj_add_flag(tile[i], LV_OBJ_FLAG_HIDDEN);
				continue;
			}
			tile_set(i, p->label, ((const char *const *)p->tab)[i]);
			tile_style(i, i == cur, g->color);
		}
		lv_label_set_text_static(tile_val[TILE_DONE], "BACK");
	}
}

static void popup_set_page(enum popup_page pg)
{
	page = pg;
	popup_open = (pg == PG_PARAMS || pg == PG_LIST);
	memset(tile_cache, 0, sizeof(tile_cache));
	if (popup_open) {
		lv_obj_remove_flag(popup, LV_OBJ_FLAG_HIDDEN);
		popup_refresh();
	} else {
		lv_obj_add_flag(popup, LV_OBJ_FLAG_HIDDEN);
		lv_obj_invalidate(canvas);      /* the trace comes back at once */
	}
}

static void tile_cb(lv_event_t *e)
{
	int i = (int)(intptr_t)lv_event_get_user_data(e);
	const struct group *g = &groups[cfg.mode][sel_group];

	last_touch_ms = k_uptime_get_32();

	if (i == TILE_DONE) {
		/* BACK: list -> parameters, parameters -> trace */
		popup_set_page(page == PG_LIST ? PG_PARAMS : PG_NONE);
		return;
	}

	if (page == PG_LIST) {
		const struct param *p = &g->p[sel_param];

		if (i < p->cnt) {
			ui_lock();
			param_set_enum(p, (uint8_t)i);
			ui_unlock();
			apply_now();
		}
		popup_set_page(PG_PARAMS);      /* applied on tap, back to the page */
		refresh_keys();
		refresh_bar();
		return;
	}

	if (i >= g->np) {
		return;
	}
	const struct param *p = &g->p[i];

	ui_lock();
	sel_param = (uint8_t)i;
	ui_unlock();

	switch (param_ctl(p)) {
	case CTL_LIST:
		popup_set_page(PG_LIST);
		break;
	case CTL_TOGGLE:
		ui_lock();
		param_step(p, 1);
		ui_unlock();
		apply_now();
		popup_refresh();
		break;
	case CTL_ACTION:
		ui_lock();
		param_step(p, 1);
		ui_unlock();
		apply_now();
		popup_set_page(PG_NONE);        /* result must be visible */
		break;
	default:
		/* STEP: collapse so the trace is visible while - / + and drag
		 * change the value (spec §1.1); the group key reopens the page */
		popup_set_page(PG_ADJUST);
		break;
	}
	refresh_keys();
	refresh_bar();
}

/* ---- drag on the trace (spec §4.2.3) ----------------------------------- */
static lv_point_t drag_last;
static bool dragging;

static void canvas_cb(lv_event_t *e)
{
	lv_event_code_t code = lv_event_get_code(e);
	lv_indev_t *indev = lv_indev_active();

	if (!indev || cfg.demo || popup_open) {
		return;
	}
	lv_point_t pt;

	lv_indev_get_point(indev, &pt);
	last_touch_ms = k_uptime_get_32();

	if (code == LV_EVENT_PRESSED) {
		drag_last = pt;
		dragging = true;
		return;
	}
	if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
		dragging = false;
		return;
	}
	if (code != LV_EVENT_PRESSING || !dragging) {
		return;
	}
	int dx = pt.x - drag_last.x, dy = pt.y - drag_last.y;

	if (abs(dx) < 3 && abs(dy) < 3) {
		return;
	}
	drag_last = pt;

	ui_lock();
	if (cfg.mode == MODE_FFT) {
		if (abs(dy) >= abs(dx)) {
			static const float dbdiv_tab[5] = { 1, 2, 5, 10, 20 };

			cfg.fft.ref_db = (int8_t)(cfg.fft.ref_db +
				(int)(dy * dbdiv_tab[cfg.fft.db_div_idx] / DIV_Y));
		}
	} else if (abs(dy) >= abs(dx)) {
		const struct group *g = &groups[cfg.mode][sel_group];

		if (strcmp(g->name, "TRIG") == 0) {
			cfg.trig.level_v -= (float)dy / DIV_Y *
					    vdiv_tab[cfg.ch[cfg.trig.source].vdiv_idx];
		} else {
			uint8_t ch = (strcmp(g->name, "CH2") == 0) ? 1 : 0;

			cfg.ch[ch].offset_div -= (float)dy / DIV_Y;
		}
	} else {
		cfg.hpos_div += (float)dx / DIV_X;
	}
	validate_cfg(NULL);
	ui_unlock();
	apply_now();
	refresh_keys();
	refresh_bar();
}

/* ---- AUTOSET (spec §3) -------------------------------------------------- */
static void autoset_action(void)
{
	/* uses the last measurement of CH1: 1 Vpp -> ~2 divisions, ~4 periods */
	if (!last_meas.valid) {
		return;
	}
	for (int i = 0; i < N_CH; i++) {
		uint8_t v = 0;

		while (v + 1 < vdiv_cnt && last_meas.vpp / vdiv_tab[v] > 4.0f) {
			v++;
		}
		cfg.ch[i].vdiv_idx = v;
	}
	cfg.ch[0].offset_div = 1.5f;
	cfg.ch[1].offset_div = -1.5f;
	if (last_meas.freq_hz > 1.0f) {
		float want = 4.0f / last_meas.freq_hz / 12.0f;   /* s per div */
		uint8_t t = 0;

		while (t + 1 < tdiv_cnt && tdiv_tab[t] < want) {
			t++;
		}
		cfg.tdiv_idx = t;
	}
	cfg.hpos_div = 0.0f;
	cfg.trig.level_v = last_meas.vavg;
	cfg.trig.mode = 0;
	cfg.running = true;
}

/* ---- requests from other threads, timeouts, watchdog (spec §1.5, §6) ---- */
void ui_request_demo_toggle(void)
{
	demo_toggle_req = true;              /* consumed by ui_service() */
}

void ui_service(void)
{
	uint32_t now = k_uptime_get_32();

	if (demo_toggle_req) {
		demo_toggle_req = false;
		ui_lock();
		cfg.demo = !cfg.demo;
		cfg.gen.out_on = false;          /* output never survives a switch */
		sel_group = 0;
		sel_param = 0;
		ui_unlock();
		popup_set_page(PG_NONE);
		ui_clear_display();
		apply_now();
		refresh_keys();
		refresh_bar();
		LOG_INF("%s mode", cfg.demo ? "DEMO" : "LIVE");
	}

	/* 10 s without a touch: any panel folds back to the trace */
	if (page != PG_NONE && (now - last_touch_ms) > 10000U) {
		popup_set_page(PG_NONE);
		refresh_keys();
	}
}

static void apply_now(void)
{
	static uint8_t last_tdiv = 0xFF, last_mode = 0xFF, last_span = 0xFF;
	static struct dso_gen last_gen;
	static bool gen_valid;

	if (cfg.tdiv_idx != last_tdiv || cfg.mode != last_mode ||
	    cfg.fft.span_idx != last_span) {
		last_tdiv = cfg.tdiv_idx;
		last_mode = cfg.mode;
		last_span = cfg.fft.span_idx;
		acq_apply_cfg(&cfg);      /* writes TIM2: only when the sweep moved */
	}

	if (!gen_valid || memcmp(&last_gen, &cfg.gen, sizeof(last_gen)) != 0) {
		last_gen = cfg.gen;
		gen_valid = true;
		gen_apply(&cfg.gen);      /* rebuilds the table and restarts its DMA */
	}
}

static void key_cb(lv_event_t *e)
{
	int id = (int)(intptr_t)lv_event_get_user_data(e);

	ui_lock();
	if (id == 0) {
		if (cfg.mode == MODE_GEN) {
			/* ON needs a long press - there may be someone else's
			 * circuit on that pin; OFF is a short tap */
			if (lv_event_get_code(e) == LV_EVENT_LONG_PRESSED) {
				cfg.gen.out_on = true;
			} else if (cfg.gen.out_on) {
				cfg.gen.out_on = false;
			}
		} else if (lv_event_get_code(e) == LV_EVENT_SHORT_CLICKED) {
			cfg.running = !cfg.running;
		}
	} else if (id == 1) {
		cfg.mode = (uint8_t)((cfg.mode + 1) % MODE_COUNT);
		sel_group = 0;
		sel_param = 0;
		if (cfg.mode == MODE_SCOPE) {
			scope_defaults();
		}
		clear_buffers();
	} else {
		uint8_t g = (uint8_t)(id - 2);

		if (g < group_cnt[cfg.mode]) {
			bool same = (g == sel_group);

			sel_group = g;
			if (!same) {
				sel_param = 0;
			}
			if (!cfg.demo) {
				/* live: same key while a page is open closes it;
				 * while adjusting it reopens the page; otherwise
				 * it opens the page */
				if (same && popup_open) {
					popup_set_page(PG_NONE);
				} else {
					popup_set_page(PG_PARAMS);
				}
			}
		}
	}
	if (id <= 1 && page != PG_NONE) {
		popup_set_page(PG_NONE);      /* RUN or MODE always returns to the trace */
	}
	if (id == 1) {
		cfg.gen.out_on = false;       /* output never survives a mode change */
	}
	last_touch_ms = k_uptime_get_32();
	ui_unlock();
	apply_now();
	refresh_keys();
	refresh_bar();
	if (popup_open) {
		popup_refresh();
	}
}

static void step_cb(lv_event_t *e)
{
	int d = (int)(intptr_t)lv_event_get_user_data(e);

	ui_lock();
	bool wf_was = cfg.fft.waterfall;
	const struct param *p = &groups[cfg.mode][sel_group].p[sel_param];

	param_step(p, p->zoom ? -d : d);
	if (cfg.fft.waterfall != wf_was) {
		memset(wfall, 0, sizeof(wfall));
	}
	ui_unlock();
	apply_now();
	refresh_keys();
	refresh_bar();
	if (popup_open) {
		popup_refresh();
	}
	last_touch_ms = k_uptime_get_32();
}

static void vb_cb(lv_event_t *e)
{
	ARG_UNUSED(e);
	if (!cfg.demo) {
		/* live: the value block reopens the parameter page instead of
		 * blind cycling - the tiles are the selector (spec §4.2) */
		popup_set_page(PG_PARAMS);
		last_touch_ms = k_uptime_get_32();
		return;
	}
	ui_lock();
	sel_param = (uint8_t)((sel_param + 1) % groups[cfg.mode][sel_group].np);
	ui_unlock();
	refresh_bar();
}

/* ---- construction ------------------------------------------------------ */
static lv_obj_t *panel(lv_obj_t *par, int x, int y, int w, int h, uint32_t bg)
{
	lv_obj_t *o = lv_obj_create(par);

	lv_obj_remove_style_all(o);
	lv_obj_set_pos(o, x, y);
	lv_obj_set_size(o, w, h);
	lv_obj_set_style_bg_color(o, lv_color_hex(bg), 0);
	lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
	lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE);
	return o;
}

static lv_obj_t *label(lv_obj_t *par, int x, int y, int w, const lv_font_t *f,
		       uint32_t col, lv_text_align_t al)
{
	lv_obj_t *l = lv_label_create(par);

	lv_obj_set_pos(l, x, y);
	lv_obj_set_width(l, w);
	lv_label_set_long_mode(l, LV_LABEL_LONG_CLIP);
	lv_obj_set_style_text_font(l, f, 0);
	lv_obj_set_style_text_color(l, lv_color_hex(col), 0);
	lv_obj_set_style_text_align(l, al, 0);
	lv_label_set_text(l, "");
	return l;
}

/* one bar of a stepper sign; bars do not take touch events themselves */
static void glyph_bar(lv_obj_t *par, int x, int y, int w, int h)
{
	lv_obj_t *b = lv_obj_create(par);

	lv_obj_remove_style_all(b);
	lv_obj_set_pos(b, x, y);
	lv_obj_set_size(b, w, h);
	lv_obj_set_style_bg_color(b, lv_color_hex(C_TXT), 0);
	lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
	lv_obj_set_style_radius(b, 1, 0);
	lv_obj_remove_flag(b, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_remove_flag(b, LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_add_flag(b, LV_OBJ_FLAG_EVENT_BUBBLE);
}

static void styles_init(void)
{
	lv_style_init(&st_key);
	lv_style_set_bg_color(&st_key, lv_color_hex(C_BTN));
	lv_style_set_bg_grad_color(&st_key, lv_color_hex(C_GND2));
	lv_style_set_bg_grad_dir(&st_key, LV_GRAD_DIR_VER);
	lv_style_set_bg_opa(&st_key, LV_OPA_COVER);
	lv_style_set_border_color(&st_key, lv_color_hex(C_LINE));
	lv_style_set_border_width(&st_key, 1);
	lv_style_set_radius(&st_key, 3);
	lv_style_set_outline_width(&st_key, 1);
	lv_style_set_outline_pad(&st_key, 1);
	lv_style_set_outline_opa(&st_key, LV_OPA_TRANSP);
	lv_style_set_pad_all(&st_key, 0);

	lv_style_init(&st_key_on);
	lv_style_set_bg_color(&st_key_on, lv_color_hex(C_BTN_ON));
	lv_style_set_bg_grad_color(&st_key_on, lv_color_hex(C_BTN));
	lv_style_set_bg_grad_dir(&st_key_on, LV_GRAD_DIR_VER);
	lv_style_set_border_color(&st_key_on, lv_color_hex(C_DIM));

	/*
	 * Pressed = the key lights up like phosphor: the face brightens towards
	 * the readout hue, a halo spills around it, and the caption stays white
	 * so contrast never depends on the glow. Ignition is instant, the decay
	 * takes 260 ms - that is what makes it read as phosphor rather than as a
	 * highlight rectangle.
	 */
	static const lv_style_prop_t tr_props[] = {
		LV_STYLE_BG_COLOR, LV_STYLE_BG_GRAD_COLOR, LV_STYLE_SHADOW_WIDTH,
		LV_STYLE_SHADOW_OPA, LV_STYLE_OUTLINE_OPA, LV_STYLE_BORDER_COLOR,
		LV_STYLE_PROP_INV,
	};

	lv_style_transition_dsc_init(&tr_ignite, tr_props, lv_anim_path_linear, 0, 0, NULL);
	lv_style_transition_dsc_init(&tr_decay, tr_props, lv_anim_path_ease_out, 260, 0, NULL);
	lv_style_set_transition(&st_key, &tr_decay);
	lv_style_set_transition(&st_key_on, &tr_decay);

	lv_style_init(&st_key_pr);
	lv_style_set_bg_color(&st_key_pr, lv_color_hex(0x5A7A93));
	lv_style_set_bg_grad_color(&st_key_pr, lv_color_hex(0x2E4A63));
	lv_style_set_bg_grad_dir(&st_key_pr, LV_GRAD_DIR_VER);
	lv_style_set_border_color(&st_key_pr, lv_color_hex(C_TXT));
	lv_style_set_shadow_color(&st_key_pr, lv_color_hex(C_TXT));
	lv_style_set_shadow_width(&st_key_pr, 8);
	lv_style_set_shadow_spread(&st_key_pr, 1);
	lv_style_set_shadow_opa(&st_key_pr, LV_OPA_70);
	lv_style_set_outline_color(&st_key_pr, lv_color_hex(C_TXT));
	lv_style_set_outline_opa(&st_key_pr, LV_OPA_COVER);
	lv_style_set_transition(&st_key_pr, &tr_ignite);

	lv_style_init(&st_vb);
	lv_style_set_bg_color(&st_vb, lv_color_hex(0x0F151E));
	lv_style_set_bg_opa(&st_vb, LV_OPA_COVER);
	lv_style_set_border_color(&st_vb, lv_color_hex(C_LINE));
	lv_style_set_border_width(&st_vb, 1);
	lv_style_set_radius(&st_vb, 3);
	lv_style_set_pad_all(&st_vb, 0);
}

/*
 * Scope defaults: everything set to see a 1 kHz, 1 V signal on both channels.
 * 500 us/div puts 6 periods across the screen, 1 V/div gives a 1 Vpp trace one
 * division tall, and the two traces are parked 1.5 divisions apart so they do
 * not sit on top of each other. Applied at start-up and whenever the scope
 * mode is entered, so the instrument always opens on a usable picture.
 */
static void scope_defaults(void)
{
	cfg.tdiv_idx = 5;                 /* 500 us/div */
	cfg.hpos_div = 0.0f;
	cfg.ch[0].on = true;
	cfg.ch[0].vdiv_idx = 5;           /* 1 V/div */
	cfg.ch[0].offset_div = 1.5f;
	cfg.ch[1].on = true;
	cfg.ch[1].vdiv_idx = 5;           /* 1 V/div */
	cfg.ch[1].offset_div = -1.5f;
	cfg.trig.level_v = 0.0f;          /* mid-scale: triggers on anything */
	cfg.trig.mode = 0;                /* AUTO */
	cfg.trig.source = 0;
	cfg.running = true;
}

static void defaults(void)
{
	cfg.mode = MODE_SCOPE;
	cfg.running = true;
	cfg.demo = true;          /* the panel must look alive with nothing connected */
	cfg.tdiv_idx = 5;                     /* 500 us/div */
	cfg.persist_idx = 2;
	cfg.meas_on = true;
	cfg.grid_mode = 0;
	cfg.trig = (struct dso_trig){ .hyst_lsb = 40, .holdoff_us = 100000 };
	cfg.fft = (struct dso_fft){ .size_idx = 3, .window = 1, .ref_db = -10,
				    .db_div_idx = 3, .waterfall = true,
				    .peak_search = true };
	scope_defaults();
	cfg.gen = (struct dso_gen){ .wave = 0, .freq_hz = 10000.0f,
				    .ampl_vpp = 2.0f, .offset_v = 1.5f,
				    .duty_pct = 50.0f, .pwm_freq_hz = 1000.0f,
				    .pwm_duty_pct = 50.0f };
}

int ui_init(void)
{
	static const struct {
		int x, y;
	} kpos[8] = {
		{ 341, 26 }, { 413, 26 }, { 341, 89 }, { 413, 89 },
		{ 341, 152 }, { 413, 152 }, { 341, 215 }, { 413, 215 },
	};

	defaults();
	build_ramps();
	styles_init();

	lv_obj_t *scr = lv_screen_active();

	lv_obj_set_style_bg_color(scr, lv_color_hex(C_GND), 0);
	lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

	/* status strip */
	lv_obj_t *st = panel(scr, 0, 0, SCR_W, 22, C_GND2);

	dot = panel(st, 6, 8, 6, 6, C_RUN);
	lv_obj_set_style_radius(dot, 3, 0);
	lbl_state = label(st, 16, 5, 48, &lv_font_unscii_8, C_TXT, LV_TEXT_ALIGN_LEFT);
	lbl_a = label(st, 68, 5, 120, &lv_font_unscii_8, C_DIM, LV_TEXT_ALIGN_LEFT);
	lbl_b = label(st, 196, 5, 96, &lv_font_unscii_8, C_TXT, LV_TEXT_ALIGN_LEFT);
	lbl_c = label(st, 300, 5, 90, &lv_font_unscii_8, C_DIM, LV_TEXT_ALIGN_LEFT);
	lbl_d = label(st, 396, 5, 78, &lv_font_unscii_8, C_DIM, LV_TEXT_ALIGN_RIGHT);

	/* waveform canvas */
	canvas = lv_canvas_create(scr);
	lv_canvas_set_buffer(canvas, cbuf, WAVE_W, WAVE_H, LV_COLOR_FORMAT_RGB565);
	lv_obj_set_pos(canvas, WAVE_X, WAVE_Y);
	lv_obj_add_flag(canvas, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_add_event_cb(canvas, canvas_cb, LV_EVENT_PRESSED, NULL);
	lv_obj_add_event_cb(canvas, canvas_cb, LV_EVENT_PRESSING, NULL);
	lv_obj_add_event_cb(canvas, canvas_cb, LV_EVENT_RELEASED, NULL);
	lv_obj_add_event_cb(canvas, canvas_cb, LV_EVENT_PRESS_LOST, NULL);

	banner = label(scr, 0, WAVE_Y + 84, WAVE_W, &lv_font_montserrat_14,
		       C_ALARM, LV_TEXT_ALIGN_CENTER);
	lv_obj_add_flag(banner, LV_OBJ_FLAG_HIDDEN);

	/* settings popup over the waveform area */
	popup = panel(scr, WAVE_X, WAVE_Y, WAVE_W, WAVE_H, C_GND2);
	lv_obj_set_style_border_color(popup, lv_color_hex(C_LINE), 0);
	lv_obj_set_style_border_width(popup, 1, 0);
	for (int i = 0; i < TILE_N; i++) {
		int x = 4 + (i % TILE_COLS) * (TILE_W + TILE_GAP);
		int y = 11 + (i / TILE_COLS) * (TILE_H + TILE_GAP);

		tile[i] = panel(popup, x, y, TILE_W, TILE_H, C_BTN);
		lv_obj_add_style(tile[i], &st_key, 0);
		lv_obj_add_style(tile[i], &st_key_on, LV_STATE_CHECKED);
		lv_obj_add_style(tile[i], &st_key_pr, LV_STATE_PRESSED);
		lv_obj_add_flag(tile[i], LV_OBJ_FLAG_CLICKABLE);
		lv_obj_add_event_cb(tile[i], tile_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);
		tile_lab[i] = label(tile[i], 6, 5, 90, &lv_font_unscii_8, C_DIM,
				    LV_TEXT_ALIGN_LEFT);
		tile_val[i] = label(tile[i], 6, 22, 90, &lv_font_montserrat_14, C_TXT,
				    LV_TEXT_ALIGN_RIGHT);
	}
	lv_label_set_text_static(tile_lab[TILE_DONE], "");
	lv_label_set_text_static(tile_val[TILE_DONE], "BACK");
	lv_obj_set_style_text_color(tile_val[TILE_DONE], lv_color_hex(C_RUN), 0);
	lv_obj_set_style_text_align(tile_val[TILE_DONE], LV_TEXT_ALIGN_CENTER, 0);
	lv_obj_set_pos(tile_val[TILE_DONE], 6, 15);
	lv_obj_add_flag(popup, LV_OBJ_FLAG_HIDDEN);

	/* rail */
	panel(scr, 336, 22, 144, 250, C_GND2);
	for (int i = 0; i < 8; i++) {
		key[i] = panel(scr, kpos[i].x, kpos[i].y, 61, 52, C_BTN);
		lv_obj_add_style(key[i], &st_key, 0);
		lv_obj_add_style(key[i], &st_key_on, LV_STATE_CHECKED);
		lv_obj_add_style(key[i], &st_key_pr, LV_STATE_PRESSED);
		lv_obj_add_flag(key[i], LV_OBJ_FLAG_CLICKABLE);
		/* key 0 distinguishes short and long presses; LVGL still sends
		 * CLICKED after a long press, so CLICKED must not be used there
		 * or the long press is undone on release */
		if (i == 0) {
			lv_obj_add_event_cb(key[i], key_cb, LV_EVENT_SHORT_CLICKED,
					    (void *)(intptr_t)i);
			lv_obj_add_event_cb(key[i], key_cb, LV_EVENT_LONG_PRESSED,
					    (void *)(intptr_t)i);
		} else {
			lv_obj_add_event_cb(key[i], key_cb, LV_EVENT_CLICKED,
					    (void *)(intptr_t)i);
		}
		key_name[i] = label(key[i], 8, 6, 50, &lv_font_unscii_8, C_DIM,
				    LV_TEXT_ALIGN_LEFT);
		key_val[i] = label(key[i], 8, 26, 50, &lv_font_montserrat_12, C_TXT,
				   LV_TEXT_ALIGN_LEFT);
	}
	lv_label_set_text(key_name[1], "MODE");

	/* bottom bar */
	panel(scr, 0, 214, 336, 58, C_GND2);

	lv_obj_t *minus = panel(scr, 4, 218, 56, 48, C_BTN);
	lv_obj_t *plus = panel(scr, 276, 218, 56, 48, C_BTN);

	lv_obj_add_style(minus, &st_key, 0);
	lv_obj_add_style(minus, &st_key_pr, LV_STATE_PRESSED);
	lv_obj_add_style(plus, &st_key, 0);
	lv_obj_add_style(plus, &st_key_pr, LV_STATE_PRESSED);
	lv_obj_add_flag(minus, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_add_flag(plus, LV_OBJ_FLAG_CLICKABLE);
	/* SHORT_CLICKED + LONG_PRESSED_REPEAT: one step per tap, and a steady
	 * run of steps while the key is held - CLICKED would add a second step
	 * on release after a hold */
	lv_obj_add_event_cb(minus, step_cb, LV_EVENT_SHORT_CLICKED, (void *)(intptr_t)-1);
	lv_obj_add_event_cb(plus, step_cb, LV_EVENT_SHORT_CLICKED, (void *)(intptr_t)1);
	lv_obj_add_event_cb(minus, step_cb, LV_EVENT_LONG_PRESSED_REPEAT, (void *)(intptr_t)-1);
	lv_obj_add_event_cb(plus, step_cb, LV_EVENT_LONG_PRESSED_REPEAT, (void *)(intptr_t)1);

	/* Drawn as bars: a typed hyphen is a few pixels wide, and at arm's
	 * length on a 5 px/mm panel the sign has to be a shape, not a glyph.
	 * 28 x 6 px is half the width of the 56 px key. */
	glyph_bar(minus, 14, 21, 28, 6);
	glyph_bar(plus, 14, 21, 28, 6);
	glyph_bar(plus, 25, 10, 6, 28);

	vb = panel(scr, 71, 218, 194, 48, 0x0F151E);
	lv_obj_add_style(vb, &st_vb, 0);
	lv_obj_add_flag(vb, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_add_event_cb(vb, vb_cb, LV_EVENT_CLICKED, NULL);

	/* the page dots live in a column on the LEFT edge: a right-aligned
	 * value would otherwise run under them and "OFF" reads as garbage */
	for (int i = 0; i < 8; i++) {
		dots[i] = panel(vb, 4, 6 + i * 6, 4, 4, C_LINE);
		lv_obj_set_style_radius(dots[i], 2, 0);
	}
	lbl_par = label(vb, 14, 3, 114, &lv_font_unscii_8, C_DIM, LV_TEXT_ALIGN_LEFT);
	lbl_val = label(vb, 36, 16, 150, &lv_font_montserrat_20, C_TXT,
			LV_TEXT_ALIGN_RIGHT);
	lbl_sub = label(vb, 14, 38, 172, &lv_font_unscii_8, C_DIM, LV_TEXT_ALIGN_LEFT);

	clear_buffers();
	refresh_keys();
	refresh_bar();
	update_status(NULL, 1.0e6f);
	LOG_INF("ui ready");
	return 0;
}

/*
 * The waveform area and the controls are deliberately decoupled:
 *
 *   ui_on_frame()   dynamic area only. Fills the canvas buffer directly - no
 *                   widgets, no allocations - and invalidates one 336x192
 *                   rectangle. Runs at the frame rate.
 *   ui_sync()       static controls. Walks the caches above and touches a
 *                   widget only when its text or state really changed, so a
 *                   quiet panel produces no invalidations at all.
 *
 * lv_timer_handler() then only has to redraw what was invalidated, which on a
 * quiet panel is just the waveform rectangle.
 */
/* Called when the demo/live switch is thrown: the old picture must not linger
 * on the screen for even one frame - it would be data from the other source. */
void ui_clear_display(void)
{
	ui_lock();
	clear_buffers();
	if (cfg.mode == MODE_SCOPE) {
		scope_defaults();
	}
	ui_unlock();
	lv_obj_invalidate(canvas);
}

void ui_sync(const struct dso_meas *m, float sr)
{
	ui_lock();
	if (sel_group >= group_cnt[cfg.mode]) {
		sel_group = 0;                /* live-only group, back in demo */
		sel_param = 0;
	}
	update_status(m, sr);
	refresh_keys();
	refresh_bar();
	ui_unlock();
}

void ui_on_frame(struct dso_frame *f, const struct dso_meas *m,
		 const float *spec)
{
	ui_lock();
	if (spec) {
		memcpy(spec_db, spec, sizeof(spec_db));
	}

	if (f) {
		last_frame_ms = k_uptime_get_32();
		if (m) {
			last_meas = *m;
		}
	}

	switch (cfg.mode) {
	case MODE_SCOPE: {
		/* RUN: draw the new frame and keep a copy. STOP: keep re-drawing
		 * the frozen copy, so V/div, offset and persistence changes are
		 * still visible on the stopped picture (spec §6). */
		struct dso_frame *src = NULL;

		if (f && cfg.running) {
			memcpy(&frozen, f, offsetof(struct dso_frame, s));
			for (uint8_t ch = 0; ch < f->nch && ch < N_CH; ch++) {
				memcpy(frozen.s[ch], f->s[ch], sizeof(uint16_t) * f->n);
			}
			frozen_valid = true;
			src = f;
		} else if (!cfg.running && frozen_valid) {
			src = &frozen;
		}
		if (src) {
			if (cfg.xy_mode) {
				render_xy(src);
			} else {
				render_scope(src);
			}
		}
		break;
	}
	case MODE_FFT:
		if (cfg.running) {
			render_fft();
		}
		break;
	default:
		render_gen();
		break;
	}

	bool clip = (cfg.mode == MODE_GEN) &&
		    (cfg.gen.offset_v + cfg.gen.ampl_vpp / 2.0f > ADC_VREF);

	if (clip != uc.banner) {
		uc.banner = clip;
		if (clip) {
			lv_label_set_text(banner, "OUTPUT CLIPPING");
			lv_obj_remove_flag(banner, LV_OBJ_FLAG_HIDDEN);
		} else {
			lv_obj_add_flag(banner, LV_OBJ_FLAG_HIDDEN);
		}
	}

	ui_unlock();

	if (!popup_open) {
		lv_obj_invalidate(canvas);   /* the only area that changes every frame */
	}
}
