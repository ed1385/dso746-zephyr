/*
 * Host simulation of the instrument UI.
 *
 * The real ui.c and sim.c are compiled against a real LVGL 9 with a fake
 * display and a fake touch panel. The main loop cadence is reproduced
 * (LVGL every 10 ms, a frame every 50 ms). Touches are injected as pointer
 * events, so hit-testing, hidden objects and event ordering are exercised
 * the way the FT5336 would. Built with AddressSanitizer + UBSan, any out of
 * bounds access or use of uninitialised memory aborts with a stack trace.
 *
 *   1. directed scenario: the reported FFT -> SPAN path, every option
 *   2. random walk: thousands of taps on every key, tile, keypad button and
 *      on the trace, in every mode, demo and live
 */
#include "app.h"
#include <lvgl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern uint32_t sim_now_ms;

static bool tp_pressed;
static int32_t tp_x, tp_y;
static uint16_t fb[480 * 272];

static void flush_cb(lv_display_t *d, const lv_area_t *a, uint8_t *px)
{
	(void)a; (void)px;
	lv_display_flush_ready(d);
}

static void indev_cb(lv_indev_t *i, lv_indev_data_t *d)
{
	(void)i;
	d->point.x = tp_x;
	d->point.y = tp_y;
	d->state = tp_pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

/* ---- the main loop, host edition -------------------------------------- */
static float spec[WAVE_COLS];
static uint32_t next_frame;

static void tick(uint32_t ms)
{
	for (uint32_t t = 0; t < ms; t += 10) {
		sim_now_ms += 10;
		lv_tick_inc(10);
		ui_service();
		lv_timer_handler();

		if ((int32_t)(sim_now_ms - next_frame) >= 0) {
			next_frame += 50;
			ui_lock();
			struct dso_frame *f = sim_frame(ui_cfg());
			uint8_t mode = ui_cfg()->mode;
			uint8_t win = ui_cfg()->fft.window;
			uint8_t fsrc = ui_cfg()->fft.source;
			bool meas_on = ui_cfg()->meas_on;
			ui_unlock();

			struct dso_meas m = { 0 };
			const float *sp = NULL;

			if (mode == MODE_FFT) {
				bool ok = false;
				for (size_t i = 0; i < fft_size_cnt; i++)
					if (fft_size_tab[i] == f->n) ok = true;
				if (ok) {
					dsp_spectrum(f->s[fsrc < f->nch ? fsrc : 0], f->n,
						     win, spec, WAVE_COLS);
					sp = spec;
				}
			} else if (meas_on) {
				dsp_measure(f->s[0], f->n, f->sample_rate,
					    ADC_VREF / ADC_FULL_SCALE, 2048.0f, &m);
			}
			ui_on_frame(f, &m, sp);
			ui_sync(&m, f->sample_rate);
		}
	}
}

static void tap(int x, int y)
{
	tp_x = x; tp_y = y; tp_pressed = true;
	tick(60);
	tp_pressed = false;
	tick(60);
}

static void hold(int x, int y, uint32_t ms)
{
	tp_x = x; tp_y = y; tp_pressed = true;
	tick(ms);
	tp_pressed = false;
	tick(60);
}

static void drag(int x0, int y0, int x1, int y1)
{
	tp_x = x0; tp_y = y0; tp_pressed = true;
	tick(30);
	for (int k = 1; k <= 8; k++) {
		tp_x = x0 + (x1 - x0) * k / 8;
		tp_y = y0 + (y1 - y0) * k / 8;
		tick(20);
	}
	tp_pressed = false;
	tick(60);
}

/* ---- targets, in panel pixels (from the approved geometry) ------------- */
static const struct { int x, y; } key_c[8] = {      /* rail key centres */
	{ 371, 52 }, { 443, 52 }, { 371, 115 }, { 443, 115 },
	{ 371, 178 }, { 443, 178 }, { 371, 241 }, { 443, 241 },
};
#define RUN  0
#define MODE 1
static const int minus_c[2] = { 32, 242 }, plus_c[2] = { 304, 242 }, vb_c[2] = { 168, 242 };

static void tile_c(int i, int *x, int *y)          /* 3x3 popup tiles */
{
	*x = 4 + (i % 3) * 113 + 51;
	*y = 22 + 11 + (i / 3) * 57 + 23;
}

static void kp_c(int i, int *x, int *y)            /* 5x3 keypad */
{
	*x = 6 + (i % 5) * 67 + 28;
	*y = 22 + 16 + (i / 5) * 57 + 23;
}

static void set_mode(uint8_t want)
{
	for (int k = 0; k < 4 && ui_cfg()->mode != want; k++) {
		tap(key_c[MODE].x, key_c[MODE].y);
	}
}

/* ---- scenarios ------------------------------------------------------- */
static void scenario_fft_span(bool demo)
{
	printf("== scenario: %s FFT -> SPAN -> every option\n", demo ? "DEMO" : "LIVE");
	if (ui_cfg()->demo != demo) {
		ui_request_demo_toggle();
		tick(100);
	}
	set_mode(MODE_FFT);
	tap(key_c[2].x, key_c[2].y);              /* SRC group */
	int x, y;

	tile_c(1, &x, &y);                        /* SPAN tile */
	tap(x, y);
	for (int opt = 0; opt < 4; opt++) {
		tile_c(opt, &x, &y);
		tap(x, y);                        /* pick option, back to page */
		tick(300);                        /* several frames at that span */
		tile_c(1, &x, &y);
		tap(x, y);                        /* reopen the list */
	}
	tile_c(8, &x, &y);
	tap(x, y);                                /* BACK */
	tap(x, y);                                /* BACK again -> trace */
	tick(200);
	printf("   span scenario done, mode %u demo %u\n", ui_cfg()->mode, ui_cfg()->demo);

	/* leak probe: the same list opened and closed 300 times */
	lv_mem_monitor_t a, b;

	lv_mem_monitor(&a);
	for (int k = 0; k < 300; k++) {
		tap(key_c[2].x, key_c[2].y);          /* SRC: open page */
		tile_c(1, &x, &y); tap(x, y);          /* SPAN list */
		tile_c(k % 4, &x, &y); tap(x, y);      /* pick */
		tile_c(8, &x, &y); tap(x, y);          /* BACK to trace */
	}
	lv_mem_monitor(&b);
	printf("   leak probe: used before %u after %u (delta %d B over 300 cycles)\n",
	       (unsigned)(a.total_size - a.free_size), (unsigned)(b.total_size - b.free_size),
	       (int)((b.total_size - b.free_size) - (a.total_size - a.free_size)));
}

static void random_walk(unsigned steps, unsigned seed)
{
	printf("== random walk: %u steps, seed %u\n", steps, seed);
	srand(seed);
	for (unsigned s = 0; s < steps; s++) {
		int r = rand() % 100;
		int x, y;

		if (r < 30) {                             /* rail key */
			int k = rand() % 8;
			tap(key_c[k].x, key_c[k].y);
		} else if (r < 55) {                      /* popup tile / keypad */
			if (rand() & 1) tile_c(rand() % 9, &x, &y);
			else kp_c(rand() % 15, &x, &y);
			tap(x, y);
		} else if (r < 75) {                      /* steppers, with holds */
			const int *c = (rand() & 1) ? minus_c : plus_c;
			if (rand() % 4 == 0) hold(c[0], c[1], 900);
			else tap(c[0], c[1]);
		} else if (r < 82) {
			tap(vb_c[0], vb_c[1]);
		} else if (r < 92) {                      /* drag on the trace */
			drag(rand() % 336, 22 + rand() % 192, rand() % 336, 22 + rand() % 192);
		} else if (r < 96) {
			hold(key_c[RUN].x, key_c[RUN].y, 800);  /* long press RUN */
		} else if (r < 98) {
			ui_request_demo_toggle();
			tick(50);
		} else {
			tick(11000);                          /* popup timeout */
		}
		if ((s % 500) == 499) {
			lv_mem_monitor_t mon;

			lv_mem_monitor(&mon);
			printf("   %u steps  mode %u demo %u run %u | lvgl used %u B (%u%%) frag %u%% max-free %u\n",
			       s + 1, ui_cfg()->mode, ui_cfg()->demo, ui_cfg()->running,
			       (unsigned)(mon.total_size - mon.free_size), mon.used_pct,
			       mon.frag_pct, (unsigned)mon.free_biggest_size);
		}
	}
}

int main(int argc, char **argv)
{
	unsigned steps = argc > 1 ? (unsigned)atoi(argv[1]) : 3000;
	unsigned seed = argc > 2 ? (unsigned)atoi(argv[2]) : 1;

	lv_init();
	lv_display_t *disp = lv_display_create(480, 272);

	lv_display_set_buffers(disp, fb, NULL, sizeof(fb), LV_DISPLAY_RENDER_MODE_DIRECT);
	lv_display_set_flush_cb(disp, flush_cb);
	lv_indev_t *indev = lv_indev_create();

	lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
	lv_indev_set_read_cb(indev, indev_cb);

	ui_init();
	next_frame = 50;
	tick(200);

	scenario_fft_span(true);
	scenario_fft_span(false);
	random_walk(steps, seed);

	printf("SIMULATION PASSED\n");
	return 0;
}
