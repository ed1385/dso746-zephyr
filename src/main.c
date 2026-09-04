/*
 * DSO-746 - entry point.
 *
 * Thread map (priority in brackets, lower number = higher priority):
 *   DMA2 stream 4 ISR   flags the finished ring half, gives a semaphore
 *   acq  [-2]           trigger search, pre-trigger extraction, frame slots
 *   main/ui [7]         LVGL, measurements, spectrum, rendering
 *   input               FT5336 interrupt -> Zephyr input -> lv_indev
 *
 * The rule that keeps the display smooth: the UI never blocks acquisition.
 * If a frame cannot be handed over it is dropped and sampling continues.
 */
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/logging/log.h>
#include <lvgl.h>
#include <zephyr/input/input.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>

#include "app.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/*
 * The DMA2D draw unit installs its own interrupt handler through the dma2d
 * devicetree node declared in the board overlay, so nothing is needed here.
 */

/*
 * Two rates, on purpose.
 *
 * LVGL is serviced every 10 ms. That is what makes touch feel immediate: the
 * FT5336 raises its interrupt, the Zephyr input driver queues the event, and
 * lv_timer_handler picks it up within 10 ms instead of within a frame period.
 * Key animations - the phosphor decay - also step at this rate.
 *
 * The waveform is recomputed at 50 ms. Redrawing a trace faster than 20 Hz
 * buys nothing a human can see and costs the whole FMC budget.
 */
#define TICK_MS  10
#define FRAME_MS 50

static float spec[WAVE_COLS];

/*
 * B1 USER key (PI11) toggles demo <-> live. It arrives through the Zephyr
 * input subsystem as INPUT_KEY_0, the same path the touch panel uses, so the
 * gpio-keys driver keeps owning the pin and nothing fights over the GPIO.
 */
static void key_cb(struct input_event *evt, void *user_data)
{
	ARG_UNUSED(user_data);

	if (evt->type != INPUT_EV_KEY || evt->code != INPUT_KEY_0 || evt->value == 0) {
		return;
	}

	/* Spec §1.5: this runs in the input thread. It may not touch LVGL,
	 * so it only requests the switch; the UI thread applies it. */
	ui_request_demo_toggle();
}
INPUT_CALLBACK_DEFINE(NULL, key_cb, NULL);

int main(void)
{
	const struct device *disp = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

	if (!device_is_ready(disp)) {
		LOG_ERR("display not ready");
		return -ENODEV;
	}

	ui_init();
	dsp_init();
	gen_init();
	acq_init();

	ui_lock();
	acq_apply_cfg(ui_cfg());
	bool demo = ui_cfg()->demo;

	ui_unlock();
	/* the converters run from boot in both modes: switching to live then
	 * shows a live screen immediately instead of after the first sweep */
	acq_start();
	LOG_INF("boot in %s mode, B1 USER key switches", demo ? "DEMO" : "LIVE");

	display_blanking_off(disp);
	lv_timer_handler();

	uint32_t next_tick = k_uptime_get_32();
	uint32_t next_frame = next_tick;
	uint32_t frames = 0, dropped = 0, last_stat = next_tick;

	while (true) {
		uint32_t now_ms = k_uptime_get_32();

		/* ---- fast lane: input and widget animations only ---------- */
		if ((int32_t)(now_ms - next_frame) < 0) {
			ui_service();          /* deferred requests, timeouts */
			lv_timer_handler();
			next_tick += TICK_MS;
			int32_t s2 = (int32_t)(next_tick - k_uptime_get_32());

			k_msleep(s2 > 0 ? s2 : 1);
			continue;
		}
		next_frame += FRAME_MS;

		ui_lock();
		bool sim = ui_cfg()->demo;

		ui_unlock();

		struct dso_frame *f;
		bool owned = false;          /* only real frames go back to the slab */

		if (sim) {
			/* drop anything the hardware queued before the switch */
			struct dso_frame *stale;

			while ((stale = acq_take_frame(0)) != NULL) {
				acq_release_frame(stale);
			}
			ui_lock();
			f = sim_frame(ui_cfg());
			ui_unlock();
		} else {
			f = acq_take_frame(0);     /* never block the UI thread */
			owned = (f != NULL);
		}

		struct dso_meas m = { 0 };
		const float *sp = NULL;

		if (f) {
			ui_lock();
			uint8_t mode = ui_cfg()->mode;
			uint8_t fsrc = ui_cfg()->fft.source;
			uint8_t win = ui_cfg()->fft.window;
			bool meas_on = ui_cfg()->meas_on;

			ui_unlock();

			if (mode == MODE_FFT) {
				uint8_t ch = fsrc < f->nch ? fsrc : 0;

				dsp_spectrum(f->s[ch], f->n, win, spec, WAVE_COLS);
				sp = spec;
			} else if (meas_on) {
				dsp_measure(f->s[0], f->n, f->sample_rate,
					    ADC_VREF / ADC_FULL_SCALE, 2048.0f, &m);
			}

			/* Drain anything that piled up while we were drawing and
			 * keep only the newest: showing a stale frame is worse
			 * than showing fewer of them. */
			if (owned) {
				struct dso_frame *newer;

				while ((newer = acq_take_frame(0)) != NULL) {
					acq_release_frame(f);
					f = newer;
					dropped++;
				}
			}

			ui_on_frame(f, &m, sp);
			ui_sync(&m, f->sample_rate);
			if (owned) {
				acq_release_frame(f);
			}
			frames++;
		} else {
			ui_on_frame(NULL, NULL, NULL);   /* GEN / no trigger */
			ui_sync(NULL, 0.0f);
		}

		uint32_t now = k_uptime_get_32();

		if (now - last_stat >= 5000U) {
			LOG_INF("%u frames/s drawn, %u dropped, lvgl %u ms",
				frames / 5U, dropped / 5U,
				(unsigned)(k_uptime_get_32() - now));
			frames = dropped = 0;
			last_stat = now;
		}

		lv_timer_handler();

		if ((int32_t)(k_uptime_get_32() - next_frame) > 0) {
			next_frame = k_uptime_get_32();  /* we are behind: resync */
		}
		next_tick = k_uptime_get_32();
	}
	return 0;
}
