/*
 * Demo mode: synthesised signals, no hardware involved.
 *
 * The board boots into this mode so the panel is alive and every screen looks
 * the way it is meant to look even with nothing connected to the inputs.
 * The B1 USER key switches to live acquisition and back.
 *
 * The signals are deliberately chosen to exercise the whole UI:
 *   CH1  1 kHz sine with a 3rd and 5th harmonic - gives a clean trace and,
 *        in FFT mode, a fundamental plus two harmonics to find peaks on
 *   CH2  square at the same frequency with a duty that slowly sweeps
 *        30..70 %, so the duty readout visibly moves
 * Amplitude breathes slowly, so Vpp is never a frozen number, and a small
 * jitter is added so persistence looks like phosphor rather than a hard line.
 */
#include <zephyr/kernel.h>
#include <zephyr/linker/devicetree_regions.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "app.h"

#define SDRAM_SECTION __attribute__((section(LINKER_DT_NODE_REGION_NAME(DT_NODELABEL(sdram1)))))

static struct dso_frame demo_frame SDRAM_SECTION;
static uint32_t rng = 0xA5A5F00D;

static inline float noise(float amp)
{
	rng = rng * 1103515245u + 12345u;
	return amp * ((float)((rng >> 16) & 0xFFFF) / 32768.0f - 1.0f);
}

struct dso_frame *sim_frame(const struct dso_cfg *cfg)
{
	static float breathe;
	static float duty_ph;

	breathe += 0.05f;
	duty_ph += 0.013f;

	uint8_t tidx = cfg->tdiv_idx;

	if (tidx >= tdiv_cnt) {
		tidx = (uint8_t)(tdiv_cnt - 1);
	}

	/* the same rules the hardware follows: one sample per pixel column in
	 * scope mode, the fixed FFT rate in spectrum mode */
	float sr;
	size_t n;

	if (cfg->mode == MODE_FFT) {
		sr = fft_rate_tab[cfg->fft.span_idx < FFT_SPAN_CNT ? cfg->fft.span_idx : 0];
		n = fft_size_tab[cfg->fft.size_idx];
	} else {
		n = acq_plan(tidx, &sr);
	}

	if (n > FRAME_MAX) {
		n = FRAME_MAX;
	}

	const float f0 = 1000.0f;
	const float lsb = ADC_VREF / ADC_FULL_SCALE;
	/* 1.2 Vpp breathing by +-15 %, so the value moves without looking broken */
	float amp1 = (0.6f * (1.0f + 0.15f * sinf(breathe))) / lsb;
	float amp2 = 0.75f / lsb;
	float duty = 0.5f + 0.2f * sinf(duty_ph);

	/* a small phase jitter keeps the trigger looking real; the trace still
	 * stands still because the trigger point is the phase origin */
	float jit = noise(0.02f);

	/* the trigger point sits at the pre-trigger position, so the phase
	 * origin follows the horizontal position exactly as the hardware does */
	size_t pre = (cfg->mode == MODE_FFT) ? 0 : acq_pretrigger(n, cfg->hpos_div);

	for (size_t i = 0; i < n; i++) {
		float t = ((float)i - (float)pre) / sr;
		float ph = 2.0f * (float)M_PI * f0 * t + jit;

		float s1 = sinf(ph) + 0.18f * sinf(3.0f * ph) + 0.07f * sinf(5.0f * ph);
		float v1 = 2048.0f + amp1 * s1 + noise(6.0f);

		/* wrap into [0,1) explicitly: t is negative before the trigger
		 * point and fmodf would return a negative fraction there */
		float frac = fmodf(f0 * t + jit / (2.0f * (float)M_PI), 1.0f);

		if (frac < 0.0f) {
			frac += 1.0f;
		}
		float s2 = (frac < duty) ? 1.0f : -1.0f;
		float v2 = 2048.0f - 700.0f + amp2 * s2 + noise(5.0f);

		demo_frame.s[0][i] = (uint16_t)(v1 < 0 ? 0 : (v1 > 4095 ? 4095 : v1));
		demo_frame.s[1][i] = (uint16_t)(v2 < 0 ? 0 : (v2 > 4095 ? 4095 : v2));
	}

	demo_frame.n = (uint16_t)n;
	demo_frame.nch = 2;
	demo_frame.triggered = true;
	demo_frame.sample_rate = sr;
	return &demo_frame;
}
