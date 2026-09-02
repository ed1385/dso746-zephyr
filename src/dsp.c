/*
 * Spectrum analysis with CMSIS-DSP. arm_rfft_fast_f32 on a 216 MHz Cortex-M7
 * with FPU takes well under a millisecond at 2048 points, so the spectrum is
 * limited by the 20 Hz display rate, not by the maths.
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <arm_math.h>
#include <zephyr/linker/devicetree_regions.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "app.h"

LOG_MODULE_REGISTER(dsp, LOG_LEVEL_INF);

#define FFT_MAX 4096

static arm_rfft_fast_instance_f32 rfft[5];   /* one per size in fft_size_tab */
static bool rfft_ok[5];

/* 56 KB of working arrays. They are touched by the CPU only and at 20 Hz, so
 * SDRAM costs nothing measurable; internal SRAM is needed for the DMA ring,
 * whose non-cached MPU region already has to be power-of-two aligned. */
#define SDRAM_SECTION __attribute__((section(LINKER_DT_NODE_REGION_NAME(DT_NODELABEL(sdram1)))))

static float32_t fin[FFT_MAX] SDRAM_SECTION __aligned(4);
static float32_t fout[FFT_MAX] SDRAM_SECTION __aligned(4);
static float32_t mag[FFT_MAX / 2] SDRAM_SECTION __aligned(4);
static float32_t win_tab[FFT_MAX] SDRAM_SECTION;
static uint16_t  win_len;
static uint8_t   win_kind = 0xFF;

int dsp_init(void)
{
	for (size_t i = 0; i < fft_size_cnt; i++) {
		rfft_ok[i] = (arm_rfft_fast_init_f32(&rfft[i], fft_size_tab[i])
			      == ARM_MATH_SUCCESS);
		if (!rfft_ok[i]) {
			LOG_WRN("no FFT table for %u points", fft_size_tab[i]);
		}
	}
	return 0;
}

/* Coherent gain of each window, so the peak height stays honest when the
 * window is changed. Rectangular is 1 by definition. */
static const float win_gain[5] = { 1.0f, 0.5f, 0.54f, 0.42f, 0.2156f };

static void make_window(uint8_t kind, uint16_t n)
{
	if (kind == win_kind && n == win_len) {
		return;
	}
	for (uint16_t i = 0; i < n; i++) {
		float x = 2.0f * (float)M_PI * i / (n - 1);

		switch (kind) {
		case 1: win_tab[i] = 0.5f - 0.5f * cosf(x); break;               /* Hann */
		case 2: win_tab[i] = 0.54f - 0.46f * cosf(x); break;             /* Hamming */
		case 3: win_tab[i] = 0.35875f - 0.48829f * cosf(x)
					+ 0.14128f * cosf(2 * x)
					- 0.01168f * cosf(3 * x); break;         /* Blackman-Harris */
		case 4: win_tab[i] = 0.21557f - 0.41663f * cosf(x)
					+ 0.27726f * cosf(2 * x)
					- 0.08357f * cosf(3 * x)
					+ 0.00695f * cosf(4 * x); break;         /* flat top */
		default: win_tab[i] = 1.0f; break;
		}
	}
	win_kind = kind;
	win_len = n;
}

void dsp_spectrum(const uint16_t *s, size_t n, uint8_t window,
		  float *out_db, size_t cols)
{
	size_t idx = 0;

	for (size_t i = 0; i < fft_size_cnt; i++) {
		if (fft_size_tab[i] == n) {
			idx = i;
			break;
		}
	}
	if (!rfft_ok[idx] || n > FFT_MAX) {
		for (size_t c = 0; c < cols; c++) {
			out_db[c] = -120.0f;
		}
		return;
	}

	if (window > 4) {
		window = 1;
	}
	make_window(window, (uint16_t)n);

	const float lsb = ADC_VREF / ADC_FULL_SCALE;

	/* remove the DC pedestal before windowing, otherwise the window
	 * smears mid-scale across the first bins and buries small signals */
	float mean = 0.0f;

	for (size_t i = 0; i < n; i++) {
		mean += (float)s[i];
	}
	mean /= (float)n;

	for (size_t i = 0; i < n; i++) {
		fin[i] = ((float)s[i] - mean) * lsb * win_tab[i];
	}

	arm_rfft_fast_f32(&rfft[idx], fin, fout, 0);
	arm_cmplx_mag_f32(fout, mag, n / 2);

	const float scale = 2.0f / ((float)n * win_gain[window]);
	size_t bins = n / 2;

	for (size_t c = 0; c < cols; c++) {
		size_t a = (bins * c) / cols;
		size_t b = (bins * (c + 1)) / cols;

		if (b <= a) {
			b = a + 1;
		}
		float peak = 0.0f;

		for (size_t i = a; i < b && i < bins; i++) {
			if (mag[i] > peak) {
				peak = mag[i];
			}
		}
		float v = peak * scale;

		out_db[c] = (v > 1e-7f) ? 20.0f * log10f(v) : -140.0f;
	}
}
