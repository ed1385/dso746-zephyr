/*
 * Author:  Eduard
 * Project: DSO-746 - oscilloscope, FFT analyzer and signal generator
 * Repo:    https://github.com/ed1385/dso746-zephyr
 * License: MIT
 *
 * Trigger search, pre-trigger extraction and column reduction.
 * No hardware here on purpose: tests/host compiles this file directly.
 */
#include "app.h"

/* 1-2-5 ladders, shared with the UI so the displayed value and the value
 * the hardware is programmed with can never drift apart. */
const float vdiv_tab[] = { 0.02f, 0.05f, 0.1f, 0.2f, 0.5f, 1.0f, 2.0f };
const size_t vdiv_cnt = sizeof(vdiv_tab) / sizeof(vdiv_tab[0]);

/*
 * The ladder starts where the hardware can actually deliver one sample per
 * pixel column: WAVE_COLS columns over 12 divisions.
 *   1.8 MSa/s (two channels)      -> 336/(12*1.8e6)  = 15.6 us/div
 *   5.4 MSa/s (one channel, x3)   -> 336/(12*5.4e6)  =  5.2 us/div
 * so 10 us/div is the fastest 1-2-5 step that is honestly filled; a
 * 5 us/div step would need the 8-bit triple-interleaved mode (7.4 MSa/s).
 * Anything faster would be drawn from interpolated points, so it is simply
 * not offered. TDIV_MIN_DUAL is the first index legal with two channels.
 */
const float tdiv_tab[] = {
	1e-5f, 2e-5f, 5e-5f, 1e-4f, 2e-4f, 5e-4f,
	1e-3f, 2e-3f, 5e-3f, 1e-2f, 2e-2f, 5e-2f, 1e-1f,
};
const size_t tdiv_cnt = sizeof(tdiv_tab) / sizeof(tdiv_tab[0]);

size_t tdiv_min_idx(bool interleaved)
{
	return interleaved ? 0 : TDIV_MIN_DUAL;
}

const uint16_t fft_size_tab[] = { 256, 512, 1024, 2048, 4096 };

const float fft_rate_tab[FFT_SPAN_CNT] = { 16384.0f, 65536.0f, 262144.0f, 1.8e6f };
const size_t fft_size_cnt = sizeof(fft_size_tab) / sizeof(fft_size_tab[0]);

size_t acq_plan(uint8_t tdiv_idx, float *sample_rate)
{
	if (tdiv_idx >= tdiv_cnt) {
		tdiv_idx = (uint8_t)(tdiv_cnt - 1);
	}
	float screen = 12.0f * tdiv_tab[tdiv_idx];
	size_t k = 4;

	/* the most oversampling the ADC can afford for this sweep */
	while (k > 1 && (float)(WAVE_COLS * k) / screen > 1.8e6f) {
		k--;
	}
	size_t n = WAVE_COLS * k;
	float sr = (float)n / screen;

	if (sr > 1.8e6f) {
		sr = 1.8e6f;
	}
	if (sr < 1000.0f) {
		sr = 1000.0f;         /* slowest TIM2 rate we bother with */
	}
	*sample_rate = sr;
	return n;
}

size_t acq_pretrigger(size_t n, float hpos_div)
{
	float pre = (float)n / 4.0f + hpos_div * (float)n / 12.0f;

	if (pre < 0.0f) {
		return 0;
	}
	if (pre > (float)(n - 1)) {
		return n - 1;
	}
	return (size_t)pre;
}

int acq_find_trigger(const uint16_t *ring, size_t ring_len,
		     size_t from, size_t to,
		     uint16_t level, uint16_t hyst, uint8_t slope,
		     bool *armed)
{
	if (to > ring_len || from == 0 || from >= to) {
		return -1;
	}

	for (size_t i = from; i < to; i++) {
		uint16_t prev = ring[i - 1];
		uint16_t cur  = ring[i];

		if (!*armed) {
			/* re-arm only after the signal left the hysteresis band */
			if (slope == 0 && cur < (int)level - (int)hyst) {
				*armed = true;
			} else if (slope == 1 && cur > (int)level + (int)hyst) {
				*armed = true;
			} else if (slope == 2 &&
				   (cur < (int)level - (int)hyst ||
				    cur > (int)level + (int)hyst)) {
				*armed = true;
			}
			continue;
		}

		bool rise = (prev < level && cur >= level);
		bool fall = (prev > level && cur <= level);

		if ((slope == 0 && rise) || (slope == 1 && fall) ||
		    (slope == 2 && (rise || fall))) {
			*armed = false;
			return (int)i;
		}
	}
	return -1;
}

void acq_extract(const uint16_t *ring, size_t ring_len,
		 size_t trig, size_t pre, uint16_t *out, size_t n)
{
	size_t start = (trig + ring_len - (pre % ring_len)) % ring_len;

	for (size_t i = 0; i < n; i++) {
		size_t idx = start + i;

		while (idx >= ring_len) {
			idx -= ring_len;
		}
		out[i] = ring[idx];
	}
}

void acq_reduce_minmax(const uint16_t *src, size_t n,
		       uint16_t *ymin, uint16_t *ymax, size_t cols)
{
	if (cols == 0) {
		return;
	}
	if (n == 0) {
		for (size_t c = 0; c < cols; c++) {
			ymin[c] = ymax[c] = 0;
		}
		return;
	}

	for (size_t c = 0; c < cols; c++) {
		size_t a = (n * c) / cols;
		size_t b = (n * (c + 1)) / cols;

		if (b <= a) {
			b = a + 1;
		}
		if (b > n) {
			b = n;
		}

		uint16_t mn = src[a], mx = src[a];

		for (size_t i = a + 1; i < b; i++) {
			if (src[i] < mn) {
				mn = src[i];
			}
			if (src[i] > mx) {
				mx = src[i];
			}
		}
		ymin[c] = mn;
		ymax[c] = mx;
	}
}
