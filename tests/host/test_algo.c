/*
 * Author:  Eduard
 * Project: DSO-746 - oscilloscope, FFT analyzer and signal generator
 * Repo:    https://github.com/ed1385/dso746-zephyr
 * License: MIT
 *
 * Host tests for the hardware-free parts. Build and run:
 *     cc -I../../src -O2 -Wall -Wextra test_algo.c ../../src/acq_algo.c \
 *        ../../src/dsp_math.c -lm -o test_algo && ./test_algo
 */
#include "app.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

static int fails;

#define CHECK(cond, ...) do { if (!(cond)) { \
	printf("FAIL %s:%d ", __FILE__, __LINE__); printf(__VA_ARGS__); \
	printf("\n"); fails++; } } while (0)

static void fill_sine(uint16_t *buf, size_t n, float f, float fs,
		      float amp_codes, float dc, float noise)
{
	static unsigned rng = 12345;

	for (size_t i = 0; i < n; i++) {
		rng = rng * 1103515245u + 12345u;
		float nz = noise * (((rng >> 16) & 0xFFFF) / 32768.0f - 1.0f);
		float v = dc + amp_codes * sinf(2.0f * (float)M_PI * f * i / fs) + nz;

		if (v < 0) {
			v = 0;
		}
		if (v > 4095) {
			v = 4095;
		}
		buf[i] = (uint16_t)(v + 0.5f);
	}
}

static void test_trigger_basic(void)
{
	uint16_t ring[512];
	bool armed = true;

	for (size_t i = 0; i < 512; i++) {
		ring[i] = (i % 100) < 50 ? 500 : 3500;   /* square, edge at 50 */
	}

	int t = acq_find_trigger(ring, 512, 1, 400, 2000, 20, 0, &armed);

	CHECK(t == 50, "rising edge expected at 50, got %d", t);
	CHECK(armed == false, "trigger must consume the arm flag");

	/* after firing we must not fire again until the signal drops back */
	int t2 = acq_find_trigger(ring, 512, (size_t)t + 1, 400, 2000, 20, 0, &armed);
	CHECK(t2 == 150, "second edge expected at 150, got %d", t2);
}

static void test_trigger_hysteresis(void)
{
	/* A slow noisy sine: without hysteresis this produces several
	 * triggers on one edge. That is exactly the defect the reference
	 * project reports below 12 kHz. */
	uint16_t ring[4096];

	fill_sine(ring, 4096, 200.0f, 200000.0f, 1500.0f, 2048.0f, 120.0f);

	unsigned fires_no_hyst = 0, fires_hyst = 0;
	bool a1 = true, a2 = true;
	size_t p = 1;

	while (p < 4000) {
		int t = acq_find_trigger(ring, 4096, p, 4000, 2048, 0, 0, &a1);

		if (t < 0) {
			break;
		}
		fires_no_hyst++;
		p = (size_t)t + 1;
	}
	p = 1;
	while (p < 4000) {
		int t = acq_find_trigger(ring, 4096, p, 4000, 2048, 300, 0, &a2);

		if (t < 0) {
			break;
		}
		fires_hyst++;
		p = (size_t)t + 1;
	}

	/* 4096 samples at 200 kSa/s = 20.5 ms = ~4 periods of 200 Hz */
	printf("  triggers without hysteresis: %u, with 300 LSB: %u\n",
	       fires_no_hyst, fires_hyst);
	CHECK(fires_hyst >= 3 && fires_hyst <= 5,
	      "hysteresis run should find ~4 periods, got %u", fires_hyst);
	CHECK(fires_no_hyst > fires_hyst,
	      "no-hysteresis run should be noisier (%u vs %u)",
	      fires_no_hyst, fires_hyst);
}

static void test_extract_wrap(void)
{
	uint16_t ring[256], out[64];

	for (size_t i = 0; i < 256; i++) {
		ring[i] = (uint16_t)i;
	}

	/* trigger at 10, 32 samples of pre-trigger -> starts at 234 and wraps */
	acq_extract(ring, 256, 10, 32, out, 64);
	CHECK(out[0] == 234, "wrapped start expected 234, got %u", out[0]);
	CHECK(out[31] == 9, "sample before trigger expected 9, got %u", out[31]);
	CHECK(out[32] == 10, "trigger sample expected 10, got %u", out[32]);
	CHECK(out[63] == 41, "last sample expected 41, got %u", out[63]);
}

static void test_reduce_minmax(void)
{
	uint16_t src[1000], mn[100], mx[100];

	for (size_t i = 0; i < 1000; i++) {
		src[i] = 2048;
	}
	src[555] = 4000;                    /* a single-sample spike */
	src[556] = 100;

	acq_reduce_minmax(src, 1000, mn, mx, 100);
	CHECK(mx[55] == 4000, "spike must survive decimation, got %u", mx[55]);
	CHECK(mn[55] == 100, "negative spike must survive, got %u", mn[55]);
	CHECK(mx[10] == 2048 && mn[10] == 2048, "quiet column must be flat");

	/* fewer samples than columns must not read out of bounds or divide by 0 */
	acq_reduce_minmax(src, 7, mn, mx, 100);
	CHECK(mx[99] == 2048, "short record must still fill every column");
}

static void test_measure(void)
{
	uint16_t buf[2048];
	struct dso_meas m;
	const float fs = 1.8e6f;
	const float lsb = ADC_VREF / ADC_FULL_SCALE;

	/* 10 kHz sine, 1.0 Vpp around mid-scale */
	float amp_codes = 0.5f / lsb;        /* 0.5 V peak = 1 Vpp */

	/* 180 samples per period, so use a whole number of periods (10) or the
	 * partial last period biases Vavg by a percent of the amplitude */
	fill_sine(buf, 1800, 10000.0f, fs, amp_codes, 2048.0f, 0.0f);
	dsp_measure(buf, 1800, fs, lsb, 2048.0f, &m);

	CHECK(m.valid, "measurement must be valid");
	CHECK(fabsf(m.vpp - 1.0f) < 0.02f, "Vpp expected 1.00, got %.3f", m.vpp);
	CHECK(fabsf(m.vrms - 0.3536f) < 0.01f,
	      "Vrms expected 0.354, got %.3f", m.vrms);
	CHECK(fabsf(m.vavg) < 0.01f, "Vavg expected 0, got %.3f", m.vavg);
	CHECK(fabsf(m.freq_hz - 10000.0f) < 60.0f,
	      "freq expected 10000, got %.1f", m.freq_hz);
	printf("  sine: Vpp %.3f Vrms %.3f Vavg %.3f f %.1f Hz duty %.1f%%\n",
	       m.vpp, m.vrms, m.vavg, m.freq_hz, m.duty_pct);

	/* square wave, duty 25 % */
	for (size_t i = 0; i < 2048; i++) {
		buf[i] = (i % 180) < 45 ? 3000 : 1000;
	}
	dsp_measure(buf, 2048, fs, lsb, 2048.0f, &m);
	CHECK(fabsf(m.duty_pct - 25.0f) < 2.0f,
	      "duty expected 25%%, got %.1f", m.duty_pct);
	CHECK(fabsf(m.freq_hz - fs / 180.0f) < 200.0f,
	      "square freq expected %.0f, got %.0f", fs / 180.0f, m.freq_hz);
	printf("  square: duty %.1f%% f %.0f Hz\n", m.duty_pct, m.freq_hz);

	/* flat line must not invent a frequency */
	memset(buf, 0, sizeof(buf));
	for (size_t i = 0; i < 2048; i++) {
		buf[i] = 2048;
	}
	dsp_measure(buf, 2048, fs, lsb, 2048.0f, &m);
	CHECK(m.freq_hz == 0.0f, "flat line must report 0 Hz, got %.1f", m.freq_hz);
	CHECK(m.valid, "flat line is still a valid measurement");
}

static void test_ladders(void)
{
	/* the UI steps these by index; a gap or a wrong order shows up as a
	 * timebase that jumps backwards */
	for (size_t i = 1; i < tdiv_cnt; i++) {
		CHECK(tdiv_tab[i] > tdiv_tab[i - 1], "tdiv ladder not increasing at %zu", i);
	}
	for (size_t i = 1; i < vdiv_cnt; i++) {
		CHECK(vdiv_tab[i] > vdiv_tab[i - 1], "vdiv ladder not increasing at %zu", i);
	}
	/* the fastest timebase must be reachable with the ADC we have:
	 * WAVE_COLS columns over 12 divisions at <= 1.8 MSa/s per channel */
	float need_il   = (float)WAVE_COLS / (12.0f * tdiv_tab[tdiv_min_idx(true)]);
	float need_dual = (float)WAVE_COLS / (12.0f * tdiv_tab[tdiv_min_idx(false)]);

	printf("  fastest single-channel %.1f us/div needs %.2f MSa/s\n",
	       tdiv_tab[tdiv_min_idx(true)] * 1e6f, need_il / 1e6f);
	printf("  fastest dual-channel   %.1f us/div needs %.2f MSa/s\n",
	       tdiv_tab[tdiv_min_idx(false)] * 1e6f, need_dual / 1e6f);
	CHECK(need_il <= 5.4e6f, "fastest interleaved timebase needs %.2f MSa/s", need_il / 1e6f);
	CHECK(need_dual <= 1.8e6f, "fastest dual timebase needs %.2f MSa/s", need_dual / 1e6f);
}

static void test_plan(void)
{
	/* 1 kHz over 12 divisions at 500 us/div must show exactly 6 periods */
	float sr;
	size_t n = acq_plan(5, &sr);            /* index 5 = 500 us/div */
	float periods = (float)n / (sr / 1000.0f);

	printf("  500 us/div: %zu samples at %.0f Sa/s -> %.2f periods of 1 kHz\n",
	       n, sr, periods);
	CHECK(fabsf(periods - 6.0f) < 0.05f, "expected 6 periods, got %.2f", periods);
	CHECK(n % WAVE_COLS == 0, "record must be a whole number of columns");

	/* the fastest dual sweep must still fit the ADC */
	n = acq_plan((uint8_t)tdiv_min_idx(false), &sr);
	printf("  fastest sweep: %zu samples at %.0f Sa/s\n", n, sr);
	CHECK(sr <= 1.8e6f + 1.0f, "sweep needs %.0f Sa/s", sr);
	CHECK(n == WAVE_COLS, "fastest sweep should be one sample per column");

	/* pre-trigger follows the horizontal position and never leaves the record */
	CHECK(acq_pretrigger(1344, 0.0f) == 336, "25 %% pre-trigger at hpos 0");
	CHECK(acq_pretrigger(1344, -6.0f) == 0, "hpos -6 div clamps to 0");
	CHECK(acq_pretrigger(1344, 6.0f) < 1344, "hpos +6 div stays inside");
}

int main(void)
{
	printf("sweep planner\n");      test_plan();
	printf("trigger basic\n");      test_trigger_basic();
	printf("trigger hysteresis\n"); test_trigger_hysteresis();
	printf("pre-trigger wrap\n");   test_extract_wrap();
	printf("min/max reduce\n");     test_reduce_minmax();
	printf("measurements\n");       test_measure();
	printf("ladders\n");            test_ladders();

	printf(fails ? "\n%d FAILURE(S)\n" : "\nall checks passed\n", fails);
	return fails ? 1 : 0;
}
