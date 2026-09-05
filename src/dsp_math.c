/*
 * Author:  Eduard
 * Project: DSO-746 - oscilloscope, FFT analyzer and signal generator
 * Repo:    https://github.com/ed1385/dso746-zephyr
 * License: MIT
 *
 * Automatic measurements. Deliberately free of hardware and of CMSIS-DSP so
 * the same code runs in tests/host.
 */
#include "app.h"
#include <math.h>

void dsp_measure(const uint16_t *s, size_t n, float sample_rate,
		 float lsb_volts, float zero_code, struct dso_meas *m)
{
	m->valid = false;
	if (n < 8 || sample_rate <= 0.0f) {
		return;
	}

	uint16_t mn = s[0], mx = s[0];
	double sum = 0.0, sum2 = 0.0;

	for (size_t i = 0; i < n; i++) {
		uint16_t v = s[i];

		if (v < mn) {
			mn = v;
		}
		if (v > mx) {
			mx = v;
		}
		double d = ((double)v - (double)zero_code) * (double)lsb_volts;

		sum  += d;
		sum2 += d * d;
	}

	m->vmax = ((float)mx - zero_code) * lsb_volts;
	m->vmin = ((float)mn - zero_code) * lsb_volts;
	m->vpp  = m->vmax - m->vmin;
	m->vavg = (float)(sum / n);
	m->vrms = (float)sqrt(sum2 / n);

	/*
	 * Frequency by counting mid-level crossings with a hysteresis of an
	 * eighth of the amplitude. Counting periods between the FIRST and the
	 * LAST crossing - not dividing the crossing count by the record length -
	 * removes the partial-period error that otherwise shows up as a few
	 * percent offset on short records.
	 */
	uint16_t mid  = (uint16_t)((mn + mx) / 2);
	uint16_t hyst = (uint16_t)((mx - mn) / 8);

	if (mx - mn < 16) {                 /* flat line: no frequency */
		m->freq_hz = 0.0f;
		m->duty_pct = 0.0f;
		m->valid = true;
		return;
	}

	bool armed = false;
	long first = -1, last = -1;
	unsigned periods = 0;
	size_t high_cnt = 0;

	for (size_t i = 1; i < n; i++) {
		if (s[i] > mid) {
			high_cnt++;
		}
		if (!armed) {
			if (s[i] < (int)mid - (int)hyst) {
				armed = true;
			}
			continue;
		}
		if (s[i - 1] < mid && s[i] >= mid) {
			armed = false;
			if (first < 0) {
				first = (long)i;
			} else {
				last = (long)i;
				periods++;
			}
		}
	}

	if (periods > 0 && last > first) {
		m->freq_hz = sample_rate * (float)periods / (float)(last - first);
	} else {
		m->freq_hz = 0.0f;
	}
	m->duty_pct = 100.0f * (float)high_cnt / (float)(n - 1);
	m->valid = true;
}
