/* hardware stand-ins: the UI must behave identically whether or not these do
 * anything, which is exactly what the simulation checks */
#include "app.h"
#include <math.h>
#include <stdlib.h>

uint32_t sim_now_ms;

int  acq_init(void) { return 0; }
void acq_apply_cfg(const struct dso_cfg *cfg) { (void)cfg; }
void acq_start(void) {}
void acq_stop(void) {}
struct dso_frame *acq_take_frame(int t) { (void)t; return NULL; }
void acq_release_frame(struct dso_frame *f) { (void)f; }
uint32_t acq_overruns(void) { return 0; }
int  gen_init(void) { return 0; }
void gen_apply(const struct dso_gen *g) { (void)g; }
int  dsp_init(void) { return 0; }

/* a cheap stand-in for the CMSIS transform: peaks where the demo puts them */
void dsp_spectrum(const uint16_t *s, size_t n, uint8_t window,
		  float *out_db, size_t cols)
{
	(void)s; (void)n; (void)window;
	for (size_t c = 0; c < cols; c++) {
		float v = -95.0f + (rand() % 100) / 25.0f;
		int px[4] = { (int)(cols * 0.125), (int)(cols * 0.25),
			      (int)(cols * 0.375), (int)(cols * 0.625) };
		for (int k = 0; k < 4; k++) {
			int d = (int)c - px[k];
			float p = -10.0f - 12.0f * k - (float)(d * d) / 4.0f;
			if (p > v) v = p;
		}
		out_db[c] = v;
	}
}
