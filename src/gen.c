/*
 * Signal generator on PB4 (Arduino D3, TIM3_CH1).
 *
 * The board has no usable DAC pin: UM1907 shows PA4 = DCMI_HSYNC (routed only
 * to the camera FPC) and PA5 = USB_OTG_HS_ULPI_CK. So the output is a PWM-DAC:
 * a table of duty values is pushed by DMA into TIM3->CCR1 on every update
 * event, and an external RC filter turns it into the waveform. The firmware
 * structure - table, DMA, peripheral - is the same one a real DAC would use,
 * so swapping in a DAC later changes only the destination address.
 *
 * TIM3_UP -> DMA1 stream 2, channel 5 (RM0385 table 42).
 *
 * Carrier vs resolution at a 108 MHz timer clock:
 *      8 bit  (ARR 255)  -> 422 kHz carrier, clean output to ~20 kHz
 *     10 bit  (ARR 1023) -> 105 kHz carrier, clean output to ~5 kHz
 * The resolution is chosen from the requested frequency, aiming for at least
 * 20 carrier periods per output period.
 */
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <soc.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_dma.h>
#include <stm32_ll_tim.h>
#include <stm32_ll_gpio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "app.h"

LOG_MODULE_REGISTER(gen, LOG_LEVEL_INF);

#define TIM3_CLK_HZ 108000000U
#define TABLE_MAX   256
#define TABLE_MIN   10          /* the spec rule: >= 10 DAC updates per period */

/*
 * One PWM period is one DAC update, so the update rate is the carrier:
 *   8 bit -> 422 kHz, 10 bit -> 105 kHz.
 * The table length adapts to the frequency: as many points as the carrier
 * allows, capped at TABLE_MAX, never fewer than TABLE_MIN. That gives
 *   sine/triangle/ramp:   <= 42 kHz at 8 bit, <= 10.5 kHz at 10 bit
 * A fixed 256-point table would have limited a clean sine to 1.6 kHz.
 */
static uint16_t wave_tab[TABLE_MAX] __nocache __aligned(32);
static uint32_t cur_arr = 255;
static uint32_t cur_len = TABLE_MAX;

static float wave_sample(uint8_t wave, float ph, float duty)
{
	switch (wave) {
	case 0: return sinf(2.0f * (float)M_PI * ph);
	case 1: return ph < duty / 100.0f ? 1.0f : -1.0f;
	case 2: return 2.0f * fabsf(2.0f * ph - 1.0f) - 1.0f;
	case 3: return 2.0f * ph - 1.0f;
	case 4: return 1.0f - 2.0f * ph;
	case 5: return ph < duty / 200.0f ? 1.0f : -1.0f;
	case 6: {
		float u = (ph - 0.5f) * 20.0f;

		return u == 0.0f ? 1.0f : sinf(u) / u;
	}
	case 7: {
		static uint32_t r = 22695477u;

		r = r * 1103515245u + 12345u;
		return ((r >> 16) & 0xFFFF) / 32768.0f - 1.0f;
	}
	default: return sinf(2.0f * (float)M_PI * ph);
	}
}

static void build_table(const struct dso_gen *g, uint32_t arr, uint32_t len)
{
	float half = g->ampl_vpp / 2.0f;

	for (size_t i = 0; i < len; i++) {
		float ph = (float)i / len;
		float v = g->offset_v + half * wave_sample(g->wave, ph, g->duty_pct);

		/* the PWM-DAC output swings 0..3.3 V; clamp instead of wrapping,
		 * and let the UI show the OUTPUT CLIPPING banner */
		if (v < 0.0f) {
			v = 0.0f;
		}
		if (v > ADC_VREF) {
			v = ADC_VREF;
		}
		wave_tab[i] = (uint16_t)((v / ADC_VREF) * (float)arr + 0.5f);
	}
}

/*
 * Calibrator / hard square on PH6 (Arduino D6, TIM12_CH1). This is a plain
 * hardware PWM with no DMA: the pin the PWM group of the panel controls.
 */
static void calib_apply(const struct dso_gen *g)
{
	if (!g->pwm_on) {
		LL_TIM_CC_DisableChannel(TIM12, LL_TIM_CHANNEL_CH1);
		LL_TIM_DisableCounter(TIM12);
		return;
	}

	float f = g->pwm_freq_hz;

	if (f < 2.0f) {
		f = 2.0f;
	}
	uint32_t arr = 999;                              /* 0.1 % duty steps */
	uint32_t psc = (uint32_t)((float)TIM3_CLK_HZ / (f * (arr + 1)) + 0.5f);

	if (psc > 0) {
		psc--;
	}
	if (psc > 0xFFFF) {
		psc = 0xFFFF;
	}

	LL_TIM_SetPrescaler(TIM12, psc);
	LL_TIM_SetAutoReload(TIM12, arr);
	LL_TIM_OC_SetCompareCH1(TIM12,
		(uint32_t)((arr + 1) * g->pwm_duty_pct / 100.0f));
	LL_TIM_CC_EnableChannel(TIM12, LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableCounter(TIM12);
	LL_TIM_GenerateEvent_UPDATE(TIM12);
}

void gen_apply(const struct dso_gen *g)
{
	calib_apply(g);

	if (!g->out_on) {
		LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH1);
		LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_2);
		LL_TIM_DisableCounter(TIM3);
		return;
	}

	/* carrier: 10 bit whenever the frequency still allows TABLE_MIN
	 * points per period at 105 kHz, otherwise 8 bit at 422 kHz */
	uint32_t arr = 255;

	if (g->freq_hz * TABLE_MIN <= (float)TIM3_CLK_HZ / 1024.0f) {
		arr = 1023;
	}
	float carrier = (float)TIM3_CLK_HZ / (arr + 1.0f);

	/* points per period the carrier can afford, clamped to the table */
	uint32_t len = (uint32_t)(carrier / g->freq_hz);

	if (len > TABLE_MAX) {
		len = TABLE_MAX;
	}
	if (len < TABLE_MIN) {
		len = TABLE_MIN;             /* above 42 kHz the sine degrades */
	}

	/* update rate = freq * len; a prescaler trims the low frequencies
	 * where even 256 points leave the carrier far above what is needed */
	float upd = g->freq_hz * (float)len;
	uint32_t psc = 0;

	while (upd > 0.0f && (float)TIM3_CLK_HZ / ((psc + 1.0f) * (arr + 1.0f)) > upd
	       && psc < 0xFFFF) {
		psc++;
	}

	build_table(g, arr, len);
	cur_arr = arr;
	cur_len = len;

	LL_TIM_DisableCounter(TIM3);
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_2);
	while (LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_2)) {
	}

	LL_TIM_SetPrescaler(TIM3, psc);
	LL_TIM_SetAutoReload(TIM3, arr);
	LL_TIM_OC_SetCompareCH1(TIM3, wave_tab[0]);

	LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_2, LL_DMA_CHANNEL_5);
	LL_DMA_ConfigTransfer(DMA1, LL_DMA_STREAM_2,
			      LL_DMA_DIRECTION_MEMORY_TO_PERIPH |
			      LL_DMA_MODE_CIRCULAR |
			      LL_DMA_PERIPH_NOINCREMENT |
			      LL_DMA_MEMORY_INCREMENT |
			      LL_DMA_PDATAALIGN_HALFWORD |
			      LL_DMA_MDATAALIGN_HALFWORD |
			      LL_DMA_PRIORITY_HIGH);
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_2,
			       (uint32_t)wave_tab,
			       (uint32_t)&TIM3->CCR1,
			       LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_2, len);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_2);

	LL_TIM_EnableDMAReq_UPDATE(TIM3);
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableCounter(TIM3);
	LL_TIM_GenerateEvent_UPDATE(TIM3);

	LOG_INF("gen %u Hz: %u points/period, ARR %u, PSC %u, %s",
		(unsigned)g->freq_hz, len, arr, psc, arr == 255 ? "8 bit" : "10 bit");
}

int gen_init(void)
{
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM12);

	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_4, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_4, LL_GPIO_AF_2);   /* TIM3_CH1 */
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_4, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_4, LL_GPIO_OUTPUT_PUSHPULL);

	/* calibrator pin: PH6, AF9 = TIM12_CH1 */
	LL_GPIO_SetPinMode(GPIOH, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_0_7(GPIOH, LL_GPIO_PIN_6, LL_GPIO_AF_9);
	LL_GPIO_SetPinSpeed(GPIOH, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_HIGH);
	LL_TIM_OC_SetMode(TIM12, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_EnablePreload(TIM12, LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableARRPreload(TIM12);

	LL_TIM_OC_SetMode(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableARRPreload(TIM3);
	LL_TIM_SetCounterMode(TIM3, LL_TIM_COUNTERMODE_UP);

	LOG_INF("generator on PB4 (Arduino D3) ready, output off");
	return 0;
}
