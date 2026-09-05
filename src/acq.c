/*
 * Author:  Eduard
 * Project: DSO-746 - oscilloscope, FFT analyzer and signal generator
 * Repo:    https://github.com/ed1385/dso746-zephyr
 * License: MIT
 *
 * Acquisition: TIM2 TRGO -> ADC1 (CH1, PA0) and ADC3 (CH2, PF10) -> DMA2
 * circular, half/full interrupts. Written on the LL headers on purpose:
 * Zephyr's ADC driver only offers one-shot adc_read() and cannot do
 * continuous circular DMA, which is the whole point of an oscilloscope.
 * Everything else in this application stays ordinary Zephyr.
 *
 * DMA2 is deliberately left `status = "disabled"` in the overlay so the
 * Zephyr DMA driver does not claim the same streams and interrupts.
 *
 * Streams (RM0385 table 43):
 *   ADC1 -> DMA2 stream 4, channel 0
 *   ADC3 -> DMA2 stream 1, channel 2
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <soc.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_adc.h>
#include <stm32_ll_dma.h>
#include <stm32_ll_tim.h>
#include <stm32_ll_gpio.h>
#include <stm32_ll_rcc.h>

#include <zephyr/linker/devicetree_regions.h>

#include "app.h"

LOG_MODULE_REGISTER(acq, LOG_LEVEL_INF);

/* TIM2 sits on APB1; with APB1 prescaler 4 the timer clock is 2x APB1 */
#define TIM2_CLK_HZ   108000000U
#define ADC_CLK_HZ    27000000U        /* PCLK2 108 MHz / 4                */
#define ADC_CYCLES    15U              /* 12-bit conversion + 3 sampling   */
#define ADC_MAX_SPS   (ADC_CLK_HZ / ADC_CYCLES)   /* 1 800 000 Sa/s        */

/* The ring must not live in SDRAM: LTDC reads the framebuffer from there
 * continuously and the two would fight over FMC. It must also be outside the
 * D-cache, which DMA does not see. */
static uint16_t ring[N_CH][ADC_RING_LEN] __nocache __aligned(32);

/* a frame is extracted from ONE finished half; if this ever fails, the
 * extraction would read into the half being written */
BUILD_ASSERT(FRAME_MAX <= ADC_RING_LEN / 2, "frame must fit in half the ring");

/* frame slots are CPU-only, so they live in SDRAM; internal SRAM is reserved
 * for the DMA ring and the stacks */
static char frame_pool[FRAME_SLOTS * sizeof(struct dso_frame)]
	__attribute__((section(LINKER_DT_NODE_REGION_NAME(DT_NODELABEL(sdram1)))))
	__aligned(4);
static struct k_mem_slab frame_slab;
K_MSGQ_DEFINE(frame_q, sizeof(struct dso_frame *), FRAME_SLOTS, 4);
static K_SEM_DEFINE(half_sem, 0, 1);

static volatile uint8_t ready_half;    /* 0 = lower half, 1 = upper half   */
static volatile uint32_t overrun_cnt;
static volatile uint8_t discard_halves;  /* halves still holding old-rate samples */
static struct dso_cfg cfg_snapshot;
static bool trig_armed = true;
static float cur_sample_rate = 1.0e6f;
static uint32_t last_free_ms;

/* ---------------------------------------------------------------- clocks */
static void gpio_analog(GPIO_TypeDef *port, uint32_t pin)
{
	LL_GPIO_SetPinMode(port, pin, LL_GPIO_MODE_ANALOG);
	LL_GPIO_SetPinPull(port, pin, LL_GPIO_PULL_NO);
}

static void adc_setup(ADC_TypeDef *adc, uint32_t channel)
{
	LL_ADC_SetResolution(adc, LL_ADC_RESOLUTION_12B);
	LL_ADC_SetDataAlignment(adc, LL_ADC_DATA_ALIGN_RIGHT);
	/* the F7 LL folds the edge into the source constant; the edge is then
	 * given again to LL_ADC_REG_StartConversionExtTrig() */
	LL_ADC_REG_SetTriggerSource(adc, LL_ADC_REG_TRIG_EXT_TIM2_TRGO);
	LL_ADC_REG_SetSequencerLength(adc, LL_ADC_REG_SEQ_SCAN_DISABLE);
	LL_ADC_REG_SetSequencerRanks(adc, LL_ADC_REG_RANK_1, channel);
	LL_ADC_SetChannelSamplingTime(adc, channel, LL_ADC_SAMPLINGTIME_3CYCLES);
	LL_ADC_REG_SetContinuousMode(adc, LL_ADC_REG_CONV_SINGLE);
	LL_ADC_REG_SetDMATransfer(adc, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
	LL_ADC_Enable(adc);
}

static void dma_setup(uint32_t stream, uint32_t channel,
		      uint32_t periph_addr, uint16_t *mem)
{
	LL_DMA_DisableStream(DMA2, stream);
	while (LL_DMA_IsEnabledStream(DMA2, stream)) {
	}

	LL_DMA_SetChannelSelection(DMA2, stream, channel);
	LL_DMA_ConfigTransfer(DMA2, stream,
			      LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
			      LL_DMA_MODE_CIRCULAR |
			      LL_DMA_PERIPH_NOINCREMENT |
			      LL_DMA_MEMORY_INCREMENT |
			      LL_DMA_PDATAALIGN_HALFWORD |
			      LL_DMA_MDATAALIGN_HALFWORD |
			      LL_DMA_PRIORITY_VERYHIGH);
	LL_DMA_ConfigAddresses(DMA2, stream, periph_addr, (uint32_t)mem,
			       LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetDataLength(DMA2, stream, ADC_RING_LEN);
	LL_DMA_DisableFifoMode(DMA2, stream);
}

/* Only the CH1 stream raises interrupts. CH2 runs on the same TIM2 trigger
 * with the same length, so its write pointer is at the same offset - one
 * interrupt source is enough and halves the ISR load. */
static void dma_ch1_isr(const void *arg)
{
	ARG_UNUSED(arg);

	if (LL_DMA_IsActiveFlag_HT4(DMA2)) {
		LL_DMA_ClearFlag_HT4(DMA2);
		ready_half = 0;
		if (k_sem_count_get(&half_sem)) {
			overrun_cnt++;      /* the trigger thread fell behind */
		}
		k_sem_give(&half_sem);
	}
	if (LL_DMA_IsActiveFlag_TC4(DMA2)) {
		LL_DMA_ClearFlag_TC4(DMA2);
		ready_half = 1;
		if (k_sem_count_get(&half_sem)) {
			overrun_cnt++;
		}
		k_sem_give(&half_sem);
	}
	if (LL_DMA_IsActiveFlag_TE4(DMA2)) {
		LL_DMA_ClearFlag_TE4(DMA2);
		LOG_ERR("DMA transfer error");
	}
}

/* --------------------------------------------------------------- control */
static uint32_t rate_to_arr(float sps)
{
	if (sps > (float)ADC_MAX_SPS) {
		sps = (float)ADC_MAX_SPS;
	}
	if (sps < 1000.0f) {
		sps = 1000.0f;
	}
	uint32_t div = (uint32_t)((float)TIM2_CLK_HZ / sps + 0.5f);

	return div ? div - 1U : 0U;
}

void acq_apply_cfg(const struct dso_cfg *cfg)
{
	cfg_snapshot = *cfg;

	uint8_t idx = cfg->tdiv_idx;
	uint8_t min_idx = (uint8_t)tdiv_min_idx(false);   /* two channels */

	if (idx < min_idx) {
		idx = min_idx;
	}
	if (cfg->mode == MODE_FFT) {
		uint8_t si = cfg->fft.span_idx < FFT_SPAN_CNT ? cfg->fft.span_idx : 0;

		cur_sample_rate = fft_rate_tab[si];
	} else {
		acq_plan(idx, &cur_sample_rate);
	}

	uint32_t arr = rate_to_arr(cur_sample_rate);

	LL_TIM_SetAutoReload(TIM2, arr);
	LL_TIM_GenerateEvent_UPDATE(TIM2);
	/* the ring keeps running: the half being written and the next one
	 * still hold samples taken at the previous rate. A frame built from
	 * them would carry the new rate with old spacing - a one-frame flash
	 * of wrong time scale. Skip them. */
	discard_halves = 2;
	LOG_INF("timebase %.1f us/div, %u Sa/s, ARR %u",
		(double)(tdiv_tab[idx] * 1e6f), (unsigned)cur_sample_rate, arr);
}

void acq_start(void)
{
	LL_TIM_EnableCounter(TIM2);
	LL_ADC_REG_StartConversionExtTrig(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
	LL_ADC_REG_StartConversionExtTrig(ADC3, LL_ADC_REG_TRIG_EXT_RISING);
}

void acq_stop(void)
{
	LL_TIM_DisableCounter(TIM2);
}

/* ------------------------------------------------------------ trigger job */
static uint16_t level_to_code(const struct dso_cfg *c)
{
	/* volts -> ADC code, using the same scale the UI displays */
	float lsb = ADC_VREF / ADC_FULL_SCALE;
	float code = 2048.0f + c->trig.level_v / lsb;

	if (code < 0.0f) {
		code = 0.0f;
	}
	if (code > ADC_FULL_SCALE) {
		code = ADC_FULL_SCALE;
	}
	return (uint16_t)code;
}

static void acq_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	const size_t half = ADC_RING_LEN / 2;

	while (true) {
		k_sem_take(&half_sem, K_FOREVER);

		/* copy only what a sweep needs, under the lock for as short a
		 * time as possible: this runs up to 430 times a second */
		struct dso_cfg cfg;

		ui_lock();
		cfg.running = ui_cfg()->running;
		cfg.mode = ui_cfg()->mode;
		cfg.tdiv_idx = ui_cfg()->tdiv_idx;
		cfg.hpos_div = ui_cfg()->hpos_div;
		cfg.trig = ui_cfg()->trig;
		cfg.fft = ui_cfg()->fft;
		cfg.ch[1].on = ui_cfg()->ch[1].on;
		ui_unlock();

		if (!cfg.running && cfg.mode != MODE_FFT) {
			continue;
		}

		size_t base = ready_half ? half : 0;
		size_t want;
		float sr_unused;

		if (cfg.mode == MODE_FFT) {
			want = fft_size_tab[cfg.fft.size_idx];
		} else {
			want = acq_plan(cfg.tdiv_idx, &sr_unused);
		}
		if (want > FRAME_MAX) {
			want = FRAME_MAX;
		}
		size_t pre = (cfg.mode == MODE_FFT) ? 0
						    : acq_pretrigger(want, cfg.hpos_div);
		uint8_t src = cfg.trig.source < N_CH ? cfg.trig.source : 0;

		/* the whole window [trig - pre, trig - pre + want) must lie
		 * inside the half that is finished: pre-trigger samples before
		 * `base` would come from the half the DMA is writing right now */
		size_t search_begin = base + pre + 1;
		size_t search_end = base + half - (want - pre);
		int trig = -1;

		if (cfg.mode == MODE_SCOPE && search_end > search_begin) {
			trig = acq_find_trigger(ring[src], ADC_RING_LEN,
						search_begin, search_end,
						level_to_code(&cfg),
						cfg.trig.hyst_lsb,
						cfg.trig.slope, &trig_armed);
		}

		uint32_t now = k_uptime_get_32();
		bool auto_sweep = false;

		if (trig < 0) {
			if (cfg.mode == MODE_FFT) {
				trig = (int)(base + pre);          /* free run */
			} else if (cfg.trig.mode == 0) {
				/*
				 * AUTO means free-running sweep when nothing
				 * crosses the level - that is how the noise and
				 * the mains pickup on an open input become
				 * visible. The old code gated this on the time
				 * since the LAST FRAME, and since auto frames
				 * refreshed that timestamp too, the gate could
				 * hold shut and the screen stayed empty.
				 */
				if ((now - last_free_ms) < 40U) {
					continue;
				}
				last_free_ms = now;
				trig = (int)(base + pre);
				auto_sweep = true;
			} else {
				continue;                          /* NORMAL: wait */
			}
		}

		/* holdoff: ignore triggers that arrive too soon after the last */
		if (cfg.trig.holdoff_us > 1000U && !auto_sweep) {
			static uint32_t prev_ms;

			if ((now - prev_ms) < cfg.trig.holdoff_us / 1000U) {
				continue;
			}
			prev_ms = now;
		}

		struct dso_frame *f;

		if (k_mem_slab_alloc(&frame_slab, (void **)&f, K_NO_WAIT) != 0) {
			continue;      /* UI is behind: drop the frame, keep sampling */
		}

		f->n = (uint16_t)want;
		f->nch = (cfg.ch[1].on ? 2 : 1);
		f->triggered = !auto_sweep;
		f->sample_rate = cur_sample_rate;

		for (uint8_t ch = 0; ch < f->nch; ch++) {
			acq_extract(ring[ch], ADC_RING_LEN, (size_t)trig, pre,
				    f->s[ch], want);
		}

		if (k_msgq_put(&frame_q, &f, K_NO_WAIT) != 0) {
			k_mem_slab_free(&frame_slab, (void *)f);
		}

		if (cfg.trig.mode == 2 && f->triggered) {     /* SINGLE */
			ui_lock();
			ui_cfg()->running = false;
			ui_unlock();
		}
	}
}

K_THREAD_STACK_DEFINE(acq_stack, 2048);
static struct k_thread acq_thread_data;

uint32_t acq_overruns(void)
{
	return overrun_cnt;
}

struct dso_frame *acq_take_frame(int timeout_ms)
{
	struct dso_frame *f;

	if (k_msgq_get(&frame_q, &f, K_MSEC(timeout_ms)) == 0) {
		return f;
	}
	return NULL;
}

void acq_release_frame(struct dso_frame *f)
{
	k_mem_slab_free(&frame_slab, (void *)f);
}

int acq_init(void)
{
	k_mem_slab_init(&frame_slab, frame_pool, sizeof(struct dso_frame),
			FRAME_SLOTS);

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC3);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

	gpio_analog(GPIOA, LL_GPIO_PIN_0);    /* CH1, Arduino A0, ADC123_IN0 */
	gpio_analog(GPIOF, LL_GPIO_PIN_10);   /* CH2, Arduino A1, ADC3_IN8   */

	LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADC1),
			      LL_ADC_CLOCK_SYNC_PCLK_DIV4);

	/* TIM2 only produces TRGO; nothing is routed to a pin */
	LL_TIM_SetPrescaler(TIM2, 0);
	LL_TIM_SetAutoReload(TIM2, 59);        /* 1.8 MSa/s until reprogrammed */
	LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_UPDATE);
	LL_TIM_EnableARRPreload(TIM2);
	LL_TIM_GenerateEvent_UPDATE(TIM2);

	dma_setup(LL_DMA_STREAM_4, LL_DMA_CHANNEL_0,
		  LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
		  ring[0]);
	dma_setup(LL_DMA_STREAM_1, LL_DMA_CHANNEL_2,
		  LL_ADC_DMA_GetRegAddr(ADC3, LL_ADC_DMA_REG_REGULAR_DATA),
		  ring[1]);

	LL_DMA_EnableIT_HT(DMA2, LL_DMA_STREAM_4);
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_4);
	LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_4);

	IRQ_CONNECT(DMA2_Stream4_IRQn, 1, dma_ch1_isr, NULL, 0);
	irq_enable(DMA2_Stream4_IRQn);

	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_4);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_1);

	adc_setup(ADC1, LL_ADC_CHANNEL_0);
	adc_setup(ADC3, LL_ADC_CHANNEL_8);
	k_busy_wait(10);                       /* tSTAB after ADC enable */

	/* Highest PREEMPTIBLE priority, not cooperative. A cooperative thread
	 * that blocks on the settings mutex boosts the UI thread to cooperative
	 * while it holds the lock, and then nothing - flush, input, logging -
	 * can run until the lock is released. Preemptible keeps the trigger
	 * search prompt without ever freezing the rest of the system. */
	k_thread_create(&acq_thread_data, acq_stack, K_THREAD_STACK_SIZEOF(acq_stack),
			acq_thread, NULL, NULL, NULL, 1, 0, K_NO_WAIT);
	k_thread_name_set(&acq_thread_data, "acq");

	LOG_INF("acquisition ready, max %u Sa/s per channel", ADC_MAX_SPS);
	return 0;
}
