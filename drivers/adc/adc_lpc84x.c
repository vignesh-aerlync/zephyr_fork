/*
 * Copyright (c) 2026 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_lpc84x_adc

#include <zephyr/irq.h>
#include <zephyr/sys/util.h>

#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>

#include <fsl_adc.h>
#include <fsl_power.h>

LOG_MODULE_REGISTER(adc_lpc84x, CONFIG_ADC_LOG_LEVEL);

#define ADC_CONTEXT_USES_KERNEL_TIMER
#define ADC_CONTEXT_ENABLE_ON_COMPLETE
#include "adc_context.h"

/* Maximum number of ADC channels supported */
#define ADC_MAX_CHANNEL ADC_DAT_COUNT

/* ADC resolution in bits */
#define LPC84X_ADC_RESOLUTION_BITS 12U

/* Minimum clock divider value */
#define LPC84X_MIN_CLOCK_DIVIDER 1U

/* Maximum clock divider value */
#define LPC84X_MAX_CLOCK_DIVIDER 256U

/* Default internal reference voltage in mV */
#define LPC84X_DEFAULT_REF_INTERNAL_MV 3300U

BUILD_ASSERT(DT_INST_PROP(0, clk_divider) >= LPC84X_MIN_CLOCK_DIVIDER &&
		     DT_INST_PROP(0, clk_divider) <= LPC84X_MAX_CLOCK_DIVIDER,
	     "ADC clock divider in DTS is outside the hardware limits!");

struct lpc84x_adc_config {
	DEVICE_MMIO_ROM;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config_func)(const struct device *dev);
	uint32_t clk_divider;
	bool burst;
	uint16_t ref_internal_mv;
};

struct lpc84x_adc_data {
	DEVICE_MMIO_RAM;
	adc_config_t adc_config;
	adc_conv_seq_config_t adc_seqconfig;
	const struct device *dev;
	struct adc_context ctx;
	uint16_t *buffer;
	uint16_t *repeat_buffer;
	uint16_t *buffer_end;
	uint32_t channels;
	uint32_t configured_channels;
	int conversion_status;
};

#define DEV_BASE(dev) ((ADC_Type *)DEVICE_MMIO_GET(dev))

/**
 * @brief Start ADC conversion on configured channels
 *
 * This function configures and starts the ADC conversion sequence.
 * It supports both burst mode (continuous sampling) and software-triggered mode.
 *
 * @param dev ADC device pointer
 */
static void lpc84x_adc_start_channel(const struct device *dev)
{
	const struct lpc84x_adc_config *config = dev->config;
	struct lpc84x_adc_data *data = dev->data;
	ADC_Type *base = DEV_BASE(dev);

	/* Disable sequence before configuring */
	ADC_EnableConvSeqA(base, false);

	/* Clear any pending flags, specifically overruns, to prevent sequencer stalls */
	ADC_ClearStatusFlags(base, ADC_GetStatusFlags(base));

	/* Configure the sequence */
	data->adc_seqconfig.channelMask = data->channels;
	data->adc_seqconfig.triggerMask = 0U; /* Software trigger */
	data->adc_seqconfig.triggerPolarity = kADC_TriggerPolarityPositiveEdge;
	data->adc_seqconfig.enableSingleStep = false;
	data->adc_seqconfig.enableSyncBypass = false;
	data->adc_seqconfig.interruptMode = kADC_InterruptForEachSequence;

	ADC_SetConvSeqAConfig(base, &data->adc_seqconfig);

	/* Enable sequence */
	ADC_EnableConvSeqA(base, true);

	if (config->burst) {
		/* Burst mode: ADC continuously samples selected channels */
		ADC_EnableConvSeqABurstMode(base, true);
	} else {
		/* Software-triggered mode: single conversion per trigger */
		ADC_DoSoftwareTriggerConvSeqA(base);
	}
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct lpc84x_adc_data *data = CONTAINER_OF(ctx, struct lpc84x_adc_data, ctx);

	data->channels = ctx->sequence.channels;
	data->repeat_buffer = data->buffer;
	data->conversion_status = 0;

	lpc84x_adc_start_channel(data->dev);
}

static int lpc84x_adc_channel_setup(const struct device *dev,
				    const struct adc_channel_cfg *channel_cfg)
{
	uint8_t ch = channel_cfg->channel_id;
	struct lpc84x_adc_data *data = dev->data;

	if (ch >= ADC_MAX_CHANNEL) {
		LOG_ERR("Invalid channel ID: %u (max: %u)", ch, ADC_MAX_CHANNEL - 1);
		return -EINVAL;
	}

	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		LOG_ERR("Unsupported channel acquisition time");
		return -ENOTSUP;
	}

	if (channel_cfg->differential) {
		LOG_ERR("Differential mode is not supported");
		return -ENOTSUP;
	}

	if (channel_cfg->reference != ADC_REF_INTERNAL && channel_cfg->reference != ADC_REF_VDD_1) {
		LOG_ERR("Invalid reference: %d. Only ADC_REF_INTERNAL and ADC_REF_VDD_1 are "
			"supported.",
			channel_cfg->reference);
		return -EINVAL;
	}

	if (channel_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("Invalid gain: %d. Only ADC_GAIN_1 is supported.", channel_cfg->gain);
		return -EINVAL;
	}

	data->configured_channels |= BIT(ch);

	return 0;
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	struct lpc84x_adc_data *data = CONTAINER_OF(ctx, struct lpc84x_adc_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

static void lpc84x_adc_isr(const struct device *dev)
{
	struct lpc84x_adc_data *data = dev->data;
	ADC_Type *base = DEV_BASE(dev);
	adc_result_info_t result_info;
	uint32_t flags = ADC_GetStatusFlags(base);
	uint32_t channels = data->channels;

	/* Sequence A conversion complete interrupt */
	if (!(flags & kADC_ConvSeqAInterruptFlag)) {
		return;
	}

	ADC_ClearStatusFlags(base, flags);

	while (channels != 0U) {
		uint32_t ch = find_lsb_set(channels) - 1;

		/* Bounds check: ensure buffer pointer hasn't exceeded allocated space */
		if (data->buffer >= data->buffer_end) {
			LOG_ERR("ADC buffer overflow prevented at channel %u", ch);
			data->conversion_status = -ENOMEM;
			channels &= ~BIT(ch);
			break;
		}

		if (ADC_GetChannelConversionResult(base, ch, &result_info)) {
			*data->buffer++ = (uint16_t)result_info.result;
		} else {
			LOG_ERR("Channel %u conversion result read failed", ch);
			*data->buffer++ = 0U;
			data->conversion_status = -EIO;
		}

		channels &= ~BIT(ch);
	}

	adc_context_on_sampling_done(&data->ctx, dev);
}

static void adc_context_on_complete(struct adc_context *ctx, int status)
{
	struct lpc84x_adc_data *data = CONTAINER_OF(ctx, struct lpc84x_adc_data, ctx);
	ADC_Type *base = DEV_BASE(data->dev);
	const struct lpc84x_adc_config *config = data->dev->config;

	ADC_EnableConvSeqA(base, false);

	if (config->burst) {
		ADC_EnableConvSeqABurstMode(base, false);
	}

	/* Report conversion status if any error occurred during sampling */
	if (data->conversion_status != 0) {
		status = data->conversion_status;
		LOG_WRN("ADC conversion completed with error status: %d", status);
	}

	adc_context_release(ctx, status);
}

static int check_buffer_size(const struct adc_sequence *sequence, uint8_t active_channels)
{
	size_t needed_buffer_size;

	needed_buffer_size = active_channels * sizeof(uint16_t);
	if (sequence->options) {
		needed_buffer_size *= (1 + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < needed_buffer_size) {
		LOG_ERR("Buffer size insufficient: need %zu bytes, got %u bytes",
			needed_buffer_size, sequence->buffer_size);
		return -ENOMEM;
	}

	return 0;
}

static int lpc84x_adc_read_common(const struct device *dev, const struct adc_sequence *sequence,
				  struct k_poll_signal *async)
{
	struct lpc84x_adc_data *data = dev->data;
	int err;
	uint8_t num_active_channels = (uint8_t)POPCOUNT(sequence->channels);

	if (sequence->buffer == NULL) {
		LOG_ERR("NULL buffer provided");
		return -EINVAL;
	}

	if (sequence->channels == 0) {
		LOG_ERR("No channels selected for conversion");
		return -EINVAL;
	}

	if ((sequence->channels & ~BIT_MASK(ADC_MAX_CHANNEL))) {
		LOG_ERR("Invalid channels mask 0x%x (max mask: 0x%lx)", sequence->channels,
			BIT_MASK(ADC_MAX_CHANNEL) - 1);
		return -EINVAL;
	}

	if ((sequence->channels & ~data->configured_channels) != 0) {
		LOG_ERR("Requested channels not configured (configured: 0x%x, requested: 0x%x)",
			data->configured_channels, sequence->channels);
		return -EINVAL;
	}

	err = check_buffer_size(sequence, num_active_channels);
	if (err) {
		return err;
	}

	if (sequence->resolution != LPC84X_ADC_RESOLUTION_BITS) {
		LOG_ERR("Unsupported resolution %d. Only %d-bit is supported on LPC84x.",
			sequence->resolution, LPC84X_ADC_RESOLUTION_BITS);
		return -EINVAL;
	}

	if (sequence->oversampling) {
		LOG_ERR("Unsupported oversampling");
		return -EINVAL;
	}

	adc_context_lock(&data->ctx, async != NULL, async);
	data->buffer = sequence->buffer;
	data->buffer_end = (uint16_t *)((uint8_t *)sequence->buffer + sequence->buffer_size);
	adc_context_start_read(&data->ctx, sequence);

	return adc_context_wait_for_completion(&data->ctx);
}

static int lpc84x_adc_read(const struct device *dev, const struct adc_sequence *sequence)
{
	return lpc84x_adc_read_common(dev, sequence, NULL);
}

#ifdef CONFIG_ADC_ASYNC
static int lpc84x_adc_read_async(const struct device *dev, const struct adc_sequence *sequence,
				 struct k_poll_signal *async)
{
	return lpc84x_adc_read_common(dev, sequence, async);
}
#endif

static int lpc84x_adc_init(const struct device *dev)
{
	const struct lpc84x_adc_config *config = dev->config;
	struct lpc84x_adc_data *data = dev->data;
	uint32_t clock_freq = 0;
	uint32_t adc_clock;
	int err;

	data->dev = dev;
	data->conversion_status = 0;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR_DEVICE_NOT_READY(config->clock_dev);
		return -ENODEV;
	}

	err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		LOG_ERR("Failed to apply pinctrl state (err %d)", err);
		return err;
	}

	err = clock_control_on(config->clock_dev, config->clock_subsys);
	if (err) {
		LOG_ERR("Failed to enable ADC clock (err %d)", err);
		return err;
	}

	err = clock_control_get_rate(config->clock_dev, config->clock_subsys, &clock_freq);
	if (err) {
		LOG_ERR("Failed to get clock rate (err %d)", err);
		return err;
	}

	if (clock_freq == 0U) {
		LOG_ERR("ADC clock rate is 0 Hz — check DTS clocks cell");
		return -EINVAL;
	}

	/* *
	 * The TRM register must be configured for the appropriate operating range
	 * of the analog supply voltage VDDA/VREFP.
	 * High range: 2.7V to 3.6V, Low range: 2.4V to 2.7V.
	 */
	if (config->ref_internal_mv < 2700) {
		data->adc_config.voltageRange = kADC_LowVoltageRange;
	} else {
		data->adc_config.voltageRange = kADC_HighVoltageRange;
	}

	if (config->ref_internal_mv < 2400) {
		LOG_WRN("VREFP (%u mV) is below the minimum supported 2400 mV",
			config->ref_internal_mv);
	}

	/* *
	 * Power up the ADC peripheral.
	 * By default, the ADC_PD bit in PDRUNCFG is 1 (Powered down).
	 */
	POWER_DisablePD(kPDRUNCFG_PD_ADC0);

	/* Initialize ADC sequence config structure to zero */
	memset(&data->adc_seqconfig, 0, sizeof(data->adc_seqconfig));

	ADC_GetDefaultConfig(&data->adc_config);
	data->adc_config.clockDividerNumber = config->clk_divider - 1;

	ADC_Init(DEV_BASE(dev), &data->adc_config);

	adc_clock = clock_freq / config->clk_divider;

	if (!ADC_DoSelfCalibration(DEV_BASE(dev), adc_clock)) {
		LOG_ERR("ADC self-calibration failed");
		POWER_EnablePD(kPDRUNCFG_PD_ADC0);
		return -EIO;
	}

	config->irq_config_func(dev);

	ADC_EnableInterrupts(DEV_BASE(dev), kADC_ConvSeqAInterruptEnable);

	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}

#define LPC84X_ADC_INIT(inst)                                                                      \
	static DEVICE_API(adc, lpc84x_adc_api_##inst) = {                                          \
		.channel_setup = lpc84x_adc_channel_setup,                                         \
		.read = lpc84x_adc_read,                                                           \
		.ref_internal = DT_INST_PROP(inst, nxp_ref_internal_mv),                           \
		IF_ENABLED(CONFIG_ADC_ASYNC,							   \
				(.read_async = lpc84x_adc_read_async,))};                \
                                                                                                   \
	static void lpc84x_adc_irq_config_##inst(const struct device *dev)                         \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), lpc84x_adc_isr,       \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	}                                                                                          \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	static const struct lpc84x_adc_config lpc84x_adc_config_##inst = {                         \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),                                           \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                             \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(inst, name),           \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
		.irq_config_func = lpc84x_adc_irq_config_##inst,                                   \
		.clk_divider = DT_INST_PROP(inst, clk_divider),                                    \
		.burst = DT_INST_PROP(inst, nxp_burst_mode),                                       \
		.ref_internal_mv = DT_INST_PROP_OR(inst, nxp_ref_internal_mv,                      \
						   LPC84X_DEFAULT_REF_INTERNAL_MV),                \
	};                                                                                         \
                                                                                                   \
	static struct lpc84x_adc_data lpc84x_adc_data_##inst = {                                   \
		ADC_CONTEXT_INIT_TIMER(lpc84x_adc_data_##inst, ctx),                               \
		ADC_CONTEXT_INIT_LOCK(lpc84x_adc_data_##inst, ctx),                                \
		ADC_CONTEXT_INIT_SYNC(lpc84x_adc_data_##inst, ctx),                                \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, lpc84x_adc_init, NULL, &lpc84x_adc_data_##inst,                \
			      &lpc84x_adc_config_##inst, POST_KERNEL, CONFIG_ADC_INIT_PRIORITY,    \
			      &lpc84x_adc_api_##inst);

DT_INST_FOREACH_STATUS_OKAY(LPC84X_ADC_INIT)
