/*
 * Copyright (c) 2026 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_lpc845_captouch

#include <zephyr/device.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/input/input.h>
#include <zephyr/sys/atomic.h>

#include <fsl_capt.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(input_lpc845_cap_touch, CONFIG_INPUT_LOG_LEVEL);

#define LPC845_CAPT_MAX_PINS 9U

struct lpc845_capt_config {
	DEVICE_MMIO_ROM;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	const struct pinctrl_dev_config *pcfg;
	const uint8_t *xpins;
	uint8_t xpins_len;
	uint16_t threshold;
	uint16_t poll_count;
	bool touch_lower;
	uint8_t clock_divider;
	uint8_t inactive_xpins_mode;
	uint8_t measurement_delay;
	uint8_t reset_delay;
	void (*irq_config_func)(const struct device *dev);
	const struct device *comparator;
};

/*
 * Raw capacitance measurement count and touch detection status
 * for each configured CAPT X pin.
 */
struct lpc845_capt_pin_result {
	uint16_t count;
	bool yes_touch;
};

struct lpc845_capt_data {
	DEVICE_MMIO_RAM;
	const struct device *dev;
	struct k_work_delayable work;
	atomic_t poll_round_done;
	struct lpc845_capt_pin_result pin_data[LPC845_CAPT_MAX_PINS];
};

#define DEV_BASE(dev) ((CAPT_Type *)DEVICE_MMIO_GET(dev))

static void lpc845_capt_isr(const struct device *dev)
{
#ifdef CONFIG_INPUT_LPC845_CAP_TOUCH_INTERRUPT_MODE
	struct lpc845_capt_data *data = dev->data;
	uint32_t status;
	capt_touch_data_t touch;

	status = CAPT_GetInterruptStatusFlags(DEV_BASE(dev));
	CAPT_ClearInterruptStatusFlags(DEV_BASE(dev), status);

	/* capture raw counts and yes/no touch flag for each active X pin */
	if (status & (kCAPT_InterruptOfYesTouchStatusFlag | kCAPT_InterruptOfNoTouchStatusFlag |
		      kCAPT_InterruptOfTimeOutStatusFlag)) {
		if (CAPT_GetTouchData(DEV_BASE(dev), &touch)) {
			if (touch.XpinsIndex < LPC845_CAPT_MAX_PINS) {
				data->pin_data[touch.XpinsIndex].count = touch.count;
				data->pin_data[touch.XpinsIndex].yes_touch = touch.yesTouch;
			}
		}
	}

	if (status & kCAPT_InterruptOfPollDoneStatusFlag) {
		atomic_set(&data->poll_round_done, 1);
		k_work_schedule(&data->work, K_NO_WAIT);
	}
#else
	ARG_UNUSED(dev);
#endif
}

static void capt_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct lpc845_capt_data *data = CONTAINER_OF(dwork, struct lpc845_capt_data, work);

	atomic_set(&data->poll_round_done, 0);

	const struct lpc845_capt_config *cfg = data->dev->config;

	for (uint8_t i = 0; i < cfg->xpins_len; i++) {
		uint8_t pin = cfg->xpins[i];

		if (pin < LPC845_CAPT_MAX_PINS) {
			LOG_INF("CAPT X%d: Count=%u, Touch=%d", pin, data->pin_data[pin].count,
				data->pin_data[pin].yes_touch ? 1 : 0);

			input_report_key(data->dev, INPUT_BTN_0 + pin,
					 data->pin_data[pin].yes_touch ? 1 : 0, true, K_FOREVER);
		}
	}

#ifdef CONFIG_INPUT_LPC845_CAP_TOUCH_POLLING_MODE
	k_work_reschedule(dwork, K_MSEC(CONFIG_INPUT_LPC845_CAP_TOUCH_POLL_INTERVAL_MS));
#else
	(void)dwork;
#endif
}

static int lpc845_capt_init(const struct device *dev)
{
	const struct lpc845_capt_config *config = dev->config;
	struct lpc845_capt_data *data = dev->data;
	capt_config_t captConfig;
	uint16_t enable_pins = 0;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	data->dev = dev;

	/* Reset the poll-round-done flag */
	atomic_set(&data->poll_round_done, 0);

	memset(data->pin_data, 0, sizeof(data->pin_data));

	k_work_init_delayable(&data->work, capt_work_handler);

	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR_DEVICE_NOT_READY(config->clock_dev);
		return -ENODEV;
	}

	if (config->xpins_len == 0U) {
		LOG_ERR("No CAPT X pins configured");
		return -EINVAL;
	}

	int err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);

	if (err != 0) {
		LOG_ERR("Failed to enable pin");
		return err;
	}

	int ret = clock_control_on(config->clock_dev, config->clock_subsys);

	if (ret != 0) {
		LOG_ERR("Failed to enable CAPT clock");
		return ret;
	}

	if (config->threshold > 4095U) {
		LOG_ERR("Invalid CAPT threshold %u (valid range is 0 to 4095)", config->threshold);
		return -EINVAL;
	}

	CAPT_GetDefaultConfig(&captConfig);

	captConfig.clockDivider = config->clock_divider;

	for (uint8_t i = 0; i < config->xpins_len; i++) {
		enable_pins |= BIT(config->xpins[i]);
	}

	captConfig.enableXpins = enable_pins;

	captConfig.pollCount = config->poll_count;
	captConfig.enableTouchLower = config->touch_lower;

	if (IS_ENABLED(CONFIG_INPUT_LPC845_CAP_TOUCH_ACOMP_TRIGGER) &&
	    (config->comparator != NULL)) {
		if (!device_is_ready(config->comparator)) {
			LOG_ERR_DEVICE_NOT_READY(config->comparator);
			return -ENODEV;
		}
		captConfig.triggerMode = kCAPT_ComparatorTriggerMode;
		captConfig.XpinsMode = kCAPT_InactiveXpinsHighZMode;
		LOG_INF("CAPT Comparator Trigger Mode");
	} else {
		captConfig.triggerMode = kCAPT_YHPortTriggerMode;
		captConfig.XpinsMode = (config->inactive_xpins_mode == 0)
					       ? kCAPT_InactiveXpinsHighZMode
					       : kCAPT_InactiveXpinsDrivenLowMode;
		LOG_INF("CAPT YHPort Mode");
	}

	/* Measurement delay switch */
	switch (config->measurement_delay) {
	case 1:
		captConfig.mDelay = kCAPT_MeasureDelayWait3FCLKs;
		break;
	case 2:
		captConfig.mDelay = kCAPT_MeasureDelayWait5FCLKs;
		break;
	case 3:
		captConfig.mDelay = kCAPT_MeasureDelayWait9FCLKs;
		break;
	default:
		captConfig.mDelay = kCAPT_MeasureDelayNoWait;
		break;
	}

	/* Reset delay switch */
	switch (config->reset_delay) {
	case 0:
		captConfig.rDelay = kCAPT_ResetDelayNoWait;
		break;
	case 1:
		captConfig.rDelay = kCAPT_ResetDelayWait3FCLKs;
		break;
	case 2:
		captConfig.rDelay = kCAPT_ResetDelayWait5FCLKs;
		break;
	default:
		captConfig.rDelay = kCAPT_ResetDelayWait9FCLKs;
		break;
	}

	CAPT_Init(DEV_BASE(dev), &captConfig);

	CAPT_SetThreshold(DEV_BASE(dev), config->threshold);

#ifdef CONFIG_INPUT_LPC845_CAP_TOUCH_INTERRUPT_MODE
	config->irq_config_func(dev);

	CAPT_EnableInterrupts(DEV_BASE(dev), kCAPT_InterruptOfYesTouchEnable |
						     kCAPT_InterruptOfNoTouchEnable |
						     kCAPT_InterruptOfTimeOutEnable |
						     kCAPT_InterruptOfPollDoneEnable);
	/*
	 * Hardware must continuously scan to measure capacitance.
	 */
	CAPT_SetPollMode(DEV_BASE(dev), kCAPT_PollContinuousMode);
#else
	CAPT_DisableInterrupts(DEV_BASE(dev), kCAPT_InterruptOfYesTouchEnable |
						      kCAPT_InterruptOfNoTouchEnable |
						      kCAPT_InterruptOfTimeOutEnable |
						      kCAPT_InterruptOfPollDoneEnable);
	CAPT_SetPollMode(DEV_BASE(dev), kCAPT_PollContinuousMode);
	k_work_reschedule(&data->work, K_NO_WAIT);
#endif

	return 0;
}

#define CHECK_XPIN(node_id, prop, idx)                                                             \
	BUILD_ASSERT(DT_PROP_BY_IDX(node_id, prop, idx) < LPC845_CAPT_MAX_PINS,                    \
		     "CAPT X pin index out of range");

#ifdef CONFIG_INPUT_LPC845_CAP_TOUCH_ACOMP_TRIGGER
#define CAPT_GET_COMPARATOR(n)                                                                     \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, comparator), \
		(DEVICE_DT_GET(DT_INST_PHANDLE(n, comparator))), NULL)
#else
#define CAPT_GET_COMPARATOR(n) NULL
#endif

#define LPC845_CAPT_INIT(n)                                                                        \
	DT_INST_FOREACH_PROP_ELEM(n, xpins, CHECK_XPIN)                                            \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static void lpc845_capt_irq_cfg_##n(const struct device *dev);                             \
	static const uint8_t xpins_##n[] = DT_INST_PROP(n, xpins);                                 \
	static const struct lpc845_capt_config lpc845_capt_config_##n = {                          \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                              \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, name),              \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.xpins = xpins_##n,                                                                \
		.xpins_len = ARRAY_SIZE(xpins_##n),                                                \
		.threshold = DT_INST_PROP(n, threshold),                                           \
		.poll_count = DT_INST_PROP(n, poll_count),                                         \
		.touch_lower = DT_INST_PROP(n, touch_lower),                                       \
		.clock_divider = DT_INST_PROP(n, clock_divider),                                   \
		.inactive_xpins_mode = DT_INST_PROP(n, inactive_xpins_mode),                       \
		.measurement_delay = DT_INST_PROP(n, measurement_delay),                           \
		.reset_delay = DT_INST_PROP(n, reset_delay),                                       \
		.irq_config_func = lpc845_capt_irq_cfg_##n,                                        \
		.comparator = CAPT_GET_COMPARATOR(n),                                              \
	};                                                                                         \
	static struct lpc845_capt_data lpc845_capt_data_##n;                                       \
	DEVICE_DT_INST_DEFINE(n, lpc845_capt_init, NULL, &lpc845_capt_data_##n,                    \
			      &lpc845_capt_config_##n, POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY,    \
			      NULL);                                                               \
	static void lpc845_capt_irq_cfg_##n(const struct device *dev)                              \
	{                                                                                          \
		ARG_UNUSED(dev);                                                                   \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), lpc845_capt_isr,            \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}
DT_INST_FOREACH_STATUS_OKAY(LPC845_CAPT_INIT)
