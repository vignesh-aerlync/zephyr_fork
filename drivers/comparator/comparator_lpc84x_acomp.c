/*
 * Copyright (c) 2026 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_lpc84x_acomp

#include <zephyr/sys/atomic.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/drivers/comparator.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>

#include <fsl_acomp.h>
#include <fsl_power.h>

LOG_MODULE_REGISTER(nxp_lpc84x_acomp, CONFIG_COMPARATOR_LOG_LEVEL);

#define LPC84X_ACOMP_LADDER_VREF_VDD    0U
#define LPC84X_ACOMP_LADDER_VREF_VDDCMP 1U

struct lpc84x_acomp_config {
	DEVICE_MMIO_ROM;
	const struct pinctrl_dev_config *pincfg;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	uint8_t positive_input;
	uint8_t negative_input;
	uint8_t hysteresis;
	bool ladder_enable;
	uint8_t ladder_value;
	uint8_t ladder_vref;
	void (*irq_config_func)(const struct device *dev);
	uint32_t irq;
};

struct lpc84x_acomp_data {
	DEVICE_MMIO_RAM;
	comparator_callback_t callback;
	void *user_data;
	enum comparator_trigger trigger;
	atomic_t pending;
};

#define DEV_BASE(dev) ((ACOMP_Type *)DEVICE_MMIO_GET(dev))

static int lpc84x_acomp_get_output(const struct device *dev)
{
	ACOMP_Type *base = DEV_BASE(dev);
	uint32_t ctrl = base->CTRL;
	int out = (ctrl & ACOMP_CTRL_COMPSTAT_MASK) ? 1 : 0;

	return out;
}

static int lpc84x_acomp_set_trigger(const struct device *dev, enum comparator_trigger trigger)
{
	const struct lpc84x_acomp_config *config = dev->config;
	struct lpc84x_acomp_data *data = dev->data;
	ACOMP_Type *base = DEV_BASE(dev);

	ACOMP_EnableInterrupts(base, kACOMP_InterruptsDisable);

	ACOMP_ClearInterruptsStatusFlags(base);

	data->trigger = trigger;
	atomic_set(&data->pending, 0);

	switch (trigger) {
	case COMPARATOR_TRIGGER_NONE:
		irq_disable(config->irq);
		return 0;

	case COMPARATOR_TRIGGER_RISING_EDGE:
		ACOMP_EnableInterrupts(base, kACOMP_InterruptsRisingEdgeEnable);
		break;

	case COMPARATOR_TRIGGER_FALLING_EDGE:
		ACOMP_EnableInterrupts(base, kACOMP_InterruptsFallingEdgeEnable);
		break;

	case COMPARATOR_TRIGGER_BOTH_EDGES:
		ACOMP_EnableInterrupts(base, kACOMP_InterruptsBothEdgesEnable);
		break;

	default:
		return -ENOTSUP;
	}

	ACOMP_ClearInterruptsStatusFlags(base);

	NVIC_ClearPendingIRQ(config->irq);

	irq_enable(config->irq);

	return 0;
}

static int lpc84x_acomp_set_trigger_callback(const struct device *dev,
					     comparator_callback_t callback, void *user_data)
{
	const struct lpc84x_acomp_config *config = dev->config;
	struct lpc84x_acomp_data *data = dev->data;

	data->callback = callback;
	data->user_data = user_data;

	if (callback != NULL && data->trigger != COMPARATOR_TRIGGER_NONE) {
		irq_enable(config->irq);
	} else {
		irq_disable(config->irq);
	}

	return 0;
}

static int lpc84x_acomp_trigger_is_pending(const struct device *dev)
{
	struct lpc84x_acomp_data *data = dev->data;

	/* Atomically compare and swap: if pending is 1, set to 0 and return true */
	return atomic_cas(&data->pending, 1, 0) ? 1 : 0;
}

static void lpc84x_acomp_irq_handler(const struct device *dev)
{
	struct lpc84x_acomp_data *data = dev->data;
	ACOMP_Type *base = DEV_BASE(dev);

	ACOMP_ClearInterruptsStatusFlags(base);

	if (data->callback != NULL) {
		atomic_set(&data->pending, 0);
		data->callback(dev, data->user_data);
	} else {
		atomic_set(&data->pending, 1);
	}
}

static int lpc84x_acomp_init(const struct device *dev)
{
	const struct lpc84x_acomp_config *config = dev->config;
	struct lpc84x_acomp_data *data = dev->data;
	ACOMP_Type *base;
	acomp_config_t acomp_cfg;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	base = DEV_BASE(dev);

	data->trigger = COMPARATOR_TRIGGER_NONE;
	atomic_set(&data->pending, 0);

	/* Power up the analog comparator by disabling its power down mode */
	POWER_DisablePD(kPDRUNCFG_PD_ACMP);

	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR_DEVICE_NOT_READY(config->clock_dev);
		return -ENODEV;
	}

	int err = clock_control_on(config->clock_dev, config->clock_subsys);

	if (err < 0) {
		LOG_ERR("Failed to enable ACOMP clock (err %d)", err);
		return err;
	}

	int ret = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);

	if (ret < 0 && ret != -ENOENT) {
		LOG_ERR("Failed to apply pinctrl state (err %d)", ret);
		return ret;
	}

	ACOMP_GetDefaultConfig(&acomp_cfg);

	acomp_cfg.hysteresisSelection = (acomp_hysteresis_selection_t)config->hysteresis;

	ACOMP_Init(base, &acomp_cfg);

	if (config->ladder_enable) {
		acomp_ladder_config_t ladder_cfg;

		ladder_cfg.ladderValue = config->ladder_value;

		/* Select reference voltage VDD or VDDCMP */
		if (config->ladder_vref == LPC84X_ACOMP_LADDER_VREF_VDD) {
			ladder_cfg.referenceVoltage = kACOMP_LadderRefVoltagePinVDD;
		} else {
			ladder_cfg.referenceVoltage = kACOMP_LadderRefVoltagePinVDDCMP;
		}

		ACOMP_SetLadderConfig(base, &ladder_cfg);
	}

	ACOMP_SetInputChannel(base, config->positive_input, config->negative_input);

	LOG_DBG("ACOMP channels: positive=%u, negative=%u", config->positive_input,
		config->negative_input);

	config->irq_config_func(dev);

	ACOMP_EnableInterrupts(base, kACOMP_InterruptsDisable);

	ACOMP_ClearInterruptsStatusFlags(base);

	return 0;
}

static DEVICE_API(comparator, lpc84x_acomp_api) = {
	.get_output = lpc84x_acomp_get_output,
	.set_trigger = lpc84x_acomp_set_trigger,
	.set_trigger_callback = lpc84x_acomp_set_trigger_callback,
	.trigger_is_pending = lpc84x_acomp_trigger_is_pending,
};

#define LPC84X_ACOMP_INIT(inst)                                                                    \
	BUILD_ASSERT(DT_INST_PROP_OR(inst, ladder_value, 0) <= 31, "ladder value should be 0-31"); \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
                                                                                                   \
	static void lpc84x_acomp_irq_config_##inst(const struct device *dev)                       \
	{                                                                                          \
		ARG_UNUSED(dev);                                                                   \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority),                       \
			    lpc84x_acomp_irq_handler, DEVICE_DT_INST_GET(inst), 0);                \
	}                                                                                          \
                                                                                                   \
	static struct lpc84x_acomp_data lpc84x_acomp_data_##inst;                                  \
                                                                                                   \
	static const struct lpc84x_acomp_config lpc84x_acomp_config_##inst = {                     \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),                                           \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                    \
		.clock_dev = DEVICE_DT_GET(DT_INST_PHANDLE(inst, clocks)),                         \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(inst, name),           \
		.positive_input = DT_INST_PROP(inst, positive_input),                              \
		.negative_input = DT_INST_PROP(inst, negative_input),                              \
		.hysteresis = DT_INST_PROP(inst, hysteresis),                                      \
		.ladder_enable = DT_INST_PROP(inst, ladder_enable),                                \
		.ladder_value = DT_INST_PROP(inst, ladder_value),                                  \
		.ladder_vref = DT_INST_PROP(inst, ladder_vref),                                    \
		.irq_config_func = lpc84x_acomp_irq_config_##inst,                                 \
		.irq = DT_INST_IRQN(inst),                                                         \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, lpc84x_acomp_init, NULL, &lpc84x_acomp_data_##inst,            \
			      &lpc84x_acomp_config_##inst, POST_KERNEL,                            \
			      CONFIG_COMPARATOR_INIT_PRIORITY, &lpc84x_acomp_api);

DT_INST_FOREACH_STATUS_OKAY(LPC84X_ACOMP_INIT)
