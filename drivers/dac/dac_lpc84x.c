/*
 * Copyright (c) 2026 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_lpc84x_dac

#include <zephyr/kernel.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>

#include <fsl_dac.h>
#include <fsl_power.h>

LOG_MODULE_REGISTER(dac_mcux_lpc, CONFIG_DAC_LOG_LEVEL);

struct lpc84x_dac_config {
	DEVICE_MMIO_ROM;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	const struct pinctrl_dev_config *pincfg;
};

struct lpc84x_dac_data {
	DEVICE_MMIO_RAM;
	bool configured;
};

#define DEV_BASE(dev) ((DAC_Type *)DEVICE_MMIO_GET(dev))

static int lpc84x_dac_channel_setup(const struct device *dev,
				    const struct dac_channel_cfg *channel_cfg)
{
	struct lpc84x_dac_data *data = dev->data;

	if (channel_cfg->channel_id != 0) {
		LOG_ERR("unsupported channel %d", channel_cfg->channel_id);
		return -ENOTSUP;
	}

	if (channel_cfg->resolution != 10) {
		LOG_ERR("unsupported resolution %d (only 10-bit supported)",
			channel_cfg->resolution);
		return -ENOTSUP;
	}

	if (channel_cfg->internal) {
		LOG_ERR("Internal channels not supported");
		return -ENOTSUP;
	}

	data->configured = true;

	return 0;
}

static int lpc84x_dac_write_value(const struct device *dev, uint8_t channel, uint32_t value)
{
	struct lpc84x_dac_data *data = dev->data;

	if (!data->configured) {
		LOG_ERR("channel not initialized");
		return -EINVAL;
	}

	if (channel != 0) {
		LOG_ERR("unsupported channel %d", channel);
		return -ENOTSUP;
	}

	if (value >= 1024) {
		LOG_ERR("unsupported value %d (max is 1023)", value);
		return -EINVAL;
	}

	DAC_SetBufferValue(DEV_BASE(dev), value);

	return 0;
}

static int lpc84x_dac_init(const struct device *dev)
{
	const struct lpc84x_dac_config *config = dev->config;
	dac_config_t dac_config;
	int err;

	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR_DEVICE_NOT_READY(config->clock_dev);
		return -ENODEV;
	}

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	err = clock_control_on(config->clock_dev, config->clock_subsys);
	if (err) {
		LOG_ERR("Failed to enable clock (err %d)", err);
		return err;
	}

	err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (err) {
		LOG_ERR("Failed to configure DAC pins via pinctrl");
		return err;
	}

	if (DEV_BASE(dev) == DAC0) {
		POWER_DisablePD(kPDRUNCFG_PD_DAC0);
	} else {
		POWER_DisablePD(kPDRUNCFG_PD_DAC1);
	}

	DAC_GetDefaultConfig(&dac_config);
	dac_config.settlingTime = kDAC_SettlingTimeIs1us;

	DAC_Init(DEV_BASE(dev), &dac_config);

	return 0;
}

static DEVICE_API(dac, lpc84x_dac_driver_api) = {
	.channel_setup = lpc84x_dac_channel_setup,
	.write_value = lpc84x_dac_write_value,
};

#define LPC84X_DAC_INIT(n)                                                                         \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static struct lpc84x_dac_data lpc84x_dac_data_##n;                                         \
                                                                                                   \
	static const struct lpc84x_dac_config lpc84x_dac_config_##n = {                            \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                              \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, name),              \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                       \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, lpc84x_dac_init, NULL, &lpc84x_dac_data_##n,                      \
			      &lpc84x_dac_config_##n, POST_KERNEL, CONFIG_DAC_INIT_PRIORITY,       \
			      &lpc84x_dac_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LPC84X_DAC_INIT)
