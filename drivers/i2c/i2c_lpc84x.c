/*
 * Copyright (c) 2026 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_lpc84x_i2c

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>

#include <fsl_i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(i2c_lpc84x, CONFIG_I2C_LOG_LEVEL);

/* Max wait time (ms) for I2C bus operations */
#define I2C_TRANSFER_TIMEOUT_MS 500

/* Supported I2C speed modes (in Hz) */
#define I2C_STANDARD_BITRATE  (100000)
#define I2C_FAST_BITRATE      (400000)
#define I2C_FAST_PLUS_BITRATE (1000000)

struct lpc84x_i2c_config {
	DEVICE_MMIO_ROM;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	const struct pinctrl_dev_config *pincfg;
	uint32_t bitrate;
	void (*irq_config_func)(const struct device *dev);
};

struct lpc84x_i2c_data {
	DEVICE_MMIO_RAM;
	struct k_sem lock;
	struct k_sem transfer_sem;
	volatile status_t callback_status;
	i2c_master_config_t master_config;
	i2c_master_transfer_t transfer;
	i2c_master_handle_t handle;
};

#define DEV_BASE(dev) ((I2C_Type *)DEVICE_MMIO_GET(dev))

/**
 * @brief Set I2C bus baud rate
 */
static int lpc84x_i2c_set_bitrate(const struct device *dev, uint32_t speed)
{
	const struct lpc84x_i2c_config *config = dev->config;
	uint32_t clock_freq;
	int err;

	err = clock_control_get_rate(config->clock_dev, config->clock_subsys, &clock_freq);
	if (err) {
		LOG_ERR("Failed to get I2C clock rate (err %d)", err);
		return err;
	}

	I2C_MasterSetBaudRate(DEV_BASE(dev), speed, clock_freq);

	return 0;
}

/**
 * @brief Configure I2C bus speed (Standard/Fast/Fast+)
 */
static int lpc84x_i2c_configure(const struct device *dev, uint32_t dev_config)
{
	struct lpc84x_i2c_data *data = dev->data;
	uint32_t baud_rate;
	int ret;

	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		baud_rate = I2C_STANDARD_BITRATE;
		break;
	case I2C_SPEED_FAST:
		baud_rate = I2C_FAST_BITRATE;
		break;
	case I2C_SPEED_FAST_PLUS:
		baud_rate = I2C_FAST_PLUS_BITRATE;
		break;
	case I2C_SPEED_HIGH:
	case I2C_SPEED_ULTRA:
		LOG_ERR("High/Ultra speed not supported on LPC84x");
		return -ENOTSUP;
	default:
		LOG_ERR("Invalid I2C configuration");
		return -EINVAL;
	}

	k_sem_take(&data->lock, K_FOREVER);
	ret = lpc84x_i2c_set_bitrate(dev, baud_rate);
	k_sem_give(&data->lock);

	return ret;
}

/**
 * @brief Validate I2C message parameters
 */
static int lpc84x_i2c_msg_validate(struct i2c_msg *msgs, uint8_t num_msgs)
{
	for (uint8_t i = 0; i < num_msgs; i++) {
		if (msgs[i].flags & I2C_MSG_ADDR_10_BITS) {
			LOG_ERR("Message %u: 10-bit addressing not supported", i);
			return -EINVAL;
		}
		if (msgs[i].len > 0 && msgs[i].buf == NULL) {
			LOG_ERR("Message %u: NULL buffer with non-zero length", i);
			return -EINVAL;
		}
	}
	return 0;
}

/**
 * @brief Convert Zephyr I2C flags to FSL HAL flags
 */
static uint32_t lpc84x_i2c_flags_converter(uint8_t msg_flags)
{
	uint32_t flags = kI2C_TransferDefaultFlag;

	if (!(msg_flags & I2C_MSG_STOP)) {
		flags |= kI2C_TransferNoStopFlag;
	}

	if (msg_flags & I2C_MSG_RESTART) {
		flags |= kI2C_TransferRepeatedStartFlag;
	}

	return flags;
}

/**
 * @brief I2C master transfer completion callback (ISR context)
 */
static void lpc84x_i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status,
				       void *userData)
{
	ARG_UNUSED(base);
	ARG_UNUSED(handle);

	const struct device *dev = (const struct device *)userData;
	struct lpc84x_i2c_data *data = dev->data;

	data->callback_status = status;
	k_sem_give(&data->transfer_sem);
}

/**
 * @brief Perform I2C transfers (master mode)
 *
 * Executes I2C messages with interrupt-driven non-blocking transfers.
 * Returns: 0 on success, -EINVAL for invalid params, -EIO for transfer error, -ETIMEDOUT for
 * timeout
 */
static int lpc84x_i2c_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			       uint16_t addr)
{
	struct lpc84x_i2c_data *data = dev->data;
	status_t status;
	int ret = 0;

	if (!num_msgs) {
		return 0;
	}

	if (lpc84x_i2c_msg_validate(msgs, num_msgs) != 0) {
		LOG_ERR("I2C message validation failed");
		return -EINVAL;
	}

	k_sem_take(&data->lock, K_FOREVER);

	for (uint8_t i = 0; i < num_msgs; i++) {

		data->transfer.flags = lpc84x_i2c_flags_converter(msgs[i].flags);
		data->transfer.slaveAddress = addr;
		data->transfer.direction = (msgs[i].flags & I2C_MSG_READ) ? kI2C_Read : kI2C_Write;
		data->transfer.subaddress = 0;
		data->transfer.subaddressSize = 0;
		data->transfer.data = msgs[i].buf;
		data->transfer.dataSize = msgs[i].len;

		/* Suppress START for continued transfers unless I2C_MSG_RESTART is set */
		if (i != 0 && !(msgs[i].flags & I2C_MSG_RESTART)) {
			data->transfer.flags |= kI2C_TransferNoStartFlag;
		}

		k_sem_reset(&data->transfer_sem);

		status = I2C_MasterTransferNonBlocking(DEV_BASE(dev), &data->handle,
						       &data->transfer);

		if (status != kStatus_Success) {
			LOG_ERR("I2C_MasterTransferNonBlocking failed: status %d", status);
			I2C_MasterTransferAbort(DEV_BASE(dev), &data->handle);
			ret = -EIO;
			break;
		}

		/* Wait for completion with timeout protection (prevents stuck bus deadlock) */
		if (k_sem_take(&data->transfer_sem, K_MSEC(I2C_TRANSFER_TIMEOUT_MS)) != 0) {
			LOG_ERR("I2C transfer timeout (bus may be stuck)");
			I2C_MasterTransferAbort(DEV_BASE(dev), &data->handle);
			ret = -ETIMEDOUT;
			break;
		}

		if (data->callback_status != kStatus_Success) {
			LOG_ERR("I2C transfer failed: HAL status %d", data->callback_status);
			I2C_MasterTransferAbort(DEV_BASE(dev), &data->handle);
			ret = -EIO;
			break;
		}
	}

	k_sem_give(&data->lock);

	return ret;
}

/**
 * @brief Get current I2C bus configuration
 */
static int lpc84x_i2c_get_config(const struct device *dev, uint32_t *dev_config)
{
	struct lpc84x_i2c_data *data = dev->data;
	uint32_t config;

	switch (data->master_config.baudRate_Bps) {
	case I2C_STANDARD_BITRATE:
		config = I2C_SPEED_SET(I2C_SPEED_STANDARD);
		break;
	case I2C_FAST_BITRATE:
		config = I2C_SPEED_SET(I2C_SPEED_FAST);
		break;
	case I2C_FAST_PLUS_BITRATE:
		config = I2C_SPEED_SET(I2C_SPEED_FAST_PLUS);
		break;
	default:
		LOG_ERR("Current baud rate (%u Hz) is not a standard mode",
			data->master_config.baudRate_Bps);
		return -ERANGE;
	}

	*dev_config = config | I2C_MODE_CONTROLLER;
	return 0;
}

/**
 * @brief I2C interrupt service routine
 */
static void lpc84x_i2c_isr(const struct device *dev)
{
	struct lpc84x_i2c_data *data = dev->data;

	I2C_MasterTransferHandleIRQ(DEV_BASE(dev), &data->handle);
}

static int lpc84x_i2c_init(const struct device *dev)
{
	const struct lpc84x_i2c_config *config = dev->config;
	struct lpc84x_i2c_data *data = dev->data;
	uint32_t clock_freq = 0;
	int err;

	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR("Clock control device not ready");
		return -ENODEV;
	}

	err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (err) {
		LOG_ERR("Failed to apply pin control state (err %d)", err);
		return err;
	}

	err = clock_control_on(config->clock_dev, config->clock_subsys);
	if (err) {
		LOG_ERR("Failed to enable I2C clock (err %d)", err);
		return err;
	}

	err = clock_control_get_rate(config->clock_dev, config->clock_subsys, &clock_freq);
	if (err) {
		LOG_ERR("Failed to get I2C clock rate (err %d)", err);
		return err;
	}

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	k_sem_init(&data->lock, 1, 1);
	k_sem_init(&data->transfer_sem, 0, K_SEM_MAX_LIMIT);

	I2C_MasterGetDefaultConfig(&data->master_config);
	data->master_config.baudRate_Bps = config->bitrate;

	I2C_MasterInit(DEV_BASE(dev), &data->master_config, clock_freq);

	config->irq_config_func(dev);

	I2C_MasterTransferCreateHandle(DEV_BASE(dev), &data->handle, lpc84x_i2c_master_callback,
				       (void *)dev);

	return 0;
}

static DEVICE_API(i2c, lpc84x_i2c_driver_api) = {
	.configure = lpc84x_i2c_configure,
	.transfer = lpc84x_i2c_transfer,
	.get_config = lpc84x_i2c_get_config,
};

#define LPC84X_I2C_INIT(n)                                                                         \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	static void lpc84x_i2c_config_func_##n(const struct device *dev)                           \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), lpc84x_i2c_isr,             \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
                                                                                                   \
	static const struct lpc84x_i2c_config lpc84x_i2c_##n##_config = {                          \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                              \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, name),              \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                       \
		.bitrate = DT_INST_PROP(n, clock_frequency),                                       \
		.irq_config_func = lpc84x_i2c_config_func_##n,                                     \
	};                                                                                         \
                                                                                                   \
	static struct lpc84x_i2c_data lpc84x_i2c_##n##_data;                                       \
                                                                                                   \
	I2C_DEVICE_DT_INST_DEFINE(n, lpc84x_i2c_init, NULL, &lpc84x_i2c_##n##_data,                \
				  &lpc84x_i2c_##n##_config, POST_KERNEL, CONFIG_I2C_INIT_PRIORITY, \
				  &lpc84x_i2c_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LPC84X_I2C_INIT)
