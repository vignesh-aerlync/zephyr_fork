/* Copyright (c) 2026 Aerlync Labs Inc.
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

/* Supported I2C speed modes (in Hz) */
#define I2C_STANDARD_BITRATE  (100000)
#define I2C_FAST_BITRATE      (400000)
#define I2C_FAST_PLUS_BITRATE (1000000)

struct lpc84x_i2c_config {
	DEVICE_MMIO_ROM;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	const struct pinctrl_dev_config *pincfg;
	void (*irq_config_func)(const struct device *dev);
};

struct lpc84x_i2c_data {
	DEVICE_MMIO_RAM;
	struct k_sem lock;
	struct k_sem transfer_sem;
	volatile status_t callback_status;
	i2c_master_transfer_t transfer;
	i2c_master_handle_t handle;
};
/* Defined macro to get the Register Base Address */
#define DEV_BASE(dev) ((I2C_Type *)DEVICE_MMIO_GET(dev))

static int lpc84x_i2c_set_bitrate(const struct device *dev, uint32_t speed)
{
	const struct lpc84x_i2c_config *config = dev->config;
	uint32_t clock_freq;
	int err;

	err = clock_control_get_rate(config->clock_dev, config->clock_subsys, &clock_freq);
	if (err) {
		LOG_ERR("failed to get clock rate (err %d)", err);
		return err;
	}

	I2C_MasterSetBaudRate(DEV_BASE(dev), speed, clock_freq);

	return 0;
}

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
		LOG_ERR("Unsupported speed");
		return -ENOTSUP;
	default:
		return -EINVAL;
	}

	k_sem_take(&data->lock, K_FOREVER);

	ret = lpc84x_i2c_set_bitrate(dev, baud_rate);

	k_sem_give(&data->lock);

	return ret;
}

static int lpc84x_i2c_msg_validate(struct i2c_msg *msgs, uint8_t num_msgs)
{
	for (uint8_t i = 0; i < num_msgs; i++) {
		if ((I2C_MSG_ADDR_10_BITS & msgs[i].flags) ||
		    (msgs[i].len > 0 && msgs[i].buf == NULL)) {
			return -EINVAL;
		}
	}

	return 0;
}

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

static void lpc84x_i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status,
				       void *userData)
{
	const struct device *dev = (const struct device *)userData;
	struct lpc84x_i2c_data *data = dev->data;

	data->callback_status = status;
	k_sem_give(&data->transfer_sem);
}

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

		/* Prevent the controller from sending a start condition between
		 * messages, except if explicitly requested.
		 */
		if (i != 0 && !(msgs[i].flags & I2C_MSG_RESTART)) {
			data->transfer.flags |= kI2C_TransferNoStartFlag;
		}

		status = I2C_MasterTransferNonBlocking(DEV_BASE(dev), &data->handle,
						       &data->transfer);

		if (status != kStatus_Success) {
			I2C_MasterTransferAbort(DEV_BASE(dev), &data->handle);
			ret = -EIO;
			break;
		}

		/* Wait for transfer to complete properly */
		k_sem_take(&data->transfer_sem, K_FOREVER);

		if (data->callback_status != kStatus_Success) {
			I2C_MasterTransferAbort(DEV_BASE(dev), &data->handle);
			ret = -EIO;
			break;
		}
	}

	k_sem_give(&data->lock);

	return ret;
}

static void lpc84x_i2c_isr(const struct device *dev)
{
	struct lpc84x_i2c_data *data = dev->data;

	I2C_MasterTransferHandleIRQ(DEV_BASE(dev), &data->handle);
}

static int lpc84x_i2c_init(const struct device *dev)
{
	const struct lpc84x_i2c_config *config = dev->config;
	struct lpc84x_i2c_data *data = dev->data;
	i2c_master_config_t master_config;
	uint32_t clock_freq = 0;
	int err;

	if (!device_is_ready(config->clock_dev)) {
		return -ENODEV;
	}

	err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (err) {
		return err;
	}

	err = clock_control_on(config->clock_dev, config->clock_subsys);
	if (err) {
		LOG_ERR("failed to enable clock (err %d)", err);
		return err;
	}

	err = clock_control_get_rate(config->clock_dev, config->clock_subsys, &clock_freq);
	if (err) {
		return err;
	}

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	k_sem_init(&data->lock, 1, 1);
	k_sem_init(&data->transfer_sem, 0, 1);

	I2C_MasterGetDefaultConfig(&master_config);
	I2C_MasterEnable(DEV_BASE(dev), master_config.enableMaster);
	I2C_MasterSetBaudRate(DEV_BASE(dev), master_config.baudRate_Bps, clock_freq);

	I2C_MasterTransferCreateHandle(DEV_BASE(dev), &data->handle, lpc84x_i2c_master_callback,
				       (void *)dev);

	config->irq_config_func(dev);

	return 0;
}

static DEVICE_API(i2c, lpc84x_i2c_driver_api) = {
	.configure = lpc84x_i2c_configure,
	.transfer = lpc84x_i2c_transfer,
};

#define LPC84X_I2C_INIT(n)                                                                         \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	static void lpc84x_i2c_config_func_##n(const struct device *dev)                           \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), lpc84x_i2c_isr,             \
			    DEVICE_DT_INST_GET(n), 0);                                             \
                                                                                                   \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
                                                                                                   \
	static const struct lpc84x_i2c_config lpc84x_i2c_##n##_config = {                          \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                              \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, name),              \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                       \
		.irq_config_func = lpc84x_i2c_config_func_##n,                                     \
	};                                                                                         \
                                                                                                   \
	static struct lpc84x_i2c_data lpc84x_i2c_##n##_data;                                       \
                                                                                                   \
	I2C_DEVICE_DT_INST_DEFINE(n, lpc84x_i2c_init, NULL, &lpc84x_i2c_##n##_data,                \
				  &lpc84x_i2c_##n##_config, POST_KERNEL, CONFIG_I2C_INIT_PRIORITY, \
				  &lpc84x_i2c_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LPC84X_I2C_INIT)
