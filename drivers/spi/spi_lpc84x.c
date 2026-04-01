/*
 * Copyright (c) 2026 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_lpc84x_spi

#include <zephyr/logging/log.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <fsl_spi.h>

LOG_MODULE_REGISTER(spi_lpc84x, CONFIG_SPI_LOG_LEVEL);

#include "spi_context.h"

#define LPC84X_DUMMY_BUFFER_SIZE 256

struct spi_lpc84x_config {
	DEVICE_MMIO_ROM;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	const struct pinctrl_dev_config *pincfg;
	/* Pointer to the instance-specific IRQ connect function */
	void (*irq_config_func)(const struct device *dev);
};

struct spi_lpc84x_data {
	DEVICE_MMIO_RAM;
	struct spi_context ctx;
	bool initialized;
	const struct spi_config *last_cfg;
#ifdef CONFIG_SPI_LPC84X_INTERRUPT
	spi_master_handle_t fsl_handle;    /* FSL transactional handle            */
	struct k_sem transfer_sem;         /* signals ISR → transceive completion */
	volatile status_t transfer_status; /* HAL callback result                 */
#endif
};

#ifdef CONFIG_SPI_LPC84X_INTERRUPT
static void spi_lpc84x_transfer_callback(SPI_Type *, spi_master_handle_t *, status_t, void *);
#endif

static int spi_lpc84x_configure(const struct device *dev, const struct spi_config *spi_cfg)
{
	const struct spi_lpc84x_config *config = dev->config;
	struct spi_lpc84x_data *data = dev->data;
	spi_master_config_t master_config;
	uint32_t clock_freq;
	int ret;
	SPI_Type *base = (SPI_Type *)DEVICE_MMIO_GET(dev);

	/*
	 * Skip reconfiguration if the peripheral is already initialized with
	 * the same frequency and operation flags. Update only the context
	 * config pointer so the SPI subsystem sees the new reference.
	 */
	if (data->initialized && data->last_cfg != NULL &&
	    data->last_cfg->frequency == spi_cfg->frequency &&
	    data->last_cfg->operation == spi_cfg->operation) {
		data->ctx.config = spi_cfg;
		return 0;
	}

	if (spi_cfg->operation & SPI_HALF_DUPLEX) {
		LOG_ERR("Half-duplex not supported");
		return -ENOTSUP;
	}

	if (spi_cfg->operation & SPI_OP_MODE_SLAVE) {
		LOG_ERR("Slave mode not supported");
		return -ENOTSUP;
	}

	if (SPI_WORD_SIZE_GET(spi_cfg->operation) != 8) {
		LOG_ERR("Word sizes other than 8 bits are not supported");
		return -ENOTSUP;
	}

	SPI_MasterGetDefaultConfig(&master_config);

	if (spi_cfg->operation & SPI_MODE_CPOL) {
		master_config.clockPolarity = kSPI_ClockPolarityActiveLow;
	} else {
		master_config.clockPolarity = kSPI_ClockPolarityActiveHigh;
	}

	if (spi_cfg->operation & SPI_MODE_CPHA) {
		master_config.clockPhase = kSPI_ClockPhaseSecondEdge;
	} else {
		master_config.clockPhase = kSPI_ClockPhaseFirstEdge;
	}

	master_config.baudRate_Bps = spi_cfg->frequency;

	if (clock_control_get_rate(config->clock_dev, config->clock_subsys, &clock_freq)) {
		return -EINVAL;
	}

	if (!data->initialized) {
		SPI_MasterInit(base, &master_config, clock_freq);
		data->initialized = true;

	} else {
		ret = SPI_MasterSetBaudRate(base, master_config.baudRate_Bps, clock_freq);
		if (ret != kStatus_Success) {
			LOG_DBG("Master baudrate set failed\n");
			return -EINVAL;
		}
		uint32_t cfg = base->CFG;

		cfg &= ~(SPI_CFG_CPOL_MASK | SPI_CFG_CPHA_MASK);
		if (spi_cfg->operation & SPI_MODE_CPOL) {
			cfg |= SPI_CFG_CPOL_MASK;
		}
		if (spi_cfg->operation & SPI_MODE_CPHA) {
			cfg |= SPI_CFG_CPHA_MASK;
		}
		base->CFG = cfg;
	}

	data->ctx.config = spi_cfg;
	data->last_cfg = spi_cfg;

	return 0;
}

#ifdef CONFIG_SPI_LPC84X_INTERRUPT

/*
 * FSL transactional callback – called from SPI_MasterTransferHandleIRQ()
 * inside the Zephyr ISR dispatcher (spi_lpc84x_irq_handler below).
 *
 * NOTE: The FSL HAL passes the handle as the second parameter.  We only need
 * the `status` to signal completion; the device pointer is recovered via
 * userData.
 */
static void spi_lpc84x_transfer_callback(SPI_Type *base, spi_master_handle_t *handle,
					 status_t status, void *userData)
{
	ARG_UNUSED(base);
	ARG_UNUSED(handle);
	const struct device *dev = (const struct device *)userData;
	struct spi_lpc84x_data *data = dev->data;

	data->transfer_status = status;
	k_sem_give(&data->transfer_sem);
}

/*
 * Zephyr ISR – connected to the hardware IRQ line.
 * Delegates to the FSL IRQ handler which drives the state machine and fires
 * spi_lpc84x_transfer_callback() when a transfer finishes.
 */
static void spi_lpc84x_irq_handler(const struct device *dev)
{
	struct spi_lpc84x_data *data = dev->data;
	SPI_Type *base = (SPI_Type *)DEVICE_MMIO_GET(dev);

	SPI_MasterTransferHandleIRQ(base, &data->fsl_handle);
}

static int spi_lpc84x_transceive_irq(const struct device *dev, const struct spi_config *spi_cfg,
				     const struct spi_buf_set *tx_bufs,
				     const struct spi_buf_set *rx_bufs)
{
	struct spi_lpc84x_data *data = dev->data;
	int ret = 0;
	status_t status;
	SPI_Type *base = (SPI_Type *)DEVICE_MMIO_GET(dev);

	spi_context_lock(&data->ctx, false, NULL, NULL, spi_cfg);

	ret = spi_lpc84x_configure(dev, spi_cfg);
	if (ret != 0) {
		goto out;
	}

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);
	spi_context_cs_control(&data->ctx, true);

	while (spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx)) {
		spi_transfer_t xfer = {0};
		size_t tx_len = spi_context_tx_on(&data->ctx) ? data->ctx.tx_len : 0;
		size_t rx_len = spi_context_rx_on(&data->ctx) ? data->ctx.rx_len : 0;
		size_t chunk;

		if (tx_len > 0 && rx_len > 0) {
			chunk = MIN(tx_len, rx_len);
		} else if (tx_len > 0) {
			chunk = tx_len;
		} else {
			chunk = rx_len;
		}

		static const uint8_t dummy_tx[LPC84X_DUMMY_BUFFER_SIZE] = {0};

		/* * If we need to send data but the Zephyr tx_buf is NULL, force the HAL
		 * to read from our dummy buffer so txRemainingBytes doesn't become 0.
		 */
		xfer.txData = (tx_len > 0 && data->ctx.tx_buf != NULL)
				      ? (const uint8_t *)data->ctx.tx_buf
				      : dummy_tx;

		/* * If rx_buf is NULL, we can safely pass NULL to the HAL.
		 * The HAL will optimize the transfer by setting the RXIGNORE flag.
		 */
		xfer.rxData = (rx_len > 0 && data->ctx.rx_buf != NULL) ? (uint8_t *)data->ctx.rx_buf
								       : NULL;

		/* Prevent out-of-bounds memory reads during large dummy transfers */
		if (xfer.txData == dummy_tx && chunk > sizeof(dummy_tx)) {
			chunk = sizeof(dummy_tx);
		}

		xfer.dataSize = chunk;

		bool last_chunk = ((tx_len == 0) || (chunk == tx_len)) &&
				  ((rx_len == 0) || (chunk == rx_len));
		xfer.configFlags = last_chunk ? kSPI_EndOfTransfer : 0U;

		if (xfer.rxData != NULL) {
			base->TXCTL &= ~SPI_TXCTL_RXIGNORE_MASK;
		}
		k_sem_reset(&data->transfer_sem);

		status = SPI_MasterTransferNonBlocking(base, &data->fsl_handle, &xfer);
		if (status != kStatus_Success) {
			LOG_ERR("SPI_MasterTransferNonBlocking failed: %d", (int)status);
			ret = -EIO;
			break;
		}

		/*
		 * Block until the segment completes. Timeout is generous:
		 * at 500 kHz, 256 bytes takes ~4 ms; 500 ms covers any
		 * realistic segment size and frequency combination.
		 */
		if (k_sem_take(&data->transfer_sem, K_MSEC(500)) != 0) {
			LOG_ERR("SPI interrupt transfer timed out (%zu bytes)", chunk);
			SPI_MasterTransferAbort(base, &data->fsl_handle);
			ret = -ETIMEDOUT;
			break;
		}

		if (data->transfer_status == kStatus_SPI_Error ||
		    data->transfer_status == kStatus_SPI_Timeout) {
			LOG_ERR("SPI transfer error: %d", (int)data->transfer_status);
			ret = -EIO;
			break;
		}

		spi_context_update_tx(&data->ctx, 1, tx_len ? chunk : 0);
		spi_context_update_rx(&data->ctx, 1, rx_len ? chunk : 0);
	}

	spi_context_cs_control(&data->ctx, false);

out:
	spi_context_release(&data->ctx, ret);
	return ret;
}
#else

static int spi_lpc84x_transceive_poll(const struct device *dev, const struct spi_config *spi_cfg,
				      const struct spi_buf_set *tx_bufs,
				      const struct spi_buf_set *rx_bufs)
{
	struct spi_lpc84x_data *data = dev->data;
	int ret;
	SPI_Type *base = (SPI_Type *)DEVICE_MMIO_GET(dev);

	spi_context_lock(&data->ctx, false, NULL, NULL, spi_cfg);

	ret = spi_lpc84x_configure(dev, spi_cfg);
	if (ret != 0) {
		goto out;
	}

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	spi_context_cs_control(&data->ctx, true);

	while (spi_context_tx_buf_on(&data->ctx) || spi_context_rx_buf_on(&data->ctx)) {
		spi_transfer_t transfer = {0};
		uint8_t tx_data = 0;
		uint8_t rx_data = 0;

		if (spi_context_tx_buf_on(&data->ctx)) {
			tx_data = *data->ctx.tx_buf;
		}

		transfer.txData = &tx_data;
		transfer.rxData = &rx_data;
		transfer.dataSize = 1;
		transfer.configFlags = kSPI_EndOfTransfer;

		if (SPI_MasterTransferBlocking(base, &transfer) != kStatus_Success) {
			ret = -EIO;
			break;
		}

		LOG_DBG("irq: tx=0x%02x rx=0x%02x", tx_data, rx_data);

		if (spi_context_rx_buf_on(&data->ctx)) {
			*data->ctx.rx_buf = rx_data;
		}

		spi_context_update_tx(&data->ctx, 1, 1);
		spi_context_update_rx(&data->ctx, 1, 1);
	}

	spi_context_cs_control(&data->ctx, false);

out:
	spi_context_release(&data->ctx, ret);

	return ret;
}
#endif

static int spi_lpc84x_transceive_sync(const struct device *dev, const struct spi_config *config,
				      const struct spi_buf_set *tx_bufs,
				      const struct spi_buf_set *rx_bufs)
{
#ifdef CONFIG_SPI_LPC84X_INTERRUPT
	return spi_lpc84x_transceive_irq(dev, config, tx_bufs, rx_bufs);
#else
	return spi_lpc84x_transceive_poll(dev, config, tx_bufs, rx_bufs);
#endif
}

#ifdef CONFIG_SPI_ASYNC
static int spi_lpc84x_transceive_async(const struct device *dev, const struct spi_config *spi_cfg,
				       const struct spi_buf_set *tx_bufs,
				       const struct spi_buf_set *rx_bufs, spi_callback_t cb,
				       void *userdata)
{
	return -ENOTSUP;
}
#endif

static int spi_lpc84x_release(const struct device *dev, const struct spi_config *spi_cfg)
{
	struct spi_lpc84x_data *data = dev->data;

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static int spi_lpc84x_init(const struct device *dev)
{
	const struct spi_lpc84x_config *config = dev->config;
	struct spi_lpc84x_data *data = dev->data;
	int ret;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR("clock device is not ready");
		return -ENODEV;
	}

	ret = clock_control_on(config->clock_dev, config->clock_subsys);
	if (ret != 0) {
		return ret;
	}

	ret = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("Failed to apply pinctrl state");
		return ret;
	}

	LOG_DBG("SWM SPI0_SCK assigned to: %d", (int)((SWM0->PINASSIGN.PINASSIGN3 >> 24) & 0xFF));

	ret = spi_context_cs_configure_all(&data->ctx);
	if (ret != 0) {
		return ret;
	}

#ifdef CONFIG_SPI_LPC84X_INTERRUPT
	SPI_Type *base = (SPI_Type *)DEVICE_MMIO_GET(dev);
	/* Initialise the completion semaphore (starts at 0, max 1) */
	k_sem_init(&data->transfer_sem, 0, 1);
	/*
	 * Create the FSL transactional handle once at init time.
	 * This only sets up the software handle struct and registers
	 * the callback
	 */
	SPI_MasterTransferCreateHandle(base, &data->fsl_handle, spi_lpc84x_transfer_callback,
				       (void *)dev);

	/*
	 * Wire up the hardware IRQ.  config->irq_config_func is generated by
	 * the SPI_LPC84X_IRQ_CONNECT() macro for each DT instance.
	 */
	config->irq_config_func(dev);

	LOG_INF("spi_lpc84x: interrupt mode enabled");
#else
	LOG_INF("spi_lpc84x: polling mode enabled");
#endif

	spi_context_unlock_unconditionally(&data->ctx);
	LOG_INF("spi initialized");

	return 0;
}

static DEVICE_API(spi, spi_lpc84x_driver_api) = {
	.transceive = spi_lpc84x_transceive_sync,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_lpc84x_transceive_async,
#endif
	.release = spi_lpc84x_release,
};

/*
 * Interrupt-mode variant:
 *   Generates an irq_config_func_<n> that connects the Zephyr ISR wrapper
 *   to the hardware IRQ number/priority declared in the DT node, and adds
 *   irq_config_func to the config struct.
 */

#define SPI_LPC84X_INIT(n)                                                                         \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	COND_CODE_1(CONFIG_SPI_LPC84X_INTERRUPT, (						   \
	static void irq_config_func_##n(const struct device *dev)				   \
				{                                                                  \
				IRQ_CONNECT(DT_INST_IRQN(n),					   \
					     DT_INST_IRQ(n, priority),				   \
					     spi_lpc84x_irq_handler,				   \
					     DEVICE_DT_INST_GET(n), 0);				   \
				 irq_enable(DT_INST_IRQN(n));					   \
				 }								   \
			), ())                                       \
                                                                                                   \
	static const struct spi_lpc84x_config spi_lpc84x_config_##n = {                            \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                              \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, name),              \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                       \
		.irq_config_func =                                                                 \
			COND_CODE_1(CONFIG_SPI_LPC84X_INTERRUPT,			   \
				(irq_config_func_##n),						   \
				(NULL)),     \
                                                                                                   \
	};                                                                                         \
                                                                                                   \
	static struct spi_lpc84x_data spi_lpc84x_data_##n = {                                      \
		SPI_CONTEXT_INIT_LOCK(spi_lpc84x_data_##n, ctx),                                   \
		SPI_CONTEXT_INIT_SYNC(spi_lpc84x_data_##n, ctx),                                   \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)};                             \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, spi_lpc84x_init, NULL, &spi_lpc84x_data_##n,                      \
			      &spi_lpc84x_config_##n, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,       \
			      &spi_lpc84x_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_LPC84X_INIT)
