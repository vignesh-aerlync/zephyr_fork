/*
 * Copyright (c) 2026 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_lpc84x_dma

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/clock_control.h>
#include <fsl_dma.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(dma_lpc84x, CONFIG_DMA_LOG_LEVEL);

/* DMA alignment requirement (4-byte aligned) */
#define LPC84X_DMA_ALIGNMENT 4U

/* Maximum number of descriptor chains per channel */
#define LPC84X_MAX_DESCRIPTORS_PER_CHANNEL 8U

struct lpc84x_dma_config {
	DEVICE_MMIO_ROM;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	uint8_t num_channels;
	void (*irq_config_func)(const struct device *dev);
};

struct lpc84x_dma_channel_data {
	/** Descriptor array, 16-byte aligned for hardware access */
	dma_descriptor_t descriptors[LPC84X_MAX_DESCRIPTORS_PER_CHANNEL] __aligned(16);

	dma_handle_t handle;
	dma_callback_t callback;
	void *user_data;

	/** Transfer direction (memory to memory, peripheral to memory, etc.) */
	enum dma_channel_direction dir;

	/** Data width in bytes (1, 2, or 4) */
	uint8_t width;

	/** Source address increment (0 = no increment, 1 = increment) */
	uint8_t src_inc;

	/** Destination address increment (0 = no increment, 1 = increment) */
	uint8_t dst_inc;

	/** Current number of descriptors queued */
	uint8_t num_desc;

	/** Flag indicating whether descriptors are queued (for chaining) */
	bool descriptors_queued;

	/** Channel is actively transferring data */
	bool busy;

	/** Channel transfer is suspended (not yet resumed) */
	bool suspended;

	/** Transfer is in cyclic/continuous mode */
	bool cyclic;
};

struct lpc84x_dma_data {
	DEVICE_MMIO_RAM;
	struct dma_context ctx;
	atomic_val_t channel_flags;
	struct lpc84x_dma_channel_data channels[DMA_CHANNEL_COUNT];
};

#define DEV_BASE(dev) ((DMA_Type *)DEVICE_MMIO_GET(dev))

static int lpc84x_dma_queue_descriptors(struct lpc84x_dma_channel_data *ch,
					struct dma_block_config *block, uint8_t src_inc,
					uint8_t dst_inc, bool intA)
{
	/* Check if we have space in our local descriptor pool */
	if (ch->num_desc >= LPC84X_MAX_DESCRIPTORS_PER_CHANNEL) {
		LOG_ERR("Descriptor pool exhausted for channel");
		return -ENOMEM;
	}

	dma_descriptor_t *curr = &ch->descriptors[ch->num_desc];
	dma_descriptor_t *next = NULL;

	if (block->next_block) {
		/* Link to the next available descriptor in the pool */
		next = &ch->descriptors[ch->num_desc + 1];
	} else if (ch->cyclic) {
		/* For cyclic mode, loop back to the head descriptor in hardware SRAM */
		next = (dma_descriptor_t *)ch->handle.base->SRAMBASE + ch->handle.channel;
	}

	/*
	 * Configure the hardware transfer parameters:
	 * - reload=1 if there is a 'next' block (chaining)
	 * - clrtrig=0 (handled separately in configure)
	 * - intA: Interrupt A configuration
	 */
	uint32_t xfer_cfg = DMA_SetChannelXferConfig(next != NULL, 0, intA, 0, ch->width, src_inc,
						     dst_inc, block->block_size);
	xfer_cfg |= DMA_CHANNEL_XFERCFG_CFGVALID_MASK;

	/* Write the descriptor into local memory */
	DMA_SetupDescriptor(curr, xfer_cfg, (void *)block->source_address,
			    (void *)block->dest_address, next);

	ch->num_desc++;
	return 0;
}

static void lpc84x_dma_callback(dma_handle_t *handle, void *param, bool transferDone,
				uint32_t intmode)
{
	const struct device *dev = param;
	struct lpc84x_dma_data *data = dev->data;
	struct lpc84x_dma_channel_data *ch = &data->channels[handle->channel];
	int status;

	/* For non-cyclic transfers, mark as not busy when done */
	if (!ch->cyclic || !transferDone) {
		ch->busy = false;
	}

	/* Invoke the application callback if registered */
	if (ch->callback) {
		status = transferDone ? DMA_STATUS_COMPLETE : -EIO;
		ch->callback(dev, ch->user_data, handle->channel, status);
	}
}

static int lpc84x_dma_configure(const struct device *dev, uint32_t channel, struct dma_config *cfg)
{
	const struct lpc84x_dma_config *config = dev->config;
	struct lpc84x_dma_data *data = dev->data;
	struct lpc84x_dma_channel_data *ch;
	struct dma_block_config *block;
	uint8_t src_inc, dst_inc;
	uint32_t xfer_cfg;

	if (channel >= config->num_channels || !cfg->head_block) {
		LOG_ERR("Invalid channel %u or missing head_block", channel);
		return -EINVAL;
	}

	/* Reset channel state for fresh configuration */
	ch = &data->channels[channel];
	ch->busy = false;
	ch->suspended = false;
	ch->descriptors_queued = false;
	ch->width = cfg->source_data_size;
	ch->cyclic = cfg->cyclic;
	ch->dir = cfg->channel_direction;

	block = cfg->head_block;

	/* Convert Zephyr address adjustment types to hardware increment values */
	src_inc = (block->source_addr_adj == DMA_ADDR_ADJ_NO_CHANGE) ? 0 : 1;
	dst_inc = (block->dest_addr_adj == DMA_ADDR_ADJ_NO_CHANGE) ? 0 : 1;

	ch->src_inc = src_inc;
	ch->dst_inc = dst_inc;

	/* Create hardware handle for this channel */
	DMA_CreateHandle(&ch->handle, DEV_BASE(dev), channel);

	ch->num_desc = 0;

	/*
	 * Build descriptor chain for all blocks except the first one.
	 * The first block is configured as the primary descriptor.
	 */
	struct dma_block_config *curr_block = block->next_block;

	for (int block_idx = 1; block_idx < cfg->block_count; block_idx++) {
		/*
		 * Enable interrupt A only on the last block or if callbacks are disabled.
		 * This prevents premature interrupts during chained transfers.
		 */
		bool intA = !cfg->complete_callback_en || (block_idx == cfg->block_count - 1);

		if (lpc84x_dma_queue_descriptors(ch, curr_block, src_inc, dst_inc, intA)) {
			LOG_ERR("Failed to queue descriptors for block %d", block_idx);
			return -ENOMEM;
		}
		curr_block = curr_block->next_block;
		ch->descriptors_queued = true;
	}

	/* Configure the primary (head) descriptor that hardware starts with */
	bool head_intA = !cfg->complete_callback_en || (cfg->block_count == 1);
	bool reload = (cfg->block_count > 1 || cfg->cyclic);

	/*
	 * clrtrig=1 prevents "interrupt storms" in cyclic mode by requiring
	 * a new trigger for each transfer cycle
	 */
	bool clrtrig = (cfg->block_count == 1 || cfg->cyclic) ? 1 : 0;

	xfer_cfg = DMA_SetChannelXferConfig(reload, clrtrig, head_intA, 0, ch->width, src_inc,
					    dst_inc, block->block_size);
	xfer_cfg |= DMA_CHANNEL_XFERCFG_CFGVALID_MASK;

	/* Point hardware to the first queued descriptor if we have multiple blocks */
	dma_descriptor_t *next_desc = (cfg->block_count > 1) ? &ch->descriptors[0] : NULL;

	if (cfg->block_count == 1 && cfg->cyclic) {
		/*
		 * In cyclic mode with a single block, the next descriptor points to itself
		 * via the hardware SRAM descriptor table
		 */
		next_desc = (dma_descriptor_t *)DEV_BASE(dev)->SRAMBASE + channel;
	}

	/* Submit the configured transfer parameters to hardware registers */
	DMA_SubmitChannelTransferParameter(&ch->handle, xfer_cfg, (void *)block->source_address,
					   (void *)block->dest_address, next_desc);

	ch->callback = cfg->dma_callback;
	ch->user_data = cfg->user_data;

	/* Enable interrupts and set channel priority */
	DMA_SetCallback(&ch->handle, lpc84x_dma_callback, (void *)dev);
	DMA_SetChannelPriority(DEV_BASE(dev), channel, (dma_priority_t)cfg->channel_priority);
	DMA_EnableChannelInterrupts(DEV_BASE(dev), channel);

	LOG_DBG("Configured channel %u: blocks=%u, cyclic=%d, dir=%u", channel, cfg->block_count,
		cfg->cyclic, cfg->channel_direction);

	return 0;
}

static bool lpc84x_dma_chan_filter(const struct device *dev, int channel_id, void *filter_param)
{
	ARG_UNUSED(filter_param);

	const struct lpc84x_dma_config *config = dev->config;

	return (channel_id < config->num_channels);
}

static int lpc84x_dma_start(const struct device *dev, uint32_t channel)
{
	struct lpc84x_dma_data *data = dev->data;
	struct lpc84x_dma_channel_data *ch = &data->channels[channel];

	ch->busy = true;

	/* Enable the channel and trigger transfer start */
	DMA_EnableChannel(DEV_BASE(dev), channel);
	DMA_StartTransfer(&ch->handle);

	LOG_DBG("Started DMA transfer on channel %u", channel);

	return 0;
}

static int lpc84x_dma_stop(const struct device *dev, uint32_t channel)
{
	struct lpc84x_dma_data *data = dev->data;
	struct lpc84x_dma_channel_data *ch = &data->channels[channel];

	/* Abort active transfers and disable the channel */
	DMA_AbortTransfer(&ch->handle);
	DMA_DisableChannel(DEV_BASE(dev), channel);

	ch->busy = false;
	ch->suspended = false;

	LOG_DBG("Stopped DMA transfer on channel %u", channel);

	return 0;
}

static int lpc84x_dma_reload(const struct device *dev, uint32_t channel, uint32_t src, uint32_t dst,
			     size_t size)
{
	struct lpc84x_dma_data *data = dev->data;
	struct lpc84x_dma_channel_data *ch = &data->channels[channel];
	uint32_t xfer_cfg;

	if (!ch->descriptors_queued) {
		/* Simple reload: no descriptor chaining */
		xfer_cfg = DMA_CHANNEL_XFERCFG_CFGVALID(1) | DMA_CHANNEL_XFERCFG_RELOAD(0) |
			   DMA_CHANNEL_XFERCFG_SWTRIG(0) | DMA_CHANNEL_XFERCFG_CLRTRIG(0) |
			   DMA_CHANNEL_XFERCFG_SETINTA(1) | DMA_CHANNEL_XFERCFG_SETINTB(0) |
			   DMA_CHANNEL_XFERCFG_WIDTH(ch->width) |
			   DMA_CHANNEL_XFERCFG_SRCINC(ch->src_inc) |
			   DMA_CHANNEL_XFERCFG_DSTINC(ch->dst_inc) |
			   DMA_CHANNEL_XFERCFG_XFERCOUNT(size - 1);

		DMA_SubmitChannelTransferParameter(&ch->handle, xfer_cfg, (void *)src, (void *)dst,
						   NULL);
	} else {
		/* Reload with descriptor chaining */
		struct dma_block_config blk = {0};

		blk.source_address = src;
		blk.dest_address = dst;
		blk.block_size = size;
		ch->num_desc = 0;

		lpc84x_dma_queue_descriptors(ch, &blk, ch->src_inc, ch->dst_inc, true);
	}

	LOG_DBG("Reloaded channel %u: src=0x%x, dst=0x%x, size=%zu", channel, src, dst, size);

	return 0;
}

static int lpc84x_dma_suspend(const struct device *dev, uint32_t channel)
{
	struct lpc84x_dma_data *data = dev->data;
	struct lpc84x_dma_channel_data *ch = &data->channels[channel];

	if (ch->suspended || (!ch->busy && !DMA_ChannelIsBusy(DEV_BASE(dev), channel))) {
		LOG_WRN("Cannot suspend channel %u: not busy or already suspended", channel);
		return -EINVAL;
	}

	DMA_DisableChannel(DEV_BASE(dev), channel);
	ch->suspended = true;

	LOG_DBG("Suspended DMA transfer on channel %u", channel);

	return 0;
}

static int lpc84x_dma_resume(const struct device *dev, uint32_t channel)
{
	struct lpc84x_dma_data *data = dev->data;
	struct lpc84x_dma_channel_data *ch = &data->channels[channel];

	if (!ch->suspended) {
		return 0;
	}

	ch->busy = true;
	ch->suspended = false;

	DMA_EnableChannel(DEV_BASE(dev), channel);

	/* For memory-to-memory transfers, trigger via software */
	if (ch->dir == MEMORY_TO_MEMORY) {
		DMA_DoChannelSoftwareTrigger(DEV_BASE(dev), channel);
	}

	LOG_DBG("Resumed DMA transfer on channel %u", channel);

	return 0;
}

static int lpc84x_get_status(const struct device *dev, uint32_t channel, struct dma_status *status)
{
	struct lpc84x_dma_data *data = dev->data;
	struct lpc84x_dma_channel_data *ch = &data->channels[channel];

	status->busy = ch->busy || DMA_ChannelIsBusy(DEV_BASE(dev), channel);
	status->pending_length = DMA_GetRemainingBytes(DEV_BASE(dev), channel);
	status->dir = ch->dir;

	return 0;
}

static int lpc84x_get_attribute(const struct device *dev, uint32_t type, uint32_t *value)
{
	switch (type) {
	case DMA_ATTR_BUFFER_ADDRESS_ALIGNMENT:
	case DMA_ATTR_BUFFER_SIZE_ALIGNMENT:
	case DMA_ATTR_COPY_ALIGNMENT:
		*value = LPC84X_DMA_ALIGNMENT;
		break;
	default:
		LOG_ERR("Unknown DMA attribute type: %u", type);
		return -EINVAL;
	}

	return 0;
}

static void lpc84x_dma_isr(const struct device *dev)
{
	DMA_IRQHandle(DEV_BASE(dev));
}

static int lpc84x_dma_init(const struct device *dev)
{
	const struct lpc84x_dma_config *config = dev->config;
	struct lpc84x_dma_data *data = dev->data;
	int err;

	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR("Clock device not ready");
		return -ENODEV;
	}

	err = clock_control_on(config->clock_dev, config->clock_subsys);
	if (err) {
		LOG_ERR("Failed to enable clock: %d", err);
		return err;
	}

	/* Initialize channel data structures */
	memset(data->channels, 0, sizeof(data->channels));
	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	/* Initialize the DMA controller */
	DMA_Init(DEV_BASE(dev));

	/* Setup DMA context for Zephyr framework */
	data->ctx.magic = DMA_MAGIC;
	data->ctx.dma_channels = config->num_channels;
	data->ctx.atomic = &data->channel_flags;

	/* Configure interrupts */
	config->irq_config_func(dev);

	return 0;
}

static DEVICE_API(dma, lpc84x_dma_api) = {
	.config = lpc84x_dma_configure,
	.start = lpc84x_dma_start,
	.stop = lpc84x_dma_stop,
	.reload = lpc84x_dma_reload,
	.suspend = lpc84x_dma_suspend,
	.resume = lpc84x_dma_resume,
	.get_status = lpc84x_get_status,
	.get_attribute = lpc84x_get_attribute,
	.chan_filter = lpc84x_dma_chan_filter,
};

#define LPC84X_DMA_INIT(inst)                                                                      \
	static void lpc84x_dma_irq_config_##inst(const struct device *dev)                         \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), lpc84x_dma_isr,       \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	}                                                                                          \
                                                                                                   \
	static const struct lpc84x_dma_config lpc84x_dma_config_##inst = {                         \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),                                           \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                             \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(inst, name),           \
		.num_channels = DT_INST_PROP(inst, dma_channels),                                  \
		.irq_config_func = lpc84x_dma_irq_config_##inst,                                   \
	};                                                                                         \
                                                                                                   \
	static struct lpc84x_dma_data lpc84x_dma_data_##inst;                                      \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &lpc84x_dma_init, NULL, &lpc84x_dma_data_##inst,               \
			      &lpc84x_dma_config_##inst, PRE_KERNEL_1, CONFIG_DMA_INIT_PRIORITY,   \
			      &lpc84x_dma_api);

DT_INST_FOREACH_STATUS_OKAY(LPC84X_DMA_INIT)
