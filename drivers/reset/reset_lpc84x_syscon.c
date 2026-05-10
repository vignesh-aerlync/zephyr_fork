/*
 * Copyright (c) 2026 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_lpc84x_syscon_reset

#include <zephyr/device.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/irq.h>

/* PRESETCTRL register offset calculation */
#define LPC_RESET_OFFSET(id) ((id) >> 16)
#define LPC_RESET_BIT(id)    (BIT((id) & 0xFFFF))
#define LPC84X_SYSCON_PRESETCTRL0_OFFSET  0x88U

struct lpc84x_reset_config {
	uintptr_t base; /* Address of PRESETCTRL0 */
};

static int reset_nxp_lpc84x_status(const struct device *dev, uint32_t id, uint8_t *status)
{
	const struct lpc84x_reset_config *cfg = dev->config;
	uint32_t offset = LPC_RESET_OFFSET(id);
	uintptr_t reg = cfg->base + (offset * sizeof(uint32_t));

	*status = !(sys_read32(reg) & LPC_RESET_BIT(id)) ? 1U : 0U;

	return 0;
}

static int reset_nxp_lpc84x_line_assert(const struct device *dev, uint32_t id)
{
	const struct lpc84x_reset_config *cfg = dev->config;
	uint32_t offset = LPC_RESET_OFFSET(id);
	uintptr_t reg = cfg->base + (offset * sizeof(uint32_t));
	uint32_t val;
	unsigned int key;

	key = irq_lock();
	val = sys_read32(reg);
	sys_write32(val & ~LPC_RESET_BIT(id), reg);
	irq_unlock(key);

	return 0;
}

static int reset_nxp_lpc84x_line_deassert(const struct device *dev, uint32_t id)
{
	const struct lpc84x_reset_config *cfg = dev->config;
	uint32_t offset = LPC_RESET_OFFSET(id);
	uintptr_t reg = cfg->base + (offset * sizeof(uint32_t));
	uint32_t val;
	unsigned int key;

	key = irq_lock();
	val = sys_read32(reg);
	sys_write32(val | LPC_RESET_BIT(id), reg);
	irq_unlock(key);

	return 0;
}

static int reset_nxp_lpc84x_line_toggle(const struct device *dev, uint32_t id)
{
	reset_nxp_lpc84x_line_assert(dev, id);
	reset_nxp_lpc84x_line_deassert(dev, id);

	return 0;
}

static DEVICE_API(reset, reset_nxp_lpc84x_driver_api) = {
	.status        = reset_nxp_lpc84x_status,
	.line_assert   = reset_nxp_lpc84x_line_assert,
	.line_deassert = reset_nxp_lpc84x_line_deassert,
	.line_toggle   = reset_nxp_lpc84x_line_toggle,
};

#define LPC84X_RESET_INIT(n)                                                                       \
	static const struct lpc84x_reset_config lpc84x_reset_cfg_##n = {                           \
		.base = DT_REG_ADDR(DT_INST_PARENT(n)) + LPC84X_SYSCON_PRESETCTRL0_OFFSET,         \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, NULL, NULL, NULL, &lpc84x_reset_cfg_##n, PRE_KERNEL_1,            \
			      CONFIG_RESET_INIT_PRIORITY, &reset_nxp_lpc84x_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LPC84X_RESET_INIT)
