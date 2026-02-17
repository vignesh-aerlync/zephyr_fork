/*
 * Copyright (c) 2026, Aerlync Labs
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Clock control driver for LPC845 Breakout Board.
 */

#define DT_DRV_COMPAT nxp_lpc84x

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>
#include <fsl_clock.h>
#include <fsl_power.h>

LOG_MODULE_REGISTER(clock_lpc84x, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

/*
 * The NXP SDK uses a global SYSCON pointer. We ensure the Device Tree
 * address matches what the SDK expects for compatibility with SDK functions.
 */
#if DT_INST_REG_ADDR(0) != SYSCON_BASE
#error "Device Tree SYSCON address does not match HAL SYSCON_BASE"
#endif

struct lpc84x_clock_config {
	SYSCON_Type *base;
	uint8_t ahb_div;
};

static void lpc84x_update_src(volatile uint32_t *uen_reg, uint32_t mask)
{
	*uen_reg &= ~mask;
	*uen_reg |= mask;
	while ((*uen_reg & mask) == 0U) {
		/* Wait for clock source update */
	}
}

static int lpc84x_clock_on(const struct device *dev, clock_control_subsys_t sub)
{
	const struct lpc84x_clock_config *config = dev->config;
	uint32_t gate = (uint32_t)sub;
	uint32_t reg_offset = CLK_GATE_GET_REG(gate);
	uint32_t bit_shift = CLK_GATE_GET_BITS_SHIFT(gate);

	/* SYSAHBCLKCTRL0 is at offset 0x80 within SYSCON */
	volatile uint32_t *reg = (volatile uint32_t *)((uint8_t *)config->base + 0x80 + reg_offset);

	*reg |= (1U << bit_shift);
	return 0;
}

static int lpc84x_clock_off(const struct device *dev, clock_control_subsys_t sub)
{
	const struct lpc84x_clock_config *config = dev->config;
	uint32_t gate = (uint32_t)sub;
	uint32_t reg_offset = CLK_GATE_GET_REG(gate);
	uint32_t bit_shift = CLK_GATE_GET_BITS_SHIFT(gate);

	volatile uint32_t *reg = (volatile uint32_t *)((uint8_t *)config->base + 0x80 + reg_offset);

	*reg &= ~(1U << bit_shift);
	return 0;
}

static int lpc84x_clock_get_rate(const struct device *dev, clock_control_subsys_t sub,
				 uint32_t *rate)
{
	clock_ip_name_t ip_name = (clock_ip_name_t)sub;

	if (ip_name == kCLOCK_Uart0) {
		*rate = CLOCK_GetUart0ClkFreq();
	} else if (ip_name == kCLOCK_Uart1) {
		*rate = CLOCK_GetUart1ClkFreq();
	} else if (ip_name == kCLOCK_Uart2) {
		*rate = CLOCK_GetUart2ClkFreq();
	} else if (ip_name == kCLOCK_Uart3) {
		*rate = CLOCK_GetUart3ClkFreq();
	} else if (ip_name == kCLOCK_Uart4) {
		*rate = CLOCK_GetUart4ClkFreq();
	} else {
		*rate = CLOCK_GetFreq((clock_name_t)sub);
	}
	return 0;
}

static const struct clock_control_driver_api lpc84x_clock_api = {
	.on = lpc84x_clock_on,
	.off = lpc84x_clock_off,
	.get_rate = lpc84x_clock_get_rate,
};

static int lpc84x_clock_init(const struct device *dev)
{
	const struct lpc84x_clock_config *config = dev->config;
	SYSCON_Type *syscon = config->base;

#if DT_INST_PROP(0, enable_fro)
	clock_fro_osc_freq_t fro_freq_enum;
	uint32_t fro_idx = DT_INST_ENUM_IDX(0, fro_freq);

	if (fro_idx == 0) {
		fro_freq_enum = kCLOCK_FroOscOut18M;
	} else if (fro_idx == 1) {
		fro_freq_enum = kCLOCK_FroOscOut24M;
	} else {
		fro_freq_enum = kCLOCK_FroOscOut30M;
	}

	/* Power up FRO and FRO output */
	syscon->PDRUNCFG &= ~SYSCON_PDRUNCFG_FRO_PD_MASK;
	syscon->PDRUNCFG &= ~SYSCON_PDRUNCFG_FROOUT_PD_MASK;

	/* Setting FRO frequency via ROM API */
	CLOCK_SetFroOscFreq(fro_freq_enum);

	if (DT_INST_PROP(0, fro_low_power_boot)) {
		syscon->FROOSCCTRL &= ~SYSCON_FROOSCCTRL_FRO_DIRECT_MASK;
		LOG_DBG("FRO: LP Boot enabled");
	} else {
		syscon->FROOSCCTRL |= SYSCON_FROOSCCTRL_FRO_DIRECT_MASK;
		LOG_DBG("FRO: Direct enabled");
	}
	lpc84x_update_src(&syscon->FRODIRECTCLKUEN, SYSCON_FRODIRECTCLKUEN_ENA_MASK);
#endif

#if DT_INST_PROP(0, enable_sysosc)
	if (DT_INST_NODE_HAS_PROP(DT_DRV_INST(0), sysosc_freq)) {
		CLOCK_InitSysOsc(DT_INST_PROP(0, sysosc_freq));
		LOG_DBG("SYSOSC: %u Hz enabled", DT_INST_PROP(0, sysosc_freq));
	} else {
		LOG_ERR("sysosc-freq required for enable-sysosc");
		return -EINVAL;
	}
#endif

#if DT_INST_NODE_HAS_PROP(DT_DRV_INST(0), extclk_src)
	uint32_t ext_idx = DT_INST_ENUM_IDX(0, extclk_src);

	if (ext_idx == 0) { /* sysosc */
		syscon->EXTCLKSEL &= ~SYSCON_EXTCLKSEL_SEL_MASK;
	} else { /* clkin */
		if (DT_INST_NODE_HAS_PROP(DT_DRV_INST(0), extclk_freq)) {
			CLOCK_InitExtClkin(DT_INST_PROP(0, extclk_freq));
			syscon->EXTCLKSEL |= SYSCON_EXTCLKSEL_SEL_MASK;
			LOG_DBG("CLKIN: %u Hz enabled", DT_INST_PROP(0, extclk_freq));
		} else {
			LOG_ERR("extclk-freq required for extclk-src clkin");
			return -EINVAL;
		}
	}
#endif

#if DT_INST_NODE_HAS_PROP(DT_DRV_INST(0), mainclk_src)
	uint32_t main_idx = DT_INST_ENUM_IDX(0, mainclk_src);
	uint32_t main_src_packed;

	switch (main_idx) {
	case 0:
		main_src_packed = kCLOCK_MainClkSrcFro;
		break;
	case 1:
		main_src_packed = kCLOCK_MainClkSrcExtClk;
		break;
	case 2:
		main_src_packed = kCLOCK_MainClkSrcWdtOsc;
		LOG_WRN("Using WDTOSC as main clock (ensure initialization)");
		break;
	case 3:
		main_src_packed = kCLOCK_MainClkSrcFroDiv;
		break;
	case 4:
		main_src_packed = kCLOCK_MainClkSrcSysPll;
		break;
	default:
		return -EINVAL;
	}

	/* Implement CLOCK_SetMainClkSrc logic */
	uint32_t mainMux = CLK_MAIN_CLK_MUX_GET_MUX(main_src_packed);
	uint32_t mainPreMux = CLK_MAIN_CLK_MUX_GET_PRE_MUX(main_src_packed);

	if (((syscon->MAINCLKSEL & SYSCON_MAINCLKSEL_SEL_MASK) != mainPreMux) && (mainMux == 0U)) {
		syscon->MAINCLKSEL = (syscon->MAINCLKSEL & (~SYSCON_MAINCLKSEL_SEL_MASK)) |
				     SYSCON_MAINCLKSEL_SEL(mainPreMux);
		lpc84x_update_src(&syscon->MAINCLKUEN, SYSCON_MAINCLKUEN_ENA_MASK);
	}

	if ((syscon->MAINCLKPLLSEL & SYSCON_MAINCLKPLLSEL_SEL_MASK) != mainMux) {
		syscon->MAINCLKPLLSEL = (syscon->MAINCLKPLLSEL & (~SYSCON_MAINCLKPLLSEL_SEL_MASK)) |
					SYSCON_MAINCLKPLLSEL_SEL(mainMux);
		lpc84x_update_src(&syscon->MAINCLKPLLUEN, SYSCON_MAINCLKPLLUEN_ENA_MASK);
	}
	LOG_DBG("Main clock source set to index %d", main_idx);
#endif

	/* Set AHB clock divider */
	if (config->ahb_div == 0) {
		LOG_ERR("AHB divider cannot be 0");
		return -EINVAL;
	}
	syscon->SYSAHBCLKDIV = (syscon->SYSAHBCLKDIV & (~SYSCON_SYSAHBCLKDIV_DIV_MASK)) |
			       SYSCON_SYSAHBCLKDIV_DIV(config->ahb_div);

	/* Set Flash wait states for the frequency */
	uint32_t main_freq = CLOCK_GetMainClkFreq();

	CLOCK_SetFLASHAccessCyclesForFreq(main_freq);

	SystemCoreClockUpdate();

	LOG_INF("LPC84x Clock Controller initialized. Main freq: %u Hz", main_freq);

	return 0;
}

#define LPC84X_CLOCK_INIT(inst)                                                                    \
	static const struct lpc84x_clock_config cfg_##inst = {                                     \
		.base = (SYSCON_Type *)DT_INST_REG_ADDR(inst),                                     \
		.ahb_div = DT_INST_PROP(inst, ahb_clk_divider),                                    \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, lpc84x_clock_init, NULL, NULL, &cfg_##inst, PRE_KERNEL_1,      \
			      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &lpc84x_clock_api);

DT_INST_FOREACH_STATUS_OKAY(LPC84X_CLOCK_INIT)
