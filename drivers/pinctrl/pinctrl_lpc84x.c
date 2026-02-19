/*
 * Copyright (c) 2026 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Pin Control driver for LPC84x
 * using IOCON and SWM.
 */

#include <zephyr/drivers/pinctrl.h>
#include <fsl_swm.h>
#include <fsl_iocon.h>
#include <fsl_clock.h>

/*
 * The IOCON definition in the SDK for this device is 1-dimensional (linear array),
 * and the hardware register layout is NOT linear by Port/Pin (e.g., P0.0 is at Index 17).
 * We define this macro to force fsl_iocon.h to use the 1D (Index-based) function version,
 * which matches our hardware structure and mapping.
 */
#define FSL_FEATURE_IOCON_ONE_DIMENSION 1

/* Switch Matrix (SWM) Register Base */
#define PINCTRL_SWM_BASE ((SWM_Type *)DT_REG_ADDR(DT_NODELABEL(swm)))

/* I/O Configuration (IOCON) Register Base */
#define PINCTRL_IOCON_BASE ((IOCON_Type *)DT_REG_ADDR(DT_NODELABEL(iocon)))

/* Bit limit for PINENABLE0 register */
#define SWM_FIXED_PINENABLE0_LIMIT 32

/* Bit flag used in SDK's to denote PINENABLE1 register */
#define SWM_FIXED_REG_SEL_BIT 31

/* IOCON Configuration bitmask */
#define IOCON_CFG_BIAS_MASK 0x03U
#define IOCON_CFG_HYS_BIT   BIT(2)
#define IOCON_CFG_INV_BIT   BIT(3)
#define IOCON_CFG_OD_BIT    BIT(4)

/**
 * @brief Apply Switch Matrix (SWM) configuration for a pin.
 *
 * LPC84x uses SWM to route functions to pins. There are two types of functions:
 * 1. Movable: Can be assigned to almost any PIO pin using PINASSIGN registers.
 * 2. Fixed-pin: Tied to specific physical pins (e.g., Oscillator, ADC, SWD)
 *    and enabled/disabled via PINENABLE registers.
 *
 * The check for fixed-pin is necessary because the hardware uses completely
 * different register sets for movable vs. fixed functions.
 *
 * @param pin Pin configuration descriptor.
 */
static void lpc_pinctrl_apply_swm(pinctrl_soc_pin_t pin)
{
	uint8_t swm_pin = Z_LPC84X_GET_SWM_PIN(pin);
	uint8_t func_id = Z_LPC84X_GET_SWM_FUNC(pin);

	if (Z_LPC84X_IS_FIXED(pin)) {
		uint32_t mask;
		/*
		 * Fixed-function mapping.
		 * func_id represents the bit position in PINENABLE0 or PINENABLE1.
		 * Pins in PINENABLE0 are 0-31, PINENABLE1 are 32+.
		 * SDK expects BIT 31 to be set for PINENABLE1.
		 */
		if (func_id < SWM_FIXED_PINENABLE0_LIMIT) {
			mask = BIT(func_id);
		} else {
			mask = BIT(func_id - SWM_FIXED_PINENABLE0_LIMIT) |
				BIT(SWM_FIXED_REG_SEL_BIT);
		}
		SWM_SetFixedPinSelect(PINCTRL_SWM_BASE, (swm_select_fixed_pin_t)mask, 1);
	} else {
		/* Movable function mapping */
		SWM_SetMovablePinSelect(PINCTRL_SWM_BASE, (swm_select_movable_t)func_id,
					(swm_port_pin_type_t)swm_pin);
	}
}

/**
 * @brief Apply I/O configuration (IOCON) for a pin.
 *
 * This configures electrical characteristics like pull-up/down, hysteresis,
 * inversion, and open-drain mode.
 *
 * @param pin Pin configuration descriptor.
 */
static void lpc_pinctrl_apply_iocon(pinctrl_soc_pin_t pin)
{
	uint8_t cfg = Z_LPC84X_GET_IOCON_CFG(pin);
	uint8_t iocon_idx = Z_LPC84X_GET_IOCON_INDEX(pin);
	uint32_t iocon_val = 0;

	/* Bias (Pull-up/Pull-down/None) */
	switch (cfg & IOCON_CFG_BIAS_MASK) {
	case 1:
		iocon_val |= IOCON_MODE_PULLDOWN;
		break;
	case 2:
		iocon_val |= IOCON_MODE_PULLUP;
		break;
	default:
		iocon_val |= IOCON_MODE_INACT;
		break;
	}

	/* Hysteresis */
	if (cfg & IOCON_CFG_HYS_BIT) {
		iocon_val |= IOCON_PIO_HYS(1);
	}

	/* Input Inversion */
	if (cfg & IOCON_CFG_INV_BIT) {
		iocon_val |= IOCON_INV_EN;
	}

	/* Open-Drain Mode */
	if (cfg & IOCON_CFG_OD_BIT) {
		iocon_val |= IOCON_OPENDRAIN_EN;
	}

	/* Set the IOCON register using SDK */
	IOCON_PinMuxSet(PINCTRL_IOCON_BASE, iocon_idx, iocon_val);
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	/* We use the global SWM and IOCON register bases */
	ARG_UNUSED(reg);

	for (uint8_t i = 0; i < pin_cnt; i++) {
		lpc_pinctrl_apply_swm(pins[i]);
		lpc_pinctrl_apply_iocon(pins[i]);
	}

	return 0;
}

static int pinctrl_lpc84x_init(void)
{
	CLOCK_EnableClock(kCLOCK_Swm);
	CLOCK_EnableClock(kCLOCK_Iocon);

	return 0;
}

SYS_INIT(pinctrl_lpc84x_init, PRE_KERNEL_1, CONFIG_PINCTRL_LPC84X_INIT_PRIORITY);
