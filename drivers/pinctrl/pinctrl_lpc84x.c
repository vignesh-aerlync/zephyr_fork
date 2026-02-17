/*
 * Copyright (c) 2026 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 *@brief Pin Control driver for LPC84x
 * using IOCON and SWM.
 */

#include <zephyr/drivers/pinctrl.h>
#include <fsl_swm.h>
/*
 * The IOCON definition in the SDK for this device is 1-dimensional (linear array),
 * and the hardware register layout is NOT linear by Port/Pin (e.g., P0.0 is at Index 17).
 * We define this macro to force fsl_iocon.h to use the 1D (Index-based) function version,
 * which matches our hardware structure and mapping.
 */
#define FSL_FEATURE_IOCON_ONE_DIMENSION 1
#include <fsl_iocon.h>
#include <fsl_clock.h>

/* Switch Matrix (SWM) Register Base */
#define PINCTRL_SWM_BASE   ((SWM_Type *)DT_REG_ADDR(DT_NODELABEL(swm)))
/* I/O Configuration (IOCON) Register Base */
#define PINCTRL_IOCON_BASE ((IOCON_Type *)DT_REG_ADDR(DT_NODELABEL(iocon)))

/* Bit limit for PINENABLE0 register */
#define SWM_FIXED_PINENABLE0_LIMIT 32

/* Bit flag used in SDK's swm_select_fixed_pin_t to denote PINENABLE1 register */
#define SWM_FIXED_PINENABLE1_FLAG BIT(31)

/* IOCON Configuration bitmask extractors (matching pinctrl_soc.h) */
#define IOCON_CFG_BIAS_MASK 0x03U
#define IOCON_CFG_HYS_BIT   BIT(2)
#define IOCON_CFG_INV_BIT   BIT(3)
#define IOCON_CFG_OD_BIT    BIT(4)

/* Fixed pin masks table for LPC84x.
 * Indices match func_id from LPC84X_PINMUX_F.
 */
static const uint32_t fixed_pin_masks[] = {
	SWM_PINENABLE0_ACMP_I1_MASK,  SWM_PINENABLE0_ACMP_I2_MASK,  SWM_PINENABLE0_ACMP_I3_MASK,
	SWM_PINENABLE0_ACMP_I4_MASK,  SWM_PINENABLE0_ACMP_I5_MASK,  SWM_PINENABLE0_SWCLK_MASK,
	SWM_PINENABLE0_SWDIO_MASK,    SWM_PINENABLE0_XTALIN_MASK,   SWM_PINENABLE0_XTALOUT_MASK,
	SWM_PINENABLE0_RESETN_MASK,   SWM_PINENABLE0_CLKIN_MASK,    SWM_PINENABLE0_VDDCMP_MASK,
	SWM_PINENABLE0_I2C0_SDA_MASK, SWM_PINENABLE0_I2C0_SCL_MASK, SWM_PINENABLE0_ADC_0_MASK,
	SWM_PINENABLE0_ADC_1_MASK,    SWM_PINENABLE0_ADC_2_MASK,    SWM_PINENABLE0_ADC_3_MASK,
	SWM_PINENABLE0_ADC_4_MASK,    SWM_PINENABLE0_ADC_5_MASK,    SWM_PINENABLE0_ADC_6_MASK,
	SWM_PINENABLE0_ADC_7_MASK,    SWM_PINENABLE0_ADC_8_MASK,    SWM_PINENABLE0_ADC_9_MASK,
	SWM_PINENABLE0_ADC_10_MASK,   SWM_PINENABLE0_ADC_11_MASK,   SWM_PINENABLE0_DACOUT0_MASK,
	SWM_PINENABLE0_DACOUT1_MASK,  SWM_PINENABLE0_CAPT_X0_MASK,  SWM_PINENABLE0_CAPT_X1_MASK,
	SWM_PINENABLE0_CAPT_X2_MASK,  SWM_PINENABLE0_CAPT_X3_MASK,  SWM_PINENABLE1_CAPT_X4_MASK,
	SWM_PINENABLE1_CAPT_X5_MASK,  SWM_PINENABLE1_CAPT_X6_MASK,  SWM_PINENABLE1_CAPT_X7_MASK,
	SWM_PINENABLE1_CAPT_X8_MASK,  SWM_PINENABLE1_CAPT_YL_MASK,  SWM_PINENABLE1_CAPT_YH_MASK};

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	/* We use the global SWM and IOCON register bases */
	ARG_UNUSED(reg);

	for (uint8_t i = 0; i < pin_cnt; i++) {
		pinctrl_soc_pin_t pin = pins[i];
		bool is_fixed = Z_LPC84X_IS_FIXED(pin);
		uint8_t func_id = Z_LPC84X_GET_SWM_FUNC(pin);
		uint8_t swm_pin = Z_LPC84X_GET_SWM_PIN(pin);
		uint8_t iocon_idx = Z_LPC84X_GET_IOCON_INDEX(pin);
		uint8_t cfg = Z_LPC84X_GET_IOCON_CFG(pin);

		/* 1. Switch Matrix (SWM) Configuration */
		if (is_fixed) {
			if (func_id < ARRAY_SIZE(fixed_pin_masks)) {
				uint32_t mask = fixed_pin_masks[func_id];

				/* Bit 31 indicates PINENABLE1 for SDK helper */
				if (func_id >= SWM_FIXED_PINENABLE0_LIMIT) {
					mask |= SWM_FIXED_PINENABLE1_FLAG;
				}
				SWM_SetFixedPinSelect(PINCTRL_SWM_BASE,
						      (swm_select_fixed_pin_t)mask, true);
			}
		} else {
			SWM_SetMovablePinSelect(PINCTRL_SWM_BASE, (swm_select_movable_t)func_id,
						(swm_port_pin_type_t)swm_pin);
		}

		/* 2. I/O Configuration (IOCON) Setup */
		uint32_t iocon_val = 0;

		/* Bias (Pull-up/Pull-down/None) -> MODE bits */
		iocon_val |= IOCON_PIO_MODE(cfg & IOCON_CFG_BIAS_MASK);

		/* Hysteresis */
		if (cfg & IOCON_CFG_HYS_BIT) {
			iocon_val |= IOCON_PIO_HYS(1);
		}

		/* Input Inversion */
		if (cfg & IOCON_CFG_INV_BIT) {
			iocon_val |= IOCON_PIO_INV(1);
		}

		/* Open-Drain Mode */
		if (cfg & IOCON_CFG_OD_BIT) {
			iocon_val |= IOCON_PIO_OD(1);
		}

		/* Set the IOCON register */
		IOCON_PinMuxSet(PINCTRL_IOCON_BASE, iocon_idx, iocon_val);
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
