/*
 * Copyright (c) 2026 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_ARM_NXP_LPC_LPC84X_PINCTRL_SOC_H_
#define ZEPHYR_SOC_ARM_NXP_LPC_LPC84X_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <zephyr/types.h>

/**
 * @brief LPC84x pin descriptor type.
 *
 * Encoding:
 * - Bits 0..7:   IOCON Index (matches lpc84x-pinctrl.h)
 * - Bits 8..15:  SWM Port/Pin ID (matches lpc84x-pinctrl.h)
 * - Bits 16..22: SWM Function ID (Movable index)
 * - Bit 23:      SWM Type (0 = Movable, 1 = Fixed)
 * - Bits 24..31: IOCON Configuration (Bias, OD, Inv, Hys)
 */
typedef uint32_t pinctrl_soc_pin_t;

/* Extractor macros */
#define Z_LPC84X_GET_IOCON_INDEX(mux) ((mux) & 0xFF)
#define Z_LPC84X_GET_SWM_PIN(mux)     (((mux) >> 8) & 0xFF)
#define Z_LPC84X_GET_SWM_FUNC(mux)    (((mux) >> 16) & 0x7F)
#define Z_LPC84X_IS_FIXED(mux)        (((mux) & (1U << 23)) != 0)
#define Z_LPC84X_GET_IOCON_CFG(mux)   (((mux) >> 24) & 0xFF)

/**
 * @brief Build IOCON configuration bits for storage in pinctrl_soc_pin_t.
 * We map DT properties to a compact 8-bit representation:
 * - Bits 0..1: Bias (0=None, 1=Pull-down, 2=Pull-up) -> IOCON bits 3..4
 * - Bit 2: Hysteresis -> IOCON bit 5
 * - Bit 3: Invert -> IOCON bit 6
 * - Bit 4: Open-drain -> IOCON bit 10
 */

#define Z_PINCTRL_IOCON_PINCFG(node_id)                                                            \
	(((IF_ENABLED(DT_PROP(node_id, bias_pull_down), (1U |)) IF_ENABLED(DT_PROP(node_id, bias_pull_up), (2U |)) 0U) << 0) |   \
		      (DT_PROP(node_id, nxp_hysteresis) << 2) |                                    \
		      (DT_PROP(node_id, nxp_invert) << 3) |                                        \
		      (DT_PROP(node_id, drive_open_drain) << 4))

/**
 * @brief Utility macro to initialize a pinmux + configuration.
 */
#define Z_PINCTRL_STATE_PIN_INIT(group, pin_prop, idx)                                             \
	(DT_PROP_BY_IDX(group, pin_prop, idx) | (Z_PINCTRL_IOCON_PINCFG(group) << 24)),

/**
 * @brief Utility macro to initialize state pins.
 */
#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                                                   \
	{DT_FOREACH_CHILD_VARGS(DT_PHANDLE(node_id, prop), DT_FOREACH_PROP_ELEM, pinmux,           \
				Z_PINCTRL_STATE_PIN_INIT)}

#endif /* ZEPHYR_SOC_ARM_NXP_LPC_LPC84X_PINCTRL_SOC_H_ */
