/*
 * Copyright (c) 2026 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_NXP_LPC84X_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_NXP_LPC84X_PINCTRL_H_

#include <zephyr/dt-bindings/pinctrl/lpc84x-pinctrl.h>

/**
 * @brief Build a Movable pinmux descriptor.
 * @param func_id Movable function identifier (kSWM_USART0_TXD, etc.)
 * @param pin_id Pin identity (P0_25, etc. which includes IOCON/SWM IDs)
 */
#define LPC84X_PINMUX_M(func_id, pin_id) (((func_id) << 16) | (pin_id))

/**
 * @brief Build a Fixed pinmux descriptor.
 * @param func_id Fixed function index (into the driver's fixed_pin_masks table)
 * @param pin_id Pin identity (for IOCON configuration)
 */
#define LPC84X_PINMUX_F(func_id, pin_id) ((1U << 23) | ((func_id) << 16) | (pin_id))

/* Aliases for SWM movable functions to match common LPC84x naming conventions */
#define SWM_U0_TXD  kSWM_USART0_TXD
#define SWM_U0_RXD  kSWM_USART0_RXD
#define SWM_U0_RTS  kSWM_USART0_RTS
#define SWM_U0_CTS  kSWM_USART0_CTS
#define SWM_U0_SCLK kSWM_USART0_SCLK

#define SWM_U1_TXD  kSWM_USART1_TXD
#define SWM_U1_RXD  kSWM_USART1_RXD
#define SWM_U1_RTS  kSWM_USART1_RTS
#define SWM_U1_CTS  kSWM_USART1_CTS
#define SWM_U1_SCLK kSWM_USART1_SCLK

#define SWM_U2_TXD  kSWM_USART2_TXD
#define SWM_U2_RXD  kSWM_USART2_RXD
#define SWM_U2_RTS  kSWM_USART2_RTS
#define SWM_U2_CTS  kSWM_USART2_CTS
#define SWM_U2_SCLK kSWM_USART2_SCLK

#define SWM_U3_TXD  kSWM_USART3_TXD
#define SWM_U3_RXD  kSWM_USART3_RXD
#define SWM_U3_SCLK kSWM_USART3_SCLK

#define SWM_U4_TXD  kSWM_USART4_TXD
#define SWM_U4_RXD  kSWM_USART4_RXD
#define SWM_U4_SCLK kSWM_USART4_SCLK

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_NXP_LPC84X_PINCTRL_H_ */
