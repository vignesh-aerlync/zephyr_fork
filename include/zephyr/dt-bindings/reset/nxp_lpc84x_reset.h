/*
 * Copyright 2026 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_RESET_NXP_LPC84X_RESET_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_RESET_NXP_LPC84X_RESET_H_

#include <zephyr/dt-bindings/reset/nxp_syscon_reset_common.h>

/*
 * Encoding: ((reg_index << 16) | bit_pos)
 * reg_index 0: PRESETCTRL0, 1: PRESETCTRL1. Active-low.
 */

/* PRESETCTRL0 peripherals (reg=0) */
#define LPC84X_RST_FLASH    NXP_SYSCON_RESET(0, 4)  /**< Flash controller */
#define LPC84X_RST_I2C0     NXP_SYSCON_RESET(0, 5)  /**< I2C0 */
#define LPC84X_RST_GPIO0    NXP_SYSCON_RESET(0, 6)  /**< GPIO port 0 */
#define LPC84X_RST_SWM      NXP_SYSCON_RESET(0, 7)  /**< Switch matrix */
#define LPC84X_RST_SCT      NXP_SYSCON_RESET(0, 8)  /**< State-configurable timer */
#define LPC84X_RST_WKT      NXP_SYSCON_RESET(0, 9)  /**< Self-wake-up timer */
#define LPC84X_RST_MRT      NXP_SYSCON_RESET(0, 10) /**< Multi-rate timer */
#define LPC84X_RST_SPI0     NXP_SYSCON_RESET(0, 11) /**< SPI0 */
#define LPC84X_RST_SPI1     NXP_SYSCON_RESET(0, 12) /**< SPI1 */
#define LPC84X_RST_CRC      NXP_SYSCON_RESET(0, 13) /**< CRC engine */
#define LPC84X_RST_UART0    NXP_SYSCON_RESET(0, 14) /**< UART0 */
#define LPC84X_RST_UART1    NXP_SYSCON_RESET(0, 15) /**< UART1 */
#define LPC84X_RST_UART2    NXP_SYSCON_RESET(0, 16) /**< UART2 */
#define LPC84X_RST_IOCON    NXP_SYSCON_RESET(0, 18) /**< I/O configuration */
#define LPC84X_RST_ACMP     NXP_SYSCON_RESET(0, 19) /**< Analog comparator */
#define LPC84X_RST_GPIO1    NXP_SYSCON_RESET(0, 20) /**< GPIO port 1 */
#define LPC84X_RST_I2C1     NXP_SYSCON_RESET(0, 21) /**< I2C1 */
#define LPC84X_RST_I2C2     NXP_SYSCON_RESET(0, 22) /**< I2C2 */
#define LPC84X_RST_I2C3     NXP_SYSCON_RESET(0, 23) /**< I2C3 */
#define LPC84X_RST_ADC      NXP_SYSCON_RESET(0, 24) /**< ADC */
#define LPC84X_RST_CTIMER0  NXP_SYSCON_RESET(0, 25) /**< CTimer0 */
#define LPC84X_RST_DAC0     NXP_SYSCON_RESET(0, 27) /**< DAC0 */
#define LPC84X_RST_GPIOINT  NXP_SYSCON_RESET(0, 28) /**< GPIO interrupt (PINT) */
#define LPC84X_RST_DMA      NXP_SYSCON_RESET(0, 29) /**< DMA controller */
#define LPC84X_RST_UART3    NXP_SYSCON_RESET(0, 30) /**< UART3 */
#define LPC84X_RST_UART4    NXP_SYSCON_RESET(0, 31) /**< UART4 */

/* PRESETCTRL1 peripherals (reg=1) */
#define LPC84X_RST_CAPT     NXP_SYSCON_RESET(1, 0) /**< Capacitive touch */
#define LPC84X_RST_DAC1     NXP_SYSCON_RESET(1, 1) /**< DAC1 */
#define LPC84X_RST_FRG0     NXP_SYSCON_RESET(1, 3) /**< Fractional baud rate gen 0 */
#define LPC84X_RST_FRG1     NXP_SYSCON_RESET(1, 4) /**< Fractional baud rate gen 1 */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_RESET_NXP_LPC84X_RESET_H_ */
