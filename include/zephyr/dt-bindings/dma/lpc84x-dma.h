/*
 * Copyright (c) 2026 Aerlync Labs Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief LPC84x DMA request source definitions.
 *
 * This header provides DMA request source identifiers
 * for use in Devicetree files. Each identifier represents
 * a DMA request source defined by the hardware.
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_DMA_LPC84X_DMA_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_DMA_LPC84X_DMA_H_

/* UART */
#define LPC84X_DMA_UART0_RX 0U /**< UART0 RX DMA */
#define LPC84X_DMA_UART0_TX 1U /**< UART0 TX DMA */
#define LPC84X_DMA_UART1_RX 2U /**< UART1 RX DMA */
#define LPC84X_DMA_UART1_TX 3U /**< UART1 TX DMA */
#define LPC84X_DMA_UART2_RX 4U /**< UART2 RX DMA */
#define LPC84X_DMA_UART2_TX 5U /**< UART2 TX DMA */
#define LPC84X_DMA_UART3_RX 6U /**< UART3 RX DMA */
#define LPC84X_DMA_UART3_TX 7U /**< UART3 TX DMA */
#define LPC84X_DMA_UART4_RX 8U /**< UART4 RX DMA */
#define LPC84X_DMA_UART4_TX 9U /**< UART4 TX DMA */

/* SPI */
#define LPC84X_DMA_SPI0_RX 10U /**< SPI0 RX DMA */
#define LPC84X_DMA_SPI0_TX 11U /**< SPI0 TX DMA */
#define LPC84X_DMA_SPI1_RX 12U /**< SPI1 RX DMA */
#define LPC84X_DMA_SPI1_TX 13U /**< SPI1 TX DMA */

/* I2C */
#define LPC84X_DMA_I2C0_SLAVE  14U /**< I2C0 SLAVE DMA */
#define LPC84X_DMA_I2C0_MASTER 15U /**< I2C0 MASTER DMA */
#define LPC84X_DMA_I2C1_SLAVE  16U /**< I2C1 SLAVE DMA */
#define LPC84X_DMA_I2C1_MASTER 17U /**< I2C1 MASTER DMA */
#define LPC84X_DMA_I2C2_SLAVE  18U /**< I2C2 SLAVE DMA */
#define LPC84X_DMA_I2C2_MASTER 19U /**< I2C2 MASTER DMA */
#define LPC84X_DMA_I2C3_SLAVE  20U /**< I2C3 SLAVE DMA */
#define LPC84X_DMA_I2C3_MASTER 21U /**< I2C3 MASTER DMA */

/* DAC */
#define LPC84X_DMA_DAC0 22U /**< DAC0 DMA request */
#define LPC84X_DMA_DAC1 23U /**< DAC1 DMA request */

/* Capture */
#define LPC84X_DMA_CAPT 24U /**< Capture DMA */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_DMA_LPC84X_DMA_H_ */
