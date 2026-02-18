/*
 * Copyright (c) 2026 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SOC_H_
#define _SOC_H_

#include <zephyr/sys/util.h>

/*
 * Include MCUX device registers FIRST
 * This provides CMSIS config, IRQn_Type, and peripheral definitions
 */
#include <fsl_device_registers.h>

/*
 * Add missing CMSIS register bit definitions that Zephyr needs
 * but aren't provided by standard CMSIS headers
 */
#ifndef IPSR_ISR_Msk
#define IPSR_ISR_Msk (0x1FFUL)
#endif

#ifndef SCB_ICSR_PENDSVSET_Msk
#define SCB_ICSR_PENDSVSET_Msk (1UL << 28)
#endif

#endif /* _SOC_H_ */
