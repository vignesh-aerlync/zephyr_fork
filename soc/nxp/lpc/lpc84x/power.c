/*
 * Copyright (c) 2026 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/pm/pm.h>
#include <zephyr/init.h>
#include <zephyr/sys/printk.h>
#include <soc.h>
#include <fsl_power.h>
#include "fsl_wkt.h"

#define LPC84X_DEEP_SLEEP_ACTIVE_PARTS                                                             \
	(kPDSLEEPCFG_DeepSleepBODActive | kPDSLEEPCFG_DeepSleepWDTOscActive)
#define LPC84X_POWER_DOWN_ACTIVE_PARTS (0U)

/* Variables to store clock configuration to restore after wake-up */
static uint32_t saved_mainclksel;
static uint32_t saved_mainclkpllsel;

static void switch_to_fro(void)
{
	/* Save current clock settings */
	saved_mainclksel = SYSCON->MAINCLKSEL;
	saved_mainclkpllsel = SYSCON->MAINCLKPLLSEL;

	/* Switch Main clock to FRO */
	SYSCON->MAINCLKSEL = 0U; /* FRO */
	SYSCON->MAINCLKUEN = 0U;
	SYSCON->MAINCLKUEN = 1U;

	/* Bypass PLL, use Main clock directly */
	SYSCON->MAINCLKPLLSEL = 0U; /* Main clock pre-PLL */
	SYSCON->MAINCLKPLLUEN = 0U;
	SYSCON->MAINCLKPLLUEN = 1U;
}

static void restore_clock(void)
{
	/* Restore Main clock selection */
	SYSCON->MAINCLKSEL = saved_mainclksel;
	SYSCON->MAINCLKUEN = 0U;
	SYSCON->MAINCLKUEN = 1U;

	/* Restore PLL selection */
	SYSCON->MAINCLKPLLSEL = saved_mainclkpllsel;
	SYSCON->MAINCLKPLLUEN = 0U;
	SYSCON->MAINCLKPLLUEN = 1U;
}

void WKT_IRQHandler(void)
{
	WKT_ClearStatusFlags(WKT, kWKT_AlarmFlag);
}

/* Helper function to arm the Wake-up Timer before sleeping */
static void arm_wkt_for_sleep(uint32_t ticks_10khz)
{
	wkt_config_t config;

	/* 1. Ensure Low Power Oscillator is enabled */
	PMU->DPDCTRL |= PMU_DPDCTRL_LPOSCEN_MASK | PMU_DPDCTRL_LPOSCDPDEN_MASK;

	/* 2. Enable Asynchronous Wakeup pathway */
	EnableDeepSleepIRQ(WKT_IRQn);

	/* 3. Configure and start WKT */
	config.clockSource = kWKT_LowPowerClockSource;
	WKT_Init(WKT, &config);
	WKT_StartTimer(WKT, ticks_10khz);
}

void pm_state_set(enum pm_state state, uint8_t id)
{
	ARG_UNUSED(id);

	switch (state) {
	case PM_STATE_RUNTIME_IDLE:
		/*
		 * Sleep mode: CPU clock gated, all peripheral keep running
		 * POWER_EnterSleep() clears SCB->SCR.SLEEPDEEP then execute
		 * WFI
		 */
		POWER_EnterSleep();
		break;

	case PM_STATE_STANDBY:
		/*
		 * Deep-sleep mode: most clocks stopped, SRAM retained.
		 * POWER_EnterDeepSleep() sets PMU->PCON to kPmu_Deep_Sleep,
		 * masks PDSLEEPCFG to keep activeParts powered, sets
		 * SCB->SCR.SLEEPDEEP, then executes WFI.
		 */
		arm_wkt_for_sleep(200);
		switch_to_fro();
		POWER_EnterDeepSleep(LPC84X_DEEP_SLEEP_ACTIVE_PARTS);
		break;

	case PM_STATE_SUSPEND_TO_IDLE:
		/*
		 * Deep power-down: only the PMU wake-up logic remains.
		 * The core does not return from WFI — reset on wake.
		 */
		arm_wkt_for_sleep(1000);
		switch_to_fro();
		POWER_EnterPowerDown(LPC84X_POWER_DOWN_ACTIVE_PARTS);
		break;

	case PM_STATE_SOFT_OFF:
		/* Clear WKT Flags */
		WKT_ClearStatusFlags(WKT, kWKT_AlarmFlag);
		NVIC_ClearPendingIRQ(WKT_IRQn);

		/* 2. Set the Wakeup Timer (1000 ticks = 100ms) */
		arm_wkt_for_sleep(100000);

		/* 3. Read and print the registers directly from the CPU! */
		POWER_EnterDeepPowerDownMode();
		break;

	default:
		break;
	}
}

void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	switch (state) {
	case PM_STATE_RUNTIME_IDLE:
		break;

	case PM_STATE_STANDBY:
	case PM_STATE_SUSPEND_TO_IDLE:
		restore_clock();
		break;

	case PM_STATE_SOFT_OFF:
		break;

	default:
		break;
	}
	irq_unlock(0);
}

static int lpc84x_wkt_init(void)
{
	IRQ_CONNECT(WKT_IRQn, 0, WKT_IRQHandler, NULL, 0);
	return 0;
}

SYS_INIT(lpc84x_wkt_init, POST_KERNEL, 99);
