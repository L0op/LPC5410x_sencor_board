/*
 * @brief FreeRTOS Sensorhub Power management functions
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2014
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */
 
#include "chip.h"
#include "kernel_res_mgr.h"
#include "FreeRTOS.h"
#include "task.h"
#include "powercfg.h"

extern uint32_t pd_func_mem[];

/* Start the RTC */
static void rtc_start(uint16_t count)
{
	Chip_RTC_SetWake(LPC_RTC, count);
	Chip_RTC_EnableOptions(LPC_RTC, (RTC_CTRL_RTC1KHZ_EN | RTC_CTRL_RTC_EN));
	Chip_RTC_ClearStatus(LPC_RTC, RTC_CTRL_WAKE1KHZ); /* Clear the interrupt */
}

/* Stop RTC and get the time lapsed */
static uint16_t rtc_stop(void)
{
	uint16_t val = Chip_RTC_GetWake(LPC_RTC);
	return val;
}

/* RTC interrupt handler function */
void RTC_IRQHandler(void)
{
	void xPortSysTickHandler(void);
	Chip_RTC_ClearStatus(LPC_RTC, RTC_CTRL_WAKE1KHZ);
	xPortSysTickHandler();
}

#define portNVIC_SYSTICK_CTRL_REG			( * ( ( volatile uint32_t * ) 0xe000e010 ) )
#define portNVIC_SYSTICK_ENABLE_BIT			( 1UL << 0UL )
/* Tickless sleep */
void shTicklessSleep( TickType_t xExpectedIdleTime )
{
	uint32_t sleep_time = 0;

	if (!ResMgr_CanSleep()) {
		extern void vPortSuppressTicksAndSleep(TickType_t);
		vPortSuppressTicksAndSleep(xExpectedIdleTime);
		return;
	}
	xExpectedIdleTime *= portTICK_PERIOD_MS; /* Get the time in Milli seconds */

	/* Max possible time in RTC wakeup register */
	if (xExpectedIdleTime >= 0x10000) {
		xExpectedIdleTime = 0xFFFF;
	}
	__disable_irq();
	
	/* If a task pending no need to get into sleep */
	if (eTaskConfirmSleepModeStatus() == eAbortSleep) {
		__enable_irq();
		return;
	}

	/* Stop tick and start RTC to keep track of time during deep sleep modes */
	portNVIC_SYSTICK_CTRL_REG &= ~portNVIC_SYSTICK_ENABLE_BIT;
	rtc_start(xExpectedIdleTime);

	/* Enter low power mode here */
	/* Set voltage as low as possible */
	Chip_POWER_SetVoltage(POWER_LOW_POWER_MODE, SystemCoreClock);
	/* Now enter sleep / power down state */
	Chip_POWER_EnterPowerModeReloc(POWER_MODE, SYSCON_PDRUNCFG_PD_32K_OSC | SYSCON_PDRUNCFG_PD_SRAM0A | SYSCON_PDRUNCFG_PD_SRAM0B, (uint32_t) pd_func_mem);
	/* Return mode back to normal now that we have exited power down mode */

	portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;
	/* Out of low power mode */
	sleep_time =  xExpectedIdleTime - rtc_stop(); /* Stop RTC and get latest time */
	vTaskStepTick( (sleep_time - ((Chip_RTC_GetStatus(LPC_RTC) >> 3) & 1)) / portTICK_PERIOD_MS);
	__enable_irq(); /* Enable IRQ */
}
