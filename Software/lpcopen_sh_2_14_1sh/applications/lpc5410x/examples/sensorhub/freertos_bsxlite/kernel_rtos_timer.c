/*
 * @brief Windowed Watchdog Timer (WWDT) example
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

#include "board.h"
#include "sensorhub.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/


/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/


#include "FreeRTOS.h"
#include "task.h"

static void timer_init(void)
{
	/* Nothing to initialize here */
}

static int32_t timer_diff(uint32_t operand1, uint32_t operand2)
{
	return operand1 - operand2;
}

static uint32_t timer_add(uint32_t operand1, uint32_t operand2)
{
	return operand1 + operand2;
}

static uint32_t timer_ticksToUs(uint32_t ticks)
{
	return ticks * 1000UL * portTICK_PERIOD_MS;
}

static uint32_t timer_ticksToMs(uint32_t ticks)
{
    return ticks * portTICK_PERIOD_MS;
}

/* Converts a passed mS count to a kernel tick time */
static uint32_t timer_msToTicks(uint32_t milliseconds)
{
	return milliseconds / portTICK_PERIOD_MS;
}

static uint32_t timer_GetCurrentMsec(void)
{
	return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

void timer_delayMs(uint32_t ms)
{
	vTaskDelay(ms / portTICK_PERIOD_MS);
}

uint32_t timer_getResol(void)
{
#ifdef configSYSTICK_CLOCK_HZ
	return configSYSTICK_CLOCK_HZ;
#else
	return configCPU_CLOCK_HZ;
#endif
}

/* create the always on timer instance */
KernelTimer_Ctrl_t g_Timer = {
	0,
	0,
	0,
	timer_init,
	xTaskGetTickCount,
	timer_diff,
	timer_add,
	timer_getResol,
	timer_delayMs,
	vTaskDelay,					// Routed to sleep timer for interrupt safe delay
	timer_msToTicks,
	timer_ticksToMs,
	timer_ticksToUs,
	0,
    timer_GetCurrentMsec
};
