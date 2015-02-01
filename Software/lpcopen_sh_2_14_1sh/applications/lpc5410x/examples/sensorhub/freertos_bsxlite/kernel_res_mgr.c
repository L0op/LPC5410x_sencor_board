/*
 * @brief I2C based host interface module
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
#include "power_lib_5410x.h"
#include "sensorhub.h"
#include "kernel_res_mgr.h"
#include "powercfg.h"
#include "FreeRTOS.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
#define USE_LP_REG
#define CFG_LP_VD1			POWER_LP_V0700
#define PLL_MULTIPLIER      7  /*  7 * 12 MHz IRC = 84Mhz */

#define MAX_DURATION_US                     1000000

static ResMgr_Mode_T currMode;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/
ResMgr_Ctrl_T g_ResMgr;
volatile uint32_t no_sleep;

/*****************************************************************************
 * Private functions
 ****************************************************************************/
/* Set current mode */
static INLINE void SetCurrentMode(ResMgr_Mode_T newMode)
{
	currMode = newMode;
}

/* Initialize resource manager */
static void ResMgr_Init(void)
{
    /* Initialize the current mode to known value */
	currMode = RESMGR_STARTUP;

	/* disable RTC */
	Chip_Clock_DisableRTCOsc();
	/* disable BOD for time being.*/
    Chip_SYSCON_PowerDown(SYSCON_PDRUNCFG_PD_BOD_RST | SYSCON_PDRUNCFG_PD_BOD_INTR);

	/* Init to normal mode */
	ResMgr_EnterNormalMode();

    /* Switch over to the IRC so that the PLL can be configured */
	Chip_Clock_SetSystemPLLSource(SYSCON_PLLCLKSRC_IRC);

    /* Power down PLL to change the PLL divider ratio */
    Chip_SYSCON_PowerDown(SYSCON_PDRUNCFG_PD_SYS_PLL);

    /* First parameter is the multiplier, the second parameter is the input frequency in MHz */
	Chip_Clock_SetupSystemPLL(PLL_MULTIPLIER, Chip_Clock_GetMainClockRate());
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	Request resource manager to put system in Power-down mode.
 * @param   estSleepTime    estimate of how long (uSec) we will be in sleep mode.
 * @return	Nothing
 */
void ResMgr_EnterPowerDownMode(uint32_t estSleepTime)
{
	POWER_MODE_T  powerMode = POWER_SLEEP;

	if (estSleepTime > g_Timer.pwrDown_threshold_us) {
		powerMode = POWER_MODE;
	}
	else if (estSleepTime < g_Timer.sleep_threshold_us) {
		/* skip sleep */
		g_ResMgr.fSkipSleep = 1;
	}

    /* Make sure that we are in NORMAL mode before going to sleep */
	if (currMode == RESMGR_PLL_MODE) {
		ResMgr_EnterNormalMode();
	}

    if (g_ResMgr.fSkipSleep == 0) {
        /* Only update mode if actually going into power down */
         SetCurrentMode(RESMGR_PWRDOWN_MODE);
        /* Set voltage as low as possible */
        Chip_POWER_SetVoltage(POWER_LOW_POWER_MODE, Chip_Clock_GetMainClockRate());
        /* Now enter sleep / power down state */
        Chip_POWER_EnterPowerMode(powerMode, (SYSCON_PDRUNCFG_PD_WDT_OSC | SYSCON_PDRUNCFG_PD_SRAM0A | SYSCON_PDRUNCFG_PD_SRAM0B));
        /* Return mode back to normal now that we have exited power down mode */
        SetCurrentMode(RESMGR_NORMAL_MODE);
    }

	/* clear skip flag */
	g_ResMgr.fSkipSleep = 0;
}

/**
 * @brief	Request resource manager to put system in normal operation mode.
 * @return	Nothing
 */
void ResMgr_EnterNormalMode(void)
{
	if (currMode != RESMGR_NORMAL_MODE) {
		uint32_t clk;
		if (currMode == RESMGR_PLL_MODE) {
			Chip_Clock_SetMainClockSource(SYSCON_MAINCLKSRC_IRC);
			/* trun off the PLL */
			Chip_SYSCON_PowerDown(SYSCON_PDRUNCFG_PD_SYS_PLL);
		}

		clk = Chip_Clock_GetMainClockRate();
        /* Set to lowest power possible for our frequency */
        Chip_POWER_SetVoltage(POWER_LOW_POWER_MODE, clk);

        SetCurrentMode(RESMGR_NORMAL_MODE);
		Chip_Clock_SetSysTickClockDiv(clk / configSYSTICK_CLOCK_HZ);
	}
}

/**
 * @brief	Request resource manager to put system in PLL mode for higher computational power.
 * @return	Nothing
 */
void ResMgr_EnterPLLMode(void)
{
	if (currMode == RESMGR_NORMAL_MODE) {

        /* By default, VDx levels are set to low to medium to preserve power.
		   In order to run the code at higher speed using PLL, the VDx level
		   have to be raised. Be careful with the FLASH wait state and VD level
		   if you want to set the PLL in order to run at the highest frequency possible. */
        Chip_POWER_SetVoltage(POWER_LOW_POWER_MODE, Chip_Clock_GetSystemPLLOutClockRate(false));

		/* Turn on the PLL by clearing the power down bit.  The PLL should have already
           been setup. */
		Chip_SYSCON_PowerUp(SYSCON_PDRUNCFG_PD_SYS_PLL);

		/* Wait for PLL to lock */
		while (!Chip_Clock_IsSystemPLLLocked()) {}

		/* Set main clock source to the system PLL. This will drive 84MHz
		   for the main clock and 84MHz for the system clock */
		Chip_Clock_SetMainClockSource(SYSCON_MAINCLKSRC_PLLOUT);
		Chip_Clock_SetSysTickClockDiv(Chip_Clock_GetMainClockRate() / configSYSTICK_CLOCK_HZ);
	}
    /* Set current mode to RESMGR_PLL_MODE */
    SetCurrentMode(RESMGR_PLL_MODE);
}

/**
 * @brief	Initialize internal power handling.
 * @return	Nothing
 */
void Kernel_Init(void)
{
	/* Initialize resource manager */
	ResMgr_Init();
}
