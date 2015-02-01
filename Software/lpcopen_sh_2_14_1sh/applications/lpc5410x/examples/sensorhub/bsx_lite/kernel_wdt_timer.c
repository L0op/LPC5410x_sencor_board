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

static uint32_t volatile wdtOCount;
static uint32_t calWdtFreq, upperOfLimit;
static uint8_t volatile timer40HiByte;

/** Inter activity gap threshold in usecs to trigger back ground processing */
#define BACKGROUND_THRESHOLD_US                (10000)

/* WDT window size */
#define WDT_WINDOW_WARNING_SIZE           512

#define CFG_SENSOR_POLL

#ifdef CFG_SENSOR_POLL

#define WDT_WINDOW_MSEC 10

/* Maximum WDT timeout value */
#define WDTMAXTIMEOUTVAL    ((WDT_WINDOW_MSEC * calWdtFreq) / 1000)       // 10 msec interval

/* WDT timeout value used for this timer. Must be smaller than
   (WDTMAXTIMEOUTVAL + window size) */

#define WDTTIMEOUTVAL (WDTMAXTIMEOUTVAL + WDT_WINDOW_WARNING_SIZE)

#else

#define WDT_WINDOW_MSEC 5

/* Maximum WDT timeout value */
#define WDTMAXTIMEOUTVAL    0x00FFFFFF
//#define WDTMAXTIMEOUTVAL    ((WDT_WINDOW_MSEC * calWdtFreq) / 1000)

/* WDT timeout value used for this timer. Must be smaller than
   (WDTMAXTIMEOUTVAL + window size) */
//#define WDTTIMEOUTVAL (WDTMAXTIMEOUTVAL - (WDT_WINDOW_WARNING_SIZE + 1))
#define WDTTIMEOUTVAL (WDTMAXTIMEOUTVAL + WDT_WINDOW_WARNING_SIZE)

#endif


/* Enable this define to lower the kernel timer resolution by the shift
   value. Use 1 for /2, 2 for /4, 3 for /8, etc. This won't affect how the
     kernel timer works. Use 0 to disable. Maximum is 8. */
#define KERNELTIMERSHIFT    0

/* Timer 4 used for sleep timer */
#define SLEEPTIMER LPC_TIMER4

#define SLEEPDIV 8

static volatile bool inSleep;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Calibrate WDT oscillator as best as possible to IRC rate. Assumes the IRC
   is already running, but doesn't need to be the main clock source. */
static void wdt_calOsc(void)
{
    /* Enable input mux clock */
	Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_INPUTMUX);

	/* Setup to measure the selected target using the IRC as the reference */
	Chip_INMUX_SetFreqMeasRefClock(FREQMSR_IRC);
	Chip_INMUX_SetFreqMeasTargClock(FREQMSR_WDOSC);

	/* Start a measurement cycle and wait for it to complete. */
	Chip_SYSCON_StartFreqMeas();
	while (!Chip_SYSCON_IsFreqMeasComplete()) {}

	/* Get computed frequency */
	calWdtFreq = Chip_SYSCON_GetCompFreqMeas(Chip_Clock_GetIntOscRate());

	/* Frequency into WDT as a fixed divider of 4 */
	calWdtFreq = calWdtFreq / 4;
}

/* Initialize WDT to act as a simple counter using the WDT oscillator.
   No WDT reset. Interrupt only on WDT warning at maximum WDT time. */
static void wdt_init(void)
{
	/* Enable the power to the WDT Oscillator */
	Chip_SYSCON_PowerUp(SYSCON_PDRUNCFG_PD_WDT_OSC);

	/* Get WDT oscilaltor rate using frequency measurement */
	wdt_calOsc();

	/* Adjust kernel frequency by divider */
	calWdtFreq = calWdtFreq >> KERNELTIMERSHIFT;

	/* Determine overflow limit */
	//upperOfLimit = (0xFFFFFFFF / calWdtFreq) - 1;
	upperOfLimit = (0xFFFFFFFF / (WDTMAXTIMEOUTVAL >> KERNELTIMERSHIFT)) - 1;

	/* update thresholds */
	g_Timer.sleep_threshold_us =   SLEEP_THRESHOLD_US;
	g_Timer.pwrDown_threshold_us = PWRDOWN_THRESHOLD_US;
	g_Timer.bgProc_threshold_us =  BACKGROUND_THRESHOLD_US;

	/* Initialize WWDT (also enables WWDT clock) */
	Chip_WWDT_Init(LPC_WWDT);

	/* Set watchdog feed time constant (timeout) */
	Chip_WWDT_SetTimeOut(LPC_WWDT, WDTTIMEOUTVAL);
	Chip_WWDT_SetWarning(LPC_WWDT, WDT_WINDOW_WARNING_SIZE);

	/* Clear watchdog timeout interrupt */
	Chip_WWDT_ClearStatusFlag(LPC_WWDT, (WWDT_WDMOD_WDTOF | WWDT_WDMOD_WDINT));

	/* Allow WDT to wake from deep sleep */
	Chip_SYSCON_EnableWakeup(SYSCON_STARTER_WWDT);

	/* Clear and enable watchdog interrupt */
	NVIC_ClearPendingIRQ(WDT_IRQn);
	NVIC_EnableIRQ(WDT_IRQn);

	/* Start watchdog */
	Chip_WWDT_Start(LPC_WWDT);
}

/* Returns current WDT count (modified to 32-bit size) based on
   calibrated WDT rate (calWdtFreq). */
static uint32_t wdt_GetCurrent(void)
{
	uint32_t current;
    uint32_t lastWdtOCount; // counts # of WDT irq ( 1 msec units if sensor polling is enable */

	/* Overflow at 32-bits. Typical overflow is (0xFFFFFFFF/(500K))
	   = 8589 seconds (~2-3 hours). */

	/* Handle WDT count change during read */
	do {
		lastWdtOCount = wdtOCount;
		/* Counts down, with 6PCLK + 6WDT clock delay */
		current = (lastWdtOCount * (WDTMAXTIMEOUTVAL >> KERNELTIMERSHIFT)) +
				  ((WDTTIMEOUTVAL >> KERNELTIMERSHIFT) - (Chip_WWDT_GetCurrentCount(LPC_WWDT) >> KERNELTIMERSHIFT));
	} while (wdtOCount != lastWdtOCount);

	return current;
}

/* Returns current milli-seconds count  based on calibrated WDT rate (calWdtFreq). */
static uint32_t wdt_GetCurrentMsec(void)
{
    return  wdtOCount * WDT_WINDOW_MSEC;    
}

static int32_t wdt_diff(uint32_t operand1, uint32_t operand2)
{
	return operand1 - operand2;
}

static uint32_t wdt_add(uint32_t operand1, uint32_t operand2)
{
	return operand1 + operand2;
}

/* Returns WDT resolution in ticks per seconds */
static uint32_t wdt_GetResolution(void)
{
	return calWdtFreq;
}

/* Converts a passed mS count to a kernel tick time */
static uint32_t wdt_msToTicks(uint32_t milliseconds)
{
	return (milliseconds * calWdtFreq) / 1000;
}

/* Converts a kernel tick time to a mS count */
static uint32_t wdt_ticksToMs(uint32_t ticks)
{
    uint64_t t = ticks;
	return (t * 1000L) / calWdtFreq;
}


/* Converts a kernel tick time to a uS count */
static uint32_t wdt_ticksToUs(uint32_t ticks)
{
    uint64_t t = ticks;
	return (t * 1000000L) / calWdtFreq;
}

/* Returns timer tick rates in ticks/sec */
static uint32_t sleepGetTicksSec(void)
{
	return Chip_Clock_GetAsyncSyscon_ClockRate() / SLEEPDIV;
}

/* Starts the sleep timer for a period specified by ticks. Once started,
   enter sleep mode with a call to WFI(). The timer will be cleared and
   disabled automatically. */
static void sleepSetupTicks(uint32_t ticks)
{
	Chip_TIMER_Init(SLEEPTIMER);
	Chip_TIMER_PrescaleSet(SLEEPTIMER, (SLEEPDIV - 1));

	Chip_TIMER_MatchEnableInt(SLEEPTIMER, 1);
	Chip_TIMER_SetMatch(SLEEPTIMER, 1, ticks);
	Chip_TIMER_ResetOnMatchEnable(SLEEPTIMER, 1);

	inSleep = true;

	Chip_TIMER_Enable(SLEEPTIMER);

	if (Chip_TIMER_MatchPending(SLEEPTIMER, 1)) {
		Chip_TIMER_ClearMatch(SLEEPTIMER, 1);
		Chip_TIMER_DeInit(SLEEPTIMER);
		inSleep = false;
	}
	else {
		/* Enable sleep timer interrupt */
		NVIC_EnableIRQ(CT32B4_IRQn);
	}

	while (inSleep) {
		__WFI();
	}
}

/* Starts the sleep timer for a period specified by uS (microSeconds).
   Once started, enter sleep mode with a call to WFI(). The timer will be
   cleared and disabled automatically. */
static void sleepSetupMicroSecs(uint32_t uS)
{
	uint64_t sleepTicks = (uint64_t) uS;

	sleepTicks = sleepTicks * sleepGetTicksSec() / (1000 * 1000);
	sleepSetupTicks((uint32_t) sleepTicks);
}

/* Starts the sleep timer for a period specified by mS (milliSeconds).
   Once started, enter sleep mode with a call to WFI(). The timer will be
   cleared and disabled automatically. */
static void sleepSetupMilliSecs(uint32_t mS)
{
	sleepSetupMicroSecs(mS * 1000);
}

/* Interrupt safe delay using sleep and timer */
static void sleep_delayTicks(uint32_t ticks)
{
	sleepSetupMilliSecs(wdt_ticksToMs(ticks));
}

static uint8_t GetTimer40HiByte(void)
{
	return timer40HiByte;
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	watchdog timer Interrupt Handler
 * @return	Nothing
 * @note	Handles watchdog timer feed and overflow count
 */
void WDT_IRQHandler(void)
{
	/* A watchdog feed didn't occur prior to warning timeout */
	Chip_WWDT_ClearStatusFlag(LPC_WWDT, (WWDT_WDMOD_WDINT | WWDT_WDMOD_WDTOF));

	/* Feed WDT or reset will occur */
	Chip_WWDT_Feed(LPC_WWDT);

	/* Will fire every WDT timeout */
	wdtOCount++;
	if (wdtOCount > upperOfLimit) {
		wdtOCount = 0;
		timer40HiByte++;
	}
}

/* Timer interrupt for wakeup */
void CT32B4_IRQHandler(void)
{
	/* Clears timer interrupt and disables timer */
	if (Chip_TIMER_MatchPending(SLEEPTIMER, 1)) {
		Chip_TIMER_ClearMatch(SLEEPTIMER, 1);
		Chip_TIMER_DeInit(SLEEPTIMER);
		inSleep = false;
	}
}

/* create the always on timer instance */
KernelTimer_Ctrl_t g_Timer = {
	0,
	0,
	0,
	wdt_init,
	wdt_GetCurrent,
	wdt_diff,
	wdt_add,
	wdt_GetResolution,
	sleepSetupMilliSecs,
	sleep_delayTicks,					// Routed to sleep timer for interrupt safe delay
	wdt_msToTicks,
	wdt_ticksToMs,
	wdt_ticksToUs,
	GetTimer40HiByte,
    wdt_GetCurrentMsec
};
