/*
 * @brief Blinky example using SysTick and interrupt
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
#include "sensorhubBoard.h"
#include "sensors.h"
#include "hostif.h"
#include "sensacq_i2c.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

QueueHandle_t qSenEvent;
uint32_t pd_func_mem[32];
extern void DMA_Init(void);

/*****************************************************************************
 * Private functions
 ****************************************************************************/

STATIC void setupClocking(void)
{
	/* Set main clock source to the IRC clock  This will drive 24MHz
	   for the main clock and 24MHz for the system clock */
	Chip_Clock_SetMainClockSource(SYSCON_MAINCLKSRC_IRC);

	/* Make sure the PLL is off */
	Chip_SYSCON_PowerDown(SYSCON_PDRUNCFG_PD_SYS_PLL);

    /* Wait State setting TBD */
	/* Setup FLASH access to 2 clocks (up to 20MHz) */
    Chip_SYSCON_SetFLASHAccess(FLASHTIM_20MHZ_CPU);

	/* Set system clock divider to 1 */
	Chip_Clock_SetSysClockDiv(1);

    /* ASYSNC SYSCON needs to be on or all serial peripheral won't work.
	   Be careful if PLL is used or not, ASYNC_SYSCON source needs to be
	   selected carefully. */
	Chip_SYSCON_Enable_ASYNC_Syscon(true);
	Chip_Clock_SetAsyncSysconClockDiv(1);
	Chip_Clock_SetAsyncSysconClockSource(SYSCON_ASYNC_IRC);
}

/* Initialize the interrupts for the physical sensors */
static void PhysSensors_IntInit(void)
{
	extern signed char dev_i2c_write(unsigned char dev_addr, unsigned char reg_addr, unsigned char *reg_data, unsigned char cnt);
    uint8_t dat;

	dat = 0x55;
	dev_i2c_write(ADDR_BMI160, 0x53, &dat, 1);
	dat = 0x02;
	dev_i2c_write(ADDR_BMC150A, 0x20, &dat, 1);
	dat = 0x0F;
	dev_i2c_write(ADDR_BMI055G, 0x16, &dat, 1);

    dat = 0x0A; /* Configure IRQ as open drain */
	dev_i2c_write(ADDR_BMI055, 0x20, &dat, 1);

	dat = 0x07;
	dev_i2c_write(ADDR_BMM150, 0x4E, &dat, 1);
	dat = 0x07;
	dev_i2c_write(ADDR_BMC150A, 0x4E, &dat, 1);
}

/* Setup RTC for tickless operation */
static void rtc_setup(void)
{
	Chip_SYSCON_PowerUp(SYSCON_PDRUNCFG_PD_32K_OSC);

	/* Enable RTC Clock */
	Chip_Clock_EnableRTCOsc();
	/* Initialize RTC driver (enables RTC clocking) */
	Chip_RTC_Init(LPC_RTC);

	/* Enable RTC as a peripheral wakeup event */
	Chip_SYSCON_EnableWakeup(SYSCON_STARTER_RTC);
	Chip_RTC_Reset(LPC_RTC);
	/* Clear interrupt status */
	Chip_RTC_ClearStatus(LPC_RTC, (RTC_CTRL_OFD | RTC_CTRL_ALARM1HZ | RTC_CTRL_WAKE1KHZ));
	NVIC_ClearPendingIRQ(RTC_IRQn);
	NVIC_EnableIRQ(RTC_IRQn);
}

/**
 * @brief	Setup all the hardware configurations
 */
static void hw_setup(void)
{
	/* Update core clock variables */
	SystemCoreClockUpdate();
	/* Initialize the pinmux and clocking per board connections */
	Board_Init();

	/* First thing get the IRC configured and turn the PLL off */
	setupClocking();

	SystemCoreClockUpdate();
	rtc_setup();
	Chip_POWER_RelocAPI(pd_func_mem, sizeof(pd_func_mem));

	/* Initialize kernel */
	Kernel_Init();
}

/* Sensors initialization task */
static void SensorsInit(void)
{
	/* Initialize GPIO pin interrupt module */
	Chip_PININT_Init(LPC_PININT);

    /* Init I2C */
    dev_i2c_init();
    /* Configure all IRQ's as tri-state */
	PhysSensors_IntInit();
	/* Initialize sensors */
	PhysSensors_Init();
	/* Initialize algorithm */
	Algorithm_Init();
	/* DMA Init */
	DMA_Init();
	/* Initialize host interface */
	Hostif_Init();
	/* turn INMUX clock off */
	Chip_Clock_DisablePeriphClock(SYSCON_CLOCK_INPUTMUX);
}

/* Sensor event handling task */
static void SensorEventTask(void *arg)
{
	ResMgr_DisableSleep();
	qSenEvent = xQueueCreate(MAX_SENSOR_EVENT, sizeof(int));
	configASSERT(qSenEvent); /* Check if Sensor queue is OK */

	SensorsInit();
	while (1) {
		PhysicalSensor_t *pSens;
		int idx = 0;
		ResMgr_EnableSleep();
		/* Return value need not be checked as timeout will never occur */
		xQueueReceive(qSenEvent, &idx, portMAX_DELAY);
		ResMgr_DisableSleep();
		pSens = g_phySensors[SENSOR_EVENT_ID(idx)];
		if (idx & SENSOR_EVENT_ENABLE) {
			pSens->enabled = (idx >> 17) & 1;
			pSens->hw->activate(pSens, pSens->enabled);
		}
		if (idx & SENSOR_EVENT_READ) {
			if (!pSens->hw->read(pSens))
				Algorithm_Process(pSens);
		}
	}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * Main function where the execution starts
 */
int main(void)
{
	hw_setup(); /* Set up the hardware */

	/* UART output thread, simply counts seconds */
	xTaskCreate(SensorEventTask, "SensorTask",
				configMINIMAL_STACK_SIZE + 128, NULL, (tskIDLE_PRIORITY + 1UL),
				(xTaskHandle *) NULL);

	/* Start the scheduler */
	vTaskStartScheduler();

	return 1;
}
