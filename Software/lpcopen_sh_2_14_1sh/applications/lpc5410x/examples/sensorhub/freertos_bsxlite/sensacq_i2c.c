/*
 * @brief I2CM driver used by Sensor acquisition task
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
#include <string.h>
#include "sensorhub.h"
#include "sensacq_i2c.h"
#include "FreeRTOS.h"
#include "semphr.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
#if !defined(I2CM_IRQ_BASED)
#define I2C_MODE_FLAG	ROM_I2CM_FLAG_BLOCKING
#else
#define I2C_MODE_FLAG	0
static SemaphoreHandle_t sem_i2cm;
#endif

/* ROM driver handle for I2C master */
static ROM_I2CM_HANDLE_T i2cmHandle;

/* I2CM transfer record */
static ROM_I2CM_XFER_T i2cmXferRec;

static uint8_t g_i2cmTxBuf[CFG_MAX_SA_TX_BUFFER];

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/
extern uint8_t timeStampExtender;

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Setup I2C Master */
static void setupI2CMaster(void)
{
    ROM_I2CM_INIT_T i2cmInit;
    static uint32_t i2cmContextData[16];
    
	/* Enable I2C clock and reset I2C peripheral */
    Chip_Clock_EnablePeriphClock(I2C_SENSOR_CLOCK);
	Chip_SYSCON_PeriphReset(I2C_SENSOR_RESET);

    /* Initialize driver */
	i2cmInit.pUserData = NULL;
	i2cmInit.base = (uint32_t) I2C_SENSOR_BUS;
	i2cmHandle = ROM_I2CM_Init(i2cmContextData, &i2cmInit);
	if (i2cmHandle == NULL) {
		/* Error initializing I2C */
		/* FIXME implement some sort of error indicator */
	} 

	/* Setup I2CM transfer rate */
    ROM_I2CM_SetClockRate(i2cmHandle,
	Chip_Clock_GetAsyncSyscon_ClockRate(), I2C_SENSOR_MCLOCK_SPEED);

}

static void i2c_xfer(ROM_I2CM_HANDLE_T hI2C, ROM_I2CM_XFER_T *xfer)
{
	/* I2C master driver will block if blocking flag is used */
	xfer->flags = I2C_MODE_FLAG;

	/* Start transfer and wait for completion */
	ROM_I2CM_Transfer(hI2C, xfer);
#ifdef I2CM_IRQ_BASED
	/* Return value need not be checked as timeout will never occur */
	xSemaphoreTake(sem_i2cm, portMAX_DELAY);
#endif
}

#ifdef I2CM_IRQ_BASED
static void i2cMasterDoneCallback(ROM_I2CM_HANDLE_T handle, ROM_I2CM_XFER_T *pXfer)
{
	BaseType_t htWake = pdFALSE;
	xSemaphoreGiveFromISR(sem_i2cm, &htWake);
	portEND_SWITCHING_ISR(htWake);
}
#endif

/*****************************************************************************
 * Public functions
 ****************************************************************************/
/**
 * @brief	Handle I2C0 interrupt by calling I2CM interrupt transfer handler
 * @return	Nothing
 */
#ifdef I2CM_IRQ_BASED
void I2C_SENSOR_BUS_IRQHandler(void)
{
	/* Call I2CM transfer handler function */
	ROM_I2CM_TransferHandler(i2cmHandle);
}
#endif

/* Initialize I2C bus connected sensors */
bool dev_i2c_init(void)
{
	static bool init;

	if (init)
		return init;

	init = 1;
	/* Setup I2C and master */
	setupI2CMaster();

#ifdef I2CM_IRQ_BASED
	/* Create a binary semaphore */
	sem_i2cm = xSemaphoreCreateBinary();

	/* Check for successful semaphore creation */
	configASSERT(sem_i2cm);

	/* Register the transfer completion callback */
	ROM_I2CM_RegisterCallback(i2cmHandle, ROM_I2CM_DATACOMPLETE_CB, (void *) i2cMasterDoneCallback);

	NVIC_SetPriority(I2C_SENSOR_BUS_IRQn, SENSOR_I2C_PRIORITY);

	/* Enable the interrupt for the I2C */
	NVIC_EnableIRQ(I2C_SENSOR_BUS_IRQn);
#endif

	return init;
}

/* adaptation of the Bosch API functions to the BOARD specific function (i.e delay) */
void dev_i2c_delay(unsigned int msec)
{
	/* Will WFI for the entire sleep duration. If an interrupt occurs that wakes the device,
	   the sleep handler will automatically re-enter WFI until the duration has expired. */
	g_Timer.delayMs(msec);
}

/* write cnt bytes  */
signed char dev_i2c_write(unsigned char dev_addr, unsigned char reg_addr, unsigned char *reg_data, unsigned char cnt)
{
	/* Setup I2C transfer record */
	g_i2cmTxBuf[0] = reg_addr;
	if (cnt == 1) {
		g_i2cmTxBuf[1] = reg_data[0];
	}
	else if ((cnt > 0) && (cnt < CFG_MAX_SA_TX_BUFFER)) {
		memcpy(&g_i2cmTxBuf[1], reg_data, cnt);
	}

	memset(&i2cmXferRec, 0, sizeof(i2cmXferRec));

	i2cmXferRec.slaveAddr = dev_addr;
	i2cmXferRec.status = LPC_OK;
	i2cmXferRec.txSz = cnt + 1;
	i2cmXferRec.rxSz = 0;
	i2cmXferRec.txBuff = &g_i2cmTxBuf[0];
	i2cmXferRec.rxBuff = 0;

	/* Start transfer and wait for completion */
	i2c_xfer(i2cmHandle, &i2cmXferRec);

	return LPC_OK;
}

/* read cnt characters at i2c dev_addr and reg_addr */
signed char dev_i2c_read(unsigned char dev_addr, unsigned char reg_addr, unsigned char *reg_data, unsigned char cnt)
{
	memset(&i2cmXferRec, 0, sizeof(i2cmXferRec));
	/* Setup I2C transfer record */
	i2cmXferRec.slaveAddr = dev_addr;
	i2cmXferRec.status = LPC_OK;
	i2cmXferRec.txSz = 1;
	i2cmXferRec.rxSz = cnt;
	i2cmXferRec.txBuff = &reg_addr;
	i2cmXferRec.rxBuff = reg_data;

	/* Start transfer and wait for completion */
	i2c_xfer(i2cmHandle, &i2cmXferRec);

	return LPC_OK;
}

/* plain i2c read */
char dev_i2c_readOnly(unsigned char dev_addr, unsigned char *reg_data, unsigned char cnt)
{
	memset(&i2cmXferRec, 0, sizeof(i2cmXferRec));
	/* Setup I2C transfer record */
	i2cmXferRec.slaveAddr = dev_addr;
	i2cmXferRec.status = LPC_OK;
	i2cmXferRec.txSz = 0;
	i2cmXferRec.rxSz = cnt;
	i2cmXferRec.txBuff = 0;
	i2cmXferRec.rxBuff = reg_data;

    /* I2C master driver will block if blocking flag is used */
	i2cmXferRec.flags = I2C_MODE_FLAG;

	/* Start transfer and wait for completion */
	i2c_xfer(i2cmHandle, &i2cmXferRec);

	return LPC_OK;
}
