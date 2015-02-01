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

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

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

#ifdef I2CM_IRQ_BASED
/* Function to wait for I2CM transfer completion */
static void WaitForI2cXferComplete(ROM_I2CM_XFER_T *xferRecPtr)
{
	/* Are we still transferring data ? */
	while (xferRecPtr->status == ERR_I2C_BUSY) {
		/* Sleep until next interrupt */
		__WFI();
	}
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
	static bool init = false;

	if (init == false) {

		/* Setup I2C and master */
		setupI2CMaster();

#ifdef I2CM_IRQ_BASED
		NVIC_SetPriority(I2C_SENSOR_BUS_IRQn, SENSOR_I2C_PRIORITY);

		/* Enable the interrupt for the I2C */
		NVIC_EnableIRQ(I2C_SENSOR_BUS_IRQn);
#endif
		init = true;
	}
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
    
#ifdef I2CM_IRQ_BASED
	/* Start transfer and wait for completion */
	ROM_I2CM_Transfer(i2cmHandle, &i2cmXferRec);
	/* Wait for transfer completion */
	WaitForI2cXferComplete(&i2cmXferRec);
#else
    /* I2C master driver will block if blocking flag is used */
	i2cmXferRec.flags = ROM_I2CM_FLAG_BLOCKING;

	/* Start transfer and wait for completion */
	ROM_I2CM_Transfer(i2cmHandle, &i2cmXferRec);
   
#endif

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

#ifdef I2CM_IRQ_BASED
	/* Start transfer and wait for completion */
	ROM_I2CM_Transfer(i2cmHandle, &i2cmXferRec);
	/* Wait for transfer completion */
	WaitForI2cXferComplete(&i2cmXferRec);
#else
    /* I2C master driver will block if blocking flag is used */
	i2cmXferRec.flags = ROM_I2CM_FLAG_BLOCKING;

	/* Start transfer and wait for completion */
	ROM_I2CM_Transfer(i2cmHandle, &i2cmXferRec);
#endif

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

#ifdef I2CM_IRQ_BASED
	/* Start transfer and wait for completion */
	ROM_I2CM_Transfer(i2cmHandle, &i2cmXferRec);
	/* Wait for transfer completion */
	WaitForI2cXferComplete(&i2cmXferRec);
#else
    /* I2C master driver will block if blocking flag is used */
	i2cmXferRec.flags = ROM_I2CM_FLAG_BLOCKING;

	/* Start transfer and wait for completion */
	ROM_I2CM_Transfer(i2cmHandle, &i2cmXferRec);
#endif

	return LPC_OK;
}
