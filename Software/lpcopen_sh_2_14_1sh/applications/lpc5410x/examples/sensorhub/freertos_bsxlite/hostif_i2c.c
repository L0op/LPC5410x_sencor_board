/*
 * @brief Implements I2C slave driver for host interface module
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
#include "hostif.h"

#if defined(HOSTIF_I2C)
/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
#define RX_LENGTH                   16	// sizeof(union ShCmdHeaderUnion)

typedef struct __HOSTIF_I2CCtrl_t {
	uint8_t rxCount;			/* Bytes so far received  */
	uint8_t rxLength;			/* Expected Rx buffer length */
	uint8_t txCount;			/* Bytes so far transmitted */
	uint8_t txLength;			/* Total transfer length of Tx buffer */
	uint8_t *txBuff;			/* Tx buffer pointer */
	uint8_t rxBuff[RX_LENGTH];	/* Rx buffer */
} Hostif_I2CCtrl_t;

static Hostif_I2CCtrl_t g_i2cHostif;

#define I2C_MEM_SZ    64 /* Size of memory for I2C Slave ROM driver */

/* Handle to I2C */
static ROM_I2CS_HANDLE_T hI2C;

/* I2C transfer structure; directly manipulated by ROM driver */
static ROM_I2CS_XFER_T i2cXfer;

/* Rx state handling variable */
static int32_t rx_done;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* I2C Receive callback [Will be called when a byte is ready to be received;
 * It also resets bytesSend to 0 */
static ROM_I2CS_TRANCTRL_T i2cs_tx(ROM_I2CS_HANDLE_T hI2C, ROM_I2CS_XFER_T *pXfer)
{
	i2cXfer.txBuff = (uint8_t *)i2cXfer.txBuff + i2cXfer.txSz;
	i2cXfer.txSz ++;
	Hostif_TxCompleteCB();
	return ROM_I2CS_CONTINUE;
}

/* I2C Receive callback [Will be called when a byte is ready to be received;
 * It also resets bytesRecv to 0 */
static ROM_I2CS_TRANCTRL_T i2cs_rx(ROM_I2CS_HANDLE_T hI2C, ROM_I2CS_XFER_T *pXfer)
{
	ResMgr_DisableSleep();
	rx_done = 1;
	i2cXfer.rxBuff = (uint8_t *)i2cXfer.rxBuff + i2cXfer.rxSz;
	i2cXfer.rxSz ++;
	return ROM_I2CS_CONTINUE;
}

/* I2C Done event callback [Will be called after STOP bit is received] */
static void i2cs_done(ROM_I2CS_HANDLE_T hI2C, ROM_I2CS_XFER_T *pXfer)
{
	i2cXfer.rxBuff = g_i2cHostif.rxBuff;
	i2cXfer.txSz = i2cXfer.rxSz = 0;
	i2cXfer.txBuff = 0;
	ROM_I2CS_Transfer(hI2C, &i2cXfer);
	ResMgr_EnableSleep();
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/
/**
 * @brief	IRQ Handler for I2C Slave interface
 * @return	Nothing
 */
void I2C_HOSTIF_IRQHandler(void)
{
	int ret;
	rx_done = 0;
	ROM_I2CS_TransferHandler(hI2C);
	if (rx_done) {
		ret = Hostif_CmdProcess(g_i2cHostif.rxBuff, i2cXfer.rxSz);
		if (g_i2cHostif.rxBuff == i2cXfer.rxBuff) {
			i2cXfer.rxSz = ret;
		}
	}
}

/**
 * @brief	Primes the Tx Buffer for transfer
 * @return	Nothing
 */
void Hostif_StartTx(uint8_t *pBuf, uint16_t size, uint8_t cmd)
{
	if (!(pBuf && size))
		return;

	i2cXfer.txBuff = pBuf;
	i2cXfer.txSz = size - 1;
}

/**
 * @brief	Called to check if it is OK to sleep
 * @return	Nothing
 */
uint8_t Hostif_SleepOk(void)
{
	return (i2cXfer.txSz != 0);
}

/**
 * @brief	Initialize the Sensor Hub host/AP I2C interface
 * @return	Nothing
 */
void Hostif_Init(void)
{
	ROM_I2CS_INIT_T i2csInit;
	ROM_I2CS_SLAVE_T slaveSetup;
	static uint32_t devMem[I2C_MEM_SZ/sizeof(uint32_t)]; /* Memory for I2C ROM Driver */

	Chip_Clock_EnablePeriphClock(I2C_HOSTIF_CLK);
	Chip_SYSCON_PeriphReset(I2C_HOSTIF_RST);
	I2C_HOSTIF->CLKDIV = I2C_HOSTIF_CLOCK_DIV;

	if (sizeof(devMem) < ROM_I2CS_GetMemSize()) {
		/* Should not happen, increase I2CS_MEM_SZ */
		while (1) ;
	}

	i2csInit.pUserData = 0;
	i2csInit.base = (uint32_t) I2C_HOSTIF;
	hI2C = ROM_I2CS_Init(devMem, &i2csInit);

	ROM_I2CS_RegisterCallback(hI2C, ROM_I2CS_XFERRECV_CB, (void *)i2cs_rx);
	ROM_I2CS_RegisterCallback(hI2C, ROM_I2CS_XFERSEND_CB, (void *)i2cs_tx);
	ROM_I2CS_RegisterCallback(hI2C, ROM_I2CS_DONE_CB, (void *)i2cs_done);
	slaveSetup.slaveAddr = I2C_HOSTIF_ADDR;
	slaveSetup.SlaveIndex = 0;
	slaveSetup.EnableSlave = 1;
	ROM_I2CS_SetupSlave(hI2C, &slaveSetup);
	/* Start first transfer */
	i2cXfer.rxBuff = g_i2cHostif.rxBuff;
	i2cXfer.rxSz = 0;
	ROM_I2CS_Transfer(hI2C, &i2cXfer);

	/* init host interrupt pin */
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, HOSTIF_IRQ_PORT, HOSTIF_IRQ_PIN);

	/* initlize host command engine and related buffers */
	Hostif_CmdEngineInit();

	/* Enable the interrupt for the I2C */
	NVIC_SetPriority(I2C_HOSTIF_IRQn, HOSTIF_IRQ_PRIORITY);
	NVIC_EnableIRQ(I2C_HOSTIF_IRQn);

	/* enable I2C hostif to wake-up the sensor hub */
	Chip_SYSCON_EnableWakeup(I2C_HOSTIF_WAKE);
}
#endif
