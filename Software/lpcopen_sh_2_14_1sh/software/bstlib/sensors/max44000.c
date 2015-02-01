/*
 * @brief Ambient and infrared proximity sensor Max44000
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2013
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licenser disclaim any and
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

#include "max44000.h"
#include "sensacq_i2c.h"
#include "lpc_types.h"
#include "kernel_timer.h"

MAX44000_T g_Max44k = 
{
	/*.devAddrRd = */ MAX44K_RD_ADDR,
	/*.devAddrWr = */ MAX44K_WR_ADDR,
	/*.opMode =	*/	MODE_SHUTDOWN,
	/*.alsBitResolution =*/ BIT_RES_10,
	/*.alsMeasureGain =*/ MEAS_GAIN_4,
	/*.ledDrv =*/		PROX_LED_DRV,
	dev_i2c_read,
	dev_i2c_write,
};


static void SetOpMode(MAX44000_T* max44k,OpMode_T mode)
{
	unsigned char devReadAddr = max44k->devAddrRd;
	unsigned char devWrAddr = max44k->devAddrWr;
	unsigned char regAddr = MAX44K_MAIN_CFG_REG;
	unsigned char regData;
	
	max44k->opMode = mode;
	max44k->bus_read(devReadAddr,regAddr,&regData,1);
	regData &= ~(0x7 << MAX44K_MAIN_CFG_REG_MODE);
	regData |= (mode << MAX44K_MAIN_CFG_REG_MODE);
	max44k->bus_write(devWrAddr,regAddr,&regData,1);
	
}

static void SetALSBitRes(MAX44000_T* max44k,ALSBitRes_T bitRes)
{
	unsigned char devReadAddr = max44k->devAddrRd;
	unsigned char devWrAddr = max44k->devAddrWr;
	unsigned char regAddr = MAX44K_RCV_CFG_REG;
	unsigned char regData;

	
	max44k->alsBitResolution = bitRes;
	max44k->bus_read(devReadAddr,regAddr,&regData,1);
	regData &=~(0x3 << MAX44K_RCV_CFG_REG_ALSTIM);
	regData |= (bitRes << MAX44K_RCV_CFG_REG_ALSTIM);
	max44k->bus_write(devWrAddr,regAddr,&regData,1);
	
}

static void SetALSMeasGain(MAX44000_T* max44k,ALSMeasGain_T measGain)
{
	unsigned char devReadAddr = max44k->devAddrRd;
	unsigned char devWrAddr = max44k->devAddrWr;
	unsigned char regAddr = MAX44K_RCV_CFG_REG;
	unsigned char regData;

	
	max44k->alsMeasureGain= measGain;
	max44k->bus_read(devReadAddr,regAddr,&regData,1);
	regData &=~(0x3 << MAX44K_RCV_CFG_REG_ALSPGA);
	regData |= (measGain << MAX44K_RCV_CFG_REG_ALSPGA);
	max44k->bus_write(devWrAddr,regAddr,&regData,1);
}

static void SetLEDDrive(MAX44000_T* max44k, LED_DRV_T ledDrv)
{
	unsigned char devWrAddr = max44k->devAddrWr;
	unsigned char regAddr = MAX44K_TRSMT_CFG_REG;
	unsigned char regData;

	max44k->ledDrv = ledDrv;
	regData = ledDrv & 0x0F;
	max44k->bus_write(devWrAddr,regAddr,&regData,1);
}

static void Max44000_ProxInit(MAX44000_T* max44k)
{
	SetOpMode(max44k, MODE_SHUTDOWN);
	SetLEDDrive(max44k,max44k->ledDrv);
}

static void Max44000_AmbInit(MAX44000_T* max44k)
{
	SetOpMode(max44k, MODE_SHUTDOWN);
	SetALSBitRes(max44k, BIT_RES_10);
	SetALSMeasGain(max44k, MEAS_GAIN_16);
}

static void Max44000_ProxActivate(MAX44000_T* max44k, bool enable)
{
	if(enable)
	{
		if(max44k->opMode == MODE_SHUTDOWN || max44k->opMode == MODE_PROX)		//Ambient sensor is shut down
		{
			max44k->opMode = MODE_PROX;
			SetOpMode(max44k, MODE_PROX);
		}
		else
		{
			max44k->opMode = MODE_INTERLEAVE;
			SetOpMode(max44k, MODE_INTERLEAVE);
		}
	}
	else
	{
		if(max44k->opMode == MODE_SHUTDOWN || max44k->opMode == MODE_PROX)
		{
			max44k->opMode = MODE_SHUTDOWN;
			SetOpMode(max44k, MODE_SHUTDOWN);
		}
		else
		{
			max44k->opMode = MODE_ALS_GIR;
			SetOpMode(max44k, MODE_ALS_GIR);
			
		}
	}
}
static void Max44000_AmbActivate(MAX44000_T* max44k, bool enable)
{
	if (enable)
	{
		if(max44k->opMode == MODE_PROX || max44k->opMode == MODE_INTERLEAVE) 		//Proximity sensor is working
		{
			max44k->opMode = MODE_INTERLEAVE;
			SetOpMode(max44k,MODE_INTERLEAVE);
		}
		else
		{
			max44k->opMode = MODE_ALS_GIR;
			SetOpMode(max44k,MODE_ALS_GIR);
		}
	}
	else
	{
		if(max44k->opMode == MODE_PROX || max44k->opMode == MODE_INTERLEAVE) 		//Proximity sensor is working
		{
			max44k->opMode = MODE_PROX;
			SetOpMode(max44k,MODE_PROX);
		}
		else
		{
			max44k->opMode = MODE_SHUTDOWN;
			SetOpMode(max44k,MODE_SHUTDOWN);
		}
	}
}

static char Max44000_ReadProx(MAX44000_T* max44k,Prox_Data_t* proxData)
{
	uint8_t adcCnt;
	uint8_t devReadAddr = max44k->devAddrRd;
	uint8_t regAddr = MAX44K_PROX_DATA_Byte_REG;
	
	if(max44k->opMode != MODE_INTERLEAVE && max44k->opMode != MODE_PROX)
	{
		return ERROR;
	}
	max44k->bus_read(devReadAddr,regAddr,&adcCnt,1);
	switch (max44k->ledDrv)
	{
		case LED_DRV_DISABLE:
			return ERROR;
			
		case LED_DRV_20:
			if(adcCnt == 0)
			{
				proxData->dist= 100;
			}
			else if(adcCnt > 0 && adcCnt <= 20)
			{
				proxData->dist= 30;
			}
			else if(adcCnt > 20 && adcCnt <= 50)
			{
				proxData->dist= 25;
			}
			else if(adcCnt > 50 && adcCnt <= 75)
			{
				proxData->dist= 23;
			}
			else if(adcCnt > 75 && adcCnt <= 100)
			{
				proxData->dist= 20;
			}
			else if(adcCnt > 100 && adcCnt <= 150)
			{
				proxData->dist= 15;
			}
			else if (adcCnt > 150 && adcCnt <= 200)
			{
				proxData->dist= 13;
			}
			else if (adcCnt > 200 && adcCnt <= 250)
			{
				proxData->dist= 10;
			}
			else if(adcCnt > 250)
			{
				proxData->dist= 5;
			}
			break;
			
		case LED_DRV_50:
			if(adcCnt == 0)
			{
				proxData->dist= 100;
			}
			else if(adcCnt > 0 && adcCnt <= 15)
			{
				proxData->dist= 65;
			}
			else if(adcCnt > 15 && adcCnt <= 25)
			{
				proxData->dist= 55;
			}
			else if (adcCnt > 25 && adcCnt <= 50)
			{
				proxData->dist= 45;
			}
			else if(adcCnt > 50 && adcCnt <= 70)
			{
				proxData->dist= 37;
			}
			else if(adcCnt > 70 && adcCnt <= 100)
			{
				proxData->dist= 32;
			}
			else if(adcCnt > 100 && adcCnt <= 150)
			{
				proxData->dist= 25;
			}
			else if(adcCnt > 150 && adcCnt <= 200)
			{
				proxData->dist= 22;
			}
			else if(adcCnt > 200 && adcCnt <= 235)
			{
				proxData->dist= 21;
			}
			else if(adcCnt > 235 && adcCnt <= 250)
			{
				proxData->dist= 17;
			}
			else if(adcCnt > 250)
			{
				proxData->dist= 10;
			}
			break;
			
		case LED_DRV_110:
			if(adcCnt == 0)
			{
				proxData->dist= 140;
			}
			else if(adcCnt > 0 && adcCnt <= 23)
			{
				proxData->dist= 135;
			}
			else if(adcCnt > 23 && adcCnt <= 28)
			{
				proxData->dist= 125;
			}
			else if (adcCnt > 28 && adcCnt <= 32)
			{
				proxData->dist= 115;
			}
			else if(adcCnt > 32 && adcCnt <= 36)
			{
				proxData->dist= 105;
			}
			else if(adcCnt > 36 && adcCnt <= 40)
			{
				proxData->dist= 95;
			}
			else if(adcCnt > 40 && adcCnt <= 45)
			{
				proxData->dist= 85;
			}
			else if(adcCnt > 45 && adcCnt <= 50)
			{
				proxData->dist= 75;
			}
			else if(adcCnt > 50 && adcCnt <= 63)
			{
				proxData->dist= 65;
			}
			else if(adcCnt > 63 && adcCnt <= 88)
			{
				proxData->dist= 55;
			}
			else if(adcCnt > 88 && adcCnt <= 125)
			{
				proxData->dist= 45;
			}
			else if(adcCnt > 125 && adcCnt <= 170)
			{
				proxData->dist= 35;
			}
			else if(adcCnt > 170 && adcCnt <= 245)
			{
				proxData->dist= 30;
			}
			else if(adcCnt > 245 && adcCnt <= 250)
			{
				proxData->dist= 25;
			}
			else if(adcCnt >250)
			{
				proxData->dist= 20;
			}
			break;
			
		default:
			return ERROR;
	}
	

	return 1;
	
}

static char Max44000_Read_ALS(MAX44000_T* max44k,Amb_Data_T* ambData)
{
	uint8_t dataHigh;
	uint8_t devReadAddr = max44k->devAddrRd;
	uint8_t regAddrH = MAX44K_ALS_DATA_HIGH_BYTE_REG;
	uint8_t regAddrL = MAX44K_ALS_DATA_LOW_BYTE_REG;
	uint8_t bitRes;

	if(max44k->opMode == MODE_SHUTDOWN || max44k->opMode == MODE_PROX)
	{
		return ERROR;
	}

	switch (max44k->alsBitResolution)
	{
		case 0:
			bitRes = 14;
			break;
		case 1:
			bitRes = 12;
			break;
		case 2:
			bitRes = 10;
			break;
		case 3:
			bitRes = 8;
			break;
		default:
			bitRes = 8;
	}
	
	
	max44k->bus_read(devReadAddr,regAddrH,&dataHigh,1);
	ambData->highByte = dataHigh & ((1<<(bitRes-8))-1);
	max44k->bus_read(devReadAddr,regAddrL,&(ambData->lowByte),1);

	return 1;
}


static int32_t Prox_Init(PhysicalSensor_t* pSens)
{
	Max44000_ProxInit(&g_Max44k);
	return 0;
}
static int32_t Prox_Read(PhysicalSensor_t* pSens)
{
	Prox_Data_t proxData;
	uint32_t currTime = g_Timer.GetCurrent();

	/* update sample timestamps */
	pSens->ts_nextSample = currTime + pSens->period;
	pSens->ts_lastSample = currTime;
	
	if (Max44000_ReadProx(&g_Max44k, &proxData))
	{
		pSens->data16[0] = proxData.dist;
		return 0;
	}
	return -1;
}
static int32_t Prox_Activate(PhysicalSensor_t* pSens, bool enable)
{
	Max44000_ProxActivate(&g_Max44k,enable);
	return 0;
}
static int32_t Prox_SetDelay(PhysicalSensor_t* pSens, uint32_t mSec)
{
	pSens->period = g_Timer.getTicksFromMs(mSec);
	return 0;
}

static int32_t Ambient_Init(PhysicalSensor_t* pSens)
{
	Max44000_AmbInit(&g_Max44k);
	return 0;
}
static int32_t Ambient_Read(PhysicalSensor_t* pSens)
{
	Amb_Data_T ambData;
	uint32_t currTime = g_Timer.GetCurrent();

	/* update sample timestamps */
	pSens->ts_nextSample = currTime + pSens->period;
	pSens->ts_lastSample = currTime;
	
	if (Max44000_Read_ALS(&g_Max44k, &ambData))
	{
		pSens->data16[0] = (ambData.highByte << 8) | ambData.lowByte;
		return 0;
	}

	return 1;
}
static int32_t Ambient_Activate(PhysicalSensor_t* pSens, bool enable)
{
	Max44000_AmbActivate(&g_Max44k,enable);
	return 0;
}
static int32_t Ambient_SetDelay(PhysicalSensor_t* pSens, uint32_t mSec)
{
	pSens->period = g_Timer.getTicksFromMs(mSec);
	return 0;
}

const PhysSensorCtrl_t g_proxCtrl = 
{
	Prox_Init,
	Prox_Read,
	Prox_Activate,
	Prox_SetDelay,
};

PhysicalSensor_t g_max44000Proximity = 
{
	/* .hw */ &g_proxCtrl,
		/* .data16*/ {0, },
		/* .ts_lastSample */ 0,
		/* .ts_nextSample */ 0,
		/* .id */ PHYS_PROX_ID,
		/* .enabled */ 0,
		/* .irq_pending */ 0,
		/* .mode */ PHYS_MODE_POLL,
		/* .period */ 300,
	
};

/**
 * @brief   Sensor interface
 */
const PhysSensorCtrl_t g_ambCtrl = 
{
	Ambient_Init,       
	Ambient_Read,       
	Ambient_Activate,   
	Ambient_SetDelay,   /* Calculates and sets data rate */
};

/* Public sensor data / control */
PhysicalSensor_t g_max44000Ambient = {
	&g_ambCtrl,         /* Sensor hardware structure */
	{0, },              /* sensor data sample */
	0,                  /* time (watchdog timer ticks) the last sample was acquired */
	0,                  /* time (watchdog timer ticks) the next sample will be taken */
	PHYS_AMBIENT_ID,    /* sensor ID */
	0,                  /* sensor enabled status*/
	0,                  /* IRQ pending */
	PHYS_MODE_POLL,     /* data acquisition mode (irq, polled) */
	400,                /* data rate in mSec */
};
