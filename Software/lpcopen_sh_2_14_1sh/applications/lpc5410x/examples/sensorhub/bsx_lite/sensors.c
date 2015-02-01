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
#include "sensorhubBoard.h"
#include "sensors.h"
#include "sensorhub.h"
#include "sensacq_i2c.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/
#if (SENSOR_ACCEL == ACCEL_BMA2X2)
#define G_ACCEL_CTRL    g_bma2x2Accel
#define P_ACCEL_CTRL    &g_bma2x2Accel
#elif (SENSOR_ACCEL == ACCEL_ICM20628)
#define G_ACCEL_CTRL    g_icmAccel
#define P_ACCEL_CTRL    &g_icmAccel
#else
#define P_ACCEL_CTRL    NULL
#endif

#if (SENSOR_GYRO == GYRO_BMG160)
#define G_GYRO_CTRL 	g_bmg160Gyro
#define P_GYRO_CTRL 	&g_bmg160Gyro
#elif (SENSOR_GYRO == GYRO_BMI160)
#define G_GYRO_CTRL 	g_bmi160Gyro
#define P_GYRO_CTRL 	&g_bmi160Gyro
#elif (SENSOR_GYRO == GYRO_ICM20628)
#define G_GYRO_CTRL 	g_icmGyro
#define P_GYRO_CTRL 	&g_icmGyro
#elif (SENSOR_GYRO == GYRO_BMI055)
#define G_GYRO_CTRL 	g_bmi055Gyro
#define P_GYRO_CTRL 	&g_bmi055Gyro
#else
#define P_GYRO_CTRL 	NULL
#endif

#if (SENSOR_MAG == MAG_BMM050)
#define G_MAG_CTRL  	g_bmm050Mag
#define P_MAG_CTRL  	&g_bmm050Mag
#elif (SENSOR_MAG == MAG_AKM09912)
#define G_MAG_CTRL  	g_akmMag
#define P_MAG_CTRL  	&g_akmMag
#else
#define P_MAG_CTRL  	NULL
#endif

#if (SENSOR_BARO == BARO_BMP280)
#define G_BARO_CTRL 	g_bmp280Pressure
#define P_BARO_CTRL 	&g_bmp280Pressure
#elif (SENSOR_BARO == BARO_HSPPAD038)
#define G_BARO_CTRL 	g_alpsPressure
#define P_BARO_CTRL 	&g_alpsPressure
#else
#define P_BARO_CTRL 	NULL
#endif

#if (SENSOR_PROXI == PROXI_AMSCT1010)
#define G_PROXI_CTRL 	g_amsct1010Proximity
#define P_PROXI_CTRL 	&g_amsct1010Proximity
#elif (SENSOR_PROXI == PROXI_MAX44000)
#define G_PROXI_CTRL 	g_max44000Proximity
#define P_PROXI_CTRL 	&g_max44000Proximity
#else
#define P_PROXI_CTRL 	NULL
#endif

#if (SENSOR_AMBI == AMBI_AMSCT1010)
#define G_AMBI_CTRL 	g_amsct1010Ambient
#define P_AMBI_CTRL 	&g_amsct1010Ambient
#elif (SENSOR_AMBI == AMBI_ROHMBH1721)
#define G_AMBI_CTRL 	g_rohmbh1721Ambient
#define P_AMBI_CTRL 	&g_rohmbh1721Ambient
#elif (SENSOR_AMBI == AMBI_MAX44000)
#define G_AMBI_CTRL 	g_max44000Ambient
#define P_AMBI_CTRL 	&g_max44000Ambient
#else
#define P_AMBI_CTRL 	NULL
#endif

extern PhysicalSensor_t G_ACCEL_CTRL;
extern PhysicalSensor_t G_GYRO_CTRL;
extern PhysicalSensor_t G_MAG_CTRL;
extern PhysicalSensor_t G_BARO_CTRL;
extern PhysicalSensor_t G_PROXI_CTRL;
extern PhysicalSensor_t G_AMBI_CTRL;

/* Physical sensors array */
PhysicalSensor_t *g_phySensors[PHYS_MAX_ID] = {
	P_ACCEL_CTRL,
	P_GYRO_CTRL,
	P_MAG_CTRL,
	P_BARO_CTRL,
	P_PROXI_CTRL,
	P_AMBI_CTRL,
};

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/
/* Initialize all physical sensors */
void PhysSensors_Init(void)
{
	PhysicalSensor_t *pSens;
	int32_t i;

	for (i = 0; i < PHYS_MAX_ID; i++) {
		pSens = g_phySensors[i];

		if ((pSens != NULL) && (pSens->hw->init)) {
			pSens->hw->init(pSens);
		}
	}
}

/* Enable/disable physical sensor */
void PhysSensors_Enable(PhysSensorId_t sensorId, uint32_t enable)
{
	PhysicalSensor_t *pSens;
	int32_t i;

	for (i = 0; i < PHYS_MAX_ID; i++) {
		pSens = g_phySensors[i];

		if ((pSens != NULL) && (pSens->id == sensorId)) {
			/* if current state is not same as requested then change the state */
			if (enable != pSens->enabled) {
				pSens->hw->activate(pSens, enable);
			}
			pSens->enabled = enable;

		}
	}
}

#if (SENSOR_ACCEL == ACCEL_ICM20628)
/* Handle interrupt from PININT */
void ACCEL_IRQHandler(void)
{
	uint32_t currTime = g_Timer.GetCurrent();
	PhysicalSensor_t *pSens = g_phySensors[PHYS_ACCEL_ID];
	if (pSens->enabled) {
		pSens->ts_nextSample = currTime + ((pSens->period + (pSens->ts_nextSample - pSens->ts_lastSample)) >> 1);
		pSens->ts_lastSample = currTime;

		pSens->irq_pending++;
	}
	pSens = g_phySensors[PHYS_GYRO_ID];
	if (pSens->enabled) {
		pSens->ts_nextSample = currTime + ((pSens->period + (pSens->ts_nextSample - pSens->ts_lastSample)) >> 1);
		pSens->ts_lastSample = currTime;

		pSens->irq_pending++;
	}

	Chip_PININT_ClearIntStatus(LPC_PININT, ACCEL_PINT_CH);
	ResMgr_IRQDone();
}

#else

/* Handle interrupt from PININT */
void ACCEL_IRQHandler(void)
{
	uint32_t currTime = g_Timer.GetCurrent();
	PhysicalSensor_t *pSens = g_phySensors[PHYS_ACCEL_ID];
	
	pSens->ts_nextSample = currTime + ((pSens->period + (pSens->ts_nextSample - pSens->ts_lastSample)) >> 1);
	pSens->ts_lastSample = currTime;
	pSens->irq_pending++;

	Chip_PININT_ClearIntStatus(LPC_PININT, ACCEL_PINT_CH);
	ResMgr_IRQDone();
}

/* Handle interrupt from PININT */
void GYRO_IRQHandler(void)
{
    PhysicalSensor_t *pSens = g_phySensors[PHYS_GYRO_ID];
	uint32_t currTime = g_Timer.GetCurrent();
	
    pSens->ts_nextSample = currTime + ((pSens->period + (pSens->ts_nextSample - pSens->ts_lastSample)) >> 1);
	pSens->ts_lastSample = currTime;
    pSens->irq_pending++;

	Chip_PININT_ClearIntStatus(LPC_PININT, GYRO_PINT_CH);
	ResMgr_IRQDone();
}

#endif

/* Handle interrupt for PINTINT */
void MAG_IRQHandler(void)
{
	uint32_t currTime = g_Timer.GetCurrent();
	PhysicalSensor_t *pSens = g_phySensors[PHYS_MAG_ID];
	
	pSens->ts_nextSample = currTime + ((pSens->period + (pSens->ts_nextSample - pSens->ts_lastSample)) >> 1);
	pSens->ts_lastSample = currTime;
	pSens->irq_pending++;
	
	Chip_PININT_ClearIntStatus(LPC_PININT, MAG_PINT_CH);
	ResMgr_IRQDone();
}

/* Handle interrupt for PINTINT */
void PROXI_IRQHandler(void)
{
	uint32_t currTime = g_Timer.GetCurrent();
	PhysicalSensor_t *pSens = g_phySensors[PHYS_PROX_ID];
	if (pSens->enabled) {
		/* Note that we only update the last sample time as we are not sure if the INT is for proximity or Ambient */
		pSens->ts_lastSample = currTime;
		pSens->irq_pending++;
	}
#if(SENSOR_AMBI==AMBI_AMSCT1010)
	pSens = g_phySensors[PHYS_AMBIENT_ID];
	if (pSens->enabled) {
		/* Note that we only update the last sample time as we are not sure if the INT is for proximity or Ambient */
		pSens->ts_lastSample = currTime;
		pSens->irq_pending++;
	}
#endif
	Chip_PININT_ClearIntStatus(LPC_PININT, PROXI_PINT_CH);
	ResMgr_IRQDone();
}

/* Process sensor tasks */
uint32_t Sensor_process(void)
{
	PhysicalSensor_t *pSens;
	int32_t i, read = 0;
	uint32_t tsDiff;

	for (i = 0; i < PHYS_MAX_ID; i++) {
		pSens = g_phySensors[i];
		read = 0;
		if ((pSens != NULL) && (pSens->enabled)) {
			if (pSens->mode == PHYS_MODE_POLL) {
				/* check if it is time read POLL sensor */
				tsDiff = g_Timer.GetCurrent() - pSens->ts_lastSample;
				if (tsDiff >= pSens->period) {
					read = 1;
				}

			}
			else if (pSens->irq_pending) {
				read = 1;
				__disable_irq();
				pSens->irq_pending--;	/* Decrement atomic */
				__enable_irq();
			}
			if (read) {
				if (pSens->hw->read(pSens) == 0) {
					Algorithm_Process(pSens);
				}
			}
		}
	}

	return 0;
}
