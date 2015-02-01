/*
 * @brief Algorithm interface
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

#include <stdint.h>
#include "hostif_protocol.h"

#ifndef __ALGO_H_
#define __ALGO_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup ALGO_INTERFACE Sensor Hub: Algorithm interface
 * @ingroup SENSOR_HUB
 * @{
 */

/**
 * @brief	Initialize algorithm module
 * @return	none
 */
void Algorithm_Init(void);

/**
 * @brief	Enable/disable given virtual sensor
 * @param	hif_SensorId	: virtual sensor ID defined in host interface under enum LPCSH_SENSOR_ID
 * @param	enable			: 0 disable, 1 enable
 * @return	none
 */
void Algorithm_EnableSensor(enum LPCSH_SENSOR_ID hif_SensorId, uint8_t enable);

/**
 * @brief	Update sensor data for fusion library, execute fusion algorithm, read algorithm output data
 * @param	pSens	: Pointer to sensor which received data
 * @return	Ok to sleep indicator
 */
uint32_t Algorithm_Process(PhysicalSensor_t *pSens);

/**
 * @brief	Run background process if needed by fusion library for example calibration routines
 * @param	pEstSleepTime	: Updates estimated sleep time with the time remaining
 * @return	returns 0/false if background process is not complete else returns 1/true
 */
uint32_t Algorithm_bgProcess(int32_t *pEstSleepTime);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __ALGO_H_ */
