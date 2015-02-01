/*
 * @brief Accelerometer interface
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

#ifndef __SENSORS_H_
#define __SENSORS_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup SH_SENSORS Sensor Hub: Sensor API
 * @ingroup SENSOR_HUB
 * @{
 */

/**
 * @brief Physical sensor ID's
 */
typedef enum _PhysSensorId {
	PHYS_ACCEL_ID = 0,          /**< Accelerometer sensor ID */
	PHYS_GYRO_ID,               /**< Gyro sensor ID */
	PHYS_MAG_ID,                /**< Magnetometer sensor ID */
	PHYS_PRESSURE_ID,           /**< Pressure sensor ID */
	PHYS_PROX_ID,               /**< Proximity sensor ID */
	PHYS_AMBIENT_ID,            /**< Ambient light sensor ID */
    /** @cond INTERNAL */
	PHYS_MAX_ID                 /* internal use to count number of sensors */
	/** @endcond */ 
} PhysSensorId_t;

/**
 * @brief Sensor data acquisition modes 
 */
typedef enum _PhysSensorMode {
	PHYS_MODE_IRQ = 0,          /**< Data read triggered by IRQ */
	PHYS_MODE_POLL,             /**< Data read triggered by timed interval */
} PhysSensorMode_t;

/* Forward type declaration */
struct PhysSensorCtrl;

/**
 * @brief   Public Sensor data / control interface
 */
typedef struct _PhysSensor_CTRL {
	const struct PhysSensorCtrl *hw;	/**< Sensor control interface structure */

	int16_t data16[4];			        /**< Sample data */
	uint32_t ts_lastSample;		        /**< Timestamp when last sample is captured */
	uint32_t ts_nextSample;		        /**< Next sample timestamp in Kernel timer ticks */

	uint8_t id;					        /**< PhysSensorId_t type */
	uint8_t enabled;			        /**< flag indicating if the sensor is enabled */
	uint8_t irq_pending;		        /**< irq is pending */
	uint8_t mode;				        /**< Sensor mode, 0 - IRQ, 1 - polling */
	
	uint32_t period;			        /**< sample period */
	void *libHandle;			        /**< Fusion library's handle for the physical sensor */

} PhysicalSensor_t;

/* The following block of functions is merely for documentation.  */
#ifdef __DOXYGEN__
/**
 * @brief	Initialize sensor
 * @param	pSens : Pointer to sensor control structure
 * @return	Init status (0 on success)
 * @note    Interface detailing (* int)() api in PhysSensorCtrl structure.  Do NOT implement directly!
 */
int32_t PhysSensorCtrl_init(PhysicalSensor_t * pSens); 

/**
 * @brief	Read sensor data and store in pSens->data
 * @param	pSens : Pointer to sensor control structure
 * @return	Read status (0 on success)
 * @note    Interface detailing (* read)() api in PhysSensorCtrl structure.  Do NOT implement directly!
 */
int32_t PhysSensorCtrl_read(PhysicalSensor_t * pSens);  

/**
 * @brief	Activate / de-activate sensor
 * @param	pSens : Pointer to sensor control structure
 * @param   enable: 1 to enable, 0 to disable
 * @return	Activation status (0 on success)
 * @note    Interface detailing (* activate)() api in PhysSensorCtrl structure.  Do NOT implement directly!
 */
int32_t PhysSensorCtrl_activate(PhysicalSensor_t * pSens, bool enable); 

/**
 * @brief	Set data acquisition rate
 * @param	pSens : Pointer to sensor control structure
 * @param   msec  : Data acquisition rate in mSec
 * @return	Status (0 on success)
 * @note    Interface detailing (* setDelay)() api in PhysSensorCtrl structure.  Do NOT implement directly!
 */
int32_t PhysSensorCtrl_setDelay(PhysicalSensor_t * pSens, uint32_t msec);    
#endif

/**
 * @brief   Sensor interface
 */
typedef struct PhysSensorCtrl {
	int32_t(*const init) (PhysicalSensor_t * pSens);                        /**< \see PhysSensorCtrl_init() */
	int32_t(*const read) (PhysicalSensor_t * pSens);                        /**< \see PhysSensorCtrl_read() */
	int32_t(*const activate) (PhysicalSensor_t * pSens, bool enable);       /**< \see PhysSensorCtrl_activate() */
	int32_t(*const setDelay) (PhysicalSensor_t * pSens, uint32_t msec);     /**< \see PhysSensorCtrl_setDelay() */

} PhysSensorCtrl_t;

extern PhysicalSensor_t *g_phySensors[PHYS_MAX_ID];     /**< Global array of sensors */

/**
 * @brief	Initialize all physical sensors
 * @return	none
 */
void PhysSensors_Init(void);

/**
 * @brief	Enable/disable physical sensor
 * @param	sensorId	: Physical sensor id
 * @param	enable		: If 0 disable else enable
 * @return	none
 */
void PhysSensors_Enable(PhysSensorId_t sensorId, uint32_t enable);

/**
 * @brief	Process sensor tasks
 * @return	Return 0 when it is ok to sleep.
 */
uint32_t Sensor_process(void);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __SENSORS_H_ */
