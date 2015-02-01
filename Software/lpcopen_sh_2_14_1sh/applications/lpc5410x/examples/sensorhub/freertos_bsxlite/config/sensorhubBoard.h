/*
 * @brief SensorHub Board specific
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
#include "board.h"
#include "sensor_defines.h"

#ifndef __SENSORHUBBOARD_H_
#define __SENSORHUBBOARD_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup SH_RESMGR Sensor Hub: Resource manager interface
 * @ingroup SENSOR_HUB
 * @{
 */

/* I2C Addresses for Sensors */
#define ADDR_BMC150A  0x10
#define ADDR_BMC150M  0x12
#define ADDR_BMM150   0x11
#define ADDR_BMP280   0x76
#define ADDR_MAX44000 0x4A
#define ADDR_BMI055   0x19
#define ADDR_BMI160   0x68
#define ADDR_BMI055G  0x69

/** Accelerometer interrupt pin defines */
#define ACCEL_INT_PORT                      0x0
#define ACCEL_INT_PIN                       18
#define ACCEL_PINT_SEL                      PININTSELECT0
#define ACCEL_PINT_CH                       PININTCH0
#define ACCEL_PINT_IRQn                     PIN_INT0_IRQn
#define ACCEL_WAKE                          SYSCON_STARTER_PINT0
#define ACCEL_IRQHandler                    PIN_INT0_IRQHandler
#define ACCEL_INT2_PORT                     0x0
#define ACCEL_INT2_PIN                      7

/** Magnetometer interrupt pin defines  */
#define MAG_INT_PORT						0x0
#define MAG_INT_PIN							22
#define MAG_PINT_SEL						PININTSELECT2
#define MAG_PINT_CH							PININTCH2
#define MAG_PINT_IRQn						PIN_INT2_IRQn
#define MAG_WAKE							SYSCON_STARTER_PINT2
#define MAG_IRQHandler                      PIN_INT2_IRQHandler
#define MAG_INT3_PORT                       0x0
#define MAG_INT3_PIN                        10

/** Gyroscope interrupt pin interface */
#define GYRO_INT_PORT						0x0
#define GYRO_INT_PIN						4
#define GYRO_PINT_SEL						PININTSELECT1
#define GYRO_PINT_CH						PININTCH1
#define GYRO_PINT_IRQn						PIN_INT1_IRQn
#define GYRO_WAKE							SYSCON_STARTER_PINT1
#define GYRO_IRQHandler						PIN_INT1_IRQHandler

/** Proximity interrupt pin interface */
#define PROXI_INT_PORT						0x0
#define PROXI_INT_PIN						9
#define PROXI_PINT_SEL						PININTSELECT3
#define PROXI_PINT_CH						PININTCH3
#define PROXI_PINT_IRQn						PIN_INT3_IRQn
#define PROXI_WAKE							SYSCON_STARTER_PINT3
#define PROXI_IRQHandler					PIN_INT3_IRQHandler
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __SENSORHUB_H_ */
