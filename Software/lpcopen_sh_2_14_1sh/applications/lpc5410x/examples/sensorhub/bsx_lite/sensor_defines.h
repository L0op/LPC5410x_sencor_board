/*
 * @brief File containing manifest constants for different sensors supported by the framework.
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

#ifndef __SENSOR_DEFINES_H_
#define __SENSOR_DEFINES_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup SH_RESMGR Sensor Hub: Resource manager interface
 * @ingroup SENSOR_HUB
 * @{
 */

/* Accelerometer defines */
#define ACCEL_BMA2X2 0x10
#define ACCEL_ICM20628 0x11 
#define ACCEL_HC_STUB 0x12    

    /* Magnetometer defines */
 #define MAG_BMM050  0x20
 #define MAG_AKM09912  0x21         /**<  */
 #define MAG_HC_STUB   0x22         /**<  */

    /* Gyroscope defines */
 #define GYRO_BMG160 0x30
 #define   GYRO_ICM20628 0x31          /**<  */
 #define   GYRO_HC_STUB 0x32           /**<  */
 #define   GYRO_BMI055 0x33            /**<  */
#define    GYRO_BMI160 0x34            /**<  */

    /* Barometer defines */
#define    BARO_BMP280  0x40
#define    BARO_HSPPAD038 0x41          /**<  */

    /* Proximity defines */
#define    PROXI_AMSCT1010 0x50
#define    PROXI_MAX44000 0x51          /**<  */

#define    AMBI_AMSCT1010 0x60
#define    AMBI_ROHMBH1721 0x61        /**<  */
#define    AMBI_MAX44000 0x62           /**<  */
    



/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_DEFINES_H_ */
