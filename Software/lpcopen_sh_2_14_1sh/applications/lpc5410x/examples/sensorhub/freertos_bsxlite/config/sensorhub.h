/*
 * @brief Resource manager interface
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

#ifndef __SENSORHUB_H_
#define __SENSORHUB_H_

#include <stdint.h>
#include "board.h"
#include "sensor_defines.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup SH_CONFIG Sensor Hub: Application configuration
 * @ingroup SENSOR_HUB
 * @{
 */
#define CFG_MAJOR_VERSION					0x01        /**< Major version number */
#define CFG_MINOR_VERSION					0x23        /**< Minor version number */

#define SENSOR_ACCEL                        ACCEL_BMA2X2        /**< Accelerometer sensor definition */
#define SENSOR_GYRO                         GYRO_BMI055         /**< Gyro sensor definition */
#define SENSOR_MAG                          MAG_BMM050          /**< Magnetometer sensor definition */
#define SENSOR_BARO                         BARO_BMP280         /**< Barometer sensor definition */
#define SENSOR_PROXI                        PROXI_MAX44000      /**< Proximity sensor definition */
#define SENSOR_AMBI                         AMBI_MAX44000       /**< Ambient sensor definition */

/* Host interface type. Enable one or the other */
//#define HOSTIF_SPI                /**< Build define to enable SPI host interface */
#define HOSTIF_I2C                  /**< Build define to enable I2C host interface */
#define I2CM_IRQ_BASED              /**< Build define to enable I2C interrupt support */

/* Host interface IRQ defines */
#define HOSTIF_IRQ_PORT                     0                   /**< Declare port the HOSTIF_IRQ is on */
#define HOSTIF_IRQ_PIN                      19                  /**< Declare the pin the HOSTIF_IRQ is on */

/** Host interface (I2C slave) defines */
#ifdef HOSTIF_I2C
#define I2C_HOSTIF                          LPC_I2C2            /**< Which I2C interface to use for HOST_IF */
#define I2C_HOSTIF_IRQn                     I2C2_IRQn           /**< Which IRQ number to use for HOST_IF */
#define I2C_HOSTIF_IRQHandler               I2C2_IRQHandler     /**< Which IRQ handler to use for HOST_IF */
#define I2C_HOSTIF_WAKE                     SYSCON_STARTER_I2C2
#define I2C_HOSTIF_CLOCK_DIV                2	                /* recommended by Noah */
#define I2C_HOSTIF_ADDR                     (0x18)
#define I2C_HOSTIF_CLK                      SYSCON_CLOCK_I2C2   /**< Clock for I2C on HOST_IF */
#define I2C_HOSTIF_RST                      RESET_I2C2          /**< Reset for I2C on HOST_IF */
#endif /* HOSTIF_I2C */

/** Host interface (SPI slave) defines */
#ifdef HOSTIF_SPI
#define SPI_HOSTIF                          LPC_SPI0            /**< Which SPI interface to use for HOST_IF */
#define SPI_HOSTIF_IRQn                     SPI0_IRQn           /**< Which IRQ number to use for HOST_IF */
#define SPI_HOSTIF_IRQHandler               SPI0_IRQHandler     /**< Which IRQ handler to use for HOST_IF */
#define SPI_HOSTIF_WAKE                     SYSCON_STARTER_SPI0
/* 1MHz SPI bit-rate */
#define SPI_HOSTIF_BITRATE                  (1000000)           /**< bit rate for SPI communication */
#define SPI_HOSTIF_TX_DMACH                 ROM_DMAREQ_SPI0_TX  /**< DMA TX channel for SPI on HOST_IF */
#define SPI_HOSTIF_RX_DMACH                 ROM_DMAREQ_SPI0_RX  /**< DMA RX channel for SPI on HOST_IF */
#define SPI_HOSTIF_CLOCK					SYSCON_CLOCK_SPI0   /**< Clock for SPI on HOST_IF */
#define SPI_HOSTIF_RESET					RESET_SPI0          /**< Reset for SPI on HOST_IF */
#endif  /* HOSTIF_SPI */

/** Sensor bus interface defines */
#define I2C_SENSOR_BUS                      LPC_I2C0                /**< Which I2C interface used for SENSOR_IF */
#define I2C_SENSOR_CLOCK                    SYSCON_CLOCK_I2C0       /**< Clock for I2C used for SENSOR_IF */
#define I2C_SENSOR_RESET                    RESET_I2C0              /**< Reset for I2C on SENSOR_IF */
#define I2C_SENSOR_BUS_IRQn                 I2C0_IRQn               /**< Which IRQ number to use for SENSOR_IF */
#define I2C_SENSOR_BUS_IRQHandler           I2C0_IRQHandler         /**< IRQ handler used for SENSOR_IF */
#define I2C_SENSOR_MCLOCK_SPEED             400000	                /**< 400000 generates 375K, use 430000 for 400K */
#define CFG_MAX_SA_TX_BUFFER				32	                    /**< Maximum size of the buffer transmitted to sensor devices over I2c */

/** Sensor Hub framework IRQ priorities */
#define SENSOR_IRQ_PRIORITY                 1                       /**< Sensor IRQ (pin int) priority */
#define SENSOR_I2C_PRIORITY                 2                       /**< I2C sensor priority */
#define HOSTIF_IRQ_PRIORITY                 6                       /**< I2C HOST_IF priority */
/* DMA should be lower priority than SPI */
#define DMA_IRQ_PRIORITY                    3                       /**< DMA IRQ priority */
#define HOSTIF_SPI_IRQ_PRIORITY             1                       /**< SPI HOST_IF priority */

#define PWRDOWN_THRESHOLD_US                (400) /**< Inter activity gap threshold in usecs to enter power down mode */
#define SLEEP_THRESHOLD_US                  (8)   /**< Inter activity gap threshold in usecs to enter sleep mode */

#define MAX_SENSOR_EVENT                    16    /**< Number of sensor event for which the queue to be created */

/**
 * @}
 */

#include "kernel_timer.h"
#include "kernel_res_mgr.h"
#include "sensors.h"
#include "algorithm.h"
#include "hostif.h"

#ifdef __cplusplus
}
#endif

#endif /* __SENSORHUB_H_ */
