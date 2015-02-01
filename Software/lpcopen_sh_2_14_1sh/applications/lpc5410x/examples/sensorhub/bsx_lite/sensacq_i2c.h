/*
 * @brief Adaptation layer for Bosch Drivers
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

#ifndef __SENSACQ_I2C_H
#define __SENSACQ_I2C_H

/** @defgroup SH_SENSOR_SUPPORT Sensor Hub: Sensor Library Support
 * @ingroup SENSOR_HUB
 * @{
 */
/**
 * @brief	Initialize I2C bus connected sensors
 * @return	True if initalized
 */
bool dev_i2c_init(void);

/**
 * @brief	Bosch API for delay
 * @param   msec :  Delay for specified number of miliseconds
 * @return	Nothing
 */
void dev_i2c_delay(unsigned int msec);

/**
 * @brief	Bosch API to write data to specific register at address
 * @param   dev_addr    : I2C device address
 * @param   reg_addr    : Register in device to write to
 * @param   reg_data    : Pointer to data to write
 * @param   cnt         : Number of bytes to write
 * @return	Status : LPC_OK on success.
 */
signed char dev_i2c_write(unsigned char dev_addr, unsigned char reg_addr, unsigned char *reg_data, unsigned char cnt);

/**
 * @brief	Bosch API to read data from specific register at address
 * @param   dev_addr    : I2C device address
 * @param   reg_addr    : Register in device to write to
 * @param   reg_data    : Pointer to buffer where data will be stored
 * @param   cnt         : Number of bytes to read
 * @return	Status : LPC_OK on success.
 */
signed char dev_i2c_read(unsigned char dev_addr, unsigned char reg_addr, unsigned char *reg_data, unsigned char cnt);

/**
 * @brief	API to read data at address (assumes auto address increment)
 * @param   dev_addr    : I2C device address
 * @param   reg_data    : Pointer to buffer where data will be stored
 * @param   cnt         : Number of bytes to read
 * @return	Status : LPC_OK on success.
 */
char dev_i2c_readOnly(unsigned char dev_addr, unsigned char *reg_data, unsigned char cnt);

/**
 * @}
 */
#endif /* __SENSACQ_I2C_H */
