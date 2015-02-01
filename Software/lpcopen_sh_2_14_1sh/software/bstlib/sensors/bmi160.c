/*
****************************************************************************
* Copyright (C) 2014 Bosch Sensortec GmbH
*
* bmi160.c
* Date: 2014/10/17
* Revision: 2.0.3 $
*
* Usage: Sensor Driver for BMI160 sensor
*
****************************************************************************
* License:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
**************************************************************************/
/*! file <BMI160 >
    brief <Sensor driver for BMI160> */

/* user defined code to be added here ... */
#include "board.h"
#include "sensorhubBoard.h"
#include "sensors.h"
#include "sensorhub.h"
#include "sensacq_i2c.h"

#include "bmi160.h"

static struct bmi160_t *p_bmi160;
/* used for reading the mag trim values for compensation*/
static struct trim_data_t mag_trim;
/* used for reading the AKM compensating data */
static struct bst_akm_sensitivity_data_t akm_asa_data;
/* Assign the fifo time */
u32 V_fifo_time_U32 = C_BMI160_ZERO_U8X;
/* Used to store as accel fifo data */
struct bmi160_accel_t accel_fifo[FIFO_FRAME_CNT];
/* Used to store as gyro fifo data */
struct bmi160_gyro_t gyro_fifo[FIFO_FRAME_CNT];
/* Used to store as mag fifo data */
struct bmi160_mag_t mag_fifo[FIFO_FRAME_CNT];
/* FIFO data read for 1024 bytes of data */
u8 v_fifo_data_u8[1024] = {0};
/*************************************************************************
 * Description: *//**brief
 *        This function initialises the structure pointer and assigns the
 * I2C address.
 *
 *
 *
 *
 *
 *  \param  bmi160_t *bmi160 structure pointer.
 *
 *
 *
 *  \return communication results.
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_init(struct bmi160_t *bmi160)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	/* assign bmi160 ptr */
	p_bmi160 = bmi160;
	com_rslt =
	p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
	BMI160_USER_CHIP_ID__REG,
	&v_data_u8, 1);     /* read Chip Id */
	p_bmi160->chip_id = v_data_u8;
	return com_rslt;
}
/****************************************************************************
 * Description: *//**brief This API gives data to the given register and
 *                          the data is written in the corresponding register
 *  address
 *
 *
 *
 *  \param u8 v_addr_u8, u8 v_data_u8, u8 v_len_u8
 *          v_addr_u8 -> Address of the register
 *          v_data_u8 -> Data to be written to the register
 *          v_len_u8  -> Length of the Data
 *
 *
 *
 *  \return communication results.
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_write_reg(u8 v_addr_u8,
u8 *v_data_u8, u8 v_len_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->dev_addr,
			v_addr_u8, v_data_u8, v_len_u8);
		}
	return com_rslt;
}
/*****************************************************************************
 * Description: *//**brief This API reads the data from the given register
 * address
 *
 *
 *
 *
 *  \param u8 v_addr_u8, u8 *v_data_u8, u8 v_len_u8
 *         v_addr_u8 -> Address of the register
 *         v_data_u8 -> address of the variable, read value will be kept
 *         v_len_u8  -> Length of the data
 *
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_read_reg(u8 v_addr_u8,
u8 *v_data_u8, u8 v_len_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			v_addr_u8, v_data_u8, v_len_u8);
		}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API used to reads the fatal error
 *	from the Register 0x02 bit 0
 *
 *
 *  \param u8 * v_fatal_err_u8 : Pointer to the value of fatal error
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_fatal_err(u8
*v_fatal_err_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_FATAL_ERR__REG,
			&v_data_u8, 1);
			*v_fatal_err_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_FATAL_ERR);
		}
	return com_rslt;
}
/*************************************************************************
 *	Description: *//**brief This API used to read the error code
 *	Register 0x02 bit 1 to 4
 *
 *
 *  \param u8 * v_err_code_u8 : Pointer to the value of error_code
 *	error_code
 *	0x00	no error
 *	0x01	ACC_CONF error (Accelerometer ODR and BWP not compatible)
 *	0x02	GYR_CONF error (Gyroscope ODR and BWP not compatible)
 *	0x03	Under sampling mode and interrupt uses pre filtered data
 *	0x04	reserved
 *	0x05	Selected trigger-readout offset
 *			in MAG_IF greater than selected ODR
 *	0x06	FIFO configuration error for headerless mode
 *	0x07	Under sampling mode and pre filtered data as FIFO source
 *
 *
 *  \return results of communication routine
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_err_code(u8
*v_err_code_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_ERR_CODE__REG,
			&v_data_u8, 1);
			*v_err_code_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_ERR_CODE);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 *	Description: *//**brief This API Reads the i2c error code from the
 *	Register 0x02 bit 5
 *
 *  \param u8 * v_i2c_err_code_u8 : Pointer to the error in I2C-Master detected
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_i2c_fail_err(u8
*v_i2c_err_code_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_I2C_FAIL_ERR__REG,
			&v_data_u8, 1);
			*v_i2c_err_code_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_I2C_FAIL_ERR);
		}
	return com_rslt;
}
 /****************************************************************************
 *	Description: *//**brief This API Reads the dropped command error
 *	from the register 0x02 bit 6
 *
 *
 *  \param u8 * v_drop_cmd_err_u8 : Pointer holding the
 *				dropped command to register CMD error
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_drop_cmd_err(u8
*v_drop_cmd_err_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_DROP_CMD_ERR__REG,
			&v_data_u8, 1);
			*v_drop_cmd_err_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_DROP_CMD_ERR);
		}
	return com_rslt;
}

 /**************************************************************************
 *	Description: *//**brief This API Reads the magnetometer data ready
 *	interrupt not active from the error register 0x0x2 bit 7
 *
 *
 *
 *
 *  \param u8 * v_mag_data_rdy_err_u8 :
 *	Pointer to holding the value of mag_drdy_err
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_mag_dada_rdy_err(
u8 *v_mag_data_rdy_err_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_MAG_DADA_RDY_ERR__REG,
			&v_data_u8, 1);
			*v_mag_data_rdy_err_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_MAG_DADA_RDY_ERR);
		}
	return com_rslt;
}

 /**************************************************************************
 *	Description: *//**brief This API Reads the error status
 *	from the error register 0x02 bit 0 to 7
 *
 *
 *
 *
 *	\param u8 * v_mag_data_rdy_err_u8, *v_fatal_er_u8r, *v_err_code_u8
 *   *i2c_fail_err, *v_drop_cmd_err_u8:
 *	Pointer to the value of v_mag_data_rdy_err_u8,
 *	v_fatal_er_u8r, v_err_code_u8,
 *	i2c_fail_err, v_drop_cmd_err_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_error_status(u8 *v_fatal_er_u8r,
u8 *v_err_code_u8, u8 *v_i2c_fail_err_u8,
u8 *v_drop_cmd_err_u8, u8 *v_mag_data_rdy_err_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_ERR_STAT__REG,
			&v_data_u8, 1);

			*v_fatal_er_u8r = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_FATAL_ERR);

			*v_err_code_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_ERR_CODE);

			*v_i2c_fail_err_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_I2C_FAIL_ERR);


			*v_drop_cmd_err_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_DROP_CMD_ERR);


			*v_mag_data_rdy_err_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_MAG_DADA_RDY_ERR);
		}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads the magnetometer power mode from
 *	PMU status register 0x03 bit 0 and 1
 *
 *  \param u8 * v_mag_power_mode_stat_u8 :
 *	pointer holding the power mode status of
 *	magnetometer
 *	mag_power_mode_stat:
 *	0X00	-	Suspend
 *	0X01	-	Normal
 *	0X10	-	Low power
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_mag_power_mode_stat(u8
*v_mag_power_mode_stat_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_MAG_POWER_MODE_STAT__REG,
			&v_data_u8, 1);
			*v_mag_power_mode_stat_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_MAG_POWER_MODE_STAT);
		}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads the gyroscope power mode from
 *	PMU status register 0x03 bit 2 and 3
 *
 *  \param u8 * v_gyro_power_mode_stat_u8 :	Pointer holding the value of
 *	gyroscope power mode status
 *	gyro_power_mode_stat:
 *	0X00	- Suspend
 *	0X01	- Normal
 *	0X10	- Reserved
 *	0X11	- Fast start-up
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_power_mode_stat(u8
*v_gyro_power_mode_stat_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_GYRO_POWER_MODE_STAT__REG,
			&v_data_u8, 1);
			*v_gyro_power_mode_stat_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_GYRO_POWER_MODE_STAT);
		}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads the accelerometer power mode from
 *	PMU status register 0x03 bit 4 and 5
 *
 *
 *  \param u8 * v_accel_power_mode_stat_u8 :
 *	Pointer holding the accelerometer power
 *	mode status
 *	v_accel_power_mode_stat_u8
 *	0X00 -	Suspend
 *	0X01 -	Normal
 *	0X10 -	Low power
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_power_mode_stat(u8
*v_accel_power_mode_stat_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_ACCEL_POWER_MODE_STAT__REG,
			&v_data_u8, 1);
			*v_accel_power_mode_stat_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_ACCEL_POWER_MODE_STAT);
		}
	return com_rslt;
}

/******************************************************************************
 *	Description: *//**brief This API reads magnetometer data X values
 *	from the register 0x04 and 0x05
 *
 *
 *
 *
 *  \param structure s16 * mag_x : Pointer holding the value of mag x
 *  \param v_sensor_select_u8 : Mag selection value;
 *	0 for BMM150 and 1 for AKM09911
 *
 *
 *
 *
 *  \return results of communication routine
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_read_mag_x(s16 *v_mag_x_s16,
u8 v_sensor_select_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8[2] = {0, 0};
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		switch (v_sensor_select_u8) {
		case BST_BMM:
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_DATA_MAG_X_LSB__REG,
			v_data_u8, 2);
			/* X axis*/
			v_data_u8[0] = BMI160_GET_BITSLICE(v_data_u8[0],
			BMI160_USER_DATA_MAG_X_LSB);
			*v_mag_x_s16 = (s16)
			((((s32)((s8)v_data_u8[1]))
			<< BMI160_SHIFT_5_POSITION) | (v_data_u8[0]));
		break;
		case BST_AKM:
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_DATA_0_MAG_X_LSB__REG,
			v_data_u8, 2);
			*v_mag_x_s16 = (s16)
			((((s32)((s8)v_data_u8[1]))
			<< BMI160_SHIFT_8_POSITION) | (v_data_u8[0]));
		break;
		default:
			com_rslt = E_BMI160_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads magnetometer data Y values
 *	from the register 0x06 and 0x07
 *
 *
 *
 *
 *  \param structure s16 * v_mag_y_s16 :
 *	Pointer holding the value of v_mag_y_s16
 *  \param v_sensor_select_u8 : Mag selection value;
 *	0 for BMM150 and 1 for AKM09911
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_read_mag_y(s16 *v_mag_y_s16,
u8 v_sensor_select_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_OUT_OF_RANGE;
	u8 v_data_u8[2] = {0, 0};
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		switch (v_sensor_select_u8) {
		case BST_BMM:
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_DATA_MAG_Y_LSB__REG,
			v_data_u8, 2);
			/*Y-axis lsb value shifting*/
			v_data_u8[0] = BMI160_GET_BITSLICE(v_data_u8[0],
			BMI160_USER_DATA_MAG_Y_LSB);
			*v_mag_y_s16 = (s16)
			((((s32)((s8)v_data_u8[1]))
			<< BMI160_SHIFT_5_POSITION) | (v_data_u8[0]));
		break;
		case BST_AKM:
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_DATA_2_MAG_Y_LSB__REG,
			v_data_u8, 2);
			*v_mag_y_s16 = (s16)
			((((s32)((s8)v_data_u8[1]))
			<< BMI160_SHIFT_8_POSITION) | (v_data_u8[0]));
		break;
		default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads magnetometer data Z values
 *	from the register 0x08 and 0x09
 *
 *
 *
 *
 *  \param structure s16 * mag_z : Pointer holding the value of mag_z
 *  \param v_sensor_select_u8 : Mag selection value;
 *	0 for BMM150 and 1 for AKM09911
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_read_mag_z(s16 *v_mag_z_s16,
u8 v_sensor_select_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8[2] = {0, 0};
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		switch (v_sensor_select_u8) {
		case BST_BMM:
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_DATA_MAG_Z_LSB__REG,
			v_data_u8, 2);
			/*Z-axis lsb value shifting*/
			v_data_u8[0] = BMI160_GET_BITSLICE(v_data_u8[0],
			BMI160_USER_DATA_MAG_Z_LSB);
			*v_mag_z_s16 = (s16)
			((((s32)((s8)v_data_u8[1]))
			<< BMI160_SHIFT_7_POSITION) | (v_data_u8[0]));
		break;
		case BST_AKM:
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_DATA_4_MAG_Z_LSB__REG,
			v_data_u8, 2);
			*v_mag_z_s16 = (s16)
			((((s32)((s8)v_data_u8[1]))
			<< BMI160_SHIFT_8_POSITION) | (v_data_u8[0]));
		break;
		default:
			com_rslt = E_BMI160_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads magnetometer data RHALL values
 *	from the register 0x0A and 0x0B
 *
 *
 *
 *
 *  \param structure s16 * mag_r : Pointer holding the value of mag_r
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_read_mag_r(s16 *v_mag_r_s16)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8[2] = {0, 0};
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_DATA_6_RHALL_LSB__REG,
			v_data_u8, 2);
			/*R-axis lsb value shifting*/
			v_data_u8[0] = BMI160_GET_BITSLICE(v_data_u8[0],
			BMI160_USER_DATA_MAG_R_LSB);
			*v_mag_r_s16 = (s16)
			((((s32)((s8)v_data_u8[1]))
			<< BMI160_SHIFT_6_POSITION) | (v_data_u8[0]));
		}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads magnetometer data X,Y,Z values
 *	from the register 0x04 to 0x09
 *
 *
 *
 *
 *  \param structure bmi160_mag_t * mag : Pointer holding the value of mag xyz
 *  \param v_sensor_select_u8 : Mag selection value;
 *	0 for BMM150 and 1 for AKM09911
 *
 *
 *  \return results of communication routine *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_read_mag_xyz(
struct bmi160_mag_t *mag, u8 v_sensor_select_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8[6] = {0, 0, 0, 0, 0, 0};
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		switch (v_sensor_select_u8) {
		case BST_BMM:
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_DATA_MAG_X_LSB__REG,
			v_data_u8, 6);
			/*X-axis lsb value shifting*/
			v_data_u8[0] = BMI160_GET_BITSLICE(v_data_u8[0],
			BMI160_USER_DATA_MAG_X_LSB);
			/* Data X */
			mag->x = (s16)
			((((s32)((s8)v_data_u8[1]))
			<< BMI160_SHIFT_5_POSITION) | (v_data_u8[0]));
			/* Data Y */
			/*Y-axis lsb value shifting*/
			v_data_u8[2] = BMI160_GET_BITSLICE(v_data_u8[2],
			BMI160_USER_DATA_MAG_Y_LSB);
			mag->y = (s16)
			((((s32)((s8)v_data_u8[3]))
			<< BMI160_SHIFT_5_POSITION) | (v_data_u8[2]));

			/* Data Z */
			/*Z-axis lsb value shifting*/
			v_data_u8[4] = BMI160_GET_BITSLICE(v_data_u8[4],
			BMI160_USER_DATA_MAG_Z_LSB);
			mag->z = (s16)
			((((s32)((s8)v_data_u8[5]))
			<< BMI160_SHIFT_7_POSITION) | (v_data_u8[4]));
		break;
		case BST_AKM:
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_DATA_0_MAG_X_LSB__REG,
			v_data_u8, 6);
			/* Data X */
			mag->x = (s16)
			((((s32)((s8)v_data_u8[1]))
			<< BMI160_SHIFT_8_POSITION) | (v_data_u8[0]));
			/* Data Y */
			mag->y  = ((((s32)((s8)v_data_u8[3]))
			<< BMI160_SHIFT_8_POSITION) | (v_data_u8[2]));
			/* Data Z */
			mag->z = (s16)
			((((s32)((s8)v_data_u8[5]))
			<< BMI160_SHIFT_8_POSITION) | (v_data_u8[4]));
		break;
		default:
			com_rslt = E_BMI160_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*******************************************************************************
 *	Description: *//**brief This API reads magnetometer data X,Y,Z,r
 *	values from the register 0x04 to 0x0B
 *
 *
 *
 *
 *  \param structure bmi160_mag_t * mag : Pointer holding the value of
 *	mag xyz and r
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_read_mag_xyzr(
struct bmi160_mag_xyzr_t *mag)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_DATA_MAG_X_LSB__REG,
			v_data_u8, 8);

			/* Data X */
			/*X-axis lsb value shifting*/
			v_data_u8[0] = BMI160_GET_BITSLICE(v_data_u8[0],
			BMI160_USER_DATA_MAG_X_LSB);
			mag->x = (s16)
			((((s32)((s8)v_data_u8[1]))
			<< BMI160_SHIFT_5_POSITION) | (v_data_u8[0]));
			/* Data Y */
			/*Y-axis lsb value shifting*/
			v_data_u8[2] = BMI160_GET_BITSLICE(v_data_u8[2],
			BMI160_USER_DATA_MAG_Y_LSB);
			mag->y = (s16)
			((((s32)((s8)v_data_u8[3]))
			<< BMI160_SHIFT_5_POSITION) | (v_data_u8[2]));

			/* Data Z */
			/*Z-axis lsb value shifting*/
			v_data_u8[4] = BMI160_GET_BITSLICE(v_data_u8[4],
			BMI160_USER_DATA_MAG_Z_LSB);
			mag->z = (s16)
			((((s32)((s8)v_data_u8[5]))
			<< BMI160_SHIFT_7_POSITION) | (v_data_u8[4]));

			/* RHall */
			/*R-axis lsb value shifting*/
			v_data_u8[6] = BMI160_GET_BITSLICE(v_data_u8[6],
			BMI160_USER_DATA_MAG_R_LSB);
			mag->r = (s16)
			((((s32)((s8)v_data_u8[7]))
			<< BMI160_SHIFT_6_POSITION) | (v_data_u8[6]));
		}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads gyro data X values
 *	form the register 0x0C and 0x0D
 *
 *
 *
 *
 *  \param s16 * v_gyro_x_s16 : Pointer holding the value of gyro x
 *
 *
 *
 *  \return results of communication routine
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_read_gyro_x(s16 *v_gyro_x_s16)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8[2] = {0, 0};
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_DATA_8_GYRO_X_LSB__REG,
			v_data_u8, 2);

			*v_gyro_x_s16 = (s16)
			((((s32)((s8)v_data_u8[1]))
			<< BMI160_SHIFT_8_POSITION) | (v_data_u8[0]));
		}
	return com_rslt;
}
/*******************************************************************************
 *	Description: *//**brief This API reads gyro data Y values
 *	form the register 0x0E and 0x0F
 *
 *
 *
 *
 *  \param s16 * v_gyro_y_s16 : Pointer holding the value of gyro y
 *
 *
 *
 *  \return results of communication routine result of communication routines
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_read_gyro_y(s16 *v_gyro_y_s16)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8[2] = {0, 0};
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_DATA_10_GYRO_Y_LSB__REG,
			v_data_u8, 2);

			*v_gyro_y_s16 = (s16)
			((((s32)((s8)v_data_u8[1]))
			<< BMI160_SHIFT_8_POSITION) | (v_data_u8[0]));
		}
	return com_rslt;
}
/*******************************************************************************
 *	Description: *//**brief This API reads gyro data Z values
 *	form the register 0x10 and 0x11
 *
 *
 *
 *
 *  \param s16 * v_gyro_z_s16 : Pointer holding the value of gyro_z
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_read_gyro_z(s16 *v_gyro_z_s16)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8[2] = {0, 0};
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_DATA_12_GYRO_Z_LSB__REG,
			v_data_u8, 2);

			*v_gyro_z_s16 = (s16)
			((((s32)((s8)v_data_u8[1]))
			<< BMI160_SHIFT_8_POSITION) | (v_data_u8[0]));
		}
	return com_rslt;
}
/*******************************************************************************
 *	Description: *//**brief This API reads gyro data X,Y,Z values
 *	from the register 0x0C to 0x11
 *
 *
 *
 *
 *  \param struct bmi160_mag_t * gyro : Pointer holding the value of gyro - xyz
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_read_gyro_xyz(struct bmi160_gyro_t *gyro)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8[6] = {0, 0, 0, 0, 0, 0};
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_DATA_8_GYRO_X_LSB__REG,
			v_data_u8, 6);

			/* Data X */
			gyro->x = (s16)
			((((s32)((s8)v_data_u8[1]))
			<< BMI160_SHIFT_8_POSITION) | (v_data_u8[0]));
			/* Data Y */
			gyro->y = (s16)
			((((s32)((s8)v_data_u8[3]))
			<< BMI160_SHIFT_8_POSITION) | (v_data_u8[2]));

			/* Data Z */
			gyro->z = (s16)
			((((s32)((s8)v_data_u8[5]))
			<< BMI160_SHIFT_8_POSITION) | (v_data_u8[4]));
		}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads accelerometer data X values
 *	form the register 0x12 and 0x13
 *
 *
 *
 *
 *  \param s16 * v_accel_x_s16 : Pointer holding the value of acc_x
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_read_accel_x(s16 *v_accel_x_s16)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8[2] = {0, 0};
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_DATA_14_ACCEL_X_LSB__REG,
			v_data_u8, 2);

			*v_accel_x_s16 = (s16)
			((((s32)((s8)v_data_u8[1]))
			<< BMI160_SHIFT_8_POSITION) | (v_data_u8[0]));
		}
	return com_rslt;
}
/******************************************************************************
 *	Description: *//**brief This API reads accelerometer data Y values
 *	form the register 0x14 and 0x15
 *
 *
 *
 *
 *  \param s16 * v_accel_y_s16 : Pointer holding the value of v_accel_y_s16
 *
 *
 *
 *  \return results of communication routine
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_read_accel_y(s16 *v_accel_y_s16)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8[2] = {0, 0};
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_DATA_16_ACCEL_Y_LSB__REG,
			v_data_u8, 2);

			*v_accel_y_s16 = (s16)
			((((s32)((s8)v_data_u8[1]))
			<< BMI160_SHIFT_8_POSITION) | (v_data_u8[0]));
		}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads accelerometer data Z values
 *	form the register 0x16 and 0x17
 *
 *
 *
 *
 *  \param s16 * v_accel_z_s16 : Pointer holding the value of z
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_read_accel_z(s16 *v_accel_z_s16)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 a_data_u8r[2] = {0, 0};
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_DATA_18_ACCEL_Z_LSB__REG,
			a_data_u8r, 2);

			*v_accel_z_s16 = (s16)
			((((s32)((s8)a_data_u8r[1]))
			<< BMI160_SHIFT_8_POSITION) | (a_data_u8r[0]));
		}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads accelerometer data X,Y,Z values
 *	from the register 0x12 to 0x17
 *
 *
 *
 *
 *  \param bmi160accel_t * accel : Pointer holding the value of acc-xyz
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_read_accel_xyz(
struct bmi160_accel_t *accel)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 a_data_u8r[6] = {0, 0, 0, 0, 0, 0};
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_DATA_14_ACCEL_X_LSB__REG,
			a_data_u8r, 6);

			/* Data X */
			accel->x = (s16)
			((((s32)((s8)a_data_u8r[1]))
			<< BMI160_SHIFT_8_POSITION) | (a_data_u8r[0]));
			/* Data Y */
			accel->y = (s16)
			((((s32)((s8)a_data_u8r[3]))
			<< BMI160_SHIFT_8_POSITION) | (a_data_u8r[2]));

			/* Data Z */
			accel->z = (s16)
			((((s32)((s8)a_data_u8r[5]))
			<< BMI160_SHIFT_8_POSITION) | (a_data_u8r[4]));
		}
	return com_rslt;
}

/******************************************************************************
 *	Description: *//**brief This API reads sensor_time from the register
 *	0x18 to 0x1A
 *
 *
 *  \param u32 * v_sensor_time_u32 : Pointer holding the value of sensor time
 *
 *
 *
 *  \return results of communication routine
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_sensor_time(u32 *v_sensor_time_u32)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 a_data_u8r[3] = {0, 0, 0};
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_SENSORTIME_0_SENSOR_TIME_LSB__REG,
			a_data_u8r, 3);

			*v_sensor_time_u32 = (u32)
			((((u32)a_data_u8r[2]) << BMI160_SHIFT_16_POSITION)
			|(((u32)a_data_u8r[1]) << BMI160_SHIFT_8_POSITION)
			| (a_data_u8r[0]));
		}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads the Gyroscope self test
 *	status from the register 0x1B bit 1
 *
 *
 *  \param u8 * v_gyro_selftest( : Pointer holding the value of gyro self test
 *	v_gyro_selftest( ->
 *	0 -	Gyroscope self test is running or failed
 *	1 -	Gyroscope self test completed successfully
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_selftest(u8
*v_gyro_selftest_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_STAT_GYRO_SELFTEST_OK__REG,
			&v_data_u8, 1);
			*v_gyro_selftest_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_STAT_GYRO_SELFTEST_OK);
		}
	return com_rslt;
}
 /*****************************************************************************
 *	Description: *//**brief This API reads the status of
 *	mag manual interface operation form the register 0x1B bit 2
 *
 *
 *
 *  \param u8 * v_mag_manual_stat_u8 :
 *	Pointer holding the value of manual operation
 *	v_mag_manual_stat_u8 ->
 *	1 ->	Indicates manual magnetometer interface operation is ongoing
 *	0 ->	Indicates no manual magnetometer interface operation is ongoing
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_mag_manual_operation_stat(u8
*v_mag_manual_stat_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_STAT_MAG_MANUAL_OPERATION__REG,
			&v_data_u8, 1);
			*v_mag_manual_stat_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_STAT_MAG_MANUAL_OPERATION);
		}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads the fast offset compensation
 *	status form the register 0x1B bit 3
 *
 *
 *  \param u8 * v_foc_rdy_u8 : Pointer holding the value of foc rdy
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_foc_rdy(u8
*v_foc_rdy_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_STAT_FOC_RDY__REG, &v_data_u8, 1);
			*v_foc_rdy_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_STAT_FOC_RDY);
		}
	return com_rslt;
}
 /*****************************************************************************
 * Description: *//**brief This APIrReads the nvm_rdy status from the
 *	resister 0x1B bit 4
 *
 *
 *  \param u8 * v_nvm_rdy_u8 : Pointer holding the value of v_nvm_rdy_u8
 *	v_nvm_rdy_u8 ->
 *	0 ->	NVM write operation in progress
 *	1 ->	NVM is ready to accept a new write trigger
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_nvm_rdy(u8
*v_nvm_rdy_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_STAT_NVM_RDY__REG, &v_data_u8, 1);
			*v_nvm_rdy_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_STAT_NVM_RDY);
		}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads the status of drdy_mag
 *	from the register 0x1B bit 5
 *	The status get reset when one magneto DATA register is read out
 *
 *  \param u8 * v_data_rdy_u8 : Pointer holding the value of drdy mag
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_data_rdy_mag(u8
*v_data_rdy_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_STAT_DATA_RDY_MAG__REG, &v_data_u8, 1);
			*v_data_rdy_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_STAT_DATA_RDY_MAG);
		}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads the status of v_data_rdy_u8 form the
 *	register 0x1B bit 6
 *	The status get reset when Gyroscope DATA register read out
 *
 *
 *	\param u8 * v_data_rdy_u8 :	Pointer holding the value of drdy gyr
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_data_rdy(u8
*v_data_rdy_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_STAT_DATA_RDY_GYRO__REG, &v_data_u8, 1);
			*v_data_rdy_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_STAT_DATA_RDY_GYRO);
		}
	return com_rslt;
}
 /*****************************************************************************
 *	Description: *//**brief This API reads the status of drdy_acc form the
 *	register 0x1B bit 7
 *	The status get reset when Accel DATA register read out
 *
 *
 *	\param u8 * v_data_rdy_u8 :	Pointer holding the value of drdy acc
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_data_rdy(u8
*v_data_rdy_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_STAT_DATA_RDY_ACCEL__REG, &v_data_u8, 1);
			*v_data_rdy_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_STAT_DATA_RDY_ACCEL);
		}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads the step interrupt status
 *	from the register 0x1C bit 0
 *	flag is associated with a specific interrupt function.
 *	It is set when the single tab interrupt triggers. The
 *	setting of INT_LATCH controls if the interrupt
 *	signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *  \param u8 * v_step_intr_u8 :
 *	Pointer holding the status of step interrupt
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat0_step_intr(u8
*v_step_intr_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_INTR_STAT_0_STEP_INTR__REG, &v_data_u8, 1);
			*v_step_intr_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_STAT_0_STEP_INTR);
		}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads the
 *	significant motion interrupt status
 *	from the register 0x1C bit 1
 *	flag is associated with a specific interrupt function.
 *	It is set when the single tab interrupt triggers. The
 *	setting of INT_LATCH controls if the interrupt
 *	signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *
 *  \param u8 * v_significant_intr_u8  :
 *	Pointer holding the status of step interrupt
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat0_significant_intr(u8
*v_significant_intr_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_INTR_STAT_0_SIGNIFICANT_INTR__REG,
			&v_data_u8, 1);
			*v_significant_intr_u8  = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_STAT_0_SIGNIFICANT_INTR);
		}
	return com_rslt;
}
 /*****************************************************************************
 *	Description: *//**brief This API reads the anym intr status
 *	from the register 0x1C bit 2
 *	flag is associated with a specific interrupt function.
 *	It is set when the single tab interrupt triggers. The
 *	setting of INT_LATCH controls if the interrupt
 *	signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *
 *  \param u8 * v_any_motion_intr_u8 :
 *	Pointer holding the status of any-motion interrupt
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat0_any_motion_intr(u8
*v_any_motion_intr_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_INTR_STAT_0_ANY_MOTION__REG, &v_data_u8, 1);
			*v_any_motion_intr_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_STAT_0_ANY_MOTION);
		}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads the v_pmu_trigger_intr_u8 status
 *	from the register 0x1C bit 3
 *	flag is associated with a specific interrupt function.
 *	It is set when the single tab interrupt triggers. The
 *	setting of INT_LATCH controls if the interrupt
 *	signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *
 *  \param u8 * v_pmu_trigger_intr_u8 : Pointer holding the status of
 *					v_pmu_trigger_intr_u8 interrupt
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat0_pmu_trigger_intr(u8
*v_pmu_trigger_intr_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_INTR_STAT_0_PMU_TRIGGER__REG,
			&v_data_u8, 1);
			*v_pmu_trigger_intr_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_STAT_0_PMU_TRIGGER);
		}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads the double tab status
 *	from the register 0x1C bit 4
 *	flag is associated with a specific interrupt function.
 *	It is set when the single tab interrupt triggers. The
 *	setting of INT_LATCH controls if the interrupt
 *	signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *  \param u8 * v_double_tap_intr_u8 :
 *	Pointer holding the status of double_tap intr
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat0_double_tap_intr(u8
*v_double_tap_intr_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_INTR_STAT_0_DOUBLE_TAP_INTR__REG,
			&v_data_u8, 1);
			*v_double_tap_intr_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_STAT_0_DOUBLE_TAP_INTR);
		}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads the single tab status
 *	from the register 0x1C bit 5
 *	flag is associated with a specific interrupt function.
 *	It is set when the single tab interrupt triggers. The
 *	setting of INT_LATCH controls if the interrupt
 *	signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *  \param u8 * v_single_tap_intr_u8 :
 *	Pointer holding the status of single_tap intr
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat0_single_tap_intr(u8
*v_single_tap_intr_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_INTR_STAT_0_SINGLE_TAP_INTR__REG,
			&v_data_u8, 1);
			*v_single_tap_intr_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_STAT_0_SINGLE_TAP_INTR);
		}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads the orient status
 *	from the register 0x1C bit 6
 *	flag is associated with a specific interrupt function.
 *	It is set when the orient interrupt triggers. The
 *	setting of INT_LATCH controls if the
 *	interrupt signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *  \param u8 * v_orient_intr_u8 :  Pointer holding the status of orient intr
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat0_orient_intr(u8
*v_orient_intr_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_INTR_STAT_0_ORIENT__REG, &v_data_u8, 1);
			*v_orient_intr_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_STAT_0_ORIENT);
		}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads the flat interrupt status
 *	from the register 0x1C bit 7
 *	flag is associated with a specific interrupt function.
 *	It is set when the flat interrupt triggers. The
 *	setting of INT_LATCH controls if the
 *	interrupt signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *  \param u8 * v_flat_intr_u8 : Pointer holding the status of  flat_intr
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat0_flat_intr(u8
*v_flat_intr_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_INTR_STAT_0_FLAT__REG, &v_data_u8, 1);
			*v_flat_intr_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_STAT_0_FLAT);
		}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads the high g interrupt status
 *	from the register 0x1D bit 2
 *	flag is associated with a specific interrupt function.
 *	It is set when the high g  interrupt triggers. The
 *	setting of INT_LATCH controls if the interrupt signal and hence the
 *	respective interrupt flag will be permanently
 *	latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *  \param u8 * v_high_g_intr_u8 : Pointer holding the status of highg intr
 *
 *
 *
 *  \return results of communication routine
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat1_high_g_intr(u8
*v_high_g_intr_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_INTR_STAT_1_HIGH_G_INTR__REG,
			&v_data_u8, 1);
			*v_high_g_intr_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_STAT_1_HIGH_G_INTR);
		}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads the low g interrupt status
 *	from the register 0x1D bit 3
 *	flag is associated with a specific interrupt function.
 *	It is set when the low g  interrupt triggers. The
 *	setting of INT_LATCH controls if the interrupt signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *  \param u8 * v_low_g_intr_u8 : Pointer holding the status of lowg intr
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat1_low_g_intr(u8
*v_low_g_intr_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_INTR_STAT_1_LOW_G_INTR__REG, &v_data_u8, 1);
			*v_low_g_intr_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_STAT_1_LOW_G_INTR);
		}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads data ready low g interrupt status
 *	from the register 0x1D bit 4
 *	flag is associated with a specific interrupt function.
 *	It is set when the  data ready  interrupt triggers. The
 *	setting of INT_LATCH controls if the interrupt signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *  \param u8 * v_data_rdy_intr_u8 : Pointer holding the status of drdy intr
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat1_data_rdy_intr(u8
*v_data_rdy_intr_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_INTR_STAT_1_DATA_RDY_INTR__REG,
			&v_data_u8, 1);
			*v_data_rdy_intr_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_STAT_1_DATA_RDY_INTR);
		}
	return com_rslt;
}

/*****************************************************************************
 *	Description: *//**brief This API reads data ready FIFO full interrupt status
 *	from the register 0x1D bit 5
 *	flag is associated with a specific interrupt function.
 *	It is set when the FIFO full interrupt triggers. The
 *	setting of INT_LATCH controls if the
 *	interrupt signal and hence the
 *	respective interrupt flag will
 *	be permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *  \param u8 * v_fifo_full_intr_u8 : Pointer holding the status of ffull intr
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat1_fifo_full_intr(u8
*v_fifo_full_intr_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_INTR_STAT_1_FIFO_FULL_INTR__REG,
			&v_data_u8, 1);
			*v_fifo_full_intr_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_STAT_1_FIFO_FULL_INTR);
		}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads data
 *	 ready FIFO watermark interrupt status
 *	from the register 0x1D bit 6
 *	flag is associated with a specific interrupt function.
 *	It is set when the FIFO watermark interrupt triggers. The
 *	setting of INT_LATCH controls if the
 *	interrupt signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *  \param u8 * v_fifo_wm_intr_u8 : Pointer holding the status of fwmintr
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat1_fifo_wm_intr(u8
*v_fifo_wm_intr_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_INTR_STAT_1_FIFO_WM_INTR__REG,
			&v_data_u8, 1);
			*v_fifo_wm_intr_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_STAT_1_FIFO_WM_INTR);
		}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads data ready no motion interrupt status
 *	from the register 0x1D bit 7
 *	flag is associated with a specific interrupt function.
 *	It is set when the no motion  interrupt triggers. The
 *	setting of INT_LATCH controls if the interrupt signal and hence the
 *	respective interrupt flag will be permanently
 *	latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *  \param u8 * v_nomotion_intr_u8 : Pointer holding the status of nomointr
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat1_nomotion_intr(u8
*v_nomotion_intr_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_INTR_STAT_1_NOMOTION_INTR__REG,
			&v_data_u8, 1);
			*v_nomotion_intr_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_STAT_1_NOMOTION_INTR);
		}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads the status of anym_first x
 *	from the register 0x1E bit 0
 *
 *
 *
 *
 *  \param u8 * v_anymotion_first_x_u8 :
 *	pointer holding the status of anym firstx
 *	v_anymotion_first_x_u8 ->
 *	0	-	not triggered
 *	1	-	triggered by x axis
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat2_any_motion_first_x(u8
*v_anymotion_first_x_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_X__REG,
			&v_data_u8, 1);
			*v_anymotion_first_x_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_X);
		}
	return com_rslt;
}

/*****************************************************************************
 *	Description: *//**brief This API reads the status of anym_firsty
 *	from the register 0x1E bit 1
 *
 *
 *
 *
 *  \param u8 * v_any_motion_first_y_u8 :
 *	pointer holding the status of anym_firsty
 *	v_any_motion_first_y_u8 ->
 *	0	-	not triggered
 *	1	-	triggered by y axis
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat2_any_motion_first_y(u8
*v_any_motion_first_y_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Y__REG,
			&v_data_u8, 1);
			*v_any_motion_first_y_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Y);
		}
	return com_rslt;
}

 /*****************************************************************************
 *	Description: *//**brief This API reads the status of anymotion_firstz
 *	from the register 0x1E bit 2
 *
 *
 *
 *
 *  \param u8 * v_any_motion_first_z_u8 :
 *	pointer holding the status of anymotion_firstz
 *	v_any_motion_first_z_u8 ->
 *	0	-	not triggered
 *	1	-	triggered by z axis
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat2_any_motion_first_z(u8
*v_any_motion_first_z_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Z__REG,
			&v_data_u8, 1);
			*v_any_motion_first_z_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Z);
		}
	return com_rslt;
}
 /*****************************************************************************
 *	Description: *//**brief This API reads the anymsign status from the
 *	register 0x1E bit 3
 *
 *
 *
 *
 *  \param u8 * v_anymotion_sign_u8 : Pointer holding the status of anym sign
 *	v_anymotion_sign_u8 ->
 *	0	-	positive
 *	1	-	negative
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat2_any_motion_sign(u8
*v_anymotion_sign_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_INTR_STAT_2_ANY_MOTION_SIGN__REG,
			&v_data_u8, 1);
			*v_anymotion_sign_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_STAT_2_ANY_MOTION_SIGN);
		}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads the tap_first x status from the
 *	register 0x1E bit 4
 *
 *
 *
 *
 *  \param u8 * v_tap_first_x_u8 :
 *	Pointer holding the status of tap_first x
 *	v_tap_first_x_u8 ->
 *	0	-	not triggered
 *	1	-	triggered by x axis
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat2_any_motion_tap_first_x(u8
*v_tap_first_x_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_INTR_STAT_2_TAP_FIRST_X__REG,
			&v_data_u8, 1);
			*v_tap_first_x_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_STAT_2_TAP_FIRST_X);
		}
	return com_rslt;
}


/*****************************************************************************
 *	Description: *//**brief This API reads the tap_firsty status from the
 *	register 0x1E bit 5
 *
 *
 *
 *
 *  \param u8 * v_tap_first_y_u8 :
 *	Pointer holding the status of tap_first_ y
 *	v_tap_first_y_u8 ->
 *	0	-	not triggered
 *	1	-	triggered by y axis
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat2_any_motion_tap_first_y(u8
*v_tap_first_y_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_INTR_STAT_2_TAP_FIRST_Y__REG,
			&v_data_u8, 1);
			*v_tap_first_y_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_STAT_2_TAP_FIRST_Y);
		}
	return com_rslt;
}

/*****************************************************************************
 *	Description: *//**brief This API reads the tap_firstz status from the
 *	register 0x1E bit 6
 *
 *
 *
 *
 *  \param u8 * v_tap_first_z_u8 :
 *	Pointer holding the status of tap_firstZ
 *	v_tap_first_z_u8 ->
 *	0	-	not triggered
 *	1	-	triggered by z axis
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat2_any_motion_tap_first_z(u8
*v_tap_first_z_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_INTR_STAT_2_TAP_FIRST_Z__REG,
			&v_data_u8, 1);
			*v_tap_first_z_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_STAT_2_TAP_FIRST_Z);
		}
	return com_rslt;
}

/*****************************************************************************
 *	Description: *//**brief This API reads the tap_sign status from the
 *	register 0x1E bit 7
 *
 *
 *
 *
 *  \param u8 * v_tap_sign_u8 : Pointer holding the status of tap_sign
 *	tap_sign ->
 *	0	-	positive
 *	1	-	negative
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat2_tap_sign(u8
*v_tap_sign_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_INTR_STAT_2_TAP_SIGN__REG, &v_data_u8, 1);
			*v_tap_sign_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_STAT_2_TAP_SIGN);
		}
	return com_rslt;
}

/*****************************************************************************
 *	Description: *//**brief This API reads the high_first x status from the
 *	register 0x1F bit 0
 *
 *
 *
 *
 *  \param u8 * v_high_g_first_x_u8 :
 *	Pointer holding the status of high_first x
 *	v_high_g_first_x_u8 ->
 *	0	-	not triggered
 *	1	-	triggered by x axis
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat3_high_g_first_x(u8
*v_high_g_first_x_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_X__REG,
			&v_data_u8, 1);
			*v_high_g_first_x_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_X);
		}
	return com_rslt;
}

/*****************************************************************************
 *	Description: *//**brief This API reads the high_firsty status from the
 *	register 0x1F bit 1
 *
 *
 *
 *
 *  \param u8 * v_high_g_first_y_u8 :
 *	Pointer holding the status of high_firsty
 *	v_high_g_first_y_u8 ->
 *	0	-	not triggered
 *	1	-	triggered by y axis
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat3_high_g_first_y(u8
*v_high_g_first_y_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_Y__REG,
			&v_data_u8, 1);
			*v_high_g_first_y_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_Y);
		}
	return com_rslt;
}

/*****************************************************************************
 *	Description: *//**brief This API reads the high_firstz status from the
 *	register 0x1F bit 3
 *
 *
 *
 *
 *  \param u8 * v_high_g_first_z_u8 :
 *	Pointer holding the status of high_firstz
 *	v_high_g_first_z_u8 ->
 *	0	-	not triggered
 *	1	-	triggered by z axis
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat3_high_g_first_z(u8
*v_high_g_first_z_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_Z__REG,
			&v_data_u8, 1);
			*v_high_g_first_z_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_Z);
		}
	return com_rslt;
}

/*****************************************************************************
 *	Description: *//**brief This API reads the high sign status from the
 *	register 0x1F bit 3
 *
 *
 *
 *
 *  \param u8 * v_high_g_sign_u8 : Pointer holding the status of high sign
 *	v_high_g_sign_u8 ->
 *	0	-	positive
 *	1	-	negative
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat3_high_g_sign(u8
*v_high_g_sign_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_INTR_STAT_3_HIGH_G_SIGN__REG,
			&v_data_u8, 1);
			*v_high_g_sign_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_STAT_3_HIGH_G_SIGN);
		}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads the status of orient_xy plane
 *	from the register 0x1F bit 4 and 5
 *
 *
 *  \param u8 * v_orient_x_y_u8 :
 *	Pointer holding the status of orient_xy
 *	v_orient_x_y_u8 ->
 *	00	-	portrait upright
 *	01	-	portrait upside down
 *	10	-	landscape left
 *	11	-	landscape right
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat3_orient_xy(u8
*v_orient_xy_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_INTR_STAT_3_ORIENT_XY__REG,
			&v_data_u8, 1);
			*v_orient_xy_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_STAT_3_ORIENT_XY);
		}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads the status of orient z plane
 *	from the register 0x1F bit 6
 *
 *
 *  \param u8 * v_orient_z_u8 :
 *	Pointer holding the status of orient z
 *	v_orient_z_u8 ->
 *	0	-	upward looking
 *	1	-	downward looking
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat3_orient_z(u8
*v_orient_z_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_INTR_STAT_3_ORIENT_Z__REG, &v_data_u8, 1);
			*v_orient_z_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_STAT_3_ORIENT_Z);
		}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads the flatstatus from the register
 *	0x1F bit 7
 *
 *
 *  \param u8 * v_flat_u8 :
 *	Pointer holding the status of flat-intr
 *	v_flat_u8 ->
 *	0	-	non flat
 *	1	-	flat position
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat3_flat(u8
*v_flat_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_INTR_STAT_3_FLAT__REG, &v_data_u8, 1);
			*v_flat_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_STAT_3_FLAT);
		}
	return com_rslt;
}

/*****************************************************************************
 *	Description: *//**brief This API reads the temperature of the sensor
 *	from the register 0x21 bit 0 to 7
 *
 *
 *
 *  \param u8 * v_temp_s16 : Pointer holding the value of temperature
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_temp(s16
*v_temp_s16)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8[2] = {0, 0};
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_TEMP_LSB_VALUE__REG, v_data_u8, 2);
			*v_temp_s16 =
			(s16)(((s32)((s8) (v_data_u8[1]) <<
			BMI160_SHIFT_8_POSITION))|v_data_u8[0]);
		}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads the  of the sensor
 *	form the register 0x23 and 0x24 bit 0 to 7 and 0 to 2
 *
 *
 *  \param u8 * v_fifo_length_u32 :
 *	Pointer holding the Fifo_length
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_fifo_length(u32 *v_fifo_length_u32)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 a_data_u8r[2] = {0, 0};
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_FIFO_BYTE_COUNTER_LSB__REG, a_data_u8r, 2);

			a_data_u8r[1] =
			BMI160_GET_BITSLICE(a_data_u8r[1],
			BMI160_USER_FIFO_BYTE_COUNTER_MSB);

			*v_fifo_length_u32 =
			(u32)(((u32)((u8) (a_data_u8r[1]) <<
			BMI160_SHIFT_8_POSITION))|a_data_u8r[0]);
		}
	return com_rslt;
}
 /*****************************************************************************
 *	Description: *//**brief This API reads the fifo data of the sensor
 *	from the register 0x24
 *
 *
 *
 *  \param u8 * v_fifo_data_u8 : Pointer holding the fifo data
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_fifo_data(
u8 *v_fifo_data_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BURST_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_FIFO_DATA__REG, v_fifo_data_u8, FIFO_FRAME);
		}
	return com_rslt;
}

/*************************************************************************
 *	Description: *//**brief This API is used to get the
 *	accel output date rate form the register 0x40 bit 0 to 3
 *
 *
 *  \param  u8 * v_output_data_rate_u8 :
 *	pointer holding the value of accel output date rate
 *	0	-	BMI160_ACCEL_OUTPUT_DATA_RATE_RESERVED
 *	1	-	BMI160_ACCEL_OUTPUT_DATA_RATE_0_78HZ
 *	2	-	BMI160_ACCEL_OUTPUT_DATA_RATE_1_56HZ
 *	3	-	BMI160_ACCEL_OUTPUT_DATA_RATE_3_12HZ
 *	4	-	BMI160_ACCEL_OUTPUT_DATA_RATE_6_25HZ
 *	5	-	BMI160_ACCEL_OUTPUT_DATA_RATE_12_5HZ
 *	6	-	BMI160_ACCEL_OUTPUT_DATA_RATE_25HZ
 *	7	-	BMI160_ACCEL_OUTPUT_DATA_RATE_50HZ
 *	8	-	BMI160_ACCEL_OUTPUT_DATA_RATE_100HZ
 *	9	-	BMI160_ACCEL_OUTPUT_DATA_RATE_200HZ
 *	10	-	BMI160_ACCEL_OUTPUT_DATA_RATE_400HZ
 *	11	-	BMI160_ACCEL_OUTPUT_DATA_RATE_800HZ
 *	12	-	BMI160_ACCEL_OUTPUT_DATA_RATE_1600HZ
 *
 *
 *  \return results of communication routine
 *
 *
 ***********************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_output_data_rate(
u8 *v_output_data_rate_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_ACCEL_CONFIG_OUTPUT_DATA_RATE__REG,
			&v_data_u8, 1);
			*v_output_data_rate_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_ACCEL_CONFIG_OUTPUT_DATA_RATE);
		}
	return com_rslt;
}
 /****************************************************************************
 *	Description: *//**brief This API is used to set the
 *	accel output date rate form the register 0x40 bit 0 to 3
 *
 *
 *
 *
 *  \param u8 v_output_data_rate_u8 : The value of accel output date rate
 *
 *	0	-	BMI160_ACCEL_OUTPUT_DATA_RATE_RESERVED
 *	1	-	BMI160_ACCEL_OUTPUT_DATA_RATE_0_78HZ
 *	2	-	BMI160_ACCEL_OUTPUT_DATA_RATE_1_56HZ
 *	3	-	BMI160_ACCEL_OUTPUT_DATA_RATE_3_12HZ
 *	4	-	BMI160_ACCEL_OUTPUT_DATA_RATE_6_25HZ
 *	5	-	BMI160_ACCEL_OUTPUT_DATA_RATE_12_5HZ
 *	6	-	BMI160_ACCEL_OUTPUT_DATA_RATE_25HZ
 *	7	-	BMI160_ACCEL_OUTPUT_DATA_RATE_50HZ
 *	8	-	BMI160_ACCEL_OUTPUT_DATA_RATE_100HZ
 *	9	-	BMI160_ACCEL_OUTPUT_DATA_RATE_200HZ
 *	10	-	BMI160_ACCEL_OUTPUT_DATA_RATE_400HZ
 *	11	-	BMI160_ACCEL_OUTPUT_DATA_RATE_800HZ
 *	12	-	BMI160_ACCEL_OUTPUT_DATA_RATE_1600HZ
 *
 *
 *
 *  \return results of communication routine communication results
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_output_data_rate(
u8 v_output_data_rate_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	u8 v_data_output_datarate_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_output_data_rate_u8 <= C_BMI160_SIXTEEN_U8X) {
			switch (v_output_data_rate_u8) {
			case BMI160_ACCEL_OUTPUT_DATA_RATE_RESERVED:
				v_data_output_datarate_u8 =
				BMI160_ACCEL_OUTPUT_DATA_RATE_RESERVED;
				break;
			case BMI160_ACCEL_OUTPUT_DATA_RATE_0_78HZ:
				v_data_output_datarate_u8 =
				BMI160_ACCEL_OUTPUT_DATA_RATE_0_78HZ;
				break;
			case BMI160_ACCEL_OUTPUT_DATA_RATE_1_56HZ:
				v_data_output_datarate_u8 =
				BMI160_ACCEL_OUTPUT_DATA_RATE_1_56HZ;
				break;
			case BMI160_ACCEL_OUTPUT_DATA_RATE_3_12HZ:
				v_data_output_datarate_u8 =
				BMI160_ACCEL_OUTPUT_DATA_RATE_3_12HZ;
				break;
			case BMI160_ACCEL_OUTPUT_DATA_RATE_6_25HZ:
				v_data_output_datarate_u8 =
				BMI160_ACCEL_OUTPUT_DATA_RATE_6_25HZ;
				break;
			case BMI160_ACCEL_OUTPUT_DATA_RATE_12_5HZ:
				v_data_output_datarate_u8 =
				BMI160_ACCEL_OUTPUT_DATA_RATE_12_5HZ;
				break;
			case BMI160_ACCEL_OUTPUT_DATA_RATE_25HZ:
				v_data_output_datarate_u8 =
				BMI160_ACCEL_OUTPUT_DATA_RATE_25HZ;
				break;
			case BMI160_ACCEL_OUTPUT_DATA_RATE_50HZ:
				v_data_output_datarate_u8 =
				BMI160_ACCEL_OUTPUT_DATA_RATE_50HZ;
				break;
			case BMI160_ACCEL_OUTPUT_DATA_RATE_100HZ:
				v_data_output_datarate_u8 =
				BMI160_ACCEL_OUTPUT_DATA_RATE_100HZ;
				break;
			case BMI160_ACCEL_OUTPUT_DATA_RATE_200HZ:
				v_data_output_datarate_u8 =
				BMI160_ACCEL_OUTPUT_DATA_RATE_200HZ;
				break;
			case BMI160_ACCEL_OUTPUT_DATA_RATE_400HZ:
				v_data_output_datarate_u8 =
				BMI160_ACCEL_OUTPUT_DATA_RATE_400HZ;
				break;
			case BMI160_ACCEL_OUTPUT_DATA_RATE_800HZ:
				v_data_output_datarate_u8 =
				BMI160_ACCEL_OUTPUT_DATA_RATE_800HZ;
				break;
			case BMI160_ACCEL_OUTPUT_DATA_RATE_1600HZ:
				v_data_output_datarate_u8 =
				BMI160_ACCEL_OUTPUT_DATA_RATE_1600HZ;
				break;
			case BMI160_ACCEL_OUTPUT_DATA_RATE_RESERVED0:
				v_data_output_datarate_u8 =
				BMI160_ACCEL_OUTPUT_DATA_RATE_RESERVED0;
				break;
			case BMI160_ACCEL_OUTPUT_DATA_RATE_RESERVED1:
				v_data_output_datarate_u8 =
				BMI160_ACCEL_OUTPUT_DATA_RATE_RESERVED1;
				break;
			case BMI160_ACCEL_OUTPUT_DATA_RATE_RESERVED2:
				v_data_output_datarate_u8 =
				BMI160_ACCEL_OUTPUT_DATA_RATE_RESERVED2;
				break;
			default:
				break;
			}
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_ACCEL_CONFIG_OUTPUT_DATA_RATE__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_ACCEL_CONFIG_OUTPUT_DATA_RATE,
				v_data_output_datarate_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_ACCEL_CONFIG_OUTPUT_DATA_RATE__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/***********************************************************************
 *	Description: *//**brief This API is used to get the
 *	accel bandwidth from the register 0x40 bit 4 to 6
 *
 *
 *
 *
 *  \param  u8 * v_bw_u8 : Pointer holding the value of accel bandwidth
 *	0b000 accel_us = 0 -> OSR4 mode; accel_us = 1 -> no averaging
 *	0b001 accel_us = 0 -> OSR2 mode; accel_us = 1 -> average 2 samples
 *	0b010 accel_us = 0 -> normal mode; accel_us = 1 -> average 4 samples
 *	0b011 accel_us = 0 -> CIC mode; accel_us = 1 -> average 8 samples
 *	0b100 accel_us = 0 -> Reserved; accel_us = 1 -> average 16 samples
 *	0b101 accel_us = 0 -> Reserved; accel_us = 1 -> average 32 samples
 *	0b110 accel_us = 0 -> Reserved; accel_us = 1 -> average 64 samples
 *	0b111 accel_us = 0 -> Reserved; accel_us = 1 -> average 128 samples
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ***********************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_bw(u8 *v_bw_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_ACCEL_CONFIG_ACCEL_BW__REG, &v_data_u8, 1);
			*v_bw_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_ACCEL_CONFIG_ACCEL_BW);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API is used to set the
 *	accel bandwidth from the register 0x40 bit 4 to 6
 *
 *
 *
 *
 *  \param u8 v_bw_u8 : the value of accel bandwidth
 *	Value Description
 *	0b000 accel_us = 0 -> OSR4 mode; accel_us = 1 -> no averaging
 *	0b001 accel_us = 0 -> OSR2 mode; accel_us = 1 -> average 2 samples
 *	0b010 accel_us = 0 -> normal mode; accel_us = 1 -> average 4 samples
 *	0b011 accel_us = 0 -> CIC mode; accel_us = 1 -> average 8 samples
 *	0b100 accel_us = 0 -> Reserved; accel_us = 1 -> average 16 samples
 *	0b101 accel_us = 0 -> Reserved; accel_us = 1 -> average 32 samples
 *	0b110 accel_us = 0 -> Reserved; accel_us = 1 -> average 64 samples
 *	0b111 accel_us = 0 -> Reserved; accel_us = 1 -> average 128 samples
 *
 *
 *
 *
 *
 *  \return results of communication routine communication results
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_bw(u8 v_bw_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	BMI160_RETURN_FUNCTION_TYPE v_data_bw_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_bw_u8 < C_BMI160_EIGHT_U8X) {
			switch (v_bw_u8) {
			case BMI160_ACCEL_OSR4_AVG1:
				v_data_bw_u8 = BMI160_ACCEL_OSR4_AVG1;
				break;
			case BMI160_ACCEL_OSR2_AVG2:
				v_data_bw_u8 = BMI160_ACCEL_OSR2_AVG2;
				break;
			case BMI160_ACCEL_NORMAL_AVG4:
				v_data_bw_u8 = BMI160_ACCEL_NORMAL_AVG4;
				break;
			case BMI160_ACCEL_CIC_AVG8:
				v_data_bw_u8 = BMI160_ACCEL_CIC_AVG8;
				break;
			case BMI160_ACCEL_RES_AVG16:
				v_data_bw_u8 = BMI160_ACCEL_RES_AVG16;
				break;
			case BMI160_ACCEL_RES_AVG32:
				v_data_bw_u8 = BMI160_ACCEL_RES_AVG32;
				break;
			case BMI160_ACCEL_RES_AVG64:
				v_data_bw_u8 = BMI160_ACCEL_RES_AVG64;
				break;
			case BMI160_ACCEL_RES_AVG128:
				v_data_bw_u8 = BMI160_ACCEL_RES_AVG128;
				break;
			default:
				break;
			}
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_ACCEL_CONFIG_ACCEL_BW__REG, &v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_ACCEL_CONFIG_ACCEL_BW,
				v_data_bw_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_ACCEL_CONFIG_ACCEL_BW__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*************************************************************************
 *	Description: *//**brief This API is used to get the accel
 *	under sampling parameter form the register 0x40 bit 7
 *
 *
 *
 *
 *	\param  u8 * v_accel_under_sampling_u8 : pointer holding the
 *						value of accel under sampling
 *	0	-	enable
 *	1	-	disable
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ***********************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_under_sampling_parameter(
u8 *v_accel_under_sampling_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_ACCEL_CONFIG_ACCEL_UNDER_SAMPLING__REG,
			&v_data_u8, 1);
			*v_accel_under_sampling_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_ACCEL_CONFIG_ACCEL_UNDER_SAMPLING);
		}
	return com_rslt;
}

 /*************************************************************************
 *	Description: *//**brief This API is used to set the accel
 *	under sampling parameter form the register 0x40 bit 7
 *
 *
 *
 *	\param v_accel_under_sampling_u8 : the value of accel under sampling
 *	0	-	enable
 *	1	-	disable
 *
 *
 *  \return results of communication routine
 *
 *
 ***********************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_under_sampling_parameter(
u8 v_accel_under_sampling_u8)
{
BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
u8 v_data_u8 = C_BMI160_ZERO_U8X;
if (p_bmi160 == BMI160_NULL) {
	return E_BMI160_NULL_PTR;
	} else {
	if (v_accel_under_sampling_u8 < C_BMI160_EIGHT_U8X) {
		com_rslt =
		p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
		BMI160_USER_ACCEL_CONFIG_ACCEL_UNDER_SAMPLING__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_ACCEL_CONFIG_ACCEL_UNDER_SAMPLING,
			v_accel_under_sampling_u8);
			com_rslt +=
			p_bmi160->BMI160_BUS_WRITE_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_ACCEL_CONFIG_ACCEL_UNDER_SAMPLING__REG,
			&v_data_u8, 1);
		}
	} else {
	com_rslt = E_BMI160_OUT_OF_RANGE;
	}
}
return com_rslt;
}
/*************************************************************************
 *	Description: *//**brief This API is used to get the ranges
 *	(g values) of the accel from the register 0x41 bit 0 to 3
 *
 *
 *
 *
 *  \param u8 * v_range_u8 : Pointer holding the accel v_range_u8
 *	3 -> BMI160_ACCEL_RANGE_2G
 *	5 -> BMI160_ACCEL_RANGE_4G
 *	8 -> BMI160_ACCEL_RANGE_8G
 *	12 -> BMI160_ACCEL_RANGE_16G
 *
 *
 *
 *  \return results of communication routine
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_range(
u8 *v_range_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_ACCEL_RANGE__REG, &v_data_u8, 1);
			*v_range_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_ACCEL_RANGE);
		}
	return com_rslt;
}
 /**********************************************************************
 *	Description: *//**brief This API is used to set the v_range
 *	(g values) of the accel from the register 0x41 bit 0 to 3
 *
 *
 *
 *
 *  \param u8 * Range : The value of accel  range
 *	3 -> BMI160_ACCEL_RANGE_2G
 *	5 -> BMI160_ACCEL_RANGE_4G
 *	8 -> BMI160_ACCEL_RANGE_8G
 *	12 -> BMI160_ACCEL_RANGE_16G
 *
 *  \return results of communication routine
 *
 *
 **********************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_range(u8 v_range_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if ((v_range_u8 == C_BMI160_THREE_U8X) ||
			(v_range_u8 == C_BMI160_FIVE_U8X) ||
			(v_range_u8 == C_BMI160_EIGHT_U8X) ||
			(v_range_u8 == C_BMI160_TWELVE_U8X)) {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_ACCEL_RANGE__REG, &v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				switch (v_range_u8) {
				case BMI160_ACCEL_RANGE_2G:
					v_data_u8  = BMI160_SET_BITSLICE(
					v_data_u8,
					BMI160_USER_ACCEL_RANGE,
					C_BMI160_THREE_U8X);
					break;
				case BMI160_ACCEL_RANGE_4G:
					v_data_u8  = BMI160_SET_BITSLICE(
					v_data_u8,
					BMI160_USER_ACCEL_RANGE,
					C_BMI160_FIVE_U8X);
					break;
				case BMI160_ACCEL_RANGE_8G:
					v_data_u8  = BMI160_SET_BITSLICE(
					v_data_u8,
					BMI160_USER_ACCEL_RANGE,
					C_BMI160_EIGHT_U8X);
					break;
				case BMI160_ACCEL_RANGE_16G:
					v_data_u8  = BMI160_SET_BITSLICE(
					v_data_u8,
					BMI160_USER_ACCEL_RANGE,
					C_BMI160_TWELVE_U8X);
					break;
				default:
					break;
				}
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_ACCEL_RANGE__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/**************************************************************************
 *	Description: *//**brief This API is used to get the
 *	gyroscope output data rate from the register 0x42 bit 0 to 3
 *
 *
 *
 *
 *  \param  u8 * v_output_data_rate_u8 :
 *	Pointer holding the value of output data rate
 *	gyro_OUTPUT_DATA_RATE	->
 *	0b0000 - BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *	0b0001 - BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *	0b0010 - BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *	0b0011 - BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *	0b0100 - BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *	0b0101 - BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *	0b0110 - BMI160_GYRO_OUTPUT_DATA_RATE_25HZ
 *	0b0111 - BMI160_GYRO_OUTPUT_DATA_RATE_50HZ
 *	0b1000 - BMI160_GYRO_OUTPUT_DATA_RATE_100HZ
 *	0b1001 - BMI160_GYRO_OUTPUT_DATA_RATE_200HZ
 *	0b1010 - BMI160_GYRO_OUTPUT_DATA_RATE_400HZ
 *	0b1011 - BMI160_GYRO_OUTPUT_DATA_RATE_800HZ
 *	0b1100 - BMI160_GYRO_OUTPUT_DATA_RATE_1600HZ
 *	0b1101 - BMI160_GYRO_OUTPUT_DATA_RATE_3200HZ
 *	0b1110 - BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *	0b1111 - BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *
 *  \return results of communication routine
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_output_data_rate(
u8 *v_output_data_rate_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_GYRO_CONFIG_OUTPUT_DATA_RATE__REG,
			&v_data_u8, 1);
			*v_output_data_rate_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_GYRO_CONFIG_OUTPUT_DATA_RATE);
		}
	return com_rslt;
}
/************************************************************************
 * Description: *//**brief This API is used to set the
 *	gyroscope output data rate from the register 0x42 bit 0 to 3
 *
 *
 *
 *  \param  u8 v_output_data_rate_u8 :
 *	The value of gyro output data rate
 *
 *	0b0000 - BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *	0b0001 - BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *	0b0010 - BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *	0b0011 - BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *	0b0100 - BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *	0b0101 - BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *	0b0110 - BMI160_GYRO_OUTPUT_DATA_RATE_25HZ
 *	0b0111 - BMI160_GYRO_OUTPUT_DATA_RATE_50HZ
 *	0b1000 - BMI160_GYRO_OUTPUT_DATA_RATE_100HZ
 *	0b1001 - BMI160_GYRO_OUTPUT_DATA_RATE_200HZ
 *	0b1010 - BMI160_GYRO_OUTPUT_DATA_RATE_400HZ
 *	0b1011 - BMI160_GYRO_OUTPUT_DATA_RATE_800HZ
 *	0b1100 - BMI160_GYRO_OUTPUT_DATA_RATE_1600HZ
 *	0b1101 - BMI160_GYRO_OUTPUT_DATA_RATE_3200HZ
 *	0b1110 - BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *	0b1111 - BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *
 *
 *
 *  \return results of communication routine
 *
 *
 *************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***********************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_output_data_rate(
u8 v_output_data_rate_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	u8 v_data_output_datarate_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_output_data_rate_u8 < C_BMI160_FOURTEEN_U8X) {
			switch (v_output_data_rate_u8) {
			case BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED:
				v_data_output_datarate_u8 =
				BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED;
				break;
			case BMI160_GYRO_OUTPUT_DATA_RATE_25HZ:
				v_data_output_datarate_u8 =
				BMI160_GYRO_OUTPUT_DATA_RATE_25HZ;
				break;
			case BMI160_GYRO_OUTPUT_DATA_RATE_50HZ:
				v_data_output_datarate_u8 =
				BMI160_GYRO_OUTPUT_DATA_RATE_50HZ;
				break;
			case BMI160_GYRO_OUTPUT_DATA_RATE_100HZ:
				v_data_output_datarate_u8 =
				BMI160_GYRO_OUTPUT_DATA_RATE_100HZ;
				break;
			case BMI160_GYRO_OUTPUT_DATA_RATE_200HZ:
				v_data_output_datarate_u8 =
				BMI160_GYRO_OUTPUT_DATA_RATE_200HZ;
				break;
			case BMI160_GYRO_OUTPUT_DATA_RATE_400HZ:
				v_data_output_datarate_u8 =
				BMI160_GYRO_OUTPUT_DATA_RATE_400HZ;
				break;
			case BMI160_GYRO_OUTPUT_DATA_RATE_800HZ:
				v_data_output_datarate_u8 =
				BMI160_GYRO_OUTPUT_DATA_RATE_800HZ;
				break;
			case BMI160_GYRO_OUTPUT_DATA_RATE_1600HZ:
				v_data_output_datarate_u8 =
				BMI160_GYRO_OUTPUT_DATA_RATE_1600HZ;
				break;
			case BMI160_GYRO_OUTPUT_DATA_RATE_3200HZ:
				v_data_output_datarate_u8 =
				BMI160_GYRO_OUTPUT_DATA_RATE_3200HZ;
				break;
			default:
				break;
			}
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_GYRO_CONFIG_OUTPUT_DATA_RATE__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_GYRO_CONFIG_OUTPUT_DATA_RATE,
				v_data_output_datarate_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC
				(p_bmi160->dev_addr,
				BMI160_USER_GYRO_CONFIG_OUTPUT_DATA_RATE__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/***********************************************************************
 *	Description: *//**brief This API is used to get the
 *	v_data_bw_u8 of gyro from the register 0x42 bit 4 to 5
 *
 *
 *
 *
 *  \param  u8 * v_bw_u8 : Pointer holding the value of gyro v_data_bw_u8
 *
 *	Value	Description
 *	00 -	BMI160_GYRO_OSR4_MODE
 *	01 -	BMI160_GYRO_OSR2_MODE
 *	10 -	BMI160_GYRO_NORMAL_MODE
 *	11 -	BMI160_GYRO_CIC_MODE
 *
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ***********************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_bw(u8 *v_bw_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_GYRO_CONFIG_BW__REG, &v_data_u8, 1);
			*v_bw_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_GYRO_CONFIG_BW);
		}
	return com_rslt;
}
/**************************************************************************
 * Description: *//**brief This API is used to set the
 *	v_data_bw_u8 of gyro from the register 0x42 bit 4 to 5
 *
 *
 *
 *
 *  \param  u8 v_bw_u8 : the value of gyro v_data_bw_u8
 *
 *	Value	Description
 *	00 -	BMI160_GYRO_OSR4_MODE
 *	01 -	BMI160_GYRO_OSR2_MODE
 *	10 -	BMI160_GYRO_NORMAL_MODE
 *	11 -	BMI160_GYRO_CIC_MODE
 *
 *
 *
 *
 *  \return results of communication routine communication results
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_bw(u8 v_bw_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	u8 v_data_bw_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_bw_u8 < C_BMI160_FOUR_U8X) {
			switch (v_bw_u8) {
			case BMI160_GYRO_OSR4_MODE:
				v_data_bw_u8 = BMI160_GYRO_OSR4_MODE;
				break;
			case BMI160_GYRO_OSR2_MODE:
				v_data_bw_u8 = BMI160_GYRO_OSR2_MODE;
				break;
			case BMI160_GYRO_NORMAL_MODE:
				v_data_bw_u8 = BMI160_GYRO_NORMAL_MODE;
				break;
			case BMI160_GYRO_CIC_MODE:
				v_data_bw_u8 = BMI160_GYRO_CIC_MODE;
				break;
			default:
				break;
			}
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_GYRO_CONFIG_BW__REG, &v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_GYRO_CONFIG_BW, v_data_bw_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_GYRO_CONFIG_BW__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads the v_range_u8 from
 *	of gyro from the register 0x43 bit 0 to 2
 *
 *  \param u8 *v_range_u8 : Pointer holding the value of gyro v_range_u8
 *	Range[0....7]
 *	0 - 2000/s
 *	1 - 1000/s
 *	2 - 500/s
 *	3 - 250/s
 *	4 - 125/s
 *
 *
 *  \return results of communication routine
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_range(u8 *v_range_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_GYRO_RANGE__REG, &v_data_u8, 1);
			*v_range_u8 =
			BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_GYRO_RANGE);
		}
	return com_rslt;
}
/****************************************************************************
 * Description: *//**brief This API sets the v_range_u8 from
 *	of gyro from the register 0x43 bit 0 to 2
 *
 *  \param u8 v_range_u8 : The value of gyro v_range_u8
 *	Range[0....7]
 *	0 - 2000/s
 *	1 - 1000/s
 *	2 - 500/s
 *	3 - 250/s
 *	4 - 125/s
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_range(u8 v_range_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_range_u8 < C_BMI160_FIVE_U8X) {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_GYRO_RANGE__REG, &v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_GYRO_RANGE,
				v_range_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC
				(p_bmi160->dev_addr,
				BMI160_USER_GYRO_RANGE__REG, &v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/***************************************************************************
 *	Description: *//**brief This API is used to get the
 *	output data rate of magnetometer from the register 0x44 bit 0 to 3
 *
 *
 *
 *
 *  \param  u8 * v_output_data_rat_u8e :
 *	The pointer holding the value
 *                       0 -> Reserved
 *                       1 -> 0.78HZ
 *                       2 -> 1.56HZ
 *                       3 -> 3.12HZ
 *                       4 -> 6.25HZ
 *                       5 -> 12.5HZ
 *                       6 -> 25HZ
 *                       7 -> 50HZ
 *                       8 -> 100HZ
 *                       9 -> 200HZ
 *                       10 -> 400HZ
  *                      11->    800Hz
 *                       12->    1600hz
 *                       13->    Reserved
 *                       14->    Reserved
 *                       15->    Reserved
 *
 *
 *  \return results of communication routine
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_mag_output_data_rate(
u8 *v_output_data_rat_u8e)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_MAG_CONFIG_OUTPUT_DATA_RATE__REG,
			&v_data_u8, 1);
			*v_output_data_rat_u8e = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_MAG_CONFIG_OUTPUT_DATA_RATE);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API is used to set the value of mag v_data_bw_u8
 *	from the register 0x44 bit 0 to 3
 *
 *
 *
 *
 *  \param u8 v_output_data_rat_u8e : The value of mag v_data_bw_u8
 *                       0 -> Reserved
 *                       1 -> 0.78HZ
 *                       2 -> 1.56HZ
 *                       3 -> 3.12HZ
 *                       4 -> 6.25HZ
 *                       5 -> 12.5HZ
 *                       6 -> 25HZ
 *                       7 -> 50HZ
 *                       8 -> 100HZ
 *                       9 -> 200HZ
 *                       10-> 400HZ
  *                      11-> 800Hz
 *                       12-> 1600hz
 *                       13-> Reserved
 *                       14-> Reserved
 *                       15-> Reserved
 *
 *
 *
 *
 *  \return results of communication routine communication results
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_mag_output_data_rate(
u8 v_output_data_rat_u8e)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	u8 v_data_bw_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_output_data_rat_u8e < C_BMI160_SIXTEEN_U8X) {
			switch (v_output_data_rat_u8e) {
			case BMI160_MAG_OUTPUT_DATA_RATE_RESERVED:
				v_data_bw_u8 =
				BMI160_MAG_OUTPUT_DATA_RATE_RESERVED;
				break;
			case BMI160_MAG_OUTPUT_DATA_RATE_0_78HZ:
				v_data_bw_u8 =
				BMI160_MAG_OUTPUT_DATA_RATE_0_78HZ;
				break;
			case BMI160_MAG_OUTPUT_DATA_RATE_1_56HZ:
				v_data_bw_u8 =
				BMI160_MAG_OUTPUT_DATA_RATE_1_56HZ;
				break;
			case BMI160_MAG_OUTPUT_DATA_RATE_3_12HZ:
				v_data_bw_u8 =
				BMI160_MAG_OUTPUT_DATA_RATE_3_12HZ;
				break;
			case BMI160_MAG_OUTPUT_DATA_RATE_6_25HZ:
				v_data_bw_u8 =
				BMI160_MAG_OUTPUT_DATA_RATE_6_25HZ;
				break;
			case BMI160_MAG_OUTPUT_DATA_RATE_12_5HZ:
				v_data_bw_u8 =
				BMI160_MAG_OUTPUT_DATA_RATE_12_5HZ;
				break;
			case BMI160_MAG_OUTPUT_DATA_RATE_25HZ:
				v_data_bw_u8 =
				BMI160_MAG_OUTPUT_DATA_RATE_25HZ;
				break;
			case BMI160_MAG_OUTPUT_DATA_RATE_50HZ:
				v_data_bw_u8 =
				BMI160_MAG_OUTPUT_DATA_RATE_50HZ;
				break;
			case BMI160_MAG_OUTPUT_DATA_RATE_100HZ:
				v_data_bw_u8 =
				BMI160_MAG_OUTPUT_DATA_RATE_100HZ;
				break;
			case BMI160_MAG_OUTPUT_DATA_RATE_200HZ:
				v_data_bw_u8 =
				BMI160_MAG_OUTPUT_DATA_RATE_200HZ;
				break;
			case BMI160_MAG_OUTPUT_DATA_RATE_400HZ:
				v_data_bw_u8 =
				BMI160_MAG_OUTPUT_DATA_RATE_400HZ;
				break;
			case BMI160_MAG_OUTPUT_DATA_RATE_800HZ:
				v_data_bw_u8 =
				BMI160_MAG_OUTPUT_DATA_RATE_800HZ;
				break;
			case BMI160_MAG_OUTPUT_DATA_RATE_1600HZ:
				v_data_bw_u8 =
				BMI160_MAG_OUTPUT_DATA_RATE_1600HZ;
				break;
			case BMI160_MAG_OUTPUT_DATA_RATE_RESERVED0:
				v_data_bw_u8 =
				BMI160_MAG_OUTPUT_DATA_RATE_RESERVED0;
				break;
			case BMI160_MAG_OUTPUT_DATA_RATE_RESERVED1:
				v_data_bw_u8 =
				BMI160_MAG_OUTPUT_DATA_RATE_RESERVED1;
				break;
			case BMI160_MAG_OUTPUT_DATA_RATE_RESERVED2:
				v_data_bw_u8 =
				BMI160_MAG_OUTPUT_DATA_RATE_RESERVED2;
				break;
			default:
				break;
			}
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_MAG_CONFIG_OUTPUT_DATA_RATE__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_MAG_CONFIG_OUTPUT_DATA_RATE,
				v_data_bw_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_MAG_CONFIG_OUTPUT_DATA_RATE__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*******************************************************************************
 *	Description: *//**brief This API is used to read Down sampling
 *	for gyro (2**downs_gyro) in the register 0x45 bit 0 to 2
 *
 *
 *
 *
 *  \param u8 * v_fifo_down_gyro_u8 :Pointer to the fifo_down gyro
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_fifo_down_gyro(
u8 *v_fifo_down_gyro_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_FIFO_DOWN_GYRO__REG, &v_data_u8, 1);
			*v_fifo_down_gyro_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_FIFO_DOWN_GYRO);
		}
	return com_rslt;
}
/*******************************************************************************
 *	Description: *//**brief This API is used to set Down sampling
 *	for gyro (2**downs_gyro) from the register 0x45 bit 0 to 5
 *
 *
 *
 *
 *  \param u8 v_fifo_down_gyro_u8 : Value of the fifo_down_gyro
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_fifo_down_gyro(
u8 v_fifo_down_gyro_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_FIFO_DOWN_GYRO__REG, &v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(
				v_data_u8,
				BMI160_USER_FIFO_DOWN_GYRO,
				v_fifo_down_gyro_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_FIFO_DOWN_GYRO__REG,
				&v_data_u8, 1);
			}
		}
	return com_rslt;
}
/*******************************************************************************
 *	Description: *//**brief This API is used to read gyro_fifo_filt data
 *	from the register 0x45 bit 3
 *
 *
 *
 *  \param u8 * v_gyro_fifo_filter_data_u8 :Pointer to the gyro_fifo_filtdata
 *	v_gyro_fifo_filter_data_u8 ->
 *	0	-	Unfiltered data
 *	1	-	Filtered data
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_fifo_filter_data(
u8 *v_gyro_fifo_filter_data_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_FIFO_FILTER_GYRO__REG, &v_data_u8, 1);
			*v_gyro_fifo_filter_data_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_FIFO_FILTER_GYRO);
		}
	return com_rslt;
}
/*******************************************************************************
 *	Description: *//**brief This API is used to write filtered or unfiltered
 *	from the register 0x45 bit 3
 *
 *
 *
 *  \param u8 v_gyro_fifo_filter_data_u8 :
 *	The value ofS gyro_fifo_filter_data
 *	v_gyro_fifo_filter_data_u8 ->
 *	0	-	Unfiltered data
 *	1	-	Filtered data
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_fifo_filter_data(
u8 v_gyro_fifo_filter_data_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_gyro_fifo_filter_data_u8 < C_BMI160_TWO_U8X) {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_FIFO_FILTER_GYRO__REG, &v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(
				v_data_u8,
				BMI160_USER_FIFO_FILTER_GYRO,
				v_gyro_fifo_filter_data_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_FIFO_FILTER_GYRO__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
 /*****************************************************************************
 *	Description: *//**brief This API is used to read Down sampling
 *	for accel (2**downs_accel) from the register 0x45 bit 4 to 6
 *
 *
 *
 *
 *  \param u8 * v_fifo_down_u8 :Pointer to the fifo_downs accel
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_fifo_down_accel(
u8 *v_fifo_down_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_FIFO_DOWN_ACCEL__REG, &v_data_u8, 1);
			*v_fifo_down_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_FIFO_DOWN_ACCEL);
		}
	return com_rslt;
}
/*******************************************************************************
 *	Description: *//**brief This API is used to write Downsampling
 *	for accel (2**downs_accel) from the register 0x45 bit 4 to 6
 *
 *
 *
 *
 *  \param u8 v_fifo_down_u8 : Value of the v_fifo_down_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_fifo_down_accel(
u8 v_fifo_down_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_FIFO_DOWN_ACCEL__REG, &v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_FIFO_DOWN_ACCEL, v_fifo_down_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_FIFO_DOWN_ACCEL__REG,
				&v_data_u8, 1);
			}
		}
	return com_rslt;
}
/*******************************************************************************
 *	Description: *//**brief This API is used to read
 *	accel_fifo_filt data
 *	from the register 0x45 bit 7
 *
 *
 *
 *  \param u8 * v_accel_fifo_filter_u8 :
 *	Pointer to the accel_fifo_filt data
 *	v_accel_fifo_filter_u8 ->
 *	0	-	Unfiltered data
 *	1	-	Filtered data
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_fifo_filter_data(
u8 *v_accel_fifo_filter_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_FIFO_FILTER_ACCEL__REG, &v_data_u8, 1);
			*v_accel_fifo_filter_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_FIFO_FILTER_ACCEL);
		}
	return com_rslt;
}
/*******************************************************************************
 *	Description: *//**brief  API is used to write filtered or unfiltered
 *	from the register 0x45 bit 7
 *
 *
 *
 *  \param u8 v_accel_fifo_filter_u8 : The value of v_accel_fifo_filter_u8
 *	v_accel_fifo_filter_u8 ->
 *	0	-	Unfiltered data
 *	1	-	Filtered data
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_fifo_filter_data(
u8 v_accel_fifo_filter_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_accel_fifo_filter_u8 < C_BMI160_TWO_U8X) {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_FIFO_FILTER_ACCEL__REG, &v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_FIFO_FILTER_ACCEL,
				v_accel_fifo_filter_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_FIFO_FILTER_ACCEL__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*******************************************************************************
 *	Description: *//**brief This API is used to Trigger an interrupt
 *	when FIFO contains fifo_water_mark from the register 0x46 bit 0 to 7
 *
 *
 *
 *  \param u8 v_fifo_wm_u8 : Pointer to fifo watermark
 *
 *
 *
 *  \return results of communication
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_fifo_wm(
u8 *v_fifo_wm_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_FIFO_WM__REG,
			&v_data_u8, 1);
			*v_fifo_wm_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_FIFO_WM);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API is used to Trigger an interrupt
 *	when FIFO contains fifo_water_mark*4 bytes form the resister 0x46
 *
 *
 *
 *
 *  \param u8 v_fifo_wm_u8 : Value of the fifo watermark
 *	0	-	do not stop writing to FIFO when full
 *	1	-	Stop writing into FIFO when full.
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_fifo_wm(
u8 v_fifo_wm_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->dev_addr,
			BMI160_USER_FIFO_WM__REG,
			&v_fifo_wm_u8, 1);
		}
	return com_rslt;
}
 /****************************************************************************
 * Description: *//**brief This API Reads the fifo_stop_on full in
 * FIFO_CONFIG_1
 * Stop writing samples into FIFO when FIFO is full
 * from the register 0x47 bit 0
 *
 *
 *
 *  \param u8 * v_fifo_stop_on_full_u8 :
 *	Pointer to the fifo_stop_on full
 *	v_fifo_stop_on_full_u8 ->
 *	0	-	do not stop writing to FIFO when full
 *	1	-	Stop writing into FIFO when full.
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_fifo_stop_on_full(
u8 *v_fifo_stop_on_full_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_FIFO_STOP_ON_FULL__REG, &v_data_u8, 1);
			*v_fifo_stop_on_full_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_FIFO_STOP_ON_FULL);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**brief  This API writes the fifo_stop_on full in
 * FIFO_CONFIG_1
 * Stop writing samples into FIFO when FIFO is full
 * from the register 0x47 bit 0
 *
 *  \param u8 v_fifo_stop_on_full_u8 :
 *	Value of the fifo_stop_on full
 *	v_fifo_stop_on_full_u8
 *  0	-	 do not stop writing to FIFO when full
 *  1	-	 Stop writing into FIFO when full.
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_fifo_stop_on_full(
u8 v_fifo_stop_on_full_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_fifo_stop_on_full_u8 < C_BMI160_TWO_U8X) {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_FIFO_STOP_ON_FULL__REG, &v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 =
				BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_FIFO_STOP_ON_FULL,
				v_fifo_stop_on_full_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC
				(p_bmi160->dev_addr,
				BMI160_USER_FIFO_STOP_ON_FULL__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/******************************************************************************
 *	Description: *//**brief This API Reads fifo sensor time
 *	frame after the last valid data frame form the register  0x47 bit 1
 *
 *
 *
 *
 *  \param u8 * v_fifo_time_enable_u8 :Pointer to the fifo_time enable
 *	v_fifo_time_enable_u8
 *	0	-  do not return sensortime frame
 *	1	-  return sensortime frame
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_fifo_time_enable(
u8 *v_fifo_time_enable_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_FIFO_TIME_ENABLE__REG, &v_data_u8, 1);
			*v_fifo_time_enable_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_FIFO_TIME_ENABLE);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 *	Description: *//**brief This API writes sensortime
 *	frame after the last valid data frame form the register  0x47 bit 1
 *
 *
 *
 *
 *  \param u8 v_fifo_time_enable_u8 : The value of fifo_time enable
 *	v_fifo_time_enable_u8
 *	0	-  do not return sensortime frame
 *	1	-  return sensortime frame
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_fifo_time_enable(
u8 v_fifo_time_enable_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_fifo_time_enable_u8 < C_BMI160_TWO_U8X) {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_FIFO_TIME_ENABLE__REG, &v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_FIFO_TIME_ENABLE,
				v_fifo_time_enable_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_FIFO_TIME_ENABLE__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 *	Description: *//**brief This API Reads FIFO interrupt 2
 *	tag enable from the resister 0x47 bit 2
 *
 *  \param u8 * v_fifo_tag_intr2_u8 :
 *                 Pointer to the fifo_tag_intr2
 *	v_fifo_tag_intr2_u8 ->
 *                 0    disable tag
 *                 1    enable tag
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_fifo_tag_intr2_enable(
u8 *v_fifo_tag_intr2_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_FIFO_TAG_INTR2_ENABLE__REG, &v_data_u8, 1);
			*v_fifo_tag_intr2_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_FIFO_TAG_INTR2_ENABLE);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 *	Description: *//**brief This API writes FIFO interrupt 2
 *	tag enable from the resister 0x47 bit 2
 *
 *  \param u8 v_fifo_tag_intr2_u8 :
 *               Value of the v_fifo_tag_intr2_u8
 *	v_fifo_tag_intr2_u8 ->
 *                 0    disable tag
 *                 1    enable tag
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_fifo_tag_intr2_enable(
u8 v_fifo_tag_intr2_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_fifo_tag_intr2_u8 < C_BMI160_TWO_U8X) {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_FIFO_TAG_INTR2_ENABLE__REG, &v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_FIFO_TAG_INTR2_ENABLE,
				v_fifo_tag_intr2_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_FIFO_TAG_INTR2_ENABLE__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 *	Description: *//**brief This API Reads FIFO interrupt 1
 *	tag enable from the resister 0x47 bit 3
 *
 *  \param u8 * v_fifo_tag_intr1_u8 :
 *                 Pointer to the fifo_tag_intr1
 *	v_fifo_tag_intr1_u8 ->
 *	0    disable tag
 *	1    enable tag
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_fifo_tag_intr1_enable(
u8 *v_fifo_tag_intr1_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_FIFO_TAG_INTR1_ENABLE__REG, &v_data_u8, 1);
			*v_fifo_tag_intr1_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_FIFO_TAG_INTR1_ENABLE);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief This API writes FIFO interrupt 1
 *	tag enable from the resister 0x47 bit 3
 *
 *  \param u8 * v_fifo_tag_intr1_u8 :
 *                 The value of  fifo_tag_intr1
 *	v_fifo_tag_intr1_u8 ->
 *	0    disable tag
 *	1    enable tag
 *
 *
 *
 *  \return results of communication routine
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_fifo_tag_intr1_enable(
u8 v_fifo_tag_intr1_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_fifo_tag_intr1_u8 < C_BMI160_TWO_U8X) {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_FIFO_TAG_INTR1_ENABLE__REG, &v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_FIFO_TAG_INTR1_ENABLE,
				v_fifo_tag_intr1_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_FIFO_TAG_INTR1_ENABLE__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 *	Description: *//**brief This API Reads FIFO frame
 *	header enable from the register 0x47 bit 4
 *
 *  \param u8 * v_fifo_header_u8 :
 *                 Pointer to the fifo_header
 *	v_fifo_header_u8 ->
 *	0    no header is stored
 *	1    header is stored
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_fifo_header_enable(
u8 *v_fifo_header_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_FIFO_HEADER_ENABLE__REG, &v_data_u8, 1);
			*v_fifo_header_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_FIFO_HEADER_ENABLE);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 *	Description: *//**brief This API writes FIFO frame
 *	header enable from the register 0x47 bit 4
 *
 *
 *  \param u8 v_fifo_header_u8 :
 *               Value of the fifo_header
 *	v_fifo_header_u8 ->
 *	0    no header is stored
 *	1    header is stored
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_fifo_header_enable(
u8 v_fifo_header_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_fifo_header_u8 < C_BMI160_TWO_U8X) {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_FIFO_HEADER_ENABLE__REG, &v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_FIFO_HEADER_ENABLE,
				v_fifo_header_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_FIFO_HEADER_ENABLE__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 *	Description: *//**brief This API is used to read stored
 *	magnetometer data in FIFO (all 3 axes) from the register 0x47 bit 5
 *
 *  \param u8 * v_fifo_mag_u8 :
 *                 Pointer to the fifo_mag
 *	v_fifo_mag_u8 ->
 *	0   no magnetometer data is stored
 *	1   magnetometer data is stored
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_fifo_mag_enable(
u8 *v_fifo_mag_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_FIFO_MAG_ENABLE__REG, &v_data_u8, 1);
			*v_fifo_mag_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_FIFO_MAG_ENABLE);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 *	Description: *//**brief This API is used to write stored
 *	magnetometer data in FIFO (all 3 axes) from the register 0x47 bit 5
 *
 *  \param u8 v_fifo_mag_u8 :
 *                 The value of v_fifo_mag_u8
 *	v_fifo_mag_u8 ->
 *	0   no magnetometer data is stored
 *	1   magnetometer data is stored
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_fifo_mag_enable(
u8 v_fifo_mag_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			if (v_fifo_mag_u8 < C_BMI160_TWO_U8X) {
				com_rslt =
				p_bmi160->BMI160_BUS_READ_FUNC
				(p_bmi160->dev_addr,
				BMI160_USER_FIFO_MAG_ENABLE__REG,
				&v_data_u8, 1);
				if (com_rslt == SUCCESS) {
					v_data_u8 =
					BMI160_SET_BITSLICE(v_data_u8,
					BMI160_USER_FIFO_MAG_ENABLE,
					v_fifo_mag_u8);
					com_rslt +=
					p_bmi160->BMI160_BUS_WRITE_FUNC
					(p_bmi160->dev_addr,
					BMI160_USER_FIFO_MAG_ENABLE__REG,
					&v_data_u8, 1);
				}
			} else {
			com_rslt = E_BMI160_OUT_OF_RANGE;
			}
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 *	Description: *//**brief This API is used to read stored
 *	accel data in FIFO (all 3 axes) from the register 0x47 bit 6
 *
 *
 *  \param u8 * v_fifo_accel_u8 :
 *                 Pointer to the fifo_accel_en
 *	v_fifo_accel_u8 ->
 *	0   no accel data is stored
 *	1   accel data is stored
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_fifo_accel_enable(
u8 *v_fifo_accel_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_FIFO_ACCEL_ENABLE__REG, &v_data_u8, 1);
			*v_fifo_accel_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_FIFO_ACCEL_ENABLE);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 *	Description: *//**brief This API is used to write stored
 *	accel data in FIFO (all 3 axes) from the register 0x47 bit 6
 *
 *
 *  \param u8 * v_fifo_accel_u8 :
 *                 The value of fifo_accel
 *	v_fifo_accel_u8 ->
 *	0   no accel data is stored
 *	1   accel data is stored
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_fifo_accel_enable(
u8 v_fifo_accel_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_fifo_accel_u8 < C_BMI160_TWO_U8X) {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_FIFO_ACCEL_ENABLE__REG, &v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_FIFO_ACCEL_ENABLE, v_fifo_accel_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_FIFO_ACCEL_ENABLE__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 *	Description: *//**brief This API is used to read stored
 *	 gyro data in FIFO (all 3 axes) from the resister 0x47 bit 7
 *
 *
 *  \param u8 * v_fifo_gyro_u8 :
 *                 Pointer to the v_fifo_gyro_u8
 *	v_fifo_gyro_u8 ->
 *	0   no gyro data is stored
 *	1   gyro data is stored
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_fifo_gyro_enable(
u8 *v_fifo_gyro_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_FIFO_GYRO_ENABLE__REG, &v_data_u8, 1);
			*v_fifo_gyro_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_FIFO_GYRO_ENABLE);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 *	Description: *//**brief This API is used to write stored
 *  gyro data in FIFO (all 3 axes) from the register 0x47 bit 7
 *
 *
 *
 *  \param u8 v_fifo_gyro_u8 :
 *               Value of the v_fifo_gyro_u8
 *	v_fifo_gyro_u8 ->
 *	0   no gyro data is stored
 *	1   gyro data is stored
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_fifo_gyro_enable(
u8 v_fifo_gyro_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_fifo_gyro_u8 < C_BMI160_TWO_U8X) {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_FIFO_GYRO_ENABLE__REG, &v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_FIFO_GYRO_ENABLE, v_fifo_gyro_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_FIFO_GYRO_ENABLE__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API is used to read
 *	I2C device address of magnetometer from the register 0x4B bit 1 to 7
 *
 *
 *
 *
 *  \param u8 * v_i2c_device_addr_u8 :
 *                 Pointer to the v_i2c_device_addr_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_i2c_device_addr(
u8 *v_i2c_device_addr_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_I2C_DEVICE_ADDR__REG, &v_data_u8, 1);
			*v_i2c_device_addr_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_I2C_DEVICE_ADDR);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 *	Description: *//**brief This API is used to write
 *	I2C device address of magnetometer from the register 0x4B bit 1 to 7
 *
 *
 *
 *  \param u8 v_i2c_device_addr_u8 :
 *               Value of the v_i2c_device_addr_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_i2c_device_addr(
u8 v_i2c_device_addr_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_I2C_DEVICE_ADDR__REG, &v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_I2C_DEVICE_ADDR,
				v_i2c_device_addr_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_I2C_DEVICE_ADDR__REG,
				&v_data_u8, 1);
			}
		}
	return com_rslt;
}
 /*****************************************************************************
 *	Description: *//**brief This API is used to read
 *	Burst data length (1,2,6,8 byte) from the register 0x4C bit 0 to 1
 *
 *
 *
 *
 *  \param u8 * v_mag_burst_u8 :
 *                 Pointer to the v_mag_burst_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_mag_burst(
u8 *v_mag_burst_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_MAG_BURST__REG, &v_data_u8, 1);
			*v_mag_burst_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_MAG_BURST);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 *	Description: *//**brief This API is used to write
 *	Burst data length (1,2,6,8 byte) from the register 0x4C bit 0 to 1
 *
 *
 *
 *  \param u8 v_mag_burst_u8 :
 *               Value of the v_mag_burst_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_mag_burst(
u8 v_mag_burst_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_MAG_BURST__REG, &v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 =
				BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_MAG_BURST, v_mag_burst_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_MAG_BURST__REG, &v_data_u8, 1);
			}
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/******************************************************************************
 *	Description: *//**brief This API is used to read
 *	trigger-readout offset in units of 2.5 ms. If set to zero,
 *	the offset is maximum, i.e. after readout a trigger
 *	is issued immediately. from the register 0x4C bit 2 to 5
 *
 *
 *
 *
 *  \param u8 * v_mag_offset_u8 :
 *                 Pointer to the v_mag_offset_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_mag_offset(
u8 *v_mag_offset_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_MAG_OFFSET__REG, &v_data_u8, 1);
			*v_mag_offset_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_MAG_OFFSET);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API is used to write
 *	trigger-readout offset in units of 2.5 ms. If set to zero,
 *	the offset is maximum, i.e. after readout a trigger
 *	is issued immediately. from the register 0x4C bit 2 to 5
 *
 *
 *
 *  \param u8 v_mag_offset_u8 :
 *               Value of the v_mag_offset_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_mag_offset(
u8 v_mag_offset_u8)
{
BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
u8 v_data_u8 = C_BMI160_ZERO_U8X;
if (p_bmi160 == BMI160_NULL) {
	return E_BMI160_NULL_PTR;
	} else {
		com_rslt =
		p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
		BMI160_USER_MAG_OFFSET__REG, &v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 =
			BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_MAG_OFFSET, v_mag_offset_u8);
			com_rslt +=
			p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->dev_addr,
			BMI160_USER_MAG_OFFSET__REG, &v_data_u8, 1);
		}
	}
return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API is used to read
 *	Enable register access on MAG_IF[2] or MAG_IF[3] writes.
 *	This implies that the DATA registers are not updated with
 *	magnetometer values. Accessing magnetometer requires
 *	the magnetometer in normal mode in PMU_STATUS.
 *	from the register 0c4C bit 7
 *
 *
 *
 *  \param u8 * v_mag_manual_u8 :
 *                 Pointer to the v_mag_manual_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_mag_manual_enable(
u8 *v_mag_manual_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_MAG_MANUAL_ENABLE__REG, &v_data_u8, 1);
			*v_mag_manual_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_MAG_MANUAL_ENABLE);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API is used to write
 *	Enable register access on MAG_IF[2] or MAG_IF[3] writes.
 *	This implies that the DATA registers are not updated with
 *	magnetometer values. Accessing magnetometer requires
 *	the magnetometer in normal mode in PMU_STATUS.
 *	from the register 0x4C bit 7
 *
 *
 *  \param u8 v_mag_manual_u8 :
 *               Value of the v_mag_manual_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_mag_manual_enable(
u8 v_mag_manual_u8)
{
BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
u8 v_data_u8 = C_BMI160_ZERO_U8X;
if (p_bmi160 == BMI160_NULL) {
	return E_BMI160_NULL_PTR;
	} else {
		com_rslt =
		p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
		BMI160_USER_MAG_MANUAL_ENABLE__REG, &v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 =
			BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_MAG_MANUAL_ENABLE, v_mag_manual_u8);
			com_rslt +=
			p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->dev_addr,
			BMI160_USER_MAG_MANUAL_ENABLE__REG, &v_data_u8, 1);
		}
	}
return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API is used to read data
 *	magnetometer address to read from the register 0x4D bit 0 to 7
 *
 *
 *
 *
 *  \param u8 v_mag_read_addr_u8 :
 *                 Pointer holding the value of v_mag_read_addr_u8
 *
 *
 *
 *  \return results of communication
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_mag_read_addr(
u8 *v_mag_read_addr_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_READ_ADDR__REG, &v_data_u8, 1);
			*v_mag_read_addr_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_READ_ADDR);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API is used to write data
 *	magnetometer address to read from the register 0x4D bit 0 to 7
 *
 *
 *
 *
 *  \param u8 v_mag_read_addr_u8 :
 *               Value of the v_mag_read_addr_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_mag_read_addr(
u8 v_mag_read_addr_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->dev_addr,
			BMI160_USER_READ_ADDR__REG, &v_mag_read_addr_u8, 1);
		}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API is used to read
 *	magnetometer write address from the register 0x4E bit 0 to 7
 *
 *
 *
 *
 *  \param u8 v_mag_write_addr_u8
 *                 Pointer to v_mag_write_addr_u8
 *
 *
 *
 *  \return results of communication
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_mag_write_addr(
u8 *v_mag_write_addr_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_WRITE_ADDR__REG, &v_data_u8, 1);
			*v_mag_write_addr_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_WRITE_ADDR);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 *	Description: *//**brief This API is used to write
 *	magnetometer write address from the resister 0x4E bit 0 to 7
 *
 *
 *
 *
 *  \param u8 v_mag_write_addr_u8 :
 *               Value of the v_mag_write_addr_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_mag_write_addr(
u8 v_mag_write_addr_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->dev_addr,
			BMI160_USER_WRITE_ADDR__REG, &v_mag_write_addr_u8, 1);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 *	Description: *//**brief This API is used to read magnetometer write data
 *	form the resister 0x4F bit 0 to 7
 *
 *
 *
 *
 *  \param u8 v_mag_write_data_u8
 *                 Pointer to v_mag_write_data_u8
 *
 *
 *
 *  \return results of communication
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_mag_write_data(
u8 *v_mag_write_data_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_WRITE_DATA__REG, &v_data_u8, 1);
			*v_mag_write_data_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_WRITE_DATA);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API is used to write magnetometer write data
 *	form the resister 0x4F bit 0 to 7
 *
 *
 *
 *
 *  \param u8 v_mag_write_data_u8 :
 *               Value of the v_mag_write_data_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_mag_write_data(
u8 v_mag_write_data_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->dev_addr,
			BMI160_USER_WRITE_DATA__REG, &v_mag_write_data_u8, 1);
		}
	return com_rslt;
}
/*************************************************************************
 *	Description: *//**brief  This API is used to read
 *	interrupt enable from the register 0x50 bit 0 to 7
 *
 *
 *
 *
 *	\param u8 v_enable_u8,u8 *v_intr_enable_zero_u8 :
 *	The value of interrupt enable
 *	enable -->
 *	BMI160_ANY_MOTION_X_ENABLE       0
 *	BMI160_ANY_MOTION_Y_ENABLE       1
 *	BMI160_ANY_MOTION_Z_ENABLE       2
 *	BMI160_DOUBLE_TAP_ENABLE         4
 *	BMI160_SINGLE_TAP_ENABLE         5
 *	BMI160_ORIENT_ENABLE        6
 *	BMI160_FLAT_ENABLE          7
 *	v_intr_enable_zero_u8 --> 1
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_enable_0(
u8 v_enable_u8, u8 *v_intr_enable_zero_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		switch (v_enable_u8) {
		case BMI160_ANY_MOTION_X_ENABLE:
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr,
			BMI160_USER_INTR_ENABLE_0_ANY_MOTION_X_ENABLE__REG,
			&v_data_u8, 1);
			*v_intr_enable_zero_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_ENABLE_0_ANY_MOTION_X_ENABLE);
		break;
		case BMI160_ANY_MOTION_Y_ENABLE:
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr,
			BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Y_ENABLE__REG,
			&v_data_u8, 1);
			*v_intr_enable_zero_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Y_ENABLE);
		break;
		case BMI160_ANY_MOTION_Z_ENABLE:
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr,
			BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Z_ENABLE__REG,
			&v_data_u8, 1);
			*v_intr_enable_zero_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Z_ENABLE);
		break;
		case BMI160_DOUBLE_TAP_ENABLE:
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr,
			BMI160_USER_INTR_ENABLE_0_DOUBLE_TAP_ENABLE__REG,
			&v_data_u8, 1);
			*v_intr_enable_zero_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_ENABLE_0_DOUBLE_TAP_ENABLE);
		break;
		case BMI160_SINGLE_TAP_ENABLE:
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr,
			BMI160_USER_INTR_ENABLE_0_SINGLE_TAP_ENABLE__REG,
			&v_data_u8, 1);
			*v_intr_enable_zero_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_ENABLE_0_SINGLE_TAP_ENABLE);
		break;
		case BMI160_ORIENT_ENABLE:
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_ENABLE_0_ORIENT_ENABLE__REG,
			&v_data_u8, 1);
			*v_intr_enable_zero_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_ENABLE_0_ORIENT_ENABLE);
		break;
		case BMI160_FLAT_ENABLE:
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_ENABLE_0_FLAT_ENABLE__REG,
			&v_data_u8, 1);
			*v_intr_enable_zero_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_ENABLE_0_FLAT_ENABLE);
		break;
		default:
			com_rslt = E_BMI160_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 *	Description: *//**brief This API is used to write
 *	interrupt enable byte0 from the register 0x50 bit 0 to 7
 *
 *	\param u8 v_enable_u8,u8 *v_intr_enable_zero_u8 :
 *	The value of interrupt enable
 *	enable -->
 *	BMI160_ANY_MOTION_X_ENABLE       0
 *	BMI160_ANY_MOTION_Y_ENABLE       1
 *	BMI160_ANY_MOTION_Z_ENABLE       2
 *	BMI160_DOUBLE_TAP_ENABLE         4
 *	BMI160_SINGLE_TAP_ENABLE         5
 *	BMI160_ORIENT_ENABLE        6
 *	BMI160_FLAT_ENABLE          7
 *	v_intr_enable_zero_u8 --> 1
 *
 *
 *
 *  \return results of communication
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *********************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_enable_0(
u8 v_enable_u8, u8 v_intr_enable_zero_u8)
{
BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
u8 v_data_u8 = C_BMI160_ZERO_U8X;
if (p_bmi160 == BMI160_NULL) {
	return E_BMI160_NULL_PTR;
	} else {
	switch (v_enable_u8) {
	case BMI160_ANY_MOTION_X_ENABLE:
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
		dev_addr, BMI160_USER_INTR_ENABLE_0_ANY_MOTION_X_ENABLE__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_ENABLE_0_ANY_MOTION_X_ENABLE,
			v_intr_enable_zero_u8);
			com_rslt +=
			p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
			dev_addr,
			BMI160_USER_INTR_ENABLE_0_ANY_MOTION_X_ENABLE__REG,
			&v_data_u8, 1);
		}
		break;
	case BMI160_ANY_MOTION_Y_ENABLE:
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
		dev_addr, BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Y_ENABLE__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Y_ENABLE,
			v_intr_enable_zero_u8);
			com_rslt +=
			p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
			dev_addr,
			BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Y_ENABLE__REG,
			&v_data_u8, 1);
		}
		break;
	case BMI160_ANY_MOTION_Z_ENABLE:
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
		dev_addr, BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Z_ENABLE__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Z_ENABLE,
			v_intr_enable_zero_u8);
			com_rslt +=
			p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
			dev_addr,
			BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Z_ENABLE__REG,
			&v_data_u8, 1);
		}
		break;
	case BMI160_DOUBLE_TAP_ENABLE:
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
		dev_addr, BMI160_USER_INTR_ENABLE_0_DOUBLE_TAP_ENABLE__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_ENABLE_0_DOUBLE_TAP_ENABLE,
			v_intr_enable_zero_u8);
			com_rslt +=
			p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
			dev_addr,
			BMI160_USER_INTR_ENABLE_0_DOUBLE_TAP_ENABLE__REG,
			&v_data_u8, 1);
		}
		break;
	case BMI160_SINGLE_TAP_ENABLE:
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
		dev_addr, BMI160_USER_INTR_ENABLE_0_SINGLE_TAP_ENABLE__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_ENABLE_0_SINGLE_TAP_ENABLE,
			v_intr_enable_zero_u8);
			com_rslt +=
			p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
			dev_addr,
			BMI160_USER_INTR_ENABLE_0_SINGLE_TAP_ENABLE__REG,
			&v_data_u8, 1);
		}
		break;
	case BMI160_ORIENT_ENABLE:
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
		dev_addr, BMI160_USER_INTR_ENABLE_0_ORIENT_ENABLE__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_ENABLE_0_ORIENT_ENABLE,
			v_intr_enable_zero_u8);
			com_rslt +=
			p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
			dev_addr,
			BMI160_USER_INTR_ENABLE_0_ORIENT_ENABLE__REG,
			&v_data_u8, 1);
		}
		break;
	case BMI160_FLAT_ENABLE:
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
		dev_addr, BMI160_USER_INTR_ENABLE_0_FLAT_ENABLE__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_ENABLE_0_FLAT_ENABLE,
			v_intr_enable_zero_u8);
			com_rslt +=
			p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
			dev_addr,
			BMI160_USER_INTR_ENABLE_0_FLAT_ENABLE__REG,
			&v_data_u8, 1);
		}
		break;
	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
		break;
	}
}
return com_rslt;
}
/************************************************************************
 *	Description: *//**brief  This API is used to read
 *	interrupt enable byte1 from the register 0x51 bit 0 to 6
 *
 *
 *
 *
 *  \param u8 v_enable_u8,u8 *v_intr_enable_1_u8 :
 *	The value of interrupt enable
 *	enable -->
 *	BMI160_HIGH_G_X_ENABLE       0
 *	BMI160_HIGH_G_Y_ENABLE       1
 *	BMI160_HIGH_G_Z_ENABLE       2
 *	BMI160_LOW_G_ENABLE          3
 *	BMI160_DATA_RDY_ENABLE         4
 *	BMI160_FIFO_FULL_ENABLE        5
 *	BMI160_FIFO_WM_ENABLE          6
 *	v_intr_enable_1_u8 --> 1
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **********************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_enable_1(
u8 v_enable_u8, u8 *v_intr_enable_1_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		switch (v_enable_u8) {
		case BMI160_HIGH_G_X_ENABLE:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr,
			BMI160_USER_INTR_ENABLE_1_HIGH_G_X_ENABLE__REG,
			&v_data_u8, 1);
			*v_intr_enable_1_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_ENABLE_1_HIGH_G_X_ENABLE);
			break;
		case BMI160_HIGH_G_Y_ENABLE:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr,
			BMI160_USER_INTR_ENABLE_1_HIGH_G_Y_ENABLE__REG,
			&v_data_u8, 1);
			*v_intr_enable_1_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_ENABLE_1_HIGH_G_Y_ENABLE);
			break;
		case BMI160_HIGH_G_Z_ENABLE:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr,
			BMI160_USER_INTR_ENABLE_1_HIGH_G_Z_ENABLE__REG,
			&v_data_u8, 1);
			*v_intr_enable_1_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_ENABLE_1_HIGH_G_Z_ENABLE);
			break;
		case BMI160_LOW_G_ENABLE:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_ENABLE_1_LOW_G_ENABLE__REG,
			&v_data_u8, 1);
			*v_intr_enable_1_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_ENABLE_1_LOW_G_ENABLE);
			break;
		case BMI160_DATA_RDY_ENABLE:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr,
			BMI160_USER_INTR_ENABLE_1_DATA_RDY_ENABLE__REG,
			&v_data_u8, 1);
			*v_intr_enable_1_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_ENABLE_1_DATA_RDY_ENABLE);
			break;
		case BMI160_FIFO_FULL_ENABLE:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr,
			BMI160_USER_INTR_ENABLE_1_FIFO_FULL_ENABLE__REG,
			&v_data_u8, 1);
			*v_intr_enable_1_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_ENABLE_1_FIFO_FULL_ENABLE);
			break;
		case BMI160_FIFO_WM_ENABLE:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr,
			BMI160_USER_INTR_ENABLE_1_FIFO_WM_ENABLE__REG,
			&v_data_u8, 1);
			*v_intr_enable_1_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_ENABLE_1_FIFO_WM_ENABLE);
			break;
		default:
			com_rslt = E_BMI160_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/**************************************************************************
 *	Description: *//**brief This API is used to write
 *	interrupt enable byte1 from the register 0x51 bit 0 to 6
 *
 *
 *
 *  \param u8 v_enable_u8,u8 *v_intr_enable_1_u8 :
 *	The value of interrupt enable
 *	enable -->
 *	BMI160_HIGH_G_X_ENABLE       0
 *	BMI160_HIGH_G_Y_ENABLE       1
 *	BMI160_HIGH_G_Z_ENABLE       2
 *	BMI160_LOW_G_ENABLE          3
 *	BMI160_DATA_RDY_ENABLE         4
 *	BMI160_FIFO_FULL_ENABLE        5
 *	BMI160_FIFO_WM_ENABLE          6
 *	v_intr_enable_1_u8 --> 1
 *
 *
 *
 *  \return results of communication
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_enable_1(
u8 v_enable_u8, u8 v_intr_enable_1_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		switch (v_enable_u8) {
		case BMI160_HIGH_G_X_ENABLE:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr,
			BMI160_USER_INTR_ENABLE_1_HIGH_G_X_ENABLE__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_ENABLE_1_HIGH_G_X_ENABLE,
				v_intr_enable_1_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
				dev_addr,
				BMI160_USER_INTR_ENABLE_1_HIGH_G_X_ENABLE__REG,
				&v_data_u8, 1);
			}
		break;
		case BMI160_HIGH_G_Y_ENABLE:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr,
			BMI160_USER_INTR_ENABLE_1_HIGH_G_Y_ENABLE__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_ENABLE_1_HIGH_G_Y_ENABLE,
				v_intr_enable_1_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
				dev_addr,
				BMI160_USER_INTR_ENABLE_1_HIGH_G_Y_ENABLE__REG,
				&v_data_u8, 1);
			}
		break;
		case BMI160_HIGH_G_Z_ENABLE:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr,
			BMI160_USER_INTR_ENABLE_1_HIGH_G_Z_ENABLE__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_ENABLE_1_HIGH_G_Z_ENABLE,
				v_intr_enable_1_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
				dev_addr,
				BMI160_USER_INTR_ENABLE_1_HIGH_G_Z_ENABLE__REG,
				&v_data_u8, 1);
			}
		break;
		case BMI160_LOW_G_ENABLE:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr,
			BMI160_USER_INTR_ENABLE_1_LOW_G_ENABLE__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_ENABLE_1_LOW_G_ENABLE,
				v_intr_enable_1_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
				dev_addr,
				BMI160_USER_INTR_ENABLE_1_LOW_G_ENABLE__REG,
				&v_data_u8, 1);
			}
		break;
		case BMI160_DATA_RDY_ENABLE:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr,
			BMI160_USER_INTR_ENABLE_1_DATA_RDY_ENABLE__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_ENABLE_1_DATA_RDY_ENABLE,
				v_intr_enable_1_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
				dev_addr,
				BMI160_USER_INTR_ENABLE_1_DATA_RDY_ENABLE__REG,
				&v_data_u8, 1);
			}
		break;
		case BMI160_FIFO_FULL_ENABLE:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr,
			BMI160_USER_INTR_ENABLE_1_FIFO_FULL_ENABLE__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_ENABLE_1_FIFO_FULL_ENABLE,
				v_intr_enable_1_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
				dev_addr,
				BMI160_USER_INTR_ENABLE_1_FIFO_FULL_ENABLE__REG,
				&v_data_u8, 1);
			}
		break;
		case BMI160_FIFO_WM_ENABLE:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_ENABLE_1_FIFO_WM_ENABLE__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_ENABLE_1_FIFO_WM_ENABLE,
				v_intr_enable_1_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
				dev_addr,
				BMI160_USER_INTR_ENABLE_1_FIFO_WM_ENABLE__REG,
				&v_data_u8, 1);
			}
		break;
		default:
			com_rslt = E_BMI160_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/************************************************************************
 *	Description: *//**brief  This API is used to read
 *	interrupt enable byte2 from the register bit 0x52 bit 0 to 3
 *
 *
 *
 *
 *	\param u8 v_enable_u8,u8 *v_intr_enable_2_u8 :
 *	The value of interrupt enable
 *	enable -->
 *	BMI160_NOMOTION_X_ENABLE	0
 *	BMI160_NOMOTION_Y_ENABLE	1
 *	BMI160_NOMOTION_Z_ENABLE	2
 *
 *	v_intr_enable_2_u8 --> 1
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **********************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_enable_2(
u8 v_enable_u8, u8 *v_intr_enable_2_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		switch (v_enable_u8) {
		case BMI160_NOMOTION_X_ENABLE:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr,
			BMI160_USER_INTR_ENABLE_2_NOMOTION_X_ENABLE__REG,
			&v_data_u8, 1);
			*v_intr_enable_2_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_ENABLE_2_NOMOTION_X_ENABLE);
			break;
		case BMI160_NOMOTION_Y_ENABLE:
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr,
			BMI160_USER_INTR_ENABLE_2_NOMOTION_Y_ENABLE__REG,
			&v_data_u8, 1);
			*v_intr_enable_2_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_ENABLE_2_NOMOTION_Y_ENABLE);
			break;
		case BMI160_NOMOTION_Z_ENABLE:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr,
			BMI160_USER_INTR_ENABLE_2_NOMOTION_Z_ENABLE__REG,
			&v_data_u8, 1);
			*v_intr_enable_2_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_ENABLE_2_NOMOTION_Z_ENABLE);
			break;
		default:
			com_rslt = E_BMI160_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/**************************************************************************
 *	Description: *//**brief This API is used to write
 *	interrupt enable byte2 from the register bit 0x52 bit 0 to 3
 *
 *
 *
 *
 *	\param u8 v_enable_u8,u8 *v_intr_enable_2_u8 :
 *	The value of interrupt enable
 *	enable -->
 *	BMI160_NOMOTION_X_ENABLE	0
 *	BMI160_NOMOTION_Y_ENABLE	1
 *	BMI160_NOMOTION_Z_ENABLE	2
 *
 *	v_intr_enable_2_u8 --> 1
 *
 *
 *
 *  \return results of communication
 *
 *
**************************************************************************/
/* Scheduling:
*
*
*
* Usage guide:
*
*
* Remarks:
*
*************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_enable_2(
u8 v_enable_u8, u8 v_intr_enable_2_u8)
{
BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
u8 v_data_u8 = C_BMI160_ZERO_U8X;
if (p_bmi160 == BMI160_NULL) {
	return E_BMI160_NULL_PTR;
	} else {
	switch (v_enable_u8) {
	case BMI160_NOMOTION_X_ENABLE:
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
		dev_addr,
		BMI160_USER_INTR_ENABLE_2_NOMOTION_X_ENABLE__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_ENABLE_2_NOMOTION_X_ENABLE,
			v_intr_enable_2_u8);
			com_rslt +=
			p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
			dev_addr,
			BMI160_USER_INTR_ENABLE_2_NOMOTION_X_ENABLE__REG,
			&v_data_u8, 1);
		}
		break;
	case BMI160_NOMOTION_Y_ENABLE:
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
		dev_addr,
		BMI160_USER_INTR_ENABLE_2_NOMOTION_Y_ENABLE__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_ENABLE_2_NOMOTION_Y_ENABLE,
			v_intr_enable_2_u8);
			com_rslt +=
			p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
			dev_addr,
			BMI160_USER_INTR_ENABLE_2_NOMOTION_Y_ENABLE__REG,
			&v_data_u8, 1);
		}
		break;
	case BMI160_NOMOTION_Z_ENABLE:
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
		dev_addr,
		BMI160_USER_INTR_ENABLE_2_NOMOTION_Z_ENABLE__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_ENABLE_2_NOMOTION_Z_ENABLE,
			v_intr_enable_2_u8);
			com_rslt +=
			p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
			dev_addr,
			BMI160_USER_INTR_ENABLE_2_NOMOTION_Z_ENABLE__REG,
			&v_data_u8, 1);
		}
		break;
	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
		break;
	}
}
return com_rslt;
}
/************************************************************************
 *	Description: *//**brief This API is used to read
 *	interrupt enable step detector interrupt from
 *	the register bit 0x52 bit 3
 *
 *
 *
 *
 *	\param u8 v_step_intr_u8 :
 *	Pointer holding the value of interrupt enable
 *
 *
 *
 *  \return results of communication
 *
 *
 ************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **********************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_step_detector_enable(
u8 *v_step_intr_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr,
			BMI160_USER_INTR_ENABLE_2_STEP_DETECTOR_ENABLE__REG,
			&v_data_u8, 1);
			*v_step_intr_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_ENABLE_2_STEP_DETECTOR_ENABLE);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/**************************************************************************
 *	Description: *//**brief This API is used to write
 *	interrupt enable step detector interrupt from
 *	the register bit 0x52 bit 3
 *
 *
 *
 *
 *	\param u8 v_step_intr_u8 : The value of interrupt enable
 *
 *
 *
 *  \return results of communication
 *
 *
**************************************************************************/
/* Scheduling:
*
*
*
* Usage guide:
*
*
* Remarks:
*
*************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_step_detector_enable(
u8 v_step_intr_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
		dev_addr,
		BMI160_USER_INTR_ENABLE_2_STEP_DETECTOR_ENABLE__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_ENABLE_2_STEP_DETECTOR_ENABLE,
			v_step_intr_u8);
			com_rslt +=
			p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
			dev_addr,
			BMI160_USER_INTR_ENABLE_2_STEP_DETECTOR_ENABLE__REG,
			&v_data_u8, 1);
		}
	}
	return com_rslt;
}
/**************************************************************************
 *	Description: *//**brief  Configure trigger condition of INT1
 *	and INT2 pin from the register 0x53 bit 0
 *
 *
 *  \param u8 v_channel_u8,u8 *v_intr_edge_ctrl_u8 :
 *	The value of interrupt enable
 *	v_channel_u8 -->
 *	BMI160_INTR1_EDGE_CTRL         0
 *	BMI160_INTR2_EDGE_CTRL         1
 *	v_intr_edge_ctrl_u8 -->
 *	0            Level
 *	1            Edge
 *
 *
 *  \return results of communication routine
 *
 *
 *************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_edge_ctrl(
u8 v_channel_u8, u8 *v_intr_edge_ctrl_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMI160_INTR1_EDGE_CTRL:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR1_EDGE_CTRL__REG,
			&v_data_u8, 1);
			*v_intr_edge_ctrl_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR1_EDGE_CTRL);
			break;
		case BMI160_INTR2_EDGE_CTRL:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR2_EDGE_CTRL__REG,
			&v_data_u8, 1);
			*v_intr_edge_ctrl_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR2_EDGE_CTRL);
			break;
		default:
			com_rslt = E_BMI160_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/**************************************************************************
 *	Description: *//**brief  Configure trigger condition of INT1
 *	and INT2 pin from the register 0x53 bit 0
 *
 *
 *  \param u8 v_channel_u8,u8 *v_intr_edge_ctrl_u8 :
 *	The value of interrupt enable
 *	channel -->
 *	BMI160_INTR1_EDGE_CTRL         0
 *	BMI160_INTR2_EDGE_CTRL         1
 *	v_intr_edge_ctrl_u8 -->
 *	0            Level
 *	1            Edge
 *
 *
 *
 *
 *  \return results of communication
 *
 *
 *************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***********************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_edge_ctrl(
u8 v_channel_u8, u8 v_intr_edge_ctrl_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMI160_INTR1_EDGE_CTRL:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR1_EDGE_CTRL__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR1_EDGE_CTRL,
				v_intr_edge_ctrl_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
				dev_addr, BMI160_USER_INTR1_EDGE_CTRL__REG,
				&v_data_u8, 1);
			}
			break;
		case BMI160_INTR2_EDGE_CTRL:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR2_EDGE_CTRL__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR2_EDGE_CTRL,
				v_intr_edge_ctrl_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
				dev_addr, BMI160_USER_INTR2_EDGE_CTRL__REG,
				&v_data_u8, 1);
			}
			break;
		default:
			com_rslt = E_BMI160_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/**************************************************************************
 *	Description: *//**brief  Configure trigger condition of INT1
 *	and INT2 pin form the register 0x53 bit 1 and 5
 *
 *  \param u8 v_channel_u8,u8 *v_intr_level_u8
 *	channel -->
 *	BMI160_INTR1_LEVEL         0
 *	BMI160_INTR2_LEVEL         1
 *	v_intr_level_u8 -->
 *	0            active low
 *	1            active high
 *
 *
 *  \return results of communication routine
 *
 *
 ************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_level(
u8 v_channel_u8, u8 *v_intr_level_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMI160_INTR1_LEVEL:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR1_LEVEL__REG,
			&v_data_u8, 1);
			*v_intr_level_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR1_LEVEL);
			break;
		case BMI160_INTR2_LEVEL:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR2_LEVEL__REG,
			&v_data_u8, 1);
			*v_intr_level_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR2_LEVEL);
			break;
		default:
			com_rslt = E_BMI160_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
/**************************************************************************
 *	Description: *//**brief  Configure set the trigger condition of INT1
 *	and INT2 pin form the register 0x53 bit 1 and 5
 *
 *  \param u8 v_channel_u8,u8  v_intr_level_u8 : The value of interrupt level
 *	channel -->
 *	BMI160_INTR1_LEVEL         0
 *	BMI160_INTR2_LEVEL         1
 *	v_intr_level_u8 -->
 *	0            active low
 *	1            active high
 *
 *
 *  \return results of communication routine
 *
 *
 ************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_level(
u8 v_channel_u8, u8 v_intr_level_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMI160_INTR1_LEVEL:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR1_LEVEL__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR1_LEVEL, v_intr_level_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
				dev_addr, BMI160_USER_INTR1_LEVEL__REG,
				&v_data_u8, 1);
			}
			break;
		case BMI160_INTR2_LEVEL:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR2_LEVEL__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR2_LEVEL, v_intr_level_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
				dev_addr, BMI160_USER_INTR2_LEVEL__REG,
				&v_data_u8, 1);
			}
			break;
		default:
			com_rslt = E_BMI160_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
/*************************************************************************
 *	Description: *//**brief  Configure trigger condition of INT1
 *	from the register 0x53 bit 2 and 6
 *
 *  \param u8 v_channel_u8,u8 *v_intr_output_type_u8 :
 *	The interrupt output enable
 *	v_channel_u8 -->
 *	BMI160_INTR1_OUTPUT_TYPE         0
 *	BMI160_INTR2_OUTPUT_TYPE         1
 *	v_intr_output_type_u8 -->
 *	0            push-pull
 *	1            open drain
 *
 *
 *  \return results of communication routine
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***********************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_output_type(
u8 v_channel_u8, u8 *v_intr_output_type_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMI160_INTR1_OUTPUT_TYPE:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR1_OUTPUT_TYPE__REG,
			&v_data_u8, 1);
			*v_intr_output_type_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR1_OUTPUT_TYPE);
			break;
		case BMI160_INTR2_OUTPUT_TYPE:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR2_OUTPUT_TYPE__REG,
			&v_data_u8, 1);
			*v_intr_output_type_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR2_OUTPUT_TYPE);
			break;
		default:
			com_rslt = E_BMI160_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
/**************************************************************************
 *	Description: *//**brief Configure trigger condition of INT1
 *	from the register 0x53 bit 2 and 6
 *
 *  \param u8 v_channel_u8,u8 *v_intr_output_type_u8 :
 *	The interrupt output enable
 *	channel -->
 *	BMI160_INTR1_OUTPUT_TYPE         0
 *	BMI160_INTR2_OUTPUT_TYPE         1
 *	v_intr_output_type_u8 -->
 *	0            push-pull
 *	1            open drain
 *
 *
 *
 *  \return results of communication routine communication results
 *
 *
 *************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_output_type(
u8 v_channel_u8, u8 v_intr_output_type_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMI160_INTR1_OUTPUT_TYPE:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR1_OUTPUT_TYPE__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR1_OUTPUT_TYPE,
				v_intr_output_type_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
				dev_addr, BMI160_USER_INTR1_OUTPUT_TYPE__REG,
				&v_data_u8, 1);
			}
			break;
		case BMI160_INTR2_OUTPUT_TYPE:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR2_OUTPUT_TYPE__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR2_OUTPUT_TYPE,
				v_intr_output_type_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
				dev_addr, BMI160_USER_INTR2_OUTPUT_TYPE__REG,
				&v_data_u8, 1);
			}
			break;
		default:
			com_rslt = E_BMI160_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
/**************************************************************************
 *	Description: *//**brief Output enable for INT1
 *	and INT2 pin from the register 0x53 bit 3 and 7
 *
 *
 *  \param u8 v_channel_u8,u8 *v_output_enable_u8
 *	channel -->
 *	BMI160_INTR1_OUTPUT_ENABLE         0
 *	BMI160_INTR2_OUTPUT_EN         1
 *	v_output_enable_u8 -->
 *	0           Output
 *	1            Input
 *
 *
 *  \return results of communication routine
 *
 *
 ************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_output_enable(
u8 v_channel_u8, u8 *v_output_enable_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMI160_INTR1_OUTPUT_ENABLE:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR1_OUTPUT_ENABLE__REG,
			&v_data_u8, 1);
			*v_output_enable_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR1_OUTPUT_ENABLE);
			break;
		case BMI160_INTR2_OUTPUT_EN:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR2_OUTPUT_EN__REG,
			&v_data_u8, 1);
			*v_output_enable_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR2_OUTPUT_EN);
			break;
		default:
			com_rslt = E_BMI160_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
/**************************************************************************
 *	Description: *//**brief Output enable for INT1
 *	and INT2 pin from the register 0x53 bit 3 and 7
 *
 *
 *  \param u8 v_channel_u8,u8 *v_output_enable_u8 :
 *	The value of output enable
 *	channel -->
 *	BMI160_INTR1_OUTPUT_ENABLE         0
 *	BMI160_INTR2_OUTPUT_EN         1
 *	v_output_enable_u8 -->
 *	0           Output
 *	1            Input
 *
 *
 *  \return results of communication routine
 *
 *
 ************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_output_enable(
u8 v_channel_u8, u8 v_output_enable_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMI160_INTR1_OUTPUT_ENABLE:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR1_OUTPUT_ENABLE__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR1_OUTPUT_ENABLE,
				v_output_enable_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
				dev_addr, BMI160_USER_INTR1_OUTPUT_ENABLE__REG,
				&v_data_u8, 1);
			}
		break;
		case BMI160_INTR2_OUTPUT_EN:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR2_OUTPUT_EN__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR2_OUTPUT_EN,
				v_output_enable_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
				dev_addr, BMI160_USER_INTR2_OUTPUT_EN__REG,
				&v_data_u8, 1);
			}
		break;
		default:
			com_rslt = E_BMI160_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/************************************************************************
*	Description: *//**brief This API is used to get the latch duration
*	from the register 0x54 bit 0 to 3
*
*
*
*  \param u8 *v_latch_intr_u8 :
*	Pointer holding the value of latch duration
*	0 -> NON_LATCH
*	1 -> temporary, 312.5 us
*	2 -> temporary, 625 us
*	3 -> temporary, 1.25 ms
*	4 -> temporary, 2.5 ms
*	5 -> temporary, 5 ms
*	6 -> temporary, 10 ms
*	7 -> temporary, 20 ms
*	8 -> temporary, 40 ms
*	9 -> temporary, 80 ms
*	10 -> temporary, 160 ms
*	11 -> temporary, 320 ms
*	12 -> temporary, 640 ms
*	13 ->  temporary, 1.28 s
*	14 -> temporary, 2.56 s
*	15 -> LATCHED
*
*
*
*  \return results of communication routine
*
*
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_latch_intr(
u8 *v_latch_intr_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_LATCH__REG,
			&v_data_u8, 1);
			*v_latch_intr_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_LATCH);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
*	Description: *//**brief This API is used to get the latch duration
*	from the register 0x54 bit 0 to 3
*
*
*
*  \param u8 *v_latch_intr_u8 : Pointer holding the value of latch duration
*	0 -> NON_LATCH
*	1 -> temporary, 312.5 us
*	2 -> temporary, 625 us
*	3 -> temporary, 1.25 ms
*	4 -> temporary, 2.5 ms
*	5 -> temporary, 5 ms
*	6 -> temporary, 10 ms
*	7 -> temporary, 20 ms
*	8 -> temporary, 40 ms
*	9 -> temporary, 80 ms
*	10 -> temporary, 160 ms
*	11 -> temporary, 320 ms
*	12 -> temporary, 640 ms
*	13 ->  temporary, 1.28 s
*	14 -> temporary, 2.56 s
*	15 -> LATCHED
*
*  \return results of communication
*
*************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
**************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_latch_intr(u8 v_latch_intr_u8)
{
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_latch_intr_u8 < C_BMI160_SIXTEEN_U8X) {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_LATCH__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_LATCH, v_latch_intr_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
				dev_addr, BMI160_USER_INTR_LATCH__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
 /**************************************************************************
 *	Description: *//**brief input enable for INT1
 *	and INT2 pin from the register 0x54 bit 4 and 5
 *
 *
 *
 *  \param u8 v_channel_u8,u8 *v_input_en_u8 : The value of interrupt enable
 *	channel -->
 *	BMI160_INTR1_INPUT_EN         0
 *	BMI160_INTR2_INPUT_ENABLE         1
 *	v_input_en_u8 -->
 *	Value        Description
 *	0            output
 *	1            Input
 *
 *
 *  \return results of communication routine
 *
 *
 ************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_input_enable(
u8 v_channel_u8, u8 *v_input_en_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMI160_INTR1_INPUT_EN:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR1_INPUT_ENABLE__REG,
			&v_data_u8, 1);
			*v_input_en_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR1_INPUT_ENABLE);
			break;
		case BMI160_INTR2_INPUT_ENABLE:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR2_INPUT_ENABLE__REG,
			&v_data_u8, 1);
			*v_input_en_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR2_INPUT_ENABLE);
			break;
		default:
			com_rslt = E_BMI160_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
/**************************************************************************
 *	Description: *//**brief input enable for INT1
 *	and INT2 pin from the register 0x54 bit 4 and 5
 *
 *
 *
 *  \param u8 v_channel_u8,u8 v_input_en_u8 : The value of interrupt enable
 *	channel -->
 *	BMI160_INTR1_INPUT_EN         0
 *	BMI160_INTR2_INPUT_ENABLE     1
 *	v_input_en_u8 -->
 *	Value        Description
 *	0            output
 *	1            Input
 *
 *  \return results of communication routine
 *
 *
 ************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_input_enable(
u8 v_channel_u8, u8 v_input_en_u8)
{
BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
u8 v_data_u8 = C_BMI160_ZERO_U8X;
if (p_bmi160 == BMI160_NULL) {
	return E_BMI160_NULL_PTR;
	} else {
	switch (v_channel_u8) {
	case BMI160_INTR1_INPUT_EN:
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
		dev_addr, BMI160_USER_INTR1_INPUT_ENABLE__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR1_INPUT_ENABLE, v_input_en_u8);
			com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR1_INPUT_ENABLE__REG,
			&v_data_u8, 1);
		}
	break;
	case BMI160_INTR2_INPUT_ENABLE:
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
		dev_addr, BMI160_USER_INTR2_INPUT_ENABLE__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR2_INPUT_ENABLE, v_input_en_u8);
			com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR2_INPUT_ENABLE__REG,
			&v_data_u8, 1);
		}
	break;
	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
	break;
	}
}
return com_rslt;
}
 /**************************************************************************
 *	Description: *//**brief Reads the Low g interrupt mapped to INT1
 *	and INT2 from the register 0x55 and 0x57
 *	INT1 bit 0 in the register 0x55
 *	INT2 bit 0 in the register 0x57
 *
 *  \param u8 v_channel_u8,u8 *v_intr_low_g_u8 : The value of lowg enable
 *	channel -->
 *	BMI160_INTR1_MAP_LOW_G         0
 *	BMI160_INTR2_MAP_LOW_G         1
 *
 *
 *
 *  \return results of communication routine
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_low_g(
u8 v_channel_u8, u8 *v_intr_low_g_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMI160_INTR1_MAP_LOW_G:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_0_INTR1_LOW_G__REG,
			&v_data_u8, 1);
			*v_intr_low_g_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_0_INTR1_LOW_G);
			break;
		case BMI160_INTR2_MAP_LOW_G:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_2_INTR2_LOW_G__REG,
			&v_data_u8, 1);
			*v_intr_low_g_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_2_INTR2_LOW_G);
			break;
		default:
			com_rslt = E_BMI160_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/************************************************************************
 *	Description: *//**brief Write the Low g interrupt mapped to INT1
 *	and INT2 from the register 0x55 and 0x57
 *	INT1 bit 0 in the register 0x55
 *	INT2 bit 0 in the register 0x57
 *
 *  \param u8 v_channel_u8,u8 v_intr_low_g_u8 : The value of lowg enable
 *	channel -->
 *	BMI160_INTR1_MAP_LOW_G         0
 *	BMI160_INTR2_MAP_LOW_G         1
 *
 *
 *
 *  \return results of communication
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_low_g(
u8 v_channel_u8, u8 v_intr_low_g_u8)
{
BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
u8 v_data_u8 = C_BMI160_ZERO_U8X;
u8 v_step_cnt_stat_u8 = C_BMI160_ZERO_U8X;
u8 v_step_det_stat_u8 = C_BMI160_ZERO_U8X;

if (p_bmi160 == BMI160_NULL) {
	return E_BMI160_NULL_PTR;
	} else {

	com_rslt = bmi160_get_step_detector_enable(&v_step_det_stat_u8);

	if (v_step_det_stat_u8 != C_BMI160_ZERO_U8X)
		com_rslt += bmi160_set_step_detector_enable(C_BMI160_ZERO_U8X);

	com_rslt += bmi160_get_step_counter_enable(&v_step_cnt_stat_u8);
	if (v_step_cnt_stat_u8 != C_BMI160_ZERO_U8X)
			com_rslt += bmi160_set_step_counter_enable(
			C_BMI160_ZERO_U8X);
	switch (v_channel_u8) {
	case BMI160_INTR1_MAP_LOW_G:
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
		dev_addr, BMI160_USER_INTR_MAP_0_INTR1_LOW_G__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_0_INTR1_LOW_G, v_intr_low_g_u8);
			com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_0_INTR1_LOW_G__REG,
			&v_data_u8, 1);
		}
		break;
	case BMI160_INTR2_MAP_LOW_G:
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
		dev_addr, BMI160_USER_INTR_MAP_2_INTR2_LOW_G__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_2_INTR2_LOW_G, v_intr_low_g_u8);
			com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_2_INTR2_LOW_G__REG,
			&v_data_u8, 1);
		}
		break;
	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
		break;
	}
}
return com_rslt;
}
/**************************************************************************
 *	Description: *//**brief Reads the HIGH g interrupt mapped to INT1
 *	and INT2 from the register 0x55 and 0x57
 *	INT1 bit 1 in the register 0x55
 *	INT2 bit 1 in the register 0x57
 *
 *  \param u8 v_channel_u8,u8 v_intr_high_g_u8 : The value of highg enable
 *	channel -->
 *	BMI160_INTR1_MAP_HIGH_G         0
 *	BMI160_INTR2_MAP_HIGH_G         1
 *
 *
 *
 *  \return results of communication routine
 *
 *
 *************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_high_g(
u8 v_channel_u8, u8 *v_intr_high_g_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMI160_INTR1_MAP_HIGH_G:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_0_INTR1_HIGH_G__REG,
			&v_data_u8, 1);
			*v_intr_high_g_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_0_INTR1_HIGH_G);
		break;
		case BMI160_INTR2_MAP_HIGH_G:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_2_INTR2_HIGH_G__REG,
			&v_data_u8, 1);
			*v_intr_high_g_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_2_INTR2_HIGH_G);
		break;
		default:
			com_rslt = E_BMI160_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**brief Write the HIGH g interrupt mapped to INT1
 *	and INT2 from the register 0x55 and 0x57
 *	INT1 bit 1 in the register 0x55
 *	INT2 bit 1 in the register 0x57
 *
 *  \param u8 v_channel_u8,u8 v_intr_high_g_u8 : The value of highg enable
 *	channel -->
 *	BMI160_INTR1_MAP_HIGH_G         0
 *	BMI160_INTR2_MAP_HIGH_G         1
 *
 *
 *
 *  \return results of communication routine
 *
 *
 *************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_high_g(
u8 v_channel_u8, u8 v_intr_high_g_u8)
{
BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
u8 v_data_u8 = C_BMI160_ZERO_U8X;
if (p_bmi160 == BMI160_NULL) {
	return E_BMI160_NULL_PTR;
	} else {
	switch (v_channel_u8) {
	case BMI160_INTR1_MAP_HIGH_G:
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
		dev_addr, BMI160_USER_INTR_MAP_0_INTR1_HIGH_G__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_0_INTR1_HIGH_G, v_intr_high_g_u8);
			com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_0_INTR1_HIGH_G__REG,
			&v_data_u8, 1);
		}
	break;
	case BMI160_INTR2_MAP_HIGH_G:
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
		dev_addr, BMI160_USER_INTR_MAP_2_INTR2_HIGH_G__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_2_INTR2_HIGH_G, v_intr_high_g_u8);
			com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_2_INTR2_HIGH_G__REG,
			&v_data_u8, 1);
		}
	break;
	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
	break;
	}
}
return com_rslt;
}
/****************************************************************************
 *	Description: *//**brief Reads the Any motion interrupt
 *	interrupt mapped to INT1
 *	and INT2 from the register 0x55 and 0x57
 *	INT1 bit 2 in the register 0x55
 *	INT2 bit 2 in the register 0x57
 *
 *  \param u8 v_channel_u8,u8 v_intr_any_motion_u8 :
 *	The value of any motion interrupt enable
 *	channel -->
 *	BMI160_INTR1_MAP_ANY_MOTION         0
 *	BMI160_INTR2_MAP_ANY_MOTION         1
 *
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_any_motion(
u8 v_channel_u8, u8 *v_intr_any_motion_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMI160_INTR1_MAP_ANY_MOTION:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_0_INTR1_ANY_MOTION__REG,
			&v_data_u8, 1);
			*v_intr_any_motion_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_0_INTR1_ANY_MOTION);
		break;
		case BMI160_INTR2_MAP_ANY_MOTION:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_2_INTR2_ANY_MOTION__REG,
			&v_data_u8, 1);
			*v_intr_any_motion_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_2_INTR2_ANY_MOTION);
		break;
		default:
			com_rslt = E_BMI160_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/****************************************************************************
 *	Description: *//**brief Write the Any motion interrupt
 *	interrupt mapped to INT1
 *	and INT2 from the register 0x55 and 0x57
 *	INT1 bit 2 in the register 0x55
 *	INT2 bit 2 in the register 0x57
 *
 *  \param u8 v_channel_u8,u8 v_intr_any_motion_u8 :
* The value of any motion interrupt enable
 *	channel -->
 *	BMI160_INTR1_MAP_ANY_MOTION         0
 *	BMI160_INTR2_MAP_ANY_MOTION         1
 *
 *
 *
 *  \return results of communication routine
 *
 *************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_any_motion(
u8 v_channel_u8, u8 v_intr_any_motion_u8)
{
BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
u8 v_data_u8 = C_BMI160_ZERO_U8X;
u8 sig_mot_stat = C_BMI160_ZERO_U8X;

if (p_bmi160 == BMI160_NULL) {
	return E_BMI160_NULL_PTR;
	} else {
	com_rslt = bmi160_get_intr_significant_motion_select(&sig_mot_stat);
	if (sig_mot_stat != C_BMI160_ZERO_U8X)
		com_rslt += bmi160_set_intr_significant_motion_select(
		C_BMI160_ZERO_U8X);
	switch (v_channel_u8) {
	case BMI160_INTR1_MAP_ANY_MOTION:
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
		dev_addr, BMI160_USER_INTR_MAP_0_INTR1_ANY_MOTION__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_0_INTR1_ANY_MOTION,
			v_intr_any_motion_u8);
			com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_0_INTR1_ANY_MOTION__REG,
			&v_data_u8, 1);
		}
	break;
	case BMI160_INTR2_MAP_ANY_MOTION:
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
		dev_addr, BMI160_USER_INTR_MAP_2_INTR2_ANY_MOTION__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_2_INTR2_ANY_MOTION,
			v_intr_any_motion_u8);
			com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_2_INTR2_ANY_MOTION__REG,
			&v_data_u8, 1);
		}
	break;
	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
	break;
	}
}
return com_rslt;
}
/****************************************************************************
 *	Description: *//**brief Reads the No motion interrupt
 *	interrupt mapped to INT1
 *	and INT2 from the register 0x55 and 0x57
 *	INT1 bit 3 in the register 0x55
 *	INT2 bit 3 in the register 0x57
 *
 *  \param u8 v_channel_u8,u8 v_intr_nomotion_u8 :
 *	The value of any motion interrupt enable
 *	channel -->
 *	BMI160_INTR1_MAP_NOMO         0
 *	BMI160_INTR2_MAP_NOMO         1
 *
 *
 *  \return results of communication routine
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_nomotion(
u8 v_channel_u8, u8 *v_intr_nomotion_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMI160_INTR1_MAP_NOMO:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_0_INTR1_NOMOTION__REG,
			&v_data_u8, 1);
			*v_intr_nomotion_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_0_INTR1_NOMOTION);
			break;
		case BMI160_INTR2_MAP_NOMO:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_2_INTR2_NOMOTION__REG,
			&v_data_u8, 1);
			*v_intr_nomotion_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_2_INTR2_NOMOTION);
			break;
		default:
			com_rslt = E_BMI160_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
/****************************************************************************
 *	Description: *//**brief Write the No motion interrupt
 *	interrupt mapped to INT1
 *	and INT2 from the register 0x55 and 0x57
 *	INT1 bit 3 in the register 0x55
 *	INT2 bit 3 in the register 0x57
 *
 *  \param u8 v_channel_u8,u8 v_intr_nomotion_u8 :
 *	The value of any motion interrupt enable
 *	channel -->
 *	BMI160_INTR1_MAP_NOMO         0
 *	BMI160_INTR2_MAP_NOMO         1
 *
 *
 *
 *  \return results of communication routine
 *
 *
 *************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_nomotion(
u8 v_channel_u8, u8 v_intr_nomotion_u8)
{
BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
u8 v_data_u8 = C_BMI160_ZERO_U8X;
if (p_bmi160 == BMI160_NULL) {
	return E_BMI160_NULL_PTR;
	} else {
	switch (v_channel_u8) {
	case BMI160_INTR1_MAP_NOMO:
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
		dev_addr, BMI160_USER_INTR_MAP_0_INTR1_NOMOTION__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_0_INTR1_NOMOTION,
			v_intr_nomotion_u8);
			com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_0_INTR1_NOMOTION__REG,
			&v_data_u8, 1);
		}
		break;
	case BMI160_INTR2_MAP_NOMO:
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
		dev_addr, BMI160_USER_INTR_MAP_2_INTR2_NOMOTION__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_2_INTR2_NOMOTION,
			v_intr_nomotion_u8);
			com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_2_INTR2_NOMOTION__REG,
			&v_data_u8, 1);
		}
		break;
	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
		break;
	}
}
return com_rslt;
}
/****************************************************************************
 *	Description: *//**brief Reads the Double Tap interrupt
 *	interrupt mapped to INT1
 *	and INT2 from the register 0x55 and 0x57
 *	INT1 bit 3 in the register 0x55
 *	INT2 bit 3 in the register 0x57
 *
 *  \param u8 v_channel_u8,u8 v_intr_double_tap_u8 :
 *	The value of double tap interrupt enable
 *	channel -->
 *	BMI160_INTR1_MAP_DOUBLE_TAP         0
 *	BMI160_INTR2_MAP_DOUBLE_TAP         1
 *
 *  \return results of communication routine
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_double_tap(
u8 v_channel_u8, u8 *v_intr_double_tap_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMI160_INTR1_MAP_DOUBLE_TAP:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_0_INTR1_DOUBLE_TAP__REG,
			&v_data_u8, 1);
			*v_intr_double_tap_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_0_INTR1_DOUBLE_TAP);
			break;
		case BMI160_INTR2_MAP_DOUBLE_TAP:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_2_INTR2_DOUBLE_TAP__REG,
			&v_data_u8, 1);
			*v_intr_double_tap_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_2_INTR2_DOUBLE_TAP);
			break;
		default:
			com_rslt = E_BMI160_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**brief Write the Double Tap interrupt
 *	interrupt mapped to INT1
 *	and INT2 from the register 0x55 and 0x57
 *	INT1 bit 3 in the register 0x55
 *	INT2 bit 3 in the register 0x57
 *
 *  \param u8 v_channel_u8,u8 v_intr_double_tap_u8 :
 *	The value of double tap interrupt enable
 *	channel -->
 *	BMI160_INTR1_MAP_DOUBLE_TAP         0
 *	BMI160_INTR2_MAP_DOUBLE_TAP         1
 *
 *
 *
 *  \return results of communication routine
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_double_tap(
u8 v_channel_u8, u8 v_intr_double_tap_u8)
{
BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
u8 v_data_u8 = C_BMI160_ZERO_U8X;
if (p_bmi160 == BMI160_NULL) {
	return E_BMI160_NULL_PTR;
	} else {
	switch (v_channel_u8) {
	case BMI160_INTR1_MAP_DOUBLE_TAP:
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
		dev_addr, BMI160_USER_INTR_MAP_0_INTR1_DOUBLE_TAP__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_0_INTR1_DOUBLE_TAP,
			v_intr_double_tap_u8);
			com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_0_INTR1_DOUBLE_TAP__REG,
			&v_data_u8, 1);
		}
		break;
	case BMI160_INTR2_MAP_DOUBLE_TAP:
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
		dev_addr, BMI160_USER_INTR_MAP_2_INTR2_DOUBLE_TAP__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_2_INTR2_DOUBLE_TAP,
			v_intr_double_tap_u8);
			com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_2_INTR2_DOUBLE_TAP__REG,
			&v_data_u8, 1);
		}
		break;
	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
		break;
	}
}
return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 *	Description: *//**brief Reads the Single Tap interrupt
 *	interrupt mapped to INT1
 *	and INT2 from the register 0x55 and 0x57
 *	INT1 bit 4 in the register 0x55
 *	INT2 bit 4 in the register 0x57
 *
 *  \param u8 v_channel_u8,u8 v_intr_single_tap_u8 :
 *	The value of single tap interrupt enable
 *	channel -->
 *	BMI160_INTR1_MAP_SINGLE_TAP         0
 *	BMI160_INTR2_MAP_SINGLE_TAP         1
 *
 *
 *  \return results of communication routine
 *
 *
 *************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_single_tap(
u8 v_channel_u8, u8 *v_intr_single_tap_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMI160_INTR1_MAP_SINGLE_TAP:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_0_INTR1_SINGLE_TAP__REG,
			&v_data_u8, 1);
			*v_intr_single_tap_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_0_INTR1_SINGLE_TAP);
			break;
		case BMI160_INTR2_MAP_SINGLE_TAP:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_2_INTR2_SINGLE_TAP__REG,
			&v_data_u8, 1);
			*v_intr_single_tap_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_2_INTR2_SINGLE_TAP);
			break;
		default:
			com_rslt = E_BMI160_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/**************************************************************************
 *	Description: *//**brief Write the Single Tap interrupt
 *	interrupt mapped to INT1
 *	and INT2 from the register 0x55 and 0x57
 *	INT1 bit 4 in the register 0x55
 *	INT2 bit 4 in the register 0x57
 *
 *  \param u8 v_channel_u8,u8 v_intr_single_tap_u8 :
 *	The value of single tap interrupt enable
 *	channel -->
 *	BMI160_INTR1_MAP_SINGLE_TAP         0
 *	BMI160_INTR2_MAP_SINGLE_TAP         1
 *
 *
 *
 *  \return results of communication routine
 *
 *
 *************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_single_tap(
u8 v_channel_u8, u8 v_intr_single_tap_u8)
{
BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
u8 v_data_u8 = C_BMI160_ZERO_U8X;
if (p_bmi160 == BMI160_NULL) {
	return E_BMI160_NULL_PTR;
	} else {
	switch (v_channel_u8) {
	case BMI160_INTR1_MAP_SINGLE_TAP:
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
		dev_addr, BMI160_USER_INTR_MAP_0_INTR1_SINGLE_TAP__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_0_INTR1_SINGLE_TAP,
			v_intr_single_tap_u8);
			com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_0_INTR1_SINGLE_TAP__REG,
			&v_data_u8, 1);
		}
		break;
	case BMI160_INTR2_MAP_SINGLE_TAP:
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
		dev_addr, BMI160_USER_INTR_MAP_2_INTR2_SINGLE_TAP__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_2_INTR2_SINGLE_TAP,
			v_intr_single_tap_u8);
			com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_2_INTR2_SINGLE_TAP__REG,
			&v_data_u8, 1);
		}
		break;
	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
		break;
	}
}
return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 *	Description: *//**brief Reads the Orient interrupt
 *	interrupt mapped to INT1
 *	and INT2 from the register 0x55 and 0x57
 *	INT1 bit 6 in the register 0x55
 *	INT2 bit 6 in the register 0x57
 *
 *  \param u8 v_channel_u8,u8 v_ntr_orient_u8 :
 *	The value of orient interrupt enable
 *	channel -->
 *	BMI160_INTR1_MAP_ORIENT         0
 *	BMI160_INTR2_MAP_ORIENT         1
 *
 *
 *
 *  \return results of communication routine
 *
 *
 *************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***********************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_orient(
u8 v_channel_u8, u8 *v_ntr_orient_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMI160_INTR1_MAP_ORIENT:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_0_INTR1_ORIENT__REG,
			&v_data_u8, 1);
			*v_ntr_orient_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_0_INTR1_ORIENT);
			break;
		case BMI160_INTR2_MAP_ORIENT:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_2_INTR2_ORIENT__REG,
			&v_data_u8, 1);
			*v_ntr_orient_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_2_INTR2_ORIENT);
			break;
		default:
			com_rslt = E_BMI160_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/**************************************************************************
 *	Description: *//**brief Write the Orient interrupt
 *	interrupt mapped to INT1
 *	and INT2 from the register 0x55 and 0x57
 *	INT1 bit 6 in the register 0x55
 *	INT2 bit 6 in the register 0x57
 *
 *  \param u8 v_channel_u8,u8 v_ntr_orient_u8 :
 *	The value of orient interrupt enable
 *	channel -->
 *	BMI160_INTR1_MAP_ORIENT         0
 *	BMI160_INTR2_MAP_ORIENT         1
 *
 *
 *
 *  \return results of communication routine
 *
 *
 *************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_orient(
u8 v_channel_u8, u8 v_ntr_orient_u8)
{
BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
u8 v_data_u8 = C_BMI160_ZERO_U8X;
if (p_bmi160 == BMI160_NULL) {
	return E_BMI160_NULL_PTR;
	} else {
	switch (v_channel_u8) {
	case BMI160_INTR1_MAP_ORIENT:
		com_rslt =
		p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
		dev_addr, BMI160_USER_INTR_MAP_0_INTR1_ORIENT__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_0_INTR1_ORIENT, v_ntr_orient_u8);
			com_rslt +=
			p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_0_INTR1_ORIENT__REG,
			&v_data_u8, 1);
		}
		break;
	case BMI160_INTR2_MAP_ORIENT:
		com_rslt =
		p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
		dev_addr, BMI160_USER_INTR_MAP_2_INTR2_ORIENT__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 =
			BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_2_INTR2_ORIENT, v_ntr_orient_u8);
			com_rslt +=
			p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_2_INTR2_ORIENT__REG,
			&v_data_u8, 1);
		}
		break;
	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
		break;
	}
}
return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**brief Reads the Flat interrupt
 *	interrupt mapped to INT1
 *	and INT2 from the register 0x55 and 0x57
 *	INT1 bit 6 in the register 0x55
 *	INT2 bit 6 in the register 0x57
 *
 *  \param u8 v_channel_u8,u8 v_intr_flat_u8 :
 *	The value of Flat interrupt enable
 *	channel -->
 *	BMI160_INTR1_MAP_FLAT		0
 *	BMI160_INTR2_MAP_FLAT		1
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_flat(
u8 v_channel_u8, u8 *v_intr_flat_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMI160_INTR1_MAP_FLAT:
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_0_INTR1_FLAT__REG,
			&v_data_u8, 1);
			*v_intr_flat_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_0_INTR1_FLAT);
			break;
		case BMI160_INTR2_MAP_FLAT:
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_2_INTR2_FLAT__REG,
			&v_data_u8, 1);
			*v_intr_flat_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_2_INTR2_FLAT);
			break;
		default:
			com_rslt = E_BMI160_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/**************************************************************************
 *	Description: *//**brief Writes the Flat interrupt
 *	interrupt mapped to INT1
 *	and INT2 from the register 0x55 and 0x57
 *	INT1 bit 6 in the register 0x55
 *	INT2 bit 6 in the register 0x57
 *
 *  \param u8 v_channel_u8,u8 v_intr_flat_u8 :
 *	The value of Flat interrupt enable
 *	channel -->
 *	BMI160_INTR1_MAP_FLAT		0
 *	BMI160_INTR2_MAP_FLAT		1
 *
 *
 *
 *  \return results of communication routine
 *
 *
 *************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***********************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_flat(
u8 v_channel_u8, u8 v_intr_flat_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMI160_INTR1_MAP_FLAT:
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_0_INTR1_FLAT__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 =
				BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_MAP_0_INTR1_FLAT,
				v_intr_flat_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
				dev_addr,
				BMI160_USER_INTR_MAP_0_INTR1_FLAT__REG,
				&v_data_u8, 1);
			}
			break;
		case BMI160_INTR2_MAP_FLAT:
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_2_INTR2_FLAT__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_MAP_2_INTR2_FLAT,
				v_intr_flat_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
				dev_addr,
				BMI160_USER_INTR_MAP_2_INTR2_FLAT__REG,
				&v_data_u8, 1);
			}
			break;
		default:
			com_rslt = E_BMI160_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
/****************************************************************************
 *	Description: *//**brief Reads PMU trigger interrupt mapped to INT1
 *	and INT2 form the register 0x56 bit 0 and 4
 *
 *
 *
 *
 *  \param u8 v_channel_u8,u8 *v_intr_pmu_trig_u8 :
 *	The value of PMU trigger
 *	channel -->
 *	BMI160_INTR1_MAP_PMUTRIG         0
 *	BMI160_INTR2_MAP_PMUTRIG         1
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_pmu_trig(
u8 v_channel_u8, u8 *v_intr_pmu_trig_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMI160_INTR1_MAP_PMUTRIG:
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_1_INTR1_PMU_TRIG__REG,
			&v_data_u8, 1);
			*v_intr_pmu_trig_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_1_INTR1_PMU_TRIG);
			break;
		case BMI160_INTR2_MAP_PMUTRIG:
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_1_INTR2_PMU_TRIG__REG,
			&v_data_u8, 1);
			*v_intr_pmu_trig_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_1_INTR2_PMU_TRIG);
			break;
		default:
			com_rslt = E_BMI160_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**brief Write PMU trigger interrupt mapped to INT1
 *	and INT2 form the register 0x56 bit 0 and 4
 *
 *
 *
 *
 *  \param u8 v_channel_u8,u8 v_intr_pmu_trig_u8 : The value of PMU trigger
 *	channel -->
 *	BMI160_INTR1_MAP_PMUTRIG         0
 *	BMI160_INTR2_MAP_PMUTRIG         1
 *
 *
 *  \return results of communication routine
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_pmu_trig(
u8 v_channel_u8, u8 v_intr_pmu_trig_u8)
{
BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
u8 v_data_u8 = C_BMI160_ZERO_U8X;
if (p_bmi160 == BMI160_NULL) {
	return E_BMI160_NULL_PTR;
	} else {
	switch (v_channel_u8) {
	case BMI160_INTR1_MAP_PMUTRIG:
		com_rslt =
		p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
		dev_addr, BMI160_USER_INTR_MAP_1_INTR1_PMU_TRIG__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 =
			BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_1_INTR1_PMU_TRIG,
			v_intr_pmu_trig_u8);
			com_rslt +=
			p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_1_INTR1_PMU_TRIG__REG,
			&v_data_u8, 1);
		}
	break;
	case BMI160_INTR2_MAP_PMUTRIG:
		com_rslt =
		p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
		dev_addr, BMI160_USER_INTR_MAP_1_INTR2_PMU_TRIG__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 =
			BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_1_INTR2_PMU_TRIG,
			v_intr_pmu_trig_u8);
			com_rslt +=
			p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_1_INTR2_PMU_TRIG__REG,
			&v_data_u8, 1);
		}
	break;
	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
	break;
	}
}
return com_rslt;
}
/****************************************************************************
 *	Description: *//**brief Reads FIFO Full interrupt mapped to INT1
 *	and INT2 form the register 0x56 bit 5 and 1
 *
 *
 *
 *
 *  \param u8 v_channel_u8,u8 *v_intr_fifo_full_u8 : The value of FIFO Full
 *	channel -->
 *	BMI160_INTR1_MAP_FIFO_FULL         0
 *	BMI160_INTR2_MAP_FIFO_FULL         1
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_fifo_full(
u8 v_channel_u8, u8 *v_intr_fifo_full_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMI160_INTR1_MAP_FIFO_FULL:
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_1_INTR1_FIFO_FULL__REG,
			&v_data_u8, 1);
			*v_intr_fifo_full_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_1_INTR1_FIFO_FULL);
		break;
		case BMI160_INTR2_MAP_FIFO_FULL:
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_1_INTR2_FIFO_FULL__REG,
			&v_data_u8, 1);
			*v_intr_fifo_full_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_1_INTR2_FIFO_FULL);
		break;
		default:
			com_rslt = E_BMI160_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**brief Write FIFO Full interrupt mapped to INT1
 *	and INT2 form the register 0x56 bit 5 and 1
 *
 *
 *
 *
 *  \param u8 v_channel_u8,u8 v_intr_fifo_full_u8 : The value of FIFO Full
 *	channel -->
 *	BMI160_INTR1_MAP_FIFO_FULL         0
 *	BMI160_INTR2_MAP_FIFO_FULL         1
 *
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_fifo_full(
u8 v_channel_u8, u8 v_intr_fifo_full_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMI160_INTR1_MAP_FIFO_FULL:
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_1_INTR1_FIFO_FULL__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 =
				BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_MAP_1_INTR1_FIFO_FULL,
				v_intr_fifo_full_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
				dev_addr,
				BMI160_USER_INTR_MAP_1_INTR1_FIFO_FULL__REG,
				&v_data_u8, 1);
			}
		break;
		case BMI160_INTR2_MAP_FIFO_FULL:
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_1_INTR2_FIFO_FULL__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 =
				BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_MAP_1_INTR2_FIFO_FULL,
				v_intr_fifo_full_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
				dev_addr,
				BMI160_USER_INTR_MAP_1_INTR2_FIFO_FULL__REG,
				&v_data_u8, 1);
			}
		break;
		default:
			com_rslt = E_BMI160_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/***************************************************************************
 *	Description: *//**brief Reads FIFO Watermark interrupt mapped to INT1
 *	and INT2 form the register 0x56 bit 6 and 2
 *
 *
 *
 *
 *  \param u8 v_channel_u8,u8 *v_intr_fifo_wm_u8 : The value of FIFO Full
 *	channel -->
 *	BMI160_INTR1_MAP_FIFO_WM         0
 *	BMI160_INTR2_MAP_FIFO_WM         1
 *
 *
 *
 *  \return results of communication routine
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_fifo_wm(
u8 v_channel_u8, u8 *v_intr_fifo_wm_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMI160_INTR1_MAP_FIFO_WM:
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_1_INTR1_FIFO_WM__REG,
			&v_data_u8, 1);
			*v_intr_fifo_wm_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_1_INTR1_FIFO_WM);
			break;
		case BMI160_INTR2_MAP_FIFO_WM:
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_1_INTR2_FIFO_WM__REG,
			&v_data_u8, 1);
			*v_intr_fifo_wm_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_1_INTR2_FIFO_WM);
			break;
		default:
			com_rslt = E_BMI160_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 *	Description: *//**brief Write FIFO Watermark interrupt mapped to INT1
 *	and INT2 form the register 0x56 bit 6 and 2
 *
 *
 *
 *
 *  \param u8 v_channel_u8,u8 v_intr_fifo_wm_u8 : The value of FIFO Full
 *	channel -->
 *	BMI160_INTR1_MAP_FIFO_WM         0
 *	BMI160_INTR2_MAP_FIFO_WM         1
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_fifo_wm(
u8 v_channel_u8, u8 v_intr_fifo_wm_u8)
{
BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
u8 v_data_u8 = C_BMI160_ZERO_U8X;
if (p_bmi160 == BMI160_NULL) {
	return E_BMI160_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMI160_INTR1_MAP_FIFO_WM:
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_1_INTR1_FIFO_WM__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_MAP_1_INTR1_FIFO_WM,
				v_intr_fifo_wm_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
				dev_addr,
				BMI160_USER_INTR_MAP_1_INTR1_FIFO_WM__REG,
				&v_data_u8, 1);
			}
			break;
		case BMI160_INTR2_MAP_FIFO_WM:
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_1_INTR2_FIFO_WM__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_MAP_1_INTR2_FIFO_WM,
				v_intr_fifo_wm_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
				dev_addr,
				BMI160_USER_INTR_MAP_1_INTR2_FIFO_WM__REG,
				&v_data_u8, 1);
			}
			break;
		default:
			com_rslt = E_BMI160_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
 /****************************************************************************
 *	Description: *//**brief Reads Data Ready interrupt mapped to INT1
 *	and INT2 form the register 0x56 bit 7 and 3
 *
 *
 *
 *
 *  \param u8 v_channel_u8,u8 *v_intr_data_rdy_u8 : The value of data ready
 *	channel -->
 *	BMI160_INTR1_MAP_DATA_RDY         0
 *	BMI160_INTR2_MAP_DATA_RDY         1
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_data_rdy(
u8 v_channel_u8, u8 *v_intr_data_rdy_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMI160_INTR1_MAP_DATA_RDY:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_1_INTR1_DATA_RDY__REG,
			&v_data_u8, 1);
			*v_intr_data_rdy_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_1_INTR1_DATA_RDY);
			break;
		case BMI160_INTR2_MAP_DATA_RDY:
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_1_INTR2_DATA_RDY__REG,
			&v_data_u8, 1);
			*v_intr_data_rdy_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_1_INTR2_DATA_RDY);
			break;
		default:
			com_rslt = E_BMI160_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**brief Write Data Ready interrupt mapped to INT1
 *	and INT2 form the register 0x56 bit 7 and 3
 *
 *
 *
 *
 *  \param u8 v_channel_u8,u8 v_intr_data_rdy_u8 : The value of data ready
 *	channel -->
 *	BMI160_INTR1_MAP_DATA_RDY         0
 *	BMI160_INTR2_MAP_DATA_RDY         1
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_data_rdy(
u8 v_channel_u8, u8 v_intr_data_rdy_u8)
{
BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
u8 v_data_u8 = C_BMI160_ZERO_U8X;
if (p_bmi160 == BMI160_NULL) {
	return E_BMI160_NULL_PTR;
	} else {
	switch (v_channel_u8) {
	case BMI160_INTR1_MAP_DATA_RDY:
		com_rslt =
		p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
		dev_addr, BMI160_USER_INTR_MAP_1_INTR1_DATA_RDY__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_1_INTR1_DATA_RDY,
			v_intr_data_rdy_u8);
			com_rslt +=
			p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_1_INTR1_DATA_RDY__REG,
			&v_data_u8, 1);
		}
	break;
	case BMI160_INTR2_MAP_DATA_RDY:
		com_rslt =
		p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->
		dev_addr, BMI160_USER_INTR_MAP_1_INTR2_DATA_RDY__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_1_INTR2_DATA_RDY,
			v_intr_data_rdy_u8);
			com_rslt +=
			p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->
			dev_addr, BMI160_USER_INTR_MAP_1_INTR2_DATA_RDY__REG,
			&v_data_u8, 1);
		}
	break;
	default:
	com_rslt = E_BMI160_OUT_OF_RANGE;
	break;
	}
}
return com_rslt;
}
/***************************************************************************
 *	Description: *//**brief This API Reads Data source for the interrupt
 *	engine for the single and double tap interrupts from the register
 *	0x58 bit 3
 *
 *
 *  \param u8 * v_tap_source_u8 : Pointer holding the value of the tap src
 *	Value         Description
 *	1             unfiltered data
 *	0             filtered data
 *
 *
 *  \return results of communication routine
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_tap_source(u8 *v_tap_source_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_INTR_DATA_0_INTR_TAP_SOURCE__REG,
			&v_data_u8, 1);
			*v_tap_source_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_DATA_0_INTR_TAP_SOURCE);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 *	Description: *//**brief This API Write Data source for the interrupt
 *	engine for the single and double tap interrupts from the register
 *	0x58 bit 3
 *
 *
 *  \param u8  v_tap_source_u8 :  The value of the v_tap_source_u8
 *	Value         Description
 *	1             unfiltered data
 *	0             filtered data
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_tap_source(
u8 v_tap_source_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_tap_source_u8 < C_BMI160_TWO_U8X) {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_INTR_DATA_0_INTR_TAP_SOURCE__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_DATA_0_INTR_TAP_SOURCE,
				v_tap_source_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC
				(p_bmi160->dev_addr,
				BMI160_USER_INTR_DATA_0_INTR_TAP_SOURCE__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/***************************************************************************
 *	Description: *//**brief This API Reads Data source for the
 *	interrupt engine for the low and high g interrupts
 *	from the register 0x58 bit 7
 *
 *  \param u8 * v_low_high_source_u8 : Pointer holding the value of source
 *	Value        Description
 *	1      unfiltered data
 *	0      filtered data
 *
 *
 *  \return results of communication routine
 *
 *
 *************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_low_high_source(
u8 *v_low_high_source_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_INTR_DATA_0_INTR_LOW_HIGH_SOURCE__REG,
			&v_data_u8, 1);
			*v_low_high_source_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_DATA_0_INTR_LOW_HIGH_SOURCE);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 *	Description: *//**brief This API Write Data source for the
 *	interrupt engine for the low and high g interrupts
 *	from the register 0x58 bit 7
 *
 *  \param u8 * v_low_high_source_u8 : Pointer holding the value of source
 *	Value        Description
 *	1      unfiltered data
 *	0      filtered data
 *
 *
 *
 *  \return results of communication routine
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_low_high_source(
u8 v_low_high_source_u8)
{
BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
u8 v_data_u8 = C_BMI160_ZERO_U8X;
if (p_bmi160 == BMI160_NULL) {
	return E_BMI160_NULL_PTR;
	} else {
	if (v_low_high_source_u8 < C_BMI160_TWO_U8X) {
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
		(p_bmi160->dev_addr,
		BMI160_USER_INTR_DATA_0_INTR_LOW_HIGH_SOURCE__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_DATA_0_INTR_LOW_HIGH_SOURCE,
			v_low_high_source_u8);
			com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_INTR_DATA_0_INTR_LOW_HIGH_SOURCE__REG,
			&v_data_u8, 1);
		}
	} else {
	com_rslt = E_BMI160_OUT_OF_RANGE;
	}
}
return com_rslt;
}
/*******************************************************************************
 *	Description: *//**brief This API Reads Data source for the
 *	interrupt engine for the nomotion and anymotion interrupts
 *	from the register 0x59 bit 7
 *
 *  \param u8 * v_motion_source_u8 : Pointer holding the value of source
 *
 *	Value   Description
 *	1   unfiltered data
 *	0   filtered data
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_motion_source(
u8 *v_motion_source_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_INTR_DATA_1_INTR_MOTION_SOURCE__REG,
			&v_data_u8, 1);
			*v_motion_source_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_DATA_1_INTR_MOTION_SOURCE);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API Write Data source for the
 *	interrupt engine for the nomotion and anymotion interrupts
 *	from the register 0x59 bit 7
 *
 *  \param u8 * v_motion_source_u8 : Pointer holding the value of source
 *
 *	Value   Description
 *	1   unfiltered data
 *	0   filtered data
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_motion_source(
u8 v_motion_source_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_motion_source_u8 < C_BMI160_TWO_U8X) {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_INTR_DATA_1_INTR_MOTION_SOURCE__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_DATA_1_INTR_MOTION_SOURCE,
				v_motion_source_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_INTR_DATA_1_INTR_MOTION_SOURCE__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*******************************************************************************
 *	Description: *//**brief This API is used to read Delay
 *	time definition for the low-g interrupt from the register 0x5A bit 0 to 7
 *
 *
 *
 *
 *  \param u8 *v_low_durn_u8 : Pointer holding the value duration
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_low_g_durn(
u8 *v_low_g_durn_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_INTR_LOWHIGH_0_INTR_LOW_DURN__REG,
			&v_data_u8, 1);
			*v_low_g_durn_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_LOWHIGH_0_INTR_LOW_DURN);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API is used to write Delay
 *	time definition for the low-g interrupt from the register 0x5A bit 0 to 7
 *
 *
 *
 *
 *  \param u8 v_low_g_durn_u8 : the value of duration
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_low_g_durn(u8 v_low_g_durn_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_WRITE_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_INTR_LOWHIGH_0_INTR_LOW_DURN__REG,
			&v_low_g_durn_u8, 1);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API is used to read Threshold
 *	definition for the low-g interrupt from the register 0x5B bit 0 to 7
 *
 *
 *
 *
 *  \param u8 *v_low_g_thres_u8 : Pointer holding the value of Threshold
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_low_g_thres(
u8 *v_low_g_thres_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_INTR_LOWHIGH_1_INTR_LOW_THRES__REG,
			&v_data_u8, 1);
			*v_low_g_thres_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_LOWHIGH_1_INTR_LOW_THRES);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API is used to write Threshold
 *	definition for the low-g interrupt from the register 0x5B bit 0 to 7
 *
 *
 *
 *
 *  \param u8 *v_low_g_thres_u8 :  the value of Threshold
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_low_g_thres(
u8 v_low_g_thres_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_WRITE_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_INTR_LOWHIGH_1_INTR_LOW_THRES__REG,
			&v_low_g_thres_u8, 1);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API Reads Low-g interrupt hysteresis;
 *	according to int_low_hy*125 mg, irrespective of the selected g-v_range_u8.
 *	from the register 0x5c bit 0 to 1
 *
 *  \param u8 * v_low_hyst_u8 : Pointer holding the value of hysteresis
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_low_g_hyst(
u8 *v_low_hyst_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_HYST__REG,
			&v_data_u8, 1);
			*v_low_hyst_u8 = BMI160_GET_BITSLICE(
			v_data_u8,
			BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_HYST);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API write Low-g interrupt hysteresis;
 *	according to int_low_hy*125 mg, irrespective of the selected g-v_range_u8.
 *	from the register 0x5c bit 0 to 1
 *
 *  \param u8 * v_low_hyst_u8 : Pointer holding the value of hysteresis
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_low_g_hyst(
u8 v_low_hyst_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_HYST__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_HYST,
				v_low_hyst_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_HYST__REG,
				&v_data_u8, 1);
			}
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API Reads Low-g interrupt mode:
 *	from the register 0x5C bit 2
 *
 *  \param u8 * v_low_g_mode_u8 : Pointer holding the value of mode
 *	Value      Description
 *	0      single-axis
 *	1     axis-summing
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_low_g_mode(u8 *v_low_g_mode_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_MODE__REG,
			&v_data_u8, 1);
			*v_low_g_mode_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_MODE);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API write Low-g interrupt mode:
 *	from the register 0x5C bit 2
 *
 *  \param u8 v_low_g_mode_u8 : the value of mode
 *	Value      Description
 *	0      single-axis
 *	1     axis-summing
 *
 *
 *  \return results of communication routine
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_low_g_mode(
u8 v_low_g_mode_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_low_g_mode_u8 < C_BMI160_TWO_U8X) {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_MODE__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_MODE,
				v_low_g_mode_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_MODE__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API Reads High-g interrupt hysteresis;
 *	according to int_high_hy*125 mg (2 g v_range_u8)
 *	int_high_hy*250 mg (4 g v_range_u8)
 *	int_high_hy*500 mg (8 g v_range_u8)
 *	int_high_hy*1000 mg (16 g v_range_u8)
 *	from the register 0x5C bit 6 and 7
 *
 *  \param u8 * v_high_g_hyst_u8 : Pointer holding the value of high hyst
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_high_g_hyst(
u8 *v_high_g_hyst_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_INTR_LOWHIGH_2_INTR_HIGH_G_HYST__REG,
			&v_data_u8, 1);
			*v_high_g_hyst_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_LOWHIGH_2_INTR_HIGH_G_HYST);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API Write High-g interrupt hysteresis;
 *	according to int_high_hy*125 mg (2 g v_range_u8)
 *	int_high_hy*250 mg (4 g v_range_u8)
 *	int_high_hy*500 mg (8 g v_range_u8)
 *	int_high_hy*1000 mg (16 g v_range_u8)
 *	from the register 0x5C bit 6 and 7
 *
 *  \param u8 * v_high_g_hyst_u8 : The value of hysteresis
 *
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_high_g_hyst(
u8 v_high_g_hyst_u8)
{
BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
u8 v_data_u8 = C_BMI160_ZERO_U8X;
if (p_bmi160 == BMI160_NULL) {
	return E_BMI160_NULL_PTR;
	} else {
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
		p_bmi160->dev_addr,
		BMI160_USER_INTR_LOWHIGH_2_INTR_HIGH_G_HYST__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_LOWHIGH_2_INTR_HIGH_G_HYST,
			v_high_g_hyst_u8);
			com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_INTR_LOWHIGH_2_INTR_HIGH_G_HYST__REG,
			&v_data_u8, 1);
		}
	}
return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API is used to read Delay
 *	time definition for the high-g interrupt from the register
 *	0x5D bit 0 to 7
 *
 *
 *
 *  \param u8 v_high_g_durn_u8 : Pointer holding the value of high duration
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_high_g_durn(
u8 *v_high_g_durn_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_INTR_LOWHIGH_3_INTR_HIGH_G_DURN__REG,
			&v_data_u8, 1);
			*v_high_g_durn_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_LOWHIGH_3_INTR_HIGH_G_DURN);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API is used to write Delay
 *	time definition for the high-g interrupt from the register
 *	0x5D bit 0 to 7
 *
 *
 *
 *  \param u8 v_high_g_durn_u8 :  The value of high duration
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_high_g_durn(
u8 v_high_g_durn_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_WRITE_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_INTR_LOWHIGH_3_INTR_HIGH_G_DURN__REG,
			&v_high_g_durn_u8, 1);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API is used to read Threshold
 *	definition for the high-g interrupt from the register 0x5E 0 to 7
 *
 *
 *
 *
 *  \param u8 v_high_g_thres_u8 : Pointer holding the value of Threshold
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_high_g_thres(
u8 *v_high_g_thres_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_INTR_LOWHIGH_4_INTR_HIGH_THRES__REG,
			&v_data_u8, 1);
			*v_high_g_thres_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_LOWHIGH_4_INTR_HIGH_THRES);
	}
	return com_rslt;
}
/*******************************************************************************
 *	Description: *//**brief This API is used to write Threshold
 *	definition for the high-g interrupt from the register 0x5E 0 to 7
 *
 *
 *
 *
 *  \param u8 v_high_g_thres_u8 : The value of Threshold
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_high_g_thres(
u8 v_high_g_thres_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	} else {
		com_rslt = p_bmi160->BMI160_BUS_WRITE_FUNC(
		p_bmi160->dev_addr,
		BMI160_USER_INTR_LOWHIGH_4_INTR_HIGH_THRES__REG,
		&v_high_g_thres_u8, 1);
	}
	return com_rslt;
}

/******************************************************************************
 *	Description: *//**brief This API Reads no-motion duration
 *	from the register 0x5F bit 0 and 1
 *
 *  \param u8 * v_any_motion_durn_u8 :
 *	Pointer holding the value of anymotion
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_any_motion_durn(
u8 *v_any_motion_durn_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	} else {
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
		(p_bmi160->dev_addr,
		BMI160_USER_INTR_MOTION_0_INTR_ANY_MOTION_DURN__REG,
		&v_data_u8, 1);
		*v_any_motion_durn_u8 = BMI160_GET_BITSLICE
		(v_data_u8,
		BMI160_USER_INTR_MOTION_0_INTR_ANY_MOTION_DURN);
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API write no-motion duration
 *	from the register 0x5F bit 0 and 1
 *
 *  \param u8 * v_any_motion_durn_u8 :
 *	The value of anymotion
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_any_motion_durn(
u8 v_any_motion_durn_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	} else {
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
		(p_bmi160->dev_addr,
		BMI160_USER_INTR_MOTION_0_INTR_ANY_MOTION_DURN__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MOTION_0_INTR_ANY_MOTION_DURN,
			v_any_motion_durn_u8);
			com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_INTR_MOTION_0_INTR_ANY_MOTION_DURN__REG,
			&v_data_u8, 1);
		}
	}
	return com_rslt;
}
/*******************************************************************************
 *	Description: *//**brief This API Reads Slow/no-motion
 *	interrupt trigger delay duration from the register 0x5F bit 0 to 7
 *
 *  \param u8 * v_slow_no_motion_u8 :
 *	Pointer holding the value of v_slow_no_motion_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_slow_no_motion_durn(
u8 *v_slow_no_motion_u8)
{
BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
u8 v_data_u8 = C_BMI160_ZERO_U8X;
if (p_bmi160 == BMI160_NULL) {
	return E_BMI160_NULL_PTR;
	} else {
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
		(p_bmi160->dev_addr,
		BMI160_USER_INTR_MOTION_0_INTR_SLOW_NO_MOTION_DURN__REG,
		&v_data_u8, 1);
		*v_slow_no_motion_u8 = BMI160_GET_BITSLICE
		(v_data_u8,
		BMI160_USER_INTR_MOTION_0_INTR_SLOW_NO_MOTION_DURN);
	}
return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API Write Slow/no-motion
 *	interrupt trigger delay duration from the register 0x5F bit 0 to 7
 *
 *  \param u8 * v_slow_no_motion_u8 :
 *	The value of v_slow_no_motion_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_slow_no_motion_durn(
u8 v_slow_no_motion_u8)
{
BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
u8 v_data_u8 = C_BMI160_ZERO_U8X;
if (p_bmi160 == BMI160_NULL) {
	return E_BMI160_NULL_PTR;
	} else {
	com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
	(p_bmi160->dev_addr,
	BMI160_USER_INTR_MOTION_0_INTR_SLOW_NO_MOTION_DURN__REG,
	&v_data_u8, 1);
	if (com_rslt == SUCCESS) {
		v_data_u8 = BMI160_SET_BITSLICE
		(v_data_u8,
		BMI160_USER_INTR_MOTION_0_INTR_SLOW_NO_MOTION_DURN,
		v_slow_no_motion_u8);
		com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC
		(p_bmi160->dev_addr,
		BMI160_USER_INTR_MOTION_0_INTR_SLOW_NO_MOTION_DURN__REG,
		&v_data_u8, 1);
	}
}
return com_rslt;
}

/*****************************************************************************
 *	Description: *//**brief This API is used to read Threshold
 *	definition for the any-motion interrupt from the register 0x60 bit
 *	0 to 7
 *
 *
 *  \param u8 v_any_motion_thres_u8 : Pointer holding
 *		the value of Threshold
 *
 *
 *
 *  \return results of communication routine
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_any_motion_thres(
u8 *v_any_motion_thres_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_INTR_MOTION_1_INTR_ANY_MOTION_THRES__REG,
			&v_data_u8, 1);
			*v_any_motion_thres_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MOTION_1_INTR_ANY_MOTION_THRES);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/******************************************************************************
 *	Description: *//**brief This API is used to write Threshold
 *	definition for the any-motion interrupt from the register 0x60 bit
 *	0 to 7
 *
 *
 *  \param u8 v_any_motion_thres_u8 :
 *	The value of Threshold
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_any_motion_thres(
u8 v_any_motion_thres_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	} else {
		com_rslt = p_bmi160->BMI160_BUS_WRITE_FUNC
		(p_bmi160->dev_addr,
		BMI160_USER_INTR_MOTION_1_INTR_ANY_MOTION_THRES__REG,
		&v_any_motion_thres_u8, 1);
	}
	return com_rslt;
}
/*******************************************************************************
 *	Description: *//**brief This API is used to read Threshold
 *	definition for the slow/no-motion interrupt
 *	from the register 0x61 bit 0 to 7
 *
 *
 *
 *
 *  \param u8 v_slow_no_motion_thres_u8 : Pointer holding
 *			the value of v_slow_no_motion_thres_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_slow_no_motion_thres(
u8 *v_slow_no_motion_thres_u8)
{
BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
u8 v_data_u8 = C_BMI160_ZERO_U8X;
if (p_bmi160 == BMI160_NULL) {
	return E_BMI160_NULL_PTR;
	} else {
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
		(p_bmi160->dev_addr,
		BMI160_USER_INTR_MOTION_2_INTR_SLOW_NO_MOTION_THRES__REG,
		&v_data_u8, 1);
		*v_slow_no_motion_thres_u8 =
		BMI160_GET_BITSLICE(v_data_u8,
		BMI160_USER_INTR_MOTION_2_INTR_SLOW_NO_MOTION_THRES);
	}
return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API is used to write Threshold
 *	definition for the slow/no-motion interrupt
 *	from the register 0x61 bit 0 to 7
 *
 *
 *
 *
 *  \param u8 v_slow_no_motion_thres_u8 : The value of Threshold
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_slow_no_motion_thres(
u8 v_slow_no_motion_thres_u8)
{
BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
if (p_bmi160 == BMI160_NULL) {
	return E_BMI160_NULL_PTR;
	} else {
		com_rslt = p_bmi160->BMI160_BUS_WRITE_FUNC(
		p_bmi160->dev_addr,
		BMI160_USER_INTR_MOTION_2_INTR_SLOW_NO_MOTION_THRES__REG,
		&v_slow_no_motion_thres_u8, 1);
	}
return com_rslt;
}
/*******************************************************************************
 *	Description: *//**brief This API is used to v
 *	the slow/no-motion interrupt from the register 0x62 bit 1
 *
 *
 *
 *
 *  \param u8 v_intr_slow_no_motion_select_u8 : Pointer holding
 *		the value of slow/no-motion slow/no-motion
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_slow_no_motion_select(
u8 *v_intr_slow_no_motion_select_u8)
{
BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
u8 v_data_u8 = C_BMI160_ZERO_U8X;
if (p_bmi160 == BMI160_NULL) {
	return E_BMI160_NULL_PTR;
	} else {
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
		p_bmi160->dev_addr,
		BMI160_USER_INTR_MOTION_3_INTR_SLOW_NO_MOTION_SELECT__REG,
		&v_data_u8, 1);
		*v_intr_slow_no_motion_select_u8 =
		BMI160_GET_BITSLICE(v_data_u8,
		BMI160_USER_INTR_MOTION_3_INTR_SLOW_NO_MOTION_SELECT);
	}
return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API is used to write
 *	the slow/no-motion interrupt from the register 0x62 bit 1
 *
 *
 *
 *
 *  \param u8 v_intr_slow_no_motion_select_u8 :
 *		the value of slow/no-motion slow/no-motion
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_slow_no_motion_select(
u8 v_intr_slow_no_motion_select_u8)
{
BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
u8 v_data_u8 = C_BMI160_ZERO_U8X;
if (p_bmi160 == BMI160_NULL) {
	return E_BMI160_NULL_PTR;
} else {
if (v_intr_slow_no_motion_select_u8 < C_BMI160_TWO_U8X) {
	com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
	(p_bmi160->dev_addr,
	BMI160_USER_INTR_MOTION_3_INTR_SLOW_NO_MOTION_SELECT__REG,
	&v_data_u8, 1);
	if (com_rslt == SUCCESS) {
		v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
		BMI160_USER_INTR_MOTION_3_INTR_SLOW_NO_MOTION_SELECT,
		v_intr_slow_no_motion_select_u8);
		com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC
		(p_bmi160->dev_addr,
		BMI160_USER_INTR_MOTION_3_INTR_SLOW_NO_MOTION_SELECT__REG,
		&v_data_u8, 1);
	}
} else {
com_rslt = E_BMI160_OUT_OF_RANGE;
}
}
return com_rslt;
}
/*******************************************************************************
 *	Description: *//**brief This API is used to select
 *	the significant/ any motion  interrupt from the register 0x62 bit 1
 *
 *
 *
 *
 *  \param u8 v_intr_significant_motion_select_u8 : Pointer holding
 *		the value of int_sig_mot_sel
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_significant_motion_select(
u8 *v_intr_significant_motion_select_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_INTR_SIGNIFICATION_MOTION_SELECT__REG,
			&v_data_u8, 1);
			*v_intr_significant_motion_select_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_SIGNIFICATION_MOTION_SELECT);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API is used to select
 *	the significant/ any motion  interrupt from the register 0x62 bit 1
 *
 *
 *
 *
 *  \param u8 v_intr_significant_motion_select_u8 :
 *	the value of int_sig_mot_sel
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_significant_motion_select(
u8 v_intr_significant_motion_select_u8)
{
BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
u8 v_data_u8 = C_BMI160_ZERO_U8X;
if (p_bmi160 == BMI160_NULL) {
	return E_BMI160_NULL_PTR;
	} else {
	if (v_intr_significant_motion_select_u8 < C_BMI160_TWO_U8X) {
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
		(p_bmi160->dev_addr,
		BMI160_USER_INTR_SIGNIFICATION_MOTION_SELECT__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_SIGNIFICATION_MOTION_SELECT,
			v_intr_significant_motion_select_u8);
			com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_INTR_SIGNIFICATION_MOTION_SELECT__REG,
			&v_data_u8, 1);
		}
	} else {
	com_rslt = E_BMI160_OUT_OF_RANGE;
	}
}
return com_rslt;
}
/*******************************************************************************
 *	Description: *//**brief This API is used to select
 *	the significant/ any motion skip time from the register 0x62 bit  2 and 3
 *
 *
 *
 *
 *  \param u8 v_int_sig_mot_skip_u8 : Pointer holding
 *		the value of v_int_sig_mot_skip_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_significant_motion_skip(
u8 *v_int_sig_mot_skip_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_INTR_SIGNIFICANT_MOTION_SKIP__REG,
			&v_data_u8, 1);
			*v_int_sig_mot_skip_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_SIGNIFICANT_MOTION_SKIP);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API is used to select
 *	the significant/ any motion skip time from the register 0x62 bit  2 and 3
 *
 *
 *
 *
 *  \param u8 v_int_sig_mot_skip_u8 :
 *	the value of v_int_sig_mot_skip_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_significant_motion_skip(
u8 v_int_sig_mot_skip_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_int_sig_mot_skip_u8 < C_BMI160_EIGHT_U8X) {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_INTR_SIGNIFICANT_MOTION_SKIP__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_SIGNIFICANT_MOTION_SKIP,
				v_int_sig_mot_skip_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC
				(p_bmi160->dev_addr,
				BMI160_USER_INTR_SIGNIFICANT_MOTION_SKIP__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*******************************************************************************
 *	Description: *//**brief This API is used to select
 *	the significant/ any motion proof time from the register 0x62 bit  4 and 5
 *
 *
 *
 *
 *  \param u8 v_significant_motion_proof_u8 : Pointer holding
 *		the value of int_sig_mot_proof
 *	Value  Name/Description
 *	0 -  proof_0_25s proof time 0.25 seconds
 *	1 -  proof_0_5s proof time 0.5 seconds
 *	2 -  proof_1s proof time 1 seconds
 *	3 -  proof_2s proof time 2 seconds
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_significant_motion_proof(
u8 *v_significant_motion_proof_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_INTR_SIGNIFICANT_MOTION_PROOF__REG,
			&v_data_u8, 1);
			*v_significant_motion_proof_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_SIGNIFICANT_MOTION_PROOF);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API is used to select
 *	the significant/ any motion proof time from the register 0x62 bit  4 and 5
 *
 *
 *
 *
 *  \param u8 v_significant_motion_proof_u8 : The value of int_sig_mot_proof
 *	Value  Name/Description
 *	0 -  proof_0_25s proof time 0.25 seconds
 *	1 -  proof_0_5s proof time 0.5 seconds
 *	2 -  proof_1s proof time 1 seconds
 *	3 -  proof_2s proof time 2 seconds
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_significant_motion_proof(
u8 v_significant_motion_proof_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_significant_motion_proof_u8 < C_BMI160_EIGHT_U8X) {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_INTR_SIGNIFICANT_MOTION_PROOF__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_SIGNIFICANT_MOTION_PROOF,
				v_significant_motion_proof_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC
				(p_bmi160->dev_addr,
				BMI160_USER_INTR_SIGNIFICANT_MOTION_PROOF__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*******************************************************************************
 *	Description: *//**brief This API is used to get the tap duration
 *	from the register 0x63 bit 0 to 2
 *
 *
 *
 *  \param u8 *v_tap_durn_u8 : Pointer holding the value of duration
 *                  0 -> 50ms
 *                  1 -> 100mS
 *                  2 -> 150mS
 *                  3 -> 200mS
 *                  4 -> 250mS
 *                  5 -> 375mS
 *                  6 -> 500mS
 *                  7 -> 700mS
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_tap_durn(
u8 *v_tap_durn_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_INTR_TAP_0_INTR_TAP_DURN__REG,
			&v_data_u8, 1);
			*v_tap_durn_u8 = BMI160_GET_BITSLICE(
			v_data_u8,
			BMI160_USER_INTR_TAP_0_INTR_TAP_DURN);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API is used to set the tap duration
 *	from the register 0x63 bit 0 to 2
 *
 *
 *
 *  \param u8 *v_tap_durn_u8 :  the value of duration
 *                  0 -> 50ms
 *                  1 -> 100mS
 *                  2 -> 150mS
 *                  3 -> 200mS
 *                  4 -> 250mS
 *                  5 -> 375mS
 *                  6 -> 500mS
 *                  7 -> 700mS
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_tap_durn(
u8 v_tap_durn_u8)
{
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_tap_durn_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_tap_durn_u8 < C_BMI160_EIGHT_U8X) {
			switch (v_tap_durn_u8) {
			case BMI160_TAP_DURN_50MS:
				v_data_tap_durn_u8 = BMI160_TAP_DURN_50MS;
				break;
			case BMI160_TAP_DURN_100MS:
				v_data_tap_durn_u8 = BMI160_TAP_DURN_100MS;
				break;
			case BMI160_TAP_DURN_150MS:
				v_data_tap_durn_u8 = BMI160_TAP_DURN_150MS;
				break;
			case BMI160_TAP_DURN_200MS:
				v_data_tap_durn_u8 = BMI160_TAP_DURN_200MS;
				break;
			case BMI160_TAP_DURN_250MS:
				v_data_tap_durn_u8 = BMI160_TAP_DURN_250MS;
				break;
			case BMI160_TAP_DURN_375MS:
				v_data_tap_durn_u8 = BMI160_TAP_DURN_375MS;
				break;
			case BMI160_TAP_DURN_500MS:
				v_data_tap_durn_u8 = BMI160_TAP_DURN_500MS;
				break;
			case BMI160_TAP_DURN_700MS:
				v_data_tap_durn_u8 = BMI160_TAP_DURN_700MS;
				break;
			default:
				break;
			}
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_INTR_TAP_0_INTR_TAP_DURN__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_TAP_0_INTR_TAP_DURN,
				v_data_tap_durn_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC
				(p_bmi160->dev_addr,
				BMI160_USER_INTR_TAP_0_INTR_TAP_DURN__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*******************************************************************************
 *	Description: *//**brief This API Reads the
 *	tap shock duration from the register 0x63 bit 2
 *
 *  \param u8 * v_tap_shock_u8 : Pointer holding
 *	the value of shock
 *	Value	Description
 *	0		50 ms
 *	1		75 ms
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_tap_shock(
u8 *v_tap_shock_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_INTR_TAP_0_INTR_TAP_SHOCK__REG,
			&v_data_u8, 1);
			*v_tap_shock_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_TAP_0_INTR_TAP_SHOCK);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API Write the
 *	tap shock duration from the register 0x63 bit 2
 *
 *  \param u8 * v_tap_shock_u8 : Pointer holding
 *	the value of shock
 *	Value	Description
 *	0		50 ms
 *	1		75 ms
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_tap_shock(u8 v_tap_shock_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_tap_shock_u8 < C_BMI160_TWO_U8X) {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_INTR_TAP_0_INTR_TAP_SHOCK__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_TAP_0_INTR_TAP_SHOCK,
				v_tap_shock_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC
				(p_bmi160->dev_addr,
				BMI160_USER_INTR_TAP_0_INTR_TAP_SHOCK__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API Reads Selects
 *	tap quiet duration from the register 0x63 bit 7
 *
 *
 *  \param u8 * v_tap_quiet_u8 : Pointer holding the value of quiet
 *	Value  Description
 *	0         30 ms
 *	1         20 ms
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_tap_quiet(
u8 *v_tap_quiet_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_INTR_TAP_0_INTR_TAP_QUIET__REG,
			&v_data_u8, 1);
			*v_tap_quiet_u8 = BMI160_GET_BITSLICE(
			v_data_u8,
			BMI160_USER_INTR_TAP_0_INTR_TAP_QUIET);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/**************************************************************************
 *	Description: *//**brief This API Write Selects
 *	tap quiet duration from the register 0x63 bit 7
 *
 *
 *  \param u8  v_tap_quiet_u8 : The value of quiet
 *	Value  Description
 *	0         30 ms
 *	1         20 ms
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_tap_quiet(u8 v_tap_quiet_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_tap_quiet_u8 < C_BMI160_TWO_U8X) {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_INTR_TAP_0_INTR_TAP_QUIET__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_TAP_0_INTR_TAP_QUIET,
				v_tap_quiet_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC
				(p_bmi160->dev_addr,
				BMI160_USER_INTR_TAP_0_INTR_TAP_QUIET__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*******************************************************************************
 *	Description: *//**brief This API Reads Threshold of the
 *	single/double tap interrupt corresponding to
 *	int_tap_th - 62.5 mg (2 g v_range_u8)
 *	int_tap_th - 125 mg (4 g v_range_u8)
 *	int_tap_th - 250 mg (8 g v_range_u8)
 *	int_tap_th - 500 mg (16 g v_range_u8)
 *	from the register 0x64 bit 0 to 4
 *
 *	\param u8 * v_tap_thres_u8 : Pointer holding the value of Threshold
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_tap_thres(
u8 *v_tap_thres_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_INTR_TAP_1_INTR_TAP_THRES__REG,
			&v_data_u8, 1);
			*v_tap_thres_u8 = BMI160_GET_BITSLICE
			(v_data_u8,
			BMI160_USER_INTR_TAP_1_INTR_TAP_THRES);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API write Threshold of the
 *	single/double tap interrupt corresponding to
 *	int_tap_th - 62.5 mg (2 g v_range_u8)
 *	int_tap_th - 125 mg (4 g v_range_u8)
 *	int_tap_th - 250 mg (8 g v_range_u8)
 *	int_tap_th - 500 mg (16 g v_range_u8)
 *	from the register 0x64 bit 0 to 4
 *
 *	\param u8 v_tap_thres_u8 : The value of Threshold
 *
 *
 *
 *  \return results of communication routine
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_tap_thres(
u8 v_tap_thres_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_INTR_TAP_1_INTR_TAP_THRES__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_TAP_1_INTR_TAP_THRES,
				v_tap_thres_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC
				(p_bmi160->dev_addr,
				BMI160_USER_INTR_TAP_1_INTR_TAP_THRES__REG,
				&v_data_u8, 1);
			}
		}
	return com_rslt;
}
/*******************************************************************************
 *	Description: *//**brief This API Reads the threshold for
 *	switching between different orientations.
 *	from the register 0x65 bit 0 and 1
 *
 *  \param u8 * v_orient_mode_u8 : Pointer holding the value of mode
 *                           Value      Description
 *                           0b00       symmetrical
 *                           0b01       high-asymmetrical
 *                           0b10       low-asymmetrical
 *                           0b11       symmetrical
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_orient_mode(
u8 *v_orient_mode_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_MODE__REG,
			&v_data_u8, 1);
			*v_orient_mode_u8 = BMI160_GET_BITSLICE
			(v_data_u8,
			BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_MODE);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API write the threshold for
 *	switching between different orientations.
 *	from the register 0x65 bit 0 and 1
 *
 *  \param u8 v_orient_mode_u8 : the value of mode
 *                           Value      Description
 *                           0b00       symmetrical
 *                           0b01       high-asymmetrical
 *                           0b10       low-asymmetrical
 *                           0b11       symmetrical
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_orient_mode(
u8 v_orient_mode_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_orient_mode_u8 < C_BMI160_FOUR_U8X) {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_MODE__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_MODE,
				v_orient_mode_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC
				(p_bmi160->dev_addr,
				BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_MODE__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API Reads Sets the blocking mode
 *	that is used for the generation of the orientation interrupt.
 *	from the register 0x65 bit 2 and 3
 *
 *  \param u8 * v_orient_blocking_u8 :
 *	Pointer holding the value of v_orient_blocking_u8
 *                 Value   Description
 *                 0b00    no blocking
 *                 0b01    theta blocking or acceleration in any axis > 1.5g
 *                 0b10    theta blocking or acceleration slope in any axis >
 *                             0.2g or acceleration in any axis > 1.5g
 *                 0b11    theta blocking or acceleration slope in any axis >
 *                             0.4g or acceleration in any axis >
 *                             1.5g and value of orient is not stable
 *                             for at least 100 ms
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_orient_blocking(
u8 *v_orient_blocking_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_BLOCKING__REG,
			&v_data_u8, 1);
			*v_orient_blocking_u8 = BMI160_GET_BITSLICE
			(v_data_u8,
			BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_BLOCKING);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API write Sets the blocking mode
 *	that is used for the generation of the orientation interrupt.
 *	from the register 0x65 bit 2 and 3
 *
 *  \param u8 v_orient_blocking_u8 : the value of v_orient_blocking_u8
 *                 Value   Description
 *                 0b00    no blocking
 *                 0b01    theta blocking or acceleration in any axis > 1.5g
 *                 0b10    theta blocking or acceleration slope in any axis >
 *                             0.2g or acceleration in any axis > 1.5g
 *                 0b11    theta blocking or acceleration slope in any axis >
 *                             0.4g or acceleration in any axis >
 *                             1.5g and value of orient is not stable
 *                             for at least 100 ms
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_orient_blocking(
u8 v_orient_blocking_u8)
{
BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
u8 v_data_u8 = C_BMI160_ZERO_U8X;
if (p_bmi160 == BMI160_NULL) {
	return E_BMI160_NULL_PTR;
	} else {
	if (v_orient_blocking_u8 < C_BMI160_FOUR_U8X) {
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
		(p_bmi160->dev_addr,
		BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_BLOCKING__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_BLOCKING,
			v_orient_blocking_u8);
			com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_BLOCKING__REG,
			&v_data_u8, 1);
		}
	} else {
	com_rslt = E_BMI160_OUT_OF_RANGE;
	}
}
return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API Reads Orient interrupt
 *	hysteresis; 1 LSB corresponds to 62.5 mg,
 *	irrespective of the selected g-v_range_u8.
 *	from the register 0x64 bit 4 to 7
 *
 *  \param u8 * v_orient_hyst_u8 : Pointer holding
 *	the value of hysteresis
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_orient_hyst(
u8 *v_orient_hyst_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_HYST__REG,
			&v_data_u8, 1);
			*v_orient_hyst_u8 = BMI160_GET_BITSLICE
			(v_data_u8,
			BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_HYST);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API write Orient interrupt
 *	hysteresis; 1 LSB corresponds to 62.5 mg,
 *	irrespective of the selected g-v_range_u8.
 *	from the register 0x64 bit 4 to 7
 *
 *  \param u8 * v_orient_hyst_u8 : The value of hysteresis
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_orient_hyst(
u8 v_orient_hyst_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_HYST__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_HYST,
				v_orient_hyst_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC
				(p_bmi160->dev_addr,
				BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_HYST__REG,
				&v_data_u8, 1);
			}
		}
	return com_rslt;
}
/*******************************************************************************
 *	Description: *//**brief This API Reads Orient
 *	blocking angle (0 to 44.8) from the register 0x66 bit 0 to 5
 *
 *  \param u8 * v_orient_theta_u8 : Pointer holding
 *	the value of v_orient_theta_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_orient_theta(
u8 *v_orient_theta_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_THETA__REG,
			&v_data_u8, 1);
			*v_orient_theta_u8 = BMI160_GET_BITSLICE
			(v_data_u8,
			BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_THETA);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API write Orient
 *	blocking angle (0 to 44.8) from the register 0x66 bit 0 to 5
 *
 *  \param u8 v_orient_theta_u8 : the value of v_orient_theta_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_orient_theta(
u8 v_orient_theta_u8)
{
BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
u8 v_data_u8 = C_BMI160_ZERO_U8X;
if (p_bmi160 == BMI160_NULL) {
	return E_BMI160_NULL_PTR;
	} else {
	if (v_orient_theta_u8 <= C_BMI160_THIRTYONE_U8X) {
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
		(p_bmi160->dev_addr,
		BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_THETA__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_THETA,
			v_orient_theta_u8);
			com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_THETA__REG,
			&v_data_u8, 1);
		}
	} else {
	com_rslt = E_BMI160_OUT_OF_RANGE;
	}
}
return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API Reads Change
 *	of up/down bit from the register 0x66 bit 6
 *
 *  \param u8 * v_orient_ud_u8 : Pointer holding
 *	the value of v_orient_ud_u8
 *                  Value       Description
 *                         0    is ignored
 *                         1    generates orientation interrupt
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_orient_ud_enable(
u8 *v_orient_ud_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_UD_ENABLE__REG,
			&v_data_u8, 1);
			*v_orient_ud_u8 = BMI160_GET_BITSLICE
			(v_data_u8,
			BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_UD_ENABLE);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API write Change
 *	of up/down bit from the register 0x66 bit 6
 *
 *  \param u8  v_orient_ud_u8 : the value of v_orient_ud_u8
 *                  Value       Description
 *                         0    is ignored
 *                         1    generates orientation interrupt
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_orient_ud_enable(
u8 v_orient_ud_u8)
{
BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
u8 v_data_u8 = C_BMI160_ZERO_U8X;
if (p_bmi160 == BMI160_NULL) {
	return E_BMI160_NULL_PTR;
	} else {
	if (v_orient_ud_u8 < C_BMI160_TWO_U8X) {
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
		(p_bmi160->dev_addr,
		BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_UD_ENABLE__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_UD_ENABLE,
			v_orient_ud_u8);
			com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_UD_ENABLE__REG,
			&v_data_u8, 1);
		}
	} else {
	com_rslt = E_BMI160_OUT_OF_RANGE;
	}
}
return com_rslt;
}
/*******************************************************************************
 *	Description: *//**brief This API Reads Change
 *	of up/down bit
 *	exchange x- and z-axis in algorithm, i.e x or z is relevant axis for
 *	upward/downward looking recognition (0=z, 1=x)
 *	from the register 0x66 bit 7
 *
 *  \param u8 * v_orient_axes_u8 : Pointer holding
 *	the value of orient_axes
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_orient_axes_enable(
u8 *v_orient_axes_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_AXES_EX__REG,
			&v_data_u8, 1);
			*v_orient_axes_u8 = BMI160_GET_BITSLICE
			(v_data_u8,
			BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_AXES_EX);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API write Change
 *	of up/down bit
 *	exchange x- and z-axis in algorithm, i.e x or z is relevant axis for
 *	upward/downward looking recognition (0=z, 1=x)
 *	from the register 0x66 bit 7
 *
 *  \param u8 v_orient_axes_u8 : the value of v_orient_axes_u8
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_orient_axes_enable(
u8 v_orient_axes_u8)
{
BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
u8 v_data_u8 = C_BMI160_ZERO_U8X;
if (p_bmi160 == BMI160_NULL) {
	return E_BMI160_NULL_PTR;
	} else {
	if (v_orient_axes_u8 < C_BMI160_TWO_U8X) {
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
		(p_bmi160->dev_addr,
		BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_AXES_EX__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_AXES_EX,
			v_orient_axes_u8);
			com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_AXES_EX__REG,
			&v_data_u8, 1);
		}
	} else {
	com_rslt = E_BMI160_OUT_OF_RANGE;
	}
}
return com_rslt;
}
/*******************************************************************************
 *	Description: *//**brief This API Reads Flat
 *	threshold angle (0 to 44.8) for flat interrupt
 *	from the register 0x67 bit 0 to 5
 *
 *  \param u8 * v_flat_theta_u8 : Pointer holding
 *	the value of v_flat_theta_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_flat_theta(
u8 *v_flat_theta_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_INTR_FLAT_0_INTR_FLAT_THETA__REG,
			&v_data_u8, 1);
			*v_flat_theta_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_FLAT_0_INTR_FLAT_THETA);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API write Flat
 *	threshold angle (0 to 44.8) for flat interrupt
 *	from the register 0x67 bit 0 to 5
 *
 *  \param u8 v_flat_theta_u8 : the value of v_flat_theta_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_flat_theta(
u8 v_flat_theta_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_flat_theta_u8 <= C_BMI160_THIRTYONE_U8X) {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_INTR_FLAT_0_INTR_FLAT_THETA__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_FLAT_0_INTR_FLAT_THETA,
				v_flat_theta_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC
				(p_bmi160->dev_addr,
				BMI160_USER_INTR_FLAT_0_INTR_FLAT_THETA__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API Reads Flat interrupt hold time;
 *	delay time for which the flat value must remain stable for
 *	the flat interrupt to be generated.
 *	from the register 0x68 bit 4 and 5
 *
 *  \param u8 * v_flat_hold_u8 : Pointer holding the value of v_flat_hold_u8
 *                      Value   Description
 *                      0b00    0 ms
 *                      0b01    512 ms
 *                      0b10    1024 ms
 *                      0b11    2048 ms
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_flat_hold(
u8 *v_flat_hold_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_INTR_FLAT_1_INTR_FLAT_HOLD__REG,
			&v_data_u8, 1);
			*v_flat_hold_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_FLAT_1_INTR_FLAT_HOLD);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API write Flat interrupt hold time;
 *	delay time for which the flat value must remain stable for
 *	the flat interrupt to be generated.
 *	from the register 0x68 bit 4 and 5
 *
 *  \param u8 v_flat_hold_u8 : the value of v_flat_hold_u8
 *                      Value   Description
 *                      0b00    0 ms
 *                      0b01    512 ms
 *                      0b10    1024 ms
 *                      0b11    2048 ms
 *
 *
 *
 *  \return results of communication routine
***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_flat_hold(
u8 v_flat_hold_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_flat_hold_u8 < C_BMI160_FOUR_U8X) {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_INTR_FLAT_1_INTR_FLAT_HOLD__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_FLAT_1_INTR_FLAT_HOLD,
				v_flat_hold_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC
				(p_bmi160->dev_addr,
				BMI160_USER_INTR_FLAT_1_INTR_FLAT_HOLD__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API Reads Flat interrupt hysteresis;
 *	flat value must change by more than twice the value of
 *	int_flat_hy to detect a state change.
 *	from the register 0x68 bit 0 to 3
 *
 *  \param u8 * v_flat_hyst_u8 : Pointer holding
 *	the value of v_flat_hyst_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_flat_hyst(
u8 *v_flat_hyst_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_INTR_FLAT_1_INTR_FLAT_HYST__REG,
			&v_data_u8, 1);
			*v_flat_hyst_u8 = BMI160_GET_BITSLICE(
			v_data_u8,
			BMI160_USER_INTR_FLAT_1_INTR_FLAT_HYST);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API write Flat interrupt hysteresis;
 *	flat value must change by more than twice the value of
 *	int_flat_hy to detect a state change.
 *	from the register 0x68 bit 0 to 3
 *
 *  \param u8 v_flat_hyst_u8 : the value of v_flat_hyst_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_flat_hyst(
u8 v_flat_hyst_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_flat_hyst_u8 < C_BMI160_SIXTEEN_U8X) {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_INTR_FLAT_1_INTR_FLAT_HYST__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_FLAT_1_INTR_FLAT_HYST,
				v_flat_hyst_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC
				(p_bmi160->dev_addr,
				BMI160_USER_INTR_FLAT_1_INTR_FLAT_HYST__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/******************************************************************************
 *	Description: *//**brief This API Reads offset compensation
 *	target value for z-axis is:
 *	from the register 0x69 bit 0 and 1
 *
 *  \param u8 * v_foc_accel_z_u8 :
 *      Pointer to the v_foc_accel_z_u8
 *                      Value    Description
 *                      0b00    disabled
 *                      0b01    +1 g
 *                      0b10    -1 g
 *                      0b11    0 g
 *
 *
 *  \return results of communication routine
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_foc_accel_z(u8 *v_foc_accel_z_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_FOC_ACCEL_Z__REG,
			&v_data_u8, 1);
			*v_foc_accel_z_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_FOC_ACCEL_Z);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API Writes offset compensation
 *	target value for z-axis is:
 *	from the register 0x69 bit 0 and 1
 *
 *  \param u8 v_foc_accel_z_u8 :
 *    Value of the v_foc_accel_z_u8
 *                     Value    Description
 *                      0b00    disabled
 *                      0b01    +1 g
 *                      0b10    -1 g
 *                      0b11    0 g
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_foc_accel_z(
u8 v_foc_accel_z_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_FOC_ACCEL_Z__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_FOC_ACCEL_Z,
				v_foc_accel_z_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC
				(p_bmi160->dev_addr,
				BMI160_USER_FOC_ACCEL_Z__REG,
				&v_data_u8, 1);
			}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API Reads offset compensation
 *	target value for y-axis is:
 *	from the register 0x69 bit 2 and 3
 *
 *  \param u8 * v_foc_accel_y_u8 :
 *      Pointer to the v_foc_accel_y_u8
 *                     Value    Description
 *                      0b00    disabled
 *                      0b01    +1 g
 *                      0b10    -1 g
 *                      0b11    0 g
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_foc_accel_y(u8 *v_foc_accel_y_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_FOC_ACCEL_Y__REG,
			&v_data_u8, 1);
			*v_foc_accel_y_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_FOC_ACCEL_Y);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API Writes offset compensation
 *	target value for y-axis is:
 *	from the register 0x69 bit 2 and 3
 *
 *  \param u8 v_foc_accel_y_u8 :
 *    Value of the v_foc_accel_y_u8
 *					 Value    Description
 *                      0b00    disabled
 *                      0b01    +1 g
 *                      0b10    -1 g
 *                      0b11    0 g
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_foc_accel_y(u8 v_foc_accel_y_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_foc_accel_y_u8 < C_BMI160_FOUR_U8X) {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_FOC_ACCEL_Y__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_FOC_ACCEL_Y,
				v_foc_accel_y_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC
				(p_bmi160->dev_addr,
				BMI160_USER_FOC_ACCEL_Y__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API Reads offset compensation
 *	target value for x-axis is:
 *	from the register 0x69 bit 4 and 5
 *
 *  \param u8 * v_foc_accel_x_u8 :
 *      Pointer to the v_foc_accel_x_u8
 *                     Value    Description
 *                      0b00    disabled
 *                      0b01    +1 g
 *                      0b10    -1 g
 *                      0b11    0 g
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_foc_accel_x(u8 *v_foc_accel_x_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	} else {
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
		p_bmi160->dev_addr,
		BMI160_USER_FOC_ACCEL_X__REG,
		&v_data_u8, 1);
		*v_foc_accel_x_u8 = BMI160_GET_BITSLICE(v_data_u8,
		BMI160_USER_FOC_ACCEL_X);
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API Writes offset compensation
 *	target value for x-axis is:
 *	from the register 0x69 bit 4 and 5
 *
 *  \param u8 v_foc_accel_x_u8 :
 *    Value of the v_foc_accel_x_u8
 *                     Value    Description
 *                      0b00    disabled
 *                      0b01    +1 g
 *                      0b10    -1 g
 *                      0b11    0 g
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_foc_accel_x(u8 v_foc_accel_x_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_foc_accel_x_u8 < C_BMI160_FOUR_U8X) {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_FOC_ACCEL_X__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_FOC_ACCEL_X,
				v_foc_accel_x_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_FOC_ACCEL_X__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API Writes fast offset compensation
 *	target value for foc_acc \
 *	from the register 0x69 bit 6
 *
 *  \param u8 v_foc_accel_u8:
 *    Value of the v_foc_accel_u8
 *	Value    Description
 *	0b00    disabled
 *	0b01    +1 g
 *	0b10    -1 g
 *	0b11    0 g
 *  \param u8 v_axis_u8:
 *	Value    Description
 *	0		FOC_X_AXIS
 *	1		FOC_Y_AXIS
 *	2		FOC_Z_AXIS
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_foc_trigger(u8 v_axis_u8,
u8 v_foc_accel_u8, s8 *v_accel_offset_s8)
{
BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
u8 v_data_u8 = C_BMI160_ZERO_U8X;
s8 v_status_s8 = SUCCESS;
u8 v_timeout_u8 = C_BMI160_ZERO_U8X;
s8 v_foc_accel_offset_x_s8  = C_BMI160_ZERO_U8X;
s8 v_foc_accel_offset_y_s8 =  C_BMI160_ZERO_U8X;
s8 v_foc_accel_offset_z_s8 =  C_BMI160_ZERO_U8X;
u8 focstatus = C_BMI160_ZERO_U8X;
if (p_bmi160 == BMI160_NULL) {
	return E_BMI160_NULL_PTR;
	} else {
		v_status_s8 = bmi160_set_accel_offset_enable(
		ACCEL_OFFSET_ENABLE);
		if (v_status_s8 == SUCCESS) {
			switch (v_axis_u8) {
			case FOC_X_AXIS:
				com_rslt =
				p_bmi160->BMI160_BUS_READ_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_FOC_ACCEL_X__REG,
				&v_data_u8, 1);
				if (com_rslt == SUCCESS) {
					v_data_u8 =
					BMI160_SET_BITSLICE(v_data_u8,
					BMI160_USER_FOC_ACCEL_X,
					v_foc_accel_u8);
					com_rslt +=
					p_bmi160->BMI160_BUS_WRITE_FUNC(
					p_bmi160->dev_addr,
					BMI160_USER_FOC_ACCEL_X__REG,
					&v_data_u8, 1);
				}

				/* trigger the
				FOC need to write
				0x03 in the register 0x7e*/
				com_rslt +=
				bmi160_set_command_register(
				START_FOC_ACCEL_GYRO);

				com_rslt +=
				bmi160_get_foc_rdy(&focstatus);
				if ((com_rslt != SUCCESS) ||
				(focstatus != C_BMI160_ONE_U8X)) {
					while ((com_rslt != SUCCESS) ||
					(focstatus != C_BMI160_ONE_U8X
					&& v_timeout_u8 <
					BMI160_MAXIMUM_TIMEOUT)) {
						p_bmi160->delay_msec(
						BMI160_DELAY_SETTLING_TIME);
						com_rslt = bmi160_get_foc_rdy(
						&focstatus);
						v_timeout_u8++;
					}
				}
				if ((com_rslt == SUCCESS) &&
					(focstatus == C_BMI160_ONE_U8X)) {
					com_rslt +=
					bmi160_get_accel_off_comp_xaxis(
					&v_foc_accel_offset_x_s8);
					*v_accel_offset_s8 =
					v_foc_accel_offset_x_s8;
				}
			break;
			case FOC_Y_AXIS:
				com_rslt =
				p_bmi160->BMI160_BUS_READ_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_FOC_ACCEL_Y__REG,
				&v_data_u8, 1);
				if (com_rslt == SUCCESS) {
					v_data_u8 =
					BMI160_SET_BITSLICE(v_data_u8,
					BMI160_USER_FOC_ACCEL_Y,
					v_foc_accel_u8);
					com_rslt +=
					p_bmi160->BMI160_BUS_WRITE_FUNC(
					p_bmi160->dev_addr,
					BMI160_USER_FOC_ACCEL_Y__REG,
					&v_data_u8, 1);
				}

				/* trigger the FOC
				need to write 0x03
				in the register 0x7e*/
				com_rslt +=
				bmi160_set_command_register(
				START_FOC_ACCEL_GYRO);

				com_rslt +=
				bmi160_get_foc_rdy(&focstatus);
				if ((com_rslt != SUCCESS) ||
				(focstatus != C_BMI160_ONE_U8X)) {
					while ((com_rslt != SUCCESS) ||
					(focstatus != C_BMI160_ONE_U8X
					&& v_timeout_u8 <
					BMI160_MAXIMUM_TIMEOUT)) {
						p_bmi160->delay_msec(
						BMI160_DELAY_SETTLING_TIME);
						com_rslt = bmi160_get_foc_rdy(
						&focstatus);
						v_timeout_u8++;
					}
				}
				if ((com_rslt == SUCCESS) &&
				(focstatus == C_BMI160_ONE_U8X)) {
					com_rslt +=
					bmi160_get_accel_off_comp_yaxis(
					&v_foc_accel_offset_y_s8);
					*v_accel_offset_s8 =
					v_foc_accel_offset_y_s8;
				}
			break;
			case FOC_Z_AXIS:
				com_rslt =
				p_bmi160->BMI160_BUS_READ_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_FOC_ACCEL_Z__REG,
				&v_data_u8, 1);
				if (com_rslt == SUCCESS) {
					v_data_u8 =
					BMI160_SET_BITSLICE(v_data_u8,
					BMI160_USER_FOC_ACCEL_Z,
					v_foc_accel_u8);
					com_rslt +=
					p_bmi160->BMI160_BUS_WRITE_FUNC(
					p_bmi160->dev_addr,
					BMI160_USER_FOC_ACCEL_Z__REG,
					&v_data_u8, 1);
				}

				/* trigger the FOC need to write
				0x03 in the register 0x7e*/
				com_rslt +=
				bmi160_set_command_register(
				START_FOC_ACCEL_GYRO);

				com_rslt +=
				bmi160_get_foc_rdy(&focstatus);
				if ((com_rslt != SUCCESS) ||
				(focstatus != C_BMI160_ONE_U8X)) {
					while ((com_rslt != SUCCESS) ||
					(focstatus != C_BMI160_ONE_U8X
					&& v_timeout_u8 <
					BMI160_MAXIMUM_TIMEOUT)) {
						p_bmi160->delay_msec(
						BMI160_DELAY_SETTLING_TIME);
						com_rslt = bmi160_get_foc_rdy(
						&focstatus);
						v_timeout_u8++;
					}
				}
				if ((com_rslt == SUCCESS) &&
				(focstatus == C_BMI160_ONE_U8X)) {
					com_rslt +=
					bmi160_get_accel_off_comp_zaxis(
					&v_foc_accel_offset_z_s8);
					*v_accel_offset_s8 =
					v_foc_accel_offset_z_s8;
				}
			break;
			default:
			break;
			}
		} else {
		com_rslt =  ERROR;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**brief This API Writes fast offset compensation
 * target value for z-v_foc_accel_x_u8, v_foc_accel_x_u8 and v_foc_accel_x_u8.
 *                     Value    Description
 *                      0b00    disabled
 *                      0b01	+1g
 *                      0b10	-1g
 *                      0b11	0g
 *
 *  \param u8 v_foc_accel_x_u8, v_foc_accel_y_u8, v_foc_accel_z_u8 :
 *    Value of the v_foc_accel_x_u8, v_foc_accel_y_u8, v_foc_accel_z_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_accel_foc_trigger_xyz(u8 v_foc_accel_x_u8,
u8 v_foc_accel_y_u8, u8 v_foc_accel_z_u8, s8 *v_accel_off_x_s8,
s8 *v_accel_off_y_s8, s8 *v_accel_off_z_s8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 focx = C_BMI160_ZERO_U8X;
	u8 focy = C_BMI160_ZERO_U8X;
	u8 focz = C_BMI160_ZERO_U8X;
	s8 v_foc_accel_offset_x_s8 = C_BMI160_ZERO_U8X;
	s8 v_foc_accel_offset_y_s8 = C_BMI160_ZERO_U8X;
	s8 v_foc_accel_offset_z_s8 = C_BMI160_ZERO_U8X;
	u8 v_status_s8 = SUCCESS;
	u8 v_timeout_u8 = C_BMI160_ZERO_U8X;
	u8 focstatus = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			v_status_s8 = bmi160_set_accel_offset_enable(
			ACCEL_OFFSET_ENABLE);
			if (v_status_s8 == SUCCESS) {
				/* foc x axis*/
				com_rslt =
				p_bmi160->BMI160_BUS_READ_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_FOC_ACCEL_X__REG,
				&focx, 1);
				if (com_rslt == SUCCESS) {
					focx = BMI160_SET_BITSLICE(focx,
					BMI160_USER_FOC_ACCEL_X,
					v_foc_accel_x_u8);
					com_rslt +=
					p_bmi160->BMI160_BUS_WRITE_FUNC(
					p_bmi160->dev_addr,
					BMI160_USER_FOC_ACCEL_X__REG,
					&focx, 1);
				}

				/* foc y axis*/
				com_rslt +=
				p_bmi160->BMI160_BUS_READ_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_FOC_ACCEL_Y__REG,
				&focy, 1);
				if (com_rslt == SUCCESS) {
					focy = BMI160_SET_BITSLICE(focy,
					BMI160_USER_FOC_ACCEL_Y,
					v_foc_accel_y_u8);
					com_rslt +=
					p_bmi160->BMI160_BUS_WRITE_FUNC(
					p_bmi160->dev_addr,
					BMI160_USER_FOC_ACCEL_Y__REG,
					&focy, 1);
				}

				/* foc z axis*/
				com_rslt +=
				p_bmi160->BMI160_BUS_READ_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_FOC_ACCEL_Z__REG,
				&focz, 1);
				if (com_rslt == SUCCESS) {
					focz = BMI160_SET_BITSLICE(focz,
					BMI160_USER_FOC_ACCEL_Z,
					v_foc_accel_z_u8);
					com_rslt +=
					p_bmi160->BMI160_BUS_WRITE_FUNC(
					p_bmi160->dev_addr,
					BMI160_USER_FOC_ACCEL_Z__REG,
					&focz, 1);
				}

				/* trigger the FOC need to
				write 0x03 in the register 0x7e*/
				com_rslt += bmi160_set_command_register(
				START_FOC_ACCEL_GYRO);

				com_rslt += bmi160_get_foc_rdy(
				&focstatus);
				if ((com_rslt != SUCCESS) ||
				(focstatus != C_BMI160_ONE_U8X)) {
					while ((com_rslt != SUCCESS) ||
					(focstatus != C_BMI160_ONE_U8X
					&& v_timeout_u8 <
					BMI160_MAXIMUM_TIMEOUT)) {
						p_bmi160->delay_msec(
						BMI160_DELAY_SETTLING_TIME);
						com_rslt = bmi160_get_foc_rdy(
						&focstatus);
						v_timeout_u8++;
					}
				}
				if ((com_rslt == SUCCESS) &&
				(focstatus == C_BMI160_ONE_U8X)) {
					com_rslt +=
					bmi160_get_accel_off_comp_xaxis(
					&v_foc_accel_offset_x_s8);
					*v_accel_off_x_s8 =
					v_foc_accel_offset_x_s8;
					com_rslt +=
					bmi160_get_accel_off_comp_yaxis(
					&v_foc_accel_offset_y_s8);
					*v_accel_off_y_s8 =
					v_foc_accel_offset_y_s8;
					com_rslt +=
					bmi160_get_accel_off_comp_zaxis(
					&v_foc_accel_offset_z_s8);
					*v_accel_off_z_s8 =
					v_foc_accel_offset_z_s8;
				}
			} else {
			com_rslt =  ERROR;
			}
		}
	return com_rslt;
}
/*******************************************************************************
 * Description: *//**brief This API Reads Enables fast offset
 * compensation for all three axis of the gyro
 *                          Value       Description
 *                              0       fast offset compensation disabled
 *                              1       fast offset compensation enabled
 *
 *  \param u8 * v_foc_gyro_u8 :
 *      Pointer to the v_foc_gyro_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_foc_gyro_enable(
u8 *v_foc_gyro_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_FOC_GYRO_ENABLE__REG,
			&v_data_u8, 1);
			*v_foc_gyro_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_FOC_GYRO_ENABLE);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**brief This API Writes Enables fast offset
 * compensation for all three axis of the gyro
 *                          Value       Description
 *                              0       fast offset compensation disabled
 *                              1       fast offset compensation enabled
 *
 *  \param u8 v_foc_gyro_u8 :
 *    Value of the v_foc_gyro_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_foc_gyro_enable(
u8 v_foc_gyro_u8, s16 *v_gyro_off_x_s16,
s16 *v_gyro_off_y_s16, s16 *v_gyro_off_z_s16)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	u8 v_status_s8 = SUCCESS;
	u8 v_timeout_u8 = C_BMI160_ZERO_U8X;
	s16 offsetx = C_BMI160_ZERO_U8X;
	s16 offsety = C_BMI160_ZERO_U8X;
	s16 offsetz = C_BMI160_ZERO_U8X;
	u8 focstatus = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			v_status_s8 = bmi160_set_gyro_offset_enable(
			GYRO_OFFSET_ENABLE);
			if (v_status_s8 == SUCCESS) {
				com_rslt =
				p_bmi160->BMI160_BUS_READ_FUNC
				(p_bmi160->dev_addr,
				BMI160_USER_FOC_GYRO_ENABLE__REG,
				&v_data_u8, 1);
				if (com_rslt == SUCCESS) {
					v_data_u8 =
					BMI160_SET_BITSLICE(v_data_u8,
					BMI160_USER_FOC_GYRO_ENABLE,
					v_foc_gyro_u8);
					com_rslt +=
					p_bmi160->BMI160_BUS_WRITE_FUNC
					(p_bmi160->dev_addr,
					BMI160_USER_FOC_GYRO_ENABLE__REG,
					&v_data_u8, 1);
				}

				/* trigger the FOC need to write 0x03
				in the register 0x7e*/
				com_rslt += bmi160_set_command_register
				(START_FOC_ACCEL_GYRO);

				com_rslt += bmi160_get_foc_rdy(&focstatus);
				if ((com_rslt != SUCCESS) ||
				(focstatus != C_BMI160_ONE_U8X)) {
					while ((com_rslt != SUCCESS) ||
					(focstatus != C_BMI160_ONE_U8X
					&& v_timeout_u8 <
					BMI160_MAXIMUM_TIMEOUT)) {
						p_bmi160->delay_msec(
						BMI160_DELAY_SETTLING_TIME);
						com_rslt = bmi160_get_foc_rdy(
						&focstatus);
						v_timeout_u8++;
					}
				}
				if ((com_rslt == SUCCESS) &&
				(focstatus == C_BMI160_ONE_U8X)) {
					com_rslt +=
					bmi160_get_gyro_off_comp_xaxis
					(&offsetx);
					*v_gyro_off_x_s16 = offsetx;

					com_rslt +=
					bmi160_get_gyro_off_comp_yaxis
					(&offsety);
					*v_gyro_off_y_s16 = offsety;

					com_rslt +=
					bmi160_get_gyro_off_comp_zaxis(
					&offsetz);
					*v_gyro_off_z_s16 = offsetz;
				}
			} else {
			com_rslt = ERROR;
			}
		}
	return com_rslt;
}
/*******************************************************************************
 * Description: *//**brief This API Reads Enable
 * NVM programming the register 0x6A bit 1
 *             Value    Description
 *                0     disable
 *                1     enable
 *
 *  \param u8 * v_nvm_prog_u8 :
 *      Pointer to the v_nvm_prog_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_nvm_prog_enable(
u8 *v_nvm_prog_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_CONFIG_NVM_PROG_ENABLE__REG,
			&v_data_u8, 1);
			*v_nvm_prog_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_CONFIG_NVM_PROG_ENABLE);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**brief This API Writes Enable
 * NVM programming from the register 0x6A bit 1
 *             Value    Description
 *                0     disable
 *                1     enable
 *
 *  \param u8 v_nvm_prog_u8 :
 *    Value of the v_nvm_prog_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_nvm_prog_enable(
u8 v_nvm_prog_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_nvm_prog_u8 < C_BMI160_TWO_U8X) {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_CONFIG_NVM_PROG_ENABLE__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_CONFIG_NVM_PROG_ENABLE,
				v_nvm_prog_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_CONFIG_NVM_PROG_ENABLE__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**brief This API Reads to Configure SPI
 * Interface Mode for primary and OIS interface
 * from the register 0x6B bit 0
 *                     Value    Description
 *                         0    SPI 4-wire mode
 *                         1    SPI 3-wire mode
 *
 *  \param u8 * v_spi3_u8 :
 *      Pointer to the v_spi3_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_spi3(
u8 *v_spi3_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_IF_CONFIG_SPI3__REG,
			&v_data_u8, 1);
			*v_spi3_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_IF_CONFIG_SPI3);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**brief This API Writes  to Configure SPI
 * Interface Mode for primary and OIS interface
 * from the register 0x6B bit 0
 *                     Value    Description
 *                         0    SPI 4-wire mode
 *                         1    SPI 3-wire mode
 *
 *  \param u8 v_spi3_u8 :
 *    Value of the v_spi3_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_spi3(
u8 v_spi3_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_spi3_u8 < C_BMI160_TWO_U8X) {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_IF_CONFIG_SPI3__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_IF_CONFIG_SPI3,
				v_spi3_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_IF_CONFIG_SPI3__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**brief This API Reads Select timer
 * period for I2C Watchdog from the register 0x70 bit 1
 *            Value     Description
 *              0       I2C watchdog v_timeout_u8 after 1 ms
 *              1       I2C watchdog v_timeout_u8 after 50 ms
 *
 *  \param u8 * v_i2c_wdt_u8 :
 *      Pointer to the v_i2c_wdt_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_i2c_wdt_select(
u8 *v_i2c_wdt_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_IF_CONFIG_I2C_WDT_SELECT__REG,
			&v_data_u8, 1);
			*v_i2c_wdt_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_IF_CONFIG_I2C_WDT_SELECT);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**brief This API Writes  Select timer
 * period for I2C Watchdog from the register 0x70 bit 1
 *            Value     Description
 *              0       I2C watchdog v_timeout_u8 after 1 ms
 *              1       I2C watchdog v_timeout_u8 after 50 ms
 *
 *  \param u8 v_i2c_wdt_u8 :
 *    Value of the v_i2c_wdt_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_i2c_wdt_select(
u8 v_i2c_wdt_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_i2c_wdt_u8 < C_BMI160_TWO_U8X) {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_IF_CONFIG_I2C_WDT_SELECT__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_IF_CONFIG_I2C_WDT_SELECT,
				v_i2c_wdt_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_IF_CONFIG_I2C_WDT_SELECT__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**brief This API Reads I2C Watchdog
 * at the SDI pin in I2C interface mode
 * from the register 0x70 bit 2
 *                 Value        Description
 *                   0  Disable I2C watchdog
 *                   1  Enable I2C watchdog
 *
 *  \param u8 * v_i2c_wdt_u8 :
 *      Pointer to the v_i2c_wdt_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_i2c_wdt_enable(
u8 *v_i2c_wdt_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_IF_CONFIG_I2C_WDT_ENABLE__REG,
			&v_data_u8, 1);
			*v_i2c_wdt_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_IF_CONFIG_I2C_WDT_ENABLE);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**brief This API Reads I2C Watchdog
 * at the SDI pin in I2C interface mode
 * from the register 0x70 bit 2
 *                 Value        Description
 *                   0  Disable I2C watchdog
 *                   1  Enable I2C watchdog
 *
 *  \param u8 * v_i2c_wdt_u8 :
 *      Pointer to the v_i2c_wdt_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_i2c_wdt_enable(
u8 v_i2c_wdt_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_i2c_wdt_u8 < C_BMI160_TWO_U8X) {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_IF_CONFIG_I2C_WDT_ENABLE__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_IF_CONFIG_I2C_WDT_ENABLE,
				v_i2c_wdt_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_IF_CONFIG_I2C_WDT_ENABLE__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}


/*******************************************************************************
 * Description: *//**brief This API Reads I2C Watchdog
 * at the SDI pin in I2C interface mode
 * from the register 0x6B bit 4 and 5
 *          Value        Description
 *          0b00 Primary interface:autoconfig / secondary interface:off
 *          0b01 Primary interface:I2C / secondary interface:OIS
 *          0b10 Primary interface:autoconfig/secondary interface:Magnetometer
 *          0b11 Reserved
 *  \param u8 v_if_mode_u8 :
 *    Value of the v_if_mode_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_if_mode(
u8 *v_if_mode_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_IF_CONFIG_IF_MODE__REG,
			&v_data_u8, 1);
			*v_if_mode_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_IF_CONFIG_IF_MODE);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/******************************************************************************
 * Description: *//**brief This API Writes   I2C Watchdog
 * at the SDI pin in I2C interface mode
 * from the register 0x6B bit 4 and 5
 *          Value        Description
 *          0b00 Primary interface:autoconfig / secondary interface:off
 *          0b01 Primary interface:I2C / secondary interface:OIS
 *          0b10 Primary interface:autoconfig/secondary interface:Magnetometer
 *          0b11 Reserved
 *  \param u8 v_if_mode_u8 :
 *    Value of the v_if_mode_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_if_mode(
u8 v_if_mode_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_if_mode_u8 <= C_BMI160_FOUR_U8X) {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_IF_CONFIG_IF_MODE__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_IF_CONFIG_IF_MODE,
				v_if_mode_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_IF_CONFIG_IF_MODE__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*******************************************************************************
 * Description: *//**brief This API Reads
 *           Gyro sleep trigger. when both trigger conditions are enabled,
 *           one is sufficient to trigger the transition.
 *			 from the register 0x6C bit 0 to 2
 *            Value     Description
 *            0b000     nomotion: no / Not INT1 pin: no / INT2 pin: no
 *            0b001     nomotion: no / Not INT1 pin: no / INT2 pin: yes
 *            0b010     nomotion: no / Not INT1 pin: yes / INT2 pin: no
 *            0b011     nomotion: no / Not INT1 pin: yes / INT2 pin: yes
 *            0b100     nomotion: yes / Not INT1 pin: no / INT2 pin: no
 *            0b001     anymotion: yes / Not INT1 pin: no / INT2 pin: yes
 *            0b010     anymotion: yes / Not INT1 pin: yes / INT2 pin: no
 *            0b011     anymotion: yes / Not INT1 pin: yes / INT2 pin: yes
 *
 *  \param u8 * v_gyro_sleep_trigger_u8 :
 *      Pointer to the v_gyro_sleep_trigger_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_sleep_trigger(
u8 *v_gyro_sleep_trigger_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_GYRO_SLEEP_TRIGGER__REG,
			&v_data_u8, 1);
			*v_gyro_sleep_trigger_u8 =
			BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_GYRO_SLEEP_TRIGGER);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**brief This API Writes
 *           Gyro sleep trigger. when both trigger conditions are enabled,
 *           one is sufficient to trigger the transition.
 *			 from the register 0x6C bit 0 to 2
 *            Value     Description
 *            0b000     nomotion: no / Not INT1 pin: no / INT2 pin: no
 *            0b001     nomotion: no / Not INT1 pin: no / INT2 pin: yes
 *            0b010     nomotion: no / Not INT1 pin: yes / INT2 pin: no
 *            0b011     nomotion: no / Not INT1 pin: yes / INT2 pin: yes
 *            0b100     nomotion: yes / Not INT1 pin: no / INT2 pin: no
 *            0b001     anymotion: yes / Not INT1 pin: no / INT2 pin: yes
 *            0b010     anymotion: yes / Not INT1 pin: yes / INT2 pin: no
 *            0b011     anymotion: yes / Not INT1 pin: yes / INT2 pin: yes
 *
 *  \param u8 v_gyro_sleep_trigger_u8 :
 *    Value of the v_gyro_sleep_trigger_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_sleep_trigger(
u8 v_gyro_sleep_trigger_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_gyro_sleep_trigger_u8 <= C_BMI160_SEVEN_U8X) {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_GYRO_SLEEP_TRIGGER__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_GYRO_SLEEP_TRIGGER,
				v_gyro_sleep_trigger_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_GYRO_SLEEP_TRIGGER__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**brief This API Reads
 *           Gyro wakeup trigger. When both trigger conditions are enabled,
 *           both conditions must be active to trigger the transition
 *			from the register 0x6C bit 3 and 4
 *             Value    Description
 *             0b00     anymotion: no / INT1 pin: no
 *             0b01     anymotion: no / INT1 pin: yes
 *             0b10     anymotion: yes / INT1 pin: no
 *             0b11     anymotion: yes / INT1 pin: yes
 *
 *  \param u8 * v_gyro_wakeup_trigger_u8 :
 *      Pointer to the v_gyro_wakeup_trigger_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_wakeup_trigger(
u8 *v_gyro_wakeup_trigger_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_GYRO_WAKEUP_TRIGGER__REG,
			&v_data_u8, 1);
			*v_gyro_wakeup_trigger_u8 = BMI160_GET_BITSLICE(
			v_data_u8,
			BMI160_USER_GYRO_WAKEUP_TRIGGER);
	  }
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**brief This API Writes
 *           Gyro wakeup trigger. When both trigger conditions are enabled,
 *           both conditions must be active to trigger the
 *			from the register 0x6C bit 3 and 4
 *             Value    Description
 *             0b00     anymotion: no / INT1 pin: no
 *             0b01     anymotion: no / INT1 pin: yes
 *             0b10     anymotion: yes / INT1 pin: no
 *             0b11     anymotion: yes / INT1 pin: yes
 *
 *  \param u8 v_gyro_wakeup_trigger_u8 :
 *    Value of the v_gyro_wakeup_trigger_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_wakeup_trigger(
u8 v_gyro_wakeup_trigger_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_gyro_wakeup_trigger_u8 <= C_BMI160_THREE_U8X) {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_GYRO_WAKEUP_TRIGGER__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_GYRO_WAKEUP_TRIGGER,
				v_gyro_wakeup_trigger_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_GYRO_WAKEUP_TRIGGER__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**brief This API Reads
 *           Target state for gyro sleep mode
 *			from the register 0x6C bit 5
 *            Value     Description
 *               0      Sleep transition to fast wake up state
 *               1      Sleep transition to suspend state
 *
 *  \param u8 * v_gyro_sleep_state_u8 :
 *      Pointer to the v_gyro_sleep_state_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_sleep_state(
u8 *v_gyro_sleep_state_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_GYRO_SLEEP_STATE__REG,
			&v_data_u8, 1);
			*v_gyro_sleep_state_u8 = BMI160_GET_BITSLICE(
			v_data_u8,
			BMI160_USER_GYRO_SLEEP_STATE);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**brief This API Writes
 *           Target state for gyro sleep mode
 *			from the register 0x6C bit 5
 *            Value     Description
 *               0      Sleep transition to fast wake up state
 *               1      Sleep transition to suspend state
 *
 *  \param u8 v_gyro_sleep_state_u8 :
 *    Value of the v_gyro_sleep_state_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_sleep_state(
u8 v_gyro_sleep_state_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_gyro_sleep_state_u8 < C_BMI160_TWO_U8X) {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_GYRO_SLEEP_STATE__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_GYRO_SLEEP_STATE,
				v_gyro_sleep_state_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_GYRO_SLEEP_STATE__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**brief This API Reads
 *           Trigger an interrupt when a gyro wakeup
 *			from the register 0x6C bit 6
 *           is triggered
 *            Value     Description
 *                0     Disabled
 *                1     Enabled
 *
 *  \param u8 * v_gyro_wakeup_intr_u8 :
 *      Pointer to the v_gyro_wakeup_intr_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_wakeup_intr(
u8 *v_gyro_wakeup_intr_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_GYRO_WAKEUP_INTR__REG,
			&v_data_u8, 1);
			*v_gyro_wakeup_intr_u8 = BMI160_GET_BITSLICE(
			v_data_u8,
			BMI160_USER_GYRO_WAKEUP_INTR);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**brief This API Writes
 *           Trigger an interrupt when a gyro wakeup
 *           is triggered
 *			from the register 0x6C bit 6
 *            Value     Description
 *                0     Disabled
 *                1     Enabled
 *
 *  \param u8 v_gyro_wakeup_intr_u8 :
 *    Value of the v_gyro_wakeup_intr_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_wakeup_intr(
u8 v_gyro_wakeup_intr_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_gyro_wakeup_intr_u8 < C_BMI160_TWO_U8X) {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_GYRO_WAKEUP_INTR__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_GYRO_WAKEUP_INTR,
				v_gyro_wakeup_intr_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_GYRO_WAKEUP_INTR__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}

/*******************************************************************************
 * Description: *//**brief This API Reads
 *          select axis to be self-tested:
 *             Value    Description
 *              0b00    disabled
 *              0b01    x-axis
 *              0b10    y-axis
 *              0b10    z-axis
 *		from the register 0x6D bit 0 and 1
 *
 *  \param u8 * v_accel_selftest_axis_u8 :
 *      Pointer to the v_accel_selftest_axis_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_selftest_axis(
u8 *v_accel_selftest_axis_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_ACCEL_SELFTEST_AXIS__REG,
			&v_data_u8, 1);
			*v_accel_selftest_axis_u8 = BMI160_GET_BITSLICE(
			v_data_u8,
			BMI160_USER_ACCEL_SELFTEST_AXIS);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**brief This API Writes
 *          select axis to be self-tested:
 *             Value    Description
 *              0b00    disabled
 *              0b01    x-axis
 *              0b10    y-axis
 *              0b10    z-axis
 *		from the register 0x6D bit 0 and 1
 *
 *  \param u8 v_accel_selftest_axis_u8 :
 *    Value of the v_accel_selftest_axis_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_selftest_axis(
u8 v_accel_selftest_axis_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_accel_selftest_axis_u8 <= C_BMI160_THREE_U8X) {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_ACCEL_SELFTEST_AXIS__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_ACCEL_SELFTEST_AXIS,
				v_accel_selftest_axis_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_ACCEL_SELFTEST_AXIS__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**brief This API Reads
 *          select sign of self-test excitation as
 *            Value     Description
 *              0       negative
 *              1       positive
 *		from the register 0x6D bit 2
 *
 *  \param u8 * v_accel_selftest_sign_u8 :
 *      Pointer to the v_accel_selftest_sign_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_selftest_sign(
u8 *v_accel_selftest_sign_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_ACCEL_SELFTEST_SIGN__REG,
			&v_data_u8, 1);
			*v_accel_selftest_sign_u8 = BMI160_GET_BITSLICE(
			v_data_u8,
			BMI160_USER_ACCEL_SELFTEST_SIGN);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**brief This API Writes
 *          select sign of self-test excitation as
 *            Value     Description
 *              0       negative
 *              1       positive
 *		from the register 0x6D bit 2
 *
 *  \param u8 v_accel_selftest_sign_u8 :
 *    Value of the v_accel_selftest_sign_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_selftest_sign(
u8 v_accel_selftest_sign_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_accel_selftest_sign_u8 < C_BMI160_TWO_U8X) {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_ACCEL_SELFTEST_SIGN__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_ACCEL_SELFTEST_SIGN,
				v_accel_selftest_sign_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_ACCEL_SELFTEST_SIGN__REG,
				&v_data_u8, 1);
			}
		} else {
			com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**brief This API Reads
 *        select amplitude of the selftest deflection:
 *                    Value     Description
 *                           0  low
 *                           1  high
 *		from the register 0x6D bit 3
 *
 *  \param u8 * v_accel_selftest_amp_u8 :
 *      Pointer to the v_accel_selftest_amp_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_selftest_amp(
u8 *v_accel_selftest_amp_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_SELFTEST_AMP__REG,
			&v_data_u8, 1);
			*v_accel_selftest_amp_u8 = BMI160_GET_BITSLICE(
			v_data_u8,
			BMI160_USER_SELFTEST_AMP);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**brief This API Writes
  *        select amplitude of the selftest deflection:
 *                    Value     Description
 *                           0  low
 *                           1  high
 *		from the register 0x6D bit 3
 *
 *
 *  \param u8 v_accel_selftest_amp_u8 :
 *    Value of the v_accel_selftest_amp_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_selftest_amp(
u8 v_accel_selftest_amp_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_accel_selftest_amp_u8 < C_BMI160_TWO_U8X) {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_SELFTEST_AMP__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_SELFTEST_AMP,
				v_accel_selftest_amp_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_SELFTEST_AMP__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}

/******************************************************************************
 * Description: *//**brief This API Reads
 *        gyr_self_test_star
 *                    Value     Description
 *                           0  low
 *                           1  high
 *		from the register 0x6D bit 4
 *
 *  \param u8 * v_gyro_self_test_start_u8:
 *      Pointer to the v_gyro_self_test_start_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_selftest_start(
u8 *v_gyro_selftest_start_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_GYRO_SELFTEST_START__REG,
			&v_data_u8, 1);
			*v_gyro_selftest_start_u8 = BMI160_GET_BITSLICE(
			v_data_u8,
			BMI160_USER_GYRO_SELFTEST_START);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**brief This API Writes
 *        select amplitude of the selftest deflection:
 *                    Value     Description
 *                           0  low
 *                           1  high
 *		from the register 0x6D bit 4
 *
 *
 *  \param u8 v_gyro_self_test_start_u8 :
 *    Value of the v_gyro_self_test_start_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_selftest_start(
u8 v_gyro_selftest_start_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
		if (v_gyro_selftest_start_u8 < C_BMI160_TWO_U8X) {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_GYRO_SELFTEST_START__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_GYRO_SELFTEST_START,
				v_gyro_selftest_start_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_GYRO_SELFTEST_START__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/******************************************************************************
 * Description: *//**brief This API Reads
 *        v_spi_enable_u8
 *                    Value     Description
 *                           0  I2C Enable
 *                           1  I2C Disable
 *		from the register 0x70 bit 0
 *
 *  \param u8 * v_spi_enable_u8:
 *      Pointer to the v_spi_enable_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_spi_enable(u8 *v_spi_enable_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_NV_CONFIG_SPI_ENABLE__REG,
			&v_data_u8, 1);
			*v_spi_enable_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_NV_CONFIG_SPI_ENABLE);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**brief This API Writes spi enable
 *		from the register 0x70 bit 0
 *
 *
 *  \param u8 v_spi_enable_u8 :
 *    Value of the v_spi_enable_u8
 *
 *                  Value     Description
 *                           0  SPI Enable
 *                           1  SPI Disable
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_spi_enable(u8 v_spi_enable_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_NV_CONFIG_SPI_ENABLE__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_NV_CONFIG_SPI_ENABLE,
				v_spi_enable_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC
				(p_bmi160->dev_addr,
				BMI160_USER_NV_CONFIG_SPI_ENABLE__REG,
				&v_data_u8, 1);
			}
		}
	return com_rslt;
}
/******************************************************************************
 * Description: *//**brief This API Reads
 *	v_spare0_trim_u8 from the register 0x70 bit 3
 *
 *
 *
 *  \param u8 * v_spare0_trim_u8:
 *      Pointer to the v_spare0_trim_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_spare0_trim(u8 *v_spare0_trim_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_NV_CONFIG_SPARE0__REG,
			&v_data_u8, 1);
			*v_spare0_trim_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_NV_CONFIG_SPARE0);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**brief This API Writes
 *	v_spare0_trim_u8 from the register 0x70 bit 3
 *
 *  \param u8 v_spare0_trim_u8 :
 *    Value of the v_spare0_trim_u8
 *
 *
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_spare0_trim(u8 v_spare0_trim_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_NV_CONFIG_SPARE0__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_NV_CONFIG_SPARE0,
				v_spare0_trim_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_NV_CONFIG_SPARE0__REG,
				&v_data_u8, 1);
			}
		}
	return com_rslt;
}
/******************************************************************************
 *	Description: *//**brief This API Reads
 *	v_nvm_counter_u8 from the register 0x70 bit 4 to 7
 *
 *
 *
 *  \param u8 * v_nvm_counter_u8:
 *      Pointer to the v_nvm_counter_u8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_nvm_counter(u8 *v_nvm_counter_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_NV_CONFIG_NVM_COUNTER__REG,
			&v_data_u8, 1);
			*v_nvm_counter_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_NV_CONFIG_NVM_COUNTER);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API Writes
 *	v_nvm_counter_u8 from the register 0x70 bit 4
 *
 *
 *  \param u8 v_nvm_counter_u8 :
 *    Value of the v_nvm_counter_u8
 *
 *
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_nvm_counter(
u8 v_nvm_counter_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_NV_CONFIG_NVM_COUNTER__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_NV_CONFIG_NVM_COUNTER,
				v_nvm_counter_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_NV_CONFIG_NVM_COUNTER__REG,
				&v_data_u8, 1);
			}
		}
	return com_rslt;
}
/******************************************************************************
 *	Description: *//**brief This API Reads
 *	v_accel_off_x_s8 from the register 0x71 bit 0 to 7
 *
 *
 *
 *  \param s8 * v_accel_off_x_s8:
 *      Pointer to the v_accel_off_x_s8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_off_comp_xaxis(
s8 *v_accel_off_x_s8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_OFFSET_0_ACCEL_OFF_X__REG,
			&v_data_u8, 1);
			*v_accel_off_x_s8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_OFFSET_0_ACCEL_OFF_X);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API Writes
 *	v_accel_off_x_s8 from the register 0x71 bit 0 to 7
 *
 *  \param s8 v_accel_off_x_s8 :
 *    Value of the v_accel_off_x_s8
 *
 *
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_offset_compensation_xaxis(
s8 v_accel_off_x_s8)
{
BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
u8 v_data_u8 = C_BMI160_ZERO_U8X;
u8 v_status_s8 = SUCCESS;
if (p_bmi160 == BMI160_NULL) {
	return E_BMI160_NULL_PTR;
	} else {
		v_status_s8 = bmi160_set_accel_offset_enable(
		ACCEL_OFFSET_ENABLE);
		if (v_status_s8 == SUCCESS) {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_OFFSET_0_ACCEL_OFF_X__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 =
				BMI160_SET_BITSLICE(
				v_data_u8,
				BMI160_USER_OFFSET_0_ACCEL_OFF_X,
				v_accel_off_x_s8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_OFFSET_0_ACCEL_OFF_X__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt =  ERROR;
		}
	}
	return com_rslt;
}
/******************************************************************************
 *	Description: *//**brief This API Reads
 *	v_accel_off_y_s8 from the register 0x72 bit 0 to 7
 *
 *
 *
 *  \param s8 * v_accel_off_y_s8:
 *      Pointer to the v_accel_off_y_s8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_off_comp_yaxis(
s8 *v_accel_off_y_s8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_OFFSET_1_ACCEL_OFF_Y__REG,
			&v_data_u8, 1);
			*v_accel_off_y_s8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_OFFSET_1_ACCEL_OFF_Y);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API Writes
 *	v_accel_off_y_s8 from the register 0x72 bit 0 to 7
 *
 *  \param s8 v_accel_off_y_s8 :
 *    Value of the v_accel_off_y_s8
 *
 *
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_offset_compensation_yaxis(
s8 v_accel_off_y_s8)
{
BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
u8 v_data_u8 = C_BMI160_ZERO_U8X;
u8 v_status_s8 = SUCCESS;
if (p_bmi160 == BMI160_NULL) {
	return E_BMI160_NULL_PTR;
	} else {
		v_status_s8 = bmi160_set_accel_offset_enable(
		ACCEL_OFFSET_ENABLE);
		if (v_status_s8 == SUCCESS) {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_OFFSET_1_ACCEL_OFF_Y__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 =
				BMI160_SET_BITSLICE(
				v_data_u8,
				BMI160_USER_OFFSET_1_ACCEL_OFF_Y,
				v_accel_off_y_s8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_OFFSET_1_ACCEL_OFF_Y__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/******************************************************************************
 *	Description: *//**brief This API Reads
 *	v_accel_off_z_s8 from the register 0x72 bit 0 to 7
 *
 *
 *
 *  \param s8 * v_accel_off_z_s8:
 *      Pointer to the v_accel_off_z_s8
 *
 *
 *
 *  \return results of communication routine
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_off_comp_zaxis(
s8 *v_accel_off_z_s8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_OFFSET_2_ACCEL_OFF_Z__REG,
			&v_data_u8, 1);
			*v_accel_off_z_s8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_OFFSET_2_ACCEL_OFF_Z);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API Writes
 *	v_accel_off_z_s8 from the register 0x72 bit 0 to 7
 *
 *  \param s8 v_accel_off_z_s8 :
 *    Value of the v_accel_off_z_s8
 *
 *
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_offset_compensation_zaxis(
s8 v_accel_off_z_s8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	u8 v_status_s8 = SUCCESS;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			v_status_s8 = bmi160_set_accel_offset_enable(
			ACCEL_OFFSET_ENABLE);
			if (v_status_s8 == SUCCESS) {
				com_rslt =
				p_bmi160->BMI160_BUS_READ_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_OFFSET_2_ACCEL_OFF_Z__REG,
				&v_data_u8, 1);
				if (com_rslt == SUCCESS) {
					v_data_u8 =
					BMI160_SET_BITSLICE(v_data_u8,
					BMI160_USER_OFFSET_2_ACCEL_OFF_Z,
					v_accel_off_z_s8);
					com_rslt +=
					p_bmi160->BMI160_BUS_WRITE_FUNC(
					p_bmi160->dev_addr,
					BMI160_USER_OFFSET_2_ACCEL_OFF_Z__REG,
					&v_data_u8, 1);
				}
			} else {
			com_rslt = ERROR;
			}
		}
	return com_rslt;
}
/******************************************************************************
 *	Description: *//**brief This API Reads
 *	v_gyro_off_x_s16 from the register 0x74 bit 0 to 7
 *
 *
 *
 *
 *  \param u8 * v_gyro_off_x_s16:
 *      Pointer to the v_gyro_off_x_s16
 *
 *
 *
 *  \return results of communication routine
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_off_comp_xaxis(
s16 *v_gyro_off_x_s16)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data1_u8r = C_BMI160_ZERO_U8X;
	u8 v_data2_u8r = C_BMI160_ZERO_U8X;
	s16 v_data3_u8r, v_data4_u8r = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_OFFSET_3_GYRO_OFF_X__REG,
			&v_data1_u8r, 1);
			v_data1_u8r = BMI160_GET_BITSLICE(v_data1_u8r,
			BMI160_USER_OFFSET_3_GYRO_OFF_X);
			com_rslt += p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_OFFSET_6_GYRO_OFF_X__REG,
			&v_data2_u8r, 1);
			v_data2_u8r = BMI160_GET_BITSLICE(v_data2_u8r,
			BMI160_USER_OFFSET_6_GYRO_OFF_X);
			v_data3_u8r = v_data2_u8r << C_BMI160_FOURTEEN_U8X;
			v_data4_u8r =  v_data1_u8r << C_BMI160_SIX_U8X;
			v_data3_u8r = v_data3_u8r | v_data4_u8r;
			*v_gyro_off_x_s16 = v_data3_u8r >> C_BMI160_SIX_U8X;
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API Writes
 *	v_gyro_off_x_s16 from the register 0x74 bit 0 to 7
 *
 *  \param u8 v_gyro_off_x_s16 :
 *    Value of the v_gyro_off_x_s16
 *
 *
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_offset_compensation_xaxis(
s16 v_gyro_off_x_s16)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data1_u8r, v_data2_u8r = C_BMI160_ZERO_U8X;
	u16 v_data3_u8r = C_BMI160_ZERO_U8X;
	u8 v_status_s8 = SUCCESS;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			v_status_s8 = bmi160_set_gyro_offset_enable(
			GYRO_OFFSET_ENABLE);
			if (v_status_s8 == SUCCESS) {
				com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_OFFSET_3_GYRO_OFF_X__REG,
				&v_data2_u8r, 1);
				if (com_rslt == SUCCESS) {
					v_data1_u8r =
					((s8) (v_gyro_off_x_s16 & 0x00FF));
					v_data2_u8r = BMI160_SET_BITSLICE(
					v_data2_u8r,
					BMI160_USER_OFFSET_3_GYRO_OFF_X,
					v_data1_u8r);
					com_rslt +=
					p_bmi160->BMI160_BUS_WRITE_FUNC(
					p_bmi160->dev_addr,
					BMI160_USER_OFFSET_3_GYRO_OFF_X__REG,
					&v_data2_u8r, 1);
				}

				com_rslt += p_bmi160->BMI160_BUS_READ_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_OFFSET_6_GYRO_OFF_X__REG,
				&v_data2_u8r, 1);
				if (com_rslt == SUCCESS) {
					v_data3_u8r =
					(u16) (v_gyro_off_x_s16 & 0x0300);
					v_data1_u8r = (u8)(v_data3_u8r
					>> BMI160_SHIFT_8_POSITION);
					v_data2_u8r = BMI160_SET_BITSLICE(
					v_data2_u8r,
					BMI160_USER_OFFSET_6_GYRO_OFF_X,
					v_data1_u8r);
					com_rslt +=
					p_bmi160->BMI160_BUS_WRITE_FUNC(
					p_bmi160->dev_addr,
					BMI160_USER_OFFSET_6_GYRO_OFF_X__REG,
					&v_data2_u8r, 1);
				}
			} else {
			return ERROR;
			}
		}
	return com_rslt;
}
/******************************************************************************
 *	Description: *//**brief This API Reads
 *	v_gyro_off_y_s16 from the register 0x75 bit 0 to 7
 *
 *
 *
 *  \param u8 * v_gyro_off_y_s16:
 *      Pointer to the v_gyro_off_y_s16
 *
 *
 *
 *  \return results of communication routine
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_off_comp_yaxis(
s16 *v_gyro_off_y_s16)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data1_u8r = C_BMI160_ZERO_U8X;
	u8 v_data2_u8r = C_BMI160_ZERO_U8X;
	s16 v_data3_u8r, v_data4_u8r = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_OFFSET_4_GYRO_OFF_Y__REG,
			&v_data1_u8r, 1);
			v_data1_u8r = BMI160_GET_BITSLICE(v_data1_u8r,
			BMI160_USER_OFFSET_4_GYRO_OFF_Y);
			com_rslt += p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_OFFSET_6_GYRO_OFF_Y__REG,
			&v_data2_u8r, 1);
			v_data2_u8r = BMI160_GET_BITSLICE(v_data2_u8r,
			BMI160_USER_OFFSET_6_GYRO_OFF_Y);
			v_data3_u8r = v_data2_u8r << C_BMI160_FOURTEEN_U8X;
			v_data4_u8r =  v_data1_u8r << C_BMI160_SIX_U8X;
			v_data3_u8r = v_data3_u8r | v_data4_u8r;
			*v_gyro_off_y_s16 = v_data3_u8r >> C_BMI160_SIX_U8X;
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API Writes
 *	v_gyro_off_y_s16 from the register 0x75 bit 0 to 7
 *
 *  \param u8 v_gyro_off_y_s16 :
 *    Value of the v_gyro_off_y_s16
 *
 *
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_offset_compensation_yaxis(
s16 v_gyro_off_y_s16)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data1_u8r, v_data2_u8r = C_BMI160_ZERO_U8X;
	u16 v_data3_u8r = C_BMI160_ZERO_U8X;
	u8 v_status_s8 = SUCCESS;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			v_status_s8 = bmi160_set_gyro_offset_enable(
			GYRO_OFFSET_ENABLE);
			if (v_status_s8 == SUCCESS) {
				com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
				(p_bmi160->dev_addr,
				BMI160_USER_OFFSET_4_GYRO_OFF_Y__REG,
				&v_data2_u8r, 1);
				if (com_rslt == SUCCESS) {
					v_data1_u8r =
					((s8) (v_gyro_off_y_s16 & 0x00FF));
					v_data2_u8r = BMI160_SET_BITSLICE(
					v_data2_u8r,
					BMI160_USER_OFFSET_4_GYRO_OFF_Y,
					v_data1_u8r);
					com_rslt +=
					p_bmi160->BMI160_BUS_WRITE_FUNC
					(p_bmi160->dev_addr,
					BMI160_USER_OFFSET_4_GYRO_OFF_Y__REG,
					&v_data2_u8r, 1);
				}

				com_rslt += p_bmi160->BMI160_BUS_READ_FUNC
				(p_bmi160->dev_addr,
				BMI160_USER_OFFSET_6_GYRO_OFF_Y__REG,
				&v_data2_u8r, 1);
				if (com_rslt == SUCCESS) {
					v_data3_u8r =
					(u16) (v_gyro_off_y_s16 & 0x0300);
					v_data1_u8r = (u8)(v_data3_u8r
					>> BMI160_SHIFT_8_POSITION);
					v_data2_u8r = BMI160_SET_BITSLICE(
					v_data2_u8r,
					BMI160_USER_OFFSET_6_GYRO_OFF_Y,
					v_data1_u8r);
					com_rslt +=
					p_bmi160->BMI160_BUS_WRITE_FUNC
					(p_bmi160->dev_addr,
					BMI160_USER_OFFSET_6_GYRO_OFF_Y__REG,
					&v_data2_u8r, 1);
				}
			} else {
			return ERROR;
			}
		}
	return com_rslt;
}
/******************************************************************************
 *	Description: *//**brief This API Reads
 *	v_gyro_off_z_s16 from the register 0x76 bit 0 to 7
 *
 *
 *
 *  \param u8 * v_gyro_off_z_s16:
 *      Pointer to the v_gyro_off_z_s16
 *
 *
 *
 *  \return results of communication routine
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_off_comp_zaxis(
s16 *v_gyro_off_z_s16)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data1_u8r = C_BMI160_ZERO_U8X;
	u8 v_data2_u8r = C_BMI160_ZERO_U8X;
	s16 v_data3_u8r, v_data4_u8r = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_OFFSET_5_GYRO_OFF_Z__REG,
			&v_data1_u8r, 1);
			v_data1_u8r = BMI160_GET_BITSLICE
			(v_data1_u8r,
			BMI160_USER_OFFSET_5_GYRO_OFF_Z);
			com_rslt +=
			p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_OFFSET_6_GYRO_OFF_Z__REG,
			&v_data2_u8r, 1);
			v_data2_u8r = BMI160_GET_BITSLICE(
			v_data2_u8r,
			BMI160_USER_OFFSET_6_GYRO_OFF_Z);
			v_data3_u8r = v_data2_u8r << C_BMI160_FOURTEEN_U8X;
			v_data4_u8r =  v_data1_u8r << C_BMI160_SIX_U8X;
			v_data3_u8r = v_data3_u8r | v_data4_u8r;
			*v_gyro_off_z_s16 = v_data3_u8r >> C_BMI160_SIX_U8X;
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API Writes
 *	v_gyro_off_z_s16 from the register 0x76 bit 0 to 7
 *
 *  \param u8 v_gyro_off_z_s16 :
 *    Value of the v_gyro_off_z_s16
 *
 *
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_offset_compensation_zaxis(
s16 v_gyro_off_z_s16)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data1_u8r, v_data2_u8r = C_BMI160_ZERO_U8X;
	u16 v_data3_u8r = C_BMI160_ZERO_U8X;
	u8 v_status_s8 = SUCCESS;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			v_status_s8 = bmi160_set_gyro_offset_enable(
			GYRO_OFFSET_ENABLE);
			if (v_status_s8 == SUCCESS) {
				com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
				(p_bmi160->dev_addr,
				BMI160_USER_OFFSET_5_GYRO_OFF_Z__REG,
				&v_data2_u8r, 1);
				if (com_rslt == SUCCESS) {
					v_data1_u8r =
					((u8) (v_gyro_off_z_s16 & 0x00FF));
					v_data2_u8r = BMI160_SET_BITSLICE(
					v_data2_u8r,
					BMI160_USER_OFFSET_5_GYRO_OFF_Z,
					v_data1_u8r);
					com_rslt +=
					p_bmi160->BMI160_BUS_WRITE_FUNC
					(p_bmi160->dev_addr,
					BMI160_USER_OFFSET_5_GYRO_OFF_Z__REG,
					&v_data2_u8r, 1);
				}

				com_rslt += p_bmi160->BMI160_BUS_READ_FUNC
				(p_bmi160->dev_addr,
				BMI160_USER_OFFSET_6_GYRO_OFF_Z__REG,
				&v_data2_u8r, 1);
				if (com_rslt == SUCCESS) {
					v_data3_u8r =
					(u16) (v_gyro_off_z_s16 & 0x0300);
					v_data1_u8r = (u8)(v_data3_u8r
					>> BMI160_SHIFT_8_POSITION);
					v_data2_u8r = BMI160_SET_BITSLICE(
					v_data2_u8r,
					BMI160_USER_OFFSET_6_GYRO_OFF_Z,
					v_data1_u8r);
					com_rslt +=
					p_bmi160->BMI160_BUS_WRITE_FUNC
					(p_bmi160->dev_addr,
					BMI160_USER_OFFSET_6_GYRO_OFF_Z__REG,
					&v_data2_u8r, 1);
				}
			} else {
			return ERROR;
			}
		}
	return com_rslt;
}
/******************************************************************************
 *	Description: *//**brief This API Reads
 *	v_accel_off_enable_u8 from the register 0x77 bit 6
 *
 *
 *
 *  \param u8 * v_accel_off_enable_u8:
 *      Pointer to the v_accel_off_enable_u8
 *                   0 - Disabled
 *                   1 - Enabled
 *
 *
 *
 *  \return results of communication routine
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_offset_enable(
u8 *v_accel_off_enable_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_OFFSET_6_ACCEL_OFF_ENABLE__REG,
			&v_data_u8, 1);
			*v_accel_off_enable_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_OFFSET_6_ACCEL_OFF_ENABLE);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**brief This API Writes
 *	v_accel_off_enable_u8 from the register 0x77 bit 6
 *
 *  \param u8 v_accel_off_enable_u8 :
 *    Value of the v_accel_off_enable_u8
 *
 *
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_offset_enable(
u8 v_accel_off_enable_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
			} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_OFFSET_6_ACCEL_OFF_ENABLE__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_OFFSET_6_ACCEL_OFF_ENABLE,
				v_accel_off_enable_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_OFFSET_6_ACCEL_OFF_ENABLE__REG,
				&v_data_u8, 1);
			}
		}
	return com_rslt;
}
/******************************************************************************
 * Description: *//**brief This API Reads
 *	v_gyro_off_enable_u8 from the register 0x77 bit 7
 *
 *
 *
 *  \param u8 * v_gyro_off_enable_u8:
 *      Pointer to the v_gyro_off_enable_u8
 *                   0 - Disabled
 *                   1 - Enabled
 *
 *
 *
 *  \return results of communication routine
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_offset_enable(
u8 *v_gyro_off_enable_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_OFFSET_6_GYRO_OFF_EN__REG,
			&v_data_u8, 1);
			*v_gyro_off_enable_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_OFFSET_6_GYRO_OFF_EN);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API Writes
 *	v_gyro_off_enable_u8 from the register 0x77 bit 7
 *
 *  \param u8 v_gyro_off_enable_u8 :
 *    Value of the v_gyro_off_enable_u8
 *
 *
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_offset_enable(
u8 v_gyro_off_enable_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_OFFSET_6_GYRO_OFF_EN__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_OFFSET_6_GYRO_OFF_EN,
				v_gyro_off_enable_u8);
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
				p_bmi160->dev_addr,
				BMI160_USER_OFFSET_6_GYRO_OFF_EN__REG,
				&v_data_u8, 1);
			}
		}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**brief This API reads step counter value
 *	form the register 0x78 and 0x79
 *
 *
 *
 *
 *  \param s16 * v_step_cnt_s16 : Pointer holding the value of v_step_cnt_s16
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_read_step_count(s16 *v_step_cnt_s16)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 a_data_u8r[2] = {0, 0};
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt =
			p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
			BMI160_USER_STEP_COUNT_LSB__REG,
			a_data_u8r, 2);

			*v_step_cnt_s16 = (s16)
			((((s32)((s8)a_data_u8r[1]))
			<< BMI160_SHIFT_8_POSITION) | (a_data_u8r[0]));
		}
	return com_rslt;
}
/******************************************************************************
 *	Description: *//**brief This API Reads
 *	step counter configuration
 *	from the register 0x7A bit 0 to 7
 *	and from the register 0x7B bit 0 to 2 and 4 to 7
 *
 *
 *  \param u16 * v_step_config_u16   :
 *      Pointer holding the value of  step configuration
 *
 *  \return results of communication routine
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_step_config(
u16 *v_step_config_u16)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data1_u8r = C_BMI160_ZERO_U8X;
	u8 v_data2_u8r = C_BMI160_ZERO_U8X;
	u16 v_data3_u8r = C_BMI160_ZERO_U8X;
	/* Read the 0 to 7 bit*/
	com_rslt =
	p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
	BMI160_USER_STEP_CONFIG_ZERO__REG,
	&v_data1_u8r, 1);
	/* Read the 8 to 10 bit*/
	com_rslt +=
	p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
	BMI160_USER_STEP_CONFIG_ONE_CNF1__REG,
	&v_data2_u8r, 1);
	v_data2_u8r = BMI160_GET_BITSLICE(v_data2_u8r,
	BMI160_USER_STEP_CONFIG_ONE_CNF1);
	v_data3_u8r = ((u16)((((u32)
	((u8)v_data2_u8r))
	<< BMI160_SHIFT_8_POSITION) | (v_data1_u8r)));
	/* Read the 11 to 14 bit*/
	com_rslt +=
	p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
	BMI160_USER_STEP_CONFIG_ONE_CNF2__REG,
	&v_data1_u8r, 1);
	v_data1_u8r = BMI160_GET_BITSLICE(v_data1_u8r,
	BMI160_USER_STEP_CONFIG_ONE_CNF2);
	*v_step_config_u16 = ((u16)((((u32)
	((u8)v_data1_u8r))
	<< BMI160_SHIFT_8_POSITION) | (v_data3_u8r)));

	return com_rslt;
}
/******************************************************************************
 *	Description: *//**brief This API Reads
 *	step counter configuration
 *	from the register 0x7A bit 0 to 7
 *	and from the register 0x7B bit 0 to 2 and 4 to 7
 *
 *
 *  \param u8  v_step_config_u16   :
 *      the value of  Enable step configuration
 *
 *  \return results of communication routine
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_step_config(
u16 v_step_config_u16)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data1_u8r = C_BMI160_ZERO_U8X;
	u8 v_data2_u8r = C_BMI160_ZERO_U8X;
	u16 v_data3_u16 = C_BMI160_ZERO_U8X;

	/* write the 0 to 7 bit*/
	v_data1_u8r = (u8)(v_step_config_u16 & 0x00FF);
	p_bmi160->BMI160_BUS_WRITE_FUNC
	(p_bmi160->dev_addr,
	BMI160_USER_STEP_CONFIG_ZERO__REG,
	&v_data1_u8r, 1);
	/* write the 8 to 10 bit*/
	com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
	(p_bmi160->dev_addr,
	BMI160_USER_STEP_CONFIG_ONE_CNF1__REG,
	&v_data2_u8r, 1);
	if (com_rslt == SUCCESS) {
		v_data3_u16 = (u16) (v_step_config_u16 & 0x0700);
		v_data1_u8r = (u8)(v_data3_u16 >> BMI160_SHIFT_8_POSITION);
		v_data2_u8r = BMI160_SET_BITSLICE(v_data2_u8r,
		BMI160_USER_STEP_CONFIG_ONE_CNF1, v_data1_u8r);
		p_bmi160->BMI160_BUS_WRITE_FUNC
		(p_bmi160->dev_addr,
		BMI160_USER_STEP_CONFIG_ONE_CNF1__REG,
		&v_data2_u8r, 1);
	}
	/* write the 11 to 14 bit*/
	com_rslt += p_bmi160->BMI160_BUS_READ_FUNC
	(p_bmi160->dev_addr,
	BMI160_USER_STEP_CONFIG_ONE_CNF2__REG,
	&v_data2_u8r, 1);
	if (com_rslt == SUCCESS) {
		v_data3_u16 = (u16) (v_step_config_u16 & 0xF000);
		v_data1_u8r = (u8)(v_data3_u16 >> BMI160_SHIFT_12_POSITION);
		v_data2_u8r = BMI160_SET_BITSLICE(v_data2_u8r,
		BMI160_USER_STEP_CONFIG_ONE_CNF2, v_data1_u8r);
		p_bmi160->BMI160_BUS_WRITE_FUNC
		(p_bmi160->dev_addr,
		BMI160_USER_STEP_CONFIG_ONE_CNF2__REG,
		&v_data2_u8r, 1);
	}

	return com_rslt;
}
/******************************************************************************
 *	Description: *//**brief This API Reads
 *	Enable step counter
 *	from the register 0x7B bit 3
 *
 *
 *  \param u8 * v_step_counter_u8   :
 *      Pointer holding the value of  Enable step counter
 *
 *  \return results of communication routine
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_step_counter_enable(
u8 *v_step_counter_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_USER_STEP_CONFIG_1_STEP_COUNT_ENABLE__REG,
			&v_data_u8, 1);
			*v_step_counter_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_STEP_CONFIG_1_STEP_COUNT_ENABLE);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/******************************************************************************
 *	Description: *//**brief This API Reads
 *	Enable step counter
 *	from the register 0x7B bit 3
 *
 *
 *  \param u8 * v_step_counter_u8   :
 *      Pointer holding the value of  Enable step counter
 *
 *  \return results of communication routine
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_step_counter_enable(u8 v_step_counter_u8)
{
BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
u8 v_data_u8 = C_BMI160_ZERO_U8X;
if (p_bmi160 == BMI160_NULL) {
	return E_BMI160_NULL_PTR;
} else {
	if (v_step_counter_u8 < C_BMI160_THREE_U8X) {
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
		(p_bmi160->dev_addr,
		BMI160_USER_STEP_CONFIG_1_STEP_COUNT_ENABLE__REG,
		&v_data_u8, 1);
		if (com_rslt == SUCCESS) {
			v_data_u8 =
			BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_STEP_CONFIG_1_STEP_COUNT_ENABLE,
			v_step_counter_u8);
			com_rslt +=
			p_bmi160->BMI160_BUS_WRITE_FUNC
			(p_bmi160->dev_addr,
			BMI160_USER_STEP_CONFIG_1_STEP_COUNT_ENABLE__REG,
			&v_data_u8, 1);
		}
	} else {
	com_rslt = E_BMI160_OUT_OF_RANGE;
	}
}
	return com_rslt;
}
/******************************************************************************
 *	Description: *//**brief This API set
 *	Step counter modes
 *
 *
 *  \param u8 v_step_mode_u8   :
 *      The value of step counter mode
 *	BMI160_STEP_NORMAL_MODE		-	0
 *	BMI160_STEP_SENSITIVE_MODE	-	1
 *	BMI160_STEP_ROBUST_MODE		-	2
 *
 *  \return results of communication routine
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_step_mode(u8 v_step_mode_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	switch (v_step_mode_u8) {
	case BMI160_STEP_NORMAL_MODE:
		com_rslt = bmi160_set_step_config(
		STEP_CONFIG_NORMAL);
		p_bmi160->delay_msec(1);
	break;
	case BMI160_STEP_SENSITIVE_MODE:
		com_rslt = bmi160_set_step_config(
		STEP_CONFIG_SENSITIVE);
		p_bmi160->delay_msec(1);
	break;
	case BMI160_STEP_ROBUST_MODE:
		com_rslt = bmi160_set_step_config(
		STEP_CONFIG_ROBUST);
		p_bmi160->delay_msec(1);
	break;
	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
	break;
	}

	return com_rslt;
}
/******************************************************************************
 *	Description: *//**brief This API used to trigger the  signification motion
 *	interrupt
 *
 *
 *  \param u8 v_significant_u8   : The value of interrupt selection
 *
 *      BMI160_MAP_INTR1	0
 *      BMI160_MAP_INTR2	1
 *
 *  \return results of communication routine
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_map_significant_motion_intrerrupt(
u8 v_significant_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_sig_motion_u8 = C_BMI160_ZERO_U8X;
	u8 v_any_motion_intr1_stat_u8 = 0x04;
	u8 v_any_motion_intr2_stat_u8 = 0x04;
	u8 v_any_motion_axis_stat_u8 = 0x07;

	com_rslt = bmi160_get_intr_significant_motion_select(&v_sig_motion_u8);
	if (v_sig_motion_u8 != C_BMI160_ONE_U8X)
		com_rslt += bmi160_set_intr_significant_motion_select(0x01);
	switch (v_significant_u8) {
	case BMI160_MAP_INTR1:
		/* map the signification interrupt to any-motion interrupt1*/
		com_rslt = bmi160_write_reg(
		BMI160_USER_INTR_MAP_0_INTR1_ANY_MOTION__REG,
		&v_any_motion_intr1_stat_u8, 1);
		p_bmi160->delay_msec(1);
		com_rslt += bmi160_write_reg(
		BMI160_USER_INTR_ENABLE_0_ADDR,
		&v_any_motion_axis_stat_u8, 1);
		p_bmi160->delay_msec(1);
	break;

	case BMI160_MAP_INTR2:
		/* map the signification interrupt to any-motion interrupt2*/
		com_rslt = bmi160_write_reg(
		BMI160_USER_INTR_MAP_2_INTR2_ANY_MOTION__REG,
		&v_any_motion_intr2_stat_u8, 1);
		p_bmi160->delay_msec(1);
		com_rslt += bmi160_write_reg(
		BMI160_USER_INTR_ENABLE_0_ADDR,
		&v_any_motion_axis_stat_u8, 1);
		p_bmi160->delay_msec(1);
	break;

	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
	break;

	}
	return com_rslt;
}
/******************************************************************************
 *	Description: *//**brief This API used to trigger the  step detector
 *	interrupt
 *
 *
 *  \param u8 v_step_detector_u8   : The value of interrupt selection
 *
 *      BMI160_MAP_INTR1	0
 *      BMI160_MAP_INTR2	1
 *
 *  \return results of communication routine
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_map_step_detector_intr(
u8 v_step_detector_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_step_det_u8 = C_BMI160_ZERO_U8X;
	u8 v_low_g_intr_u81_stat_u8 = 0x01;
	u8 v_low_g_intr_u82_stat_u8 = 0x01;
	u8 v_low_g_enable_u8 = 0x08;
	/* read the v_status_s8 of step detector interrupt*/
	com_rslt = bmi160_get_step_detector_enable(&v_step_det_u8);
	if (v_step_det_u8 != C_BMI160_ONE_U8X)
		com_rslt += bmi160_set_step_detector_enable(C_BMI160_ONE_U8X);
	switch (v_step_detector_u8) {
	case BMI160_MAP_INTR1:
		/* map the step detector interrupt
		to Low-g interrupt 1*/
		com_rslt = bmi160_write_reg(
		BMI160_USER_INTR_MAP_0_INTR1_LOW_G__REG,
		&v_low_g_intr_u81_stat_u8, 1);
		p_bmi160->delay_msec(1);
		/* Enable the Low-g interrupt*/
		com_rslt += bmi160_write_reg(
		BMI160_USER_INTR_ENABLE_1_LOW_G_ENABLE__REG,
		&v_low_g_enable_u8, 1);

		p_bmi160->delay_msec(1);
	break;
	case BMI160_MAP_INTR2:
		/* map the step detector interrupt
		to Low-g interrupt 1*/
		com_rslt = bmi160_write_reg(
		BMI160_USER_INTR_MAP_2_INTR2_LOW_G__REG,
		&v_low_g_intr_u82_stat_u8, 1);
		p_bmi160->delay_msec(1);
		/* Enable the Low-g interrupt*/
		com_rslt += bmi160_write_reg(
		BMI160_USER_INTR_ENABLE_1_LOW_G_ENABLE__REG,
		&v_low_g_enable_u8, 1);
		p_bmi160->delay_msec(1);
	break;
	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
	break;
	}
	return com_rslt;
}
/******************************************************************************
 *	Description: *//**brief This API used to clear the step counter interrupt
 *	interrupt
 *
 *
 *  \param  : None
 *
 *
 *  \return results of communication routine
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_clear_step_counter(void)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	/* clear the step counter*/
	com_rslt = bmi160_set_command_register(0xB2);
	p_bmi160->delay_msec(5);

	return com_rslt;

}
/*******************************************************************************
 *	Description: *//**brief This API Writes
 *	from the register 0x7E bit 0 to 7
 *
 *
 *  \param u8 v_command_reg_u8 :
 *    Value of the v_command_reg_u8
 *
 *
 *	0x00	-	Reserved
 *  0x03	-	Starts fast offset calibration for the accel and gyro
 *	0x10	-	Sets the PMU mode for the Accelerometer to suspend
 *	0x11	-	Sets the PMU mode for the Accelerometer to normal
 *	0x12	-	Sets the PMU mode for the Accelerometer Lowpower
 *  0x14	-	Sets the PMU mode for the Gyroscope to suspend
 *	0x15	-	Sets the PMU mode for the Gyroscope to normal
 *	0x16	-	Reserved
 *	0x17	-	Sets the PMU mode for the Gyroscope to fast start-up
 *  0x18	-	Sets the PMU mode for the Magnetometer to suspend
 *	0x19	-	Sets the PMU mode for the Magnetometer to normal
 *	0x1A	-	Sets the PMU mode for the Magnetometer to Lowpower
 *	0xB0	-	Clears all data in the FIFO
 *  0xB1	-	Resets the interrupt engine
 *	0xB2	-	step_cnt_clr Clears the step counter
 *	0xB6	-	Triggers a reset
 *	0x37	-	See extmode_en_last
 *	0x9A	-	See extmode_en_last
 *	0xC0	-	Enable the extended mode
 *  0xC4	-	Erase NVM cell
 *	0xC8	-	Load NVM cell
 *	0xF0	-	Reset acceleration data path
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_command_register(u8 v_command_reg_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	} else {
			com_rslt = p_bmi160->BMI160_BUS_WRITE_FUNC(
			p_bmi160->dev_addr,
			BMI160_CMD_COMMANDS__REG,
			&v_command_reg_u8, 1);
		}
	return com_rslt;
}
/******************************************************************************
 *	Description: *//**brief This API Reads
 *	target page from the register 0x7F bit 4 and 5
 *
 *
 *
 *  \param u8 * v_target_page_u8:
 *      Pointer to the v_target_page_u8
 *                   00 - User data/configure page
 *                   01 - Chip level trim/test page
 *					 10	- Accelerometer trim
 *					 11 - Gyroscope trim
 *
 *  \return results of communication routine
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_target_page(u8 *v_target_page_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_CMD_TARGET_PAGE__REG,
			&v_data_u8, 1);
			*v_target_page_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_CMD_TARGET_PAGE);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API Writes
 *	target page from the register 0x7F bit 4 and 5
 *
 *  \param u8 v_target_page_u8 :
 *    Value of the v_target_page_u8
 *
 *
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_target_page(u8 v_target_page_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	} else {
		if (v_target_page_u8 < C_BMI160_FOUR_U8X) {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_CMD_TARGET_PAGE__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 =
				BMI160_SET_BITSLICE(v_data_u8,
				BMI160_CMD_TARGET_PAGE,
				v_target_page_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC
				(p_bmi160->dev_addr,
				BMI160_CMD_TARGET_PAGE__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/******************************************************************************
 *	Description: *//**brief This API Reads
 *	page enable from the register 0x7F bit 7
 *
 *
 *
 *  \param u8 * v_page_enable_u8:
 *      Pointer to the v_page_enable_u8
 *                   0 - Disabled
 *                   1 - Enabled
 *
 *
 *
 *  \return results of communication routine
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_paging_enable(u8 *v_page_enable_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_CMD_PAGING_EN__REG,
			&v_data_u8, 1);
			*v_page_enable_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_CMD_PAGING_EN);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *	Description: *//**brief This API Writes
 *	page enable from the register 0x7F bit 7
 *
 *  \param u8 v_page_enable_u8 :
 *    Value of the v_page_enable_u8
 *
 *				     0 - Disabled
 *                   1 - Enabled
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_paging_enable(
u8 v_page_enable_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	} else {
		if (v_page_enable_u8 < C_BMI160_TWO_U8X) {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_CMD_PAGING_EN__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 =
				BMI160_SET_BITSLICE(v_data_u8,
				BMI160_CMD_PAGING_EN,
				v_page_enable_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC
				(p_bmi160->dev_addr,
				BMI160_CMD_PAGING_EN__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/******************************************************************************
 *	Description: *//**brief This API Reads
 *	pull up configuration from the register 0X85 bit 4 an 5
 *
 *
 *
 *  \param u8 * v_control_pullup_u8:
 *      Pointer to the v_control_pullup_u8
 *                   0 - Disabled
 *                   1 - Enabled
 *
 *
 *
 *  \return results of communication routine
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_pullup_configuration(
u8 *v_control_pullup_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	} else {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
			p_bmi160->dev_addr,
			BMI160_COM_C_TRIM_FIVE__REG,
			&v_data_u8, 1);
			*v_control_pullup_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_COM_C_TRIM_FIVE);
		}
	return com_rslt;

}
/*******************************************************************************
 *	Description: *//**brief This API Writes
 *	pull up configuration from the register 0X85 bit 4 an 5
 *
 *  \param u8 v_control_pullup_u8 :
 *    Value of the v_control_pullup_u8
 *
 *				     0 - Disabled
 *                   1 - Enabled
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_pullup_configuration(
u8 v_control_pullup_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	} else {
		if (v_control_pullup_u8 < C_BMI160_FOUR_U8X) {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC
			(p_bmi160->dev_addr,
			BMI160_COM_C_TRIM_FIVE__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				v_data_u8 =
				BMI160_SET_BITSLICE(v_data_u8,
				BMI160_COM_C_TRIM_FIVE,
				v_control_pullup_u8);
				com_rslt +=
				p_bmi160->BMI160_BUS_WRITE_FUNC
				(p_bmi160->dev_addr,
				BMI160_COM_C_TRIM_FIVE__REG,
				&v_data_u8, 1);
			}
		} else {
		com_rslt = E_BMI160_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/***************************************************************************
 *	Description: This function used for read the compensated value of mag
 *	Before start reading the mag compensated data's
 *	make sure the following two points are addressed
 *	1.	Make sure the mag interface is enabled or not,
 *		by using the bmi160_get_if_mode() function.
 *		If mag interface is not enabled set the value of 0x02
 *		to the function bmi160_get_if_mode(0x02)
 *	2.	And also confirm the secondary-interface power mode
 *		is not in the SUSPEND mode.
 *		by using the function bmi160_get_mag_pmu_status().
 *		If the secondary-interface power mode is in SUSPEND mode
 *		set the value of 0x19(NORMAL mode)by using the
 *		bmi160_set_command_register(0x19) function.

 *
 *  \return results of communication routine
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_mag_compensate_xyz(
struct bmi160_mag_xyz_s32_t *mag_comp_xyz)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	struct bmi160_mag_xyzr_t mag_xyzr;
	com_rslt = bmi160_read_mag_xyzr(&mag_xyzr);
	/* Compensation for X axis */
	mag_comp_xyz->x = bmi160_mag_compensate_X(mag_xyzr.x, mag_xyzr.r);

	/* Compensation for Y axis */
	mag_comp_xyz->y = bmi160_mag_compensate_Y(mag_xyzr.y, mag_xyzr.r);

	/* Compensation for Z axis */
	mag_comp_xyz->z = bmi160_mag_compensate_Z(mag_xyzr.z, mag_xyzr.r);

	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**\brief This API used to get the compensated X data
 *	the out put of X as s32
 *	Before start reading the mag compensated X data
 *	make sure the following two points are addressed
 *	1.	Make sure the mag interface is enabled or not,
 *		by using the bmi160_get_if_mode() function.
 *		If mag interface is not enabled set the value of 0x02
 *		to the function bmi160_get_if_mode(0x02)
 *	2.	And also confirm the secondary-interface power mode
 *		is not in the SUSPEND mode.
 *		by using the function bmi160_get_mag_pmu_status().
 *		If the secondary-interface power mode is in SUSPEND mode
 *		set the value of 0x19(NORMAL mode)by using the
 *		bmi160_set_command_register(0x19) function.
 *
 *
 *
 *  \param s16 v_mag_data_x_s16 : The value of X data
 *			u16 v_data_r_u16 : The value of R data
 *
 *	\return results of compensated X data value output as s32
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
s32 bmi160_mag_compensate_X(s16 v_mag_data_x_s16, u16 v_data_r_u16)
{
	s32 inter_retval = C_BMI160_ZERO_U8X;
	/* no overflow */
	if (v_mag_data_x_s16 != BMI160_MAG_FLIP_OVERFLOW_ADCVAL
	   ) {
		inter_retval = ((s32)(((u16)
		((((s32)mag_trim.dig_xyz1) << 14)/
		 (v_data_r_u16 != 0 ? v_data_r_u16 : mag_trim.dig_xyz1))) -
		((u16)0x4000)));
		inter_retval = ((s32)((((s32)v_mag_data_x_s16) *
				((((((((s32)mag_trim.dig_xy2) *
				((((s32)inter_retval) *
				((s32)inter_retval)) >> 7)) +
			     (((s32)inter_retval) *
			      ((s32)(((s16)mag_trim.dig_xy1)
			      << 7)))) >> 9) +
			   ((s32)0x100000)) *
			  ((s32)(((s16)mag_trim.dig_x2) +
			  ((s16)0xA0)))) >> 12)) >> 13)) +
			(((s16)mag_trim.dig_x1) << 3);
		/* check the overflow output */
		if (inter_retval == (s32)BMI160_MAG_OVERFLOW_OUTPUT)
			inter_retval = BMI160_MAG_OVERFLOW_OUTPUT_S32;
	} else {
		/* overflow */
		inter_retval = BMI160_MAG_OVERFLOW_OUTPUT;
	}
	return inter_retval;
}
/*****************************************************************************
 *	Description: *//**\brief This API used to get the compensated Y data
 *	the out put of Y as s32
 *	Before start reading the mag compensated Y data
 *	make sure the following two points are addressed
 *	1.	Make sure the mag interface is enabled or not,
 *		by using the bmi160_get_if_mode() function.
 *		If mag interface is not enabled set the value of 0x02
 *		to the function bmi160_get_if_mode(0x02)
 *	2.	And also confirm the secondary-interface power mode
 *		is not in the SUSPEND mode.
 *		by using the function bmi160_get_mag_pmu_status().
 *		If the secondary-interface power mode is in SUSPEND mode
 *		set the value of 0x19(NORMAL mode)by using the
 *		bmi160_set_command_register(0x19) function.
 *
 *
 *  \param s16 v_mag_data_y_s16 : The value of Y data
 *			u16 v_data_r_u16 : The value of R data
 *
 *	\return results of compensated Y data value output as s32
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
s32 bmi160_mag_compensate_Y(s16 v_mag_data_y_s16, u16 v_data_r_u16)
{
	s32 inter_retval = C_BMI160_ZERO_U8X;
	/* no overflow */
	if (v_mag_data_y_s16 != BMI160_MAG_FLIP_OVERFLOW_ADCVAL
	   ) {
		inter_retval = ((s32)(((u16)(((
			(s32)mag_trim.dig_xyz1) << 14)/
			(v_data_r_u16 != 0 ?
			 v_data_r_u16 : mag_trim.dig_xyz1))) -
			((u16)0x4000)));
		inter_retval = ((s32)((((s32)v_mag_data_y_s16) * ((((((((s32)
			mag_trim.dig_xy2) * ((((s32) inter_retval) *
			((s32)inter_retval)) >> 7)) + (((s32)inter_retval) *
			((s32)(((s16)mag_trim.dig_xy1) << 7)))) >> 9) +
			((s32)0x100000)) * ((s32)(((s16)mag_trim.dig_y2)
			+ ((s16)0xA0)))) >> 12)) >> 13)) +
			(((s16)mag_trim.dig_y1) << 3);
		/* check the overflow output */
		if (inter_retval == (s32)BMI160_MAG_OVERFLOW_OUTPUT)
			inter_retval = BMI160_MAG_OVERFLOW_OUTPUT_S32;
	} else {
		/* overflow */
		inter_retval = BMI160_MAG_OVERFLOW_OUTPUT;
	}
	return inter_retval;
}
/*****************************************************************************
 *	Description: *//**\brief This API used to get the compensated Z data
 *	the out put of Z as s32
 *	Before start reading the mag compensated Z data
 *	make sure the following two points are addressed
 *	1.	Make sure the mag interface is enabled or not,
 *		by using the bmi160_get_if_mode() function.
 *		If mag interface is not enabled set the value of 0x02
 *		to the function bmi160_get_if_mode(0x02)
 *	2.	And also confirm the secondary-interface power mode
 *		is not in the SUSPEND mode.
 *		by using the function bmi160_get_mag_pmu_status().
 *		If the secondary-interface power mode is in SUSPEND mode
 *		set the value of 0x19(NORMAL mode)by using the
 *		bmi160_set_command_register(0x19) function.
 *
 *
 *
 *  \param s16 v_mag_data_z_s16 : The value of Z data
 *			u16 v_data_r_u16 : The value of R data
 *
 *	\return results of compensated Z data value output as s32
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
s32 bmi160_mag_compensate_Z(s16 v_mag_data_z_s16, u16 v_data_r_u16)
{
	s32 retval = C_BMI160_ZERO_U8X;
	if (v_mag_data_z_s16 != BMI160_MAG_HALL_OVERFLOW_ADCVAL) {
		retval = (((((s32)(v_mag_data_z_s16 - mag_trim.dig_z4)) << 15) -
		((((s32)mag_trim.dig_z3) *
		((s32)(((s16)v_data_r_u16) -
		((s16)mag_trim.dig_xyz1))))>>2))/
		(mag_trim.dig_z2 +
		((s16)(((((s32)mag_trim.dig_z1) *
		((((s16)v_data_r_u16) << 1)))+(1<<15))>>16))));
	} else {
		retval = BMI160_MAG_OVERFLOW_OUTPUT;
	}
		return retval;
}
/***************************************************************************
 *	Description: This function used for initialize the magnetometer
 *
 *
 *  \return results of communication routine
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_magnetometer_intrerface_init(void)
{
	/* This variable used for provide the communication
	results*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_pull_value_u8 = C_BMI160_ZERO_U8X;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	com_rslt = bmi160_set_command_register(0x19);
	p_bmi160->delay_msec(1);
	bmi160_get_accel_power_mode_stat(&v_data_u8);
	/* register 0x7E write the 0x37, 0x9A and 0x30*/
	com_rslt += bmi160_set_command_register(0x37);
	p_bmi160->delay_msec(1);
	com_rslt += bmi160_set_command_register(0x9A);
	p_bmi160->delay_msec(1);
	com_rslt += bmi160_set_command_register(0xC0);
	p_bmi160->delay_msec(1);
	/*switch the page1*/
	com_rslt += bmi160_set_target_page(0x01);
	p_bmi160->delay_msec(1);
	bmi160_get_target_page(&v_data_u8);
	p_bmi160->delay_msec(1);
	com_rslt += bmi160_set_paging_enable(0x01);
	p_bmi160->delay_msec(1);
	bmi160_get_paging_enable(&v_data_u8);
	p_bmi160->delay_msec(1);
	/* enable the pullup configuration from
	the register 0x85 bit 4 and 5 */
	bmi160_get_pullup_configuration(&v_pull_value_u8);
	p_bmi160->delay_msec(1);
	v_pull_value_u8 = v_pull_value_u8 | 0x03;
	com_rslt += bmi160_set_pullup_configuration(v_pull_value_u8);
	p_bmi160->delay_msec(1);

	/*switch the page0*/
	com_rslt += bmi160_set_target_page(0x00);
	p_bmi160->delay_msec(1);
	bmi160_get_target_page(&v_data_u8);
	p_bmi160->delay_msec(1);

	/* enable the mag interface to manual mode*/
	com_rslt += bmi160_set_mag_manual_enable(0x01);
	p_bmi160->delay_msec(1);
	bmi160_get_mag_manual_enable(&v_data_u8);
	p_bmi160->delay_msec(1);
	/*Enable the MAG interface */
	com_rslt += bmi160_set_if_mode(0x02);
	p_bmi160->delay_msec(1);
	bmi160_get_if_mode(&v_data_u8);
	p_bmi160->delay_msec(1);

	/* Mag normal mode*/
	com_rslt += bmi160_set_mag_write_data(0x01);
	p_bmi160->delay_msec(1);

	com_rslt += bmi160_set_mag_write_addr(0x4B);
	p_bmi160->delay_msec(5);

	com_rslt += bmi160_set_mag_write_data(0x06);
	p_bmi160->delay_msec(1);

	com_rslt += bmi160_set_mag_write_addr(0x4C);
	p_bmi160->delay_msec(5);

	/* write the XY and Z repetitions*/
	com_rslt += bmi160_set_mag_presetmode(BMI160_MAG_PRESETMODE_REGULAR);

	/* read the mag trim values*/
	com_rslt += bmi160_read_mag_trim();

	/* Set the power mode of mag as force mode*/
	/* The v_data_u8 have to write for the register
	It write the value in the register 0x4F */
	com_rslt += bmi160_set_mag_write_data(0x02);
	p_bmi160->delay_msec(1);

	com_rslt += bmi160_set_mag_write_addr(0x4C);
	p_bmi160->delay_msec(1);
	/* write the mag v_data_bw_u8 as 25Hz*/
	com_rslt += bmi160_set_mag_output_data_rate(
	BMI160_MAG_OUTPUT_DATA_RATE_25HZ);
	p_bmi160->delay_msec(1);

	/* When mag interface is auto mode - The mag read address
	starts the register 0x42*/
	com_rslt += bmi160_set_mag_read_addr(0x42);
	p_bmi160->delay_msec(1);
	/* enable mag interface to auto mode*/
	com_rslt += bmi160_set_mag_manual_enable(0x00);
	p_bmi160->delay_msec(1);
	bmi160_get_mag_manual_enable(&v_data_u8);
	p_bmi160->delay_msec(1);

	return com_rslt;
}
/***************************************************************************
 *	Description: This function used for set the magnetometer
 *	power mode.
 *	Before set the mag power mode
 *	make sure the following two point is addressed
 *		Make sure the mag interface is enabled or not,
 *		by using the bmi160_get_if_mode() function.
 *		If mag interface is not enabled set the value of 0x02
 *		to the function bmi160_get_if_mode(0x02)
 *
 *	\param mag_sec_if_pow_mode : The value of mag power mode
 *
 *	BMI160_MAG_FORCE_MODE		0
 *	BMI160_MAG_SUSPEND_MODE		1
 *
 *
 *  \return results of communication routine
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_mag_and_secondary_if_power_mode(
u8 v_mag_sec_if_pow_mode_u8)
{
	/* This variable used for provide the communication
	results*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	switch (v_mag_sec_if_pow_mode_u8) {
	case BMI160_MAG_FORCE_MODE:
		/* set mag interface manual mode*/
		com_rslt = bmi160_set_mag_manual_enable(0x01);
		p_bmi160->delay_msec(1);
		/* set the secondary mag power mode as NORMAL*/
		com_rslt += bmi160_set_command_register(0x19);
		p_bmi160->delay_msec(5);
		/* set the mag power mode as FORCE mode*/
		com_rslt += bmi160_mag_set_power_mode(FORCE_MODE);
		p_bmi160->delay_msec(1);
		/* set mag interface auto mode*/
		com_rslt += bmi160_set_mag_manual_enable(0x00);
		p_bmi160->delay_msec(1);
	break;
	case BMI160_MAG_SUSPEND_MODE:
		/* set mag interface manual mode*/
		com_rslt = bmi160_set_mag_manual_enable(0x01);
		p_bmi160->delay_msec(1);
		/* set the mag power mode as SUSPEND mode*/
		com_rslt += bmi160_mag_set_power_mode(SUSPEND_MODE);
		p_bmi160->delay_msec(1);
		/* set the secondary mag power mode as SUSPEND*/
		com_rslt += bmi160_set_command_register(0x18);
		p_bmi160->delay_msec(5);
	break;
	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
	break;
	}
	return com_rslt;
}
/***************************************************************************
 *	Description: This function used for set the magnetometer
 *	power mode.
 *	Before set the mag power mode
 *	make sure the following two points are addressed
 *	1.	Make sure the mag interface is enabled or not,
 *		by using the bmi160_get_if_mode() function.
 *		If mag interface is not enabled set the value of 0x02
 *		to the function bmi160_get_if_mode(0x02)
 *	2.	And also confirm the secondary-interface power mode
 *		is not in the SUSPEND mode.
 *		by using the function bmi160_get_mag_pmu_status().
 *		If the secondary-interface power mode is in SUSPEND mode
 *		set the value of 0x19(NORMAL mode)by using the
 *		bmi160_set_command_register(0x19) function.
 *
 *	\param v_mag_pow_mode_u8 : The value of mag power mode
 *
 *	FORCE_MODE		0
 *	SUSPEND_MODE	1
 *
 *
 *  \return results of communication routine
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_mag_set_power_mode(
u8 v_mag_pow_mode_u8)
{
	/* This variable used for provide the communication
	results*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	switch (v_mag_pow_mode_u8) {
	case FORCE_MODE:
		/* set mag interface manual mode*/
		com_rslt = bmi160_set_mag_manual_enable(0x01);
		p_bmi160->delay_msec(1);
		/* Set the power mode of mag as force mode*/
		com_rslt += bmi160_set_mag_write_data(0x01);
		p_bmi160->delay_msec(1);
		com_rslt += bmi160_set_mag_write_addr(0x4B);
		p_bmi160->delay_msec(15);
		com_rslt += bmi160_set_mag_write_data(0x02);
		p_bmi160->delay_msec(1);
		com_rslt += bmi160_set_mag_write_addr(0x4C);
		p_bmi160->delay_msec(15);
		/* set the preset mode */
		com_rslt += bmi160_set_mag_presetmode(
		BMI160_MAG_PRESETMODE_REGULAR);
		p_bmi160->delay_msec(1);
		com_rslt += bmi160_set_mag_read_addr(0x42);
		p_bmi160->delay_msec(1);
		/* set mag interface auto mode*/
		com_rslt += bmi160_set_mag_manual_enable(0x00);
		p_bmi160->delay_msec(1);
	break;
	case SUSPEND_MODE:
		/* set mag interface manual mode*/
		com_rslt = bmi160_set_mag_manual_enable(0x01);
		p_bmi160->delay_msec(1);
		/* Set the power mode of mag as suspend mode*/
		com_rslt += bmi160_set_mag_write_data(0x00);
		p_bmi160->delay_msec(1);
		com_rslt += bmi160_set_mag_write_addr(0x4B);
		p_bmi160->delay_msec(5);
	break;
	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
	break;
	}
	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**\brief This API used to set the pre-set modes
 *	The pre-set mode setting is depend on v_data_u8 rate, xy and z repetitions
 *
 *	Before set the mag preset mode
 *	make sure the following two points are addressed
 *	1.	Make sure the mag interface is enabled or not,
 *		by using the bmi160_get_if_mode() function.
 *		If mag interface is not enabled set the value of 0x02
 *		to the function bmi160_get_if_mode(0x02)
 *	2.	And also confirm the secondary-interface power mode
 *		is not in the SUSPEND mode.
 *		by using the function bmi160_get_mag_pmu_status().
 *		If the secondary-interface power mode is in SUSPEND mode
 *		set the value of 0x19(NORMAL mode)by using the
 *		bmi160_set_command_register(0x19) function.
 *
 *
 *  \param u8 v_mode_u8: The value of pre-set mode selection value
 *
 *	BMI160_MAG_PRESETMODE_LOWPOWER                  1
 *	BMI160_MAG_PRESETMODE_REGULAR                   2
 *	BMI160_MAG_PRESETMODE_HIGHACCURACY              3
 *	BMI160_MAG_PRESETMODE_ENHANCED                  4
 *
 *	\return results of bus communication function
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_mag_presetmode(u8 v_mode_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	switch (v_mode_u8) {
	case BMI160_MAG_PRESETMODE_LOWPOWER:
		/* write the XY and Z repetitions*/
		/* The v_data_u8 have to write for the register
		It write the value in the register 0x4F*/
		com_rslt = bmi160_set_mag_write_data(
		BMI160_MAG_LOWPOWER_REPXY);
		p_bmi160->delay_msec(1);
		com_rslt += bmi160_set_mag_write_addr(0x51);
		p_bmi160->delay_msec(1);
		/* write the Z repetitions*/
		/* The v_data_u8 have to write for the register
		It write the value in the register 0x4F*/
		com_rslt += bmi160_set_mag_write_data(
		BMI160_MAG_LOWPOWER_REPZ);
		p_bmi160->delay_msec(1);
		com_rslt += bmi160_set_mag_write_addr(0x52);
		p_bmi160->delay_msec(1);
		/* set the mag v_data_u8 rate as 10 to the register 0x4C*/
		com_rslt += bmi160_set_mag_write_data(
		BMI160_MAG_LOWPOWER_DR);
		p_bmi160->delay_msec(1);
		com_rslt += bmi160_set_mag_write_addr(0x4C);
		p_bmi160->delay_msec(1);
	break;
	case BMI160_MAG_PRESETMODE_REGULAR:
		/* write the XY and Z repetitions*/
		/* The v_data_u8 have to write for the register
		It write the value in the register 0x4F*/
		com_rslt = bmi160_set_mag_write_data(
		BMI160_MAG_REGULAR_REPXY);
		p_bmi160->delay_msec(1);
		com_rslt += bmi160_set_mag_write_addr(0x51);
		p_bmi160->delay_msec(1);
		/* write the Z repetitions*/
		/* The v_data_u8 have to write for the register
		It write the value in the register 0x4F*/
		com_rslt += bmi160_set_mag_write_data(
		BMI160_MAG_REGULAR_REPZ);
		p_bmi160->delay_msec(1);
		com_rslt += bmi160_set_mag_write_addr(0x52);
		p_bmi160->delay_msec(1);
		/* set the mag v_data_u8 rate as 10 to the register 0x4C*/
		com_rslt += bmi160_set_mag_write_data(
		BMI160_MAG_REGULAR_DR);
		p_bmi160->delay_msec(1);
		com_rslt += bmi160_set_mag_write_addr(0x4C);
		p_bmi160->delay_msec(1);
	break;
	case BMI160_MAG_PRESETMODE_HIGHACCURACY:
		/* write the XY and Z repetitions*/
		/* The v_data_u8 have to write for the register
		It write the value in the register 0x4F*/
		com_rslt = bmi160_set_mag_write_data(
		BMI160_MAG_HIGHACCURACY_REPXY);
		p_bmi160->delay_msec(1);
		com_rslt += bmi160_set_mag_write_addr(0x51);
		p_bmi160->delay_msec(1);
		/* write the Z repetitions*/
		/* The v_data_u8 have to write for the register
		It write the value in the register 0x4F*/
		com_rslt += bmi160_set_mag_write_data(
		BMI160_MAG_HIGHACCURACY_REPZ);
		p_bmi160->delay_msec(1);
		com_rslt += bmi160_set_mag_write_addr(0x52);
		p_bmi160->delay_msec(1);
		/* set the mag v_data_u8 rate as 20 to the register 0x4C*/
		com_rslt += bmi160_set_mag_write_data(
		BMI160_MAG_HIGHACCURACY_DR);
		p_bmi160->delay_msec(1);
		com_rslt += bmi160_set_mag_write_addr(0x4C);
		p_bmi160->delay_msec(1);
	break;
	case BMI160_MAG_PRESETMODE_ENHANCED:
		/* write the XY and Z repetitions*/
		/* The v_data_u8 have to write for the register
		It write the value in the register 0x4F*/
		com_rslt = bmi160_set_mag_write_data(
		BMI160_MAG_ENHANCED_REPXY);
		p_bmi160->delay_msec(1);
		com_rslt += bmi160_set_mag_write_addr(0x51);
		p_bmi160->delay_msec(1);
		/* write the Z repetitions*/
		/* The v_data_u8 have to write for the register
		It write the value in the register 0x4F*/
		com_rslt += bmi160_set_mag_write_data(
		BMI160_MAG_ENHANCED_REPZ);
		p_bmi160->delay_msec(1);
		com_rslt += bmi160_set_mag_write_addr(0x52);
		p_bmi160->delay_msec(1);
		/* set the mag v_data_u8 rate as 10 to the register 0x4C*/
		com_rslt += bmi160_set_mag_write_data(
		BMI160_MAG_ENHANCED_DR);
		p_bmi160->delay_msec(1);
		com_rslt += bmi160_set_mag_write_addr(0x4C);
		p_bmi160->delay_msec(1);
	break;
	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
	break;
	}
	return com_rslt;
}
/***************************************************************************
 *	Description: This function used for read the trim values of magnetometer
 *
 *	Before reading the mag trimming values
 *	make sure the following two points are addressed
 *	1.	Make sure the mag interface is enabled or not,
 *		by using the bmi160_get_if_mode() function.
 *		If mag interface is not enabled set the value of 0x02
 *		to the function bmi160_get_if_mode(0x02)
 *	2.	And also confirm the secondary-interface power mode
 *		is not in the SUSPEND mode.
 *		by using the function bmi160_get_mag_pmu_status().
 *		If the secondary-interface power mode is in SUSPEND mode
 *		set the value of 0x19(NORMAL mode)by using the
 *		bmi160_set_command_register(0x19) function.
 *
 *  \return results of communication routine
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_read_mag_trim(void)
{
	/* This variable used for provide the communication
	results*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	/* read dig_x1 value */
	com_rslt = bmi160_set_mag_read_addr(BMI160_MAG_DIG_X1);
	p_bmi160->delay_msec(1);
	/* 0x04 is v_mag_x_s16 lsb register */
	com_rslt += bmi160_read_reg(0x04, &v_data_u8[0], 1);
	p_bmi160->delay_msec(1);
	mag_trim.dig_x1 = v_data_u8[0];
	/* read dig_y1 value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_MAG_DIG_Y1);
	p_bmi160->delay_msec(1);
	/* 0x04 is v_mag_x_s16 lsb register */
	com_rslt += bmi160_read_reg(0x04, &v_data_u8[1], 1);
	p_bmi160->delay_msec(1);
	mag_trim.dig_y1 = v_data_u8[1];

	/* read dig_x2 value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_MAG_DIG_X2);
	p_bmi160->delay_msec(1);
	/* 0x04 is v_mag_x_s16 lsb register */
	com_rslt += bmi160_read_reg(0x04, &v_data_u8[2], 1);
	p_bmi160->delay_msec(1);
	mag_trim.dig_x2 = v_data_u8[2];
	/* read dig_y2 value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_MAG_DIG_Y2);
	p_bmi160->delay_msec(1);
	/* 0x04 is v_mag_x_s16 lsb register */
	com_rslt += bmi160_read_reg(0x04, &v_data_u8[3], 1);
	p_bmi160->delay_msec(1);
	mag_trim.dig_y2 = v_data_u8[3];

	/* read dig_xy1 value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_MAG_DIG_XY1);
	p_bmi160->delay_msec(1);
	/* 0x04 is v_mag_x_s16 lsb register */
	com_rslt += bmi160_read_reg(0x04, &v_data_u8[4], 1);
	p_bmi160->delay_msec(1);
	mag_trim.dig_xy1 = v_data_u8[4];
	/* read dig_xy2 value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_MAG_DIG_XY2);
	p_bmi160->delay_msec(1);
	/* 0x04 is v_mag_x_s16 ls register */
	com_rslt += bmi160_read_reg(0x04, &v_data_u8[5], 1);
	p_bmi160->delay_msec(1);
	mag_trim.dig_xy2 = v_data_u8[5];

	/* read dig_z1 lsb value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_MAG_DIG_Z1_LSB);
	p_bmi160->delay_msec(1);
	/* 0x04 is v_mag_x_s16 lsb register */
	com_rslt += bmi160_read_reg(0x04, &v_data_u8[6], 1);
	p_bmi160->delay_msec(1);
	/* read dig_z1 msb value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_MAG_DIG_Z1_MSB);
	p_bmi160->delay_msec(1);
	/* 0x04 is v_mag_x_s16 msb register */
	com_rslt += bmi160_read_reg(0x04, &v_data_u8[7], 1);
	p_bmi160->delay_msec(1);
	mag_trim.dig_z1 = (u16)((((u32)((u8)v_data_u8[7]))
			<< BMI160_SHIFT_8_POSITION) | (v_data_u8[6]));

	/* read dig_z2 lsb value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_MAG_DIG_Z2_LSB);
	p_bmi160->delay_msec(1);
	/* 0x04 is v_mag_x_s16 lsb register */
	com_rslt += bmi160_read_reg(0x04, &v_data_u8[8], 1);
	p_bmi160->delay_msec(1);
	/* read dig_z2 msb value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_MAG_DIG_Z2_MSB);
	p_bmi160->delay_msec(1);
	/* 0x04 is v_mag_x_s16 msb register */
	com_rslt += bmi160_read_reg(0x04, &v_data_u8[9], 1);
	p_bmi160->delay_msec(1);
	mag_trim.dig_z2 = (s16)((((s32)((s8)v_data_u8[9]))
			<< BMI160_SHIFT_8_POSITION) | (v_data_u8[8]));

	/* read dig_z3 lsb value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_MAG_DIG_Z3_LSB);
	p_bmi160->delay_msec(1);
	/* 0x04 is v_mag_x_s16 lsb register */
	com_rslt += bmi160_read_reg(0x04, &v_data_u8[10], 1);
	p_bmi160->delay_msec(1);
	/* read dig_z3 msb value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_MAG_DIG_Z3_MSB);
	p_bmi160->delay_msec(1);
	/* 0x04 is v_mag_x_s16 msb register */
	com_rslt += bmi160_read_reg(0x04, &v_data_u8[11], 1);
	p_bmi160->delay_msec(1);
	mag_trim.dig_z3 = (s16)((((s32)((s8)v_data_u8[11]))
			<< BMI160_SHIFT_8_POSITION) | (v_data_u8[10]));

	/* read dig_z4 lsb value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_MAG_DIG_Z4_LSB);
	p_bmi160->delay_msec(1);
	/* 0x04 is v_mag_x_s16 lsb register */
	com_rslt += bmi160_read_reg(0x04, &v_data_u8[12], 1);
	p_bmi160->delay_msec(1);
	/* read dig_z4 msb value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_MAG_DIG_Z4_MSB);
	p_bmi160->delay_msec(1);
	/* 0x04 is v_mag_x_s16 msb register */
	com_rslt += bmi160_read_reg(0x04, &v_data_u8[13], 1);
	p_bmi160->delay_msec(1);
	mag_trim.dig_z4 = (s16)((((s32)((s8)v_data_u8[13]))
			<< BMI160_SHIFT_8_POSITION) | (v_data_u8[12]));

	/* read dig_xyz1 lsb value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_MAG_DIG_XYZ1_LSB);
	p_bmi160->delay_msec(1);
	/* 0x04 is v_mag_x_s16 lsb register */
	com_rslt += bmi160_read_reg(0x04, &v_data_u8[14], 1);
	p_bmi160->delay_msec(1);
	/* read dig_xyz1 msb value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_MAG_DIG_XYZ1_MSB);
	p_bmi160->delay_msec(1);
	/* 0x04 is v_mag_x_s16 msb register */
	com_rslt += bmi160_read_reg(0x04, &v_data_u8[15], 1);
	p_bmi160->delay_msec(1);
	mag_trim.dig_xyz1 = (u16)((((u32)((u8)v_data_u8[15]))
			<< BMI160_SHIFT_8_POSITION) | (v_data_u8[14]));

	return com_rslt;
}
/***************************************************************************
 *	Description: This function used for initialize the AKM09911 sensor
 *
 *
 *  \return results of communication routine
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_bst_akm_mag_intrerface_init(void)
{
	/* This variable used for provide the communication
	results*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_pull_value_u8 = C_BMI160_ZERO_U8X;
	u8 v_data_u8 = C_BMI160_ZERO_U8X;
	com_rslt = bmi160_set_command_register(0x19);
	p_bmi160->delay_msec(60);
	bmi160_get_accel_power_mode_stat(&v_data_u8);
	/* register 0x7E write the 0x37, 0x9A and 0x30*/
	com_rslt += bmi160_set_command_register(0x37);
	p_bmi160->delay_msec(5);
	com_rslt += bmi160_set_command_register(0x9A);
	p_bmi160->delay_msec(5);
	com_rslt += bmi160_set_command_register(0xC0);
	p_bmi160->delay_msec(5);
	/*switch the page1*/
	com_rslt += bmi160_set_target_page(0x01);
	p_bmi160->delay_msec(1);
	bmi160_get_target_page(&v_data_u8);
	p_bmi160->delay_msec(1);
	com_rslt += bmi160_set_paging_enable(0x01);
	p_bmi160->delay_msec(1);
	bmi160_get_paging_enable(&v_data_u8);
	p_bmi160->delay_msec(1);
	/* enable the pullup configuration from
	the register 0x85 bit 4 and 5 */
	bmi160_get_pullup_configuration(&v_pull_value_u8);
	p_bmi160->delay_msec(1);
	v_pull_value_u8 = v_pull_value_u8 | 0x03;
	com_rslt += bmi160_set_pullup_configuration(v_pull_value_u8);
	p_bmi160->delay_msec(1);

	/*switch the page0*/
	com_rslt += bmi160_set_target_page(0x00);
	p_bmi160->delay_msec(1);
	bmi160_get_target_page(&v_data_u8);
	p_bmi160->delay_msec(1);
	/* Write the AKM i2c address*/
	com_rslt += bmi160_set_i2c_device_addr(0x0C);
	p_bmi160->delay_msec(1);
	/* enable the mag interface to manual mode*/
	com_rslt += bmi160_set_mag_manual_enable(0x01);
	p_bmi160->delay_msec(1);
	bmi160_get_mag_manual_enable(&v_data_u8);
	p_bmi160->delay_msec(1);
	/*Enable the MAG interface */
	com_rslt += bmi160_set_if_mode(0x02);
	p_bmi160->delay_msec(1);
	bmi160_get_if_mode(&v_data_u8);
	p_bmi160->delay_msec(1);

	/* Set the AKM Fuse ROM mode */
	/* Set value for fuse ROM mode*/
	com_rslt += bmi160_set_mag_write_data(0x1F);
	p_bmi160->delay_msec(1);
	/* AKM mode address is 0x31*/
	com_rslt += bmi160_set_mag_write_addr(0x31);
	p_bmi160->delay_msec(5);
	/* Read the Fuse ROM v_data_u8 from registers
	0x60,0x61 and 0x62*/
	/* ASAX v_data_u8 */
	com_rslt += bmi160_read_bst_akm_sensitivity_data();
	p_bmi160->delay_msec(5);
	/* Set value power down mode mode*/
	com_rslt += bmi160_set_mag_write_data(0x00);
	p_bmi160->delay_msec(1);
	/* AKM mode address is 0x31*/
	com_rslt += bmi160_set_mag_write_addr(0x31);
	p_bmi160->delay_msec(5);
	/* Set AKM Force mode*/
	com_rslt += bmi160_set_mag_write_data(0x01);
	p_bmi160->delay_msec(1);
	/* AKM mode address is 0x31*/
	com_rslt += bmi160_set_mag_write_addr(0x31);
	p_bmi160->delay_msec(5);
	/* Set the AKM read xyz v_data_u8 address*/
	com_rslt += bmi160_set_mag_read_addr(0x11);
	/* write the mag v_data_bw_u8 as 25Hz*/
	com_rslt += bmi160_set_mag_output_data_rate(
	BMI160_MAG_OUTPUT_DATA_RATE_25HZ);
	p_bmi160->delay_msec(1);
	/* Enable mag interface to auto mode*/
	com_rslt += bmi160_set_mag_manual_enable(0x00);
	p_bmi160->delay_msec(1);
	bmi160_get_mag_manual_enable(&v_data_u8);
	p_bmi160->delay_msec(1);

	return com_rslt;
}
/***************************************************************************
 *	Description: This function used for read the sensitivity data of
 *	AKM09911
 *
 *	Before reading the mag sensitivity values
 *	make sure the following two points are addressed
 *	1.	Make sure the mag interface is enabled or not,
 *		by using the bmi160_get_if_mode() function.
 *		If mag interface is not enabled set the value of 0x02
 *		to the function bmi160_get_if_mode(0x02)
 *	2.	And also confirm the secondary-interface power mode
 *		is not in the SUSPEND mode.
 *		by using the function bmi160_get_mag_pmu_status().
 *		If the secondary-interface power mode is in SUSPEND mode
 *		set the value of 0x19(NORMAL mode)by using the
 *		bmi160_set_command_register(0x19) function.
 *
 *  \return results of communication routine
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_read_bst_akm_sensitivity_data(void)
{
	/* This variable used for provide the communication
	results*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8[3] = {0, 0, 0};
	/* read asax value */
	com_rslt = bmi160_set_mag_read_addr(BMI160_BST_AKM_ASAX);
	p_bmi160->delay_msec(1);
	/* 0x04 is v_mag_x_s16 lsb register */
	com_rslt += bmi160_read_reg(0x04, &v_data_u8[0], 1);
	p_bmi160->delay_msec(1);
	akm_asa_data.asax = v_data_u8[0];
	/* read asay value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_BST_AKM_ASAY);
	p_bmi160->delay_msec(1);
	/* 0x04 is v_mag_x_s16 lsb register */
	com_rslt += bmi160_read_reg(0x04, &v_data_u8[1], 1);
	p_bmi160->delay_msec(1);
	akm_asa_data.asay = v_data_u8[1];
	/* read asaz value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_BST_AKM_ASAZ);
	p_bmi160->delay_msec(1);
	/* 0x04 is v_mag_x_s16 lsb register */
	com_rslt += bmi160_read_reg(0x04, &v_data_u8[2], 1);
	p_bmi160->delay_msec(1);
	akm_asa_data.asaz = v_data_u8[2];

	return com_rslt;
}
/*****************************************************************************
 *	Description: *//**\brief This API used to get the compensated Z v_data_u8
 *	of AKM09911 the out put of X as s16
 *	Before start reading the mag compensated Z v_data_u8
 *	make sure the following two points are addressed
 *	1.	Make sure the mag interface is enabled or not,
 *		by using the bmi160_get_if_mode() function.
 *		If mag interface is not enabled set the value of 0x02
 *		to the function bmi160_get_if_mode(0x02)
 *	2.	And also confirm the secondary-interface power mode
 *		is not in the SUSPEND mode.
 *		by using the function bmi160_get_mag_pmu_status().
 *		If the secondary-interface power mode is in SUSPEND mode
 *		set the value of 0x19(NORMAL mode)by using the
 *		bmi160_set_command_register(0x19) function.
 *
 *
 *  \param s16 v_bst_akm_x_s16 : The value of X v_data_u8
 *
 *	\return results of compensated X v_data_u8 value output as s16
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
s16 bmi160_bst_akm_compensate_X(s16 v_bst_akm_x_s16)
{
	/*Return value of AKM x compensated v_data_u8*/
	s16 retval = C_BMI160_ZERO_U8X;
	/* Convert raw v_data_u8 into compensated v_data_u8*/
	retval = (v_bst_akm_x_s16 * ((akm_asa_data.asax/128) + 1));
	return retval;
}
/*****************************************************************************
 *	Description: *//**\brief This API used to get the compensated Y v_data_u8
 *	of AKM09911 the out put of Y as s16
 *	Before start reading the AKM compensated Y v_data_u8
 *	make sure the following two points are addressed
 *	1.	Make sure the mag interface is enabled or not,
 *		by using the bmi160_get_if_mode() function.
 *		If mag interface is not enabled set the value of 0x02
 *		to the function bmi160_get_if_mode(0x02)
 *	2.	And also confirm the secondary-interface power mode
 *		is not in the SUSPEND mode.
 *		by using the function bmi160_get_mag_pmu_status().
 *		If the secondary-interface power mode is in SUSPEND mode
 *		set the value of 0x19(NORMAL mode)by using the
 *		bmi160_set_command_register(0x19) function.
 *
 *
 *  \param s16 v_bst_akm_y_s16 : The value of Y v_data_u8
 *
 *	\return results of compensated Y v_data_u8 value output as s16
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
s16 bmi160_bst_akm_compensate_Y(s16 v_bst_akm_y_s16)
{
	/*Return value of AKM y compensated v_data_u8*/
	s16 retval = C_BMI160_ZERO_U8X;
	/* Convert raw v_data_u8 into compensated v_data_u8*/
	retval = (v_bst_akm_y_s16 * ((akm_asa_data.asay/128) + 1));
	return retval;
}
/*****************************************************************************
 *	Description: *//**\brief This API used to get the compensated Z v_data_u8
 *	of AKM09911 the out put of Z as s16
 *	Before start reading the AKM compensated Z v_data_u8
 *	make sure the following two points are addressed
 *	1.	Make sure the mag interface is enabled or not,
 *		by using the bmi160_get_if_mode() function.
 *		If mag interface is not enabled set the value of 0x02
 *		to the function bmi160_get_if_mode(0x02)
 *	2.	And also confirm the secondary-interface power mode
 *		is not in the SUSPEND mode.
 *		by using the function bmi160_get_mag_pmu_status().
 *		If the secondary-interface power mode is in SUSPEND mode
 *		set the value of 0x19(NORMAL mode)by using the
 *		bmi160_set_command_register(0x19) function.
 *
 *
 *  \param s16 v_bst_akm_z_s16 : The value of Z v_data_u8
 *
 *	\return results of compensated Z v_data_u8 value output as s16
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
s16 bmi160_bst_akm_compensate_Z(s16 v_bst_akm_z_s16)
{
	/*Return value of AKM z compensated v_data_u8*/
	s16 retval = C_BMI160_ZERO_U8X;
	/* Convert raw v_data_u8 into compensated v_data_u8*/
	retval = (v_bst_akm_z_s16 * ((akm_asa_data.asaz/128) + 1));
	return retval;
}
/***************************************************************************
 *	Description: This function used for read the compensated value of
 *	AKM09911
 *	Before start reading the mag compensated v_data_u8's
 *	make sure the following two points are addressed
 *	1.	Make sure the mag interface is enabled or not,
 *		by using the bmi160_get_if_mode() function.
 *		If mag interface is not enabled set the value of 0x02
 *		to the function bmi160_get_if_mode(0x02)
 *	2.	And also confirm the secondary-interface power mode
 *		is not in the SUSPEND mode.
 *		by using the function bmi160_get_mag_pmu_status().
 *		If the secondary-interface power mode is in SUSPEND mode
 *		set the value of 0x19(NORMAL mode)by using the
 *		bmi160_set_command_register(0x19) function.

 *
 *  \return results of communication routine
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_bst_akm_compensate_xyz(
struct bmi160_bst_akm_xyz_t *bst_akm_xyz)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	struct bmi160_mag_t mag_xyz;
	com_rslt = bmi160_read_mag_xyz(&mag_xyz, 1);
	/* Compensation for X axis */
	bst_akm_xyz->x = bmi160_bst_akm_compensate_X(mag_xyz.x);

	/* Compensation for Y axis */
	bst_akm_xyz->y = bmi160_bst_akm_compensate_Y(mag_xyz.y);

	/* Compensation for Z axis */
	bst_akm_xyz->z = bmi160_bst_akm_compensate_Z(mag_xyz.z);

	return com_rslt;
}
/***************************************************************************
 *	Description: This function used for set the AKM09911
 *	power mode.
 *	Before set the AKM power mode
 *	make sure the following two points are addressed
 *	1.	Make sure the mag interface is enabled or not,
 *		by using the bmi160_get_if_mode() function.
 *		If mag interface is not enabled set the value of 0x02
 *		to the function bmi160_get_if_mode(0x02)
 *	2.	And also confirm the secondary-interface power mode
 *		is not in the SUSPEND mode.
 *		by using the function bmi160_get_mag_pmu_status().
 *		If the secondary-interface power mode is in SUSPEND mode
 *		set the value of 0x19(NORMAL mode)by using the
 *		bmi160_set_command_register(0x19) function.
 *
 *	\param v_akm_pow_mode_u8 : The value of mag power mode
 *
 *	AKM_POWER_DOWN_MODE		0
 *	AKM_SINGLE_MEAS_MODE	1
 *	FUSE_ROM_MODE			2
 *
 *
 *  \return results of communication routine
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_bst_akm_set_powermode(
u8 v_akm_pow_mode_u8)
{
	/* This variable used for provide the communication
	results*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	switch (v_akm_pow_mode_u8) {
	case AKM_POWER_DOWN_MODE:
		/* set mag interface manual mode*/
		com_rslt = bmi160_set_mag_manual_enable(0x01);
		p_bmi160->delay_msec(1);
		/* Set the power mode of AKM as power down mode*/
		com_rslt += bmi160_set_mag_write_data(0x00);
		p_bmi160->delay_msec(1);
		com_rslt += bmi160_set_mag_write_addr(0x31);
		p_bmi160->delay_msec(5);
		/* set mag interface auto mode*/
		com_rslt += bmi160_set_mag_manual_enable(0x00);
		p_bmi160->delay_msec(1);
	break;
	case AKM_SINGLE_MEAS_MODE:
		/* set mag interface manual mode*/
		com_rslt = bmi160_set_mag_manual_enable(0x01);
		p_bmi160->delay_msec(1);
		/* Set the power mode of AKM as
		single measurement mode*/
		com_rslt += bmi160_set_mag_write_data(0x01);
		p_bmi160->delay_msec(1);
		com_rslt += bmi160_set_mag_write_addr(0x31);
		p_bmi160->delay_msec(5);
		/* set mag interface auto mode*/
		com_rslt += bmi160_set_mag_manual_enable(0x00);
		p_bmi160->delay_msec(1);
	break;
	case FUSE_ROM_MODE:
		/* set mag interface manual mode*/
		com_rslt = bmi160_set_mag_manual_enable(0x01);
		p_bmi160->delay_msec(1);
		/* Set the power mode of AKM as
		Fuse ROM mode*/
		com_rslt += bmi160_set_mag_write_data(0x1F);
		p_bmi160->delay_msec(1);
		com_rslt += bmi160_set_mag_write_addr(0x31);
		p_bmi160->delay_msec(5);
		/* Sensitivity v_data_u8 */
		com_rslt += bmi160_read_bst_akm_sensitivity_data();
		p_bmi160->delay_msec(5);
		/* power down mode*/
		com_rslt += bmi160_set_mag_write_data(0x00);
		p_bmi160->delay_msec(1);
		com_rslt += bmi160_set_mag_write_addr(0x31);
		p_bmi160->delay_msec(5);
		/* set mag interface auto mode*/
		com_rslt += bmi160_set_mag_manual_enable(0x00);
		p_bmi160->delay_msec(1);
	break;
	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
	break;
	}
	return com_rslt;
}
/***************************************************************************
 *	Description: This function used for set the magnetometer
 *	power mode.
 *	Before set the mag power mode
 *	make sure the following two point is addressed
 *		Make sure the mag interface is enabled or not,
 *		by using the bmi160_get_if_mode() function.
 *		If mag interface is not enabled set the value of 0x02
 *		to the function bmi160_get_if_mode(0x02)
 *
 *	\param mag_sec_if_pow_mode : The value of mag power mode
 *
 *	BMI160_MAG_FORCE_MODE		0
 *	BMI160_MAG_SUSPEND_MODE		1
 *
 *
 *  \return results of communication routine
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_bst_akm_and_secondary_if_powermode(
u8 v_mag_sec_if_pow_mode_u8)
{
	/* This variable used for provide the communication
	results*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	switch (v_mag_sec_if_pow_mode_u8) {
	case BMI160_MAG_FORCE_MODE:
		/* set mag interface manual mode*/
		com_rslt = bmi160_set_mag_manual_enable(0x01);
		p_bmi160->delay_msec(1);
		/* set the secondary mag power mode as NORMAL*/
		com_rslt += bmi160_set_command_register(0x19);
		p_bmi160->delay_msec(5);
		/* set the akm power mode as single measurement mode*/
		com_rslt += bmi160_bst_akm_set_powermode(AKM_SINGLE_MEAS_MODE);
		p_bmi160->delay_msec(5);
		/* set mag interface auto mode*/
		com_rslt += bmi160_set_mag_manual_enable(0x00);
		p_bmi160->delay_msec(1);
	break;
	case BMI160_MAG_SUSPEND_MODE:
		/* set mag interface manual mode*/
		com_rslt = bmi160_set_mag_manual_enable(0x01);
		p_bmi160->delay_msec(1);
		/* set the akm power mode as power down mode*/
		com_rslt += bmi160_bst_akm_set_powermode(AKM_POWER_DOWN_MODE);
		p_bmi160->delay_msec(5);
		/* set the secondary mag power mode as SUSPEND*/
		com_rslt += bmi160_set_command_register(0x18);
		p_bmi160->delay_msec(5);
		/* set mag interface auto mode*/
		com_rslt += bmi160_set_mag_manual_enable(0x00);
		p_bmi160->delay_msec(1);
	break;
	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
	break;
	}
	return com_rslt;
}
/***************************************************************************
 *	Description: This function used for read the FIFO v_data_u8 as header mode
 *
 *
 *  \return results of communication routine
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_read_fifo_header_data(u32 v_fifo_length_u32)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_accel_index_u8 = C_BMI160_ZERO_U8X;
	u8 v_gyro_index_u8 = C_BMI160_ZERO_U8X;
	u8 v_mag_index_u8 = C_BMI160_ZERO_U8X;
	s8 v_last_return_stat_s8 = C_BMI160_ZERO_U8X;
	u16 v_fifo_index_u16 = C_BMI160_ZERO_U8X;
	u16 v_mag_data_r_u16 = C_BMI160_ZERO_U8X;
	u8 v_frame_head_u8 = C_BMI160_ZERO_U8X;
	u8 *ptr_fifo_value_u8;
	s16 v_mag_data_s16 = C_BMI160_ZERO_U8X;
	s16 v_mag_x_s16, v_mag_y_s16, v_mag_z_s16 = C_BMI160_ZERO_U8X;
	u16 v_mag_r_s16 = C_BMI160_ZERO_U8X;
	/* read fifo v_data_u8*/
	com_rslt = bmi160_fifo_data(&v_fifo_data_u8[0]);
	ptr_fifo_value_u8 = v_fifo_data_u8;
	for (v_fifo_index_u16 = 0; v_fifo_index_u16 < v_fifo_length_u32;) {
		v_frame_head_u8 = ptr_fifo_value_u8[v_fifo_index_u16];
		switch (v_frame_head_u8) {
		/* Header frame of accel */
		case FIFO_HEAD_A:
		{	/*fifo v_data_u8 frame index + 1*/
			v_fifo_index_u16 = v_fifo_index_u16 + 1;

			if (v_fifo_index_u16 + 6 > v_fifo_length_u32) {
				v_last_return_stat_s8 = FIFO_A_OVER_LEN;
			break;
			}
			/* Accel raw x v_data_u8 */
			accel_fifo[v_accel_index_u8].x =
			*(ptr_fifo_value_u8 + v_fifo_index_u16 + 1) << 8
			| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 0);
			/* Accel raw y v_data_u8 */
			accel_fifo[v_accel_index_u8].y =
			*(ptr_fifo_value_u8 + v_fifo_index_u16 + 3) << 8
			| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 2);
			/* Accel raw z v_data_u8 */
			accel_fifo[v_accel_index_u8].z =
			*(ptr_fifo_value_u8 + v_fifo_index_u16 + 5) << 8
			| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 4);
			/* index adde to 6 accel alone*/
			v_fifo_index_u16 = v_fifo_index_u16 + 6;
			v_accel_index_u8++;

		break;
		}
		/* Header frame of gyro */
		case FIFO_HEAD_G:
		{	/*fifo v_data_u8 frame index + 1*/
			v_fifo_index_u16 = v_fifo_index_u16 + 1;

			if (v_fifo_index_u16 + 6 > v_fifo_length_u32) {
				v_last_return_stat_s8 = FIFO_G_OVER_LEN;
			break;
			}
			/* Gyro raw x v_data_u8 */
			gyro_fifo[v_gyro_index_u8].x  =
			*(ptr_fifo_value_u8 + v_fifo_index_u16 + 1) << 8
			| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 0);
			/* Gyro raw y v_data_u8 */
			gyro_fifo[v_gyro_index_u8].y =
			*(ptr_fifo_value_u8 + v_fifo_index_u16 + 3) << 8
			| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 2);
			/* Gyro raw z v_data_u8 */
			gyro_fifo[v_gyro_index_u8].z  =
			*(ptr_fifo_value_u8 + v_fifo_index_u16 + 5) << 8
			| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 4);
			/*fifo G v_data_u8 frame index + 6*/
			v_fifo_index_u16 = v_fifo_index_u16 + 6;
			v_gyro_index_u8++;

		break;
		}
		/* Header frame of mag */
		case FIFO_HEAD_M:
		{	/*fifo v_data_u8 frame index + 1*/
			v_fifo_index_u16 = v_fifo_index_u16 + 1;

			if (v_fifo_index_u16 + 8 > v_fifo_length_u32) {
				v_last_return_stat_s8 = FIFO_M_OVER_LEN;
			break;
			}
			/* Raw mag x*/
			v_mag_data_s16 =
			*(ptr_fifo_value_u8 + v_fifo_index_u16 + 1) << 8
			| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 0);
			v_mag_x_s16 = (s16) (v_mag_data_s16 >> 3);
			/* Raw mag y*/
			v_mag_data_s16 =
			*(ptr_fifo_value_u8 + v_fifo_index_u16 + 3) << 8
			| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 2);
			v_mag_y_s16 = (s16) (v_mag_data_s16 >> 3);
			/* Raw mag z*/
			v_mag_data_s16  =
			*(ptr_fifo_value_u8 + v_fifo_index_u16 + 5) << 8
			| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 4);
			v_mag_z_s16 = (s16)(v_mag_data_s16 >> 1);
			/* Raw mag r*/
			v_mag_data_r_u16 =
			*(ptr_fifo_value_u8 + v_fifo_index_u16 + 7) << 8
			| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 6);
			v_mag_r_s16 = (u16) (v_mag_data_r_u16 >> 2);
			/* Compensated mag x v_data_u8 */
			mag_fifo[v_mag_index_u8].x =
			bmi160_mag_compensate_X(v_mag_x_s16, v_mag_r_s16);
			/* Compensated mag y v_data_u8 */
			mag_fifo[v_mag_index_u8].y =
			bmi160_mag_compensate_Y(v_mag_y_s16, v_mag_r_s16);
			/* Compensated mag z v_data_u8 */
			mag_fifo[v_mag_index_u8].z =
			bmi160_mag_compensate_Z(v_mag_z_s16, v_mag_r_s16);

			v_mag_index_u8++;
			/*fifo M v_data_u8 frame index + 8*/
			v_fifo_index_u16 = v_fifo_index_u16 + 8;
		break;
		}
		/* Header frame of gyro and accel */
		case FIFO_HEAD_G_A:
			v_fifo_index_u16 = v_fifo_index_u16 + 1;
			/* Raw gyro x */
			gyro_fifo[v_gyro_index_u8].x   =
			*(ptr_fifo_value_u8 + v_fifo_index_u16 + 1) << 8
			| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 0);
			/* Raw gyro y */
			gyro_fifo[v_gyro_index_u8].y  =
			*(ptr_fifo_value_u8 + v_fifo_index_u16 + 3) << 8
			| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 2);
			/* Raw gyro z */
			gyro_fifo[v_gyro_index_u8].z  =
			*(ptr_fifo_value_u8 + v_fifo_index_u16 + 5) << 8
			| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 4);
			/* Raw accel x */
			accel_fifo[v_accel_index_u8].x =
			*(ptr_fifo_value_u8 + v_fifo_index_u16 + 7) << 8
			| *(ptr_fifo_value_u8+v_fifo_index_u16 + 6);
			/* Raw accel y */
			accel_fifo[v_accel_index_u8].y =
			*(ptr_fifo_value_u8 + v_fifo_index_u16 + 9) << 8
			| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 8);
			/* Raw accel z */
			accel_fifo[v_accel_index_u8].z =
			*(ptr_fifo_value_u8 + v_fifo_index_u16 + 11) << 8
			| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 10);
			/* Index added to 12 for gyro and accel*/
			v_fifo_index_u16 = v_fifo_index_u16 + 12;
			v_gyro_index_u8++;
			v_accel_index_u8++;
		break;
		/* Header frame of mag, gyro and accel */
		case FIFO_HEAD_M_G_A:
			{	/*fifo v_data_u8 frame index + 1*/
			v_fifo_index_u16 = v_fifo_index_u16 + 1;

			if (v_fifo_index_u16 + 20 > v_fifo_length_u32) {
				v_last_return_stat_s8 = FIFO_M_G_A_OVER_LEN;
				break;
			}
			/* Mag raw x v_data_u8 */
			v_mag_data_s16 =
				*(ptr_fifo_value_u8 + v_fifo_index_u16 + 1) << 8
				| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 0);
			v_mag_x_s16 = (s16) (v_mag_data_s16 >> 3);
			/* Mag raw y v_data_u8 */
			v_mag_data_s16 =
				*(ptr_fifo_value_u8 + v_fifo_index_u16 + 3) << 8
				| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 2);
			v_mag_y_s16 = (s16) (v_mag_data_s16 >> 3);
			/* Mag raw z v_data_u8 */
			v_mag_data_s16  =
				*(ptr_fifo_value_u8 + v_fifo_index_u16 + 5) << 8
				| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 4);
			v_mag_z_s16 = (s16)(v_mag_data_s16 >> 1);
			/* Mag raw r v_data_u8 */
			v_mag_data_r_u16 =
				*(ptr_fifo_value_u8 + v_fifo_index_u16 + 7) << 8
				| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 6);
			v_mag_r_s16 = (u16) (v_mag_data_r_u16 >> 2);
			/* Mag x compensation */
			mag_fifo[v_mag_index_u8].x =
			bmi160_mag_compensate_X(v_mag_x_s16, v_mag_r_s16);
			/* Mag y compensation */
			mag_fifo[v_mag_index_u8].y =
			bmi160_mag_compensate_Y(v_mag_y_s16, v_mag_r_s16);
			/* Mag z compensation */
			mag_fifo[v_mag_index_u8].z =
			bmi160_mag_compensate_Z(v_mag_z_s16, v_mag_r_s16);
			/* Gyro raw x v_data_u8 */
			gyro_fifo[v_gyro_index_u8].x =
				*(ptr_fifo_value_u8 + v_fifo_index_u16 + 9) << 8
				| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 8);
			/* Gyro raw y v_data_u8 */
			gyro_fifo[v_gyro_index_u8].y =
				*(ptr_fifo_value_u8 +
				v_fifo_index_u16 + 11) << 8
				| *(ptr_fifo_value_u8 +
				v_fifo_index_u16 + 10);
			/* Gyro raw z v_data_u8 */
			gyro_fifo[v_gyro_index_u8].z =
				*(ptr_fifo_value_u8 +
				v_fifo_index_u16 + 13) << 8
				| *(ptr_fifo_value_u8 +
				v_fifo_index_u16 + 12);
			/* Accel raw x v_data_u8 */
			accel_fifo[v_accel_index_u8].x =
				*(ptr_fifo_value_u8 +
				v_fifo_index_u16 + 15) << 8
				| *(ptr_fifo_value_u8+v_fifo_index_u16 + 14);
			/* Accel raw y v_data_u8 */
			accel_fifo[v_accel_index_u8].y =
				*(ptr_fifo_value_u8 +
				v_fifo_index_u16 + 17) << 8
				| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 16);
			/* Accel raw z v_data_u8 */
			accel_fifo[v_accel_index_u8].z =
				*(ptr_fifo_value_u8 +
				v_fifo_index_u16 + 19) << 8
				| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 18);
			/* Index adde to 20 for mag, gyro and accel*/
			v_fifo_index_u16 = v_fifo_index_u16 + 20;
			v_accel_index_u8++;
			v_mag_index_u8++;
			v_gyro_index_u8++;
		break;
			}
		/* Header frame of mag and accel */
		case FIFO_HEAD_M_A:
			{	/*fifo v_data_u8 frame index + 1*/
			v_fifo_index_u16 = v_fifo_index_u16 + 1;

			if (v_fifo_index_u16 + 14 > v_fifo_length_u32) {
				v_last_return_stat_s8 = FIFO_M_A_OVER_LEN;
				break;
			}
			/* mag raw x v_data_u8 */
			v_mag_data_s16 =
			*(ptr_fifo_value_u8 + v_fifo_index_u16 + 1) << 8
			| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 0);
			v_mag_x_s16 = (s16) (v_mag_data_s16 >> 3);
			/* mag raw y v_data_u8 */
			v_mag_data_s16 =
			*(ptr_fifo_value_u8 + v_fifo_index_u16 + 3) << 8
			| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 2);
			v_mag_y_s16 = (s16) (v_mag_data_s16 >> 3);
			/* mag raw z v_data_u8 */
			v_mag_data_s16  =
			*(ptr_fifo_value_u8 + v_fifo_index_u16 + 5) << 8
			| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 4);
			v_mag_z_s16 = (s16)(v_mag_data_s16 >> 1);
			/* mag raw r v_data_u8 */
			v_mag_data_r_u16 =
			*(ptr_fifo_value_u8 + v_fifo_index_u16 + 7) << 8
			| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 6);
			v_mag_r_s16 = (u16) (v_mag_data_r_u16 >> 2);
			/* Mag x compensation */
			mag_fifo[v_mag_index_u8].x =
			bmi160_mag_compensate_X(v_mag_x_s16, v_mag_r_s16);
			/* Mag y compensation */
			mag_fifo[v_mag_index_u8].y =
			bmi160_mag_compensate_Y(v_mag_y_s16, v_mag_r_s16);
			/* Mag z compensation */
			mag_fifo[v_mag_index_u8].z =
			bmi160_mag_compensate_Z(v_mag_z_s16, v_mag_r_s16);
			/* Accel raw x v_data_u8 */
			accel_fifo[v_accel_index_u8].x =
			*(ptr_fifo_value_u8 + v_fifo_index_u16 + 9) << 8
			| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 8);
			/* Accel raw y v_data_u8 */
			accel_fifo[v_accel_index_u8].y =
			*(ptr_fifo_value_u8 + v_fifo_index_u16 + 11) << 8
			| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 10);
			/* Accel raw z v_data_u8 */
			accel_fifo[v_accel_index_u8].z =
			*(ptr_fifo_value_u8 + v_fifo_index_u16 + 13) << 8
			| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 12);
			/*fifo AM v_data_u8 frame index + 14(8+6)*/
			v_fifo_index_u16 = v_fifo_index_u16 + 14;
			v_accel_index_u8++;
			v_mag_index_u8++;
		break;
			}
			/* Header frame of mag and gyro */
		case FIFO_HEAD_M_G:
			{
			/*fifo v_data_u8 frame index + 1*/
			v_fifo_index_u16 = v_fifo_index_u16 + 1;

			if (v_fifo_index_u16 + 14 > v_fifo_length_u32) {
				v_last_return_stat_s8 = FIFO_M_G_OVER_LEN;
				break;
			}
			/* Mag raw x v_data_u8 */
			v_mag_data_s16 =
			*(ptr_fifo_value_u8 + v_fifo_index_u16 + 1) << 8
			| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 0);
			v_mag_x_s16 = (s16) (v_mag_data_s16 >> 3);
			/* Mag raw y v_data_u8 */
			v_mag_data_s16 =
			*(ptr_fifo_value_u8 + v_fifo_index_u16 + 3) << 8
			| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 2);
			v_mag_y_s16 = (s16) (v_mag_data_s16 >> 3);
			/* Mag raw z v_data_u8 */
			v_mag_data_s16  =
			*(ptr_fifo_value_u8 + v_fifo_index_u16 + 5) << 8
			| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 4);
			v_mag_z_s16 = (s16)(v_mag_data_s16 >> 1);
			/* Mag raw r v_data_u8 */
			v_mag_data_r_u16 =
			*(ptr_fifo_value_u8 + v_fifo_index_u16 + 7) << 8
			| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 6);
			v_mag_r_s16 = (u16) (v_mag_data_r_u16 >> 2);
			/* Mag x compensation */
			mag_fifo[v_mag_index_u8].x =
			bmi160_mag_compensate_X(v_mag_x_s16, v_mag_r_s16);
			/* Mag y compensation */
			mag_fifo[v_mag_index_u8].y =
			bmi160_mag_compensate_Y(v_mag_y_s16, v_mag_r_s16);
			/* Mag z compensation */
			mag_fifo[v_mag_index_u8].z =
			bmi160_mag_compensate_Z(v_mag_z_s16, v_mag_r_s16);
			/* Gyro raw x v_data_u8 */
			gyro_fifo[v_gyro_index_u8].x =
			*(ptr_fifo_value_u8 + v_fifo_index_u16 + 9) << 8
			| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 8);
			/* Gyro raw y v_data_u8 */
			gyro_fifo[v_gyro_index_u8].y =
			*(ptr_fifo_value_u8 + v_fifo_index_u16 + 11) << 8
			| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 10);
			/* Gyro raw z v_data_u8 */
			gyro_fifo[v_gyro_index_u8].z =
			*(ptr_fifo_value_u8 + v_fifo_index_u16 + 13) << 8
			| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 12);
			/*fifo GM v_data_u8 frame index + 14(8+6)*/
			v_fifo_index_u16 = v_fifo_index_u16 + 14;
			v_mag_index_u8++;
			v_gyro_index_u8++;
		break;
			}
		/* Header frame of sensor time */
		case FIFO_HEAD_SENSOR_TIME:
			v_fifo_index_u16 = v_fifo_index_u16 + 1;

			if (v_fifo_index_u16 + 3 > v_fifo_length_u32) {
				v_last_return_stat_s8 = FIFO_SENSORTIME_RETURN;
			break;
			}
			/* Sensor time */
			V_fifo_time_U32 = (*(ptr_fifo_value_u8 +
			v_fifo_index_u16 + 2) << 16 |
			*(ptr_fifo_value_u8 + v_fifo_index_u16 + 1) << 8 |
			*(ptr_fifo_value_u8 + v_fifo_index_u16 + 0));

			v_fifo_index_u16 = v_fifo_index_u16 + 3;
		break;
		/* Header frame of skip frame */
		case FIFO_HEAD_SKIP_FRAME:
			/*fifo v_data_u8 frame index + 1*/
				v_fifo_index_u16 = v_fifo_index_u16 + 1;
				if (v_fifo_index_u16 + 1 > v_fifo_length_u32) {
					v_last_return_stat_s8 =
					FIFO_SKIP_OVER_LEN;
				break;
				}
				v_fifo_index_u16 = v_fifo_index_u16 + 1;
		break;
		/* Header frame of over read fifo v_data_u8 */
		case FIFO_HEAD_OVER_READ_LSB:
			{
		/*fifo v_data_u8 frame index + 1*/
			v_fifo_index_u16 = v_fifo_index_u16 + 1;

			if (v_fifo_index_u16 + 1 > v_fifo_length_u32) {
				v_last_return_stat_s8 = FIFO_OVER_READ_RETURN;
			break;
			}
			if (ptr_fifo_value_u8[v_fifo_index_u16] ==
			FIFO_HEAD_OVER_READ_MSB) {
				/*fifo over read frame index + 1*/
				v_fifo_index_u16 = v_fifo_index_u16 + 1;
			break;
			} else {
				v_last_return_stat_s8 = FIFO_OVER_READ_RETURN;
			break;
			}
			}

		default:
			v_last_return_stat_s8 = 1;
		break;
		}
	if (v_last_return_stat_s8)
		break;
	}
return com_rslt;
}
/***************************************************************************
 *	Description: This function used for reading the
 *	fifo header less mode v_data_u8
 *
 *  \return results of communication routine
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_read_fifo_headerless_mode(
u32 v_fifo_length_u32) {
u8 mag_en = C_BMI160_ZERO_U8X;
u8 accel_en = C_BMI160_ZERO_U8X;
u8 gyro_en = C_BMI160_ZERO_U8X;
u8 *ptr_fifo_value_u8;
u32 v_fifo_index_u16 = C_BMI160_ZERO_U8X;
s16 v_mag_data_s16 =  C_BMI160_ZERO_U8X;
u16 v_mag_data_r_u16 =  C_BMI160_ZERO_U8X;
u16 v_mag_r_s16 =  C_BMI160_ZERO_U8X;
s16 v_mag_x_s16, v_mag_y_s16, v_mag_z_s16 = C_BMI160_ZERO_U8X;
u8 v_accel_index_u8 = C_BMI160_ZERO_U8X;
u8 v_gyro_index_u8 = C_BMI160_ZERO_U8X;
u8 v_mag_index_u8 = C_BMI160_ZERO_U8X;
BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
com_rslt = bmi160_set_fifo_header_enable(0x00);
com_rslt += bmi160_fifo_data(&v_fifo_data_u8[0]);
ptr_fifo_value_u8 = v_fifo_data_u8;
for (v_fifo_index_u16 = 0; v_fifo_index_u16 < v_fifo_length_u32;) {
	/* condition for mag, gyro and accel enable*/
	if (mag_en == C_BMI160_ONE_U8X && gyro_en == C_BMI160_ONE_U8X
		&& accel_en == C_BMI160_ONE_U8X) {
		/* Raw mag x*/
		v_mag_data_s16 =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 1) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 0);
		v_mag_x_s16 = (s16) (v_mag_data_s16 >> 3);
		/* Raw mag y*/
		v_mag_data_s16 =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 3) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 2);
		v_mag_y_s16 = (s16) (v_mag_data_s16 >> 3);
		/* Raw mag z*/
		v_mag_data_s16  =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 5) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 4);
		v_mag_z_s16 = (s16)(v_mag_data_s16 >> 1);
		/* Raw mag r*/
		v_mag_data_r_u16 =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 7) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 6);
		v_mag_r_s16 = (u16) (v_mag_data_r_u16 >> 2);
		/* Compensated mag x v_data_u8 */
		mag_fifo[v_mag_index_u8].x =
		bmi160_mag_compensate_X(v_mag_x_s16, v_mag_r_s16);
		/* Compensated mag y v_data_u8 */
		mag_fifo[v_mag_index_u8].y =
		bmi160_mag_compensate_Y(v_mag_y_s16, v_mag_r_s16);
		/* Compensated mag z v_data_u8 */
		mag_fifo[v_mag_index_u8].z =
		bmi160_mag_compensate_Z(v_mag_z_s16, v_mag_r_s16);
		/* Gyro raw x v_data_u8 */
		gyro_fifo[v_gyro_index_u8].x  =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 9) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 8);
		/* Gyro raw y v_data_u8 */
		gyro_fifo[v_gyro_index_u8].y =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 11) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 10);
		/* Gyro raw z v_data_u8 */
		gyro_fifo[v_gyro_index_u8].z  =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 13) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 12);
		/* Accel raw x v_data_u8 */
		accel_fifo[v_accel_index_u8].x =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 15) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 14);
		/* Accel raw y v_data_u8 */
		accel_fifo[v_accel_index_u8].y =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 17) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 16);
		/* Accel raw z v_data_u8 */
		accel_fifo[v_accel_index_u8].z =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 19) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 18);
		v_accel_index_u8++;
		v_mag_index_u8++;
		v_gyro_index_u8++;
	   v_fifo_index_u16 = v_fifo_index_u16 + 20;
	}
	/* condition for mag and gyro enable*/
	else if (mag_en == C_BMI160_ONE_U8X && gyro_en == C_BMI160_ONE_U8X
	&& accel_en == C_BMI160_ZERO_U8X) {
		/* Raw mag x*/
		v_mag_data_s16 =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 1) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 0);
		v_mag_x_s16 = (s16) (v_mag_data_s16 >> 3);
		/* Raw mag y*/
		v_mag_data_s16 =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 3) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 2);
		v_mag_y_s16 = (s16) (v_mag_data_s16 >> 3);
		/* Raw mag z*/
		v_mag_data_s16  =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 5) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 4);
		v_mag_z_s16 = (s16)(v_mag_data_s16 >> 1);
		/* Raw mag r*/
		v_mag_data_r_u16 =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 7) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 6);
		v_mag_r_s16 = (u16) (v_mag_data_r_u16 >> 2);
		/* Compensated mag x v_data_u8 */
		mag_fifo[v_mag_index_u8].x =
		bmi160_mag_compensate_X(v_mag_x_s16, v_mag_r_s16);
		/* Compensated mag y v_data_u8 */
		mag_fifo[v_mag_index_u8].y =
		bmi160_mag_compensate_Y(v_mag_y_s16, v_mag_r_s16);
		/* Compensated mag z v_data_u8 */
		mag_fifo[v_mag_index_u8].z =
		bmi160_mag_compensate_Z(v_mag_z_s16, v_mag_r_s16);
		/* Gyro raw x v_data_u8 */
		gyro_fifo[v_gyro_index_u8].x  =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 9) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 8);
		/* Gyro raw y v_data_u8 */
		gyro_fifo[v_gyro_index_u8].y =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 11) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 10);
		/* Gyro raw z v_data_u8 */
		gyro_fifo[v_gyro_index_u8].z  =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 13) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 12);
		v_gyro_index_u8++;
		v_mag_index_u8++;
		v_fifo_index_u16 = v_fifo_index_u16 + 14;
	}
	/* condition for mag and accel enable*/
	else if (mag_en == C_BMI160_ONE_U8X && accel_en == C_BMI160_ONE_U8X
	&& gyro_en == C_BMI160_ZERO_U8X) {
		/* Raw mag x*/
		v_mag_data_s16 =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 1) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 0);
		v_mag_x_s16 = (s16) (v_mag_data_s16 >> 3);
		/* Raw mag y*/
		v_mag_data_s16 =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 3) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 2);
		v_mag_y_s16 = (s16) (v_mag_data_s16 >> 3);
		/* Raw mag z*/
		v_mag_data_s16  =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 5) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 4);
		v_mag_z_s16 = (s16)(v_mag_data_s16 >> 1);
		/* Raw mag r*/
		v_mag_data_r_u16 =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 7) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 6);
		v_mag_r_s16 = (u16) (v_mag_data_r_u16 >> 2);
		/* Compensated mag x v_data_u8 */
		mag_fifo[v_mag_index_u8].x =
		bmi160_mag_compensate_X(v_mag_x_s16, v_mag_r_s16);
		/* Compensated mag y v_data_u8 */
		mag_fifo[v_mag_index_u8].y =
		bmi160_mag_compensate_Y(v_mag_y_s16, v_mag_r_s16);
		/* Compensated mag z v_data_u8 */
		mag_fifo[v_mag_index_u8].z =
		bmi160_mag_compensate_Z(v_mag_z_s16, v_mag_r_s16);
		/* Accel raw x v_data_u8 */
		accel_fifo[v_accel_index_u8].x =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 9) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 8);
		/* Accel raw y v_data_u8 */
		accel_fifo[v_accel_index_u8].y =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 11) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 10);
		/* Accel raw z v_data_u8 */
		accel_fifo[v_accel_index_u8].z =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 13) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 12);
		v_accel_index_u8++;
		v_mag_index_u8++;
		v_fifo_index_u16 = v_fifo_index_u16 + 14;
	}
	/* condition for gyro and accel enable*/
	else if (gyro_en == C_BMI160_ONE_U8X && accel_en == C_BMI160_ONE_U8X
	&& mag_en == C_BMI160_ZERO_U8X) {
		/* Gyro raw x v_data_u8 */
		gyro_fifo[v_gyro_index_u8].x  =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 1) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 0);
		/* Gyro raw y v_data_u8 */
		gyro_fifo[v_gyro_index_u8].y =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 3) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 2);
		/* Gyro raw z v_data_u8 */
		gyro_fifo[v_gyro_index_u8].z  =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 5) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 4);
		/* Accel raw x v_data_u8 */
		accel_fifo[v_accel_index_u8].x =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 7) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 6);
		/* Accel raw y v_data_u8 */
		accel_fifo[v_accel_index_u8].y =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 9) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 8);
		/* Accel raw z v_data_u8 */
		accel_fifo[v_accel_index_u8].z =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 11) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 10);
		v_accel_index_u8++;
		v_gyro_index_u8++;
		v_fifo_index_u16 = v_fifo_index_u16 + 12;
	}
	/* condition  for gyro enable*/
	else if (gyro_en == C_BMI160_ONE_U8X && accel_en == C_BMI160_ZERO_U8X
	&& mag_en == C_BMI160_ZERO_U8X) {
		/* Gyro raw x v_data_u8 */
		gyro_fifo[v_gyro_index_u8].x  =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 1) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 0);
		/* Gyro raw y v_data_u8 */
		gyro_fifo[v_gyro_index_u8].y =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 3) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 2);
		/* Gyro raw z v_data_u8 */
		gyro_fifo[v_gyro_index_u8].z  =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 5) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 4);
		v_fifo_index_u16 = v_fifo_index_u16 + 6;
		v_gyro_index_u8++;
	}
	/* condition  for accel enable*/
	else if (gyro_en == C_BMI160_ZERO_U8X && accel_en == C_BMI160_ONE_U8X
	&& mag_en == C_BMI160_ZERO_U8X) {
		/* Accel raw x v_data_u8 */
		accel_fifo[v_accel_index_u8].x =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 1) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 0);
		/* Accel raw y v_data_u8 */
		accel_fifo[v_accel_index_u8].y =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 3) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 2);
		/* Accel raw z v_data_u8 */
		accel_fifo[v_accel_index_u8].z =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 5) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 4);
		v_fifo_index_u16 = v_fifo_index_u16 + 6;
		v_accel_index_u8++;
	}
	/* condition  for accel enable*/
	else if (gyro_en == C_BMI160_ZERO_U8X && accel_en == C_BMI160_ZERO_U8X
	&& mag_en == C_BMI160_ONE_U8X) {
		/* Raw mag x*/
		v_mag_data_s16 =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 1) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 0);
		v_mag_x_s16 = (s16) (v_mag_data_s16 >> 3);
		/* Raw mag y*/
		v_mag_data_s16 =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 3) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 2);
		v_mag_y_s16 = (s16) (v_mag_data_s16 >> 3);
		/* Raw mag z*/
		v_mag_data_s16  =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 5) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 4);
		v_mag_z_s16 = (s16)(v_mag_data_s16 >> 1);
		/* Raw mag r*/
		v_mag_data_r_u16 =
		*(ptr_fifo_value_u8 + v_fifo_index_u16 + 7) << 8
		| *(ptr_fifo_value_u8 + v_fifo_index_u16 + 6);
		v_mag_r_s16 = (u16) (v_mag_data_r_u16 >> 2);
		/* Compensated mag x v_data_u8 */
		mag_fifo[v_mag_index_u8].x =
		bmi160_mag_compensate_X(v_mag_x_s16, v_mag_r_s16);
		/* Compensated mag y v_data_u8 */
		mag_fifo[v_mag_index_u8].y =
		bmi160_mag_compensate_Y(v_mag_y_s16, v_mag_r_s16);
		/* Compensated mag z v_data_u8 */
		mag_fifo[v_mag_index_u8].z =
		bmi160_mag_compensate_Z(v_mag_z_s16, v_mag_r_s16);
		v_fifo_index_u16 = v_fifo_index_u16 + 8;
		v_mag_index_u8++;
	}
	/* condition  for fifo over read enable*/
	if (ptr_fifo_value_u8[v_fifo_index_u16] == 0x00 &&
	ptr_fifo_value_u8[v_fifo_index_u16+1] == 0x80 &&
	ptr_fifo_value_u8[v_fifo_index_u16+2] == 0x00 &&
	ptr_fifo_value_u8[v_fifo_index_u16+3] == 0x80) {
		return FIFO_OVER_READ_RETURN;
		}
	}
	return com_rslt;
}
/***************************************************************************
 *	Description: This function used for reading
 *	bmi160_t structure
 *
 *  \return the reference and values of bmi160_t
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
struct bmi160_t *bmi160_get_ptr(void)
{
	return  p_bmi160;
}

/**************************************************************************************************
* Copyright (C) 2014 NXP
* Following code is adoption layer to NXP framework
**************************************************************************************************/
static void WriteBMIRegister(uint8_t address, uint8_t value, uint32_t delaymSeconds)
{
    dev_i2c_write(p_bmi160->dev_addr, address, &value, 1);
   
    if (delaymSeconds) {
        dev_i2c_delay(delaymSeconds);
    }
}

static int32_t gyro_SetInterval(PhysicalSensor_t *pSens, uint32_t msec)
{
	unsigned char gyroConfig = (2<<4); /* default gyr_bwp value */
    uint32_t freq = (1000 / msec);
    
	/* Possible data rates in Hz - 100, 200, 400, 1K, 2K */
    if (freq >= 3200) {
        gyroConfig |= 13;
    }
    else if (freq >= 1600) {
        gyroConfig |= 12;
    }
    else if (freq >= 800) {
        gyroConfig |= 11;
    }
    else if (freq >= 400) {
        gyroConfig |= 10;
    }
    else if (freq >= 200) {
        gyroConfig |= 9;
    }
    else if (freq >= 100) {
        gyroConfig |= 8;
    }
    else if (freq >= 50) {
        gyroConfig |= 7;
    }
    else if (freq >= 25) {
        gyroConfig |= 6;
    }
    else if (freq >= 12) {
        gyroConfig |= 5;
    }
    else if (freq >= 6) {
        gyroConfig |= 4;
    }
    else if (freq >= 3) {
        gyroConfig |= 3;
    }
    else if (freq >= 2) {
        gyroConfig |= 2;
    }
    else {
        gyroConfig |= 1;
    }
    
    pSens->period = g_Timer.getTicksFromMs(msec);
    
    WriteBMIRegister(BMI160_USER_GYRO_CONFIG_ADDR, gyroConfig, 0);

	return 0;
}

static struct bmi160_t *gyro_construct(void)
{
    static struct bmi160_t g_bmi160;
    
    g_bmi160.dev_addr = BMI160_I2C_ADDR1;
    g_bmi160.bus_write  = dev_i2c_write;
	g_bmi160.bus_read   = dev_i2c_read;
	g_bmi160.delay_msec = dev_i2c_delay;
    
    return &g_bmi160;
}

static int32_t gyro_init(PhysicalSensor_t *pSens)
{
	/* Init the Bosch Driver for Gyro */
	bmi160_init(gyro_construct());

	/* Reset the sensor */
    WriteBMIRegister(BMI160_CMD_COMMANDS_ADDR, 0xb6, 100);
    
    /* Set fast power up mode */
    WriteBMIRegister(BMI160_CMD_COMMANDS_ADDR, 0x17, 60);
    
	gyro_SetInterval(pSens, 10);
#if defined(BSX_LITE)
	bmi160_set_gyro_range(C_BMI160_TWO_U8X);// Zero = 500dps
#else
	bmi160_set_gyr_range(C_BMI160_ZERO_U8X);// Zero = 2000dps
#endif

	/* GYRO INT1 irq setup */
	NVIC_DisableIRQ(GYRO_PINT_IRQn);
	NVIC_SetPriority(GYRO_PINT_IRQn, SENSOR_IRQ_PRIORITY);

	Chip_GPIO_SetPinDIRInput(LPC_GPIO, GYRO_INT_PORT, GYRO_INT_PIN);
	Chip_INMUX_PinIntSel(GYRO_PINT_SEL, GYRO_INT_PORT, GYRO_INT_PIN);/* Configure INMUX block */
	Chip_PININT_SetPinModeEdge(LPC_PININT, GYRO_PINT_CH);	/* edge sensitive and rising edge interrupt */
	Chip_PININT_EnableIntHigh(LPC_PININT, GYRO_PINT_CH);
	Chip_SYSCON_EnableWakeup(GYRO_WAKE);/* enable to wake from sleep */
	//	Chip_SYSCTL_EnableWakeup(STARTERP0_CTIMER0);/* enable to wake from sleep */ /* ##KW## Not sure why? */
    /* Make sure there aren't any pending IRQ's */
    Chip_PININT_ClearIntStatus(LPC_PININT, GYRO_PINT_CH);
	return 0;
}

static int32_t gyro_read(PhysicalSensor_t *pSens)
{
#if 1
	struct bmi160_gyro_t data;

	/* Uses Bosch library functions for sensor data read */
	/* ##KW## Read counts or diffs? */
	bmi160_read_gyro_xyz(&data);

	pSens->data16[0] = (uint16_t) data.x;
	pSens->data16[1] = (uint16_t) data.y;
	pSens->data16[2] = (uint16_t) data.z;

#else
	/* ##KW## Optimized single read function is possible? */
#endif

	return 0;
}

static int32_t gyro_activate(PhysicalSensor_t *pSens, bool enable)
{
	if (enable) {
		/* Normal mode */
        WriteBMIRegister(BMI160_CMD_COMMANDS_ADDR, 0x15, 1);
        
        #if 0
		bmi160_set_int_od(BMI160_INT2_OD, 0); /* Set as output */
        bmi160_set_int_edge_ctrl(BMI160_INT2_LVL, 1); /* set active high */ 
		bmi160_set_int_drdy(BMI160_INT2_MAP_DRDY, 1); /*enable DRDY irq IRQ2 channel */
		bmi160_set_latch_int(7);
		bmi160_set_output_en(BMI160_INT2_OUTPUT_EN, 1); /* enable IRQ2 for output */
        #endif
        
         /* Enable DRDY IRQ ONLY */
        WriteBMIRegister(BMI160_USER_INTR_ENABLE_0_ADDR, 0, 0);
        WriteBMIRegister(BMI160_USER_INTR_ENABLE_1_ADDR, 0x10, 0);
        WriteBMIRegister(BMI160_USER_INTR_ENABLE_2_ADDR, 0, 0);
        
        WriteBMIRegister(BMI160_USER_INTR_OUT_CTRL_ADDR, 0xa0, 0); /* int2_out_en, push-pull, active high */
        WriteBMIRegister(BMI160_USER_INTR_MAP_1_ADDR, 0x8, 0);  /* IRQ2 enable DRDY */
        
        /* Wait for 1ms after writing */
        dev_i2c_delay(1);
		/* Read to clear device */
		gyro_read(pSens);

		/* Enable interrupt in the NVIC */
		NVIC_EnableIRQ(GYRO_PINT_IRQn);
	}
	else {
        /* Disable DRDY IRQ */
        WriteBMIRegister(BMI160_USER_INTR_ENABLE_1_ADDR, 0, 0);
		
		NVIC_DisableIRQ(GYRO_PINT_IRQn);
        
        /* Set to Fast powerup mode */
        WriteBMIRegister(BMI160_CMD_COMMANDS_ADDR, 0x17, 1);
		
		NVIC_DisableIRQ(GYRO_PINT_IRQn);
	}

	/* ##KW## Optimized register write function? */

	return 0;
}

/**
 * @brief   Sensor interface
 */
static const PhysSensorCtrl_t g_gyroCtrl = {
	gyro_init,          
	gyro_read,          
	gyro_activate,      
	gyro_SetInterval,  /* Calculates and sets data rate */
};

/* Public sensor data / control */
PhysicalSensor_t g_bmi160Gyro = {
	&g_gyroCtrl,        /* Sensor hardware structure */
	{0, },              /* sensor data sample */
	0,                  /* time (watchdog timer ticks) the last sample was acquired */
	0,                  /* time (watchdog timer ticks) the next sample will be taken */
	PHYS_GYRO_ID,       /* sensor ID */
	0,                  /* sensor enabled status*/
	0,                  /* IRQ pending */
	PHYS_MODE_IRQ,      /* data acquisition mode (irq, polled) */
	0,                  /* data rate in mSec */
};
