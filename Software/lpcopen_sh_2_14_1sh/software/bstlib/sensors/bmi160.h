/*
****************************************************************************
* Copyright (C) 2014 Bosch Sensortec GmbH
*
* bmi160.h
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
/*! \file bmi160.h
    \brief BMI160 Sensor Driver Support Header File */
/* user defined code to be added here ... */
#ifndef __BMI160_H__
#define __BMI160_H__

/*******************************************************
* These definition uses for define the data types
********************************************************
* While porting the API please consider the following
* Please check the version of C standard
* Are you using Linux platform
*******************************************************/

/*********************************************************
* This definition uses for the Linux platform support
* Please use the types.h for your data types definitions
*********************************************************/
#ifdef	__KERNEL__

#include <linux/types.h>

#else /* ! __KERNEL__ */
/**********************************************************
* These definition uses for define the C
* standard version data types
***********************************************************/
# if !defined(__STDC_VERSION__)

/************************************************
 * compiler is C11 C standard
************************************************/
#if (__STDC_VERSION__ == 201112L)

/************************************************/
#include <stdint.h>
/************************************************/

/*unsigned integer types*/
#define	u8	uint8_t
#define	u16	uint16_t
#define	u32	uint32_t
#define	u64	uint64_t

/*signed integer types*/
#define	s8	int8_t
#define	s16	int16_t
#define	s32	int32_t
#define	s64	int64_t
/************************************************
 * compiler is C99 C standard
************************************************/

#elif (__STDC_VERSION__ == 199901L)

/* stdint.h is a C99 supported c library.
which is used to fixed the integer size*/
/************************************************/
#include <stdint.h>
/************************************************/

/*unsigned integer types*/
#define	u8	uint8_t
#define	u16	uint16_t
#define	u32	uint32_t
#define	u64	uint64_t

/*signed integer types*/
#define s8	int8_t
#define	s16	int16_t
#define	s32	int32_t
#define	s64	int64_t
/************************************************
 * compiler is C89 or other C standard
************************************************/
#else /*  !defined(__STDC_VERSION__) */
/*	By default it is defined as 32 bit machine configuration*/
/*	define the definition based on your machine configuration*/
/*	define the data types based on your
	machine/compiler/controller configuration*/
#define  MACHINE_32_BIT

/* If your machine support 16 bit
define the MACHINE_16_BIT*/
#ifdef MACHINE_16_BIT
#include <limits.h>
/*signed integer types*/
#define	s8	signed char
#define	s16	signed short int
#define	s32	signed long int

#if defined(LONG_MAX) && LONG_MAX == 0x7fffffffffffffffL
#define s64 long int
#define u64 unsigned long int
#elif defined(LLONG_MAX) && (LLONG_MAX == 0x7fffffffffffffffLL)
#define s64 long long int
#define u64 unsigned long long int
#else
#warning Either the correct data type for signed 64 bit integer \
could not be found, or 64 bit integers are not supported in your environment.
#warning If 64 bit integers are supported on your platform, \
please set s64 manually.
#endif

/*unsigned integer types*/
#define	u8	unsigned char
#define	u16	unsigned short int
#define	u32	unsigned long int

/* If your machine support 32 bit
define the MACHINE_32_BIT*/
#elif defined MACHINE_32_BIT
/*signed integer types*/
#define	s8	signed char
#define	s16	signed short int
#define	s32	signed int
#define	s64	signed long long int

/*unsigned integer types*/
#define	u8	unsigned char
#define	u16	unsigned short int
#define	u32	unsigned int
#define	u64	unsigned long long int

/* If your machine support 64 bit
define the MACHINE_64_BIT*/
#elif defined MACHINE_64_BIT
/*signed integer types*/
#define	s8	signed char
#define	s16	signed short int
#define	s32	signed int
#define	s64	signed long int

/*unsigned integer types*/
#define	u8	unsigned char
#define	u16	unsigned short int
#define	u32	unsigned int
#define	u64	unsigned long int

#else
#warning The data types defined above which not supported \
define the data types manually
#endif
#endif

/*** This else will execute for the compilers
 *	which are not supported the C standards
 *	Like C89/C99/C11***/
#else
/*	By default it is defined as 32 bit machine configuration*/
/*	define the definition based on your machine configuration*/
/*	define the data types based on your
	machine/compiler/controller configuration*/
#define  MACHINE_32_BIT

/* If your machine support 16 bit
define the MACHINE_16_BIT*/
#ifdef MACHINE_16_BIT
#include <limits.h>
/*signed integer types*/
#define	s8	signed char
#define	s16	signed short int
#define	s32	signed long int

#if defined(LONG_MAX) && LONG_MAX == 0x7fffffffffffffffL
#define s64 long int
#define u64 unsigned long int
#elif defined(LLONG_MAX) && (LLONG_MAX == 0x7fffffffffffffffLL)
#define s64 long long int
#define u64 unsigned long long int
#else
#warning Either the correct data type for signed 64 bit integer \
could not be found, or 64 bit integers are not supported in your environment.
#warning If 64 bit integers are supported on your platform, \
please set s64 manually.
#endif

/*unsigned integer types*/
#define	u8	unsigned char
#define	u16	unsigned short int
#define	u32	unsigned long int

/* If your machine support 32 bit
define the MACHINE_32_BIT*/
#elif defined MACHINE_32_BIT
/*signed integer types*/
#define	s8	signed char
#define	s16	signed short int
#define	s32	signed int
#define	s64	signed long long int

/*unsigned integer types*/
#define	u8	unsigned char
#define	u16	unsigned short int
#define	u32	unsigned int
#define	u64	unsigned long long int

/* If your machine support 64 bit
define the MACHINE_64_BIT*/
#elif defined  MACHINE_64_BIT
/*signed integer types*/
#define	s8	signed char
#define	s16	signed short int
#define	s32	signed int
#define	s64	signed long int

/*unsigned integer types*/
#define	u8	unsigned char
#define	u16	unsigned short int
#define	u32	unsigned int
#define	u64	unsigned long int

#else
#warning The data types defined above which not supported \
define the data types manually
#endif
#endif
#endif
/*Example....*/
/*#define YOUR_H_DEFINE */ /**< <Doxy Comment for YOUR_H_DEFINE> */
/** Define the calling convention of YOUR bus communication routine.
	\note This includes types of parameters. This example shows the
	configuration for an SPI bus link.

    If your communication function looks like this:

    write_my_bus_xy(u8 device_addr, u8 register_addr,
    u8 * data, u8 length);

    The BMI160_WR_FUNC_PTR would equal:

    #define     BMI160_WR_FUNC_PTR s8 (* bus_write)(u8,
    u8, u8 *, u8)

    Parameters can be mixed as needed refer to the
    \ref BMI160_BUS_WRITE_FUNC  macro.


*/
#define BMI160_WR_FUNC_PTR s8 (*bus_write)(u8, u8 ,\
u8 *, u8)



/** link macro between API function calls and bus write function
	\note The bus write function can change since this is a
	system dependant issue.

    If the bus_write parameter calling order is like: reg_addr,
    reg_data, wr_len it would be as it is here.

    If the parameters are differently ordered or your communication
    function like I2C need to know the device address,
    you can change this macro accordingly.


    define BMI160_BUS_WRITE_FUNC(dev_addr, reg_addr, reg_data, wr_len)\
    bus_write(dev_addr, reg_addr, reg_data, wr_len)

    This macro lets all API functions call YOUR communication routine in a
    way that equals your definition in the
    \ref BMI160_WR_FUNC_PTR definition.

*/
#define BMI160_BUS_WRITE_FUNC(dev_addr, reg_addr, reg_data, wr_len)\
				bus_write(dev_addr, reg_addr, reg_data, wr_len)


/** Define the calling convention of YOUR bus communication routine.
	\note This includes types of parameters. This example shows the
	configuration for an SPI bus link.

    If your communication function looks like this:

    read_my_bus_xy(u8 device_addr, u8 register_addr,
    u8 * data, u8 length);

    The BMI160_RD_FUNC_PTR would equal:

    #define     BMI160_RD_FUNC_PTR s8 (* bus_read)(u8,
    u8, u8 *, u8)

    Parameters can be mixed as needed refer to the
    \ref BMI160_BUS_READ_FUNC  macro.


*/

#define BMI160_SPI_RD_MASK 0x80   /* for spi read transactions on SPI the
			MSB has to be set */
#define BMI160_RD_FUNC_PTR s8 (*bus_read)(u8,\
			u8 , u8 *, u8)

#define BMI160_BRD_FUNC_PTR s8 \
(*burst_read)(u8, u8, u8 *, u32)

/** link macro between API function calls and bus read function
	\note The bus write function can change since this is a
	system dependant issue.

    If the bus_read parameter calling order is like: reg_addr,
    reg_data, wr_len it would be as it is here.

    If the parameters are differently ordered or your communication
    function like I2C need to know the device address,
    you can change this macro accordingly.


    define BMI160_BUS_READ_FUNC(dev_addr, reg_addr, reg_data, wr_len)\
    bus_read(dev_addr, reg_addr, reg_data, wr_len)

    This macro lets all API functions call YOUR communication routine in a
    way that equals your definition in the
    \ref BMI160_WR_FUNC_PTR definition.

    \note: this macro also includes the "MSB='1'"
    for reading BMI160 addresses.

*/
#define BMI160_BUS_READ_FUNC(dev_addr, reg_addr, reg_data, r_len)\
				bus_read(dev_addr, reg_addr, reg_data, r_len)

#define BMI160_BURST_READ_FUNC(device_addr, \
register_addr, register_data, rd_len)\
burst_read(device_addr, register_addr, register_data, rd_len)


#define BMI160_MDELAY_DATA_TYPE                 u32

/** BMI160 I2C Address
*/
#define BMI160_I2C_ADDR1	0x68 /* I2C Address needs to be changed */
#define BMI160_I2C_ADDR2    0x69 /* I2C Address needs to be changed */

#define         C_BMI160_ZERO_U8X                       ((u8)0)
#define         C_BMI160_ONE_U8X                        ((u8)1)
#define         C_BMI160_TWO_U8X                        ((u8)2)
#define         C_BMI160_THREE_U8X                      ((u8)3)
#define         C_BMI160_FOUR_U8X                       ((u8)4)
#define         C_BMI160_FIVE_U8X                       ((u8)5)
#define         C_BMI160_SIX_U8X                        ((u8)6)
#define         C_BMI160_SEVEN_U8X                      ((u8)7)
#define         C_BMI160_EIGHT_U8X                      ((u8)8)
#define         C_BMI160_NINE_U8X                       ((u8)9)
#define         C_BMI160_ELEVEN_U8X                     ((u8)11)
#define         C_BMI160_TWELVE_U8X                     ((u8)12)
#define         C_BMI160_FOURTEEN_U8X                   ((u8)14)

#define         C_BMI160_FIFTEEN_U8X                    ((u8)15)
#define         C_BMI160_SIXTEEN_U8X                    ((u8)16)
#define         C_BMI160_THIRTYONE_U8X                  ((u8)31)
#define         C_BMI160_THIRTYTWO_U8X                  ((u8)32)
#define         BMI160_MAXIMUM_TIMEOUT                  ((u8)10)

/*  BMI160 API error codes */

#define E_BMI160_NULL_PTR              ((s8)-127)
#define E_BMI160_COMM_RES              ((s8)-1)
#define E_BMI160_OUT_OF_RANGE          ((s8)-2)
#define E_BMI160_EEPROM_BUSY           ((s8)-3)

/* Constants */
#define BMI160_NULL                          0
/*This refers BMI160 return type as s8 */
#define BMI160_RETURN_FUNCTION_TYPE        s8
/*------------------------------------------------------------------------
* Register Definitions of User start
*------------------------------------------------------------------------*/
#define BMI160_USER_CHIP_ID_ADDR				0x00
#define BMI160_USER_ERROR_ADDR					0X02
#define BMI160_USER_PMU_STAT_ADDR				0X03
#define BMI160_USER_DATA_0_ADDR					0X04
#define BMI160_USER_DATA_1_ADDR					0X05
#define BMI160_USER_DATA_2_ADDR					0X06
#define BMI160_USER_DATA_3_ADDR					0X07
#define BMI160_USER_DATA_4_ADDR					0X08
#define BMI160_USER_DATA_5_ADDR					0X09
#define BMI160_USER_DATA_6_ADDR					0X0A
#define BMI160_USER_DATA_7_ADDR					0X0B
#define BMI160_USER_DATA_8_ADDR					0X0C
#define BMI160_USER_DATA_9_ADDR					0X0D
#define BMI160_USER_DATA_10_ADDR				0X0E
#define BMI160_USER_DATA_11_ADDR				0X0F
#define BMI160_USER_DATA_12_ADDR				0X10
#define BMI160_USER_DATA_13_ADDR				0X11
#define BMI160_USER_DATA_14_ADDR				0X12
#define BMI160_USER_DATA_15_ADDR				0X13
#define BMI160_USER_DATA_16_ADDR				0X14
#define BMI160_USER_DATA_17_ADDR				0X15
#define BMI160_USER_DATA_18_ADDR				0X16
#define BMI160_USER_DATA_19_ADDR				0X17
#define BMI160_USER_SENSORTIME_0_ADDR			0X18
#define BMI160_USER_SENSORTIME_1_ADDR			0X19
#define BMI160_USER_SENSORTIME_2_ADDR			0X1A
#define BMI160_USER_STAT_ADDR					0X1B
#define BMI160_USER_INTR_STAT_0_ADDR			0X1C
#define BMI160_USER_INTR_STAT_1_ADDR			0X1D
#define BMI160_USER_INTR_STAT_2_ADDR			0X1E
#define BMI160_USER_INTR_STAT_3_ADDR			0X1F
#define BMI160_USER_TEMPERATURE_0_ADDR			0X20
#define BMI160_USER_TEMPERATURE_1_ADDR			0X21
#define BMI160_USER_FIFO_LENGTH_0_ADDR			0X22
#define BMI160_USER_FIFO_LENGTH_1_ADDR			0X23
#define BMI160_USER_FIFO_DATA_ADDR				0X24
#define BMI160_USER_ACCEL_CONFIG_ADDR				0X40
#define BMI160_USER_ACCEL_RANGE_ADDR              0X41
#define BMI160_USER_GYRO_CONFIG_ADDR               0X42
#define BMI160_USER_GYRO_RANGE_ADDR              0X43
#define BMI160_USER_MAG_CONFIG_ADDR				0X44
#define BMI160_USER_FIFO_DOWN_ADDR             0X45
#define BMI160_USER_FIFO_CONFIG_0_ADDR          0X46
#define BMI160_USER_FIFO_CONFIG_1_ADDR          0X47
#define BMI160_USER_MAG_IF_0_ADDR				0X4B
#define BMI160_USER_MAG_IF_1_ADDR				0X4C
#define BMI160_USER_MAG_IF_2_ADDR				0X4D
#define BMI160_USER_MAG_IF_3_ADDR				0X4E
#define BMI160_USER_MAG_IF_4_ADDR				0X4F
#define BMI160_USER_INTR_ENABLE_0_ADDR				0X50
#define BMI160_USER_INTR_ENABLE_1_ADDR               0X51
#define BMI160_USER_INTR_ENABLE_2_ADDR               0X52
#define BMI160_USER_INTR_OUT_CTRL_ADDR			0X53
#define BMI160_USER_INTR_LATCH_ADDR				0X54
#define BMI160_USER_INTR_MAP_0_ADDR				0X55
#define BMI160_USER_INTR_MAP_1_ADDR				0X56
#define BMI160_USER_INTR_MAP_2_ADDR				0X57
#define BMI160_USER_INTR_DATA_0_ADDR				0X58
#define BMI160_USER_INTR_DATA_1_ADDR				0X59
#define BMI160_USER_INTR_LOWHIGH_0_ADDR			0X5A
#define BMI160_USER_INTR_LOWHIGH_1_ADDR			0X5B
#define BMI160_USER_INTR_LOWHIGH_2_ADDR			0X5C
#define BMI160_USER_INTR_LOWHIGH_3_ADDR			0X5D
#define BMI160_USER_INTR_LOWHIGH_4_ADDR			0X5E
#define BMI160_USER_INTR_MOTION_0_ADDR			0X5F
#define BMI160_USER_INTR_MOTION_1_ADDR			0X60
#define BMI160_USER_INTR_MOTION_2_ADDR			0X61
#define BMI160_USER_INTR_MOTION_3_ADDR			0X62
#define BMI160_USER_INTR_TAP_0_ADDR				0X63
#define BMI160_USER_INTR_TAP_1_ADDR				0X64
#define BMI160_USER_INTR_ORIENT_0_ADDR			0X65
#define BMI160_USER_INTR_ORIENT_1_ADDR			0X66
#define BMI160_USER_INTR_FLAT_0_ADDR				0X67
#define BMI160_USER_INTR_FLAT_1_ADDR				0X68
#define BMI160_USER_FOC_CONFIG_ADDR				0X69
#define BMI160_USER_CONFIG_ADDR					0X6A
#define BMI160_USER_IF_CONFIG_ADDR				0X6B
#define BMI160_USER_PMU_TRIGGER_ADDR			0X6C
#define BMI160_USER_SELF_TEST_ADDR				0X6D
#define BMI160_USER_NV_CONFIG_ADDR				0x70
#define BMI160_USER_OFFSET_0_ADDR				0X71
#define BMI160_USER_OFFSET_1_ADDR				0X72
#define BMI160_USER_OFFSET_2_ADDR				0X73
#define BMI160_USER_OFFSET_3_ADDR				0X74
#define BMI160_USER_OFFSET_4_ADDR				0X75
#define BMI160_USER_OFFSET_5_ADDR				0X76
#define BMI160_USER_OFFSET_6_ADDR				0X77
#define BMI160_USER_STEP_COUNT_0_ADDR				0X78
#define BMI160_USER_STEP_COUNT_1_ADDR				0X79
#define BMI160_USER_STEP_CONFIG_0_ADDR			0X7A
#define BMI160_USER_STEP_CONFIG_1_ADDR			0X7B
/*------------------------------------------------------------------------
* End of Register Definitions of User
*------------------------------------------------------------------------*/
/*------------------------------------------------------------------------
* Start of Register Definitions of CMD
*------------------------------------------------------------------------*/
 #define BMI160_CMD_COMMANDS_ADDR				0X7E
 #define BMI160_CMD_EXT_MODE_ADDR				0X7F

 #define BMI160_COM_C_TRIM_FIVE_ADDR		0X85
/*------------------------------------------------------------------------
* End of Register Definitions of CMD
*------------------------------------------------------------------------*/

#define BMI160_SHIFT_1_POSITION                 1
#define BMI160_SHIFT_2_POSITION                 2
#define BMI160_SHIFT_3_POSITION                 3
#define BMI160_SHIFT_4_POSITION                 4
#define BMI160_SHIFT_5_POSITION                 5
#define BMI160_SHIFT_6_POSITION                 6
#define BMI160_SHIFT_7_POSITION                 7
#define BMI160_SHIFT_8_POSITION                 8
#define BMI160_SHIFT_12_POSITION                12
#define BMI160_SHIFT_16_POSITION                16

#define	SUCCESS						((u8)0)
#define	ERROR						((s8)-1)
/** bmi160 magnetometer data
* brief Structure containing magnetometer values for x,y and
* z-axis in s16
*/

#define BMI160_DELAY_SETTLING_TIME         5

struct bmi160_mag_t {
s16 x;
s16 y;
s16 z;
};

struct bmi160_mag_xyzr_t {
s16 x;
s16 y;
s16 z;
u16 r;
};

/** bmi160 gyro data
* brief Structure containing gyro values for x,y and
* z-axis in s16
*/
struct bmi160_gyro_t {
s16 x,
y,
z;
};

/** bmi160 accelerometer data
* brief Structure containing accelerometer values for x,y and
* z-axis in s16
*/
struct bmi160_accel_t {
s16 x,
y,
z;
};
/* used to read the mag compensated values
output as s32*/
struct bmi160_mag_xyz_s32_t {
s32 x;
s32 y;
s32 z;
};
/* used to read the akm compensated values*/
struct bmi160_bst_akm_xyz_t {
s16 x;
s16 y;
s16 z;
};
/* struct used for read the value
of mag trim values*/
struct trim_data_t {
s8 dig_x1;
s8 dig_y1;

s8 dig_x2;
s8 dig_y2;

u16 dig_z1;
s16 dig_z2;
s16 dig_z3;
s16 dig_z4;

u8 dig_xy1;
s8 dig_xy2;

u16 dig_xyz1;
};
/* bmi160 structure
* This structure holds all relevant information about bmi160
*/
struct bmi160_t {
u8 chip_id;
u8 dev_addr;
BMI160_WR_FUNC_PTR;
BMI160_RD_FUNC_PTR;
BMI160_BRD_FUNC_PTR;
void (*delay_msec)(BMI160_MDELAY_DATA_TYPE);
};

/* Structure for reading AKM compensating data*/
struct bst_akm_sensitivity_data_t {
s8 asax;
s8 asay;
s8 asaz;
};

/* USER DATA REGISTERS DEFINITION START */
/* Chip ID Description - Reg Addr --> 0x00, Bit --> 0...7 */
#define BMI160_USER_CHIP_ID__POS             0
#define BMI160_USER_CHIP_ID__MSK            0xFF
#define BMI160_USER_CHIP_ID__LEN             8
#define BMI160_USER_CHIP_ID__REG             BMI160_USER_CHIP_ID_ADDR

/* Error Description - Reg Addr --> 0x02, Bit --> 0 */
#define BMI160_USER_ERR_STAT__POS               0
#define BMI160_USER_ERR_STAT__LEN               8
#define BMI160_USER_ERR_STAT__MSK               0xFF
#define BMI160_USER_ERR_STAT__REG               BMI160_USER_ERROR_ADDR

#define BMI160_USER_FATAL_ERR__POS               0
#define BMI160_USER_FATAL_ERR__LEN               1
#define BMI160_USER_FATAL_ERR__MSK               0x01
#define BMI160_USER_FATAL_ERR__REG               BMI160_USER_ERROR_ADDR

/* Error Description - Reg Addr --> 0x02, Bit --> 1...4 */
#define BMI160_USER_ERR_CODE__POS               1
#define BMI160_USER_ERR_CODE__LEN               4
#define BMI160_USER_ERR_CODE__MSK               0x1E
#define BMI160_USER_ERR_CODE__REG               BMI160_USER_ERROR_ADDR

/* Error Description - Reg Addr --> 0x02, Bit --> 5 */
#define BMI160_USER_I2C_FAIL_ERR__POS               5
#define BMI160_USER_I2C_FAIL_ERR__LEN               1
#define BMI160_USER_I2C_FAIL_ERR__MSK               0x20
#define BMI160_USER_I2C_FAIL_ERR__REG               BMI160_USER_ERROR_ADDR

/* Error Description - Reg Addr --> 0x02, Bit --> 6 */
#define BMI160_USER_DROP_CMD_ERR__POS              6
#define BMI160_USER_DROP_CMD_ERR__LEN              1
#define BMI160_USER_DROP_CMD_ERR__MSK              0x40
#define BMI160_USER_DROP_CMD_ERR__REG              BMI160_USER_ERROR_ADDR

/* Error Description - Reg Addr --> 0x02, Bit --> 7 */
#define BMI160_USER_MAG_DADA_RDY_ERR__POS               7
#define BMI160_USER_MAG_DADA_RDY_ERR__LEN               1
#define BMI160_USER_MAG_DADA_RDY_ERR__MSK               0x80
#define BMI160_USER_MAG_DADA_RDY_ERR__REG               BMI160_USER_ERROR_ADDR

/* PMU_Status Description of MAG - Reg Addr --> 0x03, Bit --> 1..0 */
#define BMI160_USER_MAG_POWER_MODE_STAT__POS		0
#define BMI160_USER_MAG_POWER_MODE_STAT__LEN		2
#define BMI160_USER_MAG_POWER_MODE_STAT__MSK		0x03
#define BMI160_USER_MAG_POWER_MODE_STAT__REG		\
BMI160_USER_PMU_STAT_ADDR

/* PMU_Status Description of GYRO - Reg Addr --> 0x03, Bit --> 3...2 */
#define BMI160_USER_GYRO_POWER_MODE_STAT__POS               2
#define BMI160_USER_GYRO_POWER_MODE_STAT__LEN               2
#define BMI160_USER_GYRO_POWER_MODE_STAT__MSK               0x0C
#define BMI160_USER_GYRO_POWER_MODE_STAT__REG		      \
BMI160_USER_PMU_STAT_ADDR

/* PMU_Status Description of ACCEL - Reg Addr --> 0x03, Bit --> 5...4 */
#define BMI160_USER_ACCEL_POWER_MODE_STAT__POS               4
#define BMI160_USER_ACCEL_POWER_MODE_STAT__LEN               2
#define BMI160_USER_ACCEL_POWER_MODE_STAT__MSK               0x30
#define BMI160_USER_ACCEL_POWER_MODE_STAT__REG		    \
BMI160_USER_PMU_STAT_ADDR

/* Mag_X(LSB) Description - Reg Addr --> 0x04, Bit --> 0...7 */
#define BMI160_USER_DATA_0_MAG_X_LSB__POS           0
#define BMI160_USER_DATA_0_MAG_X_LSB__LEN           8
#define BMI160_USER_DATA_0_MAG_X_LSB__MSK          0xFF
#define BMI160_USER_DATA_0_MAG_X_LSB__REG           BMI160_USER_DATA_0_ADDR

/* Mag_X(LSB) Description - Reg Addr --> 0x04, Bit --> 3...7 */
#define BMI160_USER_DATA_MAG_X_LSB__POS           3
#define BMI160_USER_DATA_MAG_X_LSB__LEN           5
#define BMI160_USER_DATA_MAG_X_LSB__MSK          0xF8
#define BMI160_USER_DATA_MAG_X_LSB__REG          BMI160_USER_DATA_0_ADDR

/* Mag_X(MSB) Description - Reg Addr --> 0x05, Bit --> 0...7 */
#define BMI160_USER_DATA_1_MAG_X_MSB__POS           0
#define BMI160_USER_DATA_1_MAG_X_MSB__LEN           8
#define BMI160_USER_DATA_1_MAG_X_MSB__MSK          0xFF
#define BMI160_USER_DATA_1_MAG_X_MSB__REG          BMI160_USER_DATA_1_ADDR

/* Mag_Y(LSB) Description - Reg Addr --> 0x06, Bit --> 0...7 */
#define BMI160_USER_DATA_2_MAG_Y_LSB__POS           0
#define BMI160_USER_DATA_2_MAG_Y_LSB__LEN           8
#define BMI160_USER_DATA_2_MAG_Y_LSB__MSK          0xFF
#define BMI160_USER_DATA_2_MAG_Y_LSB__REG          BMI160_USER_DATA_2_ADDR

/* Mag_Y(LSB) Description - Reg Addr --> 0x06, Bit --> 3...7 */
#define BMI160_USER_DATA_MAG_Y_LSB__POS           3
#define BMI160_USER_DATA_MAG_Y_LSB__LEN           5
#define BMI160_USER_DATA_MAG_Y_LSB__MSK          0xF8
#define BMI160_USER_DATA_MAG_Y_LSB__REG          BMI160_USER_DATA_2_ADDR

/* Mag_Y(MSB) Description - Reg Addr --> 0x07, Bit --> 0...7 */
#define BMI160_USER_DATA_3_MAG_Y_MSB__POS           0
#define BMI160_USER_DATA_3_MAG_Y_MSB__LEN           8
#define BMI160_USER_DATA_3_MAG_Y_MSB__MSK          0xFF
#define BMI160_USER_DATA_3_MAG_Y_MSB__REG          BMI160_USER_DATA_3_ADDR

/* Mag_Z(LSB) Description - Reg Addr --> 0x08, Bit --> 0...7 */
#define BMI160_USER_DATA_4_MAG_Z_LSB__POS           0
#define BMI160_USER_DATA_4_MAG_Z_LSB__LEN           8
#define BMI160_USER_DATA_4_MAG_Z_LSB__MSK          0xFF
#define BMI160_USER_DATA_4_MAG_Z_LSB__REG          BMI160_USER_DATA_4_ADDR

/* Mag_X(LSB) Description - Reg Addr --> 0x08, Bit --> 3...7 */
#define BMI160_USER_DATA_MAG_Z_LSB__POS           1
#define BMI160_USER_DATA_MAG_Z_LSB__LEN           7
#define BMI160_USER_DATA_MAG_Z_LSB__MSK          0xFE
#define BMI160_USER_DATA_MAG_Z_LSB__REG          BMI160_USER_DATA_4_ADDR

/* Mag_Z(MSB) Description - Reg Addr --> 0x09, Bit --> 0...7 */
#define BMI160_USER_DATA_5_MAG_Z_MSB__POS           0
#define BMI160_USER_DATA_5_MAG_Z_MSB__LEN           8
#define BMI160_USER_DATA_5_MAG_Z_MSB__MSK          0xFF
#define BMI160_USER_DATA_5_MAG_Z_MSB__REG          BMI160_USER_DATA_5_ADDR

/* RHALL(LSB) Description - Reg Addr --> 0x0A, Bit --> 0...7 */
#define BMI160_USER_DATA_6_RHALL_LSB__POS           0
#define BMI160_USER_DATA_6_RHALL_LSB__LEN           8
#define BMI160_USER_DATA_6_RHALL_LSB__MSK          0xFF
#define BMI160_USER_DATA_6_RHALL_LSB__REG          BMI160_USER_DATA_6_ADDR

/* Mag_R(LSB) Description - Reg Addr --> 0x0A, Bit --> 3...7 */
#define BMI160_USER_DATA_MAG_R_LSB__POS           2
#define BMI160_USER_DATA_MAG_R_LSB__LEN           6
#define BMI160_USER_DATA_MAG_R_LSB__MSK          0xFC
#define BMI160_USER_DATA_MAG_R_LSB__REG          BMI160_USER_DATA_6_ADDR

/* RHALL(MSB) Description - Reg Addr --> 0x0B, Bit --> 0...7 */
#define BMI160_USER_DATA_7_RHALL_MSB__POS           0
#define BMI160_USER_DATA_7_RHALL_MSB__LEN           8
#define BMI160_USER_DATA_7_RHALL_MSB__MSK          0xFF
#define BMI160_USER_DATA_7_RHALL_MSB__REG          BMI160_USER_DATA_7_ADDR

/* GYR_X (LSB) Description - Reg Addr --> 0x0C, Bit --> 0...7 */
#define BMI160_USER_DATA_8_GYRO_X_LSB__POS           0
#define BMI160_USER_DATA_8_GYRO_X_LSB__LEN           8
#define BMI160_USER_DATA_8_GYRO_X_LSB__MSK          0xFF
#define BMI160_USER_DATA_8_GYRO_X_LSB__REG          BMI160_USER_DATA_8_ADDR

/* GYR_X (MSB) Description - Reg Addr --> 0x0D, Bit --> 0...7 */
#define BMI160_USER_DATA_9_GYRO_X_MSB__POS           0
#define BMI160_USER_DATA_9_GYRO_X_MSB__LEN           8
#define BMI160_USER_DATA_9_GYRO_X_MSB__MSK          0xFF
#define BMI160_USER_DATA_9_GYRO_X_MSB__REG          BMI160_USER_DATA_9_ADDR

/* GYR_Y (LSB) Description - Reg Addr --> 0x0E, Bit --> 0...7 */
#define BMI160_USER_DATA_10_GYRO_Y_LSB__POS           0
#define BMI160_USER_DATA_10_GYRO_Y_LSB__LEN           8
#define BMI160_USER_DATA_10_GYRO_Y_LSB__MSK          0xFF
#define BMI160_USER_DATA_10_GYRO_Y_LSB__REG          BMI160_USER_DATA_10_ADDR

/* GYR_Y (MSB) Description - Reg Addr --> 0x0F, Bit --> 0...7 */
#define BMI160_USER_DATA_11_GYRO_Y_MSB__POS           0
#define BMI160_USER_DATA_11_GYRO_Y_MSB__LEN           8
#define BMI160_USER_DATA_11_GYRO_Y_MSB__MSK          0xFF
#define BMI160_USER_DATA_11_GYRO_Y_MSB__REG          BMI160_USER_DATA_11_ADDR

/* GYR_Z (LSB) Description - Reg Addr --> 0x10, Bit --> 0...7 */
#define BMI160_USER_DATA_12_GYRO_Z_LSB__POS           0
#define BMI160_USER_DATA_12_GYRO_Z_LSB__LEN           8
#define BMI160_USER_DATA_12_GYRO_Z_LSB__MSK          0xFF
#define BMI160_USER_DATA_12_GYRO_Z_LSB__REG          BMI160_USER_DATA_12_ADDR

/* GYR_Z (MSB) Description - Reg Addr --> 0x11, Bit --> 0...7 */
#define BMI160_USER_DATA_13_GYRO_Z_MSB__POS           0
#define BMI160_USER_DATA_13_GYRO_Z_MSB__LEN           8
#define BMI160_USER_DATA_13_GYRO_Z_MSB__MSK          0xFF
#define BMI160_USER_DATA_13_GYRO_Z_MSB__REG          BMI160_USER_DATA_13_ADDR

/* ACC_X (LSB) Description - Reg Addr --> 0x12, Bit --> 0...7 */
#define BMI160_USER_DATA_14_ACCEL_X_LSB__POS           0
#define BMI160_USER_DATA_14_ACCEL_X_LSB__LEN           8
#define BMI160_USER_DATA_14_ACCEL_X_LSB__MSK          0xFF
#define BMI160_USER_DATA_14_ACCEL_X_LSB__REG          BMI160_USER_DATA_14_ADDR

/* ACC_X (MSB) Description - Reg Addr --> 0x13, Bit --> 0...7 */
#define BMI160_USER_DATA_15_ACCEL_X_MSB__POS           0
#define BMI160_USER_DATA_15_ACCEL_X_MSB__LEN           8
#define BMI160_USER_DATA_15_ACCEL_X_MSB__MSK          0xFF
#define BMI160_USER_DATA_15_ACCEL_X_MSB__REG          BMI160_USER_DATA_15_ADDR

/* ACC_Y (LSB) Description - Reg Addr --> 0x14, Bit --> 0...7 */
#define BMI160_USER_DATA_16_ACCEL_Y_LSB__POS           0
#define BMI160_USER_DATA_16_ACCEL_Y_LSB__LEN           8
#define BMI160_USER_DATA_16_ACCEL_Y_LSB__MSK          0xFF
#define BMI160_USER_DATA_16_ACCEL_Y_LSB__REG          BMI160_USER_DATA_16_ADDR

/* ACC_Y (MSB) Description - Reg Addr --> 0x15, Bit --> 0...7 */
#define BMI160_USER_DATA_17_ACCEL_Y_MSB__POS           0
#define BMI160_USER_DATA_17_ACCEL_Y_MSB__LEN           8
#define BMI160_USER_DATA_17_ACCEL_Y_MSB__MSK          0xFF
#define BMI160_USER_DATA_17_ACCEL_Y_MSB__REG          BMI160_USER_DATA_17_ADDR

/* ACC_Z (LSB) Description - Reg Addr --> 0x16, Bit --> 0...7 */
#define BMI160_USER_DATA_18_ACCEL_Z_LSB__POS           0
#define BMI160_USER_DATA_18_ACCEL_Z_LSB__LEN           8
#define BMI160_USER_DATA_18_ACCEL_Z_LSB__MSK          0xFF
#define BMI160_USER_DATA_18_ACCEL_Z_LSB__REG          BMI160_USER_DATA_18_ADDR

/* ACC_Z (MSB) Description - Reg Addr --> 0x17, Bit --> 0...7 */
#define BMI160_USER_DATA_19_ACCEL_Z_MSB__POS           0
#define BMI160_USER_DATA_19_ACCEL_Z_MSB__LEN           8
#define BMI160_USER_DATA_19_ACCEL_Z_MSB__MSK          0xFF
#define BMI160_USER_DATA_19_ACCEL_Z_MSB__REG          BMI160_USER_DATA_19_ADDR

/* SENSORTIME_0 (LSB) Description - Reg Addr --> 0x18, Bit --> 0...7 */
#define BMI160_USER_SENSORTIME_0_SENSOR_TIME_LSB__POS           0
#define BMI160_USER_SENSORTIME_0_SENSOR_TIME_LSB__LEN           8
#define BMI160_USER_SENSORTIME_0_SENSOR_TIME_LSB__MSK          0xFF
#define BMI160_USER_SENSORTIME_0_SENSOR_TIME_LSB__REG          \
		BMI160_USER_SENSORTIME_0_ADDR

/* SENSORTIME_1 (MSB) Description - Reg Addr --> 0x19, Bit --> 0...7 */
#define BMI160_USER_SENSORTIME_1_SENSOR_TIME_MSB__POS           0
#define BMI160_USER_SENSORTIME_1_SENSOR_TIME_MSB__LEN           8
#define BMI160_USER_SENSORTIME_1_SENSOR_TIME_MSB__MSK          0xFF
#define BMI160_USER_SENSORTIME_1_SENSOR_TIME_MSB__REG          \
		BMI160_USER_SENSORTIME_1_ADDR

/* SENSORTIME_2 (MSB) Description - Reg Addr --> 0x1A, Bit --> 0...7 */
#define BMI160_USER_SENSORTIME_2_SENSOR_TIME_MSB__POS           0
#define BMI160_USER_SENSORTIME_2_SENSOR_TIME_MSB__LEN           8
#define BMI160_USER_SENSORTIME_2_SENSOR_TIME_MSB__MSK          0xFF
#define BMI160_USER_SENSORTIME_2_SENSOR_TIME_MSB__REG          \
		BMI160_USER_SENSORTIME_2_ADDR

/* Status Description - Reg Addr --> 0x1B, Bit --> 1 */
#define BMI160_USER_STAT_GYRO_SELFTEST_OK__POS          1
#define BMI160_USER_STAT_GYRO_SELFTEST_OK__LEN          1
#define BMI160_USER_STAT_GYRO_SELFTEST_OK__MSK          0x02
#define BMI160_USER_STAT_GYRO_SELFTEST_OK__REG         \
		BMI160_USER_STAT_ADDR

/* Status Description - Reg Addr --> 0x1B, Bit --> 2 */
#define BMI160_USER_STAT_MAG_MANUAL_OPERATION__POS          2
#define BMI160_USER_STAT_MAG_MANUAL_OPERATION__LEN          1
#define BMI160_USER_STAT_MAG_MANUAL_OPERATION__MSK          0x04
#define BMI160_USER_STAT_MAG_MANUAL_OPERATION__REG          \
		BMI160_USER_STAT_ADDR

/* Status Description - Reg Addr --> 0x1B, Bit --> 3 */
#define BMI160_USER_STAT_FOC_RDY__POS          3
#define BMI160_USER_STAT_FOC_RDY__LEN          1
#define BMI160_USER_STAT_FOC_RDY__MSK          0x08
#define BMI160_USER_STAT_FOC_RDY__REG          BMI160_USER_STAT_ADDR

/* Status Description - Reg Addr --> 0x1B, Bit --> 4 */
#define BMI160_USER_STAT_NVM_RDY__POS           4
#define BMI160_USER_STAT_NVM_RDY__LEN           1
#define BMI160_USER_STAT_NVM_RDY__MSK           0x10
#define BMI160_USER_STAT_NVM_RDY__REG           BMI160_USER_STAT_ADDR

/* Status Description - Reg Addr --> 0x1B, Bit --> 5 */
#define BMI160_USER_STAT_DATA_RDY_MAG__POS           5
#define BMI160_USER_STAT_DATA_RDY_MAG__LEN           1
#define BMI160_USER_STAT_DATA_RDY_MAG__MSK           0x20
#define BMI160_USER_STAT_DATA_RDY_MAG__REG           BMI160_USER_STAT_ADDR

/* Status Description - Reg Addr --> 0x1B, Bit --> 6 */
#define BMI160_USER_STAT_DATA_RDY_GYRO__POS           6
#define BMI160_USER_STAT_DATA_RDY_GYRO__LEN           1
#define BMI160_USER_STAT_DATA_RDY_GYRO__MSK           0x40
#define BMI160_USER_STAT_DATA_RDY_GYRO__REG           BMI160_USER_STAT_ADDR

/* Status Description - Reg Addr --> 0x1B, Bit --> 7 */
#define BMI160_USER_STAT_DATA_RDY_ACCEL__POS           7
#define BMI160_USER_STAT_DATA_RDY_ACCEL__LEN           1
#define BMI160_USER_STAT_DATA_RDY_ACCEL__MSK           0x80
#define BMI160_USER_STAT_DATA_RDY_ACCEL__REG           BMI160_USER_STAT_ADDR

/* Int_Status_0 Description - Reg Addr --> 0x1C, Bit --> 0 */
#define BMI160_USER_INTR_STAT_0_STEP_INTR__POS           0
#define BMI160_USER_INTR_STAT_0_STEP_INTR__LEN           1
#define BMI160_USER_INTR_STAT_0_STEP_INTR__MSK          0x01
#define BMI160_USER_INTR_STAT_0_STEP_INTR__REG          \
		BMI160_USER_INTR_STAT_0_ADDR

/* Int_Status_0 Description - Reg Addr --> 0x1C, Bit --> 1 */
#define BMI160_USER_INTR_STAT_0_SIGNIFICANT_INTR__POS		1
#define BMI160_USER_INTR_STAT_0_SIGNIFICANT_INTR__LEN		1
#define BMI160_USER_INTR_STAT_0_SIGNIFICANT_INTR__MSK		0x02
#define BMI160_USER_INTR_STAT_0_SIGNIFICANT_INTR__REG       \
		BMI160_USER_INTR_STAT_0_ADDR

/* Int_Status_0 Description - Reg Addr --> 0x1C, Bit --> 2 */
#define BMI160_USER_INTR_STAT_0_ANY_MOTION__POS           2
#define BMI160_USER_INTR_STAT_0_ANY_MOTION__LEN           1
#define BMI160_USER_INTR_STAT_0_ANY_MOTION__MSK          0x04
#define BMI160_USER_INTR_STAT_0_ANY_MOTION__REG          \
		BMI160_USER_INTR_STAT_0_ADDR

/* Int_Status_0 Description - Reg Addr --> 0x1C, Bit --> 3 */
#define BMI160_USER_INTR_STAT_0_PMU_TRIGGER__POS           3
#define BMI160_USER_INTR_STAT_0_PMU_TRIGGER__LEN           1
#define BMI160_USER_INTR_STAT_0_PMU_TRIGGER__MSK          0x08
#define BMI160_USER_INTR_STAT_0_PMU_TRIGGER__REG          \
		BMI160_USER_INTR_STAT_0_ADDR

/* Int_Status_0 Description - Reg Addr --> 0x1C, Bit --> 4 */
#define BMI160_USER_INTR_STAT_0_DOUBLE_TAP_INTR__POS           4
#define BMI160_USER_INTR_STAT_0_DOUBLE_TAP_INTR__LEN           1
#define BMI160_USER_INTR_STAT_0_DOUBLE_TAP_INTR__MSK          0x10
#define BMI160_USER_INTR_STAT_0_DOUBLE_TAP_INTR__REG          \
		BMI160_USER_INTR_STAT_0_ADDR

/* Int_Status_0 Description - Reg Addr --> 0x1C, Bit --> 5 */
#define BMI160_USER_INTR_STAT_0_SINGLE_TAP_INTR__POS           5
#define BMI160_USER_INTR_STAT_0_SINGLE_TAP_INTR__LEN           1
#define BMI160_USER_INTR_STAT_0_SINGLE_TAP_INTR__MSK          0x20
#define BMI160_USER_INTR_STAT_0_SINGLE_TAP_INTR__REG          \
		BMI160_USER_INTR_STAT_0_ADDR

/* Int_Status_0 Description - Reg Addr --> 0x1C, Bit --> 6 */
#define BMI160_USER_INTR_STAT_0_ORIENT__POS           6
#define BMI160_USER_INTR_STAT_0_ORIENT__LEN           1
#define BMI160_USER_INTR_STAT_0_ORIENT__MSK          0x40
#define BMI160_USER_INTR_STAT_0_ORIENT__REG          \
		BMI160_USER_INTR_STAT_0_ADDR

/* Int_Status_0 Description - Reg Addr --> 0x1C, Bit --> 7 */
#define BMI160_USER_INTR_STAT_0_FLAT__POS           7
#define BMI160_USER_INTR_STAT_0_FLAT__LEN           1
#define BMI160_USER_INTR_STAT_0_FLAT__MSK          0x80
#define BMI160_USER_INTR_STAT_0_FLAT__REG          \
		BMI160_USER_INTR_STAT_0_ADDR

/* Int_Status_1 Description - Reg Addr --> 0x1D, Bit --> 2 */
#define BMI160_USER_INTR_STAT_1_HIGH_G_INTR__POS               2
#define BMI160_USER_INTR_STAT_1_HIGH_G_INTR__LEN               1
#define BMI160_USER_INTR_STAT_1_HIGH_G_INTR__MSK              0x04
#define BMI160_USER_INTR_STAT_1_HIGH_G_INTR__REG              \
		BMI160_USER_INTR_STAT_1_ADDR

/* Int_Status_1 Description - Reg Addr --> 0x1D, Bit --> 3 */
#define BMI160_USER_INTR_STAT_1_LOW_G_INTR__POS               3
#define BMI160_USER_INTR_STAT_1_LOW_G_INTR__LEN               1
#define BMI160_USER_INTR_STAT_1_LOW_G_INTR__MSK              0x08
#define BMI160_USER_INTR_STAT_1_LOW_G_INTR__REG              \
		BMI160_USER_INTR_STAT_1_ADDR

/* Int_Status_1 Description - Reg Addr --> 0x1D, Bit --> 4 */
#define BMI160_USER_INTR_STAT_1_DATA_RDY_INTR__POS               4
#define BMI160_USER_INTR_STAT_1_DATA_RDY_INTR__LEN               1
#define BMI160_USER_INTR_STAT_1_DATA_RDY_INTR__MSK               0x10
#define BMI160_USER_INTR_STAT_1_DATA_RDY_INTR__REG               \
		BMI160_USER_INTR_STAT_1_ADDR

/* Int_Status_1 Description - Reg Addr --> 0x1D, Bit --> 5 */
#define BMI160_USER_INTR_STAT_1_FIFO_FULL_INTR__POS               5
#define BMI160_USER_INTR_STAT_1_FIFO_FULL_INTR__LEN               1
#define BMI160_USER_INTR_STAT_1_FIFO_FULL_INTR__MSK               0x20
#define BMI160_USER_INTR_STAT_1_FIFO_FULL_INTR__REG               \
		BMI160_USER_INTR_STAT_1_ADDR

/* Int_Status_1 Description - Reg Addr --> 0x1D, Bit --> 6 */
#define BMI160_USER_INTR_STAT_1_FIFO_WM_INTR__POS               6
#define BMI160_USER_INTR_STAT_1_FIFO_WM_INTR__LEN               1
#define BMI160_USER_INTR_STAT_1_FIFO_WM_INTR__MSK               0x40
#define BMI160_USER_INTR_STAT_1_FIFO_WM_INTR__REG               \
		BMI160_USER_INTR_STAT_1_ADDR

/* Int_Status_1 Description - Reg Addr --> 0x1D, Bit --> 7 */
#define BMI160_USER_INTR_STAT_1_NOMOTION_INTR__POS               7
#define BMI160_USER_INTR_STAT_1_NOMOTION_INTR__LEN               1
#define BMI160_USER_INTR_STAT_1_NOMOTION_INTR__MSK               0x80
#define BMI160_USER_INTR_STAT_1_NOMOTION_INTR__REG               \
		BMI160_USER_INTR_STAT_1_ADDR

/* Int_Status_2 Description - Reg Addr --> 0x1E, Bit --> 0 */
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_X__POS               0
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_X__LEN               1
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_X__MSK               0x01
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_X__REG               \
		BMI160_USER_INTR_STAT_2_ADDR

/* Int_Status_2 Description - Reg Addr --> 0x1E, Bit --> 1 */
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Y__POS               1
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Y__LEN               1
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Y__MSK               0x02
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Y__REG               \
		BMI160_USER_INTR_STAT_2_ADDR

/* Int_Status_2 Description - Reg Addr --> 0x1E, Bit --> 2 */
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Z__POS               2
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Z__LEN               1
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Z__MSK               0x04
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Z__REG               \
		BMI160_USER_INTR_STAT_2_ADDR

/* Int_Status_2 Description - Reg Addr --> 0x1E, Bit --> 3 */
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_SIGN__POS               3
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_SIGN__LEN               1
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_SIGN__MSK               0x08
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_SIGN__REG               \
		BMI160_USER_INTR_STAT_2_ADDR

/* Int_Status_2 Description - Reg Addr --> 0x1E, Bit --> 4 */
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_X__POS               4
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_X__LEN               1
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_X__MSK               0x10
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_X__REG               \
		BMI160_USER_INTR_STAT_2_ADDR

/* Int_Status_2 Description - Reg Addr --> 0x1E, Bit --> 5 */
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_Y__POS               5
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_Y__LEN               1
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_Y__MSK               0x20
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_Y__REG               \
		BMI160_USER_INTR_STAT_2_ADDR

/* Int_Status_2 Description - Reg Addr --> 0x1E, Bit --> 6 */
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_Z__POS               6
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_Z__LEN               1
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_Z__MSK               0x40
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_Z__REG               \
		BMI160_USER_INTR_STAT_2_ADDR

/* Int_Status_2 Description - Reg Addr --> 0x1E, Bit --> 7 */
#define BMI160_USER_INTR_STAT_2_TAP_SIGN__POS               7
#define BMI160_USER_INTR_STAT_2_TAP_SIGN__LEN               1
#define BMI160_USER_INTR_STAT_2_TAP_SIGN__MSK               0x80
#define BMI160_USER_INTR_STAT_2_TAP_SIGN__REG               \
		BMI160_USER_INTR_STAT_2_ADDR

/* Int_Status_2 Description - Reg Addr --> 0x1E, Bit --> 0...7 */
#define BMI160_USER_INTR_STAT_2__POS               0
#define BMI160_USER_INTR_STAT_2__LEN               8
#define BMI160_USER_INTR_STAT_2__MSK               0xFF
#define BMI160_USER_INTR_STAT_2__REG               \
		BMI160_USER_INTR_STAT_2_ADDR

/* Int_Status_3 Description - Reg Addr --> 0x1F, Bit --> 0 */
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_X__POS               0
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_X__LEN               1
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_X__MSK               0x01
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_X__REG               \
		BMI160_USER_INTR_STAT_3_ADDR

/* Int_Status_3 Description - Reg Addr --> 0x1E, Bit --> 1 */
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_Y__POS               1
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_Y__LEN               1
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_Y__MSK               0x02
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_Y__REG               \
		BMI160_USER_INTR_STAT_3_ADDR

/* Int_Status_3 Description - Reg Addr --> 0x1F, Bit --> 2 */
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_Z__POS               2
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_Z__LEN               1
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_Z__MSK               0x04
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_Z__REG               \
		BMI160_USER_INTR_STAT_3_ADDR

/* Int_Status_3 Description - Reg Addr --> 0x1F, Bit --> 3 */
#define BMI160_USER_INTR_STAT_3_HIGH_G_SIGN__POS               3
#define BMI160_USER_INTR_STAT_3_HIGH_G_SIGN__LEN               1
#define BMI160_USER_INTR_STAT_3_HIGH_G_SIGN__MSK               0x08
#define BMI160_USER_INTR_STAT_3_HIGH_G_SIGN__REG               \
		BMI160_USER_INTR_STAT_3_ADDR

/* Int_Status_3 Description - Reg Addr --> 0x1F, Bit --> 4...5 */
#define BMI160_USER_INTR_STAT_3_ORIENT_XY__POS               4
#define BMI160_USER_INTR_STAT_3_ORIENT_XY__LEN               2
#define BMI160_USER_INTR_STAT_3_ORIENT_XY__MSK               0x30
#define BMI160_USER_INTR_STAT_3_ORIENT_XY__REG               \
		BMI160_USER_INTR_STAT_3_ADDR

/* Int_Status_3 Description - Reg Addr --> 0x1F, Bit --> 6 */
#define BMI160_USER_INTR_STAT_3_ORIENT_Z__POS               6
#define BMI160_USER_INTR_STAT_3_ORIENT_Z__LEN               1
#define BMI160_USER_INTR_STAT_3_ORIENT_Z__MSK               0x40
#define BMI160_USER_INTR_STAT_3_ORIENT_Z__REG               \
		BMI160_USER_INTR_STAT_3_ADDR

/* Int_Status_3 Description - Reg Addr --> 0x1F, Bit --> 7 */
#define BMI160_USER_INTR_STAT_3_FLAT__POS               7
#define BMI160_USER_INTR_STAT_3_FLAT__LEN               1
#define BMI160_USER_INTR_STAT_3_FLAT__MSK               0x80
#define BMI160_USER_INTR_STAT_3_FLAT__REG               \
		BMI160_USER_INTR_STAT_3_ADDR

/* Int_Status_3 Description - Reg Addr --> 0x1F, Bit --> 0...7 */
#define BMI160_USER_INTR_STAT_3__POS               0
#define BMI160_USER_INTR_STAT_3__LEN               8
#define BMI160_USER_INTR_STAT_3__MSK               0xFF
#define BMI160_USER_INTR_STAT_3__REG               \
		BMI160_USER_INTR_STAT_3_ADDR

/* Temperature Description - LSB Reg Addr --> 0x20, Bit --> 0...7 */
#define BMI160_USER_TEMP_LSB_VALUE__POS               0
#define BMI160_USER_TEMP_LSB_VALUE__LEN               8
#define BMI160_USER_TEMP_LSB_VALUE__MSK               0xFF
#define BMI160_USER_TEMP_LSB_VALUE__REG               \
		BMI160_USER_TEMPERATURE_0_ADDR

/* Temperature Description - LSB Reg Addr --> 0x21, Bit --> 0...7 */
#define BMI160_USER_TEMP_MSB_VALUE__POS               0
#define BMI160_USER_TEMP_MSB_VALUE__LEN               8
#define BMI160_USER_TEMP_MSB_VALUE__MSK               0xFF
#define BMI160_USER_TEMP_MSB_VALUE__REG               \
		BMI160_USER_TEMPERATURE_1_ADDR

/* Fifo_Length0 Description - Reg Addr --> 0x22, Bit --> 0...7 */
#define BMI160_USER_FIFO_BYTE_COUNTER_LSB__POS           0
#define BMI160_USER_FIFO_BYTE_COUNTER_LSB__LEN           8
#define BMI160_USER_FIFO_BYTE_COUNTER_LSB__MSK          0xFF
#define BMI160_USER_FIFO_BYTE_COUNTER_LSB__REG          \
		BMI160_USER_FIFO_LENGTH_0_ADDR

/*Fifo_Length1 Description - Reg Addr --> 0x23, Bit --> 0...2 */
#define BMI160_USER_FIFO_BYTE_COUNTER_MSB__POS           0
#define BMI160_USER_FIFO_BYTE_COUNTER_MSB__LEN           3
#define BMI160_USER_FIFO_BYTE_COUNTER_MSB__MSK          0x07
#define BMI160_USER_FIFO_BYTE_COUNTER_MSB__REG          \
		BMI160_USER_FIFO_LENGTH_1_ADDR


/* Fifo_Data Description - Reg Addr --> 0x24, Bit --> 0...7 */
#define BMI160_USER_FIFO_DATA__POS           0
#define BMI160_USER_FIFO_DATA__LEN           8
#define BMI160_USER_FIFO_DATA__MSK          0xFF
#define BMI160_USER_FIFO_DATA__REG          BMI160_USER_FIFO_DATA_ADDR


/* Acc_Conf Description - Reg Addr --> 0x40, Bit --> 0...3 */
#define BMI160_USER_ACCEL_CONFIG_OUTPUT_DATA_RATE__POS               0
#define BMI160_USER_ACCEL_CONFIG_OUTPUT_DATA_RATE__LEN               4
#define BMI160_USER_ACCEL_CONFIG_OUTPUT_DATA_RATE__MSK               0x0F
#define BMI160_USER_ACCEL_CONFIG_OUTPUT_DATA_RATE__REG		       \
BMI160_USER_ACCEL_CONFIG_ADDR

/* Acc_Conf Description - Reg Addr --> 0x40, Bit --> 4...6 */
#define BMI160_USER_ACCEL_CONFIG_ACCEL_BW__POS               4
#define BMI160_USER_ACCEL_CONFIG_ACCEL_BW__LEN               3
#define BMI160_USER_ACCEL_CONFIG_ACCEL_BW__MSK               0x70
#define BMI160_USER_ACCEL_CONFIG_ACCEL_BW__REG	BMI160_USER_ACCEL_CONFIG_ADDR

/* Acc_Conf Description - Reg Addr --> 0x40, Bit --> 7 */
#define BMI160_USER_ACCEL_CONFIG_ACCEL_UNDER_SAMPLING__POS           7
#define BMI160_USER_ACCEL_CONFIG_ACCEL_UNDER_SAMPLING__LEN           1
#define BMI160_USER_ACCEL_CONFIG_ACCEL_UNDER_SAMPLING__MSK           0x80
#define BMI160_USER_ACCEL_CONFIG_ACCEL_UNDER_SAMPLING__REG	\
BMI160_USER_ACCEL_CONFIG_ADDR

/* Acc_Range Description - Reg Addr --> 0x41, Bit --> 0...3 */
#define BMI160_USER_ACCEL_RANGE__POS               0
#define BMI160_USER_ACCEL_RANGE__LEN               4
#define BMI160_USER_ACCEL_RANGE__MSK               0x0F
#define BMI160_USER_ACCEL_RANGE__REG               BMI160_USER_ACCEL_RANGE_ADDR

/* Gyro_Conf Description - Reg Addr --> 0x42, Bit --> 0...3 */
#define BMI160_USER_GYRO_CONFIG_OUTPUT_DATA_RATE__POS               0
#define BMI160_USER_GYRO_CONFIG_OUTPUT_DATA_RATE__LEN               4
#define BMI160_USER_GYRO_CONFIG_OUTPUT_DATA_RATE__MSK               0x0F
#define BMI160_USER_GYRO_CONFIG_OUTPUT_DATA_RATE__REG               \
BMI160_USER_GYRO_CONFIG_ADDR

/* Gyro_Conf Description - Reg Addr --> 0x42, Bit --> 4...5 */
#define BMI160_USER_GYRO_CONFIG_BW__POS               4
#define BMI160_USER_GYRO_CONFIG_BW__LEN               2
#define BMI160_USER_GYRO_CONFIG_BW__MSK               0x30
#define BMI160_USER_GYRO_CONFIG_BW__REG               \
BMI160_USER_GYRO_CONFIG_ADDR

/* Gyr_Range Description - Reg Addr --> 0x43, Bit --> 0...2 */
#define BMI160_USER_GYRO_RANGE__POS               0
#define BMI160_USER_GYRO_RANGE__LEN               3
#define BMI160_USER_GYRO_RANGE__MSK               0x07
#define BMI160_USER_GYRO_RANGE__REG               BMI160_USER_GYRO_RANGE_ADDR

/* Mag_Conf Description - Reg Addr --> 0x44, Bit --> 0...3 */
#define BMI160_USER_MAG_CONFIG_OUTPUT_DATA_RATE__POS               0
#define BMI160_USER_MAG_CONFIG_OUTPUT_DATA_RATE__LEN               4
#define BMI160_USER_MAG_CONFIG_OUTPUT_DATA_RATE__MSK               0x0F
#define BMI160_USER_MAG_CONFIG_OUTPUT_DATA_RATE__REG               \
BMI160_USER_MAG_CONFIG_ADDR


/* Fifo_Downs Description - Reg Addr --> 0x45, Bit --> 0...2 */
#define BMI160_USER_FIFO_DOWN_GYRO__POS               0
#define BMI160_USER_FIFO_DOWN_GYRO__LEN               3
#define BMI160_USER_FIFO_DOWN_GYRO__MSK               0x07
#define BMI160_USER_FIFO_DOWN_GYRO__REG	BMI160_USER_FIFO_DOWN_ADDR

/* Fifo_filt Description - Reg Addr --> 0x45, Bit --> 3 */
#define BMI160_USER_FIFO_FILTER_GYRO__POS               3
#define BMI160_USER_FIFO_FILTER_GYRO__LEN               1
#define BMI160_USER_FIFO_FILTER_GYRO__MSK               0x08
#define BMI160_USER_FIFO_FILTER_GYRO__REG	  BMI160_USER_FIFO_DOWN_ADDR

/* Fifo_Downs Description - Reg Addr --> 0x45, Bit --> 4...6 */
#define BMI160_USER_FIFO_DOWN_ACCEL__POS               4
#define BMI160_USER_FIFO_DOWN_ACCEL__LEN               3
#define BMI160_USER_FIFO_DOWN_ACCEL__MSK               0x70
#define BMI160_USER_FIFO_DOWN_ACCEL__REG	BMI160_USER_FIFO_DOWN_ADDR

/* Fifo_FILT Description - Reg Addr --> 0x45, Bit --> 7 */
#define BMI160_USER_FIFO_FILTER_ACCEL__POS               7
#define BMI160_USER_FIFO_FILTER_ACCEL__LEN               1
#define BMI160_USER_FIFO_FILTER_ACCEL__MSK               0x80
#define BMI160_USER_FIFO_FILTER_ACCEL__REG	BMI160_USER_FIFO_DOWN_ADDR

/* Fifo_Config_0 Description - Reg Addr --> 0x46, Bit --> 0...7 */
#define BMI160_USER_FIFO_WM__POS               0
#define BMI160_USER_FIFO_WM__LEN               8
#define BMI160_USER_FIFO_WM__MSK               0xFF
#define BMI160_USER_FIFO_WM__REG	BMI160_USER_FIFO_CONFIG_0_ADDR

/* Fifo_Config_1 Description - Reg Addr --> 0x47, Bit --> 0 */
#define BMI160_USER_FIFO_STOP_ON_FULL__POS		0
#define BMI160_USER_FIFO_STOP_ON_FULL__LEN		1
#define BMI160_USER_FIFO_STOP_ON_FULL__MSK		0x01
#define BMI160_USER_FIFO_STOP_ON_FULL__REG	BMI160_USER_FIFO_CONFIG_1_ADDR

/* Fifo_Config_1 Description - Reg Addr --> 0x47, Bit --> 1 */
#define BMI160_USER_FIFO_TIME_ENABLE__POS               1
#define BMI160_USER_FIFO_TIME_ENABLE__LEN               1
#define BMI160_USER_FIFO_TIME_ENABLE__MSK               0x02
#define BMI160_USER_FIFO_TIME_ENABLE__REG	BMI160_USER_FIFO_CONFIG_1_ADDR

/* Fifo_Config_1 Description - Reg Addr --> 0x47, Bit --> 2 */
#define BMI160_USER_FIFO_TAG_INTR2_ENABLE__POS               2
#define BMI160_USER_FIFO_TAG_INTR2_ENABLE__LEN               1
#define BMI160_USER_FIFO_TAG_INTR2_ENABLE__MSK               0x04
#define BMI160_USER_FIFO_TAG_INTR2_ENABLE__REG	BMI160_USER_FIFO_CONFIG_1_ADDR

/* Fifo_Config_1 Description - Reg Addr --> 0x47, Bit --> 3 */
#define BMI160_USER_FIFO_TAG_INTR1_ENABLE__POS               3
#define BMI160_USER_FIFO_TAG_INTR1_ENABLE__LEN               1
#define BMI160_USER_FIFO_TAG_INTR1_ENABLE__MSK               0x08
#define BMI160_USER_FIFO_TAG_INTR1_ENABLE__REG	BMI160_USER_FIFO_CONFIG_1_ADDR

/* Fifo_Config_1 Description - Reg Addr --> 0x47, Bit --> 4 */
#define BMI160_USER_FIFO_HEADER_ENABLE__POS               4
#define BMI160_USER_FIFO_HEADER_ENABLE__LEN               1
#define BMI160_USER_FIFO_HEADER_ENABLE__MSK               0x10
#define BMI160_USER_FIFO_HEADER_ENABLE__REG		         \
BMI160_USER_FIFO_CONFIG_1_ADDR

/* Fifo_Config_1 Description - Reg Addr --> 0x47, Bit --> 5 */
#define BMI160_USER_FIFO_MAG_ENABLE__POS               5
#define BMI160_USER_FIFO_MAG_ENABLE__LEN               1
#define BMI160_USER_FIFO_MAG_ENABLE__MSK               0x20
#define BMI160_USER_FIFO_MAG_ENABLE__REG		     \
BMI160_USER_FIFO_CONFIG_1_ADDR

/* Fifo_Config_1 Description - Reg Addr --> 0x47, Bit --> 6 */
#define BMI160_USER_FIFO_ACCEL_ENABLE__POS               6
#define BMI160_USER_FIFO_ACCEL_ENABLE__LEN               1
#define BMI160_USER_FIFO_ACCEL_ENABLE__MSK               0x40
#define BMI160_USER_FIFO_ACCEL_ENABLE__REG		        \
BMI160_USER_FIFO_CONFIG_1_ADDR

/* Fifo_Config_1 Description - Reg Addr --> 0x47, Bit --> 7 */
#define BMI160_USER_FIFO_GYRO_ENABLE__POS               7
#define BMI160_USER_FIFO_GYRO_ENABLE__LEN               1
#define BMI160_USER_FIFO_GYRO_ENABLE__MSK               0x80
#define BMI160_USER_FIFO_GYRO_ENABLE__REG		       \
BMI160_USER_FIFO_CONFIG_1_ADDR



/* Mag_IF_0 Description - Reg Addr --> 0x4b, Bit --> 1...7 */
#define BMI160_USER_I2C_DEVICE_ADDR__POS               1
#define BMI160_USER_I2C_DEVICE_ADDR__LEN               7
#define BMI160_USER_I2C_DEVICE_ADDR__MSK               0xFE
#define BMI160_USER_I2C_DEVICE_ADDR__REG	BMI160_USER_MAG_IF_0_ADDR

/* Mag_IF_1 Description - Reg Addr --> 0x4c, Bit --> 0...1 */
#define BMI160_USER_MAG_BURST__POS               0
#define BMI160_USER_MAG_BURST__LEN               2
#define BMI160_USER_MAG_BURST__MSK               0x03
#define BMI160_USER_MAG_BURST__REG               BMI160_USER_MAG_IF_1_ADDR

/* Mag_IF_1 Description - Reg Addr --> 0x4c, Bit --> 2...5 */
#define BMI160_USER_MAG_OFFSET__POS               2
#define BMI160_USER_MAG_OFFSET__LEN               4
#define BMI160_USER_MAG_OFFSET__MSK               0x3C
#define BMI160_USER_MAG_OFFSET__REG               BMI160_USER_MAG_IF_1_ADDR

/* Mag_IF_1 Description - Reg Addr --> 0x4c, Bit --> 7 */
#define BMI160_USER_MAG_MANUAL_ENABLE__POS               7
#define BMI160_USER_MAG_MANUAL_ENABLE__LEN               1
#define BMI160_USER_MAG_MANUAL_ENABLE__MSK               0x80
#define BMI160_USER_MAG_MANUAL_ENABLE__REG               \
BMI160_USER_MAG_IF_1_ADDR

/* Mag_IF_2 Description - Reg Addr --> 0x4d, Bit -->0... 7 */
#define BMI160_USER_READ_ADDR__POS               0
#define BMI160_USER_READ_ADDR__LEN               8
#define BMI160_USER_READ_ADDR__MSK               0xFF
#define BMI160_USER_READ_ADDR__REG               BMI160_USER_MAG_IF_2_ADDR

/* Mag_IF_3 Description - Reg Addr --> 0x4e, Bit -->0... 7 */
#define BMI160_USER_WRITE_ADDR__POS               0
#define BMI160_USER_WRITE_ADDR__LEN               8
#define BMI160_USER_WRITE_ADDR__MSK               0xFF
#define BMI160_USER_WRITE_ADDR__REG               BMI160_USER_MAG_IF_3_ADDR

/* Mag_IF_4 Description - Reg Addr --> 0x4f, Bit -->0... 7 */
#define BMI160_USER_WRITE_DATA__POS               0
#define BMI160_USER_WRITE_DATA__LEN               8
#define BMI160_USER_WRITE_DATA__MSK               0xFF
#define BMI160_USER_WRITE_DATA__REG               BMI160_USER_MAG_IF_4_ADDR

/* Int_En_0 Description - Reg Addr --> 0x50, Bit -->0 */
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_X_ENABLE__POS               0
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_X_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_X_ENABLE__MSK               0x01
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_X_ENABLE__REG	              \
BMI160_USER_INTR_ENABLE_0_ADDR

/* Int_En_0 Description - Reg Addr --> 0x50, Bit -->1 */
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Y_ENABLE__POS               1
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Y_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Y_ENABLE__MSK               0x02
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Y_ENABLE__REG	          \
BMI160_USER_INTR_ENABLE_0_ADDR

/* Int_En_0 Description - Reg Addr --> 0x50, Bit -->2 */
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Z_ENABLE__POS               2
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Z_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Z_ENABLE__MSK               0x04
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Z_ENABLE__REG	            \
BMI160_USER_INTR_ENABLE_0_ADDR

/* Int_En_0 Description - Reg Addr --> 0x50, Bit -->4 */
#define BMI160_USER_INTR_ENABLE_0_DOUBLE_TAP_ENABLE__POS               4
#define BMI160_USER_INTR_ENABLE_0_DOUBLE_TAP_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_0_DOUBLE_TAP_ENABLE__MSK               0x10
#define BMI160_USER_INTR_ENABLE_0_DOUBLE_TAP_ENABLE__REG	        \
BMI160_USER_INTR_ENABLE_0_ADDR

/* Int_En_0 Description - Reg Addr --> 0x50, Bit -->5 */
#define BMI160_USER_INTR_ENABLE_0_SINGLE_TAP_ENABLE__POS               5
#define BMI160_USER_INTR_ENABLE_0_SINGLE_TAP_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_0_SINGLE_TAP_ENABLE__MSK               0x20
#define BMI160_USER_INTR_ENABLE_0_SINGLE_TAP_ENABLE__REG	       \
BMI160_USER_INTR_ENABLE_0_ADDR

/* Int_En_0 Description - Reg Addr --> 0x50, Bit -->6 */
#define BMI160_USER_INTR_ENABLE_0_ORIENT_ENABLE__POS               6
#define BMI160_USER_INTR_ENABLE_0_ORIENT_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_0_ORIENT_ENABLE__MSK               0x40
#define BMI160_USER_INTR_ENABLE_0_ORIENT_ENABLE__REG	           \
BMI160_USER_INTR_ENABLE_0_ADDR

/* Int_En_0 Description - Reg Addr --> 0x50, Bit -->7 */
#define BMI160_USER_INTR_ENABLE_0_FLAT_ENABLE__POS               7
#define BMI160_USER_INTR_ENABLE_0_FLAT_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_0_FLAT_ENABLE__MSK               0x80
#define BMI160_USER_INTR_ENABLE_0_FLAT_ENABLE__REG	           \
BMI160_USER_INTR_ENABLE_0_ADDR

/* Int_En_1 Description - Reg Addr --> 0x51, Bit -->0 */
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_X_ENABLE__POS               0
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_X_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_X_ENABLE__MSK               0x01
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_X_ENABLE__REG	           \
BMI160_USER_INTR_ENABLE_1_ADDR

/* Int_En_1 Description - Reg Addr --> 0x51, Bit -->1 */
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_Y_ENABLE__POS               1
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_Y_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_Y_ENABLE__MSK               0x02
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_Y_ENABLE__REG	           \
BMI160_USER_INTR_ENABLE_1_ADDR

/* Int_En_1 Description - Reg Addr --> 0x51, Bit -->2 */
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_Z_ENABLE__POS               2
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_Z_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_Z_ENABLE__MSK               0x04
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_Z_ENABLE__REG	           \
BMI160_USER_INTR_ENABLE_1_ADDR

/* Int_En_1 Description - Reg Addr --> 0x51, Bit -->3 */
#define BMI160_USER_INTR_ENABLE_1_LOW_G_ENABLE__POS               3
#define BMI160_USER_INTR_ENABLE_1_LOW_G_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_1_LOW_G_ENABLE__MSK               0x08
#define BMI160_USER_INTR_ENABLE_1_LOW_G_ENABLE__REG	          \
BMI160_USER_INTR_ENABLE_1_ADDR

/* Int_En_1 Description - Reg Addr --> 0x51, Bit -->4 */
#define BMI160_USER_INTR_ENABLE_1_DATA_RDY_ENABLE__POS               4
#define BMI160_USER_INTR_ENABLE_1_DATA_RDY_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_1_DATA_RDY_ENABLE__MSK               0x10
#define BMI160_USER_INTR_ENABLE_1_DATA_RDY_ENABLE__REG	            \
BMI160_USER_INTR_ENABLE_1_ADDR

/* Int_En_1 Description - Reg Addr --> 0x51, Bit -->5 */
#define BMI160_USER_INTR_ENABLE_1_FIFO_FULL_ENABLE__POS               5
#define BMI160_USER_INTR_ENABLE_1_FIFO_FULL_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_1_FIFO_FULL_ENABLE__MSK               0x20
#define BMI160_USER_INTR_ENABLE_1_FIFO_FULL_ENABLE__REG	              \
BMI160_USER_INTR_ENABLE_1_ADDR

/* Int_En_1 Description - Reg Addr --> 0x51, Bit -->6 */
#define BMI160_USER_INTR_ENABLE_1_FIFO_WM_ENABLE__POS               6
#define BMI160_USER_INTR_ENABLE_1_FIFO_WM_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_1_FIFO_WM_ENABLE__MSK               0x40
#define BMI160_USER_INTR_ENABLE_1_FIFO_WM_ENABLE__REG	           \
BMI160_USER_INTR_ENABLE_1_ADDR

/* Int_En_2 Description - Reg Addr --> 0x52, Bit -->0 */
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_X_ENABLE__POS               0
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_X_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_X_ENABLE__MSK               0x01
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_X_ENABLE__REG	  \
BMI160_USER_INTR_ENABLE_2_ADDR

/* Int_En_2 Description - Reg Addr --> 0x52, Bit -->1 */
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_Y_ENABLE__POS               1
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_Y_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_Y_ENABLE__MSK               0x02
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_Y_ENABLE__REG	  \
BMI160_USER_INTR_ENABLE_2_ADDR

/* Int_En_2 Description - Reg Addr --> 0x52, Bit -->2 */
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_Z_ENABLE__POS               2
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_Z_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_Z_ENABLE__MSK               0x04
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_Z_ENABLE__REG	  \
BMI160_USER_INTR_ENABLE_2_ADDR

/* Int_En_2 Description - Reg Addr --> 0x52, Bit -->3 */
#define BMI160_USER_INTR_ENABLE_2_STEP_DETECTOR_ENABLE__POS               3
#define BMI160_USER_INTR_ENABLE_2_STEP_DETECTOR_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_2_STEP_DETECTOR_ENABLE__MSK               0x08
#define BMI160_USER_INTR_ENABLE_2_STEP_DETECTOR_ENABLE__REG	  \
BMI160_USER_INTR_ENABLE_2_ADDR

/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->0 */
#define BMI160_USER_INTR1_EDGE_CTRL__POS               0
#define BMI160_USER_INTR1_EDGE_CTRL__LEN               1
#define BMI160_USER_INTR1_EDGE_CTRL__MSK               0x01
#define BMI160_USER_INTR1_EDGE_CTRL__REG		\
BMI160_USER_INTR_OUT_CTRL_ADDR

/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->1 */
#define BMI160_USER_INTR1_LEVEL__POS               1
#define BMI160_USER_INTR1_LEVEL__LEN               1
#define BMI160_USER_INTR1_LEVEL__MSK               0x02
#define BMI160_USER_INTR1_LEVEL__REG               \
BMI160_USER_INTR_OUT_CTRL_ADDR

/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->2 */
#define BMI160_USER_INTR1_OUTPUT_TYPE__POS               2
#define BMI160_USER_INTR1_OUTPUT_TYPE__LEN               1
#define BMI160_USER_INTR1_OUTPUT_TYPE__MSK               0x04
#define BMI160_USER_INTR1_OUTPUT_TYPE__REG               \
BMI160_USER_INTR_OUT_CTRL_ADDR

/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->3 */
#define BMI160_USER_INTR1_OUTPUT_ENABLE__POS               3
#define BMI160_USER_INTR1_OUTPUT_ENABLE__LEN               1
#define BMI160_USER_INTR1_OUTPUT_ENABLE__MSK               0x08
#define BMI160_USER_INTR1_OUTPUT_ENABLE__REG		\
BMI160_USER_INTR_OUT_CTRL_ADDR

/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->4 */
#define BMI160_USER_INTR2_EDGE_CTRL__POS               4
#define BMI160_USER_INTR2_EDGE_CTRL__LEN               1
#define BMI160_USER_INTR2_EDGE_CTRL__MSK               0x10
#define BMI160_USER_INTR2_EDGE_CTRL__REG		\
BMI160_USER_INTR_OUT_CTRL_ADDR

/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->5 */
#define BMI160_USER_INTR2_LEVEL__POS               5
#define BMI160_USER_INTR2_LEVEL__LEN               1
#define BMI160_USER_INTR2_LEVEL__MSK               0x20
#define BMI160_USER_INTR2_LEVEL__REG               \
BMI160_USER_INTR_OUT_CTRL_ADDR

/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->6 */
#define BMI160_USER_INTR2_OUTPUT_TYPE__POS               6
#define BMI160_USER_INTR2_OUTPUT_TYPE__LEN               1
#define BMI160_USER_INTR2_OUTPUT_TYPE__MSK               0x40
#define BMI160_USER_INTR2_OUTPUT_TYPE__REG               \
BMI160_USER_INTR_OUT_CTRL_ADDR

/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->7 */
#define BMI160_USER_INTR2_OUTPUT_EN__POS               7
#define BMI160_USER_INTR2_OUTPUT_EN__LEN               1
#define BMI160_USER_INTR2_OUTPUT_EN__MSK               0x80
#define BMI160_USER_INTR2_OUTPUT_EN__REG		\
BMI160_USER_INTR_OUT_CTRL_ADDR

/* Int_Latch Description - Reg Addr --> 0x54, Bit -->0...3 */
#define BMI160_USER_INTR_LATCH__POS               0
#define BMI160_USER_INTR_LATCH__LEN               4
#define BMI160_USER_INTR_LATCH__MSK               0x0F
#define BMI160_USER_INTR_LATCH__REG               BMI160_USER_INTR_LATCH_ADDR

/* Int_Latch Description - Reg Addr --> 0x54, Bit -->4 */
#define BMI160_USER_INTR1_INPUT_ENABLE__POS               4
#define BMI160_USER_INTR1_INPUT_ENABLE__LEN               1
#define BMI160_USER_INTR1_INPUT_ENABLE__MSK               0x10
#define BMI160_USER_INTR1_INPUT_ENABLE__REG               \
BMI160_USER_INTR_LATCH_ADDR

/* Int_Latch Description - Reg Addr --> 0x54, Bit -->5*/
#define BMI160_USER_INTR2_INPUT_ENABLE__POS               5
#define BMI160_USER_INTR2_INPUT_ENABLE__LEN               1
#define BMI160_USER_INTR2_INPUT_ENABLE__MSK               0x20
#define BMI160_USER_INTR2_INPUT_ENABLE__REG              \
BMI160_USER_INTR_LATCH_ADDR

/* Int_Map_0 Description - Reg Addr --> 0x55, Bit -->0 */
#define BMI160_USER_INTR_MAP_0_INTR1_LOW_G__POS               0
#define BMI160_USER_INTR_MAP_0_INTR1_LOW_G__LEN               1
#define BMI160_USER_INTR_MAP_0_INTR1_LOW_G__MSK               0x01
#define BMI160_USER_INTR_MAP_0_INTR1_LOW_G__REG	BMI160_USER_INTR_MAP_0_ADDR

/* Int_Map_0 Description - Reg Addr --> 0x55, Bit -->1 */
#define BMI160_USER_INTR_MAP_0_INTR1_HIGH_G__POS               1
#define BMI160_USER_INTR_MAP_0_INTR1_HIGH_G__LEN               1
#define BMI160_USER_INTR_MAP_0_INTR1_HIGH_G__MSK               0x02
#define BMI160_USER_INTR_MAP_0_INTR1_HIGH_G__REG	\
BMI160_USER_INTR_MAP_0_ADDR

/* Int_Map_0 Description - Reg Addr --> 0x55, Bit -->2 */
#define BMI160_USER_INTR_MAP_0_INTR1_ANY_MOTION__POS               2
#define BMI160_USER_INTR_MAP_0_INTR1_ANY_MOTION__LEN               1
#define BMI160_USER_INTR_MAP_0_INTR1_ANY_MOTION__MSK               0x04
#define BMI160_USER_INTR_MAP_0_INTR1_ANY_MOTION__REG BMI160_USER_INTR_MAP_0_ADDR

/* Int_Map_0 Description - Reg Addr --> 0x55, Bit -->3 */
#define BMI160_USER_INTR_MAP_0_INTR1_NOMOTION__POS               3
#define BMI160_USER_INTR_MAP_0_INTR1_NOMOTION__LEN               1
#define BMI160_USER_INTR_MAP_0_INTR1_NOMOTION__MSK               0x08
#define BMI160_USER_INTR_MAP_0_INTR1_NOMOTION__REG BMI160_USER_INTR_MAP_0_ADDR

/* Int_Map_0 Description - Reg Addr --> 0x55, Bit -->4 */
#define BMI160_USER_INTR_MAP_0_INTR1_DOUBLE_TAP__POS               4
#define BMI160_USER_INTR_MAP_0_INTR1_DOUBLE_TAP__LEN               1
#define BMI160_USER_INTR_MAP_0_INTR1_DOUBLE_TAP__MSK               0x10
#define BMI160_USER_INTR_MAP_0_INTR1_DOUBLE_TAP__REG	\
BMI160_USER_INTR_MAP_0_ADDR

/* Int_Map_0 Description - Reg Addr --> 0x55, Bit -->5 */
#define BMI160_USER_INTR_MAP_0_INTR1_SINGLE_TAP__POS               5
#define BMI160_USER_INTR_MAP_0_INTR1_SINGLE_TAP__LEN               1
#define BMI160_USER_INTR_MAP_0_INTR1_SINGLE_TAP__MSK               0x20
#define BMI160_USER_INTR_MAP_0_INTR1_SINGLE_TAP__REG	      \
BMI160_USER_INTR_MAP_0_ADDR

/* Int_Map_0 Description - Reg Addr --> 0x55, Bit -->6 */
#define BMI160_USER_INTR_MAP_0_INTR1_ORIENT__POS               6
#define BMI160_USER_INTR_MAP_0_INTR1_ORIENT__LEN               1
#define BMI160_USER_INTR_MAP_0_INTR1_ORIENT__MSK               0x40
#define BMI160_USER_INTR_MAP_0_INTR1_ORIENT__REG	          \
BMI160_USER_INTR_MAP_0_ADDR

/* Int_Map_0 Description - Reg Addr --> 0x56, Bit -->7 */
#define BMI160_USER_INTR_MAP_0_INTR1_FLAT__POS               7
#define BMI160_USER_INTR_MAP_0_INTR1_FLAT__LEN               1
#define BMI160_USER_INTR_MAP_0_INTR1_FLAT__MSK               0x80
#define BMI160_USER_INTR_MAP_0_INTR1_FLAT__REG	BMI160_USER_INTR_MAP_0_ADDR

/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->0 */
#define BMI160_USER_INTR_MAP_1_INTR2_PMU_TRIG__POS               0
#define BMI160_USER_INTR_MAP_1_INTR2_PMU_TRIG__LEN               1
#define BMI160_USER_INTR_MAP_1_INTR2_PMU_TRIG__MSK               0x01
#define BMI160_USER_INTR_MAP_1_INTR2_PMU_TRIG__REG BMI160_USER_INTR_MAP_1_ADDR

/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->1 */
#define BMI160_USER_INTR_MAP_1_INTR2_FIFO_FULL__POS               1
#define BMI160_USER_INTR_MAP_1_INTR2_FIFO_FULL__LEN               1
#define BMI160_USER_INTR_MAP_1_INTR2_FIFO_FULL__MSK               0x02
#define BMI160_USER_INTR_MAP_1_INTR2_FIFO_FULL__REG	         \
BMI160_USER_INTR_MAP_1_ADDR

/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->2 */
#define BMI160_USER_INTR_MAP_1_INTR2_FIFO_WM__POS               2
#define BMI160_USER_INTR_MAP_1_INTR2_FIFO_WM__LEN               1
#define BMI160_USER_INTR_MAP_1_INTR2_FIFO_WM__MSK               0x04
#define BMI160_USER_INTR_MAP_1_INTR2_FIFO_WM__REG	         \
BMI160_USER_INTR_MAP_1_ADDR

/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->3 */
#define BMI160_USER_INTR_MAP_1_INTR2_DATA_RDY__POS               3
#define BMI160_USER_INTR_MAP_1_INTR2_DATA_RDY__LEN               1
#define BMI160_USER_INTR_MAP_1_INTR2_DATA_RDY__MSK               0x08
#define BMI160_USER_INTR_MAP_1_INTR2_DATA_RDY__REG	      \
BMI160_USER_INTR_MAP_1_ADDR

/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->4 */
#define BMI160_USER_INTR_MAP_1_INTR1_PMU_TRIG__POS               4
#define BMI160_USER_INTR_MAP_1_INTR1_PMU_TRIG__LEN               1
#define BMI160_USER_INTR_MAP_1_INTR1_PMU_TRIG__MSK               0x10
#define BMI160_USER_INTR_MAP_1_INTR1_PMU_TRIG__REG BMI160_USER_INTR_MAP_1_ADDR

/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->5 */
#define BMI160_USER_INTR_MAP_1_INTR1_FIFO_FULL__POS               5
#define BMI160_USER_INTR_MAP_1_INTR1_FIFO_FULL__LEN               1
#define BMI160_USER_INTR_MAP_1_INTR1_FIFO_FULL__MSK               0x20
#define BMI160_USER_INTR_MAP_1_INTR1_FIFO_FULL__REG	       \
BMI160_USER_INTR_MAP_1_ADDR

/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->6 */
#define BMI160_USER_INTR_MAP_1_INTR1_FIFO_WM__POS               6
#define BMI160_USER_INTR_MAP_1_INTR1_FIFO_WM__LEN               1
#define BMI160_USER_INTR_MAP_1_INTR1_FIFO_WM__MSK               0x40
#define BMI160_USER_INTR_MAP_1_INTR1_FIFO_WM__REG	\
BMI160_USER_INTR_MAP_1_ADDR

/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->7 */
#define BMI160_USER_INTR_MAP_1_INTR1_DATA_RDY__POS               7
#define BMI160_USER_INTR_MAP_1_INTR1_DATA_RDY__LEN               1
#define BMI160_USER_INTR_MAP_1_INTR1_DATA_RDY__MSK               0x80
#define BMI160_USER_INTR_MAP_1_INTR1_DATA_RDY__REG	\
BMI160_USER_INTR_MAP_1_ADDR

/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->0 */
#define BMI160_USER_INTR_MAP_2_INTR2_LOW_G__POS               0
#define BMI160_USER_INTR_MAP_2_INTR2_LOW_G__LEN               1
#define BMI160_USER_INTR_MAP_2_INTR2_LOW_G__MSK               0x01
#define BMI160_USER_INTR_MAP_2_INTR2_LOW_G__REG	BMI160_USER_INTR_MAP_2_ADDR

/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->1 */
#define BMI160_USER_INTR_MAP_2_INTR2_HIGH_G__POS               1
#define BMI160_USER_INTR_MAP_2_INTR2_HIGH_G__LEN               1
#define BMI160_USER_INTR_MAP_2_INTR2_HIGH_G__MSK               0x02
#define BMI160_USER_INTR_MAP_2_INTR2_HIGH_G__REG	\
BMI160_USER_INTR_MAP_2_ADDR

/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->2 */
#define BMI160_USER_INTR_MAP_2_INTR2_ANY_MOTION__POS               2
#define BMI160_USER_INTR_MAP_2_INTR2_ANY_MOTION__LEN               1
#define BMI160_USER_INTR_MAP_2_INTR2_ANY_MOTION__MSK               0x04
#define BMI160_USER_INTR_MAP_2_INTR2_ANY_MOTION__REG BMI160_USER_INTR_MAP_2_ADDR

/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->3 */
#define BMI160_USER_INTR_MAP_2_INTR2_NOMOTION__POS               3
#define BMI160_USER_INTR_MAP_2_INTR2_NOMOTION__LEN               1
#define BMI160_USER_INTR_MAP_2_INTR2_NOMOTION__MSK               0x08
#define BMI160_USER_INTR_MAP_2_INTR2_NOMOTION__REG BMI160_USER_INTR_MAP_2_ADDR

/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->4 */
#define BMI160_USER_INTR_MAP_2_INTR2_DOUBLE_TAP__POS               4
#define BMI160_USER_INTR_MAP_2_INTR2_DOUBLE_TAP__LEN               1
#define BMI160_USER_INTR_MAP_2_INTR2_DOUBLE_TAP__MSK               0x10
#define BMI160_USER_INTR_MAP_2_INTR2_DOUBLE_TAP__REG	\
BMI160_USER_INTR_MAP_2_ADDR

/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->5 */
#define BMI160_USER_INTR_MAP_2_INTR2_SINGLE_TAP__POS               5
#define BMI160_USER_INTR_MAP_2_INTR2_SINGLE_TAP__LEN               1
#define BMI160_USER_INTR_MAP_2_INTR2_SINGLE_TAP__MSK               0x20
#define BMI160_USER_INTR_MAP_2_INTR2_SINGLE_TAP__REG	\
BMI160_USER_INTR_MAP_2_ADDR

/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->6 */
#define BMI160_USER_INTR_MAP_2_INTR2_ORIENT__POS               6
#define BMI160_USER_INTR_MAP_2_INTR2_ORIENT__LEN               1
#define BMI160_USER_INTR_MAP_2_INTR2_ORIENT__MSK               0x40
#define BMI160_USER_INTR_MAP_2_INTR2_ORIENT__REG	\
BMI160_USER_INTR_MAP_2_ADDR

/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->7 */

#define BMI160_USER_INTR_MAP_2_INTR2_FLAT__POS               7
#define BMI160_USER_INTR_MAP_2_INTR2_FLAT__LEN               1
#define BMI160_USER_INTR_MAP_2_INTR2_FLAT__MSK               0x80
#define BMI160_USER_INTR_MAP_2_INTR2_FLAT__REG	BMI160_USER_INTR_MAP_2_ADDR


/* Int_Data_0 Description - Reg Addr --> 0x58, Bit --> 3 */
#define BMI160_USER_INTR_DATA_0_INTR_TAP_SOURCE__POS               3
#define BMI160_USER_INTR_DATA_0_INTR_TAP_SOURCE__LEN               1
#define BMI160_USER_INTR_DATA_0_INTR_TAP_SOURCE__MSK               0x08
#define BMI160_USER_INTR_DATA_0_INTR_TAP_SOURCE__REG	           \
BMI160_USER_INTR_DATA_0_ADDR


/* Int_Data_0 Description - Reg Addr --> 0x58, Bit --> 7 */
#define BMI160_USER_INTR_DATA_0_INTR_LOW_HIGH_SOURCE__POS           7
#define BMI160_USER_INTR_DATA_0_INTR_LOW_HIGH_SOURCE__LEN           1
#define BMI160_USER_INTR_DATA_0_INTR_LOW_HIGH_SOURCE__MSK           0x80
#define BMI160_USER_INTR_DATA_0_INTR_LOW_HIGH_SOURCE__REG            \
BMI160_USER_INTR_DATA_0_ADDR


/* Int_Data_1 Description - Reg Addr --> 0x59, Bit --> 7 */
#define BMI160_USER_INTR_DATA_1_INTR_MOTION_SOURCE__POS               7
#define BMI160_USER_INTR_DATA_1_INTR_MOTION_SOURCE__LEN               1
#define BMI160_USER_INTR_DATA_1_INTR_MOTION_SOURCE__MSK               0x80
#define BMI160_USER_INTR_DATA_1_INTR_MOTION_SOURCE__REG               \
		BMI160_USER_INTR_DATA_1_ADDR

/* Int_LowHigh_0 Description - Reg Addr --> 0x5a, Bit --> 0...7 */
#define BMI160_USER_INTR_LOWHIGH_0_INTR_LOW_DURN__POS               0
#define BMI160_USER_INTR_LOWHIGH_0_INTR_LOW_DURN__LEN               8
#define BMI160_USER_INTR_LOWHIGH_0_INTR_LOW_DURN__MSK               0xFF
#define BMI160_USER_INTR_LOWHIGH_0_INTR_LOW_DURN__REG               \
		BMI160_USER_INTR_LOWHIGH_0_ADDR

/* Int_LowHigh_1 Description - Reg Addr --> 0x5b, Bit --> 0...7 */
#define BMI160_USER_INTR_LOWHIGH_1_INTR_LOW_THRES__POS               0
#define BMI160_USER_INTR_LOWHIGH_1_INTR_LOW_THRES__LEN               8
#define BMI160_USER_INTR_LOWHIGH_1_INTR_LOW_THRES__MSK               0xFF
#define BMI160_USER_INTR_LOWHIGH_1_INTR_LOW_THRES__REG               \
		BMI160_USER_INTR_LOWHIGH_1_ADDR

/* Int_LowHigh_2 Description - Reg Addr --> 0x5c, Bit --> 0...1 */
#define BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_HYST__POS               0
#define BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_HYST__LEN               2
#define BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_HYST__MSK               0x03
#define BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_HYST__REG               \
		BMI160_USER_INTR_LOWHIGH_2_ADDR

/* Int_LowHigh_2 Description - Reg Addr --> 0x5c, Bit --> 2 */
#define BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_MODE__POS               2
#define BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_MODE__LEN               1
#define BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_MODE__MSK               0x04
#define BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_MODE__REG               \
		BMI160_USER_INTR_LOWHIGH_2_ADDR

/* Int_LowHigh_2 Description - Reg Addr --> 0x5c, Bit --> 6...7 */
#define BMI160_USER_INTR_LOWHIGH_2_INTR_HIGH_G_HYST__POS               6
#define BMI160_USER_INTR_LOWHIGH_2_INTR_HIGH_G_HYST__LEN               2
#define BMI160_USER_INTR_LOWHIGH_2_INTR_HIGH_G_HYST__MSK               0xC0
#define BMI160_USER_INTR_LOWHIGH_2_INTR_HIGH_G_HYST__REG               \
		BMI160_USER_INTR_LOWHIGH_2_ADDR

/* Int_LowHigh_3 Description - Reg Addr --> 0x5d, Bit --> 0...7 */
#define BMI160_USER_INTR_LOWHIGH_3_INTR_HIGH_G_DURN__POS               0
#define BMI160_USER_INTR_LOWHIGH_3_INTR_HIGH_G_DURN__LEN               8
#define BMI160_USER_INTR_LOWHIGH_3_INTR_HIGH_G_DURN__MSK               0xFF
#define BMI160_USER_INTR_LOWHIGH_3_INTR_HIGH_G_DURN__REG               \
		BMI160_USER_INTR_LOWHIGH_3_ADDR

/* Int_LowHigh_4 Description - Reg Addr --> 0x5e, Bit --> 0...7 */
#define BMI160_USER_INTR_LOWHIGH_4_INTR_HIGH_THRES__POS               0
#define BMI160_USER_INTR_LOWHIGH_4_INTR_HIGH_THRES__LEN               8
#define BMI160_USER_INTR_LOWHIGH_4_INTR_HIGH_THRES__MSK               0xFF
#define BMI160_USER_INTR_LOWHIGH_4_INTR_HIGH_THRES__REG               \
		BMI160_USER_INTR_LOWHIGH_4_ADDR

/* Int_Motion_0 Description - Reg Addr --> 0x5f, Bit --> 0...1 */
#define BMI160_USER_INTR_MOTION_0_INTR_ANY_MOTION_DURN__POS               0
#define BMI160_USER_INTR_MOTION_0_INTR_ANY_MOTION_DURN__LEN               2
#define BMI160_USER_INTR_MOTION_0_INTR_ANY_MOTION_DURN__MSK               0x03
#define BMI160_USER_INTR_MOTION_0_INTR_ANY_MOTION_DURN__REG               \
		BMI160_USER_INTR_MOTION_0_ADDR

	/* Int_Motion_0 Description - Reg Addr --> 0x5f, Bit --> 2...7 */
#define BMI160_USER_INTR_MOTION_0_INTR_SLOW_NO_MOTION_DURN__POS      2
#define BMI160_USER_INTR_MOTION_0_INTR_SLOW_NO_MOTION_DURN__LEN      6
#define BMI160_USER_INTR_MOTION_0_INTR_SLOW_NO_MOTION_DURN__MSK      0xFC
#define BMI160_USER_INTR_MOTION_0_INTR_SLOW_NO_MOTION_DURN__REG       \
		BMI160_USER_INTR_MOTION_0_ADDR

/* Int_Motion_1 Description - Reg Addr --> 0x60, Bit --> 0...7 */
#define BMI160_USER_INTR_MOTION_1_INTR_ANY_MOTION_THRES__POS               0
#define BMI160_USER_INTR_MOTION_1_INTR_ANY_MOTION_THRES__LEN               8
#define BMI160_USER_INTR_MOTION_1_INTR_ANY_MOTION_THRES__MSK               0xFF
#define BMI160_USER_INTR_MOTION_1_INTR_ANY_MOTION_THRES__REG               \
		BMI160_USER_INTR_MOTION_1_ADDR

/* Int_Motion_2 Description - Reg Addr --> 0x61, Bit --> 0...7 */
#define BMI160_USER_INTR_MOTION_2_INTR_SLOW_NO_MOTION_THRES__POS       0
#define BMI160_USER_INTR_MOTION_2_INTR_SLOW_NO_MOTION_THRES__LEN       8
#define BMI160_USER_INTR_MOTION_2_INTR_SLOW_NO_MOTION_THRES__MSK       0xFF
#define BMI160_USER_INTR_MOTION_2_INTR_SLOW_NO_MOTION_THRES__REG       \
		BMI160_USER_INTR_MOTION_2_ADDR

/* Int_Motion_3 Description - Reg Addr --> 0x62, Bit --> 0 */
#define BMI160_USER_INTR_MOTION_3_INTR_SLOW_NO_MOTION_SELECT__POS	0
#define BMI160_USER_INTR_MOTION_3_INTR_SLOW_NO_MOTION_SELECT__LEN	1
#define BMI160_USER_INTR_MOTION_3_INTR_SLOW_NO_MOTION_SELECT__MSK	0x01
#define BMI160_USER_INTR_MOTION_3_INTR_SLOW_NO_MOTION_SELECT__REG   \
BMI160_USER_INTR_MOTION_3_ADDR

/* Int_Motion_3 Description - Reg Addr --> 0x62, Bit --> 1 */
#define BMI160_USER_INTR_SIGNIFICATION_MOTION_SELECT__POS	1
#define BMI160_USER_INTR_SIGNIFICATION_MOTION_SELECT__LEN		1
#define BMI160_USER_INTR_SIGNIFICATION_MOTION_SELECT__MSK		0x02
#define BMI160_USER_INTR_SIGNIFICATION_MOTION_SELECT__REG		\
		BMI160_USER_INTR_MOTION_3_ADDR

/* Int_Motion_3 Description - Reg Addr --> 0x62, Bit --> 3..2 */
#define BMI160_USER_INTR_SIGNIFICANT_MOTION_SKIP__POS		2
#define BMI160_USER_INTR_SIGNIFICANT_MOTION_SKIP__LEN		2
#define BMI160_USER_INTR_SIGNIFICANT_MOTION_SKIP__MSK		0x0C
#define BMI160_USER_INTR_SIGNIFICANT_MOTION_SKIP__REG		\
		BMI160_USER_INTR_MOTION_3_ADDR

/* Int_Motion_3 Description - Reg Addr --> 0x62, Bit --> 5..4 */
#define BMI160_USER_INTR_SIGNIFICANT_MOTION_PROOF__POS		4
#define BMI160_USER_INTR_SIGNIFICANT_MOTION_PROOF__LEN		2
#define BMI160_USER_INTR_SIGNIFICANT_MOTION_PROOF__MSK		0x30
#define BMI160_USER_INTR_SIGNIFICANT_MOTION_PROOF__REG		\
		BMI160_USER_INTR_MOTION_3_ADDR

/* INT_TAP_0 Description - Reg Addr --> 0x63, Bit --> 0..2*/
#define BMI160_USER_INTR_TAP_0_INTR_TAP_DURN__POS               0
#define BMI160_USER_INTR_TAP_0_INTR_TAP_DURN__LEN               3
#define BMI160_USER_INTR_TAP_0_INTR_TAP_DURN__MSK               0x07
#define BMI160_USER_INTR_TAP_0_INTR_TAP_DURN__REG	\
BMI160_USER_INTR_TAP_0_ADDR

/* Int_Tap_0 Description - Reg Addr --> 0x63, Bit --> 6 */
#define BMI160_USER_INTR_TAP_0_INTR_TAP_SHOCK__POS               6
#define BMI160_USER_INTR_TAP_0_INTR_TAP_SHOCK__LEN               1
#define BMI160_USER_INTR_TAP_0_INTR_TAP_SHOCK__MSK               0x40
#define BMI160_USER_INTR_TAP_0_INTR_TAP_SHOCK__REG BMI160_USER_INTR_TAP_0_ADDR

/* Int_Tap_0 Description - Reg Addr --> 0x63, Bit --> 7 */
#define BMI160_USER_INTR_TAP_0_INTR_TAP_QUIET__POS               7
#define BMI160_USER_INTR_TAP_0_INTR_TAP_QUIET__LEN               1
#define BMI160_USER_INTR_TAP_0_INTR_TAP_QUIET__MSK               0x80
#define BMI160_USER_INTR_TAP_0_INTR_TAP_QUIET__REG BMI160_USER_INTR_TAP_0_ADDR

/* Int_Tap_1 Description - Reg Addr --> 0x64, Bit --> 0...4 */
#define BMI160_USER_INTR_TAP_1_INTR_TAP_THRES__POS               0
#define BMI160_USER_INTR_TAP_1_INTR_TAP_THRES__LEN               5
#define BMI160_USER_INTR_TAP_1_INTR_TAP_THRES__MSK               0x1F
#define BMI160_USER_INTR_TAP_1_INTR_TAP_THRES__REG BMI160_USER_INTR_TAP_1_ADDR

/* Int_Orient_0 Description - Reg Addr --> 0x65, Bit --> 0...1 */
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_MODE__POS               0
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_MODE__LEN               2
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_MODE__MSK               0x03
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_MODE__REG               \
		BMI160_USER_INTR_ORIENT_0_ADDR

/* Int_Orient_0 Description - Reg Addr --> 0x65, Bit --> 2...3 */
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_BLOCKING__POS               2
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_BLOCKING__LEN               2
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_BLOCKING__MSK               0x0C
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_BLOCKING__REG               \
		BMI160_USER_INTR_ORIENT_0_ADDR

/* Int_Orient_0 Description - Reg Addr --> 0x65, Bit --> 4...7 */
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_HYST__POS               4
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_HYST__LEN               4
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_HYST__MSK               0xF0
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_HYST__REG               \
		BMI160_USER_INTR_ORIENT_0_ADDR

/* Int_Orient_1 Description - Reg Addr --> 0x66, Bit --> 0...5 */
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_THETA__POS               0
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_THETA__LEN               6
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_THETA__MSK               0x3F
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_THETA__REG               \
		BMI160_USER_INTR_ORIENT_1_ADDR

/* Int_Orient_1 Description - Reg Addr --> 0x66, Bit --> 6 */
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_UD_ENABLE__POS               6
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_UD_ENABLE__LEN               1
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_UD_ENABLE__MSK               0x40
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_UD_ENABLE__REG               \
		BMI160_USER_INTR_ORIENT_1_ADDR

/* Int_Orient_1 Description - Reg Addr --> 0x66, Bit --> 7 */
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_AXES_EX__POS               7
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_AXES_EX__LEN               1
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_AXES_EX__MSK               0x80
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_AXES_EX__REG               \
		BMI160_USER_INTR_ORIENT_1_ADDR

/* Int_Flat_0 Description - Reg Addr --> 0x67, Bit --> 0...5 */
#define BMI160_USER_INTR_FLAT_0_INTR_FLAT_THETA__POS               0
#define BMI160_USER_INTR_FLAT_0_INTR_FLAT_THETA__LEN               6
#define BMI160_USER_INTR_FLAT_0_INTR_FLAT_THETA__MSK               0x3F
#define BMI160_USER_INTR_FLAT_0_INTR_FLAT_THETA__REG  \
		BMI160_USER_INTR_FLAT_0_ADDR

/* Int_Flat_1 Description - Reg Addr --> 0x68, Bit --> 0...3 */
#define BMI160_USER_INTR_FLAT_1_INTR_FLAT_HYST__POS		0
#define BMI160_USER_INTR_FLAT_1_INTR_FLAT_HYST__LEN		4
#define BMI160_USER_INTR_FLAT_1_INTR_FLAT_HYST__MSK		0x0F
#define BMI160_USER_INTR_FLAT_1_INTR_FLAT_HYST__REG	 \
BMI160_USER_INTR_FLAT_1_ADDR

/* Int_Flat_1 Description - Reg Addr --> 0x68, Bit --> 4...5 */
#define BMI160_USER_INTR_FLAT_1_INTR_FLAT_HOLD__POS                4
#define BMI160_USER_INTR_FLAT_1_INTR_FLAT_HOLD__LEN                2
#define BMI160_USER_INTR_FLAT_1_INTR_FLAT_HOLD__MSK                0x30
#define BMI160_USER_INTR_FLAT_1_INTR_FLAT_HOLD__REG  \
		BMI160_USER_INTR_FLAT_1_ADDR

/* Foc_Conf Description - Reg Addr --> 0x69, Bit --> 0...1 */
#define BMI160_USER_FOC_ACCEL_Z__POS               0
#define BMI160_USER_FOC_ACCEL_Z__LEN               2
#define BMI160_USER_FOC_ACCEL_Z__MSK               0x03
#define BMI160_USER_FOC_ACCEL_Z__REG               BMI160_USER_FOC_CONFIG_ADDR

/* Foc_Conf Description - Reg Addr --> 0x69, Bit --> 2...3 */
#define BMI160_USER_FOC_ACCEL_Y__POS               2
#define BMI160_USER_FOC_ACCEL_Y__LEN               2
#define BMI160_USER_FOC_ACCEL_Y__MSK               0x0C
#define BMI160_USER_FOC_ACCEL_Y__REG               BMI160_USER_FOC_CONFIG_ADDR

/* Foc_Conf Description - Reg Addr --> 0x69, Bit --> 4...5 */
#define BMI160_USER_FOC_ACCEL_X__POS               4
#define BMI160_USER_FOC_ACCEL_X__LEN               2
#define BMI160_USER_FOC_ACCEL_X__MSK               0x30
#define BMI160_USER_FOC_ACCEL_X__REG               BMI160_USER_FOC_CONFIG_ADDR

/* Foc_Conf Description - Reg Addr --> 0x69, Bit --> 6 */
#define BMI160_USER_FOC_GYRO_ENABLE__POS               6
#define BMI160_USER_FOC_GYRO_ENABLE__LEN               1
#define BMI160_USER_FOC_GYRO_ENABLE__MSK               0x40
#define BMI160_USER_FOC_GYRO_ENABLE__REG               \
BMI160_USER_FOC_CONFIG_ADDR

/* CONF Description - Reg Addr --> 0x6A, Bit --> 1 */
#define BMI160_USER_CONFIG_NVM_PROG_ENABLE__POS               1
#define BMI160_USER_CONFIG_NVM_PROG_ENABLE__LEN               1
#define BMI160_USER_CONFIG_NVM_PROG_ENABLE__MSK               0x02
#define BMI160_USER_CONFIG_NVM_PROG_ENABLE__REG               \
BMI160_USER_CONFIG_ADDR

/*IF_CONF Description - Reg Addr --> 0x6B, Bit --> 0 */

#define BMI160_USER_IF_CONFIG_SPI3__POS               0
#define BMI160_USER_IF_CONFIG_SPI3__LEN               1
#define BMI160_USER_IF_CONFIG_SPI3__MSK               0x01
#define BMI160_USER_IF_CONFIG_SPI3__REG               BMI160_USER_IF_CONFIG_ADDR

/*IF_CONF Description - Reg Addr --> 0x6B, Bit --> 5..4 */
#define BMI160_USER_IF_CONFIG_IF_MODE__POS               4
#define BMI160_USER_IF_CONFIG_IF_MODE__LEN               2
#define BMI160_USER_IF_CONFIG_IF_MODE__MSK               0x30
#define BMI160_USER_IF_CONFIG_IF_MODE__REG		\
BMI160_USER_IF_CONFIG_ADDR

/* Pmu_Trigger Description - Reg Addr --> 0x6c, Bit --> 0...2 */
#define BMI160_USER_GYRO_SLEEP_TRIGGER__POS               0
#define BMI160_USER_GYRO_SLEEP_TRIGGER__LEN               3
#define BMI160_USER_GYRO_SLEEP_TRIGGER__MSK               0x07
#define BMI160_USER_GYRO_SLEEP_TRIGGER__REG	BMI160_USER_PMU_TRIGGER_ADDR

/* Pmu_Trigger Description - Reg Addr --> 0x6c, Bit --> 3...4 */
#define BMI160_USER_GYRO_WAKEUP_TRIGGER__POS               3
#define BMI160_USER_GYRO_WAKEUP_TRIGGER__LEN               2
#define BMI160_USER_GYRO_WAKEUP_TRIGGER__MSK               0x18
#define BMI160_USER_GYRO_WAKEUP_TRIGGER__REG	BMI160_USER_PMU_TRIGGER_ADDR

/* Pmu_Trigger Description - Reg Addr --> 0x6c, Bit --> 5 */
#define BMI160_USER_GYRO_SLEEP_STATE__POS               5
#define BMI160_USER_GYRO_SLEEP_STATE__LEN               1
#define BMI160_USER_GYRO_SLEEP_STATE__MSK               0x20
#define BMI160_USER_GYRO_SLEEP_STATE__REG	BMI160_USER_PMU_TRIGGER_ADDR

/* Pmu_Trigger Description - Reg Addr --> 0x6c, Bit --> 6 */
#define BMI160_USER_GYRO_WAKEUP_INTR__POS               6
#define BMI160_USER_GYRO_WAKEUP_INTR__LEN               1
#define BMI160_USER_GYRO_WAKEUP_INTR__MSK               0x40
#define BMI160_USER_GYRO_WAKEUP_INTR__REG	BMI160_USER_PMU_TRIGGER_ADDR

/* Self_Test Description - Reg Addr --> 0x6d, Bit --> 0...1 */
#define BMI160_USER_ACCEL_SELFTEST_AXIS__POS               0
#define BMI160_USER_ACCEL_SELFTEST_AXIS__LEN               2
#define BMI160_USER_ACCEL_SELFTEST_AXIS__MSK               0x03
#define BMI160_USER_ACCEL_SELFTEST_AXIS__REG	BMI160_USER_SELF_TEST_ADDR

/* Self_Test Description - Reg Addr --> 0x6d, Bit --> 2 */
#define BMI160_USER_ACCEL_SELFTEST_SIGN__POS               2
#define BMI160_USER_ACCEL_SELFTEST_SIGN__LEN               1
#define BMI160_USER_ACCEL_SELFTEST_SIGN__MSK               0x04
#define BMI160_USER_ACCEL_SELFTEST_SIGN__REG	BMI160_USER_SELF_TEST_ADDR

/* Self_Test Description - Reg Addr --> 0x6d, Bit --> 3 */
#define BMI160_USER_SELFTEST_AMP__POS               3
#define BMI160_USER_SELFTEST_AMP__LEN               1
#define BMI160_USER_SELFTEST_AMP__MSK               0x08
#define BMI160_USER_SELFTEST_AMP__REG		BMI160_USER_SELF_TEST_ADDR

/* Self_Test Description - Reg Addr --> 0x6d, Bit --> 4 */
#define BMI160_USER_GYRO_SELFTEST_START__POS               4
#define BMI160_USER_GYRO_SELFTEST_START__LEN               1
#define BMI160_USER_GYRO_SELFTEST_START__MSK               0x10
#define BMI160_USER_GYRO_SELFTEST_START__REG		    \
BMI160_USER_SELF_TEST_ADDR

/* NV_CONF Description - Reg Addr --> 0x70, Bit --> 0 */
#define BMI160_USER_NV_CONFIG_SPI_ENABLE__POS               0
#define BMI160_USER_NV_CONFIG_SPI_ENABLE__LEN               1
#define BMI160_USER_NV_CONFIG_SPI_ENABLE__MSK               0x01
#define BMI160_USER_NV_CONFIG_SPI_ENABLE__REG	 BMI160_USER_NV_CONFIG_ADDR

/*IF_CONF Description - Reg Addr --> 0x70, Bit --> 1 */
#define BMI160_USER_IF_CONFIG_I2C_WDT_SELECT__POS               1
#define BMI160_USER_IF_CONFIG_I2C_WDT_SELECT__LEN               1
#define BMI160_USER_IF_CONFIG_I2C_WDT_SELECT__MSK               0x02
#define BMI160_USER_IF_CONFIG_I2C_WDT_SELECT__REG		\
BMI160_USER_NV_CONFIG_ADDR

/*IF_CONF Description - Reg Addr --> 0x70, Bit --> 2 */
#define BMI160_USER_IF_CONFIG_I2C_WDT_ENABLE__POS               2
#define BMI160_USER_IF_CONFIG_I2C_WDT_ENABLE__LEN               1
#define BMI160_USER_IF_CONFIG_I2C_WDT_ENABLE__MSK               0x04
#define BMI160_USER_IF_CONFIG_I2C_WDT_ENABLE__REG		\
BMI160_USER_NV_CONFIG_ADDR

/* NV_CONF Description - Reg Addr --> 0x70, Bit --> 3 */
#define BMI160_USER_NV_CONFIG_SPARE0__POS               3
#define BMI160_USER_NV_CONFIG_SPARE0__LEN               1
#define BMI160_USER_NV_CONFIG_SPARE0__MSK               0x08
#define BMI160_USER_NV_CONFIG_SPARE0__REG	BMI160_USER_NV_CONFIG_ADDR

/* NV_CONF Description - Reg Addr --> 0x70, Bit --> 4...7 */
#define BMI160_USER_NV_CONFIG_NVM_COUNTER__POS               4
#define BMI160_USER_NV_CONFIG_NVM_COUNTER__LEN               4
#define BMI160_USER_NV_CONFIG_NVM_COUNTER__MSK               0xF0
#define BMI160_USER_NV_CONFIG_NVM_COUNTER__REG	BMI160_USER_NV_CONFIG_ADDR

/* Offset_0 Description - Reg Addr --> 0x71, Bit --> 0...7 */
#define BMI160_USER_OFFSET_0_ACCEL_OFF_X__POS               0
#define BMI160_USER_OFFSET_0_ACCEL_OFF_X__LEN               8
#define BMI160_USER_OFFSET_0_ACCEL_OFF_X__MSK               0xFF
#define BMI160_USER_OFFSET_0_ACCEL_OFF_X__REG	BMI160_USER_OFFSET_0_ADDR

/* Offset_1 Description - Reg Addr --> 0x72, Bit --> 0...7 */
#define BMI160_USER_OFFSET_1_ACCEL_OFF_Y__POS               0
#define BMI160_USER_OFFSET_1_ACCEL_OFF_Y__LEN               8
#define BMI160_USER_OFFSET_1_ACCEL_OFF_Y__MSK               0xFF
#define BMI160_USER_OFFSET_1_ACCEL_OFF_Y__REG	BMI160_USER_OFFSET_1_ADDR

/* Offset_2 Description - Reg Addr --> 0x73, Bit --> 0...7 */
#define BMI160_USER_OFFSET_2_ACCEL_OFF_Z__POS               0
#define BMI160_USER_OFFSET_2_ACCEL_OFF_Z__LEN               8
#define BMI160_USER_OFFSET_2_ACCEL_OFF_Z__MSK               0xFF
#define BMI160_USER_OFFSET_2_ACCEL_OFF_Z__REG	BMI160_USER_OFFSET_2_ADDR

/* Offset_3 Description - Reg Addr --> 0x74, Bit --> 0...7 */
#define BMI160_USER_OFFSET_3_GYRO_OFF_X__POS               0
#define BMI160_USER_OFFSET_3_GYRO_OFF_X__LEN               8
#define BMI160_USER_OFFSET_3_GYRO_OFF_X__MSK               0xFF
#define BMI160_USER_OFFSET_3_GYRO_OFF_X__REG	BMI160_USER_OFFSET_3_ADDR

/* Offset_4 Description - Reg Addr --> 0x75, Bit --> 0...7 */
#define BMI160_USER_OFFSET_4_GYRO_OFF_Y__POS               0
#define BMI160_USER_OFFSET_4_GYRO_OFF_Y__LEN               8
#define BMI160_USER_OFFSET_4_GYRO_OFF_Y__MSK               0xFF
#define BMI160_USER_OFFSET_4_GYRO_OFF_Y__REG	BMI160_USER_OFFSET_4_ADDR

/* Offset_5 Description - Reg Addr --> 0x76, Bit --> 0...7 */
#define BMI160_USER_OFFSET_5_GYRO_OFF_Z__POS               0
#define BMI160_USER_OFFSET_5_GYRO_OFF_Z__LEN               8
#define BMI160_USER_OFFSET_5_GYRO_OFF_Z__MSK               0xFF
#define BMI160_USER_OFFSET_5_GYRO_OFF_Z__REG	BMI160_USER_OFFSET_5_ADDR


/* Offset_6 Description - Reg Addr --> 0x77, Bit --> 0..1 */
#define BMI160_USER_OFFSET_6_GYRO_OFF_X__POS               0
#define BMI160_USER_OFFSET_6_GYRO_OFF_X__LEN               2
#define BMI160_USER_OFFSET_6_GYRO_OFF_X__MSK               0x03
#define BMI160_USER_OFFSET_6_GYRO_OFF_X__REG	BMI160_USER_OFFSET_6_ADDR

/* Offset_6 Description - Reg Addr --> 0x77, Bit --> 2...3 */
#define BMI160_USER_OFFSET_6_GYRO_OFF_Y__POS               2
#define BMI160_USER_OFFSET_6_GYRO_OFF_Y__LEN               2
#define BMI160_USER_OFFSET_6_GYRO_OFF_Y__MSK               0x0C
#define BMI160_USER_OFFSET_6_GYRO_OFF_Y__REG	BMI160_USER_OFFSET_6_ADDR

/* Offset_6 Description - Reg Addr --> 0x77, Bit --> 4...5 */
#define BMI160_USER_OFFSET_6_GYRO_OFF_Z__POS               4
#define BMI160_USER_OFFSET_6_GYRO_OFF_Z__LEN               2
#define BMI160_USER_OFFSET_6_GYRO_OFF_Z__MSK               0x30
#define BMI160_USER_OFFSET_6_GYRO_OFF_Z__REG	 BMI160_USER_OFFSET_6_ADDR

/* Offset_6 Description - Reg Addr --> 0x77, Bit --> 6 */
#define BMI160_USER_OFFSET_6_ACCEL_OFF_ENABLE__POS               6
#define BMI160_USER_OFFSET_6_ACCEL_OFF_ENABLE__LEN               1
#define BMI160_USER_OFFSET_6_ACCEL_OFF_ENABLE__MSK               0x40
#define BMI160_USER_OFFSET_6_ACCEL_OFF_ENABLE__REG	 \
BMI160_USER_OFFSET_6_ADDR

/* Offset_6 Description - Reg Addr --> 0x77, Bit -->  7 */
#define BMI160_USER_OFFSET_6_GYRO_OFF_EN__POS               7
#define BMI160_USER_OFFSET_6_GYRO_OFF_EN__LEN               1
#define BMI160_USER_OFFSET_6_GYRO_OFF_EN__MSK               0x80
#define BMI160_USER_OFFSET_6_GYRO_OFF_EN__REG	 BMI160_USER_OFFSET_6_ADDR

/* STEP_CNT_0  Description - Reg Addr --> 0x78, Bit -->  0 to 7 */
#define BMI160_USER_STEP_COUNT_LSB__POS               0
#define BMI160_USER_STEP_COUNT_LSB__LEN               7
#define BMI160_USER_STEP_COUNT_LSB__MSK               0xFF
#define BMI160_USER_STEP_COUNT_LSB__REG	 BMI160_USER_STEP_COUNT_0_ADDR

/* STEP_CNT_1  Description - Reg Addr --> 0x79, Bit -->  0 to 7 */
#define BMI160_USER_STEP_COUNT_MSB__POS               0
#define BMI160_USER_STEP_COUNT_MSB__LEN               7
#define BMI160_USER_STEP_COUNT_MSB__MSK               0xFF
#define BMI160_USER_STEP_COUNT_MSB__REG	 BMI160_USER_STEP_COUNT_1_ADDR

/* STEP_CONFIG_0  Description - Reg Addr --> 0x7A, Bit -->  0 to 7 */
#define BMI160_USER_STEP_CONFIG_ZERO__POS               0
#define BMI160_USER_STEP_CONFIG_ZERO__LEN               7
#define BMI160_USER_STEP_CONFIG_ZERO__MSK               0xFF
#define BMI160_USER_STEP_CONFIG_ZERO__REG	 BMI160_USER_STEP_CONFIG_0_ADDR


/* STEP_CONFIG_1  Description - Reg Addr --> 0x7B, Bit -->  0 to 2 and
4 to 7 */
#define BMI160_USER_STEP_CONFIG_ONE_CNF1__POS               0
#define BMI160_USER_STEP_CONFIG_ONE_CNF1__LEN               3
#define BMI160_USER_STEP_CONFIG_ONE_CNF1__MSK               0x07
#define BMI160_USER_STEP_CONFIG_ONE_CNF1__REG	 BMI160_USER_STEP_CONFIG_1_ADDR

#define BMI160_USER_STEP_CONFIG_ONE_CNF2__POS               4
#define BMI160_USER_STEP_CONFIG_ONE_CNF2__LEN               4
#define BMI160_USER_STEP_CONFIG_ONE_CNF2__MSK               0xF0
#define BMI160_USER_STEP_CONFIG_ONE_CNF2__REG	 BMI160_USER_STEP_CONFIG_1_ADDR

/* STEP_CONFIG_1  Description - Reg Addr --> 0x7B, Bit -->  0 to 2 */
#define BMI160_USER_STEP_CONFIG_1_STEP_COUNT_ENABLE__POS		3
#define BMI160_USER_STEP_CONFIG_1_STEP_COUNT_ENABLE__LEN		1
#define BMI160_USER_STEP_CONFIG_1_STEP_COUNT_ENABLE__MSK		0x08
#define BMI160_USER_STEP_CONFIG_1_STEP_COUNT_ENABLE__REG	\
BMI160_USER_STEP_CONFIG_1_ADDR

/* USER REGISTERS DEFINITION END */
/**************************************************************************/
/* CMD REGISTERS DEFINITION START */
/* Command description address - Reg Addr --> 0x7E, Bit -->  0....7 */
#define BMI160_CMD_COMMANDS__POS              0
#define BMI160_CMD_COMMANDS__LEN              8
#define BMI160_CMD_COMMANDS__MSK              0xFF
#define BMI160_CMD_COMMANDS__REG	 BMI160_CMD_COMMANDS_ADDR

/* Target page address - Reg Addr --> 0x7F, Bit -->  4....5 */
#define BMI160_CMD_TARGET_PAGE__POS           4
#define BMI160_CMD_TARGET_PAGE__LEN           2
#define BMI160_CMD_TARGET_PAGE__MSK           0x30
#define BMI160_CMD_TARGET_PAGE__REG	  BMI160_CMD_EXT_MODE_ADDR

/* Target page address - Reg Addr --> 0x7F, Bit -->  4....5 */
#define BMI160_CMD_PAGING_EN__POS           7
#define BMI160_CMD_PAGING_EN__LEN           1
#define BMI160_CMD_PAGING_EN__MSK           0x80
#define BMI160_CMD_PAGING_EN__REG		BMI160_CMD_EXT_MODE_ADDR

/* Target page address - Reg Addr --> 0x7F, Bit -->  4....5 */
#define BMI160_COM_C_TRIM_FIVE__POS           4
#define BMI160_COM_C_TRIM_FIVE__LEN           2
#define BMI160_COM_C_TRIM_FIVE__MSK           0x30
#define BMI160_COM_C_TRIM_FIVE__REG		BMI160_COM_C_TRIM_FIVE_ADDR



/**************************************************************************/
/* CMD REGISTERS DEFINITION END */

/* CONSTANTS */
#define FIFO_FRAME		1024

/* Mag sensor select */
#define BST_BMM		0
#define BST_AKM		1
/* Accel Range */
#define BMI160_ACCEL_RANGE_2G           0X03
#define BMI160_ACCEL_RANGE_4G           0X05
#define BMI160_ACCEL_RANGE_8G           0X08
#define BMI160_ACCEL_RANGE_16G          0X0C

/* BMI160 Accel ODR */
#define BMI160_ACCEL_OUTPUT_DATA_RATE_RESERVED       0x00
#define BMI160_ACCEL_OUTPUT_DATA_RATE_0_78HZ         0x01
#define BMI160_ACCEL_OUTPUT_DATA_RATE_1_56HZ         0x02
#define BMI160_ACCEL_OUTPUT_DATA_RATE_3_12HZ         0x03
#define BMI160_ACCEL_OUTPUT_DATA_RATE_6_25HZ         0x04
#define BMI160_ACCEL_OUTPUT_DATA_RATE_12_5HZ         0x05
#define BMI160_ACCEL_OUTPUT_DATA_RATE_25HZ           0x06
#define BMI160_ACCEL_OUTPUT_DATA_RATE_50HZ           0x07
#define BMI160_ACCEL_OUTPUT_DATA_RATE_100HZ          0x08
#define BMI160_ACCEL_OUTPUT_DATA_RATE_200HZ          0x09
#define BMI160_ACCEL_OUTPUT_DATA_RATE_400HZ          0x0A
#define BMI160_ACCEL_OUTPUT_DATA_RATE_800HZ          0x0B
#define BMI160_ACCEL_OUTPUT_DATA_RATE_1600HZ         0x0C
#define BMI160_ACCEL_OUTPUT_DATA_RATE_RESERVED0      0x0D
#define BMI160_ACCEL_OUTPUT_DATA_RATE_RESERVED1      0x0E
#define BMI160_ACCEL_OUTPUT_DATA_RATE_RESERVED2      0x0F

/* Accel bandwidth parameter */
#define BMI160_ACCEL_OSR4_AVG1			0x00
#define BMI160_ACCEL_OSR2_AVG2			0x01
#define BMI160_ACCEL_NORMAL_AVG4		0x02
#define BMI160_ACCEL_CIC_AVG8			0x03
#define BMI160_ACCEL_RES_AVG16			0x04
#define BMI160_ACCEL_RES_AVG32			0x05
#define BMI160_ACCEL_RES_AVG64			0x06
#define BMI160_ACCEL_RES_AVG128			0x07

/* BMI160 Gyro ODR */
#define BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED		0x00
#define BMI160_GYRO_OUTPUT_DATA_RATE_25HZ			0x06
#define BMI160_GYRO_OUTPUT_DATA_RATE_50HZ			0x07
#define BMI160_GYRO_OUTPUT_DATA_RATE_100HZ			0x08
#define BMI160_GYRO_OUTPUT_DATA_RATE_200HZ			0x09
#define BMI160_GYRO_OUTPUT_DATA_RATE_400HZ			0x0A
#define BMI160_GYRO_OUTPUT_DATA_RATE_800HZ			0x0B
#define BMI160_GYRO_OUTPUT_DATA_RATE_1600HZ			0x0C
#define BMI160_GYRO_OUTPUT_DATA_RATE_3200HZ			0x0D

/*Gyroscope bandwidth parameter */
#define BMI160_GYRO_OSR4_MODE		0x00
#define BMI160_GYRO_OSR2_MODE		0x01
#define BMI160_GYRO_NORMAL_MODE		0x02
#define BMI160_GYRO_CIC_MODE		0x03

/* BMI160 Mag ODR */
#define BMI160_MAG_OUTPUT_DATA_RATE_RESERVED       0x00
#define BMI160_MAG_OUTPUT_DATA_RATE_0_78HZ         0x01
#define BMI160_MAG_OUTPUT_DATA_RATE_1_56HZ         0x02
#define BMI160_MAG_OUTPUT_DATA_RATE_3_12HZ         0x03
#define BMI160_MAG_OUTPUT_DATA_RATE_6_25HZ         0x04
#define BMI160_MAG_OUTPUT_DATA_RATE_12_5HZ         0x05
#define BMI160_MAG_OUTPUT_DATA_RATE_25HZ           0x06
#define BMI160_MAG_OUTPUT_DATA_RATE_50HZ           0x07
#define BMI160_MAG_OUTPUT_DATA_RATE_100HZ          0x08
#define BMI160_MAG_OUTPUT_DATA_RATE_200HZ          0x09
#define BMI160_MAG_OUTPUT_DATA_RATE_400HZ          0x0A
#define BMI160_MAG_OUTPUT_DATA_RATE_800HZ          0x0B
#define BMI160_MAG_OUTPUT_DATA_RATE_1600HZ         0x0C
#define BMI160_MAG_OUTPUT_DATA_RATE_RESERVED0      0x0D
#define BMI160_MAG_OUTPUT_DATA_RATE_RESERVED1      0x0E
#define BMI160_MAG_OUTPUT_DATA_RATE_RESERVED2      0x0F

/*Reserved*/

/* Enable accel and gyro offset */
#define ACCEL_OFFSET_ENABLE		0x01
#define GYRO_OFFSET_ENABLE		0x01

/* command register definition */
#define START_FOC_ACCEL_GYRO	0X03

 /* INT ENABLE 1 */
#define BMI160_ANY_MOTION_X_ENABLE       0
#define BMI160_ANY_MOTION_Y_ENABLE       1
#define BMI160_ANY_MOTION_Z_ENABLE       2
#define BMI160_DOUBLE_TAP_ENABLE         4
#define BMI160_SINGLE_TAP_ENABLE         5
#define BMI160_ORIENT_ENABLE        6
#define BMI160_FLAT_ENABLE          7

/* INT ENABLE 1 */
#define BMI160_HIGH_G_X_ENABLE       0
#define BMI160_HIGH_G_Y_ENABLE       1
#define BMI160_HIGH_G_Z_ENABLE       2
#define BMI160_LOW_G_ENABLE          3
#define BMI160_DATA_RDY_ENABLE         4
#define BMI160_FIFO_FULL_ENABLE        5
#define BMI160_FIFO_WM_ENABLE          6

/* INT ENABLE 2 */
#define  BMI160_NOMOTION_X_ENABLE	0
#define  BMI160_NOMOTION_Y_ENABLE	1
#define  BMI160_NOMOTION_Z_ENABLE	2

/* FOC axis selection for accel*/
#define	FOC_X_AXIS		0
#define	FOC_Y_AXIS		1
#define	FOC_Z_AXIS		2

/* IN OUT CONTROL */
#define BMI160_INTR1_EDGE_CTRL		0
#define BMI160_INTR2_EDGE_CTRL		1
#define BMI160_INTR1_LEVEL				0
#define BMI160_INTR2_LEVEL				1
#define BMI160_INTR1_OUTPUT_TYPE				0
#define BMI160_INTR2_OUTPUT_TYPE				1
#define BMI160_INTR1_OUTPUT_ENABLE		0
#define BMI160_INTR2_OUTPUT_EN		1

#define BMI160_INTR1_INPUT_EN		0
#define BMI160_INTR2_INPUT_ENABLE		1

/*  INTERRUPT MAPS    */
#define BMI160_INTR1_MAP_LOW_G		0
#define BMI160_INTR2_MAP_LOW_G		1
#define BMI160_INTR1_MAP_HIGH_G		0
#define BMI160_INTR2_MAP_HIGH_G		1
#define BMI160_INTR1_MAP_ANY_MOTION		0
#define BMI160_INTR2_MAP_ANY_MOTION		1
#define BMI160_INTR1_MAP_NOMO		0
#define BMI160_INTR2_MAP_NOMO		1
#define BMI160_INTR1_MAP_DOUBLE_TAP		0
#define BMI160_INTR2_MAP_DOUBLE_TAP		1
#define BMI160_INTR1_MAP_SINGLE_TAP		0
#define BMI160_INTR2_MAP_SINGLE_TAP		1
#define BMI160_INTR1_MAP_ORIENT		0
#define BMI160_INTR2_MAP_ORIENT		1
#define BMI160_INTR1_MAP_FLAT		0
#define BMI160_INTR2_MAP_FLAT		1
#define BMI160_INTR1_MAP_DATA_RDY		0
#define BMI160_INTR2_MAP_DATA_RDY		1
#define BMI160_INTR1_MAP_FIFO_WM			0
#define BMI160_INTR2_MAP_FIFO_WM			1
#define BMI160_INTR1_MAP_FIFO_FULL       0
#define BMI160_INTR2_MAP_FIFO_FULL       1
#define BMI160_INTR1_MAP_PMUTRIG     0
#define BMI160_INTR2_MAP_PMUTRIG		1

/*  TAP DURATION */
#define BMI160_TAP_DURN_50MS     0x00
#define BMI160_TAP_DURN_100MS	0x01
#define BMI160_TAP_DURN_150MS    0x02
#define BMI160_TAP_DURN_200MS    0x03
#define BMI160_TAP_DURN_250MS    0x04
#define BMI160_TAP_DURN_375MS    0x05
#define BMI160_TAP_DURN_500MS    0x06
#define BMI160_TAP_DURN_700MS    0x07

/* Interrupt mapping*/
#define	BMI160_MAP_INTR1		0
#define	BMI160_MAP_INTR2		1

/* step detection selection modes */
#define	BMI160_STEP_NORMAL_MODE			0
#define	BMI160_STEP_SENSITIVE_MODE		1
#define	BMI160_STEP_ROBUST_MODE			2


/* Mag trim data definitions*/
#define BMI160_MAG_DIG_X1                      0x5D
#define BMI160_MAG_DIG_Y1                      0x5E
#define BMI160_MAG_DIG_Z4_LSB                  0x62
#define BMI160_MAG_DIG_Z4_MSB                  0x63
#define BMI160_MAG_DIG_X2                      0x64
#define BMI160_MAG_DIG_Y2                      0x65
#define BMI160_MAG_DIG_Z2_LSB                  0x68
#define BMI160_MAG_DIG_Z2_MSB                  0x69
#define BMI160_MAG_DIG_Z1_LSB                  0x6A
#define BMI160_MAG_DIG_Z1_MSB                  0x6B
#define BMI160_MAG_DIG_XYZ1_LSB                0x6C
#define BMI160_MAG_DIG_XYZ1_MSB                0x6D
#define BMI160_MAG_DIG_Z3_LSB                  0x6E
#define BMI160_MAG_DIG_Z3_MSB                  0x6F
#define BMI160_MAG_DIG_XY2                     0x70
#define BMI160_MAG_DIG_XY1                     0x71

/* AKM compensating data registers */
#define BMI160_BST_AKM_ASAX		0x60
#define BMI160_BST_AKM_ASAY		0x61
#define BMI160_BST_AKM_ASAZ		0x62

/* Mag pre-set mode definitions*/
#define BMI160_MAG_PRESETMODE_LOWPOWER                  1
#define BMI160_MAG_PRESETMODE_REGULAR                   2
#define BMI160_MAG_PRESETMODE_HIGHACCURACY              3
#define BMI160_MAG_PRESETMODE_ENHANCED                  4

/* PRESET MODES - DATA RATES */
#define BMI160_MAG_LOWPOWER_DR                       0x02
#define BMI160_MAG_REGULAR_DR                        0x02
#define BMI160_MAG_HIGHACCURACY_DR                   0x2A
#define BMI160_MAG_ENHANCED_DR                       0x02

/* PRESET MODES - REPETITIONS-XY RATES */
#define BMI160_MAG_LOWPOWER_REPXY                     1
#define BMI160_MAG_REGULAR_REPXY                      4
#define BMI160_MAG_HIGHACCURACY_REPXY                23
#define BMI160_MAG_ENHANCED_REPXY                     7

/* PRESET MODES - REPETITIONS-Z RATES */
#define BMI160_MAG_LOWPOWER_REPZ                      2
#define BMI160_MAG_REGULAR_REPZ                      14
#define BMI160_MAG_HIGHACCURACY_REPZ                 82
#define BMI160_MAG_ENHANCED_REPZ                     26

/* Secondary_Mag power mode selection*/
#define BMI160_MAG_FORCE_MODE		0
#define BMI160_MAG_SUSPEND_MODE		1

/* Mag power mode selection*/
#define	FORCE_MODE		0
#define	SUSPEND_MODE	1
#define	NORMAL_MODE		2

/* AKM power mode selection*/
#define AKM_POWER_DOWN_MODE			0
#define AKM_SINGLE_MEAS_MODE		1
#define FUSE_ROM_MODE				2


/* step configuration select mode*/
#define	STEP_CONFIG_NORMAL		0X315
#define	STEP_CONFIG_SENSITIVE		0X2D
#define	STEP_CONFIG_ROBUST		0X71D


/* Used for mag overflow check*/
#define BMI160_MAG_OVERFLOW_OUTPUT			((s16)-32768)
#define BMI160_MAG_OVERFLOW_OUTPUT_S32		((s32)(-2147483647-1))
#define BMI160_MAG_NEGATIVE_SATURATION_Z   ((s16)-32767)
#define BMI160_MAG_POSITIVE_SATURATION_Z   ((u16)32767)
#define BMI160_MAG_FLIP_OVERFLOW_ADCVAL		((s16)-4096)
#define BMI160_MAG_HALL_OVERFLOW_ADCVAL		((s16)-16384)

/* FIFO configurations */
#define FIFO_HEADER_ENABLE			0X01
#define FIFO_MAG_ENABLE				0X01
#define FIFO_ACCEL_ENABLE				0X01
#define FIFO_GYRO_ENABLE				0X01
#define FIFO_TIME_ENABLE			0X01
#define FIFO_STOPONFULL_ENABLE		0X01
#define FIFO_WM_INTERRUPT_ENABLE	0X01

#define ACCEL_MODE_NORMAL	0x11
#define GYRO_MODE_NORMAL	0x15
#define GYRO_MODE_SUSPEND	0x18
#define	ACCEL_LOWPOWER		0X12
#define MAG_SUSPEND_MODE	1

/* FIFO definitions*/
#define FIFO_HEAD_A        0x84
#define FIFO_HEAD_G        0x88
#define FIFO_HEAD_M        0x90

#define FIFO_HEAD_G_A	0x8C
#define FIFO_HEAD_M_A   0x94
#define FIFO_HEAD_M_G   0x98

#define FIFO_HEAD_M_G_A		0x9C

#define FIFO_HEAD_SENSOR_TIME			0x44
#define FIFO_HEAD_SKIP_FRAME			0x40
#define FIFO_HEAD_OVER_READ_LSB			0x80
#define FIFO_HEAD_OVER_READ_MSB			0x00


/*! FIFO 1024 byte, max fifo frame count not over 150 */
#define FIFO_FRAME_CNT		146

#define	FIFO_OVER_READ_RETURN		((s8)-10)
#define	FIFO_SENSORTIME_RETURN		((s8)-9)
#define	FIFO_SKIP_OVER_LEN			((s8)-8)
#define	FIFO_M_G_A_OVER_LEN			((s8)-7)
#define	FIFO_M_G_OVER_LEN			((s8)-6)
#define	FIFO_M_A_OVER_LEN			((s8)-5)
#define	FIFO_G_A_OVER_LEN			((s8)-4)
#define	FIFO_M_OVER_LEN				((s8)-3)
#define	FIFO_G_OVER_LEN				((s8)-2)
#define	FIFO_A_OVER_LEN				((s8)-1)



 /* BIT SLICE */
#define BMI160_GET_BITSLICE(regvar, bitname)\
		((regvar & bitname##__MSK) >> bitname##__POS)


#define BMI160_SET_BITSLICE(regvar, bitname, val)\
		((regvar & ~bitname##__MSK) | \
		((val<<bitname##__POS)&bitname##__MSK))

/********************* Function Declarations***************************/
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
BMI160_RETURN_FUNCTION_TYPE bmi160_init(struct bmi160_t *bmi160);
/****************************************************************************
 * Description: *//**brief This API gives data to the given register and
 *                          the data is written in the corresponding register
 *  address
 *
 *
 *
 *  \param u8 addr, u8 data, u8 len
 *          addr -> Address of the register
 *          data -> Data to be written to the register
 *          len  -> Length of the Data
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
u8 *v_data_u8, u8 v_len_u8);
/*****************************************************************************
 * Description: *//**brief This API reads the data from the given register
 * address
 *
 *
 *
 *
 *  \param u8 addr, u8 *data, u8 len
 *         addr -> Address of the register
 *         data -> address of the variable, read value will be kept
 *         len  -> Length of the data
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
u8 *v_data_u8, u8 v_len_u8);
/*****************************************************************************
 *	Description: *//**brief This API used to reads the fatal error
 *	from the Register 0x02 bit 0
 *
 *
 *  \param u8 * fatal_error : Pointer to the value of fatal_error
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
*v_fatal_err_u8);
/*************************************************************************
 *	Description: *//**brief This API used to read the error code
 *	Register 0x02 bit 1 to 4
 *
 *
 *  \param u8 * error_code : Pointer to the value of error_code
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
*v_error_code_u8);
/*****************************************************************************
 *	Description: *//**brief This API Reads the i2c error code from the
 *	Register 0x02 bit 5
 *
 *  \param u8 * i2c_error_code : Pointer to the error in I2C-Master detected
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
*v_i2c_error_code_u8);
 /****************************************************************************
 *	Description: *//**brief This API Reads the dropped command error
 *	from the register 0x02 bit 6
 *
 *
 *  \param u8 * drop_cmd_err : Pointer holding the
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
*v_drop_cmd_err_u8);
 /**************************************************************************
 *	Description: *//**brief This API Reads the magnetometer data ready
 *	interrupt not active from the error register 0x0x2 bit 7
 *
 *
 *
 *
 *  \param u8 * mag_data_rdy_err :
 *	Pointer to holding the value of mag_data_rdy_err
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
BMI160_RETURN_FUNCTION_TYPE bmi160_get_mag_dada_rdy_err(u8
*v_mag_data_rdy_err_u8);
 /**************************************************************************
 *	Description: *//**brief This API Reads the error status
 *	from the error register 0x02 bit 0 to 7
 *
 *
 *
 *
 *	\param u8 * mag_data_rdy_err, *fatal_err, *err_code
 *   *i2c_fail_err, *drop_cmd_err:
 *   Pointer to the value of mag_data_rdy_err, fatal_err, err_code,
 *   i2c_fail_err, drop_cmd_err
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
u8 *v_drop_cmd_err_u8, u8 *v_mag_data_rdy_err_u8);
/*****************************************************************************
 *	Description: *//**brief This API reads the magnetometer power mode from
 *	PMU status register 0x03 bit 0 and 1
 *
 *  \param u8 * mag_pmu_status :	pointer holding the power mode status of
 *	magnetometer
 *	mag_pmu_status:
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
*v_mag_power_mode_stat_u8);
/*****************************************************************************
 *	Description: *//**brief This API reads the gyroscope power mode from
 *	PMU status register 0x03 bit 2 and 3
 *
 *  \param u8 * gyro_power_mode_stat :	Pointer holding the value of
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
*v_gyro_power_mode_stat_u8);
/*****************************************************************************
 *	Description: *//**brief This API reads the accelerometer power mode from
 *	PMU status register 0x03 bit 4 and 5
 *
 *
 *  \param u8 * accel_pmu_status :	Pointer holding the accelerometer power
 *	mode status
 *	accel_pmu_status
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
*v_accel_power_mode_stat_u8);
/******************************************************************************
 *	Description: *//**brief This API reads magnetometer data X values
 *	from the register 0x04 and 0x05
 *
 *
 *
 *
 *  \param structure s16 * mag_x : Pointer holding the value of mag_x
 *  \param sensor_select : Mag selection value;
 *	0 for BMM150 and 1 for AKM09911
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
u8 v_sensor_select_u8);
/*****************************************************************************
 *	Description: *//**brief This API reads magnetometer data Y values
 *	from the register 0x06 and 0x07
 *
 *
 *
 *
 *  \param structure s16 * mag_y : Pointer holding the value of mag_y
 *  \param sensor_select : Mag selection value;
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
u8 v_sensor_select_u8);
/*****************************************************************************
 *	Description: *//**brief This API reads magnetometer data Z values
 *	from the register 0x08 and 0x09
 *
 *
 *  \param structure s16 * mag_z : Pointer holding the value of mag_z
 *  \param sensor_select : Mag selection value;
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
u8 v_sensor_select_u8);
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
BMI160_RETURN_FUNCTION_TYPE bmi160_read_mag_r(
s16 *v_mag_r_s16);
/*****************************************************************************
 *	Description: *//**brief This API reads magnetometer data X,Y,Z values
 *	from the register 0x04 to 0x09
 *
 *
 *
 *
 *  \param structure bmi160_mag_t * mag : Pointer holding the value of mag xyz
 *  \param sensor_select : Mag selection value;
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
struct bmi160_mag_t *mag, u8 v_sensor_select_u8);
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
struct bmi160_mag_xyzr_t *mag);
/*****************************************************************************
 *	Description: *//**brief This API reads gyro data X values
 *	form the register 0x0C and 0x0D
 *
 *
 *
 *
 *  \param structure s16 * gyro_x : Pointer holding the value of gyro_x
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
BMI160_RETURN_FUNCTION_TYPE bmi160_read_gyro_x(
s16 *v_gyro_x_s16);
/*******************************************************************************
 *	Description: *//**brief This API reads gyro data Y values
 *	form the register 0x0E and 0x0F
 *
 *
 *
 *
 *  \param s16 * gyro_y : Pointer holding the value of gyro_y
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
BMI160_RETURN_FUNCTION_TYPE bmi160_read_gyro_y(
s16 *v_gyro_y_s16);
/*******************************************************************************
 *	Description: *//**brief This API reads gyro data Z values
 *	form the register 0x10 and 0x11
 *
 *
 *
 *
 *  \param s16 * gyro_z : Pointer holding the value of gyro_z
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
BMI160_RETURN_FUNCTION_TYPE bmi160_read_gyro_z(
s16 *v_gyro_z_s16);
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
BMI160_RETURN_FUNCTION_TYPE bmi160_read_gyro_xyz(
struct bmi160_gyro_t *gyro);
/*****************************************************************************
 *	Description: *//**brief This API reads accelerometer data X values
 *	form the register 0x12 and 0x13
 *
 *
 *
 *
 *  \param s16 * acc_x : Pointer holding the value of acc_x
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
BMI160_RETURN_FUNCTION_TYPE bmi160_read_accel_x(
s16 *v_accel_x_s16);
/******************************************************************************
 *	Description: *//**brief This API reads accelerometer data Y values
 *	form the register 0x14 and 0x15
 *
 *
 *
 *
 *  \param s16 * acc_y : Pointer holding the value of acc_y
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
BMI160_RETURN_FUNCTION_TYPE bmi160_read_accel_y(
s16 *v_accel_y_s16);
/*****************************************************************************
 *	Description: *//**brief This API reads accelerometer data Z values
 *	form the register 0x16 and 0x17
 *
 *
 *
 *
 *  \param s16 * acc_z : Pointer holding the value of acc_z
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
BMI160_RETURN_FUNCTION_TYPE bmi160_read_accel_z(
s16 *v_accel_z_s16);
/*****************************************************************************
 *	Description: *//**brief This API reads accelerometer data X,Y,Z values
 *	from the register 0x12 to 0x17
 *
 *
 *
 *
 *  \param bmi160_accel_t * acc : Pointer holding the value of acc-xyz
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
struct bmi160_accel_t *accel);
/******************************************************************************
 *	Description: *//**brief This API reads sensor_time from the register
 *	0x18 to 0x1A
 *
 *
 *  \param u32 * sensor_time : Pointer holding the value of sensor_time
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
BMI160_RETURN_FUNCTION_TYPE bmi160_get_sensor_time(
u32 *v_sensor_time_u32);
/*****************************************************************************
 *	Description: *//**brief This API reads the Gyroscope self test
 *	status from the register 0x1B bit 1
 *
 *
 *  \param u8 * gyr_self_test_ok : Pointer holding the value of gyro self test
 *	gyr_self_test_ok ->
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
*v_gyro_selftest_u8);
 /*****************************************************************************
 *	Description: *//**brief This API reads the status of
 *	mag manual interface operation form the register 0x1B bit 2
 *
 *
 *
 *  \param u8 * mag_man_op : Pointer holding the value of mag_man_op
 *	mag_man_op ->
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
*v_mag_manual_stat_u8);
/*****************************************************************************
 *	Description: *//**brief This API reads the fast offset compensation
 *	status form the register 0x1B bit 3
 *
 *
 *  \param u8 * foc_rdy : Pointer holding the value of foc_rdy
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
*v_foc_rdy_u8);
 /*****************************************************************************
 * Description: *//**brief This APIrReads the nvm_rdy status from the
 *	resister 0x1B bit 4
 *
 *
 *  \param u8 * nvm_rdy : Pointer holding the value of nvm_rdy
 *	nvm_rdy ->
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
*v_nvm_rdy_u8);
/*****************************************************************************
 *	Description: *//**brief This API reads the status of drdy_mag
 *	from the register 0x1B bit 5
 *	The status get reset when one magneto DATA register is read out
 *
 *  \param u8 * drdy_mag :
 *	Pointer holding the value of drdy_mag
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
*v_data_rdy_u8);
/*****************************************************************************
 *	Description: *//**brief This API reads the status of v_data_rdy_u8 form the
 *	register 0x1B bit 6
 *	The status get reset when Gyroscope DATA register read out
 *
 *
 *	\param u8 * v_data_rdy_u8 :
 *	Pointer holding the value of v_data_rdy_u8
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
*v_data_rdy_u8);
 /*****************************************************************************
 *	Description: *//**brief This API reads the status of drdy_acc form the
 *	register 0x1B bit 7
 *	The status get reset when Accel DATA register read out
 *
 *
 *	\param u8 * drdy_acc :
 *	Pointer holding the value of drdy_acc
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
*drdy_acc);
 /*****************************************************************************
 *	Description: *//**brief This API reads the step interrupt status
 *	from the register 0x1C bit 0
 *	flag is associated with a specific interrupt function.
 *	It is set when the  data ready  interrupt triggers. The
 *	setting of INT_LATCH controls if
 *	the interrupt signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
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
*v_step_intr_u8);
/*****************************************************************************
 *	Description: *//**brief This API reads the
 *	significant motion interrupt status
 *	from the register 0x1C bit 1
 *	flag is associated with a specific interrupt function.
 *	It is set when the  data ready  interrupt triggers. The
 *	setting of INT_LATCH controls if
 *	the interrupt signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *
 *  \param u8 * sigmot_intr  :
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
*sigmot_intr);
 /*****************************************************************************
 *	Description: *//**brief This API reads the v_any_motion_intr_u8 status
 *	from the register 0x1C bit 2
 *	flag is associated with a specific interrupt function.
 *	It is set when the  data ready  interrupt triggers. The
 *	setting of INT_LATCH controls if
 *	the interrupt signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
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
*v_any_motion_intr_u8);
/*****************************************************************************
 *	Description: *//**brief This API reads the v_pmu_trigger_intr_u8 status
 *	from the register 0x1C bit 3
 *	flag is associated with a specific interrupt function.
 *	It is set when the  data ready  interrupt triggers. The
 *	setting of INT_LATCH controls if
 *	the interrupt signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *
 *  \param u8 * v_pmu_trigger_intr_u8 :
 *	Pointer holding the status of
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
*v_pmu_trigger_intr_u8);
/*****************************************************************************
 *	Description: *//**brief This API reads the double tab status
 *	from the register 0x1C bit 4
 *	flag is associated with a specific interrupt function.
 *	It is set when the  data ready  interrupt triggers. The
 *	setting of INT_LATCH controls if
 *	the interrupt signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *
 *  \param u8 * v_double_tap_intr_u8 :
 *	Pointer holding the status of v_double_tap_intr_u8
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
*v_double_tap_intr_u8);
/*****************************************************************************
 *	Description: *//**brief This API reads the single tab status
 *	from the register 0x1C bit 5
 *	flag is associated with a specific interrupt function.
 *	It is set when the  data ready  interrupt triggers. The
 *	setting of INT_LATCH controls if
 *	the interrupt signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *
 *  \param u8 * v_single_tap_intr_u8 :
 *	Pointer holding the status of v_single_tap_intr_u8
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
*v_single_tap_intr_u8);
/*****************************************************************************
 *	Description: *//**brief This API reads the orient status
 *	from the register 0x1C bit 6
 *	flag is associated with a specific interrupt function.
 *	It is set when the  data ready  interrupt triggers. The
 *	setting of INT_LATCH controls if
 *	the interrupt signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *
 *  \param u8 * v_orient_intr_u8 :
 *	Pointer holding the status of v_orient_intr_u8
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
*v_orient_intr_u8);
/*****************************************************************************
 *	Description: *//**brief This API reads the flat interrupt status
 *	from the register 0x1C bit 7
 *	flag is associated with a specific interrupt function.
 *	It is set when the  data ready  interrupt triggers. The
 *	setting of INT_LATCH controls if
 *	the interrupt signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *  \param u8 * v_flat_intr_u8 : Pointer holding the status of v_flat_intr_u8
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
*v_flat_intr_u8);
/*****************************************************************************
 *	Description: *//**brief This API reads the high g interrupt status
 *	from the register 0x1D bit 2
 *	flag is associated with a specific interrupt function.
 *	It is set when the  data ready  interrupt triggers. The
 *	setting of INT_LATCH controls if
 *	the interrupt signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *
 *  \param u8 * v_high_g_intr_u8 :
 *	Pointer holding the status of v_high_g_intr_u8
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
*v_high_g_intr_u8);
/*****************************************************************************
 *	Description: *//**brief This API reads the low g interrupt status
 *	from the register 0x1D bit 3
 *	flag is associated with a specific interrupt function.
 *	It is set when the  data ready  interrupt triggers. The
 *	setting of INT_LATCH controls if
 *	the interrupt signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *
 *  \param u8 * v_low_g_intr_u8 :
 *	Pointer holding the status of v_low_g_intr_u8
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
*v_low_g_intr_u8);
/*****************************************************************************
 *	Description: *//**brief This API reads data
 *	ready low g interrupt status
 *	from the register 0x1D bit 4
 *	flag is associated with a specific interrupt function.
 *	It is set when the  data ready  interrupt triggers. The
 *	setting of INT_LATCH controls if
 *	the interrupt signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *  \param u8 * v_data_rdy_intr_u8 :
 *	Pointer holding the status of v_data_rdy_intr_u8
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
*v_data_rdy_intr_u8);
/*****************************************************************************
 *	Description: *//**brief This API reads data ready
 *	FIFO full interrupt status
 *	from the register 0x1D bit 5
 *	flag is associated with a specific interrupt function.
 *	It is set when the FIFO full interrupt triggers. The
 *	setting of INT_LATCH controls if the interrupt signal and hence the
 *	respective interrupt flag will be permanently
 *	latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *  \param u8 * v_fifo_full_intr_u8 :
 *	Pointer holding the status of v_fifo_full_intr_u8
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
*v_fifo_full_intr_u8);
/*****************************************************************************
 *	Description: *//**brief This API reads data
 *	ready FIFO watermark interrupt status
 *	from the register 0x1D bit 6
 *	flag is associated with a specific interrupt function.
 *	It is set when the FIFO watermark interrupt triggers. The
 *	setting of INT_LATCH controls if the interrupt signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *  \param u8 * v_fifo_wm_intr_u8 :
 *	Pointer holding the status of v_fifo_wm_intr_u8
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
*v_fifo_wm_intr_u8);
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
 *  \param u8 * nomo_intr : Pointer holding the status of nomo_intr
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
*nomo_intr);
/*****************************************************************************
 *	Description: *//**brief This API reads the status of v_anymotion_first_x_u8
 *	from the register 0x1E bit 0
 *
 *
 *
 *
 *  \param u8 * v_anymotion_first_x_u8 :
 *	pointer holding the status of v_anymotion_first_x_u8
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
*v_anymotion_first_x_u8);
/*****************************************************************************
 *	Description: *//**brief This API reads the status of v_any_motion_first_y_u8
 *	from the register 0x1E bit 1
 *
 *
 *
 *
 *  \param u8 * v_any_motion_first_y_u8 :
 *	pointer holding the status of v_any_motion_first_y_u8
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
*v_any_motion_first_y_u8);
 /*****************************************************************************
 *	Description: *//**brief This API reads the status of v_any_motion_first_z_u8
 *	from the register 0x1E bit 2
 *
 *
 *
 *
 *  \param u8 * v_any_motion_first_z_u8 :
 *	pointer holding the status of v_any_motion_first_z_u8
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
*v_any_motion_first_z_u8);
 /*****************************************************************************
 *	Description: *//**brief This API reads the v_anymotion_sign_u8 status from the
 *	register 0x1E bit 3
 *
 *
 *
 *
 *  \param u8 * v_anymotion_sign_u8 :
 *	Pointer holding the status of
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
*v_anymotion_sign_u8);
/*****************************************************************************
 *	Description: *//**brief This API reads the v_tap_first_x_u8 status from the
 *	register 0x1E bit 4
 *
 *
 *
 *
 *  \param u8 * v_tap_first_x_u8 :
 *	Pointer holding the status of v_tap_first_x_u8
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
*v_tap_first_x_u8);
/*****************************************************************************
 *	Description: *//**brief This API reads the v_tap_first_y_u8 status from the
 *	register 0x1E bit 5
 *
 *
 *
 *
 *  \param u8 * v_tap_first_y_u8 :
 *	Pointer holding the status of v_tap_first_x_u8
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
*v_tap_first_y_u8);
/*****************************************************************************
 *	Description: *//**brief This API reads the v_tap_first_z_u8 status from the
 *	register 0x1E bit 6
 *
 *
 *
 *
 *  \param u8 * v_tap_first_z_u8 :
 *	Pointer holding the status of v_tap_first_x_u8
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
*v_tap_first_z_u8);

/*****************************************************************************
 *	Description: *//**brief This API reads the tap_sign status from the
 *	register 0x1E bit 7
 *
 *
 *
 *
 *  \param u8 * tap_sign : Pointer holding the status of tap_sign
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
*tap_sign);
/*****************************************************************************
 *	Description: *//**brief This API reads the v_high_g_first_x_u8 status from the
 *	register 0x1F bit 0
 *
 *
 *
 *
 *  \param u8 * v_high_g_first_x_u8 :
 *	Pointer holding the status of v_high_g_first_x_u8
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
*v_high_g_first_x_u8);
/*****************************************************************************
 *	Description: *//**brief This API reads the v_high_g_first_y_u8 status from the
 *	register 0x1F bit 1
 *
 *
 *
 *
 *  \param u8 * v_high_g_first_y_u8 :
 *	Pointer holding the status of v_high_g_first_y_u8
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
*v_high_g_first_y_u8);
/*****************************************************************************
 *	Description: *//**brief This API reads the v_high_g_first_z_u8 status from the
 *	register 0x1F bit 3
 *
 *
 *
 *
 *  \param u8 * v_high_g_first_z_u8 :
 *	Pointer holding the status of v_high_g_first_z_u8
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
*v_high_g_first_z_u8);
/*****************************************************************************
 *	Description: *//**brief This API reads the v_high_g_sign_u8 status from the
 *	register 0x1F bit 3
 *
 *
 *
 *
 *  \param u8 * v_high_g_sign_u8 :
 *	Pointer holding the status of v_high_g_sign_u8
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
*v_high_g_sign_u8);
/*****************************************************************************
 *	Description: *//**brief This API reads the status of v_orient_x_y_u8 plane
 *	from the register 0x1F bit 4 and 5
 *
 *
 *  \param u8 * v_orient_x_y_u8 : Pointer holding the status of v_orient_x_y_u8
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
*v_orient_xy_u8);
/*****************************************************************************
 *	Description: *//**brief This API reads the status of v_orient_z_u8 plane
 *	from the register 0x1F bit 6
 *
 *
 *  \param u8 * v_orient_z_u8 : Pointer holding the status of v_orient_z_u8
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
*v_orient_z_u8);
/*****************************************************************************
 *	Description: *//**brief This API reads the flat status from the register
 *	0x1F bit 7
 *
 *
 *  \param u8 * flat : Pointer holding the status of flat
 *	flat ->
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
*flat);
/*****************************************************************************
 *	Description: *//**brief This API reads the temperature of the sensor
 *	from the register 0x21 bit 0 to 7
 *
 *
 *
 *  \param u8 * temperature : Pointer holding the value of temperature
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
*v_temp_s16);
/*****************************************************************************
 *	Description: *//**brief This API reads the  of the sensor
 *	form the register 0x23 and 0x24 bit 0 to 7 and 0 to 2
 *
 *
 *  \param u8 * Fifo_length :  Pointer holding the Fifo_length
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
BMI160_RETURN_FUNCTION_TYPE bmi160_fifo_length(
u32 *v_fifo_length_u32);
 /*****************************************************************************
 *	Description: *//**brief This API reads the fifodata of the sensor
 *	from the register 0x24
 *
 *
 *
 *  \param u8 * fifodata : Pointer holding the fifodata
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
u8 *v_fifo_data_u8);
/*************************************************************************
 *	Description: *//**brief This API is used to get the
 *	accel output date rate form the register 0x40 bit 0 to 3
 *
 *
 *  \param  u8 * BW : pointer holding the value of accel output date rate
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
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_output_data_rate(u8 *odr);
 /****************************************************************************
 *	Description: *//**brief This API is used to set the
 *	accel output date rate form the register 0x40 bit 0 to 3
 *
 *
 *
 *
 *  \param u8 BW : The value of accel output date rate
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
BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_output_data_rate(u8 odr);
/***********************************************************************
 *	Description: *//**brief This API is used to get the
 *	accel bandwidth from the register 0x40 bit 4 to 6
 *
 *
 *
 *
 *  \param  u8 * v_bw_u8 : Pointer holding the value of accel bandwidth
 *	0b000 acc_us = 0 -> OSR4 mode; acc_us = 1 -> no averaging
 *	0b001 acc_us = 0 -> OSR2 mode; acc_us = 1 -> average 2 samples
 *	0b010 acc_us = 0 -> normal mode; acc_us = 1 -> average 4 samples
 *	0b011 acc_us = 0 -> CIC mode; acc_us = 1 -> average 8 samples
 *	0b100 acc_us = 0 -> Reserved; acc_us = 1 -> average 16 samples
 *	0b101 acc_us = 0 -> Reserved; acc_us = 1 -> average 32 samples
 *	0b110 acc_us = 0 -> Reserved; acc_us = 1 -> average 64 samples
 *	0b111 acc_us = 0 -> Reserved; acc_us = 1 -> average 128 samples
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
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_bw(u8 *v_bw_u8);
/*******************************************************************************
 *	Description: *//**brief This API is used to set the
 *	accel bandwidth from the register 0x40 bit 4 to 6
 *
 *
 *
 *
 *  \param u8 BW : the value of accel bandwidth
 *	Value Description
 *	0b000 acc_us = 0 -> OSR4 mode; acc_us = 1 -> no averaging
 *	0b001 acc_us = 0 -> OSR2 mode; acc_us = 1 -> average 2 samples
 *	0b010 acc_us = 0 -> normal mode; acc_us = 1 -> average 4 samples
 *	0b011 acc_us = 0 -> CIC mode; acc_us = 1 -> average 8 samples
 *	0b100 acc_us = 0 -> Reserved; acc_us = 1 -> average 16 samples
 *	0b101 acc_us = 0 -> Reserved; acc_us = 1 -> average 32 samples
 *	0b110 acc_us = 0 -> Reserved; acc_us = 1 -> average 64 samples
 *	0b111 acc_us = 0 -> Reserved; acc_us = 1 -> average 128 samples
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
BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_bw(u8 v_bw_u8);
/*************************************************************************
 *	Description: *//**brief This API is used to get the accel
 *	under sampling parameter form the register 0x40 bit 7
 *
 *
 *
 *
 *	\param  u8 * v_accel_under_sampling_u8 : pointer holding the
 *						value of accel under sampling
 *
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
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_under_sampling_parameter(
u8 *v_accel_under_sampling_u8);
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
u8 v_accel_under_sampling_u8);
/*************************************************************************
 *	Description: *//**brief This API is used to get the ranges
 *	(g values) of the accel from the register 0x41 bit 0 to 3
 *
 *
 *
 *
 *  \param u8 * Range : Pointer holding the accel v_range_u8
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
u8 *v_range_u8);
 /**********************************************************************
 *	Description: *//**brief This API is used to set the v_range_u8
 *	(g values) of the accel from the register 0x41 bit 0 to 3
 *
 *
 *
 *
 *  \param u8 * Range : The value of accel v_range_u8
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
BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_range(
u8 v_range_u8);
/**************************************************************************
 *	Description: *//**brief This API is used to get the
 *	gyroscope output data rate from the register 0x42 bit 0 to 3
 *
 *
 *
 *
 *  \param  u8 * gyro_OUTPUT_DATA_RATE :
 *	Pointer holding the value of output data rate
 *	gyro_OUTPUT_DATA_RATE	->
 *	0b0000 - BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *	0b0001 - BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *	0b0010 - BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *	0b0011 - BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *	0b0100 - BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *	0b0101 - BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *	0b0110 - BMI160_GYOR_OUTPUT_DATA_RATE_25HZ
 *	0b0111 - BMI160_GYOR_OUTPUT_DATA_RATE_50HZ
 *	0b1000 - BMI160_GYOR_OUTPUT_DATA_RATE_100HZ
 *	0b1001 - BMI160_GYOR_OUTPUT_DATA_RATE_200HZ
 *	0b1010 - BMI160_GYOR_OUTPUT_DATA_RATE_400HZ
 *	0b1011 - BMI160_GYOR_OUTPUT_DATA_RATE_800HZ
 *	0b1100 - BMI160_GYOR_OUTPUT_DATA_RATE_1600HZ
 *	0b1101 - BMI160_GYOR_OUTPUT_DATA_RATE_3200HZ
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
u8 *gyro_output_typer);
/************************************************************************
 * Description: *//**brief This API is used to set the
 *	gyroscope output data rate from the register 0x42 bit 0 to 3
 *
 *
 *
 *  \param  u8 gyro_OUTPUT_DATA_RATE : The value of gyro output data rate
 *
 *	0b0000 - BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *	0b0001 - BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *	0b0010 - BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *	0b0011 - BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *	0b0100 - BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *	0b0101 - BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *	0b0110 - BMI160_GYOR_OUTPUT_DATA_RATE_25HZ
 *	0b0111 - BMI160_GYOR_OUTPUT_DATA_RATE_50HZ
 *	0b1000 - BMI160_GYOR_OUTPUT_DATA_RATE_100HZ
 *	0b1001 - BMI160_GYOR_OUTPUT_DATA_RATE_200HZ
 *	0b1010 - BMI160_GYOR_OUTPUT_DATA_RATE_400HZ
 *	0b1011 - BMI160_GYOR_OUTPUT_DATA_RATE_800HZ
 *	0b1100 - BMI160_GYOR_OUTPUT_DATA_RATE_1600HZ
 *	0b1101 - BMI160_GYOR_OUTPUT_DATA_RATE_3200HZ
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
u8 gyro_output_typer);
/***********************************************************************
 *	Description: *//**brief This API is used to get the
 *	bandwidth of gyro from the register 0x42 bit 4 to 5
 *
 *
 *
 *
 *  \param  u8 * v_bw_u8 : Pointer holding the value of gyro bandwidth
 *
 *	Value	Description
 *	00 -	BMI160_GYRO_OSR4_MODE
 *	01 -	BMI160_GYRO_OSR2_MODE
 *	10 -	BMI160_GYRO_NORMAL_MODE
 *	11 -	BMI160_GYRO_CIC_MODE
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
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_bw(u8 *v_bw_u8);
/**************************************************************************
 * Description: *//**brief This API is used to set the
 *	bandwidth of gyro from the register 0x42 bit 4 to 5
 *
 *
 *
 *
 *  \param  u8 v_bw_u8 : the value of gyro bandwidth
 *
 *	Value Description
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
BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_bw(u8 v_bw_u8);
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
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_range(
u8 *v_range_u8);
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
BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_range(
u8 v_range_u8);
/***************************************************************************
 *	Description: *//**brief This API is used to get the
 *	output data rate of magnetometer from the register 0x44 bit 0 to 3
 *
 *
 *
 *
 *  \param  u8 * odr : The pointer holding the value of mag bandwidth
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
BMI160_RETURN_FUNCTION_TYPE bmi160_get_mag_output_data_rate(u8 *odr);
/*******************************************************************************
 *	Description: *//**brief This API is used to set the value of mag bandwidth
 *	from the register 0x44 bit 0 to 3
 *
 *
 *
 *
 *  \param u8 odr : The value of mag bandwidth
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
BMI160_RETURN_FUNCTION_TYPE bmi160_set_mag_output_data_rate(u8 odr);
/*******************************************************************************
 *	Description: *//**brief This API is used to read Down sampling
 *	for gyro (2**downs_gyro) in the register 0x45 bit 0 to 2
 *
 *
 *
 *
 *  \param u8 * v_fifo_down_gyro_u8 :Pointer to the v_fifo_down_gyro_u8
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
u8 *v_fifo_down_gyro_u8);
/*******************************************************************************
 *	Description: *//**brief This API is used to set Down sampling
 *	for gyro (2**downs_gyro) from the register 0x45 bit 0 to 5
 *
 *
 *
 *
 *  \param u8 v_fifo_down_gyro_u8 : Value of the v_fifo_down_gyro_u8
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
u8 v_fifo_down_gyro_u8);
/*******************************************************************************
 *	Description: *//**brief This API is used to read v_gyro_fifo_filter_data_u8
 *	from the register 0x45 bit 3
 *
 *
 *
 *  \param u8 * v_gyro_fifo_filter_data_u8 :
 *	Pointer to the v_gyro_fifo_filter_data_u8
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
u8 *v_gyro_fifo_filter_data_u8);
/*******************************************************************************
 *	Description: *//**brief This API is used to write filtered or unfiltered
 *	from the register 0x45 bit 3
 *
 *
 *
 *  \param u8 v_gyro_fifo_filter_data_u8 :
 *	The value ofS v_gyro_fifo_filter_data_u8
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
u8 v_gyro_fifo_filter_data_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to read Down sampling
 *	for accel (2**downs_accel) from the register 0x45 bit 4 to 6
 *
 *
 *
 *
 *  \param u8 * v_fifo_down_u8 :Pointer to the v_fifo_down_u8
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
u8 *v_fifo_down_u8);
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
u8 v_fifo_down_u8);
/*******************************************************************************
 *	Description: *//**brief This API is used to read v_accel_fifo_filter_u8
 *	from the register 0x45 bit 7
 *
 *
 *
 *  \param u8 * v_accel_fifo_filter_u8 :Pointer to the v_accel_fifo_filter_u8
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
u8 *v_accel_fifo_filter_u8);
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
u8 v_accel_fifo_filter_u8);
/*******************************************************************************
 *	Description: *//**brief This API is used to Trigger an interrupt
 *	when FIFO contains fifo_water_mark from the register 0x46 bit 0 to 7
 *
 *
 *
 *  \param u8 v_fifo_wm_u8 : Pointer to v_fifo_wm_u8
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
u8 *v_fifo_wm_u8);
/*******************************************************************************
 *	Description: *//**brief This API is used to Trigger an interrupt
 *	when FIFO contains fifo_water_mark*4 bytes form the resister 0x46
 *
 *
 *
 *
 *  \param u8 v_fifo_wm_u8 : Value of the v_fifo_wm_u8
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
BMI160_RETURN_FUNCTION_TYPE bmi160_set_fifo_wm(
u8 v_fifo_wm_u8);
/****************************************************************************
 * Description: *//**brief This API Reads the v_fifo_stop_on_full_u8 in
 * FIFO_CONFIG_1
 * Stop writing samples into FIFO when FIFO is full
 * from the register 0x47 bit 0
 *
 *
 *
 *  \param u8 * v_fifo_stop_on_full_u8 :Pointer to the v_fifo_stop_on_full_u8
 *	v_fifo_stop_on_full_u8 ->
 *	0	-	do not stop writing to FIFO when full
 *	1	-	Stop writing into FIFO when full
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
u8 *v_fifo_stop_on_full_u8);
/*******************************************************************************
 * Description: *//**brief  This API writes the v_fifo_stop_on_full_u8 in
 * FIFO_CONFIG_1
 * Stop writing samples into FIFO when FIFO is full
 * from the register 0x47 bit 0
 *
 *  \param u8 v_fifo_stop_on_full_u8 : Value of the v_fifo_stop_on_full_u8
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
u8 v_fifo_stop_on_full_u8);
/******************************************************************************
 *	Description: *//**brief This API Reads fifo sensor time
 *	frame after the last valid data frame form the register  0x47 bit 1
 *
 *
 *
 *
 *  \param u8 * v_fifo_time_enable_u8 :Pointer to the v_fifo_time_enable_u8
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
u8 *v_fifo_time_enable_u8);
/*****************************************************************************
 *	Description: *//**brief This API writes sensortime
 *	frame after the last valid data frame form the register  0x47 bit 1
 *
 *
 *
 *
 *  \param u8 v_fifo_time_enable_u8 : The value of v_fifo_time_enable_u8
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
u8 v_fifo_time_enable_u8);
/*****************************************************************************
 *	Description: *//**brief This API Reads FIFO interrupt 2
 *	tag enable from the resister 0x47 bit 2
 *
 *  \param u8 * v_fifo_tag_intr2_u8 :
 *                 Pointer to the v_fifo_tag_intr2_u8
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
u8 *v_fifo_tag_intr2_u8);
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
u8 v_fifo_tag_intr2_u8);
/*****************************************************************************
 *	Description: *//**brief This API Reads FIFO interrupt 1
 *	tag enable from the resister 0x47 bit 3
 *
 *  \param u8 * v_fifo_tag_intr1_u8 :
 *                 Pointer to the v_fifo_tag_intr1_u8
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
u8 *v_fifo_tag_intr1_u8);
/*****************************************************************************
 * Description: *//**brief This API writes FIFO interrupt 1
 *	tag enable from the resister 0x47 bit 3
 *
 *  \param u8 * v_fifo_tag_intr1_u8 :
 *                 The value of v_fifo_tag_intr1_u8
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
u8 v_fifo_tag_intr1_u8);
/*****************************************************************************
 *	Description: *//**brief This API Reads FIFO frame
 *	header enable from the register 0x47 bit 4
 *
 *  \param u8 * v_fifo_header_u8 :
 *                 Pointer to the v_fifo_header_u8
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
u8 *v_fifo_header_u8);
/*****************************************************************************
 *	Description: *//**brief This API writes FIFO frame
 *	header enable from the register 0x47 bit 4
 *
 *
 *  \param u8 v_fifo_header_u8 :
 *               Value of the v_fifo_header_u8
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
u8 v_fifo_header_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to read stored
 *	magnetometer data in FIFO (all 3 axes) from the register 0x47 bit 5
 *
 *  \param u8 * v_fifo_mag_u8 :
 *                 Pointer to the v_fifo_mag_u8
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
u8 *v_fifo_mag_u8);
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
u8 v_fifo_mag_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to read stored
 *	accel data in FIFO (all 3 axes) from the register 0x47 bit 6
 *
 *
 *  \param u8 * v_fifo_accel_u8 :
 *                 Pointer to the v_fifo_accel_u8
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
u8 *v_fifo_accel_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to write stored
 *	accel data in FIFO (all 3 axes) from the register 0x47 bit 6
 *
 *
 *  \param u8 * v_fifo_accel_u8 :
 *                 The value of v_fifo_accel_u8
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
u8 v_fifo_accel_u8);
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
u8 *v_fifo_gyro_u8);
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
u8 v_fifo_gyro_u8);
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
u8 *v_i2c_device_addr_u8);
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
u8 v_i2c_device_addr_u8);
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
u8 *v_mag_burst_u8);
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
u8 v_mag_burst_u8);
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
u8 *v_mag_offset_u8);
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
u8 v_mag_offset_u8);
/*******************************************************************************
 *	Description: *//**brief This API is used to read
 *	Enable register access on MAG_IF[2] or MAG_IF[3] writes.
 *	This implies that the DATA registers are not updated with
 *	magnetometer values. Accessing magnetometer requires
 *	the magnetometer in normal mode in PMU_STATUS.
 *	from the register 0x4C bit 7
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
u8 *v_mag_manual_u8);
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
u8 v_mag_manual_u8);
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
u8 *v_mag_read_addr_u8);
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
u8 v_mag_read_addr_u8);
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
 * *
 * Remarks:
 *
 ****************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_mag_write_addr(
u8 *v_mag_write_addr_u8);
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
u8 v_mag_write_addr_u8);
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
u8 *v_mag_write_data_u8);
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
u8 v_mag_write_data_u8);
/*************************************************************************
 *	Description: *//**brief  This API is used to read
 *	interrupt enable from the register 0x50 bit 0 to 7
 *
 *
 *
 *
 *	\param u8 enable,u8 *v_intr_enable_zero_u8 :
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
u8 enable, u8 *v_intr_enable_zero_u8);
/***************************************************************************
 *	Description: *//**brief This API is used to write
 *	interrupt enable byte0 from the register 0x50 bit 0 to 7
 *
 *	\param u8 enable,u8 *v_intr_enable_zero_u8 :
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
u8 enable, u8 v_intr_enable_zero_u8);
/************************************************************************
 *	Description: *//**brief  This API is used to read
 *	interrupt enable byte1 from the register 0x51 bit 0 to 6
 *
 *
 *
 *
 *  \param u8 enable,u8 *v_intr_enable_1_u8 : The value of interrupt enable
 *	enable -->
 *	BMI160_HIGH_G_X_ENABLE       0
 *	BMI160_HIGH_G_Y_ENABLE       1
 *	BMI160_HIGH_G_Z_ENABLE       2
 *	BMI160_LOW_EN          3
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
u8 enable, u8 *v_intr_enable_1_u8);
/**************************************************************************
 *	Description: *//**brief This API is used to write
 *	interrupt enable byte1 from the register 0x51 bit 0 to 6
 *
 *
 *
 *  \param u8 enable,u8 *v_intr_enable_1_u8 : The value of interrupt enable
 *	enable -->
 *	BMI160_HIGH_G_X_ENABLE       0
 *	BMI160_HIGH_G_Y_ENABLE       1
 *	BMI160_HIGH_G_Z_ENABLE       2
 *	BMI160_LOW_EN          3
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
u8 enable, u8 v_intr_enable_1_u8);
/************************************************************************
 *	Description: *//**brief  This API is used to read
 *	interrupt enable byte2 from the register bit 0x52 bit 0 to 3
 *
 *
 *
 *
 *	\param u8 enable,u8 *v_intr_enable_2_u8 : The value of interrupt enable
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
u8 enable, u8 *v_intr_enable_2_u8);
/************************************************************************
 *	Description: *//**brief This API is used to read
 *	interrupt enable step detector interrupt from
 *	the register bit 0x52 bit 3
 *
 *
 *
 *
 *	\param u8 v_step_intr_u8 : Pointer holding the value of interrupt enable
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
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stepdetector_enable(
u8 *v_step_intr_u8);
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
BMI160_RETURN_FUNCTION_TYPE bmi160_set_stepdetector_enable(
u8 v_step_intr_u8);
/**************************************************************************
 *	Description: *//**brief This API is used to write
 *	interrupt enable byte2 from the register bit 0x52 bit 0 to 3
 *
 *
 *
 *
 *	\param u8 enable,u8 *v_intr_enable_2_u8 :
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
u8 enable, u8 v_intr_enable_2_u8);
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
u8 v_channel_u8, u8 *v_intr_edge_ctrl_u8);
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
u8 v_channel_u8, u8 v_intr_edge_ctrl_u8);
/**************************************************************************
 *	Description: *//**brief  Configure trigger condition of INT1
 *	and INT2 pin form the register 0x53 bit 1
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
u8 v_channel_u8, u8 *v_intr_level_u8);
/**************************************************************************
 *	Description: *//**brief  Configure set the trigger condition of INT1
 *	and INT2 pin form the register 0x53 bit 1 and 5
 *
 *  \param u8 v_channel_u8,u8  v_intr_level_u8 :
 *	The value of interrupt level
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
u8 v_channel_u8, u8 v_intr_level_u8);
/*************************************************************************
 *	Description: *//**brief  Configure trigger condition of INT1
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
u8 v_channel_u8, u8 *v_intr_output_type_u8);
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
u8 v_channel_u8, u8 v_intr_output_type_u8);
/**************************************************************************
 *	Description: *//**brief Output enable for INT1
 *	and INT2 pin from the register 0x53 bit 3 and 7
 *
 *
 *  \param u8 v_channel_u8,u8 *v_output_enable_u8 : The value of output enable
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
u8 v_channel_u8, u8 *v_output_enable_u8);
/**************************************************************************
 *	Description: *//**brief Output enable for INT1
 *	and INT2 pin from the register 0x53 bit 3 and 7
 *
 *
 *  \param u8 v_channel_u8,u8 *v_output_enable_u8 : The value of output enable
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
u8 v_channel_u8, u8 v_output_enable_u8);
/************************************************************************
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
u8 *v_latch_intr_u8);
/***************************************************************************
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
BMI160_RETURN_FUNCTION_TYPE bmi160_set_latch_intr(
u8 v_latch_intr_u8);
/**************************************************************************
 *	Description: *//**brief input enable for INT1
 *	and INT2 pin from the register 0x54 bit 4 and 5
 *
 *
 *
 *  \param u8 v_channel_u8,u8 *v_input_en_u8 :
 *	The value of interrupt enable
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
u8 v_channel_u8, u8 *v_input_en_u8);
/**************************************************************************
 *	Description: *//**brief input enable for INT1
 *	and INT2 pin from the register 0x54 bit 4 and 5
 *
 *
 *
 *  \param u8 v_channel_u8,u8 v_input_en_u8 :
 *	The value of interrupt enable
 *	channel -->
 *	BMI160_INTR1_INPUT_EN         0
 *	BMI160_INTR2_INPUT_ENABLE         1
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
u8 v_channel_u8, u8 v_input_en_u8);
/**************************************************************************
 *	Description: *//**brief Reads the Low g interrupt mapped to INT1
 *	and INT2 from the register 0x55 and 0x57
 *	INT1 bit 0 in the register 0x55
 *	INT2 bit 0 in the register 0x57
 *
 *  \param u8 v_channel_u8,u8 *v_intr_low_g_u8 :
 *	The value of lowg enable
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
u8 v_channel_u8, u8 *v_intr_low_g_u8);
/************************************************************************
 *	Description: *//**brief Write the Low g interrupt mapped to INT1
 *	and INT2 from the register 0x55 and 0x57
 *	INT1 bit 0 in the register 0x55
 *	INT2 bit 0 in the register 0x57
 *
 *  \param u8 v_channel_u8,u8 v_intr_low_g_u8 :
 *	The value of lowg enable
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
u8 v_channel_u8, u8 v_intr_low_g_u8);
/**************************************************************************
 *	Description: *//**brief Reads the HIGH g interrupt mapped to INT1
 *	and INT2 from the register 0x55 and 0x57
 *	INT1 bit 1 in the register 0x55
 *	INT2 bit 1 in the register 0x57
 *
 *  \param u8 v_channel_u8,u8 v_intr_high_g_u8 :
 *	The value of highg enable
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
u8 v_channel_u8, u8 *v_intr_high_g_u8);
/****************************************************************************
 *	Description: *//**brief Write the HIGH g interrupt mapped to INT1
 *	and INT2 from the register 0x55 and 0x57
 *	INT1 bit 1 in the register 0x55
 *	INT2 bit 1 in the register 0x57
 *
 *  \param u8 v_channel_u8,u8 v_intr_high_g_u8 :
 *	The value of highg enable
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
u8 v_channel_u8, u8 v_intr_high_g_u8);
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
u8 v_channel_u8, u8 *v_intr_any_motion_u8);
/****************************************************************************
 *	Description: *//**brief Write the Any motion interrupt
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
u8 v_channel_u8, u8 v_intr_any_motion_u8);
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
u8 v_channel_u8, u8 *v_intr_nomotion_u8);
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
u8 v_channel_u8, u8 v_intr_nomotion_u8);
/****************************************************************************
 *	Description: *//**brief Reads the Double Tap interrupt
 *	interrupt mapped to INT1
 *	and INT2 from the register 0x55 and 0x57
 *	INT1 bit 3 in the register 0x55
 *	INT2 bit 3 in the register 0x57
 *
 *  \param u8 v_channel_u8,u8 v_intr_double_tap_u8 :
 *	The value of any motion interrupt enable
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
u8 v_channel_u8, u8 *v_intr_double_tap_u8);
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
u8 v_channel_u8, u8 v_intr_double_tap_u8);
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
u8 v_channel_u8, u8 *v_intr_single_tap_u8);
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
u8 v_channel_u8, u8 v_intr_single_tap_u8);
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
u8 v_channel_u8, u8 *v_ntr_orient_u8);
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
u8 v_channel_u8, u8 v_ntr_orient_u8);
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
u8 v_channel_u8, u8 *v_intr_flat_u8);
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
u8 v_channel_u8, u8 v_intr_flat_u8);
/****************************************************************************
 *	Description: *//**brief Reads PMU trigger interrupt mapped to INT1
 *	and INT2 form the register 0x56 bit 0 and 4
 *
 *
 *
 *
 *  \param u8 v_channel_u8,u8 *v_intr_pmu_trig_u8 : The value of PMU trigger
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
u8 v_channel_u8, u8 *v_intr_pmu_trig_u8);
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
u8 v_channel_u8, u8 v_intr_pmu_trig_u8);
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
u8 v_channel_u8, u8 *v_intr_fifo_full_u8);
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
u8 v_channel_u8, u8 v_intr_fifo_full_u8);
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
u8 v_channel_u8, u8 *v_intr_fifo_wm_u8);
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
u8 v_channel_u8, u8 v_intr_fifo_wm_u8);
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
u8 v_channel_u8, u8 *v_intr_data_rdy_u8);
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
u8 v_channel_u8, u8 v_intr_data_rdy_u8);
/***************************************************************************
 *	Description: *//**brief This API Reads Data source for the interrupt
 *	engine for the single and double tap interrupts from the register
 *	0x58 bit 3
 *
 *
 *  \param u8 * v_tap_source_u8 :
 *	Pointer holding the value of the v_tap_source_u8
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
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_tap_source(
u8 *v_tap_source_u8);
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
u8 v_tap_source_u8);
/***************************************************************************
 *	Description: *//**brief This API Reads Data source for the
 *	interrupt engine for the low and high g interrupts
 *	from the register 0x58 bit 7
 *
 *  \param u8 * v_low_high_source_u8 :
 *	Pointer holding the value of v_low_high_source_u8
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
u8 *v_low_high_source_u8);
/***************************************************************************
 *	Description: *//**brief This API Write Data source for the
 *	interrupt engine for the low and high g interrupts
 *	from the register 0x58 bit 7
 *
 *  \param u8 * v_low_high_source_u8 :
 *	Pointer holding the value of v_low_high_source_u8
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
u8 v_low_high_source_u8);
/*******************************************************************************
 *	Description: *//**brief This API Reads Data source for the
 *	interrupt engine for the nomotion and anymotion interrupts
 *	from the register 0x59 bit 7
 *
 *  \param u8 * v_motion_source_u8 :
 *	Pointer holding the value of v_motion_source_u8
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
u8 *v_motion_source_u8);
/*******************************************************************************
 *	Description: *//**brief This API Write Data source for the
 *	interrupt engine for the nomotion and anymotion interrupts
 *	from the register 0x59 bit 7
 *
 *  \param u8 * v_motion_source_u8 :
 *	Pointer holding the value of v_motion_source_u8
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
u8 v_motion_source_u8);
/*******************************************************************************
 *	Description: *//**brief This API is used to read Delay
 *	time definition for the low-g interrupt from the register 0x5A bit 0 to 7
 *
 *
 *
 *
 *  \param u8 *v_low_durn_u8 : Pointer holding the value v_low_durn_u8
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
u8 *v_low_durn_u8);
/*******************************************************************************
 *	Description: *//**brief This API is used to write Delay
 *	time definition for the low-g interrupt from the register 0x5A bit 0 to 7
 *
 *
 *
 *
 *  \param u8 v_low_durn_u8 : the value of v_low_durn_u8
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
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_low_g_durn(
u8 v_low_durn_u8);
/*******************************************************************************
 *	Description: *//**brief This API is used to read Threshold
 *	definition for the low-g interrupt from the register 0x5B bit 0 to 7
 *
 *
 *
 *
 *  \param u8 *v_low_g_thres_u8 : Pointer holding the value of v_low_g_thres_u8
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
u8 *v_low_g_thres_u8);
/*******************************************************************************
 *	Description: *//**brief This API is used to write Threshold
 *	definition for the low-g interrupt from the register 0x5B bit 0 to 7
 *
 *
 *
 *
 *  \param u8 *v_low_g_thres_u8 :  the value of v_low_g_thres_u8
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
u8 v_low_g_thres_u8);
/*******************************************************************************
 *	Description: *//**brief This API Reads Low-g interrupt hysteresis;
 *	according to int_low_hy*125 mg, irrespective of the selected g-v_range_u8.
 *	from the register 0x5c bit 0 to 1
 *
 *  \param u8 * v_low_hyst_u8 : Pointer holding the value of v_low_hyst_u8
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
u8 *v_low_hyst_u8);
/*******************************************************************************
 *	Description: *//**brief This API write Low-g interrupt hysteresis;
 *	according to int_low_hy*125 mg, irrespective of the selected g-v_range_u8.
 *	from the register 0x5c bit 0 to 1
 *
 *  \param u8 * v_low_hyst_u8 : Pointer holding the value of v_low_hyst_u8
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
u8 v_low_hyst_u8);
/*******************************************************************************
 *	Description: *//**brief This API Reads Low-g interrupt mode:
 *	from the register 0x5C bit 2
 *
 *  \param u8 * v_low_g_mode_u8 : Pointer holding the value of v_low_g_mode_u8
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
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_low_g_mode(
u8 *v_low_g_mode_u8);
/*******************************************************************************
 *	Description: *//**brief This API write Low-g interrupt mode:
 *	from the register 0x5C bit 2
 *
 *  \param u8 v_low_g_mode_u8 : the value of v_low_g_mode_u8
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
u8 v_low_g_mode_u8);
/*******************************************************************************
 *	Description: *//**brief This API Reads High-g interrupt hysteresis;
 *	according to int_high_hy*125 mg (2 g v_range_u8)
 *	int_high_hy*250 mg (4 g v_range_u8)
 *	int_high_hy*500 mg (8 g v_range_u8)
 *	int_high_hy*1000 mg (16 g v_range_u8)
 *	from the register 0x5C bit 6 and 7
 *
 *  \param u8 * v_high_g_hyst_u8 : Pointer holding the value of v_high_g_hyst_u8
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
u8 *v_high_g_hyst_u8);
/*******************************************************************************
 *	Description: *//**brief This API Write High-g interrupt hysteresis;
 *	according to int_high_hy*125 mg (2 g v_range_u8)
 *	int_high_hy*250 mg (4 g v_range_u8)
 *	int_high_hy*500 mg (8 g v_range_u8)
 *	int_high_hy*1000 mg (16 g v_range_u8)
 *	from the register 0x5C bit 6 and 7
 *
 *  \param u8 * v_high_g_hyst_u8 : The value of v_high_g_hyst_u8
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
u8 v_high_g_hyst_u8);
/*******************************************************************************
 *	Description: *//**brief This API is used to read Delay
 *	time definition for the high-g interrupt from the register
 *	0x5D bit 0 to 7
 *
 *
 *
 *  \param u8 v_high_g_durn_u8 : Pointer holding the value of v_high_g_durn_u8
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
u8 *v_high_g_durn_u8);
/*******************************************************************************
 *	Description: *//**brief This API is used to write Delay
 *	time definition for the high-g interrupt from the register
 *	0x5D bit 0 to 7
 *
 *
 *
 *  \param u8 v_high_g_durn_u8 :  The value of v_high_g_durn_u8
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
u8 v_high_g_durn_u8);
/*******************************************************************************
 *	Description: *//**brief This API is used to read Threshold
 *	definition for the high-g interrupt from the register 0x5E 0 to 7
 *
 *
 *
 *
 *  \param u8 v_high_g_thres_u8 : Pointer holding the value of v_high_g_thres_u8
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
u8 *v_high_g_thres_u8);
/*******************************************************************************
 *	Description: *//**brief This API is used to write Threshold
 *	definition for the high-g interrupt from the register 0x5E 0 to 7
 *
 *
 *
 *
 *  \param u8 v_high_g_thres_u8 : The value of v_high_g_thres_u8
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
u8 v_high_g_thres_u8);
/******************************************************************************
 *	Description: *//**brief This API Reads no-motion duration
 *	from the register 0x5F bit 0 and 1
 *
 *  \param u8 * v_any_motion_durn_u8 :  Pointer holding the value of anymotion
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
u8 *v_any_motion_durn_u8);
/*******************************************************************************
 *	Description: *//**brief This API write no-motion duration
 *	from the register 0x5F bit 0 and 1
 *
 *  \param u8 * v_any_motion_durn_u8 :  The value of anymotion
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
u8 nomotion);
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
u8 *v_slow_no_motion_u8);
/*******************************************************************************
 *	Description: *//**brief This API Write Slow/no-motion
 *	interrupt trigger delay duration from the register 0x5F bit 0 to 7
 *
 *  \param u8 * v_slow_no_motion_u8 : The value of v_slow_no_motion_u8
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
u8 v_slow_no_motion_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to read Threshold
 *	definition for the any-motion interrupt from the register 0x60 bit
 *	0 to 7
 *
 *
 *  \param u8 v_any_motion_thres_u8 : Pointer holding
 *		the value of v_any_motion_thres_u8
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
u8 *v_any_motion_thres_u8);
/******************************************************************************
 *	Description: *//**brief This API is used to write Threshold
 *	definition for the any-motion interrupt from the register 0x60 bit
 *	0 to 7
 *
 *
 *  \param u8 v_any_motion_thres_u8 : The value of v_any_motion_thres_u8
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
u8 v_any_motion_thres_u8);
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
u8 *v_slow_no_motion_thres_u8);
/*******************************************************************************
 *	Description: *//**brief This API is used to write Threshold
 *	definition for the slow/no-motion interrupt
 *	from the register 0x61 bit 0 to 7
 *
 *
 *
 *
 *  \param u8 v_slow_no_motion_thres_u8 : The value of v_slow_no_motion_thres_u8
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
u8 v_slow_no_motion_thres_u8);
/*******************************************************************************
 *	Description: *//**brief This API is used to select
 *	the slow/no-motion interrupt from the register 0x62 bit 1
 *
 *
 *
 *
 *  \param u8 v_intr_slow_no_motion_select_u8 : Pointer holding
 *		the value of v_intr_slow_no_motion_select_u8
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
u8 *v_intr_slow_no_motion_select_u8);
/*******************************************************************************
 *	Description: *//**brief This API is used to write
 *	the slow/no-motion interrupt from the register 0x62 bit 1
 *
 *
 *
 *
 *  \param u8 v_intr_slow_no_motion_select_u8 :
 *	The value of v_intr_slow_no_motion_select_u8
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
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_slow_no_motion_select(
u8 v_intr_slow_no_motion_select_u8);
/*******************************************************************************
 *	Description: *//**brief This API is used to select
 *	the significant/ any motion  interrupt from the register 0x62 bit 1
 *
 *
 *
 *
 *  \param u8 int_sig_mot_sel : Pointer holding
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
u8 *int_sig_mot_sel);
/*******************************************************************************
 *	Description: *//**brief This API is used to select
 *	the significant/ any motion  interrupt from the register 0x62 bit 1
 *
 *
 *
 *
 *  \param u8 int_sig_mot_sel :
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
u8 int_sig_mot_sel);
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
u8 *v_int_sig_mot_skip_u8);
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
u8 v_int_sig_mot_skip_u8);
/*******************************************************************************
 *	Description: *//**brief This API is used to select
 *	the significant/ any motion proof time from the register 0x62 bit  4 and 5
 *
 *
 *
 *
 *  \param u8 int_sig_mot_proof : Pointer holding
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
u8 *int_sig_mot_proof);
/*******************************************************************************
 *	Description: *//**brief This API is used to select
 *	the significant/ any motion proof time from the register 0x62 bit  4 and 5
 *
 *
 *
 *
 *  \param u8 int_sig_mot_proof : The value of int_sig_mot_proof
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
u8 int_sig_mot_proof);
/*******************************************************************************
 *	Description: *//**brief This API is used to get the tap duration
 *	from the register 0x63 bit 0 to 2
 *
 *
 *
 *  \param u8 *v_tap_durn_u8 : Pointer holding the value of v_tap_durn_u8
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
u8 *v_tap_durn_u8);
/*******************************************************************************
 *	Description: *//**brief This API is used to set the tap duration
 *	from the register 0x63 bit 0 to 2
 *
 *
 *
 *  \param u8 *v_tap_durn_u8 :  the value of v_tap_durn_u8
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
u8 v_tap_durn_u8);
/*******************************************************************************
 *	Description: *//**brief This API Reads the
 *	tap shock duration from the register 0x63 bit 2
 *
 *  \param u8 * v_tap_shock_u8 : Pointer holding
 *	the value of v_tap_shock_u8
 *	Value	Description
 *	0		50 ms
 *	1		75 ms
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
u8 *v_tap_shock_u8);
/*******************************************************************************
 *	Description: *//**brief This API Write the
 *	tap shock duration from the register 0x63 bit 2
 *
 *  \param u8 * v_tap_shock_u8 : Pointer holding
 *	the value of v_tap_shock_u8
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
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_tap_shock(
u8 v_tap_shock_u8);
/*******************************************************************************
 *	Description: *//**brief This API Reads Selects
 *	tap quiet duration from the register 0x63 bit 7
 *
 *
 *  \param u8 * v_tap_quiet_u8 : Pointer holding the value of v_tap_quiet_u8
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
u8 *v_tap_quiet_u8);
/**************************************************************************
 *	Description: *//**brief This API Write Selects
 *	tap quiet duration from the register 0x63 bit 7
 *
 *
 *  \param u8  v_tap_quiet_u8 : The value of v_tap_quiet_u8
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
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_tap_quiet(
u8 v_tap_quiet_u8);
/*******************************************************************************
 *	Description: *//**brief This API Reads Threshold of the
 *	single/double tap interrupt corresponding to
 *	int_tap_th - 62.5 mg (2 g v_range_u8)
 *	int_tap_th - 125 mg (4 g v_range_u8)
 *	int_tap_th - 250 mg (8 g v_range_u8)
 *	int_tap_th - 500 mg (16 g v_range_u8)
 *	from the register 0x64 bit 0 to 4
 *
 *	\param u8 * v_tap_thres_u8 : Pointer holding the value of v_tap_thres_u8
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
u8 *v_tap_thres_u8);
/*******************************************************************************
 *	Description: *//**brief This API write Threshold of the
 *	single/double tap interrupt corresponding to
 *	int_tap_th - 62.5 mg (2 g v_range_u8)
 *	int_tap_th - 125 mg (4 g v_range_u8)
 *	int_tap_th - 250 mg (8 g v_range_u8)
 *	int_tap_th - 500 mg (16 g v_range_u8)
 *	from the register 0x64 bit 0 to 4
 *
 *	\param u8 v_tap_thres_u8 : The value of v_tap_thres_u8
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
u8 v_tap_thres_u8);
/*******************************************************************************
 *	Description: *//**brief This API Reads the threshold for
 *	switching between different orientations.
 *	from the register 0x65 bit 0 and 1
 *
 *  \param u8 * v_orient_mode_u8 : Pointer holding the value of v_orient_mode_u8
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
u8 *v_orient_mode_u8);
/*******************************************************************************
 *	Description: *//**brief This API write the threshold for
 *	switching between different orientations.
 *	from the register 0x65 bit 0 and 1
 *
 *  \param u8  v_orient_mode_u8 : the value of v_orient_mode_u8
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
u8 v_orient_mode_u8);
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
u8 *v_orient_blocking_u8);
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
u8 v_orient_blocking_u8);
/*******************************************************************************
 *	Description: *//**brief This API Reads Orient interrupt
 *	hysteresis; 1 LSB corresponds to 62.5 mg,
 *	irrespective of the selected g-v_range_u8.
 *	from the register 0x64 bit 4 to 7
 *
 *  \param u8 * v_orient_hyst_u8 : Pointer holding
 *	the value of v_orient_hyst_u8
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
u8 *v_orient_hyst_u8);
/*******************************************************************************
 *	Description: *//**brief This API write Orient interrupt
 *	hysteresis; 1 LSB corresponds to 62.5 mg,
 *	irrespective of the selected g-v_range_u8.
 *	from the register 0x64 bit 4 to 7
 *
 *  \param u8 * v_orient_hyst_u8 : The value of v_orient_hyst_u8
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
u8 v_orient_hyst_u8);
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
u8 *v_orient_theta_u8);
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
u8 v_orient_theta_u8);
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
u8 *v_orient_ud_u8);
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
u8 v_orient_ud_u8);
/*******************************************************************************
 *	Description: *//**brief This API Reads Change
 *	of up/down bit
 *	exchange x- and z-axis in algorithm, i.e x or z is relevant axis for
 *	upward/downward looking recognition (0=z, 1=x)
 *	from the register 0x66 bit 7
 *
 *  \param u8 * v_orient_axes_u8 : Pointer holding
 *	the value of v_orient_axes_u8
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
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_orient_axes_enable(
u8 *v_orient_axes_u8);
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
u8 v_orient_axes_u8);
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
u8 *v_flat_theta_u8);
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
u8 v_flat_theta_u8);
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
u8 *v_flat_hold_u8);
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
u8 v_flat_hold_u8);
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
u8 *v_flat_hyst_u8);
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
u8 v_flat_hyst_u8);
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
BMI160_RETURN_FUNCTION_TYPE bmi160_get_foc_accel_z(
u8 *v_foc_accel_z_u8);
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
u8 v_foc_accel_z_u8);
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
BMI160_RETURN_FUNCTION_TYPE bmi160_get_foc_accel_y(
u8 *v_foc_accel_y_u8);
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
BMI160_RETURN_FUNCTION_TYPE bmi160_set_foc_accel_y(
u8 v_foc_accel_y_u8);
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
BMI160_RETURN_FUNCTION_TYPE bmi160_get_foc_accel_x(
u8 *v_foc_accel_x_u8);
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
BMI160_RETURN_FUNCTION_TYPE bmi160_set_foc_accel_x(
u8 v_foc_accel_x_u8);
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
s16 *v_gyro_off_y_s16, s16 *v_gyro_off_z_s16);
/*******************************************************************************
 * Description: *//**brief This API Reads Enable
 * NVM programming form the register 0x6A bit 1
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
u8 *v_nvm_prog_u8);
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
u8 v_nvm_prog_u8);
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
u8 *v_spi3_u8);
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
u8 v_spi3_u8);
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
u8 *v_foc_gyro_u8);
/*******************************************************************************
 * Description: *//**brief This API Reads Select timer
 * period for I2C Watchdog from the register 0x70 bit 1
 *            Value     Description
 *              0       I2C watchdog timeout after 1 ms
 *              1       I2C watchdog timeout after 50 ms
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
u8 *v_i2c_wdt_u8);
/*******************************************************************************
 * Description: *//**brief This API Writes  Select timer
 * period for I2C Watchdog from the register 0x70 bit 1
 *            Value     Description
 *              0       I2C watchdog timeout after 1 ms
 *              1       I2C watchdog timeout after 50 ms
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
BMI160_RETURN_FUNCTION_TYPE
bmi160_set_i2c_wdt_select(u8 v_i2c_wdt_u8);
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
u8 *v_i2c_wdt_u8);
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
u8 v_i2c_wdt_u8);
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
u8 *v_if_mode_u8);
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
u8 v_if_mode_u8);
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
u8 *v_gyro_sleep_trigger_u8);
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
u8 v_gyro_sleep_trigger_u8);
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
u8 *v_gyro_wakeup_trigger_u8);
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
u8 v_gyro_wakeup_trigger_u8);
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
u8 *v_gyro_sleep_state_u8);
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
u8 v_gyro_sleep_state_u8);
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
u8 *v_gyro_wakeup_intr_u8);
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
u8 v_gyro_wakeup_intr_u8);
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
 *  \param u8 * acc_selftest_axis :
 *      Pointer to the acc_selftest_axis
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
u8 *acc_selftest_axis);
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
 *  \param u8 acc_selftest_axis :
 *    Value of the acc_selftest_axis
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
u8 acc_selftest_axis);
/*******************************************************************************
 * Description: *//**brief This API Reads
 *          select sign of self-test excitation as
 *            Value     Description
 *              0       negative
 *              1       positive
 *		from the register 0x6D bit 2
 *
 *  \param u8 * acc_selftest_sign :
 *      Pointer to the acc_selftest_sign
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
u8 *acc_selftest_sign);
/*******************************************************************************
 * Description: *//**brief This API Writes
 *          select sign of self-test excitation as
 *            Value     Description
 *              0       negative
 *              1       positive
 *		from the register 0x6D bit 2
 *
 *  \param u8 acc_selftest_sign :
 *    Value of the acc_selftest_sign
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
u8 acc_selftest_sign);
/*******************************************************************************
 * Description: *//**brief This API Reads
 *        select amplitude of the selftest deflection:
 *                    Value     Description
 *                           0  low
 *                           1  high
 *		from the register 0x6D bit 3
 *
 *  \param u8 * acc_selftest_amp :
 *      Pointer to the acc_selftest_amp
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
u8 *acc_selftest_amp);
/*******************************************************************************
 * Description: *//**brief This API Writes
  *        select amplitude of the selftest deflection:
 *                    Value     Description
 *                           0  low
 *                           1  high
 *		from the register 0x6D bit 3
 *
 *
 *  \param u8 acc_selftest_amp :
 *    Value of the acc_selftest_amp
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
u8 acc_selftest_amp);
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
u8 *v_gyro_selftest_start_u8);
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
u8 v_gyro_selftest_start_u8);
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

BMI160_RETURN_FUNCTION_TYPE bmi160_get_spi_enable(
u8 *v_spi_enable_u8);
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
BMI160_RETURN_FUNCTION_TYPE bmi160_set_spi_enable(
u8 v_spi_enable_u8);
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
BMI160_RETURN_FUNCTION_TYPE bmi160_get_spare0_trim
(u8 *v_spare0_trim_u8);
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
BMI160_RETURN_FUNCTION_TYPE bmi160_set_spare0_trim
(u8 v_spare0_trim_u8);
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
BMI160_RETURN_FUNCTION_TYPE bmi160_get_nvm_counter(
u8 *v_nvm_counter_u8);
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
u8 v_nvm_counter_u8);
/******************************************************************************
 *	Description: *//**brief This API Reads
 *	acc_off_x from the register 0x71 bit 0 to 7
 *
 *
 *
 *  \param s8 * acc_off_x:
 *      Pointer to the acc_off_x
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
s8 *acc_off_x);
/*******************************************************************************
 *	Description: *//**brief This API Writes
 *	acc_off_x from the register 0x71 bit 0 to 7
 *
 *  \param s8 acc_off_x :
 *    Value of the acc_off_x
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
s8 acc_off_x);
/******************************************************************************
 *	Description: *//**brief This API Reads
 *	acc_off_y from the register 0x72 bit 0 to 7
 *
 *
 *
 *  \param s8 * acc_off_y:
 *      Pointer to the acc_off_y
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
s8 *acc_off_x);
/*******************************************************************************
 *	Description: *//**brief This API Writes
 *	acc_off_y from the register 0x72 bit 0 to 7
 *
 *  \param s8 acc_off_y :
 *    Value of the acc_off_y
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
s8 acc_off_y);
/******************************************************************************
 *	Description: *//**brief This API Reads
 *	acc_off_z from the register 0x73 bit 0 to 7
 *
 *
 *
 *  \param s8 * acc_off_z:
 *      Pointer to the acc_off_z
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
s8 *acc_off_z);
/*******************************************************************************
 *	Description: *//**brief This API Writes
 *	acc_off_z from the register 0x73 bit 0 to 7
 *
 *  \param s8 acc_off_z :
 *    Value of the acc_off_z
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
s8 acc_off_z);
/******************************************************************************
 *	Description: *//**brief This API Reads
 *	v_gyro_off_x_s16 from the register 0x74 bit 0 to 7
 *
 *
 *
 *
 *  \param s16 * v_gyro_off_x_s16:
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
s16 *v_gyro_off_x_s16);
/*******************************************************************************
 *	Description: *//**brief This API Writes
 *	v_gyro_off_x_s16 from the register 0x74 bit 0 to 7
 *
 *  \param s16 v_gyro_off_x_s16 :
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
s16 v_gyro_off_x_s16);
/******************************************************************************
 *	Description: *//**brief This API Reads
 *	v_gyro_off_y_s16 from the register 0x75 bit 0 to 7
 *
 *
 *
 *  \param s16 * v_gyro_off_y_s16:
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
s16 *v_gyro_off_y_s16);
/*******************************************************************************
 *	Description: *//**brief This API Writes
 *	v_gyro_off_y_s16 from the register 0x75 bit 0 to 7
 *
 *  \param s16 v_gyro_off_y_s16 :
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
s16 v_gyro_off_y_s16);
/******************************************************************************
 *	Description: *//**brief This API Reads
 *	v_gyro_off_z_s16 from the register 0x76 bit 0 to 7
 *
 *
 *
 *  \param s16 * v_gyro_off_z_s16:
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
s16 *v_gyro_off_z_s16);
/*******************************************************************************
 *	Description: *//**brief This API Writes
 *	v_gyro_off_z_s16 from the register 0x76 bit 0 to 7
 *
 *  \param s16 v_gyro_off_z_s16 :
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
s16 v_gyro_off_z_s16);
/*******************************************************************************
 *	Description: *//**brief This API Writes fast offset compensation
 *	target value for foc_acc \
 *	from the register 0x69 bit 6
 *
 *  \param u8 v_foc_accel_z_u8:
 *    Value of the v_foc_accel_z_u8
 *	Value    Description
 *	0b00    disabled
 *	0b01    +1 g
 *	0b10    -1 g
 *	0b11    0 g
 *  \param u8 axis:
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
BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_foc_trigger(u8 axis,
u8 foc_acc, s8 *accel_offset);
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
u8 v_foc_accel_y_u8, u8 v_foc_accel_z_u8,
s8 *acc_off_x, s8 *acc_off_y, s8 *acc_off_z);
/******************************************************************************
 *	Description: *//**brief This API Reads
 *	cc_off_en from the register 0x77 bit 6
 *
 *
 *
 *  \param u8 * acc_off_en:
 *      Pointer to the acc_off_en
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
u8 *acc_off_en);
/*******************************************************************************
 * Description: *//**brief This API Writes
 *	cc_off_en from the register 0x77 bit 6
 *
 *  \param u8 acc_off_en :
 *    Value of the acc_off_en
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
u8 acc_off_en);
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
u8 *v_gyro_off_enable_u8);
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
u8 v_gyro_off_enable_u8);
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
BMI160_RETURN_FUNCTION_TYPE bmi160_read_step_count(s16 *v_step_cnt_s16);
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
u16 *v_step_config_u16);
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
BMI160_RETURN_FUNCTION_TYPE bmi160_set_step_configuration(
u16 v_step_config_u16);
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
u8 *v_step_counter_u8);
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
BMI160_RETURN_FUNCTION_TYPE bmi160_set_step_counter_enable(
u8 v_step_counter_u8);
/******************************************************************************
 *	Description: *//**brief This API get
 *	Step counter modes
 *
 *
 *  \param u8 v_step_mode_u8   :
 *      The value of step counter mode
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
BMI160_RETURN_FUNCTION_TYPE bmi160_set_step_mode(u8 v_step_mode_u8);
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
u8 v_significant_u8);
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
u8 v_step_detector_u8);
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
BMI160_RETURN_FUNCTION_TYPE bmi160_clear_step_counter(void);
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
 *  0x14	-	Sets the PMU mode for the Gyroscope to suspend
 *	0x15	-	Sets the PMU mode for the Gyroscope to normal
 *	0x16	-	Reserved
 *	0x17	-	Sets the PMU mode for the Gyroscope to fast start-up
 *  0x18	-	Sets the PMU mode for the Magnetometer to suspend
 *	0x19	-	Sets the PMU mode for the Magnetometer to normal
 *	0xB0	-	Clears all data in the FIFO
 *  0xB1	-	Resets the interrupt engine
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
BMI160_RETURN_FUNCTION_TYPE bmi160_set_command_register(
u8 v_command_reg_u8);
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
BMI160_RETURN_FUNCTION_TYPE bmi160_get_target_page(
u8 *v_target_page_u8);
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
BMI160_RETURN_FUNCTION_TYPE bmi160_set_target_page(
u8 v_target_page_u8);
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
BMI160_RETURN_FUNCTION_TYPE bmi160_get_paging_enable(
u8 *v_page_enable_u8);
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
u8 v_page_enable_u8);
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
u8 *v_control_pullup_u8);
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
u8 v_control_pullup_u8);
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
BMI160_RETURN_FUNCTION_TYPE bmi160_magnetometer_intrerface_init(void);
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
 *		set the value of 0x19(NORMAL mode) by using the
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
BMI160_RETURN_FUNCTION_TYPE bmi160_read_mag_trim(void);
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
 *		set the value of 0x19(NORMAL mode) by using the
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
struct bmi160_mag_xyz_s32_t *mag_comp_xyz);
/*****************************************************************************
 *	Description: *//**\brief This API used to get the compensated Z data
 *	the out put of Z as s32
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
 *		set the value of 0x19(NORMAL mode) by using the
 *		bmi160_set_command_register(0x19) function.
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
s32 bmi160_mag_compensate_X(s16 v_mag_data_x_s16, u16 v_data_r_u16);
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
s32 bmi160_mag_compensate_Y(s16 v_mag_data_y_s16, u16 v_data_r_u16);
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
s32 bmi160_mag_compensate_Z(s16 v_mag_data_z_s16, u16 v_data_r_u16);
/*****************************************************************************
 *	Description: *//**\brief This API used to set the pre-set modes
 *	The pre-set mode setting is depend on data rate, xy and z repetitions
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
 *		set the value of 0x19(NORMAL mode) by using the
 *		bmi160_set_command_register(0x19) function.
 *
 *
 *  \param u8 mode: The value of pre-set mode selection value
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
BMI160_RETURN_FUNCTION_TYPE bmi160_set_mag_presetmode(u8 mode);
/***************************************************************************
 *	Description: This function used for set the magnetometer
 *	power mode.
 *	Before set the mag power mode
 *	make sure the following point is addressed
 *	1.	Make sure the mag interface is enabled or not,
 *		by using the bmi160_get_if_mode() function.
 *		If mag interface is not enabled set the value of 0x02
 *		to the function bmi160_get_if_mode(0x02)
 *
 *	\param mag_pow_mode : The value of mag power mode
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
BMI160_RETURN_FUNCTION_TYPE bmi160_mag_set_power_mode(u8 mag_pow_mode);
/***************************************************************************
 *	Description: This function used for set the magnetometer
 *	power mode.
 *	Before set the mag power mode
 *	make sure the following point is addressed
 *	1.	Make sure the mag interface is enabled or not,
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
 *
 *************************************************************************/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_mag_and_secondary_if_power_mode(
u8 mag_sec_if_pow_mode);

/***************************************************************************
 *	Description: This function used for initialize the AKM sensor
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
BMI160_RETURN_FUNCTION_TYPE bmi160_bst_akm_mag_intrerface_init(void);
/***************************************************************************
 *	Description: This function used for read the sensitivity data
 *	of AKM09911
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
BMI160_RETURN_FUNCTION_TYPE bmi160_read_bst_akm_sensitivity_data(void);
/*****************************************************************************
 *	Description: *//**\brief This API used to get the compensated Z data
 *	of AKM09911 the out put of X as s16
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
 *  \param s16 v_bst_akm_x_s16 : The value of X data
 *
 *	\return results of compensated X data value output as s16
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
s16 bmi160_bst_akm_compensate_X(s16 v_bst_akm_x_s16);
/*****************************************************************************
 *	Description: *//**\brief This API used to get the compensated Y data
 *	of AKM09911 the out put of Y as s16
 *	Before start reading the AKM compensated Y data
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
 *  \param s16 v_bst_akm_y_s16 : The value of Y data
 *
 *	\return results of compensated Y data value output as s16
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
s16 bmi160_bst_akm_compensate_Y(s16 v_bst_akm_y_s16);
/*****************************************************************************
 *	Description: *//**\brief This API used to get the compensated Z data
 *	of AKM09911 the out put of Z as s16
 *	Before start reading the AKM compensated Z data
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
 *  \param s16 v_bst_akm_z_s16 : The value of Z data
 *
 *	\return results of compensated Z data value output as s16
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
s16 bmi160_bst_akm_compensate_Z(s16 v_bst_akm_z_s16);
/***************************************************************************
 *	Description: This function used for read the compensated value
 *	of AKM09911
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
BMI160_RETURN_FUNCTION_TYPE bmi160_bst_akm_compensate_xyz(
struct bmi160_bst_akm_xyz_t *bst_akm_xyz);
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
BMI160_RETURN_FUNCTION_TYPE bmi160_bst_akm_set_powermode(u8 v_akm_pow_mode_u8);
/***************************************************************************
 *	Description: This function used for set the magnetometer
 *	power mode of AKM09911.
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
u8 mag_sec_if_pow_mode);
/***************************************************************************
 *	Description: This function used for read the FIFO data
 *	for header less mode
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
BMI160_RETURN_FUNCTION_TYPE bmi160_read_fifo_headerless_mode(
u32 v_fifo_length_u32);
 /***************************************************************************
 *	Description: This function used for read the FIFO data
 *	for header mode
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
BMI160_RETURN_FUNCTION_TYPE bmi160_read_fifo_header_data(u32 v_fifo_length_u32);
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
struct bmi160_t *bmi160_get_ptr(void);
#endif
