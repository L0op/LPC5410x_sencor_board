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

#ifndef _MAX44000_H_
#define _MAX44000_H_

#include <stdint.h>
#include "board.h"
#include "sensors.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 ***************************
 *Hardware Specific definition
 ******************************
*/

#define MAX44K_RD_ADDR	0x4A
#define MAX44K_WR_ADDR	0x4A

#define AMBIENT_BIT_RESOLUTION 10
#define SCL_MAX_Frequency	400000
#define LUX_BY_LSB	0.03125

/******************************************
 *Register Definition
 ********************************************
*/
#define MAX44K_INT_STATUS_REG	0x00
#define MAX44K_MAIN_CFG_REG	0x01
#define MAX44K_RCV_CFG_REG		0x02
#define	MAX44K_TRSMT_CFG_REG	0x03
#define MAX44K_ALS_DATA_HIGH_BYTE_REG	0x04
#define MAX44K_ALS_DATA_LOW_BYTE_REG	0x05
#define MAX44K_PROX_DATA_Byte_REG	0x16
#define	MAX44K_ALS_UPPER_THRHD_HB_REG	0x06
#define MAX44K_ALS_UPPER_THRHD_LB_REG	0x07
#define MAX44K_ALS_LOWER_THRHD_HB_REG	0x08
#define MAX44K_ALS_LOWER_THRHD_LB_REG	0x09
#define	MAX44K_THRHD_PES_TIMER_REG	0x0A
#define MAX44K_PROX_THRHD_INDC_REG	0x0B
#define MAX44K_PROX_THRHD_REG	0x0C
#define MAX44K_DIG_GAIN_TRIM_GREEN_REG	0x0F
#define MAX44K_DIG_GAIN_TRIM_INFRARED_REG	0x10

/*
 *	Main Configuration Register
*/
//Mode definition 
#define MAX44K_MODE_SHUTDOWN	0		//Analog circuits shutdown, digital register retains
#define MAX44K_MODE_ALS_GIR		1		//Standard ALS mode stores the difference between green and infrared channel readings. Proximity Disabled
#define MAX44K_MODE_ALS_G		2		//ALS green channel only. Proximity channel operation and updates are disabled
#define MAX44K_MODE_ALS_IR		3		//Infrared channel only. Proximity channel operation and updates are disabled
#define MAX44K_MODE_ALS_PROX	4		//ALS and PROX are interleaved continuously.
#define MAX44K_MODE_PROX		5		//PROX only continuously. ALS channel operation and updates are disabled

#define MAX44K_MAIN_CFG_REG_ALSINTE	(1<<0)		//ALS interrupt enable 
#define MAX44K_MAIN_CFG_REG_PRXINTE	(1<<1)		//Proximity interrupt enable 
#define MAX44K_MAIN_CFG_REG_MODE	(2)		//Operation mode
#define MAX44K_MAIN_CFG_REG_TRIM	(1<<5)		//TRIM_GAIN selection

/*
 *	Receive Configuration Register
*/
//Ambient ADC bit resolution and convertion time 
#define MAX44K_RCV_CFG_REG_ALSTIM_8bit	3
#define MAX44K_RCV_CFG_REG_ALSTIM_10bit	2
#define MAX44K_RCV_CFG_REG_ALSTIM_12bit	1
#define MAX44K_RCV_CFG_REG_ALSTIM_14bit	0

//Ambient Light Measurement Gain
#define MAX44K_RCV_CFG_REG_ALSPGA_GAIN0	0
#define MAX44K_RCV_CFG_REG_ALSPGA_GAIN1	1
#define MAX44K_RCV_CFG_REG_ALSPGA_GAIN2	2
#define MAX44K_RCV_CFG_REG_ALSPGA_GAIN3	3

#define MAX44K_RCV_CFG_REG_ALSPGA	(0)
#define MAX44K_RCV_CFG_REG_ALSTIM	(2)

/*
 *	Transmit Configuration Register
*/
#define	MAX44K_TRSMT_CFG_REG_DRV_Disable	0

#define	MAX44K_TRSMT_CFG_REG_DRV	(0)

/*
 *	ALS Data Register
*/
#define MAX44K_ALS_DATA_HIGH_BYTE_REG_OFL	(6<<1)



#define MAX44000_WR_FUNC_PTR \
	signed char (*bus_write)(unsigned char, unsigned char,	\
					  unsigned char *, unsigned char)

#define MAX44000_RD_FUNC_PTR \
	signed char (*bus_read)(unsigned char, unsigned char, \
					 unsigned char *, unsigned char)

typedef enum _OpMode_T
{
	MODE_SHUTDOWN = 0,
	MODE_ALS_GIR = 1,
	MODE_ALS_G = 2,
	MODE_ALS_IR =3,
	MODE_INTERLEAVE = 4,
	MODE_PROX = 5,
}OpMode_T;

typedef enum _ALSBitRes_T
{
	BIT_RES_14 = 0,
	BIT_RES_12 = 1,
	BIT_RES_10 = 2,
	BIT_RES_8 = 3,
}ALSBitRes_T;

typedef enum _ALSMeasGain_T
{
	MEAS_GAIN_1 = 0,
	MEAS_GAIN_4 = 1,
	MEAS_GAIN_16 = 2,
	MEAS_GAIN_128 = 3,
}ALSMeasGain_T;

typedef struct _Amb_Data_T_
{
	uint8_t highByte;
	uint8_t lowByte;
}Amb_Data_T;

typedef struct _Prox_Data_T
{
	uint8_t dist;
}Prox_Data_t;

typedef enum _LED_DRV_T
{
	LED_DRV_DISABLE = 0,
	LED_DRV_20 = 2,
	LED_DRV_50 = 5,
	LED_DRV_110 = 0xF,
}LED_DRV_T;

typedef enum _DIST_TRIG_LEV_T
{
	DIST_TRIG_
}DIST_TRIG_LEV_T;

typedef struct _MAX44000_T
{
	uint8_t devAddrRd;
	uint8_t devAddrWr;
	OpMode_T opMode;
	ALSBitRes_T alsBitResolution;
	ALSMeasGain_T alsMeasureGain;
	LED_DRV_T ledDrv;
	
	MAX44000_RD_FUNC_PTR;
	MAX44000_WR_FUNC_PTR;
	
}MAX44000_T;

#define PROX_LED_DRV	LED_DRV_50
#define PROX_DISTANCE_TRIG_LEV


static void Max44000_ProxInit(MAX44000_T* max44k);
static void Max44000_AmbInit(MAX44000_T* max44k);
static char Max44000_ReadProx(MAX44000_T* max44k,Prox_Data_t* proxData);
static char Max44000_Read_ALS(MAX44000_T* max44k,Amb_Data_T* ambData);
static void Max44000_ProxActivate(MAX44000_T* max44k, bool enable);
static void Max44000_AmbActivate(MAX44000_T* max44k, bool enable);


static int32_t Prox_Init(PhysicalSensor_t* pSens);
static int32_t Prox_Read(PhysicalSensor_t* pSens);
static int32_t Prox_Activate(PhysicalSensor_t* pSens, bool enable);
static int32_t Prox_SetDelay(PhysicalSensor_t* pSens, uint32_t mSec);

static int32_t Ambient_Init(PhysicalSensor_t* pSens);
static int32_t Ambient_Read(PhysicalSensor_t* pSens);
static int32_t Ambient_Activate(PhysicalSensor_t* pSens, bool enable);
static int32_t Ambient_SetDelay(PhysicalSensor_t* pSens, uint32_t mSec);

extern PhysicalSensor_t g_prox;
extern PhysicalSensor_t g_amb;




#ifdef __cplusplus
}
#endif

#endif

