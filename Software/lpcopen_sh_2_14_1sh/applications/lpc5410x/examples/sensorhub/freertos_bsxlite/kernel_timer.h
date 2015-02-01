/*
 * @brief Always on timer interface
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

#ifndef __KERNEL_TIMER_H_
#define __KERNEL_TIMER_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup KERNEL_TIMER Sensor Hub: Kernel timer interface
 * @ingroup SENSOR_HUB
 * @{
 */

/**
 * @brief Kernel Timer Structure
 */
typedef struct _SH_TIMER_CTRL {
	uint32_t sleep_threshold_us;	/*!< Sleep Threshold in microseconds */
	uint32_t pwrDown_threshold_us;	/*!< Power Down Threshold in microseconds */
	uint32_t bgProc_threshold_us;	/*!< Background Process Threshold in microseconds */

	void(*const init) (void);	/*!< Initialization function pointer */
	uint32_t(*const GetCurrent) (void);	/*!< Function pointer for reading current time */
	int32_t(*const diff) (uint32_t operand1, uint32_t operand2);	/*!< Function pointer for calculating time difference */
	uint32_t(*const add) (uint32_t operand1, uint32_t operand2);	/*!< Function pointer for calculating sum of timestamps */
	uint32_t(*const GetResolution) (void);	/*!< Function pointer to read timer resolution */
	void(*const delayMs) (uint32_t milliseconds);	/*!< Function pointer for delay in milliseconds */
	void(*const delayTicks) (uint32_t ticks);	/*!< Function pointer for delay in timer ticks */
	uint32_t(*const getTicksFromMs) (uint32_t milliseconds);	/*!< Converts a passed mS count to a kernel tick time */
	uint32_t(*const getMsFromTicks) (uint32_t ticks);	/*!< Converts a kernel tick time to a mS count */
	uint32_t(*const getUsFromTicks) (uint32_t ticks);	/*!< Converts a kernel tick time to a uS count */
	uint8_t(*const GetTimer40HiByte) (void);	/*!< Function pointer  to retrieve higher byte of the 40 bit watchdog count */
	uint32_t(*const GetCurrentMsec) (void);	/*!< Function pointer  to retrieve watchdog count in milliseconds */

} KernelTimer_Ctrl_t;

extern KernelTimer_Ctrl_t g_Timer; /**< Data structure which holds kernel timer function pointers structure */

/**
 * @brief	Initialize kernel sub-system
 * @return	none
 */
void Kernel_Init(void);

/**
 * @brief	Get current time
 * @return	Returns current 32 bit tick count
 */
static INLINE uint32_t Kernel_GetCurrentTime(void)
{
	return g_Timer.GetCurrent();
}

/**
 * @brief	Get the high byte of 40 bit timer count
 * @return	Returns high byte of 40 bit timer count
 */
static INLINE uint8_t Kernel_GetTimer40HiByte(void)
{
	return g_Timer.GetTimer40HiByte();
}

/**
 * @brief	Get kernel timer tick resolution
 * @return	Returns number of kernel tick in one second
 */
static INLINE uint32_t Kernel_GetResolution(void)
{
	return g_Timer.GetResolution();
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __KERNEL_TIMER_H_ */
