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

#include <stdint.h>

#ifndef __KERNEL_RESMGR_H_
#define __KERNEL_RESMGR_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup SH_RESMGR Sensor Hub: Resource manager interface
 * @ingroup SENSOR_HUB
 * @{
 */

typedef enum _RESMGR_MODES {
	RESMGR_STARTUP = 0,         /**< Startup mode */
	RESMGR_NORMAL_MODE,         /**< Normal mode (IRC 12Mhz) */
	RESMGR_PLL_MODE,            /**< PLL mode (84Mhz from PLL) */
	RESMGR_PWRDOWN_MODE,        /**< Power down mode */
} ResMgr_Mode_T;

/**
 * @brief Resource Manager Structure
 */
typedef struct _RESMGR_CTRL {
	volatile uint32_t fSkipSleep;        /**< Flag to enable sleep when processing is finished */
} ResMgr_Ctrl_T;

extern ResMgr_Ctrl_T g_ResMgr;  /**< Data structure which hold resource manager control data */


/**
 * @brief	Puts the system in power down mode
 * @return	none
 */
void ResMgr_EnterPowerDownMode(uint32_t estSleepTime);

/**
 * @brief	Sets the system to run off of IRC
 * @return	none
 */
void ResMgr_EnterNormalMode(void);

/**
 * @brief	Sets the system to run off of PLL
 * @return	none
 */
void ResMgr_EnterPLLMode(void);

/**
 * @brief	Estimate how long we can sleep
 * @return	none
 */
int32_t ResMgr_EstimateSleepTime(void);

/**
 * @brief	This routine should be called from all IRQs in system
 * @return	none
 */
static INLINE void ResMgr_IRQDone(void) {
	g_ResMgr.fSkipSleep++;
}

/**
 * @brief	Temporary Disable entering into deepsleep or powerdown mode
 */
__STATIC_INLINE void ResMgr_DisableSleep(void)
{
	uint32_t pmsk = __get_PRIMASK();
	__disable_irq();
	g_ResMgr.fSkipSleep++;
	__set_PRIMASK(pmsk);
}

/**
 * @brief	Enable entering deepsleep or powerdown modes
 */
__STATIC_INLINE void ResMgr_EnableSleep(void)
{
	uint32_t pmsk = __get_PRIMASK();
	__disable_irq();
	if (g_ResMgr.fSkipSleep)
		g_ResMgr.fSkipSleep--;
	__set_PRIMASK(pmsk);
}

/**
 * @brief	Returns 1 if sleep mode is OK 0 otherwise
 */
__STATIC_INLINE int32_t ResMgr_CanSleep(void)
{
	return !g_ResMgr.fSkipSleep;
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __KERNEL_RESMGR_H_ */
