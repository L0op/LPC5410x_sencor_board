/*
 * @brief Power configuration header
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

#ifndef __POWERCFG_H_
#define __POWERCFG_H_

/**
 * @ingroup SH_RESMGR
 * @{
 */

/** @brief POWER Mode configuration
 *
 * This macro will set the default power mode to use during sleeping
 * the possible values are #POWER_POWER_DOWN and #POWER_SLEEP
 *
 * #POWER_SLEEP makes the core enter normal sleep mode which might
 * consume more power than entering POWER_DOWN mode.
 *
 * #POWER_POWER_DOWN makes the core enter the powerdown mode that
 * will save more power than SLEEP mode.
 *
 * @note	When set to #POWER_POWER_DOWN the core will enter power
 * down mode, if a debug session is in progress it will be disconnected,
 * and the debugger might not be able to connect to the core until the
 * core is reset using ISP.
 */
#define POWER_MODE  POWER_SLEEP

/**
 * @}
 */
#endif /* __POWERCFG_H_ */
