/*
 * @brief Setup Heap for FreeRTOS
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
#include "chip.h"
#include "FreeRTOS.h"

#if defined(__CC_ARM)
#define HEAP_SIZE configTOTAL_HEAP_SIZE
static uint32_t heap_mem[HEAP_SIZE/sizeof(uint32_t)];

/* Heap allocation function for Keil Example */
unsigned __rt_heap_extend(unsigned size, void **block)
{
	*block = heap_mem;
	return HEAP_SIZE;
}

unsigned __user_heap_extend(int dummy, void **base, unsigned requested_size)
{
	*base = heap_mem;
	return HEAP_SIZE;
}
#endif
