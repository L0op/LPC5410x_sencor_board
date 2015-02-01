/*
 * @brief DMA router for Niobe Sensor app
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

#include "board.h"
#include "sensorhub.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#ifdef __ICCARM__
#define ALIGNSTR(x) #x
#define ALIGN(x) _Pragma(ALIGNSTR(data_alignment=##x))
#else
#define ALIGN(x) __attribute__((aligned(x)))
#endif

/* Keil alignement to 512 bytes */
ALIGN(512) ROM_DMA_DESC_T EXChip_DMA_Table[MAX_DMA_CHANNEL];

static uint32_t dmaDrvData[16];


/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

ROM_DMA_HANDLE_T g_dmaHandle;

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/
 
/* DMA controller interrupt handler */
void DMA_IRQHandler(void)
{
	ROM_DMA_DMAHandler(g_dmaHandle);
}

void DMA_Init(void)
{
	uint32_t memSize, *devMem;
	ROM_DMA_INIT_T dmaInit;
	/* Enable DMA clocking prior to calling DMA init functions */
	Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_DMA);

	/* Get needed size for DMA driver context memory */
	memSize = ROM_DMA_GetMemSize();
	if (memSize > sizeof(dmaDrvData)) {
		while(1);
	}
	devMem = dmaDrvData; /* Or just use malloc(memSize) */

	/* Initialize DMA driver */
	dmaInit.pUserData = (void *) NULL;
	dmaInit.base = (uint32_t) LPC_DMA;
	dmaInit.sramBase = (uint32_t) EXChip_DMA_Table;
	g_dmaHandle = ROM_DMA_Init(devMem, &dmaInit);
	if (g_dmaHandle == NULL) {
		while(1);
	}
}

