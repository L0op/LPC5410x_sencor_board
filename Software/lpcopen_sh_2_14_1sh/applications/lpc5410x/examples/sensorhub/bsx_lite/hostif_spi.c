/*
 * @brief Implements I2C slave driver for host interface module
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
#include <string.h>
#include "sensorhub.h"
#include "hostif.h"

#if defined(HOSTIF_SPI)

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*
    HIF_SPI_IDLE_ST:
        - Entry:
        - 16 byte CMD RX buffer queued
        - IRQn deasserted
        -
    HIF_SPI_RX_CMD_ST:
        - Entry: On data recieved from HIF_SPI_IDLE_ST
        - IRQn deasserted
        - process command after SPI CS deasserted
        - If response needed then load DMA descriptors 2 Tx (4 byte length field and response) + 1 Rx (4 + response size), Assert IRQn and move HIF_SPI_TX_LENGTH_ST
        - If no response needed then go to HIF_SPI_IDLE_ST
    HIF_SPI_TX_LENGTH_ST:
        - Entry: When SPI transmit initiated
        - Queue data length + response
        - Upon SPI CS deassertion move to HIF_SPI_TX_DATA_ST and Deassert IRQ
    HIF_SPI_TX_DATA_ST:
    - Entry: From HIF_SPI_TX_LENGTH_ST when SPI CS is deasserted
        - On completion of transfer move to HIF_SPI_IDLE_ST only if there is no transmit pending else move to HIF_SPI_TX_LENGTH_ST
 */

typedef enum {
	HIF_SPI_IDLE_ST = 0,
	HIF_SPI_RX_CMD_ST,
	HIF_SPI_TX_LENGTH_ST,
	HIF_SPI_TX_DATA_ST,

} Hostif_SPIState_t;

typedef struct __HOSTIF_SPISlave_Msg {

	uint8_t type;		/*!< Message type 0 - notification; 1 - command response */
	uint8_t cmd;		/*!< Command id if msg.type is response. */
	uint16_t length;	/*!< Length of the data following this message. */
} HOSTIF_SPISlave_Msg_t;

typedef struct __HOSTIF_SPICtrl_t {
	volatile Hostif_SPIState_t state;	/* State of interface */
	volatile uint32_t spiBusy;
	uint8_t rxBuff[HOSTIF_MAX_BUFFER_SIZE + sizeof(HOSTIF_SPISlave_Msg_t)];	/* Rx buffer */
	union {
		HOSTIF_SPISlave_Msg_t msg;
		uint32_t data;
	} m;

} Hostif_SPICtrl_t;

Hostif_SPICtrl_t g_hostif_spi;

#ifdef __ICCARM__
#define ALIGNSTR(x) #x
#define ALIGN(x) _Pragma(ALIGNSTR(data_alignment=##x))
#else
#define ALIGN(x) __attribute__((aligned(x)))
#endif

/* DMA descriptors must be aligned to 16 bytes */
ALIGN(16) static ROM_DMA_DESC_T dmaSPITXDesc[2];
ALIGN(16) static ROM_DMA_DESC_T dmaSPIRXDesc;

#define BUFFSENDSIZE 130

/* ROM driver handle for SPI slave */
static ROM_SPIS_HANDLE_T spisHandle;

/* SPI and DMA driver context areas */
static uint32_t spiDrvData[16];
ROM_DMA_XFERDESC_CFG_T dmaXferCfg;
ROM_DMA_QUEUE_T spiDmaTXQueue, spiDmaRXQueue;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/
 
extern ROM_DMA_HANDLE_T g_dmaHandle;

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Display error string and spin */
static void errorOut(char *errStr)
{
	while (1);
}

/* Start a DMA slave write operation */
static void spis_dma_WriteStart(void *buff, uint32_t bytes)
{		
	uint8_t flen;

	/* The DMA needs to be setup for 8- or 16-bit transfers based on the configured
     SPI transfer size. For the example, we'll get the size from the SPI peripherals
		 config. */
	flen = 1 + (uint8_t) ((SPI_HOSTIF->TXCTRL >> 24) & 0xF);
	
	/* Flush any previous transmission */
	ROM_DMA_StopQueue(g_dmaHandle, &spiDmaTXQueue);
	ROM_DMA_StopQueue(g_dmaHandle, &spiDmaRXQueue);
	ROM_DMA_FlushQueue(g_dmaHandle, &spiDmaTXQueue);
	ROM_DMA_FlushQueue(g_dmaHandle, &spiDmaRXQueue);
	

	dmaXferCfg.src  = (void *) &g_hostif_spi.m;
	dmaXferCfg.dest = (void *) &SPI_HOSTIF->TXDAT;

	/* Setup source to desination copy for trigger for memory */
	dmaXferCfg.xferCount = sizeof(HOSTIF_SPISlave_Msg_t);				/* Transfer data values of size width */
	dmaXferCfg.swTrig = 1;									/* No software triggering */
	dmaXferCfg.clrTrig = 0;									/* Do not clear trigger after this descriptor completes */
	dmaXferCfg.fireDescCB = 0;							/* Do not fire descriptor callback on complettion of this descriptor */
	dmaXferCfg.enabCirc = 0;							/* Do not use circular buffer */
	dmaXferCfg.dmaCh = spiDmaTXQueue.dmaCh;							/* DMA channel */
	dmaXferCfg.stallDesc = 0;					/* No descriptor stalling */
	if (flen > 8) {
		dmaXferCfg.width = ROM_DMA_WIDTH_2;			/* Width is 2 bytes */
	}
	else {
		dmaXferCfg.width = ROM_DMA_WIDTH_1;			/* Width is 1 byte */
	}
	dmaXferCfg.srcInc = ROM_DMA_ADDRINC_1X;	/* Increment source address by width for each data transfer */
	dmaXferCfg.dstInc = ROM_DMA_ADDRINC_0X; /* Increment destination address by width for each data transfer */
	
	if (ROM_DMA_BuildDescriptorChain(g_dmaHandle, &dmaXferCfg, &dmaSPITXDesc[0], NULL) != LPC_OK) {
		errorOut("Error building descriptor chain (SPI TX single link)\r\n");
	}
	/* Queue descriptor for SPI TX transfer */
	ROM_DMA_QueueDescriptor(g_dmaHandle, &spiDmaTXQueue, &dmaSPITXDesc[0]);	
	
	dmaXferCfg.src  = (void *) buff;
	dmaXferCfg.dest = (void *) &SPI_HOSTIF->TXDAT;

	/* Setup source to desination copy for trigger for memory */
	dmaXferCfg.xferCount = bytes;				/* Transfer data values of size width */
	dmaXferCfg.swTrig = 1;									/* No software triggering */
	dmaXferCfg.clrTrig = 0;									/* Do not clear trigger after this descriptor completes */
	dmaXferCfg.fireDescCB = 0;							/* Do not fire descriptor callback on complettion of this descriptor */
	dmaXferCfg.enabCirc = 0;							/* Do not use circular buffer */
	dmaXferCfg.dmaCh = spiDmaTXQueue.dmaCh;							/* DMA channel */
	dmaXferCfg.stallDesc = 0;					/* No descriptor stalling */
	if (flen > 8) {
		dmaXferCfg.width = ROM_DMA_WIDTH_2;			/* Width is 2 bytes */
	}
	else {
		dmaXferCfg.width = ROM_DMA_WIDTH_1;			/* Width is 1 byte */
	}
	dmaXferCfg.srcInc = ROM_DMA_ADDRINC_1X;	/* Increment source address by width for each data transfer */
	dmaXferCfg.dstInc = ROM_DMA_ADDRINC_0X; /* Increment destination address by width for each data transfer */
	
	if (ROM_DMA_BuildDescriptorChain(g_dmaHandle, &dmaXferCfg, &dmaSPITXDesc[1], NULL) != LPC_OK) {
		errorOut("Error building descriptor chain (SPI TX single link)\r\n");
	}
	/* Disable the INTA and INTB for last descriptor to avoid DMA IRQ when SPI IRQ is modifying the descriptors */
	dmaSPITXDesc[1].xfercfg &= ~(DMA_XFERCFG_SETINTA | DMA_XFERCFG_SETINTB);
	/* Queue descriptor for SPI TX transfer */
	ROM_DMA_QueueDescriptor(g_dmaHandle, &spiDmaTXQueue, &dmaSPITXDesc[1]);	
	
	/***** Rx Descriptor *****/
	
	dmaXferCfg.src = (void *) &SPI_HOSTIF->RXDAT;
	dmaXferCfg.dest  = (void *) &g_hostif_spi.rxBuff[0];

	/* Setup source to desination copy for trigger for memory */
	dmaXferCfg.xferCount = bytes + sizeof(HOSTIF_SPISlave_Msg_t);			/* Transfer data values of size width */
	dmaXferCfg.swTrig = 1;									/* No software triggering */
	dmaXferCfg.clrTrig = 0;									/* Do not clear trigger after this descriptor completes */
	dmaXferCfg.fireDescCB = 0;							/* Do not fire descriptor callback on complettion of this descriptor */
	dmaXferCfg.enabCirc = 0;								/* Not a circular buffer */
	dmaXferCfg.dmaCh = spiDmaRXQueue.dmaCh;							/* Do not use circularo buffer */
	dmaXferCfg.stallDesc = 0;					/* No descriptor stalling */
	if (flen > 8) {
		dmaXferCfg.width = ROM_DMA_WIDTH_2;			/* Width is 2 bytes */
	}
	else {
		dmaXferCfg.width = ROM_DMA_WIDTH_1;			/* Width is 1 byte */
	}
	dmaXferCfg.srcInc = ROM_DMA_ADDRINC_0X;	/* Increment source address by width for each data transfer */
	dmaXferCfg.dstInc = ROM_DMA_ADDRINC_1X; /* Increment destination address by width for each data transfer */
	
	if (ROM_DMA_BuildDescriptorChain(g_dmaHandle, &dmaXferCfg, &dmaSPIRXDesc, NULL) != LPC_OK) {
		errorOut("Error building descriptor chain (SPI TX single link)\r\n");
	}
	/* Disable the INTA and INTB for Rx descriptor to avoid DMA IRQ when SPI IRQ is modifying the descriptors */
	dmaSPIRXDesc.xfercfg &= ~(DMA_XFERCFG_SETINTA | DMA_XFERCFG_SETINTB);

	/* Queue descriptor for SPI RX transfer */
	ROM_DMA_QueueDescriptor(g_dmaHandle, &spiDmaRXQueue, &dmaSPIRXDesc);

	/* Start Tx queue */
	ROM_DMA_StartQueue(g_dmaHandle, &spiDmaTXQueue);
	
	/* Start Rx queue */
	ROM_DMA_StartQueue(g_dmaHandle, &spiDmaRXQueue);	
	
}

/* Start a DMA slave read operation */
static void spis_dma_ReadStart(void)
{		
	uint8_t flen;

	/* The DMA needs to be setup for 8- or 16-bit transfers based on the configured
     SPI transfer size. For the example, we'll get the size from the SPI peripherals
		 config. */
	flen = 1 + (uint8_t) ((SPI_HOSTIF->TXCTRL >> 24) & 0xF);
	
	ROM_DMA_StopQueue(g_dmaHandle, &spiDmaTXQueue);
	ROM_DMA_StopQueue(g_dmaHandle, &spiDmaRXQueue);
	ROM_DMA_FlushQueue(g_dmaHandle, &spiDmaTXQueue);
	ROM_DMA_FlushQueue(g_dmaHandle, &spiDmaRXQueue);

	dmaXferCfg.src = (void *) &SPI_HOSTIF->RXDAT;
	dmaXferCfg.dest  = (void *) &g_hostif_spi.rxBuff[0];

	/* Setup source to desination copy for trigger for memory */
	dmaXferCfg.xferCount = LPCSH_MAX_CMD_LENGTH;			/* Transfer data values of size width */
	dmaXferCfg.swTrig = 1;									/* No software triggering */
	dmaXferCfg.clrTrig = 0;									/* Do not clear trigger after this descriptor completes */
	dmaXferCfg.fireDescCB = 0;							/* Do not fire descriptor callback on complettion of this descriptor */
	dmaXferCfg.enabCirc = 0;								/* Not a circular buffer */
	dmaXferCfg.dmaCh = spiDmaRXQueue.dmaCh;							/* Do not use circularo buffer */
	dmaXferCfg.stallDesc = 0;					/* No descriptor stalling */
	if (flen > 8) {
		dmaXferCfg.width = ROM_DMA_WIDTH_2;			/* Width is 2 bytes */
	}
	else {
		dmaXferCfg.width = ROM_DMA_WIDTH_1;			/* Width is 1 byte */
	}
	dmaXferCfg.srcInc = ROM_DMA_ADDRINC_0X;	/* Increment source address by width for each data transfer */
	dmaXferCfg.dstInc = ROM_DMA_ADDRINC_1X; /* Increment destination address by width for each data transfer */
	
	if (ROM_DMA_BuildDescriptorChain(g_dmaHandle, &dmaXferCfg, &dmaSPIRXDesc, NULL) != LPC_OK) {
		errorOut("Error building descriptor chain (SPI RX single link)\r\n");
	}

	/* Queue descriptor for SPI RX transfer */
	ROM_DMA_QueueDescriptor(g_dmaHandle, &spiDmaRXQueue, &dmaSPIRXDesc);
	
	/* Start Rx queue */
	ROM_DMA_StartQueue(g_dmaHandle, &spiDmaRXQueue);		
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/*DMA controller transfer descriptor error callback */
void dmaSPITXTransferError(ROM_DMA_HANDLE_T g_dmaHandle, struct ROM_DMA_QUEUE *pQueue, ROM_DMA_DESC_T *pTranDesc)
{
	/* Go to idle state and prime DMA for Rx upon error */
	g_hostif_spi.state = HIF_SPI_IDLE_ST;
	spis_dma_ReadStart();
}

/*DMA controller transfer descriptor error callback */
void dmaSPIRXTransferError(ROM_DMA_HANDLE_T g_dmaHandle, struct ROM_DMA_QUEUE *pQueue, ROM_DMA_DESC_T *pTranDesc)
{
	/* Go to idle state and prime DMA for Rx upon error */
	g_hostif_spi.state = HIF_SPI_IDLE_ST;
	spis_dma_ReadStart();
}

void SPI_HOSTIF_IRQHandler(void)
{
	ROM_SPIS_TransferHandler(spisHandle);
	ResMgr_IRQDone();
}

void CBspiSlaveXferCSAssertCB(ROM_SPIS_HANDLE_T spisHandle, uint8_t slaveNum)
{
	g_hostif_spi.spiBusy = 1;
	/*If state is idle then change it to receive command */
	if (g_hostif_spi.state == HIF_SPI_IDLE_ST) {
		g_hostif_spi.state = HIF_SPI_RX_CMD_ST;
	}
}

void CBspiSlaveXferCSDeAssertCB(ROM_SPIS_HANDLE_T spisHandle, ROM_SPIS_XFER_T *pXfer)
{	
	uint16_t rx_len;
	g_hostif_spi.spiBusy = 0;
	/* If state is transmitting length then DeAssert IRQ and change to transmit data state */
	if (g_hostif_spi.state == HIF_SPI_TX_LENGTH_ST) {
		g_hostif_spi.state = HIF_SPI_TX_DATA_ST;
		Hostif_DeassertIRQ();
	}
	/* If state is transmit data then check for queued transmit */
	else if (g_hostif_spi.state == HIF_SPI_TX_DATA_ST) {
		g_hostif_spi.state = HIF_SPI_IDLE_ST;
		/* Check for any queued transmit */
		Hostif_CheckTx();
		/* If no transmit was queued then setup DMA for receiving commands */
		if (g_hostif_spi.state == HIF_SPI_IDLE_ST) {
			spis_dma_ReadStart();
		}
	}
	/* If state was receive command then process received command */
	if (g_hostif_spi.state == HIF_SPI_RX_CMD_ST) {
		/* Calculate received length */
		rx_len = ((LPC_DMA->DMACH[spiDmaRXQueue.dmaCh].XFERCFG >> 16) & 0x3FF);
		/* Queued DMA length has been received, rollover from 0 */
		if (rx_len == 0x3FF) {
			rx_len = LPCSH_MAX_CMD_LENGTH;
		}
		else {
			/* Received Length is queued length - pending length */
			rx_len = LPCSH_MAX_CMD_LENGTH - rx_len;
		}
		/* Process command */
		Hostif_CmdProcess(&g_hostif_spi.rxBuff[0], rx_len);
		/* If no transmit was queued when command was processed, then check for queued Transmit */
		if (g_hostif_spi.state == HIF_SPI_RX_CMD_ST) {
			g_hostif_spi.state = HIF_SPI_IDLE_ST;
			/* Check for any queued transmit */
			Hostif_CheckTx();
			/* If no transmit was queued then setup DMA for receiving commands */
			if (g_hostif_spi.state == HIF_SPI_IDLE_ST) {
				spis_dma_ReadStart();
			}
		}
	}
}

/** Initialize the Sensor Hub host/AP interface */
void Hostif_Init(void)
{
	uint32_t memSize, *devMem;
	ROM_SPIS_INIT_T spisInit;
	ROM_SPIS_SLAVE_T spisConfig;

	const ROM_DMA_CHAN_CFG_T chanCfg = {
		1,											/* Use peripheral DMA request */
		0,											/* Hardware trigger polarity high */
		0,											/* Hardware trigger edge triggered */
		0,											/* Single transfer on each trigger */
		ROM_DMA_BURSTPOWER_1,		/* Burst size of 1 datum */
		0,											/* Disable source burst wrap */
		0,											/* Disable destination burst wrap */
		0,											/* Channel priority = 0 (highest) */
		0,											/* reserved */
	};

	/* reset host IF control data structure */
	memset(&g_hostif_spi, 0, sizeof(Hostif_SPICtrl_t));

	/******************************************************************/
	/* START OF DMA SETUP */
	/******************************************************************/

	/* Setup SPI1 TX channel for peripheral request and init queue */
	if (ROM_DMA_SetupChannel(g_dmaHandle, (ROM_DMA_CHAN_CFG_T *) &chanCfg, SPI_HOSTIF_TX_DMACH) != LPC_OK) {
		errorOut("Error setting up SPI TX DMA channel\r\n");
	}
	if (ROM_DMA_InitQueue(g_dmaHandle, SPI_HOSTIF_TX_DMACH, &spiDmaTXQueue) != LPC_OK) {
		errorOut("Error initializing SPI TX DMA queue\r\n");
	}

	/* Setup SPI1 RX channel for peripheral request and init queue */
	if (ROM_DMA_SetupChannel(g_dmaHandle, (ROM_DMA_CHAN_CFG_T *) &chanCfg, SPI_HOSTIF_RX_DMACH) != LPC_OK) {
		errorOut("Error setting up SPI RX DMA channel\r\n");
	}
	if (ROM_DMA_InitQueue(g_dmaHandle, SPI_HOSTIF_RX_DMACH, &spiDmaRXQueue) != LPC_OK) {
		errorOut("Error initializing SPI RX DMA queue\r\n");
	}

	/* Register error, descriptor completion, and descriptor chain completion callbacks for SPI TX channel */
	ROM_DMA_RegisterQueueCallback(g_dmaHandle, &spiDmaTXQueue, ROM_DMA_XFERERROR_CB, (void *)dmaSPITXTransferError);

	/* Register error, descriptor completion, and descriptor chain completion callbacks for SPI RX channel */
	ROM_DMA_RegisterQueueCallback(g_dmaHandle, &spiDmaRXQueue, ROM_DMA_XFERERROR_CB, (void *)dmaSPIRXTransferError);
	
	/******************************************************************/
	/* START OF SPI SLAVE SETUP */
	/******************************************************************/
	
	Chip_Clock_EnablePeriphClock(SPI_HOSTIF_CLOCK);
	Chip_SYSCON_PeriphReset(SPI_HOSTIF_RESET);

	/* Get needed size for SPI driver context memory */
	memSize = ROM_SPIS_GetMemSize();
	if (memSize > sizeof(spiDrvData)) {
		errorOut("Can't allocate memory for SPI driver context\r\n");
	}
	devMem = spiDrvData; /* Or just use malloc(memSize) */

	/* Initialize driver */
	spisInit.pUserData = (void *) NULL;
	spisInit.base = (uint32_t) SPI_HOSTIF;
	spisInit.spiPol[0] = 0; /* Active low select for SSEL0 */
	spisInit.spiPol[1] = 0;
	spisInit.spiPol[2] = 0;
	spisInit.spiPol[3] = 0;
	spisHandle = ROM_SPIS_Init(devMem, &spisInit);
	if (spisHandle == NULL) {
		/* Error initializing SPI */
		errorOut("Error initializing ROM\r\n");
	}

	/* Set SPI transfer configuration */
	spisConfig.mode = ROM_SPI_CLOCK_MODE0;
	spisConfig.lsbFirst = 0;
	spisConfig.dataBits = 8;
	if (ROM_SPIS_SetupSlave(spisHandle, &spisConfig) != LPC_OK) {
		errorOut("SPI configuration is invalid\r\n");
	}

	/* Callback registration  */
	ROM_SPIS_RegisterCallback(spisHandle, ROM_SPIS_ASSERTSSEL_CB, (void *)CBspiSlaveXferCSAssertCB);
	ROM_SPIS_RegisterCallback(spisHandle, ROM_SPIS_DEASSERTSSEL_CB, (void *)CBspiSlaveXferCSDeAssertCB);
	
	/* init host interrupt pin */
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, HOSTIF_IRQ_PORT, HOSTIF_IRQ_PIN);
	
	/* initlize host command engine and related buffers */
	Hostif_CmdEngineInit();
	/* enable SPI Rx */
	spis_dma_ReadStart();
	/* DMA interrupt is only used for error handling */
	NVIC_SetPriority(DMA_IRQn, DMA_IRQ_PRIORITY);
	NVIC_EnableIRQ(DMA_IRQn);

	NVIC_SetPriority(SPI_HOSTIF_IRQn, HOSTIF_SPI_IRQ_PRIORITY);
	/* enable SPI IRQ */
	NVIC_EnableIRQ(SPI_HOSTIF_IRQn);
	/* enable SPI hostif to wake-up the sensor hub */
	Chip_SYSCON_EnableWakeup(SPI_HOSTIF_WAKE);

}

void Hostif_StartTx(uint8_t *pBuf, uint16_t size, uint8_t cmd)
{
	g_hostif_spi.state = HIF_SPI_TX_LENGTH_ST;
	/* queue the buffer length and asert host IRQ */
	g_hostif_spi.m.data = 0;
	/* queue notification packet */
	g_hostif_spi.m.msg.type = (cmd) ? 1 : 0;
	g_hostif_spi.m.msg.cmd = cmd;
	g_hostif_spi.m.msg.length = size;
	spis_dma_WriteStart(pBuf, size);
}

/** Is SPI Ready for Tx*/
uint8_t Hostif_TxReady(void)
{
	return (g_hostif_spi.state == HIF_SPI_IDLE_ST) && (!(SPI_HOSTIF->STAT & SPI_STAT_SSA));
}

uint8_t Hostif_SleepOk(void)
{
	return (g_hostif_spi.state != HIF_SPI_IDLE_ST);
}

#endif /*HOSTIF_SPI*/
