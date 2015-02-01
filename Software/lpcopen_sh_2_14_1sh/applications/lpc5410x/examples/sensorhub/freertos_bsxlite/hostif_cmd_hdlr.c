/*
 * @brief Implements buffer management for host interface module
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

//FIXME - The Host Interface command handler needs to be made common for both I2C and SPI

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
/** Manifest constants for WHOAMI and version numbers */
#define CFG_SPI_WHO_AM_I					0x53        /**< SPI who am I value */
#define CFG_I2C_WHO_AM_I					0x54        /**< I2C who am I value */
#define CFG_BOOT_WHO_AM_I					0x55        /**< CRP boot who am I value */

#if defined(HOSTIF_I2C)
#define HOSTIF_NUM_DATA_BUFFERS 5

struct Hostif_CmdCtrl_t {
    uint64_t sensorEnable;			/* Variable to Sensor enable state.  */
   
    uint16_t wr;		
    uint16_t rd;
    uint16_t data_buffer_used[HOSTIF_NUM_DATA_BUFFERS];
    uint16_t last_transmitted_offset[HOSTIF_NUM_DATA_BUFFERS];

    uint8_t data_buffer[HOSTIF_NUM_DATA_BUFFERS][HOSTIF_MAX_DATA_SIZE];
    uint8_t  data_buffer_flags[HOSTIF_NUM_DATA_BUFFERS];

    uint16_t data_buffer_commited_length;
    
    uint8_t cmdRespBuff[LPCSH_MAX_CMD_LENGTH];    
    uint8_t currentOpCode;
};

static struct Hostif_CmdCtrl_t g_hostif_cmd;

#endif

#if defined(HOSTIF_SPI)

typedef struct __HOSTIF_CmdCtrl_t {
	uint64_t sensorEnable;			/* Variable to Sensor enable state.  */
	uint16_t wrPtr;
	uint16_t rdPtr;
	uint8_t buffEmpty;
	uint8_t lastCmd;
	uint8_t pad[2];

	uint8_t buffer[HOSTIF_MAX_BUFFER_SIZE];
	uint8_t cmdRespBuff[LPCSH_MAX_CMD_LENGTH];
} Hostif_CmdCtrl_t;

Hostif_CmdCtrl_t g_hostif_cmd;

#endif

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

#if defined(HOSTIF_I2C)

static void  Hostif_calculate_tx_buffer_size(void)
{
	g_hostif_cmd.data_buffer_commited_length = 0;

	/* if buffer writting is done, and nothing left in it, goto next buffer */
	if ((g_hostif_cmd.data_buffer_flags[g_hostif_cmd.rd]) &&
		(g_hostif_cmd.last_transmitted_offset[g_hostif_cmd.rd] >= g_hostif_cmd.data_buffer_used[g_hostif_cmd.rd])) {
			
		g_hostif_cmd.last_transmitted_offset[g_hostif_cmd.rd] = 0;
		g_hostif_cmd.data_buffer_used[g_hostif_cmd.rd] = 0;
		g_hostif_cmd.data_buffer_flags[g_hostif_cmd.rd] = 0;

		/* point to next circular TX buffer */
    ++g_hostif_cmd.rd;
		g_hostif_cmd.rd %= HOSTIF_NUM_DATA_BUFFERS;
	}

	/* if buffer has some records */
	if (g_hostif_cmd.data_buffer_used[g_hostif_cmd.rd] <= g_hostif_cmd.last_transmitted_offset[g_hostif_cmd.rd]) {
		g_hostif_cmd.last_transmitted_offset[g_hostif_cmd.rd] = 0;
		g_hostif_cmd.data_buffer_used[g_hostif_cmd.rd] = 0;
		g_hostif_cmd.data_buffer_flags[g_hostif_cmd.rd] = 0;

		g_hostif_cmd.data_buffer_commited_length = 0;
	}
	else {
		g_hostif_cmd.data_buffer_commited_length = g_hostif_cmd.data_buffer_used[g_hostif_cmd.rd] - g_hostif_cmd.last_transmitted_offset[g_hostif_cmd.rd];
	}

	if (g_hostif_cmd.data_buffer_commited_length > 0) {
		Hostif_AssertIRQ();
	}
	else {
		Hostif_DeassertIRQ();
	}
}

static void Hostif_commit_data_tx(void)
{

	g_hostif_cmd.last_transmitted_offset[g_hostif_cmd.rd] += g_hostif_cmd.data_buffer_commited_length;

	/* set current used size to length */
	if (g_hostif_cmd.last_transmitted_offset[g_hostif_cmd.rd] >= g_hostif_cmd.data_buffer_used[g_hostif_cmd.rd]) {
		/* If current buffer was full and data is read then move to next */
		if (g_hostif_cmd.data_buffer_flags[g_hostif_cmd.rd]) {
			g_hostif_cmd.last_transmitted_offset[g_hostif_cmd.rd] = 0;
			g_hostif_cmd.data_buffer_used[g_hostif_cmd.rd] = 0;
			g_hostif_cmd.data_buffer_flags[g_hostif_cmd.rd] = 0;

			/* point to next circular TX buffer */
      ++g_hostif_cmd.rd;
			g_hostif_cmd.rd %=  HOSTIF_NUM_DATA_BUFFERS;

		}
	}
}

static uint8_t Hostif_writeRingBuf(const uint8_t *pBuffer, uint16_t length)
{
	uint8_t err = 0;

	/* if current WR buffer not full */
	if (g_hostif_cmd.data_buffer_flags[g_hostif_cmd.wr]) {
		/* no space in buffer, record is lost */
		err = 1;
	}
	else {
		/* if current WR buffer not full */
		if ((g_hostif_cmd.data_buffer_used[g_hostif_cmd.wr] + length) > HOSTIF_MAX_DATA_SIZE) {
			/* if no room for new record, mark current WR buffer as full */
			g_hostif_cmd.data_buffer_flags[g_hostif_cmd.wr] = 1;
			/* point to next circular TX buffer */
      ++g_hostif_cmd.wr;
			g_hostif_cmd.wr %= HOSTIF_NUM_DATA_BUFFERS;

			/* if current WR buffer not full */
			if (g_hostif_cmd.data_buffer_flags[g_hostif_cmd.wr]) {
				/* no space in buffer, record is lost */
				err = 1;
				Hostif_DeassertIRQ();
				Hostif_AssertIRQ();
			}
		}
		if (!err) {
			/* copy new record into buffer */
			memcpy(&g_hostif_cmd.data_buffer[g_hostif_cmd.wr][g_hostif_cmd.data_buffer_used[g_hostif_cmd.wr]], pBuffer, length);
			/* set current used size to length */
			g_hostif_cmd.data_buffer_used[g_hostif_cmd.wr] += length;
      Hostif_AssertIRQ();
		}
	}
	return err;
}

#endif

#if defined(HOSTIF_SPI)

static void startPhyTx(void)
{
	uint16_t curRdIndex = 0;
	uint16_t length = 0;
	/* Read length byte from ring buffer */
	length = g_hostif_cmd.buffer[g_hostif_cmd.rdPtr++];
	/* No padding added if length is greater than zero */
	if (length > 0) {
		/* Read from the next index */
		curRdIndex = g_hostif_cmd.rdPtr;
		/* Update read pointer */
		g_hostif_cmd.rdPtr += length;
		/* Check for rollover */
		if (g_hostif_cmd.rdPtr >= HOSTIF_MAX_BUFFER_SIZE) {
			g_hostif_cmd.rdPtr = 0;
		}
	}
	/* Length is zero so Padding added start from index 0*/
	else {
		/* Ring Buffer is padded start from index 0 */
		g_hostif_cmd.rdPtr = 0;
		/* Read Length */
		length = g_hostif_cmd.buffer[g_hostif_cmd.rdPtr++];
		/* Read from the next index */
		curRdIndex = g_hostif_cmd.rdPtr;
		/* Update read pointer */
		g_hostif_cmd.rdPtr += length;
		/* Check for rollover */
		if (g_hostif_cmd.rdPtr >= HOSTIF_MAX_BUFFER_SIZE) {
			g_hostif_cmd.rdPtr = 0;
		}
	}
	/* When read pointer matches write pointer then buffer is empty */
	if(g_hostif_cmd.rdPtr == g_hostif_cmd.wrPtr) {
		g_hostif_cmd.buffEmpty = 1;
	}
	/* Queue DMA transmission */
	Hostif_StartTx(&g_hostif_cmd.buffer[curRdIndex], length, g_hostif_cmd.lastCmd);
	g_hostif_cmd.lastCmd = 0;
}

/* Write Tx data to Ring Buffer */
static void writeRingBuf(const uint8_t *pBuffer, uint16_t length)
{
	uint32_t buff_full = 0;
	uint32_t remain = HOSTIF_MAX_BUFFER_SIZE - g_hostif_cmd.wrPtr;
	/* Length increased to accomodate length byte, but length byte is not transmitted only used to
	        identify padded ring buffer */
	if (length + 1 > remain) {
		/* Check if the write to overwrite the unread data */
		if((g_hostif_cmd.buffEmpty == 0) && ((g_hostif_cmd.rdPtr >= g_hostif_cmd.wrPtr) || (g_hostif_cmd.rdPtr < length))) {
			buff_full = 1;
		}
		else {
			/* fill the remaining with dummy data */
			g_hostif_cmd.buffer[g_hostif_cmd.wrPtr] = HOSTIF_DUMMY_PACKET_ID;
			g_hostif_cmd.wrPtr = 0;
		}
	}
	else {
		/* Check if the write to overwrite the unread data */
		if((g_hostif_cmd.buffEmpty == 0) && (g_hostif_cmd.rdPtr >= g_hostif_cmd.wrPtr) && (g_hostif_cmd.rdPtr < (g_hostif_cmd.wrPtr + length))) {
			buff_full = 1;
		}
	}
	/* If buffer is not full then copy write data to buffer */
	if(buff_full == 0) {
		g_hostif_cmd.buffEmpty = 0;
		/* Copy length byte */
		g_hostif_cmd.buffer[g_hostif_cmd.wrPtr++] = length;
		/* copy the data to ring buffer */
		memcpy(&g_hostif_cmd.buffer[g_hostif_cmd.wrPtr], pBuffer, length);
		g_hostif_cmd.wrPtr += length;
		if (g_hostif_cmd.wrPtr >= HOSTIF_MAX_BUFFER_SIZE) {
			g_hostif_cmd.wrPtr = 0;
		}
	}
}

#endif


/*****************************************************************************
 * Public functions
 ****************************************************************************/

#if defined(HOSTIF_I2C)

/** Initialize the buffer and command engine */
void Hostif_CmdEngineInit(void)
{
	/* Init host buffer */
	memset(&g_hostif_cmd, 0, sizeof(g_hostif_cmd));
  Hostif_DeassertIRQ();
}

void Hostif_TxCompleteCB(void)
{
	if (g_hostif_cmd.currentOpCode == LPCSH_CMD_GET_DATA) {
		Hostif_commit_data_tx();
		Hostif_calculate_tx_buffer_size();
	}
	
}

uint8_t Hostif_isSensorEnable(uint8_t sensorId)
{
	if (sensorId < LPCSH_SENSOR_ID_COUNT) {
		return (g_hostif_cmd.sensorEnable & (1L << sensorId)) ? 1 : 0;
	}
	return 0;
}

uint8_t Hostif_QueueBuffer(const uint8_t *pBuffer, uint16_t length)
{
	/* Write to Ring Buffer and increased length to include header */
	return Hostif_writeRingBuf(pBuffer, length + sizeof(struct lpcsh_sensor_node_header));
}

uint8_t  Hostif_CmdProcess(uint8_t *rx_buf, uint16_t length)
{
    uint8_t remaining = 0;
    enum LPCSH_SENSOR_ID sensorId;
    struct LPCSH_CMD_t *pCmd = (struct LPCSH_CMD_t *) rx_buf;
    g_hostif_cmd.currentOpCode = pCmd->id;
    switch (length ) {
    case 1:
        switch (pCmd->id) {
					case LPCSH_CMD_GET_DATA_LENGTH:
							Hostif_calculate_tx_buffer_size();
							Hostif_StartTx((uint8_t *)&g_hostif_cmd.data_buffer_commited_length, sizeof(g_hostif_cmd.data_buffer_commited_length), pCmd->id);
							break;
					case LPCSH_CMD_GET_DATA:
							Hostif_StartTx(&g_hostif_cmd.data_buffer[g_hostif_cmd.rd][g_hostif_cmd.last_transmitted_offset[g_hostif_cmd.rd]], g_hostif_cmd.data_buffer_commited_length, pCmd->id);
							break;
					case LPCSH_CMD_WHO_AM_I:
							g_hostif_cmd.cmdRespBuff[0] = CFG_I2C_WHO_AM_I;		/* NXP Sensor Hub variant of WHO_AM_I */
							/* Queue DMA transmission */
							Hostif_StartTx(&g_hostif_cmd.cmdRespBuff[0], 1, pCmd->id);
							break;
					case LPCSH_CMD_GET_VERSION:		/* get version number */
							g_hostif_cmd.cmdRespBuff[0] = CFG_MAJOR_VERSION;		/* Major version */
							g_hostif_cmd.cmdRespBuff[1] = CFG_MINOR_VERSION;		/* Minor version */
							/* Queue DMA transmission */
							Hostif_StartTx(&g_hostif_cmd.cmdRespBuff[0], 2, pCmd->id);
							break;
					case LPCSH_CMD_RESET:	/* Reset sensor hub host interface */
							for (sensorId = LPCSH_SENSOR_ID_FIRST; sensorId < LPCSH_SENSOR_ID_COUNT; sensorId++) {
									if (g_hostif_cmd.sensorEnable & (1 << sensorId)) {
											Algorithm_EnableSensor(sensorId, 0);
											g_hostif_cmd.sensorEnable &= ~(1 << sensorId);
									}
							}
							Hostif_CmdEngineInit();
							Algorithm_Init();
							break;
					case LPCSH_CMD_SENSOR_ENABLE:
							Hostif_StartTx(NULL, 0, 0);
							remaining = 2;              /* sensorId, enable boolean byte */
							break;
					case LPCSH_CMD_GET_DELAY:
					case LPCSH_CMD_GET_SENSOR_STATE:
							Hostif_StartTx(NULL, 0, 0);
							remaining = 1;              /* sensor id */
							break;
					case LPCSH_CMD_SET_DELAY:
							Hostif_StartTx(NULL, 0, 0);
							remaining = 5;              /* sensor id, 32 bit delay */
							break;
					default:
							Hostif_StartTx(NULL, 0, 0);
							break;
        }
        break;
    case 2:
        switch (pCmd->id) {
					case LPCSH_CMD_GET_DELAY:		/* get sensor delay */
						if (pCmd->params[0] < PHYS_MAX_ID) {
								*((uint16_t *) &g_hostif_cmd.cmdRespBuff[0]) = (uint16_t) g_Timer.getMsFromTicks(g_phySensors[pCmd->params[0]]->period);
						}
						else {
								g_hostif_cmd.cmdRespBuff[0] = 0xFF;
								g_hostif_cmd.cmdRespBuff[1] = 0xFF;
						}
						/* Queue DMA transmission */
						Hostif_StartTx(&g_hostif_cmd.cmdRespBuff[0], 2, pCmd->id);
						break;
							
					case LPCSH_CMD_GET_SENSOR_STATE:		/* get sensor enable */
							if (g_hostif_cmd.sensorEnable & (1 << pCmd->params[0])) {
									g_hostif_cmd.cmdRespBuff[0] = 1;
							}
							else {
									g_hostif_cmd.cmdRespBuff[0] = 0;
							}
							/* Queue DMA transmission */
							Hostif_StartTx(&g_hostif_cmd.cmdRespBuff[0], 1, pCmd->id);
							break;
        }
        break;
    case 3:
        switch (pCmd->id) {
					case LPCSH_CMD_SENSOR_ENABLE:
						if (pCmd->params[1] == 0) {
							if (g_hostif_cmd.sensorEnable & (1 << pCmd->params[0])) {
									Algorithm_EnableSensor((enum LPCSH_SENSOR_ID) pCmd->params[0], 0);
									g_hostif_cmd.sensorEnable &= ~(1 << pCmd->params[0]);
							}
						}
						else {
							if ((g_hostif_cmd.sensorEnable & (1 << pCmd->params[0])) == 0) {
									Algorithm_EnableSensor((enum LPCSH_SENSOR_ID) pCmd->params[0], 1);
									g_hostif_cmd.sensorEnable |= (1 << pCmd->params[0]);
							}
						}
						break;
        }
        break;
		case 4:
			 switch (pCmd->id) {
				 case LPCSH_CMD_SET_DELAY:		/* set sensor delay */
					if (pCmd->params[0] < PHYS_MAX_ID) {
						g_phySensors[pCmd->params[0]]->hw->setDelay(g_phySensors[pCmd->params[0]],
											(pCmd->params[1] | (pCmd->params[2] << 8)));
					}
					break;
			 }
			break;
    default:
        remaining = 0;
        break;
    }
    return remaining;
}

#endif


#if defined(HOSTIF_SPI)

/** Initialize the buffer and command engine */
void Hostif_CmdEngineInit(void)
{
	/* Init host buffer */
	memset(&g_hostif_cmd, 0, sizeof(g_hostif_cmd));
	g_hostif_cmd.buffEmpty = 1;
	Hostif_DeassertIRQ();
}

uint8_t  Hostif_CmdProcess(uint8_t *rx_buf, uint16_t length)
{
	HOSTIF_CMD_t *pCmd = (HOSTIF_CMD_t *) rx_buf;
	enum LPCSH_SENSOR_ID sensorId;

	switch (pCmd->id) {
	case LPCSH_CMD_WHO_AM_I: /* provide ID */
		g_hostif_cmd.cmdRespBuff[0] = CFG_SPI_WHO_AM_I;		/* SPI variant of WHO_AM_I */
		/* Queue DMA transmission */
		Hostif_StartTx(&g_hostif_cmd.cmdRespBuff[0], 1, pCmd->id);
		Hostif_AssertIRQ();
		break;

	case LPCSH_CMD_GET_VERSION:		/* get version number */
		g_hostif_cmd.cmdRespBuff[0] = CFG_MAJOR_VERSION;		/* Major version */
		g_hostif_cmd.cmdRespBuff[1] = CFG_MINOR_VERSION;		/* Minor version */
		/* Queue DMA transmission */
		Hostif_StartTx(&g_hostif_cmd.cmdRespBuff[0], 2, pCmd->id);
		Hostif_AssertIRQ();
		break;

	case LPCSH_CMD_RESET:	/* Reset sensor hub host interface */
		for (sensorId = LPCSH_SENSOR_ID_FIRST; sensorId < LPCSH_SENSOR_ID_COUNT; sensorId++) {
			if (g_hostif_cmd.sensorEnable & (1 << sensorId)) {
				Algorithm_EnableSensor(sensorId, 0);
				g_hostif_cmd.sensorEnable &= ~(1 << sensorId);
			}
		}

		Hostif_CmdEngineInit();
		Algorithm_Init();
		g_hostif_cmd.cmdRespBuff[0] = 1;
		/* Queue DMA transmission */
		Hostif_StartTx(&g_hostif_cmd.cmdRespBuff[0], 1, pCmd->id);
		Hostif_AssertIRQ();
		break;

	case LPCSH_CMD_SENSOR_ENABLE:		/* sensor enable */
		if (pCmd->params[1] == 0) {
			if (g_hostif_cmd.sensorEnable & (1 << pCmd->params[0])) {
				Algorithm_EnableSensor((enum LPCSH_SENSOR_ID) pCmd->params[0], 0);
				g_hostif_cmd.sensorEnable &= ~(1 << pCmd->params[0]);
			}
		}
		else {
			if ((g_hostif_cmd.sensorEnable & (1 << pCmd->params[0])) == 0) {
				Algorithm_EnableSensor((enum LPCSH_SENSOR_ID) pCmd->params[0], 1);
				g_hostif_cmd.sensorEnable |= (1 << pCmd->params[0]);
			}
		}
		g_hostif_cmd.cmdRespBuff[0] = 1;
		/* Queue DMA transmission */
		Hostif_StartTx(&g_hostif_cmd.cmdRespBuff[0], 1, pCmd->id);
		Hostif_AssertIRQ();
		break;

	case LPCSH_CMD_GET_SENSOR_STATE:		/* get sensor enable */
		if (g_hostif_cmd.sensorEnable & (1 << pCmd->params[0])) {
			g_hostif_cmd.cmdRespBuff[0] = 1;
		}
		else {
			g_hostif_cmd.cmdRespBuff[0] = 0;
		}
		/* Queue DMA transmission */
		Hostif_StartTx(&g_hostif_cmd.cmdRespBuff[0], 1, pCmd->id);
		Hostif_AssertIRQ();
		break;

	case LPCSH_CMD_SET_DELAY:		/* set sensor delay */
		if (pCmd->params[0] < PHYS_MAX_ID) {
			g_hostif_cmd.cmdRespBuff[0] = 1;
			g_phySensors[pCmd->params[0]]->hw->setDelay(g_phySensors[pCmd->params[0]],
														(pCmd->params[1] | (pCmd->params[2] << 8)));
		}
		else {
			g_hostif_cmd.cmdRespBuff[0] = 0xFF;
		}
		/* Queue DMA transmission */
		Hostif_StartTx(&g_hostif_cmd.cmdRespBuff[0], 1, pCmd->id);
		Hostif_AssertIRQ();
		break;

	case LPCSH_CMD_GET_DELAY:		/* get sensor delay */
		if (pCmd->params[0] < PHYS_MAX_ID) {
			*((uint16_t *) &g_hostif_cmd.cmdRespBuff[0]) =
				(uint16_t) g_Timer.getMsFromTicks(g_phySensors[pCmd->params[0]]->period);
		}
		else {
			g_hostif_cmd.cmdRespBuff[0] = 0xFF;
			g_hostif_cmd.cmdRespBuff[1] = 0xFF;
		}
		/* Queue DMA transmission */
		Hostif_StartTx(&g_hostif_cmd.cmdRespBuff[0], 2, pCmd->id);
		Hostif_AssertIRQ();
		break;
		
	default:
		break;
	}

	return 0;
}

uint8_t Hostif_isSensorEnable(uint8_t sensorId)
{
	if (sensorId < LPCSH_SENSOR_ID_COUNT) {
		return (g_hostif_cmd.sensorEnable & (1L << sensorId)) ? 1 : 0;
	}
	return 0;
}

uint8_t Hostif_QueueBuffer(const uint8_t *pBuffer, uint16_t length)
{
	/* Increased length by 2 to include SensorId and compression bytes */
	length += sizeof(struct lpcsh_sensor_node_header);
	writeRingBuf(pBuffer, length);
	/* if SPI state is idle, then queue transmission */
	if (Hostif_TxReady()) {
		Hostif_AssertIRQ();
		startPhyTx();
	}

	return 0;
}

/* Function to check for any queued Tx */
void Hostif_CheckTx(void)
{
	if ((g_hostif_cmd.wrPtr != g_hostif_cmd.rdPtr) && (Hostif_TxReady())) {
		Hostif_AssertIRQ();
		startPhyTx();
	}
}

#endif

