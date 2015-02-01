/*
 * @brief Host interface module
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
#include "hostif_protocol.h"

#ifndef _HOSTIF_H_
#define _HOSTIF_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup SH_HOSTIF Sensor Hub: Host Interface
 * @ingroup SENSOR_HUB
 * @{
 */

#define HOSTIF_MAX_BUFFER_SIZE		256
#define HOSTIF_DUMMY_PACKET_ID		0x00

/**
 * @brief Host Interface Command Structure
 */
typedef struct __HOSTIF_CMD {
	uint8_t id;
	uint8_t params[LPCSH_MAX_CMD_LENGTH - 1];
} HOSTIF_CMD_t;

/**
 * @brief	Initialize host interface subsystem
 * @return	none
 */
void Hostif_Init(void);

/**
 * @brief	Start Host interface transfer
 * @param	pBuf	: Pointer to buffer to be transmitted
 * @param	size	: Size of the buffer
 * @param	cmd		: Last command received.
 * @return	none
 * @note The "cmd" param specifies the command Id for which
 *  this response is sent. For transmitting notification data this param
 *  should be set to 0. This is used by the SPI Host interface during 
 *	handshaking with host.
 */
void Hostif_StartTx(uint8_t *pBuf, uint16_t size, uint8_t cmd);

/**
 * @brief	Check if physical interface is ready for transmit. This is used for SPI Host interface.
 * @return	Returns TRUE if ready else FALSE.
 */
uint8_t Hostif_TxReady(void);

/**
 * @brief	Tells caller if the host transmission is complete and if it is ok to sleep
 * @return	0 - Ok to Sleep.
 *          1 - Not ok to sleep (Hostif Tx might be pending);
 */
uint8_t Hostif_SleepOk(void);

/**
 * @brief	Assert interrupt line to low to indicate Host/AP that there is data to read
 * @return	none
 */
static INLINE void Hostif_AssertIRQ(void)
{
	Chip_GPIO_SetPinState(LPC_GPIO, HOSTIF_IRQ_PORT, HOSTIF_IRQ_PIN, 0);
}

/**
 * @brief	De-assert interrupt line to high to indicate Host/AP that there is no data to read
 * @return	none
 */
static INLINE void Hostif_DeassertIRQ(void)
{
	Chip_GPIO_SetPinState(LPC_GPIO, HOSTIF_IRQ_PORT, HOSTIF_IRQ_PIN, 1);
}

/**
 * @brief	Checks and returns state of interrupt line to Host/AP
 * @return	Returns TRUE if interrupt line is asserted low, else returns FALSE
 */
static INLINE bool Hostif_IRQActive(void)
{
	return Chip_GPIO_GetPinState(LPC_GPIO, HOSTIF_IRQ_PORT, HOSTIF_IRQ_PIN) == false;
}



/*------------------------------------------------------------------
    Host interface routines exposed by other frameworks
   ------------------------------------------------------------------*/
/**
 * @brief	Initialize HOST I/F command processing engine.
 * @return	none
 */
void Hostif_CmdEngineInit(void);

/**
 * @brief	Process the host command
 * @param	rx_buf	: Pointer to buffer received from host
 * @param	length	: Size of the buffer
 * @return	Returns the size of parameter buffer to be received after the command.
 *			Returns 0 if no more data is needed. Note that this is used for I2C
 */
uint8_t Hostif_CmdProcess(uint8_t *rx_buf, uint16_t length);

/**
 * @brief	Checks if given virtual sensor is enabled
 * @param	sensorId	: Virtual sensor ID.
 * @return	0 - sensor is disabled; 1 - sensor is enabled
 */
uint8_t Hostif_isSensorEnable(uint8_t sensorId);

/**
 * @brief	Add the given buffer to transmit queue
 * @param	pBuffer	: Pointer to buffer.
 * @param	length	: Size of the buffer.
 * @return	Returns error code.
 *		0 - successful; non-zero - Tx queue is full
 */
uint8_t Hostif_QueueBuffer(const uint8_t *pBuffer, uint16_t length);

/**
 * @brief	Callback for Transmit completion.
 * @return	none
 * @note Callback routine called by the low-level hostIF driver(I2C only)
 *		when it completes the transfer of given buffer.
 */
void Hostif_TxCompleteCB(void);

/**
 * @brief	This function checks if there is data to be transmitted.
 * @return	none
 * @note Callback routine called by the low-level hostIF driver(SPI Only)
 *		to check if there is anything to transmit if it will start transmission.
 */
void Hostif_CheckTx(void);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif  /* _HOSTIF_H_ */
