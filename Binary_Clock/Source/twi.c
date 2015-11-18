/****************************************************************************
* twi.c
*
* Two wire interface protocol library for Atmel AVR micro controllers.
* Supports transmitting and receiving of data over the two wire interface.
*
*   Copyright (C) 2015 Michael Williamson. All rights reserved.
*   Authors: Michael Williamson <mikesmodz@gmail.com>.
*   Based on TWI/I2C library for Wiring & Arduino by Nicholas Zambetti.
*   Modified 2012 by Todd Krein <todd@krein.org>.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/

/****************************************************************************
* Included Files
****************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <util/delay.h>
#include "avrlibtypes.h"
#include "twi.h"
#include <util/twi.h>

/****************************************************************************
* Private Function Prototypes
****************************************************************************/
static void (*TWI_OnSlaveTransmit)(void);
static void (*TWI_OnSlaveReceive)(u08*, int);

/****************************************************************************
* Private Data
****************************************************************************/
static volatile u08 TWI_State;
static volatile u08 TWI_SlaveRW;
static volatile u08 TWI_SendStop;
static volatile u08 TWI_InRepStart;
static u08 TWI_MasterBuffer[TWI_BUFFER_LENGTH];
static volatile u08 TWI_MasterBufferIndex;
static volatile u08 TWI_MasterBufferLength;
static u08 TWI_TxBuffer[TWI_BUFFER_LENGTH];
static volatile u08 TWI_TxBufferIndex;
static volatile u08 TWI_TxBufferLength;
static uint8_t TWI_RxBuffer[TWI_BUFFER_LENGTH];
static volatile uint8_t TWI_RxBufferIndex;
static volatile u08 TWI_Error;

/****************************************************************************
* Public Functions
****************************************************************************/

/****************************************************************************
* Name: TWI_Init
*
* Description:
* 	Initialises the TWI hardware.
*
* Input Parameters:
* 	None.
*
* Returned Value:
* 	Nothing.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
void TWI_Init(void)
{
	/* Initialise states */
	TWI_State = TWI_STATE_READY;
	TWI_SendStop = TRUE;
	TWI_InRepStart = FALSE;

	/* Enable pull ups */
	TWI_PORT |= _BV(TWI_SDA_PIN);
	TWI_PORT |= _BV(TWI_SCL_PIN);

	/* Initialise prescaler and bitrate */
	cbi(TWI_STATUS, TWI_PRESCALER_0);
	cbi(TWI_STATUS, TWI_PRESCALER_1);
	TWI_BIT_RATE = ((F_CPU / TWI_FREQUENCY) - 16) / 2;

	/* Enable TWI, ACK and interrupt */
	TWI_CONTROL = _BV(TWI_ENABLE) |
				  _BV(TWI_INTERRUPT_ENABLE) |
				  _BV(TWI_ENABLE_ACK);
}

/****************************************************************************
* Name: TWI_SetSlaveAddress
*
* Description:
* 	Sets the TWI slave address.
*
* Input Parameters:
* 	Address = Slave address.
*
* Returned Value:
* 	Nothing.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
void TWI_SetSlaveAddress(u08 Address)
{
	TWI_SLAVE_ADDRESS = Address << 1;
}

/****************************************************************************
* Name: TWI_ReadFrom
*
* Description:
* 	Sets TWI master and reads a number of bytes from a device on the bus.
*
* Input Parameters:
* 	Address = 7 bit device address.
*	*Data = Pointer to array of data to read to.
*	Length = Number of bytes to read.
*	SendStop = Send stop command at end TRUE or FALSE.
*
* Returned Value:
* 	Number of bytes read.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
u08 TWI_ReadFrom(u08 Address, u08* Data, u08 Length, u08 SendStop)
{
	u08 i = 0;

	/* Check data will fit */
	if (Length > TWI_BUFFER_LENGTH)
	{
		return 0;
	}

	/* Wait for TWI become ready */
	while (TWI_State != TWI_STATE_READY)
	{
		continue;
	}

	TWI_State = TWI_STATE_MASTER_RX;
	TWI_SendStop = SendStop;

	/* Reset error state */
	TWI_Error = 0xFF;

	/* Initialise buffer variables */
	TWI_MasterBufferIndex = 0;
	TWI_MasterBufferLength = Length - 1;

	/* Set slave device address */
	TWI_SlaveRW = TW_READ;
	TWI_SlaveRW |= Address << 1;

	if (TWI_InRepStart)
	{
		TWI_DATA = TWI_SlaveRW;
		TWI_CONTROL = _BV(TWI_INTERRUPT) | 
					  _BV(TWI_ENABLE_ACK) |
					  _BV(TWI_ENABLE) |
					  _BV(TWI_INTERRUPT_ENABLE);
	}
	else
	{
		TWI_CONTROL = _BV(TWI_ENABLE) |
					  _BV(TWI_INTERRUPT_ENABLE) |
					  _BV(TWI_ENABLE_ACK) |
					  _BV(TWI_INTERRUPT) |
					  _BV(TWSTA);
	}

	/* Wait for completion */
	while (TWI_State == TWI_STATE_MASTER_RX)
	{
		continue;
	}

	/* Check length */
	if (TWI_MasterBufferIndex < Length)
	{
		Length = TWI_MasterBufferIndex;
	}

	/* Copy data */
	for (i = 0; i < Length; i++)
	{
		Data[i] = TWI_MasterBuffer[i];
	}

	return Length;
}

/****************************************************************************
* Name: TWI_WriteTo
*
* Description:
* 	Sets TWI master and writes a number of bytes to a device on the bus.
*
* Input Parameters:
* 	Address = 7 bit device address.
*	*Data = Pointer to array of data to write from.
*	Length = Number of bytes to write.
*	SendStop = Send stop command at end TRUE or FALSE.
*
* Returned Value:
* 	Status: 0 = Success.
*			1 = Length to long.
*			2 = Address send, NACK received.
*			3 = Data send, NACK received.
*			4 = Other error.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
u08 TWI_WriteTo(u08 Address, const u08* Data, u08 Length, u08 Wait, u08 SendStop)
{
	u08 i = 0;

	/* Check data will fit */
	if (Length > TWI_BUFFER_LENGTH)
	{
		return 0;
	}

	/* Wait for TWI become ready */
	while (TWI_State != TWI_STATE_READY)
	{
		continue;
	}

	TWI_State = TWI_STATE_MASTER_TX;
	TWI_SendStop = SendStop;

	/* Reset error state */
	TWI_Error = 0xFF;

	/* Initialise buffer variables */
	TWI_MasterBufferIndex = 0;
	TWI_MasterBufferLength = Length;

	/* Copy data */
	for (i = 0; i < Length; i++)
	{
		TWI_MasterBuffer[i] = Data[i];
	}

	/* Set slave device address */
	TWI_SlaveRW = TW_WRITE;
	TWI_SlaveRW |= Address << 1;

	if (TWI_InRepStart)
	{
		TWI_InRepStart = FALSE;
		TWI_DATA = TWI_SlaveRW;
		TWI_CONTROL = _BV(TWI_INTERRUPT) |
					  _BV(TWI_ENABLE_ACK) |
					  _BV(TWI_ENABLE) |
					  _BV(TWI_INTERRUPT_ENABLE);
	}
	else
	{
		TWI_CONTROL = _BV(TWI_INTERRUPT) |
					  _BV(TWI_ENABLE_ACK) |
					  _BV(TWI_ENABLE) |
					  _BV(TWI_INTERRUPT_ENABLE) |
					  _BV(TWI_START);
	}

	/* Wait for operation to complete */
	while (Wait && (TWI_State == TWI_STATE_MASTER_TX))
	{
		continue;
	}

	if (TWI_Error == 0xFF)
	{
		/* OK */
		return 0;
	}
	else if (TWI_Error == TW_MT_SLA_NACK)
	{
		/* Address send, nack received */
		return 2;
	}

	else if (TWI_Error == TW_MT_DATA_NACK)
	{
		/* Data send, nack received */
		return 3;
	}
	else
	{
		/* Other error */
		return 4;
	}
}

/****************************************************************************
* Name: TWI_Transmit
*
* Description:
* 	Fills the slave TX buffer with data. Must be called in slave TX 
*	event callback.
*
* Input Parameters:
* 	*Data = Pointer to array of data to read from.
*	Length = Number of bytes to read.
*
* Returned Value:
* 	Status: 0 = Success.
*			1 = Length to long.
*			2 = Not slave transmitter.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
u08 TWI_Transmit(const u08* Data, u08 Length)
{
	u08 i = 0;

	/* Check data will fit */
	if (Length > TWI_BUFFER_LENGTH)
	{
		return 1;
	}

	/* Ensure slave TX */
	if (TWI_State != TWI_STATE_SLAVE_TX)
	{
		return 2;
	}

	/* Set length */
	TWI_TxBufferLength = Length;

	/* Copy data */
	for (i = 0; i < Length; i++)
	{
		TWI_TxBuffer[i] = Data[i];
	}

	return 0;
}

/****************************************************************************
* Name: TWI_AttachSlaveRxEvent
*
* Description:
* 	Sets function called before a slave read operation.
*
* Input Parameters:
* 	Pointer to call back function.
*
* Returned Value:
* 	Nothing.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
void TWI_AttachSlaveRxEvent(void (*function)(uint8_t*, int))
{
	TWI_OnSlaveReceive = function;
}

/****************************************************************************
* Name: TWI_AttachSlaveTxEvent
*
* Description:
* 	Sets function called before a slave write operation.
*
* Input Parameters:
* 	Pointer to call back function.
*
* Returned Value:
* 	Nothing.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
void TWI_AttachSlaveTxEvent(void (*function)(void))
{
	TWI_OnSlaveTransmit = function;
}

/****************************************************************************
* Name: TWI_Reply
*
* Description:
* 	Sends a byte with or without acknowledge.
*
* Input Parameters:
* 	Ack = TRUE or FALSE.
*
* Returned Value:
* 	Nothing.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
void TWI_Reply(u08 Ack)
{
	if (Ack)
	{
		/* Transmit master ready with acknowledge */
		TWI_CONTROL = _BV(TWI_ENABLE) |
					  _BV(TWI_INTERRUPT_ENABLE) |
					  _BV(TWI_INTERRUPT) |
					  _BV(TWI_ENABLE_ACK);
	}
	else
	{
		/* Transmit master ready with without acknowledge */
		TWI_CONTROL = _BV(TWI_ENABLE) |
					  _BV(TWI_INTERRUPT_ENABLE) |
					  _BV(TWI_INTERRUPT);
	}
}

/****************************************************************************
* Name: TWI_Stop
*
* Description:
* 	Sends a stop condition.
*
* Input Parameters:
* 	None.
*
* Returned Value:
* 	Nothing.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
void TWI_Stop(void)
{
	/* Send stop */
	TWI_CONTROL = _BV(TWI_ENABLE) |
				  _BV(TWI_INTERRUPT_ENABLE) |
				  _BV(TWI_ENABLE_ACK) |
				  _BV(TWI_INTERRUPT) |
				  _BV(TWI_STOP);

	/* Wait for stop to be executed */
	while (TWI_CONTROL & _BV(TWI_STOP))
	{
		continue;
	}

	/* Update TWI state */
	TWI_State = TWI_STATE_READY;
}

/****************************************************************************
* Name: TWI_ReleaseBus
*
* Description:
* 	Releases the bus.
*
* Input Parameters:
* 	None.
*
* Returned Value:
* 	Nothing.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
void TWI_ReleaseBus(void)
{
	/* Release the bus */
	TWI_CONTROL = _BV(TWI_ENABLE) |
				  _BV(TWI_INTERRUPT_ENABLE) |
				  _BV(TWI_ENABLE_ACK) |
				  _BV(TWI_INTERRUPT);

	/* Update TWI state */
	TWI_State = TWI_STATE_READY;
}

/****************************************************************************
* Name: TWI_vect
*
* Description:
* 	Services the TWI interrupt.
*
* Input Parameters:
* 	None.
*
* Returned Value:
* 	Nothing.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
ISR(TWI_vect)
{
	switch (TW_STATUS)
	{
		/* All Master */
		case TW_START:					/* Sent start condition */
		case TW_REP_START:				/* Sent repeated start condition */

			/* Copy device address and r/w bit to 
			   output register and ack */
			TWI_DATA = TWI_SlaveRW;
			TWI_Reply(1);

			break;

		/* Master Transmitter */
		case TW_MT_SLA_ACK:				/* Slave receiver acked address */
		case TW_MT_DATA_ACK:			/* Slave receiver acked data */

			/* If there is data to send, send it, otherwise stop */
			if (TWI_MasterBufferIndex < TWI_MasterBufferLength)
			{
				/* Copy data to output register and ack */
				TWI_DATA = TWI_MasterBuffer[TWI_MasterBufferIndex++];
				TWI_Reply(1);
			}
			else
			{
				if (TWI_SendStop)
				{
					TWI_Stop();
				}
				else
				{
					/* We're gonna send the START */
					TWI_InRepStart = TRUE;
					TWI_CONTROL = _BV(TWI_INTERRUPT) |
								  _BV(TWI_START) |
								  _BV(TWI_ENABLE);
					TWI_State = TWI_STATE_READY;
				}
			}

			break;

		/* Address sent, nack received */
		case TW_MT_SLA_NACK:

			TWI_Error = TW_MT_SLA_NACK;
			TWI_Stop();

			break;

		/* Data sent, nack received */
		case TW_MT_DATA_NACK:

			TWI_Error = TW_MT_DATA_NACK;
			TWI_Stop();

			break;

		/* Lost bus arbitration */
		case TW_MT_ARB_LOST:

			TWI_Error = TW_MT_ARB_LOST;
			TWI_ReleaseBus();

			break;

		/* Master Receiver */
		case TW_MR_DATA_ACK:			/* Data received, ack sent */

			/* Save data */
			TWI_MasterBuffer[TWI_MasterBufferIndex++] = TWI_DATA;

		/* Address sent, ack received */
		case TW_MR_SLA_ACK:

			/* Ack if more bytes are expected, otherwise nack */
			if (TWI_MasterBufferIndex < TWI_MasterBufferLength)
			{
				TWI_Reply(1);
			}
			else
			{
				TWI_Reply(0);
			}

			break;

		/* Data received, nack sent */
		case TW_MR_DATA_NACK:

			/* Put final byte into buffer */
			TWI_MasterBuffer[TWI_MasterBufferIndex++] = TWI_DATA;

			if (TWI_SendStop)
			{
				TWI_Stop();
			}
			else
			{
				/* We're gonna send the START */
				TWI_InRepStart = TRUE;
				TWI_CONTROL = _BV(TWI_INTERRUPT) |
							  _BV(TWI_START) |
							  _BV(TWI_ENABLE);
				TWI_State = TWI_STATE_READY;
			}

			break;

		/* Address sent, nack received */
		case TW_MR_SLA_NACK:

			TWI_Stop();

			break;

			/* TW_MR_ARB_LOST handled by TW_MT_ARB_LOST case */

		/* Slave Receiver */
		case TW_SR_SLA_ACK:				/* Addressed, returned ack */
		case TW_SR_GCALL_ACK:			/* Addressed generally, returned ack */
		case TW_SR_ARB_LOST_SLA_ACK:	/* Lost arbitration, returned ack */
		case TW_SR_ARB_LOST_GCALL_ACK:	/* Lost arbitration, returned ack */

			/* Enter slave receiver mode */
			TWI_State = TWI_STATE_SLAVE_RX;

			/* Indicate that rx buffer can be overwritten and ack */
			TWI_RxBufferIndex = 0;
			TWI_Reply(1);

			break;

		case TW_SR_DATA_ACK:			/* Data received, returned ack */
		case TW_SR_GCALL_DATA_ACK:		/* Data received generally, returned ack */

			/* If there is still room in the rx buffer */
			if (TWI_RxBufferIndex < TWI_BUFFER_LENGTH)
			{
				/* Put byte in buffer and ack */
				TWI_RxBuffer[TWI_RxBufferIndex++] = TWI_DATA;
				TWI_Reply(1);
			}
			else
			{
				/* Otherwise nack */
				TWI_Reply(0);
			}

			break;

		/* Stop or repeated start condition received */
		case TW_SR_STOP:

			/* Put a null char after data if there's room */
			if (TWI_RxBufferIndex < TWI_BUFFER_LENGTH)
			{
				TWI_RxBuffer[TWI_RxBufferIndex] = '\0';
			}

			/* Sends ack and stops interface for clock stretching */
			TWI_Stop();

			/* Callback to user defined callback */
			TWI_OnSlaveReceive(TWI_RxBuffer, TWI_RxBufferIndex);

			/* Since we submit rx buffer to "wire" library, we can reset it */
			TWI_RxBufferIndex = 0;

			/* Ack future responses and leave slave receiver state */
			TWI_ReleaseBus();

			break;

		case TW_SR_DATA_NACK:			/* Data received, returned nack */
		case TW_SR_GCALL_DATA_NACK:		/* Data received generally, returned nack */

			/* Nack back at master */
			TWI_Reply(0);

			break;

		/* Slave Transmitter */
		case TW_ST_SLA_ACK:				/* Addressed, returned ack */
		case TW_ST_ARB_LOST_SLA_ACK:	/* Arbitration lost, returned ack */

			/* Enter slave transmitter mode */
			TWI_State = TWI_STATE_SLAVE_TX;

			/* Ready the tx buffer index for iteration */
			TWI_TxBufferIndex = 0;

			/* Set tx buffer length to be zero, to verify if user changes it */
			TWI_TxBufferLength = 0;

			/* Request for txBuffer to be filled and length to be set
			   note: user must call twi_transmit(bytes, length) to do this */
			TWI_OnSlaveTransmit();

			/* If they didn't change buffer & length, initialize it */
			if (TWI_TxBufferLength == 0)
			{
				TWI_TxBufferLength = 1;
				TWI_TxBuffer[0] = 0x00;
			}

		/* Transmit first byte from buffer, fall */
		case TW_ST_DATA_ACK:			/* Byte sent, ack returned */

			/* Copy data to output register */
			TWI_DATA = TWI_TxBuffer[TWI_TxBufferIndex++];

			/* If there is more to send, ack, otherwise nack */
			if (TWI_TxBufferIndex < TWI_TxBufferLength)
			{
				TWI_Reply(1);
			}
			else
			{
				TWI_Reply(0);
			}

			break;

		case TW_ST_DATA_NACK:			/* Received nack, we are done */
		case TW_ST_LAST_DATA:			/* Received ack, but we are done already! */

			/* Ack future responses */
			TWI_Reply(1);

			/* Leave slave receiver state */
			TWI_State = TWI_STATE_READY;

			break;

			/* All */
		case TW_NO_INFO:				/* No state information */
			break;

		case TW_BUS_ERROR:				/* Bus error, illegal stop/start */

			TWI_Error = TW_BUS_ERROR;
			TWI_Stop();

			break;
	}
}
