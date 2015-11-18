/****************************************************************************
* twi.h
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
#ifndef __TWI_H__
#define __TWI_H__

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
 * Pre-processor Definitions
****************************************************************************/

/* TWI frequency */
#define TWI_FREQUENCY					400000L

#ifndef TWI_FREQUENCY
 #define TWI_FREQUENCY					100000L
#endif

/* TWI buffer length */
#ifndef TWI_BUFFER_LENGTH
 #define TWI_BUFFER_LENGTH				32
#endif

/* TWI hardware definitions */
#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
 #define TWI_BIT_RATE					TWBR
 #define TWI_STATUS						TWSR
 #define TWI_SLAVE_ADDRESS				TWAR
 #define TWI_CONTROL					TWCR
 #define TWI_DATA						TWDR
 #define TWI_ENABLE						TWEN
 #define TWI_INTERRUPT					TWINT
 #define TWI_INTERRUPT_ENABLE			TWIE
 #define TWI_START						TWSTA
 #define TWI_STOP						TWSTO
 #define TWI_ENABLE_ACK					TWEA
 #define TWI_PRESCALER_1				TWPS1
 #define TWI_PRESCALER_0				TWPS0
 #define TWI_PORT						PORTD
 #define TWI_SDA_PIN					PD1
 #define TWI_SCL_PIN					PD0
#elif defined(__AVR_ATmega328P__)
 #define TWI_BIT_RATE					TWBR
 #define TWI_STATUS						TWSR
 #define TWI_SLAVE_ADDRESS				TWAR
 #define TWI_CONTROL					TWCR
 #define TWI_DATA						TWDR
 #define TWI_ENABLE						TWEN
 #define TWI_INTERRUPT					TWINT
 #define TWI_INTERRUPT_ENABLE			TWIE
 #define TWI_START						TWSTA
 #define TWI_STOP						TWSTO
 #define TWI_ENABLE_ACK					TWEA
 #define TWI_PRESCALER_1				TWPS1
 #define TWI_PRESCALER_0				TWPS0
 #define TWI_PORT						PORTC
 #define TWI_SDA_PIN					PC4
 #define TWI_SCL_PIN					PC5
#else
 #error "No TWI definition for MCU available"
#endif

/* TWI communication states */
#define TWI_STATE_READY					0
#define TWI_STATE_MASTER_RX				1
#define TWI_STATE_MASTER_TX				2
#define TWI_STATE_SLAVE_RX				3
#define TWI_STATE_SLAVE_TX				4

/* Clear bit macro */
#ifndef cbi
 #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

/* Set bit macro */
#ifndef sbi
 #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

/****************************************************************************
* Public Function Prototypes
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
void TWI_Init(void);

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
void TWI_SetSlaveAddress(u08 Address);

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
u08 TWI_ReadFrom(u08 Address, u08* Data, u08 Length, u08 SendStop);

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
u08 TWI_WriteTo(u08 Address, const u08* Data, u08 Length, u08 Wait, u08 SendStop);

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
u08 TWI_Transmit(const u08* Data, u08 Length);

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
void TWI_AttachSlaveRxEvent(void (*function)(uint8_t*, int));

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
void TWI_AttachSlaveTxEvent(void (*function)(void));

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
void TWI_Reply(u08 Ack);

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
void TWI_Stop(void);

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
void TWI_ReleaseBus(void);

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
ISR(TWI_vect);

#endif /* __TWI_H__ */
