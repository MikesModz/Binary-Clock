/****************************************************************************
* ds3231.c
*
* Maxim DS3231 Real Time Clock (RTC) library for Atmel AVR micro controllers.
* Supports setting and reading of current date and time. Setting of alarms 1
* and 2. Reading of current temperature.
*
*   Copyright (C) 2015 Michael Williamson. All rights reserved.
*   Authors: Michael Williamson <mikesmodz@gmail.com>
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
#include "ds3231.h"
#include "twi.h"

/****************************************************************************
* Private Function Prototypes
****************************************************************************/
static u08 DS3231_WriteRegisters(u08 DestRegister, u08 NoRegisters, const u08 *ptrRegisters);
static u08 DS3231_ReadRegisters(u08 SourceRegister, u08 NoRegisters, u08 *ptrRegisters);

/****************************************************************************
* Private Functions
****************************************************************************/

/****************************************************************************
* Name: DS3231_WriteRegisters
*
* Description:
* 	Writes to DS3231 registers.
*
* Input Parameters:
*	DestRegister = Start write register address.
*	NoRegisters = Number of registers to write.
*	*ptrRegisters = Pointer to buffer containing data to write.
*
* Returned Value:
*	Success = TRUE Failure = FALSE.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
static u08 DS3231_WriteRegisters(u08 DestRegister, u08 NoRegisters, const u08 *ptrRegisters)
{
	u08 buffer[20] = {0};
	u08 i = 0;
	
	buffer[0] = DestRegister;
	for (i=0; i<NoRegisters; i++)
	{
		buffer[i+1] = *ptrRegisters++;
	}
	if (TWI_WriteTo(DS3231_SLAVE_ADDRESS, buffer, NoRegisters+1, TRUE, TRUE))
	{
		return FALSE;	
	}
	
	return TRUE;
}

/****************************************************************************
* Name: DS3231_ReadRegisters
*
* Description:
* 	Reads from DS3231 registers.
*
* Input Parameters:
*	SourceRegister = Start read register address.
*	NoRegisters = Number of registers to read.
*	*ptrRegisters = Pointer to buffer to hold data read.
*
* Returned Value:
*	Success = TRUE Failure = FALSE.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
static u08 DS3231_ReadRegisters(u08 SourceRegister, u08 NoRegisters, u08 *ptrRegisters)
{
	u08 buffer[2] = {0};
	
	buffer[0] = SourceRegister;
	if (TWI_WriteTo(DS3231_SLAVE_ADDRESS, buffer, 1, TRUE, TRUE))
	{
		return FALSE;
	}
	if (TWI_ReadFrom(DS3231_SLAVE_ADDRESS, ptrRegisters, NoRegisters, TRUE)
	!= NoRegisters )
	{
		return FALSE;
	}
	return TRUE;
}

/****************************************************************************
* Public Functions
****************************************************************************/

/****************************************************************************
* Name: DS3231_BIN2BCD
*
* Description:
* 	Converts a value from binary to BCD.
*
* Input Parameters:
*	inByte = Value to convert.
*
* Returned Value:
*	BCD result.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
u08 DS3231_BIN2BCD(u08 inByte)
{
	u08 temp = inByte%10;
	temp = ((inByte/10)<<4)|temp;
	return temp;
}

/****************************************************************************
* Name: DS3231_BCD2BIN
*
* Description:
* 	Converts a value from BCD to binary.
*
* Input Parameters:
*	inByte = Value to convert.
*
* Returned Value:
*	Binary result.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
u08 DS3231_BCD2BIN(u08 inByte)
{
	u08 temp = inByte&0x0F;
	temp = (((inByte&0xF0)>>4)*10)+temp;
	return temp;
}

/****************************************************************************
* Name: DS3231_Init
*
* Description:
* 	Initialises the DS3231 device.
*
* Input Parameters:
*	None.
*
* Returned Value:
*	Success = TRUE Failure = FALSE.
*
* Assumptions/Limitations:
*	Assumes the TWI interface has been initialised.
****************************************************************************/
u08 DS3231_Init(void)
{
	u08 temp = 0;
	u08 i = 0;
	
	/* Initialise the control register. Set INTCN = 1 enables assertion 
	   of INT/SQW pin with match between time keeping registers and 
	   alarm register. */
	temp = _BV(DS3231_CONTROL_BIT_INTCN);
	if (!DS3231_WriteRegisters(DS3231_REG_CONTROL, 1, (const u08*)&temp))
	{
		/* Return send failure */
		return FALSE;
	}
		
	/* Small delay */
	_delay_ms(10);
		
	/* Initialise the status register */
	temp = 0;
	if (!DS3231_WriteRegisters(DS3231_REG_STATUS, 1, (const u08*)&temp))
	{
		/* Return send failure */
		return FALSE;
	}
		
	/* Small delay */
	_delay_ms(10);
		
	/* Clear the alarm registers */
	temp = 0;
	for (i=0; i<7; i++)
	{
		if (!DS3231_WriteRegisters(DS3231_REG_A1_SECS+i, 1, (const u08*)&temp))
		{
			/* Return send failure */
			return FALSE;
		}
	}
		
	/* Read hours register contents */
	if (!DS3231_ReadRegisters(DS3231_REG_HOURS, 1, (u08*)&temp))
	{
		/* Return send failure */
		return FALSE;
	}
		
	/* Configure 24 hour mode */
	temp &= ~_BV(DS3231_HOURS_12_24);
		
	/* Write hours register contents */
	if (!DS3231_WriteRegisters(DS3231_REG_HOURS, 1, (const u08*)&temp))
	{
		/* Return send failure */
		return FALSE;
	}
		
	/* Small delay */
	_delay_ms(10);
		
	/* Return send success */
	return TRUE;
}

/****************************************************************************
* Name: DS3231_SetTime
*
* Description:
* 	Sets the current date and time.
*
* Input Parameters:
*	*ptrSetTime = Pointer to structure containing the current time.
*
* Returned Value:
*	Success = TRUE Failure = FALSE.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
u08 DS3231_SetTime(DS3231_TimeTypeDef *ptrSetTime)
{	
	/* Variable to store the DS3231 register contents */
	u08 tempRTCRegs[7] = {0};
	
	/* Convert these time and dates to BCD */
	tempRTCRegs[0] = DS3231_BIN2BCD(ptrSetTime->Secs);
	tempRTCRegs[1] = DS3231_BIN2BCD(ptrSetTime->Mins);
	tempRTCRegs[2] = DS3231_BIN2BCD(ptrSetTime->Hours) & 0b10111111;
	tempRTCRegs[3] = DS3231_BIN2BCD(ptrSetTime->Day);
	tempRTCRegs[4] = DS3231_BIN2BCD(ptrSetTime->Date);
	tempRTCRegs[5] = DS3231_BIN2BCD(ptrSetTime->Month);
	tempRTCRegs[6] = DS3231_BIN2BCD(ptrSetTime->Year - 2000);
	
	/* Write all the time & date registers */
	if (!DS3231_WriteRegisters(DS3231_REG_SECS, 7, (const u08*)&tempRTCRegs))
	{
		/* Return send failure */
		return FALSE;
	}

	/* Return send success */
	return TRUE;
}

/****************************************************************************
* Name: DS3231_GetTime
*
* Description:
* 	Gets the current date and time.
*
* Input Parameters:
*	*ptrSetTime = Pointer to structure to save the current time.
*
* Returned Value:
*	Success = TRUE Failure = FALSE.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
u08 DS3231_GetTime(DS3231_TimeTypeDef *ptrGetTime)
{
	/* Variable to store the DS3231 register contents */
	u08 tempRTCRegs[7] = {0};

	/* Read all the time & date registers */
	if (!DS3231_ReadRegisters(DS3231_REG_SECS, 7, (u08*)&tempRTCRegs))
	{
		/* Return send failure */
		return FALSE;
	}
	
	/* Convert these time and dates to binary */
	ptrGetTime->Secs = DS3231_BCD2BIN(tempRTCRegs[0]);
	ptrGetTime->Mins = DS3231_BCD2BIN(tempRTCRegs[1]);
	ptrGetTime->Hours = DS3231_BCD2BIN(tempRTCRegs[2] & ~0b11000000);
	ptrGetTime->Day = DS3231_BCD2BIN(tempRTCRegs[3]);
	ptrGetTime->Date = DS3231_BCD2BIN(tempRTCRegs[4]);
	ptrGetTime->Month = DS3231_BCD2BIN(tempRTCRegs[5]);
	ptrGetTime->Year = DS3231_BCD2BIN(tempRTCRegs[6]) + 2000;

	/* Return read success */
	return TRUE;
}

/****************************************************************************
* Name: DS3231_GetTemperature
*
* Description:
* 	Gets the current temperature.
*
* Input Parameters:
*	*ptrTemperature = Pointer to variable to save temperature to.
*
* Returned Value:
*	Success = TRUE Failure = FALSE.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
u08 DS3231_GetTemperature(float *ptrTemperature)
{
	/* Variable to store the DS3231 register contents */
	u08 tempRTCRegs[2] = {0};
		
	/* Temperature */
	float temp = 0.0;
		
	/* Read all the time & date registers */
	if (!DS3231_ReadRegisters(DS3231_REG_TEMP_MSB, 2, (u08*)&tempRTCRegs))
	{
		/* Return send failure */
		return FALSE;
	}
	
	/* If temperature is negative */
	if (tempRTCRegs[0] & 0b10000000)
	{
		tempRTCRegs[0] ^= 0b11111111;	 
		tempRTCRegs[0] += 0x01;
		temp = tempRTCRegs[0] + ((tempRTCRegs[1] >> 6) * 0.25);
		temp *= -1.0;
	}
	else
	{
		temp = tempRTCRegs[0] + ((tempRTCRegs[1] >> 6) * 0.25);
	}
	
	/* Save result */
	*ptrTemperature = temp;
		
	/* Return read success */
	return TRUE;
}

/****************************************************************************
* Name: DS3231_EnableAlarm1
*
* Description:
* 	Configures alarm 1.
*
* Input Parameters:
*	*ptrAlarmSetting = Pointer to structure containing alarm configuration.
*
* Returned Value:
*	Success = TRUE Failure = FALSE.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
u08 DS3231_EnableAlarm1(DS3231_Alarm1TypeDef *ptrAlarmSetting)
{	
	/* Variable to store the DS3231 register contents */
	u08 tempRTCRegs[4] = {0};
	
	/* Temporary variable */
	u08 temp = 0;
	
	/* Convert these time and dates to BCD */
	tempRTCRegs[0] = DS3231_BIN2BCD(ptrAlarmSetting->Secs);
	tempRTCRegs[1] = DS3231_BIN2BCD(ptrAlarmSetting->Mins);
	tempRTCRegs[2] = DS3231_BIN2BCD(ptrAlarmSetting->Hours) & 0b10111111;
	tempRTCRegs[3] = DS3231_BIN2BCD(ptrAlarmSetting->Day);
	
	/* Select alarm type */
	switch (ptrAlarmSetting->AlarmType)
	{
		case ALARM1_EverySecond:
			tempRTCRegs[0] |= _BV(DS3231_ALARM_ENABLE_MASK);
			tempRTCRegs[1] |= _BV(DS3231_ALARM_ENABLE_MASK);
			tempRTCRegs[2] |= _BV(DS3231_ALARM_ENABLE_MASK);
			tempRTCRegs[3] |= _BV(DS3231_ALARM_ENABLE_MASK);
		break;
		
		case ALARM1_MatchSeconds:
			tempRTCRegs[1] |= _BV(DS3231_ALARM_ENABLE_MASK);
			tempRTCRegs[2] |= _BV(DS3231_ALARM_ENABLE_MASK);
			tempRTCRegs[3] |= _BV(DS3231_ALARM_ENABLE_MASK);
		break;
		
		case ALARM1_MatchMinutesSeconds:
			tempRTCRegs[2] |= _BV(DS3231_ALARM_ENABLE_MASK);
			tempRTCRegs[3] |= _BV(DS3231_ALARM_ENABLE_MASK);
		break;
		
		case ALARM1_MatchHoursMinutesSeconds:
			tempRTCRegs[3] |= _BV(DS3231_ALARM_ENABLE_MASK);
		break;
		
		case ALARM1_MatchDateHoursMinutesSeconds:
		default:
		break;
		
		case ALARM1_MatchDayHoursMinutesSeconds:
			tempRTCRegs[3] |= _BV(DS3231_ALARM_ENABLE_DYDT);
		break;
	}
	
	/* Read the status register */
	if (!DS3231_ReadRegisters(DS3231_REG_STATUS, 1, (u08*)&temp))
	{
		/* Return send failure */
		return FALSE;
	}
	
	/* Make sure alarm 1 flag is clear */
	temp &=	~_BV(DS3231_STATUS_BIT_A1F);
	
	/* Write back the status register */
	if (!DS3231_WriteRegisters(DS3231_REG_STATUS, 1, (const u08*)&temp))
	{
		/* Return send failure */
		return FALSE;
	}
	
	/* Write alarm 1 registers */
	if (!DS3231_WriteRegisters(DS3231_REG_A1_SECS, 4, (const u08*)&tempRTCRegs))
	{
		/* Return send failure */
		return FALSE;
	}
	
	/* Read control register */
	if (!DS3231_ReadRegisters(DS3231_REG_CONTROL, 1, (u08*)&temp))
	{
		/* Return send failure */
		return FALSE;
	}
	
	/* Enable alarm 1 interrupt and interrupt control */
	temp |= ( _BV(DS3231_CONTROL_BIT_A1IE) |
			  _BV(DS3231_CONTROL_BIT_INTCN) );

	/* Write control register */
	if (!DS3231_WriteRegisters(DS3231_REG_CONTROL, 1, (const u08*)&temp))
	{
		/* Return send failure */
		return FALSE;
	}
	
	/* Return read success */
	return TRUE;
}

/****************************************************************************
* Name: DS3231_EnableAlarm2
*
* Description:
* 	Configures alarm 1.
*
* Input Parameters:
*	*ptrAlarmSetting = Pointer to structure containing alarm configuration.
*
* Returned Value:
*	Success = TRUE Failure = FALSE.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
u08 DS3231_EnableAlarm2(DS3231_Alarm2TypeDef *ptrAlarmSetting)
{
	/* Variable to store the DS3231 register contents */
	u08 tempRTCRegs[3] = {0};
	
	/* Temporary variable */
	u08 temp = 0;
	
	/* Convert these time and dates to BCD */
	tempRTCRegs[0] = DS3231_BIN2BCD(ptrAlarmSetting->Mins);
	tempRTCRegs[1] = DS3231_BIN2BCD(ptrAlarmSetting->Hours) & 0b10111111;
	tempRTCRegs[2] = DS3231_BIN2BCD(ptrAlarmSetting->Day);
	
	/* Select alarm type */
	switch (ptrAlarmSetting->AlarmType)
	{
		case ALARM2_EveryMinute:
			tempRTCRegs[0] |= _BV(DS3231_ALARM_ENABLE_MASK);
			tempRTCRegs[1] |= _BV(DS3231_ALARM_ENABLE_MASK);
			tempRTCRegs[2] |= _BV(DS3231_ALARM_ENABLE_MASK);
		break;
		
		case ALARM2_MatchMinutes:
			tempRTCRegs[1] |= _BV(DS3231_ALARM_ENABLE_MASK);
			tempRTCRegs[2] |= _BV(DS3231_ALARM_ENABLE_MASK);
		break;
		
		case ALARM2_MatchHoursMinutes:
			tempRTCRegs[2] |= _BV(DS3231_ALARM_ENABLE_MASK);
		break;
		
		case ALARM2_MatchDateHoursMinutes:
		default:
		break;
		
		case ALARM2_MatchDayHoursMinutes:
			tempRTCRegs[2] |= _BV(DS3231_ALARM_ENABLE_DYDT);
		break;
	}
	
	/* Read the status register */
	if (!DS3231_ReadRegisters(DS3231_REG_STATUS, 1, (u08*)&temp))
	{
		/* Return send failure */
		return FALSE;
	}
	
	/* Make sure alarm 1 flag is clear */
	temp &=	~_BV(DS3231_STATUS_BIT_A2F);
	
	/* Write back the status register */
	if (!DS3231_WriteRegisters(DS3231_REG_STATUS, 1, (const u08*)&temp))
	{
		/* Return send failure */
		return FALSE;
	}
	
	/* Write alarm 2 registers */
	if (!DS3231_WriteRegisters(DS3231_REG_A2_MINS, 3, (const u08*)&tempRTCRegs))
	{
		/* Return send failure */
		return FALSE;
	}
	
	/* Read control register */
	if (!DS3231_ReadRegisters(DS3231_REG_CONTROL, 1, (u08*)&temp))
	{
		/* Return send failure */
		return FALSE;
	}
	
	/* Enable alarm 2 interrupt and interrupt control */
	temp |= ( _BV(DS3231_CONTROL_BIT_A2IE) |
			  _BV(DS3231_CONTROL_BIT_INTCN) );
	
	/* Write control register */
	if (!DS3231_WriteRegisters(DS3231_REG_CONTROL, 1, (const u08*)&temp))
	{
		/* Return send failure */
		return FALSE;
	}
	
	/* Return read success */
	return TRUE;
}

/****************************************************************************
* Name: DS3231_ReenableAlarm1Flag
*
* Description:
* 	Re-enables alarm 1.
*
* Input Parameters:
*	None.
*
* Returned Value:
*	Success = TRUE Failure = FALSE.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
u08 DS3231_ReenableAlarm1Flag(void)
{
	/* Temporary variable */
	u08 temp = 0;
	
	/* Read the status register */
	if (!DS3231_ReadRegisters(DS3231_REG_STATUS, 1, (u08*)&temp))
	{
		/* Return send failure */
		return FALSE;
	}
	
	/* Clear alarm 1 flag */
	temp &=	~_BV(DS3231_STATUS_BIT_A1F);
	
	/* Write back the status register */
	if (!DS3231_WriteRegisters(DS3231_REG_STATUS, 1, (const u08*)&temp))
	{
		/* Return send failure */
		return FALSE;
	}
	
	/* Return read success */
	return TRUE;
}

/****************************************************************************
* Name: DS3231_ReenableAlarm2Flag
*
* Description:
* 	Re-enables alarm 2.
*
* Input Parameters:
*	None.
*
* Returned Value:
*	Success = TRUE Failure = FALSE.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
u08 DS3231_ReenableAlarm2Flag(void)
{
	/* Temporary variable */
	u08 temp = 0;
	
	/* Read the status register */
	if (!DS3231_ReadRegisters(DS3231_REG_STATUS, 1, (u08*)&temp))
	{
		/* Return send failure */
		return FALSE;
	}
	
	/* Clear alarm 1 flag */
	temp &=	~_BV(DS3231_STATUS_BIT_A2F);
	
	/* Write back the status register */
	if (!DS3231_WriteRegisters(DS3231_REG_STATUS, 1, (const u08*)&temp))
	{
		/* Return send failure */
		return FALSE;
	}
	
	/* Return read success */
	return TRUE;
}

/****************************************************************************
* Name: DS3231_EnableSquareWaveOutput
*
* Description:
* 	Enables the square wave output on the INT/SQW pin.
*
* Input Parameters:
*	Frequency = Square wave frequency.
*
* Returned Value:
*	Success = TRUE Failure = FALSE.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
u08 DS3231_EnableSquareWaveOutput(u08 Frequency)
{
	/* Temporary variable */
	u08 temp = 0;
	
	/* Read control register */
	if (!DS3231_ReadRegisters(DS3231_REG_CONTROL, 1, (u08*)&temp))
	{
		/* Return send failure */
		return FALSE;
	}
	
	/* Disable interrupt control */
	temp &= ~_BV(DS3231_CONTROL_BIT_INTCN);

	/* Select frequency */
	switch (Frequency)
	{
		case DS3231_SQW_FREQ_1HZ:
			temp &= ~( _BV(DS3231_CONTROL_BIT_RS2) |
					   _BV(DS3231_CONTROL_BIT_RS1) );
		default:
		break;
		
		case DS3231_SQW_FREQ_1024HZ:
			temp &= ~_BV(DS3231_CONTROL_BIT_RS2);
			temp |= _BV(DS3231_CONTROL_BIT_RS1);
		break;
		
		case DS3231_SQW_FREQ_4096HZ:
			temp |= _BV(DS3231_CONTROL_BIT_RS2);
			temp &= ~_BV(DS3231_CONTROL_BIT_RS1);
		break;
		
		case DS3231_SQW_FREQ_8192HZ:
			temp |= ( _BV(DS3231_CONTROL_BIT_RS2) |
					  _BV(DS3231_CONTROL_BIT_RS1) );
		break;
	}
	
	/* Write control register */
	if (!DS3231_WriteRegisters(DS3231_REG_CONTROL, 1, (const u08*)&temp))
	{
		/* Return send failure */
		return FALSE;
	}
	
	/* Return read success */
	return TRUE;	
	
}

/****************************************************************************
* Name: DS3231_GetFATTime
*
* Description:
* 	Gets the current date & time as packed FAT file time.
*
* Input Parameters:
*	None.
*
* Returned Value:
*	Packed FAT file time.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
u32 DS3231_GetFATTime(void)
{
	/* Temporary time */
	DS3231_TimeTypeDef tmpTime = {0};
	
	/* FAT time */
	u32 fatTime = 0;
	
	/* Get current time */
	DS3231_GetTime(&tmpTime);
	
	/* Convert to FAT time */
	fatTime = ( (u32)(tmpTime.Year-1980)<<25)
			   | ((u32)tmpTime.Month<<21)
			   | ((u32)tmpTime.Date<<16)
			   | ((u32)tmpTime.Hours<<11)
			   | ((u32)tmpTime.Mins<<5)
			   | ((u32)tmpTime.Secs>>1);
	
	/* Return packed time */
	return fatTime;
}
