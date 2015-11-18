/****************************************************************************
* ds3231.h
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
#ifndef __DS3231_H__
#define __DS3231_H__

/****************************************************************************
* Included Files
****************************************************************************/
#include <avr/io.h>
#include "avrlibtypes.h"

/****************************************************************************
 * Pre-processor Definitions
****************************************************************************/

/* DS3231 slave address */
#define DS3231_SLAVE_ADDRESS 			0x68

/* DS3231 registers */
#define	DS3231_REG_SECS					0x00		/* Seconds register */
#define DS3231_REG_MINS					0x01		/* Minutes register */
#define DS3231_REG_HOURS				0x02		/* Hours register */
#define DS3231_REG_DAY					0x03		/* Day register */
#define DS3231_REG_DATE					0x04		/* Date register */
#define DS3231_REG_MONTH				0x05		/* Month register */
#define DS3231_REG_YEAR					0x06		/* Year register */
#define DS3231_REG_A1_SECS				0x07		/* Alarm 1 seconds register */
#define DS3231_REG_A1_MINS				0x08		/* Alarm 1 minutes register */
#define DS3231_REG_A1_HOURS				0x09		/* Alarm 1 hours register */
#define DS3231_REG_A1_DAY				0x0A		/* Alarm 1 day register */
#define DS3231_REG_A2_MINS				0x0B		/* Alarm 2 minutes register */
#define DS3231_REG_A2_HOURS				0x0C		/* Alarm 2 hours register */
#define DS3231_REG_A2_DAY_DATE			0x0D		/* Alarm 2 day/date register */
#define DS3231_REG_CONTROL				0x0E		/* Control register */
#define DS3231_REG_STATUS				0x0F		/* Status register */
#define DS3231_REG_AGING_OFFSET			0x10		/* Aging offset register */
#define DS3231_REG_TEMP_MSB				0x11		/* Temperature MSB register */
#define DS3231_REG_TEMP_LSB				0x12		/* Temperature LSB register */

/* DS3231 control registers */
#define DS3231_CONTROL_BIT_A1IE			0			/* Alarm 1 interrupt enable */
#define DS3231_CONTROL_BIT_A2IE			1			/* Alarm 2 interrupt enable */
#define DS3231_CONTROL_BIT_INTCN		2			/* Interrupt control */
#define DS3231_CONTROL_BIT_RS1			3			/* Rate select 1 */
#define DS3231_CONTROL_BIT_RS2			4			/* Rate select 2 */
#define DS3231_CONTROL_BIT_CONV			5			/* Convert temperature */
#define DS3231_CONTROL_BIT_BBSQW		6			/* Battery backed up square wave enable */
#define DS3231_CONTROL_BIT_DOSC			7			/* Disable Oscillator */

/* DS3231 status registers */
#define DS3231_STATUS_BIT_A1F			0			/* Alarm 1 flag */
#define DS3231_STATUS_BIT_A2F			1			/* Alarm 2 flag */
#define DS3231_STATUS_BIT_BSY			2			/* Busy */
#define DS3231_STATUS_BIT_EN32K			3			/* Enable 32kHz output */
#define DS3231_STATUS_BIT_OSF			7			/* Oscillator stop flag */

/* DS3231 alarm masks */
#define DS3231_ALARM_ENABLE_DYDT		6			/* Alarm day/date flag */
#define DS3231_ALARM_ENABLE_MASK		7			/* Alarm enable mask */

/* DS3231 hour masks */
#define DS3231_HOURS_AM_PM				5			/* AM/PM flag*/
#define DS3231_HOURS_12_24				6			/* 12/24 hour flag */

/* DS3231 square wave output freq */
#define DS3231_SQW_FREQ_1HZ				0			/* 1Hz */
#define DS3231_SQW_FREQ_1024HZ			1			/* 1.024kHz */
#define DS3231_SQW_FREQ_4096HZ			2			/* 4.096kHz */
#define DS3231_SQW_FREQ_8192HZ			3			/* 8.192kHz */

/****************************************************************************
* Public Types
****************************************************************************/

/* DS3231 alarm 1 type definitions */
typedef enum Alarm1Type
{
	ALARM1_EverySecond,
	ALARM1_MatchSeconds,
	ALARM1_MatchMinutesSeconds,
	ALARM1_MatchHoursMinutesSeconds,
	ALARM1_MatchDateHoursMinutesSeconds,
	ALARM1_MatchDayHoursMinutesSeconds
} ALARM1_TYPE;

/* DS3231 alarm 2 type definitions */
typedef enum Alarm2Type
{
	ALARM2_EveryMinute,
	ALARM2_MatchMinutes,
	ALARM2_MatchHoursMinutes,
	ALARM2_MatchDateHoursMinutes,
	ALARM2_MatchDayHoursMinutes
} ALARM2_TYPE;

/* DS3231 time setting structure */
typedef struct
{
	u08 Hours;
	u08 Mins;
	u08 Secs;
	u08 Day;
	u08 Date;
	u08 Month;
	u16 Year;
} DS3231_TimeTypeDef;

/* DS3231 alarm 1 setting structure */
typedef struct
{
	u08 Hours;
	u08 Mins;
	u08 Secs;
	u08 Day;
	ALARM1_TYPE AlarmType;
} DS3231_Alarm1TypeDef;

/* DS3231 alarm 2 setting structure */
typedef struct
{
	u08 Hours;
	u08 Mins;
	u08 Day;
	ALARM2_TYPE AlarmType;
} DS3231_Alarm2TypeDef;

/****************************************************************************
* Public Function Prototypes
****************************************************************************/

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
*	None.
****************************************************************************/
u08 DS3231_Init(void);

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
u08 DS3231_SetTime(DS3231_TimeTypeDef *ptrSetTime);

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
u08 DS3231_GetTime(DS3231_TimeTypeDef *ptrGetTime);

/****************************************************************************
* Name: DS3231_GetTemperature
*
* Description:
* 	Gets the current temperature.
*
* Input Parameters:
*	*ptrTemperature = Pointer to variable to save temperature.
*
* Returned Value:
*	Success = TRUE Failure = FALSE.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
u08 DS3231_GetTemperature(float *ptrTemperature);

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
u08 DS3231_EnableAlarm1(DS3231_Alarm1TypeDef *ptrAlarmSetting);

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
u08 DS3231_EnableAlarm2(DS3231_Alarm2TypeDef *ptrAlarmSetting);

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
*	Nothing.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
u08 DS3231_ReenableAlarm1Flag(void);

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
*	Nothing.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
u08 DS3231_ReenableAlarm2Flag(void);

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
u08 DS3231_EnableSquareWaveOutput(u08 Frequency);

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
u32 DS3231_GetFATTime(void);

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
u08 DS3231_BIN2BCD(u08 inByte);

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
u08 DS3231_BCD2BIN(u08 inByte);

#endif /* __DS3231_H__ */
