/****************************************************************************
* binary_clock.c
*
* Binary clock project.
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
#include "avrlibtypes.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/pgmspace.h>
#include "ssd1306.h"
#include "gfx.h"
#include "ds3231.h"
#include "twi.h"
#include "binary_clock_rectangles.h"

/****************************************************************************
* Private Data
****************************************************************************/
static volatile u08 AlarmFlag = 0;
static const u08 MonthStrings[][12] = {"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};
	
/****************************************************************************
* Private Functions
****************************************************************************/
static void FillRectangle(const u08 Rectangles[][2], u08 i, u08 Colour);
static void DrawRectangle(const u08 Rectangles[][2], u08 i, u08 Colour);

/****************************************************************************
* Name: FillRectangle
*
* Description:
*	Draws a filled rectangle on the display.
*
* Input Parameters:
*	Rectangles[][2] = Pointer to array containing rectangle coordinates.
*	i = Entry in array.
*	Colour = Rectangle colour.
*
* Returned Value:
*	Nothing.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
static void FillRectangle(const u08 Rectangles[][2], u08 i, u08 Colour)
{										
	GFX_FillRect(pgm_read_byte(&Rectangles[i][0]),
				 pgm_read_byte(&Rectangles[i][1]),
				 10,
				 8,
				 Colour);
}

/****************************************************************************
* Name: DrawRectangle
*
* Description:
*	Draws a outlined rectangle on the display.
*
* Input Parameters:
*	Rectangles[][2] = Pointer to array containing rectangle coordinates.
*	i = Entry in array.
*	Colour = Rectangle colour.
*
* Returned Value:
*	Nothing.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
static void DrawRectangle(const u08 Rectangles[][2], u08 i, u08 Colour)
{
	GFX_DrawRect(pgm_read_byte(&Rectangles[i][0]),
				 pgm_read_byte(&Rectangles[i][1]),
				 10,
				 8,
				 Colour);
}

/****************************************************************************
* Public Functions
****************************************************************************/
int main(void)
{
	u08 buffer[40] = {0};
	float temp_c = 0.0;
	u08 temp = 0;
	u08 i = 0;
	
	static u08 LastHours = 0xFF;
	static u08 LastMinutes = 0xFF;
	static u08 toggle = 0;
	
	DS3231_TimeTypeDef DS3231_Time;
	
	/* Enable global interrupts */
	sei();
	
	/* Initialise the two wire interface */
	TWI_Init();
	
	/* Initialise real time clock */
	DS3231_Init();
	
	/* Enable the square wave output */
	DS3231_EnableSquareWaveOutput(DS3231_SQW_FREQ_1HZ);
	
	/* Initialise the display */
	SSD1306_Init();
	
	/* Configure falling edge on INT0 */
	EICRA = _BV(ISC01);
	
	/* Enable INT0 interrupt */
	EIMSK = _BV(INT0);
	
	/* Clear INT0 flag */
	EIFR = _BV(INTF0);
	
	/* Clear the display */
	SSD1306_ClearDisplay();
	
	/* Main loop */
	for (;;)
	{
		if (AlarmFlag)
		{
			/* Get current time */
			DS3231_GetTime(&DS3231_Time);

			/* If hours changed */
			if( DS3231_Time.Hours != LastHours )
			{
				temp = DS3231_BIN2BCD(DS3231_Time.Hours);
				for (i=0; i<6; i++)
				{
					if (temp & _BV(i)) FillRectangle(HourRects, i, SSD1306_COL_WHITE);
					else
					{
						FillRectangle(HourRects, i, SSD1306_COL_BLACK);
						DrawRectangle(HourRects, i, SSD1306_COL_WHITE);
					}
				}
				LastHours = DS3231_Time.Hours;
			}
			
			/* If minutes changed */
			if( DS3231_Time.Mins != LastMinutes )
			{
				temp = DS3231_BIN2BCD(DS3231_Time.Mins);
				for (i=0; i<7; i++)
				{
					if (temp & _BV(i)) FillRectangle(MinRects, i, SSD1306_COL_WHITE);
					else
					{
						FillRectangle(MinRects, i, SSD1306_COL_BLACK);
						DrawRectangle(MinRects, i, SSD1306_COL_WHITE);
					}
				}
				LastMinutes = DS3231_Time.Mins;
			}
			
			/* Update seconds */
			temp = DS3231_BIN2BCD(DS3231_Time.Secs);
			for (i=0; i<7; i++)
			{
				if (temp & _BV(i))
				{
					FillRectangle(SecRects, i, SSD1306_COL_WHITE);
				}
				else
				{
					FillRectangle(SecRects, i, SSD1306_COL_BLACK);
					DrawRectangle(SecRects, i, SSD1306_COL_WHITE);
				}
			}

			/* Get current temperature */
			DS3231_GetTemperature(&temp_c);
			
			if (toggle < 5)
			{
				/* Show temperature and date across the top */
				GFX_FillRect(0,0,128,10, SSD1306_COL_BLACK);
				sprintf((char*)buffer, "%02u-%s-%02u", DS3231_Time.Date, MonthStrings[DS3231_Time.Month-1], DS3231_Time.Year);
				GFX_DrawStringXY((128-(strlen((char*)buffer) * g_Font5x7.Width))/2, 0, buffer, &g_Font5x7);	
			}
			else if (toggle < 10)
			{
				/* Show temperature and date across the top */
				GFX_FillRect(0,0,128,10, SSD1306_COL_BLACK);
				sprintf((char*)buffer, "%02u:%02u:%02u", DS3231_Time.Hours, DS3231_Time.Mins, DS3231_Time.Secs);
				GFX_DrawStringXY((128-(strlen((char*)buffer) * g_Font5x7.Width))/2, 0, buffer, &g_Font5x7);	
			}
			else
			{
				/* Show temperature and date across the top */
				GFX_FillRect(0,0,128,10, SSD1306_COL_BLACK);
				sprintf((char*)buffer, "%0.2f Deg C", temp_c);
				GFX_DrawStringXY((128-(strlen((char*)buffer) * g_Font5x7.Width))/2, 0, buffer, &g_Font5x7);	
			}
			
			/* Repaint the display now */
			SSD1306_UpdateDisplay();
			
			/* Update toggle counter */
			toggle++;
			toggle %= 15;
			
			/* Clear alarm flag */
			AlarmFlag = 0;
		}
	}
}

/****************************************************************************
* Name: INT0_vect
*
* Description:
* 	Services the external INT0 interrupt.
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
ISR(INT0_vect)
{
	/* Set alarm flag */
	AlarmFlag = 1;
}
