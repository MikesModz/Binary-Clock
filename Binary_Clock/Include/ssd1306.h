/****************************************************************************
* ssd1306.h
*
* SSD1306 OLED display driver for Atmel AVR micro controllers.
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
#ifndef __SSD1306_H__
#define __SSD1306_H__

/****************************************************************************
* Included Files
****************************************************************************/
#include <avr/io.h>
#include "avrlibtypes.h"

/****************************************************************************
 * Pre-processor Definitions
****************************************************************************/

/* SSD1306 device address */
#define SSD1306_DEVICE_ADDRESS	0x3C

/* SSD1306 display type */
#define SSD1306_128_64
//#define SSD1306_128_32
//#define SSD1306_96_16

#if !defined SSD1306_128_64 && !defined SSD1306_128_32 && !defined SSD1306_96_16
#error "At least one SSD1306 display must be specified in ssd1306.h"
#endif

/* SSD1306 128x64 display size */
#if defined SSD1306_128_64
 #define SSD1306_LCD_WIDTH		128
 #define SSD1306_LCD_HEIGHT		64
#endif

/* SSD1306 128x32 display size */
#if defined SSD1306_128_32
 #define SSD1306_LCD_WIDTH		128
 #define SSD1306_LCD_HEIGHT		32
#endif

/* SSD1306 96x16 display size */
#if defined SSD1306_96_16
 #define SSD1306_LCD_WIDTH		96
 #define SSD1306_LCD_HEIGHT		16
#endif
  
/* SSD1306 commands */
#define SSD1306_SETCONTRAST				0x81
#define SSD1306_DISPLAYALLON_RESUME		0xA4
#define SSD1306_DISPLAYALLON			0xA5
#define SSD1306_NORMALDISPLAY			0xA6
#define SSD1306_INVERTDISPLAY			0xA7
#define SSD1306_DISPLAYOFF				0xAE
#define SSD1306_DISPLAYON				0xAF
#define SSD1306_SETDISPLAYOFFSET		0xD3
#define SSD1306_SETCOMPINS				0xDA
#define SSD1306_SETVCOMDETECT			0xDB
#define SSD1306_SETDISPLAYCLOCKDIV		0xD5
#define SSD1306_SETPRECHARGE			0xD9
#define SSD1306_SETMULTIPLEX			0xA8
#define SSD1306_SETLOWCOLUMN			0x00
#define SSD1306_SETHIGHCOLUMN			0x10
#define SSD1306_SETSTARTLINE			0x40
#define SSD1306_MEMORYMODE				0x20
#define SSD1306_COLUMNADDR				0x21
#define SSD1306_PAGEADDR				0x22
#define SSD1306_COMSCANINC				0xC0
#define SSD1306_COMSCANDEC				0xC8
#define SSD1306_SEGREMAP				0xA0
#define SSD1306_CHARGEPUMP				0x8D

/* SSD1306 scroll settings */
#define SSD1306_ACTIVATE_SCROLL							0x2F
#define SSD1306_DEACTIVATE_SCROLL						0x2E
#define SSD1306_SET_VERTICAL_SCROLL_AREA				0xA3
#define SSD1306_RIGHT_HORIZONTAL_SCROLL					0x26
#define SSD1306_LEFT_HORIZONTAL_SCROLL					0x27
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL	0x29
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL		0x2A

/* SSD1306 colours */
#define SSD1306_COL_WHITE		0
#define SSD1306_COL_BLACK		1
#define SSD1306_COL_INVERSE		2

/****************************************************************************
* Public Function Prototypes
****************************************************************************/

/****************************************************************************
* Name: SSD1306_Init
*
* Description:
* 	Initialises the display.
*
* Input Parameters:
* 	None.
*
* Returned Value:
*	Nothing.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
void SSD1306_Init(void);

/****************************************************************************
* Name: SSD1306_UpdateDisplay
*
* Description:
* 	Copies the contents of the frame buffer to the display.
*
* Input Parameters:
* 	None.
*
* Returned Value:
*	Nothing.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
void SSD1306_UpdateDisplay(void);

/****************************************************************************
* Name: SSD1306_InvertDisplay
*
* Description:
* 	Inverts the display.
*
* Input Parameters:
* 	Invert = TRUE or FALSE.
*
* Returned Value:
*	Nothing.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
void SSD1306_InvertDisplay(u08 Invert);

/****************************************************************************
* Name: SSD1306_DimDisplay
*
* Description:
* 	Dims the display.
*
* Input Parameters:
* 	Dim = TRUE or FALSE.
*
* Returned Value:
*	Nothing.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
void SSD1306_DimDisplay(u08 Dim);

/****************************************************************************
* Name: SSD1306_ScrollRight
*
* Description:
* 	Starts the display scrolling right.
*
* Input Parameters:
* 	Start = Row start.
*	Stop = Row end.
*
* Returned Value:
*	Nothing.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
void SSD1306_ScrollRight(u08 Start, u08 Stop);

/****************************************************************************
* Name: SSD1306_ScrollLeft
*
* Description:
* 	Starts the display scrolling right.
*
* Input Parameters:
* 	Start = Row start.
*	Stop = Row end.
*
* Returned Value:
*	Nothing.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
void SSD1306_ScrollLeft(u08 Start, u08 Stop);

/****************************************************************************
* Name: SSD1306_ScrollDiagRight
*
* Description:
* 	Starts the display scrolling right.
*
* Input Parameters:
* 	Start = Row start.
*	Stop = Row end.
*
* Returned Value:
*	Nothing.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
void SSD1306_ScrollDiagRight(u08 Start, u08 Stop);

/****************************************************************************
* Name: SSD1306_ScrollDiagLeft
*
* Description:
* 	Starts the display scrolling right.
*
* Input Parameters:
* 	Start = Row start.
*	Stop = Row end.
*
* Returned Value:
*	Nothing.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
void SSD1306_ScrollDiagLeft(u08 Start, u08 Stop);

/****************************************************************************
* Name: SSD1306_StopScroll
*
* Description:
* 	Stops the display from scrolling..
*
* Input Parameters:
* 	None.
*
* Returned Value:
*	Nothing.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
void SSD1306_StopScroll(void);

/****************************************************************************
* Name: SSD1306_ClearDisplay
*
* Description:
* 	Clears the display frame buffer.
*
* Input Parameters:
* 	None.
*
* Returned Value:
*	Nothing.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
void SSD1306_ClearDisplay(void);

/****************************************************************************
* Name: SSD1306_DrawPixel
*
* Description:
* 	Draws a single pixel on the display.
*
* Input Parameters:
* 	X = Horizontal coordinate.
*	Y = Vertical coordinate.
*	Colour = SSD1306_COL_WHITE, SSD1306_COL_BLACK or SSD1306_COL_INVERSE
*
* Returned Value:
*	Nothing.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
void SSD1306_DrawPixel(s16 X, s16 Y, u16 Colour);

#endif /* __SSD1306_H__ */
