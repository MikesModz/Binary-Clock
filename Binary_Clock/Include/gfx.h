/****************************************************************************
* gfx.h
*
* Core graphics library providing various graphics functions such as drawing
* points, lines, circles etc. Requires hardware specific display driver.
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
#ifndef __GFX_H__
#define __GFX_H__

/****************************************************************************
* Included Files
****************************************************************************/
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "avrlibtypes.h"

/****************************************************************************
 * Pre-processor Definitions
****************************************************************************/

/* Define fonts to be included */
//#define INCLUDE_FONT_18X31
//#define INCLUDE_FONT_9X17
#define INCLUDE_FONT_5X7
//#define INCLUDE_FONT_18X36
//#define INCLUDE_FONT_20X36
//#define INCLUDE_FONT_12X19
//#define INCLUDE_FONT_11X14
//#define INCLUDE_FONT_15X21
//#define INCLUDE_FONT_17X17
//#define INCLUDE_FONT_20X28
//#define INCLUDE_FONT_27X36

#define swap(a, b) { s16 t = a; a = b; b = t; }

/****************************************************************************
* Public Types
****************************************************************************/

/* Font table type */
typedef enum
{
	FONT_TYPE_STANG, 
	FONT_TYPE_MIKRO,
	FONT_TYPE_GLCD_UTILS
} GFX_FontTableType;

/* Font structure */
typedef struct
{
	PGM_P FontTable;
	u08 Width;
	u08 Height;
	u08 StartChar;
	u08 EndChar;
	GFX_FontTableType TableType;
} GFX_FontConfigType;

/****************************************************************************
* Public Data
****************************************************************************/
extern GFX_FontConfigType g_Font18x31;
extern GFX_FontConfigType g_Font9x17;
extern GFX_FontConfigType g_Font5x7;
extern GFX_FontConfigType g_Font18x36;
extern GFX_FontConfigType g_Font20x36;
extern GFX_FontConfigType g_Font12x19;
extern GFX_FontConfigType g_Font11x14;
extern GFX_FontConfigType g_Font15x21;
extern GFX_FontConfigType g_Font17x17;
extern GFX_FontConfigType g_Font20x28;
extern GFX_FontConfigType g_Font27x36;

/****************************************************************************
* Public Function Prototypes
****************************************************************************/

/****************************************************************************
* Name: GFX_DrawCharXYLCD
*
* Description:
* 	Reads the DS18B20 scratch pad register.
*
* Input Parameters:
* 	X = Characters start x position.
* 	Y = Characters start y position.
*	Character = Character to draw.
*	*Font = Pointer to font to use.
*
* Returned Value:
* 	Character width.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
u08 GFX_DrawCharXYLCD(u08 X, u08 Y, u08 Character, GFX_FontConfigType *Font);

/****************************************************************************
* Name: GFX_DrawStringXY
*
* Description:
* 	Reads the DS18B20 scratch pad register.
*
* Input Parameters:
* 	X = Strings starting x position.
* 	Y = Strings starting y position.
*	*Str = Pointer to string to draw.
*	*Font = Pointer to font to use.
*
* Returned Value:
* 	Nothing.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
void GFX_DrawStringXY(u08 X, u08 Y, u08 *Str, GFX_FontConfigType *Font);

void GFX_DrawRect(s16 X, s16 Y, s16 W, s16 H, s16 Colour);
void GFX_FillRect(s16 X, s16 Y, s16 W, s16 H, u16 Colour);
void GFX_DrawFastVLine(s16 X, s16 Y, s16 H, u16 Colour);
void GFX_DrawFastHLine(s16 X, s16 Y, s16 W, u16 Colour);
void GFX_DrawLine(s16 X0, s16 Y0, s16 X1, s16 Y1, u16 Colour);

#endif /* __GFX_H__ */
