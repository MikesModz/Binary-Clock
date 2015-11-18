/****************************************************************************
* gfx.c
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

/****************************************************************************
* Included Files
****************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <string.h>
#include <util/delay.h>
#include <stdlib.h>
#include "avrlibtypes.h"
#include "ssd1306.h"
#include "gfx.h"
#include "Impact18x31_Numbers.h"
#include "Impact9x17_Numbers.h"
#include "font5x7.h"
#include "Bebas_Neue18x36_Numbers.h"
#include "Bebas_Neue20x36_Bold_Numbers.h"
#include "Earthbound_12x19_48to57.h"
#include "Liberation_Sans11x14_Numbers.h"
#include "Liberation_Sans15x21_Numbers.h"
#include "Liberation_Sans17x17_Alpha.h"
#include "Liberation_Sans20x28_Numbers.h"
#include "Liberation_Sans27x36_Numbers.h"

/****************************************************************************
* Public Data
****************************************************************************/
#if defined(INCLUDE_FONT_18X31)
GFX_FontConfigType g_Font18x31 =
{ 
	Impact18x31_Numbers,			/* Font table */
	18,								/* Width */
	31,								/* Height */
	46,								/* First character */
	57,								/* Last character */
	FONT_TYPE_MIKRO					/* Font type */
};
#endif

#if defined(INCLUDE_FONT_9X17)
GFX_FontConfigType g_Font9x17 =
{ 
	Impact9x17_Numbers,				/* Font table */
	9,								/* Width */
	17,								/* Height */
	46,								/* First character */
	57,								/* Last character */
	FONT_TYPE_MIKRO					/* Font type */
};
#endif

#if defined(INCLUDE_FONT_5X7)
GFX_FontConfigType g_Font5x7 =
{ 
	Font5x7,						/* Font table */
	5,								/* Width */
	7,								/* Height */
	32,								/* First character */
	127,							/* Last character */
	FONT_TYPE_STANG					/* Font type */
};
#endif

#if defined(INCLUDE_FONT_18X36)
GFX_FontConfigType g_Font18x36 =
{ 
	Bebas_Neue18x36_Numbers,		/* Font table */
	18,								/* Width */
	36,								/* Height */
	46,								/* First character */
	57,								/* Last character */
	FONT_TYPE_MIKRO					/* Font type */
};
#endif

#if defined(INCLUDE_FONT_20X36)
GFX_FontConfigType g_Font20x36 =
{ 
	Bebas_Neue20x36_Bold_Numbers,	/* Font table */
	20,								/* Width */
	36,								/* Height */
	46,								/* First character */
	57,								/* Last character */
	FONT_TYPE_MIKRO					/* Font type */
};
#endif
	
#if defined(INCLUDE_FONT_12X19)
GFX_FontConfigType g_Font12x19 =
{ 
	Earthbound_12x19_48to57,		/* Font table */
	12,								/* Width */
	19,								/* Height */
	48,								/* First character */
	57,								/* Last character */
	FONT_TYPE_STANG					/* Font type */
};
#endif
	
#if defined(INCLUDE_FONT_11X14)
GFX_FontConfigType g_Font11x14 =
{ 
	Liberation_Sans11x14_Numbers,	/* Font table */
	11,								/* Width */
	14,								/* Height */
	46,								/* First character */
	57,								/* Last character */
	FONT_TYPE_MIKRO					/* Font type */
};
#endif
	
#if defined(INCLUDE_FONT_15X21)
GFX_FontConfigType g_Font15x21 =
{ 
	Liberation_Sans15x21_Numbers,	/* Font table */
	15,								/* Width */
	21,								/* Height */
	46,								/* First character */
	57,								/* Last character */
	FONT_TYPE_MIKRO					/* Font type */
};
#endif

#if defined(INCLUDE_FONT_17X17)
GFX_FontConfigType g_Font17x17 =
{ 
	Liberation_Sans17x17_Alpha,		/* Font table */
	17,								/* Width */
	17,								/* Height */
	65,								/* First character */
	90,								/* Last character */
	FONT_TYPE_MIKRO					/* Font type */
};
#endif

#if defined(INCLUDE_FONT_20X28)
GFX_FontConfigType g_Font20x28 =
{ 
	Liberation_Sans20x28_Numbers,	/* Font table */
	20,								/* Width */
	28,								/* Height */
	46,								/* First character */
	57,								/* Last character */
	FONT_TYPE_MIKRO					/* Font type */
};
#endif
	
#if defined(INCLUDE_FONT_27X36)
GFX_FontConfigType g_Font27x36 =
{ 
	Liberation_Sans27x36_Numbers,	/* Font table */
	27,								/* Width */
	36,								/* Height */
	46,								/* First character */
	57,								/* Last character */
	FONT_TYPE_MIKRO					/* Font type */
};
#endif

/****************************************************************************
* Public Functions
****************************************************************************/

/****************************************************************************
* Name: GFX_DrawCharXYLCD
*
* Description:
* 	Writes a character to the display.
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
u08 GFX_DrawCharXYLCD(u08 X, u08 Y, u08 Character, GFX_FontConfigType *Font)
{
	u08 i = 0;
	u08 varWidth = 0;
	u08 bytesHigh = 0;
	u08 bytesPerChar = 0;
	const char *p;
	u08 j = 0;
	u08 data = 0;
	u08 bit = 0;

	bytesHigh = Font->Height / 8 + 1;
	bytesPerChar = Font->Width * bytesHigh + 1;

	/* Check pass character is supported by the font */
	if (Character < Font->StartChar || Character > Font->EndChar)
	{
		Character = '.';
	}

	if (Font->TableType == FONT_TYPE_STANG)
	{
		for (i = 0; i < Font->Width; i++)
		{
			u08 data = pgm_read_byte( Font->FontTable + 
				((Character - Font->StartChar) * (Font->Width)) + i);

			for (j = 0; j < 8; j++)
			{
				if (X + i >= SSD1306_LCD_WIDTH || 
					Y + j >= SSD1306_LCD_HEIGHT)
				{
					/* Don't write past the dimensions of the LCD, 
					   skip the entire char */
					return 0;
				}
				
				if (data & (1 << j))
				{
					SSD1306_DrawPixel(X + i, Y + j, SSD1306_COL_WHITE);
				}
				else
				{
					SSD1306_DrawPixel(X + i, Y + j, SSD1306_COL_BLACK);
				}
			}
		}
		return Font->Width;
	}
	else if (Font->TableType == FONT_TYPE_MIKRO)
	{
		/* Get pointer to the character in the font table */
		p = Font->FontTable + (Character - Font->StartChar) * bytesPerChar;

		/* The first byte per character is always the width of 
		   the character */
		varWidth = pgm_read_byte(p);

		/* Step over the variable width field */
		p++;

		for (i = 0; i < varWidth; i++)
		{
			for (j = 0; j < bytesHigh; j++)
			{
				data = pgm_read_byte(p + i * bytesHigh + j);
				for (bit = 0; bit < 8; bit++)
				{
					if (X + i >= SSD1306_LCD_WIDTH || 
						Y + j * 8 + bit >= SSD1306_LCD_HEIGHT)
					{
						/* Don't write past the dimensions of the LCD, 
						   skip the entire char */
						return 0;
					}

					/* We should not write if the y bit exceeds 
					   font height */
					if ((j * 8 + bit) >= Font->Height)
					{
						/* Skip the bit */
						continue;
					}

					if (data & (1 << bit))
					{
						SSD1306_DrawPixel(X + i, Y + j * 8 + bit,
								SSD1306_COL_WHITE);
					}
					else
					{
						SSD1306_DrawPixel(X + i, Y + j * 8 + bit,
								SSD1306_COL_BLACK);
					}
				}
			}
		}
		return varWidth;
	}
	else
	{
		return 0;
	}
}

/****************************************************************************
* Name: GFX_DrawStringXY
*
* Description:
* 	Writes a string of characters to the display.
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
void GFX_DrawStringXY(u08 X, u08 Y, u08 *Str, GFX_FontConfigType *Font)
{
	u08 width = 0;

	if (Y > (SSD1306_LCD_HEIGHT - Font->Height - 1))
	{
		/* Character won't fit */
		return;
	}

	while (*Str)
	{
		width = GFX_DrawCharXYLCD(X, Y, *Str, Font);
		X += (width + 1);
		Str++;
	}
}

/****************************************************************************
* Name: GFX_DrawRect
*
* Description:
* 	Draws a rectangle on the display.
*
* Input Parameters:
* 	X = Horizontal start position.
* 	Y = Vertical start position.
*	W = Rectangle width.
*	H = Rectangle height.
*	Colour = Rectangle colour.
*
* Returned Value:
* 	Nothing.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
void GFX_DrawRect(s16 X, s16 Y, s16 W, s16 H, s16 Colour)
{
	GFX_DrawFastHLine(X, Y, W, Colour);
	GFX_DrawFastHLine(X, Y+H-1, W, Colour);
	GFX_DrawFastVLine(X, Y, H, Colour);
	GFX_DrawFastVLine(X+W-1, Y, H, Colour);
}

/****************************************************************************
* Name: GFX_FillRect
*
* Description:
* 	Draws a filled rectangle on the display.
*
* Input Parameters:
* 	X = Horizontal start position.
* 	Y = Vertical start position.
*	W = Rectangle width.
*	H = Rectangle height.
*	Colour = Rectangle colour.
*
* Returned Value:
* 	Nothing.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
void GFX_FillRect(s16 X, s16 Y, s16 W, s16 H, u16 Colour) 
{
	for (s16 i=X; i<X+W; i++)
	{
		GFX_DrawFastVLine(i, Y, H, Colour);
	}
}

/****************************************************************************
* Name: GFX_DrawFastVLine
*
* Description:
* 	Draws a vertical line on the display.
*
* Input Parameters:
* 	X = Horizontal start position.
* 	Y = Vertical start position.
*	H = Line height.
*	Colour = Line colour.
*
* Returned Value:
* 	Nothing.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
void GFX_DrawFastVLine(s16 X, s16 Y, s16 H, u16 Colour)
{
	GFX_DrawLine(X, Y, X, Y+H-1, Colour);
}

/****************************************************************************
* Name: GFX_DrawFastHLine
*
* Description:
* 	Draws a horizontal line on the display.
*
* Input Parameters:
* 	X = Horizontal start position.
* 	Y = Vertical start position.
*	W = Line width.
*	Colour = Line colour.
*
* Returned Value:
* 	Nothing.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
void GFX_DrawFastHLine(s16 X, s16 Y, s16 W, u16 Colour)
{
	GFX_DrawLine(X, Y, X+W-1, Y, Colour);
}

/****************************************************************************
* Name: GFX_DrawLine
*
* Description:
* 	Draws a line on the display.
*
* Input Parameters:
* 	X0 = Horizontal start position.
* 	Y0 = Vertical start position.
* 	X1 = Horizontal end position.
* 	Y1 = Vertical end position.
*	Colour = Line colour.
*
* Returned Value:
* 	Nothing.
*
* Assumptions/Limitations:
*	None.
****************************************************************************/
void GFX_DrawLine(s16 X0, s16 Y0, s16 X1, s16 Y1, u16 Colour)
{
	s16 steep = abs(Y1 - Y0) > abs(X1 - X0);
	
	if (steep)
	{
		swap(X0, Y0);
		swap(X1, Y1);
	}

	if (X0 > X1) 
	{
		swap(X0, X1);
		swap(Y0, Y1);
	}

	s16 dx, dy;
	dx = X1 - X0;
	dy = abs(Y1 - Y0);

	s16 err = dx / 2;
	s16 ystep;

	if (Y0 < Y1) 
	{
		ystep = 1;
	}
	else
	{
		ystep = -1;
	}

	for (; X0<=X1; X0++)
	{
		if (steep) 
		{
			SSD1306_DrawPixel(Y0, X0, Colour);
		}
		else 
		{
			SSD1306_DrawPixel(X0, Y0, Colour);
		}
		err -= dy;
		if (err < 0)
		{
			Y0 += ystep;
			err += dx;
		}
	}
}
