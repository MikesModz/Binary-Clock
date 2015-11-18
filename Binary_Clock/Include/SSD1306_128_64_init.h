/****************************************************************************
* SSD1306_128_64_init.h
*
* Initialisation table for 128x32 SSD1306 display.
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
#ifndef __SSD1306_128_64_INIT_H__
#define __SSD1306_128_64_INIT_H__

/****************************************************************************
* Included Files
****************************************************************************/
#include "avrlibtypes.h"
#include "ssd1306.h"

/****************************************************************************
* Public Data
****************************************************************************/

const u08 SSD1306_128_64_Init[] =
{
	25,
	SSD1306_DISPLAYOFF,
	SSD1306_SETDISPLAYCLOCKDIV,
	0x80,
	SSD1306_SETMULTIPLEX,
	0x3F,
	SSD1306_SETDISPLAYOFFSET,
	0x0,
	SSD1306_SETSTARTLINE|0x0,
	SSD1306_CHARGEPUMP,
	0x14,
	SSD1306_MEMORYMODE,
	0x00,
	SSD1306_SEGREMAP|0x1,
	SSD1306_COMSCANDEC,
	SSD1306_SETCOMPINS,
	0x12,
	SSD1306_SETCONTRAST,
	0xCF,
	SSD1306_SETPRECHARGE,
	0xF1,
	SSD1306_SETVCOMDETECT,
	0x40,
	SSD1306_DISPLAYALLON_RESUME,
	SSD1306_NORMALDISPLAY,
	SSD1306_DISPLAYON
};

#endif /* __SSD1306_128_64_INIT_H__ */
