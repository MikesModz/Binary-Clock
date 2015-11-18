/****************************************************************************
* binary_clock_rectangles.h
*
* Clock rectangles for binary clock.
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
#ifndef __BINARY_CLOCK_RECTANGLES_H__
#define __BINARY_CLOCK_RECTANGLES_H__

/****************************************************************************
* Included Files
****************************************************************************/
#include "avrlibtypes.h"

/****************************************************************************
* Public Data
****************************************************************************/

static const u08 SecRects[7][2] PROGMEM =
{
	/* Seconds */
	{   106,  54}, /* 2^0 */
	{   106,  42}, /* 2^1 */
	{   106,  30}, /* 2^2 */
	{   106,  18}, /* 2^4 */
	/* Ten seconds */
	{    92,  54}, /* 2^0 */
	{    92,  42}, /* 2^1 */
	{    92,  30}  /* 2^2 */
};

static const u08 MinRects[7][2] PROGMEM =
{	
	/* Minutes */
	{   67,  54}, /* 2^0 */
	{   67,  42}, /* 2^1 */
	{   67,  30}, /* 2^2 */
	{   67,  18}, /* 2^4 */
	/* Ten minutes */
	{   53,  54}, /* 2^0 */
	{   53,  42}, /* 2^1 */
	{   53,  30}  /* 2^2 */
};

static const u08 HourRects[6][2] PROGMEM =
{
	/* Hours */
	{   28,  54}, /* 2^0 */
	{   28,  42}, /* 2^1 */
	{   28,  30}, /* 2^2 */
	{   28,  18}, /* 2^4 */
	/* Ten hours */
	{   14,  54}, /* 2^0 */
	{   14,  42}  /* 2^1 */
};

#endif /* __BINARY_CLOCK_RECTANGLES_H__ */
