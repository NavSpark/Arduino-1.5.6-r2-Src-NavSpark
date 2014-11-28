/*
  wiring_shift.h - header file for Arduino pseudo serial functions
  Copyright (c) 2013 NavSpark.

  This library is free software; you can redistribute it under the terms
  of the GNU Lesser General Public License as published by the Free Software
  Foundation; either version 2.1 of the License, or (at your option) any
  later version.

  This library is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
  or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
  License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this library; if not, write to the Free Software Foundation,
  Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

	Created 25 December 2013 by Ming-Jen Chen

	$Id$
*/

#ifndef _WIRING_SHIFT_
#define _WIRING_SHIFT_

#include "stdint.h"

#define	PCD8544_INSTRUCTION	0
#define	PCD8544_NORMAL_DATA	1
#define	PCD8544_RESET				2

typedef	struct _pcd844_data
{
	uint16_t numData;
	uint8_t *type; // 0: instruction, 1: normal data, 2: reset
  uint8_t *data;
} sPCD8544Data, *pPCD8544Data;

typedef	struct _pcd844_digit
{
	uint8_t type[5];
  uint8_t data[5];
} sPCD8544Digit, *pPCD8544Digit;

#ifdef __cplusplus
extern "C" {
#endif

// unmark following line in case of one more clock needed before sampling
// 1st bit of data in "shiftIn()"
#define	SHIFT_IN_PRE_CLOCK

// declarations for public functions
void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val);
uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder);

// defines for pins for PCD8544
#define	PCD8544_LED_PIN		3
#define	PCD8544_SCE_PIN		10
#define	PCD8544_DC_PIN		16	// gpio 22 has bug waiting for fix
#define	PCD8544_SCLK_PIN	30
#define	PCD8544_SDIN_PIN	28
#define	PCD8544_RES_PIN		5

void pcd8544SerialBus(uint8_t sce_n, uint8_t dc_n, uint8_t sclk, uint8_t sdin, uint8_t res_n, pPCD8544Data ptr);
void pcd8544Init(void);
void pcd8544CleanScreen(void);
void pcd8544SetCursor(uint8_t bank, uint8_t x);
void pcd8544DigitPlot(void);
void pcd8544FractalPlot(void);

#ifdef __cplusplus
}
#endif

#endif // _WIRING_SHIFT_
