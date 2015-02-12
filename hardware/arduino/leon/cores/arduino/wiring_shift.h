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

#ifdef __cplusplus
extern "C" {
#endif

// unmark following line in case of one more clock needed before sampling
// 1st bit of data in "shiftIn()"
#define	SHIFT_IN_PRE_CLOCK

// declarations for public functions
void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val);
uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder);

#ifdef __cplusplus
}
#endif

#endif // _WIRING_SHIFT_
