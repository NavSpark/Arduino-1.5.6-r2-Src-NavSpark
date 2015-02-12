/*
  wiring_shift.c - C file for Arduino pseudo serial functions
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

#include "Arduino.h"
#include "wiring_digital.h"
#include "wiring_shift.h"

#ifdef __cplusplus
extern "C" {
#endif

// **********************************************************************
// Description: Shifts out a byte of data bit-by-bit with clock toggled
//
// Syntax: shiftOut(dataPin, clockPin, bitOrder, val)
//      -- dataPin: the number of the pin on which the serial data is out
//      -- clockPin: the number of the pin as a clock pin to toggle
//      -- bitOrder: flag to indicate which bit (MSB or LSB) is sent first
//      -- val: the 8-bit data to be sent serially on dataPin
// **********************************************************************
void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val)
{
	uint8_t k;

	// check if dataPin and clockPin are in OUTPUT mode

	// drive the data serially out
	digitalWrite(clockPin, LOW);
	for (k = 0; k < 8; k++) {
		if (bitOrder == LSBFIRST) {
			digitalWrite(dataPin, (val >> k) & 0x01);
		}
		else if (bitOrder == MSBFIRST) {
			digitalWrite(dataPin, (val >> (7-k)) & 0x01);
		}
		digitalWrite(clockPin, HIGH); // rising edge
		digitalWrite(clockPin, LOW);
	}
}

// **********************************************************************
// Description: Shifts in a byte of data bit-by-bit with clock toggled
//
// Syntax: ret = shiftIn(dataPin, clockPin, bitOrder)
//      -- ret: 8-bit data received on dataPin
//      -- dataPin: the number of the pin on which the serial data is received
//      -- clockPin: the number of the pin as a clock pin to toggle
//      -- bitOrder: flag to indicate where to place the 1st received bit
// **********************************************************************
uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder)
{
	uint8_t value = 0;
	uint8_t k;

	// check if dataPin is in INPUT mode and clockPin is in OUTPUT mode

	// sample the data in serially
	digitalWrite(clockPin, LOW);
#ifdef SHIFT_IN_PRE_CLOCK
	digitalWrite(clockPin, HIGH); // rising edge
	digitalWrite(clockPin, LOW);
#endif
	for (k = 0; k < 8; k++) {
		if (bitOrder == LSBFIRST) {
			value |= (digitalRead(dataPin) << k);
		}
		else if (bitOrder == MSBFIRST) {
			value |= (digitalRead(dataPin) << (7-k));
		}
		digitalWrite(clockPin, HIGH); // rising edge
		digitalWrite(clockPin, LOW);
	}
	return value;
}

#ifdef __cplusplus
}
#endif
