/*
  wiring_pulse.c - C file for Arduino pulse measurment function
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
#include "wiring.h"
#include "wiring_digital.h"

#ifdef __cplusplus
extern "C" {
#endif

// **********************************************************************
// Description: Reads a pulse (either HIGH or LOW) on a pin and calculate
//              the how long it lasted in miliseconds.
//
// Syntax: ret = pulseIn(pin, state, timeout)
//      -- ret: either one of following values
//         (1) ret = timeout: the state specified on the specified pin kept
//                   for more than "timeout" value
//         (2) ret = 0: the state specified never appears on the specified
//                      pin
//         (3) otherwise: the duration in ms for the pulse
//      -- pin: pin number specified to detect the pulse
//      -- state: HIGH or LOW for pulse to detect
//      -- timeout: expiration time in ms
// **********************************************************************
unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout)
{
	boolean	state_found = false;
	boolean	probe_done = false;
	uint8_t	state_inv;
	unsigned long	begin_time;
	unsigned long	probe_time;

	state_inv = (state == HIGH) ? LOW : HIGH;

	// check if the pin specified is in INPUT mode

	// record current time
	begin_time = millis();
	probe_time = begin_time;
	// waits for the "state"
	while (state_found == false) {
		if ((millis()-begin_time)>=timeout) {
			return 0;
		}
		else {
			if (digitalRead(pin) == state) {
				probe_time = millis();
				state_found = true;
			}
		}
	}
	// waits for the "state" changes
	while (probe_done == false) {
		if ((millis()-begin_time)>=timeout) {
			return timeout;
		}
		else {
			if (digitalRead(pin) == state_inv) {
				probe_done = true;
			}
		}
	}
	return (millis() - probe_time);
}

#ifdef __cplusplus
}
#endif
