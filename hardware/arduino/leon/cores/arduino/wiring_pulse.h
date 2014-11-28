/*
  wiring_pulse.h - header file for Arduino pulse measurment function
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

#ifndef _WIRING_PULSE_
#define _WIRING_PULSE_

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

// declarations for public functions
extern unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout);

#ifdef __cplusplus
}
#endif

#endif // _WIRING_PULSE_
