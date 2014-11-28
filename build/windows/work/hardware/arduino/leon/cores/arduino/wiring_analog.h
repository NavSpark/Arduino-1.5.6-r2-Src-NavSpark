/*
  wiring_analog.h - header file for Arduino analog I/O functions
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

#ifndef _WIRING_ANALOG_
#define _WIRING_ANALOG_

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

extern sGPIOConf gpioConf[33];

// declarations for PWM variables
extern uint32_t v822_pwm0_out_hz;
// declarations for PWM functions
extern void analogPWMPeriod( uint8_t pin, uint32_t value );
extern void analogPWMFreq( uint8_t pin, uint32_t value );
extern void analogPWMWrite( uint8_t pin, uint16_t value );
void analogWrite( uint8_t pin, uint16_t value );

// declarations for ADC variables
extern uint32_t v822_adc2_sample_hz;
extern uint8_t	v822_adc2_sample_div;
// declarations for ADC functions
extern void analogADCClock( uint8_t pin, uint32_t value );
extern void analogReference(uint8_t mode);
extern uint16_t analogRead( uint8_t pin );

#ifdef __cplusplus
}
#endif

#endif // _WIRING_ANALOG_
