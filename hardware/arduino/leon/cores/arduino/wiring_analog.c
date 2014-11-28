/*
  wiring_analog.c - C file for Arduino analog I/O functions
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

#ifdef __cplusplus
extern "C" {
#endif

// declarations for global variables
static uint8_t	V822_adc2_analog_reference = EXTERNAL;

// **********************************************************************
// Description: Set the output frequency of PWM pins
//
// Syntax: analogPWMPeriod(pin, value)
//      -- pin: the number of the PWM pin you want to set output frequency
//      -- value: the desired period (in unit of ns) of PWM
//                the range of legal period. is 1000ns ~ 1000000000ns
// **********************************************************************
// C body has been moved to static library

// **********************************************************************
// Description: Set the output frequency of PWM pins
//
// Syntax: analogPWMFreq(pin, value)
//      -- pin: the number of the PWM pin you want to set output frequency
//      -- value: the desired frequency (in Hz) of PWM
//                the range of legal freq. is 1Hz ~ 1MHz
// **********************************************************************
// C body has been moved to static library

// **********************************************************************
// Description: Set the ratio of desired duty cycle of PWM pins
//
// Syntax: analogWrite(pin, value)
//      -- pin: the number of the PWM pin you want to configure
//      -- value: the ratio of PWM, 0 => duty cycle 0%
//                                  V822_PWM0_DUTY_RESOLUTION => 100%
// **********************************************************************
void analogWrite( uint8_t pin, uint16_t value )
{
	if (pin != GPIO20_PWM0) return;
	gpioConf[pin].io_mode = 0xff;	// prevent user from calling digital functions when PWM is active
	analogPWMWrite(pin, value);
}
//
// Example-1: Following codes will generate a PWM on pin 20 with total period
//            2.5us (400KHz) and duty cycle 20%
//
// analogPWMPeriod(20, 2500); or analogPWMFreq(20, 400000);
// analogWrite(20, 51);	// 255 * 20% = 51
//
// Example-2: Following codes will turn off the PWM
//
// analogPWMFreq(20, 0);
// analogWrite(20, 0);
//

// **********************************************************************
// Description: Configure the sampling frequency of ADC
//
// Syntax: analogADCClock(pin, value)
//      -- pin: the number of the ADC pin
//      -- value: the frequency of sampling clock desired for ADC
// **********************************************************************
#if 0
__attribute__((weak)) void analogADCClock(uint8_t pin, uint32_t value)
{
	// C body has been moved to static library
}
#endif

// **********************************************************************
// Description: Configures the reference voltage used for analog input
//
// Syntax: analogReference(mode)
//      -- mode: DEFAULT, INTERNAL or EXTERNAL
// **********************************************************************
void analogReference(uint8_t mode)
{
	// only external is supported
	V822_adc2_analog_reference = EXTERNAL;
}

// **********************************************************************
// Description: Read the 10-bit output value of ADC
//
// Syntax: analogRead(pin)
//      -- ret: the 10-bit output of ADC
//      -- pin: the number of the ADC pin you want to read
// **********************************************************************
#if 0
__attribute__((weak)) uint16_t analogRead(uint8_t pin)
{
	// C body has been moved to static library
	return 0;
}
#endif

#ifdef __cplusplus
}
#endif
