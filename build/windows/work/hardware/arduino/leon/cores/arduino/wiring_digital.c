/*
  wiring_digital.c - C file for Arduino digital I/O functions
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

#include "stdint.h"
#include "sti_gnss_lib.h"
#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

// NOTE: Please do NOT modify "gpioConf" directly unless you know very clear
//       about what you do.
#if defined(V822A_PACKAGE)
sGPIOConf gpioConf[33] = {
	{GPIO0_LED, 0x02, 0xff, 0xff, NULL},
	{GPIO1_RXD2, 0x03, 0xff, 0xff, NULL},
	{GPIO2_TXD2, 0x03, 0xff, 0xff, NULL},
	{GPIO3_P1PPS, 0x02, 0xff, 0xff, NULL},
	{GPIO4_SCL, 0x03, 0xff, 0xff, NULL},
	{GPIO5_SDA, 0x03, 0xff, 0xff, NULL},
	{GPIO6_WHEEL_TIC_MEAS, 0x03, 0xff, 0xff, NULL},
	{0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0},
	{GPIO10_TRIG, 0x03, 0xff, 0xff, NULL},
	{0, 0, 0, 0, 0},
	{GPIO12_THERM_IN1, 0x03, 0xff, 0xff, NULL},
	{GPIO13_THERM_OUT1, 0x03, 0xff, 0xff, NULL},
	{GPIO14_THERM_IN2, 0x03, 0xff, 0xff, NULL},
	{0, 0, 0, 0, 0},
	{GPIO16_CHEAP_XTALO, 0x03, 0xff, 0xff, NULL},
	{0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0},
	{GPIO20_PWM0, 0x03, 0xff, 0xff, NULL},
	{0, 0, 0, 0, 0},
	{GPIO22_SPISL_MCS1, 0x03, 0xff, 0xff, NULL},
	{0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0},
	{GPIO28_SPIMS_CSN, 0x03, 0xff, 0xff, NULL},
	{GPIO29_SPIMS_SCK, 0x03, 0xff, 0xff, NULL},
	{GPIO30_SPIMS_MOSI, 0x03, 0xff, 0xff, NULL},
	{GPIO31_SPIMS_MISO, 0x03, 0xff, 0xff, NULL},
	{0, 0, 0, 0, 0}
};
#elif defined(V822_PACKAGE)
sGPIOConf gpioConf[33] = {
	{GPIO0_LED, 0x02, 0xff, 0xff, NULL},
	{GPIO1_RXD2, 0x03, 0xff, 0xff, NULL},
	{GPIO2_TXD2, 0x03, 0xff, 0xff, NULL},
	{GPIO3_P1PPS, 0x02, 0xff, 0xff, NULL},
	{GPIO4_SCL, 0x03, 0xff, 0xff, NULL},
	{GPIO5_SDA, 0x03, 0xff, 0xff, NULL},
	{0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0},
	{GPIO8, 0x03, 0xff, 0xff, NULL},
	{0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0},
	{GPIO12_THERM_IN1, 0x03, 0xff, 0xff, NULL},
	{GPIO13_THERM_OUT1, 0x03, 0xff, 0xff, NULL},
	{GPIO14_THERM_IN2, 0x03, 0xff, 0xff, NULL},
	{GPIO15_THERM_OUT2, 0x03, 0xff, 0xff, NULL},
	{GPIO16_CHEAP_XTALO, 0x03, 0xff, 0xff, NULL},
	{0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0},
	{GPIO20_PWM0, 0x03, 0xff, 0xff, NULL},
	{0, 0, 0, 0, 0},
	{GPIO22_SPISL_MCS1, 0x03, 0xff, 0xff, NULL},
	{0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0},
	{GPIO28_SPIMS_CSN, 0x03, 0xff, 0xff, NULL},
	{GPIO29_SPIMS_SCK, 0x03, 0xff, 0xff, NULL},
	{GPIO30_SPIMS_MOSI, 0x03, 0xff, 0xff, NULL},
	{GPIO31_SPIMS_MISO, 0x03, 0xff, 0xff, NULL},
	{0, 0, 0, 0, 0}
};
#else

#endif

// **********************************************************************
// Description: Configures the specified pin to behave either as an input
//              or an output.
//
// Syntax: pinMode(pin, mode)
//      -- pin: the number of the pin whose mode you wish to set. (int)
//      -- mode: either INPUT or OUTPUT.
// **********************************************************************
void pinMode(uint8_t pin, uint8_t mode)
{
	if ((gpioConf[pin].io_capability & 0x01) && (mode == INPUT))
	{
		gpioConf[pin].io_mode = INPUT;
		gnss_gpio_set_mux_mode(pin, 1);
		gnss_gpio_set_input(pin);
	}

	if ((gpioConf[pin].io_capability & 0x02) && (mode == OUTPUT))
	{
		gpioConf[pin].io_mode = OUTPUT;
		gnss_gpio_set_mux_mode(pin, 1);
		gnss_gpio_set_output(pin);
	}

	if (mode == SPECIAL)
	{
		gpioConf[pin].io_mode = SPECIAL;
		gnss_gpio_set_mux_mode(pin, 0);
	}
}

// **********************************************************************
// Description: Sets a pin configured as OUTPUT to either a HIGH or a LOW
//              state at the specified pin.
//
// Syntax: digitalWrite(pin, val)
//      -- pin: the number of the pin whose value you wish to set. (int)
//      -- mode: either HIGH or LOW.
// **********************************************************************
void digitalWrite(uint8_t pin, uint8_t val)
{
	// check if the specified pin is in OUTPUT mode
	if (gpioConf[pin].io_mode != OUTPUT) return;

	// set the output value
	if      (val == HIGH)	gnss_gpio_high(pin);
	else if (val == LOW)	gnss_gpio_low(pin);
}

// **********************************************************************
// Description: Reads the value from a specified digital pin, it returns
//              either HIGH or LOW.
//
// Syntax: ret = digitalRead(pin)
//      -- ret: HIGH, LOW or -1
//      -- pin: the number of the pin whose value you wish to read. (int)
// **********************************************************************
int digitalRead(uint8_t pin)
{
	// check if the specified pin is in INPUT mode
	if (gpioConf[pin].io_mode != INPUT) return -1;

	// read the level on pin
	return gnss_gpio_read_input(pin);
}

#ifdef __cplusplus
}
#endif
