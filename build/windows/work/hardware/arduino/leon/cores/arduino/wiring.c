/*
  wiring.c - C file for Arduino misc functions
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

#include <stdio.h>
#include "stdint.h"
#include "sti_gnss_lib.h"
#include "Arduino.h"
#include "wiring_intr.h"

extern void hwISRFunc(void);

#ifdef __cplusplus
extern "C" {
#endif

volatile U32 msBootTime;
volatile U32 msDownCounter;

void init(void)
{
	// user-defined system version
	gnss_system_version(NAVSPARK_VERSION, NAVSPARK_REVISION);

  // turn off timers
  gnss_timer_disable(0);
  gnss_timer_disable(1);
  gnss_timer_disable(2);

  // setup the callback function for milli-second interrupt
  msBootTime = 0;
  msDownCounter = 0;
	gnss_1ms_setup_callbacks(msTaskFunc);

	// setup the ISR for UART/Two-Wire/SPI
	gnss_hw_peripheral_isr_setup_callbacks(hwISRFunc);

	// NOTE: due to the request to change GNSS parameters by user, the
	//       initialization to timer ISR was removed to "GNSS.cpp" so
	//       it can be done after calling gnss_init().
}

// **********************************************************************
// Description: The function will be excuted every 1 ms
// **********************************************************************
void msTaskFunc(void)
{
	// up count timer for millis()
	msBootTime ++;
	// down count timer for delay()
	if (msDownCounter != 0) msDownCounter --;
}

// **********************************************************************
// Description: Returns the time in milliseconds since the Arduino board
//              began running the current program the how long it lasted
//              in miliseconds.
//
// Syntax: ret = millis()
//      -- ret: the time in unit of milliseconds
// **********************************************************************
unsigned long millis()
{
	return msBootTime;
}

// **********************************************************************
// Description: Pauses your program for the amount of time (in miliseconds)
//              specified as parameter.
//
// Syntax: delay(ms)
//      -- ms: the time in unit of milliseconds by which to pause
// **********************************************************************
void delay(unsigned long ms)
{
	volatile U32 i;

	gnss_watchdog_disable();
	msDownCounter = ms;
	while (msDownCounter > 0) {
		for (i = 0; i < 100; i++) { asm("nop"); }
	}
	gnss_watchdog_enable(5);
}

// **********************************************************************
// Description: Pauses your program for the amount of time (in microseconds)
//              specified as parameter.
//
// Syntax: delayMicroseconds(us)
//      -- us: the time in unit of microseconds by which to pause
// **********************************************************************
void delayMicroseconds(unsigned int us)
{
	volatile U32 i;
	U32	cpu_clk_pulse;
	U32	cpu_nop_cycle;

	cpu_clk_pulse = 1000000000UL / gnss_get_cpu_clock();
	cpu_nop_cycle = us * 1000 / cpu_clk_pulse / 7; // 7 is the number of machine instructions
	for (i = 0; i < cpu_nop_cycle; i++) {
		asm("nop");
	}
}

#ifdef __cplusplus
}
#endif
