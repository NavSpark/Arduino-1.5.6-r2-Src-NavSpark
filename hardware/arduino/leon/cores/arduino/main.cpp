/*
  main.cpp - main C++ file for Arduino
  Copyright (c) 2013 NavSpark.  All right reserved.

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

#include <asm-leon/irq.h>
#include "sti_gnss_lib.h"
#include "Arduino.h"
#include "wiring_shift.h"

// -- Note --
// In case of not using "-fno-exceptions" as option of sparc-elf-g++, declare
// the variable in below for linking issue.
// void *__gxx_personality_v0;

// -- Note --
// To provide a default function of "task_called_after_GNSS_update" which can be
// overwritten in sketch for any task relative to GNSS or function should be excuted
// every update.
__attribute__((weak)) void task_called_after_GNSS_update(void)
{
	// please create a function with same name in sketch if necessary
	// , adding your code at here directly is NOT recommended
}

// -- Note --
// The entry of Arduino user program.
int main(void)
{
	init(); // defined in "wiring.c"
	setup(); // defined in sketch

	for (;;) {
		if (gnss_process())
		{
			// add necessary code which must be executed every update here
			task_called_after_GNSS_update();
		}
		gnss_uart_process();
		loop(); // defined in sketch
	}
	gnss_close();

	return 0;
}

