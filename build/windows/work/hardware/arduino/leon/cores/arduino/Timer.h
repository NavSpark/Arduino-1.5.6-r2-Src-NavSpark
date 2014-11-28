/*
  Timer.h - Header file of library for Timer
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

	Created 28 Feb. 2014 by Ming-Jen Chen

	$Id$
*/

#ifndef _TIMER_H_
#define _TIMER_H_

class TIMER
{
	private:
		bool enabled;
		uint8_t	timerId;
		uint16_t count;

		void (*userFuncForTimer)(void);

	public:
		TIMER(uint8_t tmrId);
		TIMER(void);

		// function to return if timer is idle
		bool isIdle(void);

		// wrapper for "userFuncForTimer"
		void isr(void);

		// functions for starting timer, period in unit of mili-second
		uint8_t every(uint32_t period, void (*callback)(void));
		uint8_t every(uint32_t period, void (*callback)(void), uint16_t repeatCount);
		uint8_t after(uint32_t period, void (*callback)(void));

		// function to check if timer expires
		bool expire(void);

		// function to stop timer
		void stop(void);

		uint16_t remainTimes(void);
};

extern TIMER Timer0;
extern TIMER Timer1;
extern TIMER Timer2;

#endif // _TIMER_H_
