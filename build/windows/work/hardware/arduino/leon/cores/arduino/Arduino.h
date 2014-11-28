/*
  Arduino.h - header file for Arduino global definitions
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

#ifndef _ARDUINO_H_
#define _ARDUINO_H_

#ifndef	ST_CONST_INCLUDED
#if defined(ST_CONST_SEL) && (ST_CONST_SEL==1)
#include "st_const_gps.h"
#endif
#if defined(ST_CONST_SEL) && (ST_CONST_SEL==2)
#include "st_const_gps_bd2.h"
#endif
#if defined(ST_CONST_SEL) && (ST_CONST_SEL==3)
#include "st_const_gps_gln.h"
#endif
#endif

#ifndef ST_TYPE_INCLUDED
#include "st_type.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

// declarations for Arduino sketch (compiled by g++)
void init(void);
void setup(void);
void loop(void);
void task_called_after_GNSS_update(void);

// defines for BUILD info.
// the format of __DATE__ is "mmm dd yyyy", e.g. "Jan 14 2012".
#define BUILD_YEAR \
( (__DATE__ [7]  - '0') * 1000 + \
  (__DATE__ [8]  - '0') * 100  + \
  (__DATE__ [9]  - '0') * 10   + \
  (__DATE__ [10] - '0') )

#define BUILD_DAY \
( ( (__DATE__ [4] == ' ') ? 0 : (__DATE__ [4] - '0')) * 10 + \
	(__DATE__ [5] - '0') )

#define BUILD_MONTH \
( (__DATE__ [2] == 'n') ? (__DATE__ [1] == 'a' ? 1 : 6) \
: (__DATE__ [2] == 'b') ? 2 \
: (__DATE__ [2] == 'r') ? (__DATE__ [0] == 'M' ? 3 : 4) \
: (__DATE__ [2] == 'y') ? 5 \
: (__DATE__ [2] == 'l') ? 7 \
: (__DATE__ [2] == 'g') ? 8 \
: (__DATE__ [2] == 'p') ? 9 \
: (__DATE__ [2] == 't') ? 10 \
: (__DATE__ [2] == 'v') ? 11 : 12 )

#define	NAVSPARK_REVISION	(((BUILD_YEAR%100)<<16) | (BUILD_MONTH<<8) | BUILD_DAY)
#define	NAVSPARK_VERSION	0x010101

// Unmark following define to use "gnss_sti_sprintf()" to replace
// "sprintf()" for saving memory. However, the "gnss_sti_sprintf()"
// is under development and is not bug-free.
#if defined(USE_UART1_FOR_NMEA) && (USE_UART1_FOR_NMEA==1)
#define USE_GNSS_STI_SPRINTF
#endif

#ifndef NULL
#define	NULL	0
#endif

// defines for logic level
#define HIGH	0x1
#define LOW		0x0

// defines for I/O direction
#define INPUT		0x0
#define OUTPUT	0x1
#define	SPECIAL	0xff

// defines for boolean
#define true	0x1
#define false	0x0

// defines for GPIO interrupt trigger
#define	FALLING	0x2
#define	RISING	0x3

// defines for analog reference
#define DEFAULT		0
#define EXTERNAL	1
#define INTERNAL	2

// defines for shift functions
#define LSBFIRST	0
#define MSBFIRST	1

// defines for constants
#define DEG_TO_RAD	0.017453292519943295769236907684886
#define RAD_TO_DEG	57.295779513082320876798154814105
#define PI	3.1415926
#define TWO_PI	(2 * PI)

// defines for pseudo functions
#define min(a,b)	((a)<(b)?(a):(b))
#define max(a,b)	((a)>(b)?(a):(b))
#define constrain(amt,low,high)	((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)	((x)>=0?(uint32_t)((x)+0.5):(long)((x)-0.5))
#define radians(deg)	((deg)*DEG_TO_RAD)
#define degrees(rad)	((rad)*RAD_TO_DEG)
#define sq(x)			((x)*(x))
//#define abs(x)		((x)>0?(x):-(x))

#define lowByte(w)	((uint8_t) (((uint32_t)w) & 0x000000ff))
#define highByte(w) ((uint8_t) ((((uint32_t)w) & 0xff000000)>>24))

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#define bit(b) (1UL << (b))

#ifdef __cplusplus
} // extern "C"
#endif

#define	V822A_PACKAGE

// include files for C
#include "pins_arduino.h"
#include "wiring.h"
#include "wiring_digital.h"
#include "wiring_analog.h"
#include "wiring_utoa.h"
#include "wiring_intr.h"

// include files for C++
#ifdef __cplusplus
#include "WMath.h"
#include "WCharacter.h"
#include "WString.h"
#include "Timer.h"
#include "HardwareSerial.h"
#include "TwoWire.h"
#include "SPI.h"
#include "GNSS.h"
#endif

#endif // _ARDUINO_H_

// Note: The memory map for Venus-8 series are shown in below:
//
// 0x00000000 ~ 0x1FFFFFFF	cachable area (Flash)
// 0x20000000 ~ 0x3FFFFFFF	I/O area (control register)
// 0x40000000 ~ 0x5FFFFFFF  external RAM area (SRAM)
// 0x60000000 ~ 0x7FFFFFFF	internal RAM area (192KB now)

