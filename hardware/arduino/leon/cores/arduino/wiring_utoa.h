/*
  wiring_utoa.h - header file for Arduino digital-to-string functions
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

#ifndef _WIRING_UTOA_
#define _WIRING_UTOA_

#ifdef __cplusplus
extern "C" {
#endif

// declarations for public functions
extern void itoa(short n, char* buffer, unsigned char base);
extern void utoa(unsigned short n, char* buffer, unsigned char base);
extern void ltoa(long l, char* buffer, unsigned char base);
extern void ultoa(unsigned long l, char* buffer, unsigned char base);

#ifdef __cplusplus
}
#endif

#endif // _WIRING_UTOA_
