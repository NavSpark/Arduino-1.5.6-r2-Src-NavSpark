/*
  stdint.h - Header file for standard types of 8/16/32/64-bit

  Copyright (c) 2009-10 Hernando Barragan.  All right reserved.
  Copyright 2011, Paul Stoffregen, paul@pjrc.com

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

  Modified 25 December 2013 by Ming-Jen Chen

  $Id$
*/

#ifndef _STDINT_H_
#define _STDINT_H_

typedef signed char		CHAR;
typedef unsigned char	UCHAR;
typedef unsigned char	BYTE;
typedef	unsigned char	uint8_t;

typedef short	SHORT;
typedef short int16_t;
typedef unsigned short	USHORT;
typedef unsigned short	WORD;
typedef unsigned short	WCHAR;
typedef	unsigned short	uint16_t;

typedef int	INT;
typedef	signed int		int32_t;
typedef unsigned int	UINT;
typedef	unsigned int	uint32_t;

typedef long	LONG;
typedef unsigned long	ULONG;
typedef unsigned long	DWORD;

typedef uint8_t boolean;
typedef uint8_t byte;

#endif // _STDINT_H_
