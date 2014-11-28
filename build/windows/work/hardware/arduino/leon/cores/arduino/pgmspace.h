/*
  pgmspace.h - header file for Arduino/AVR PGM functions (for compatiable
               purpose only)
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

// Note: In original AVR environment, the size of flash is much larger than
//       what SRAM can support. The AVR provides a mechanism to use space on
//       flash as "pseudo SRAM" to store some data by adding "PROGMEM" after
//       declaration of variables. However, in LEON-3 the size of SRAM is up
//       to 192KB and it is unnecessary to use such technology.

//#define	__ATTR_PROGMEM__	__attribute__((__progmem__))
//#define	PROGMEM	__ATTR_PROGMEM__
//
#define	PROGMEM

// Note: As same as above reason, the assembly access used in AVR's PROGMEM
//       is no longer needed and is replaced by normal C pointer coding.
/*
#define __LPM_enhanced__(addr)	\
(__extension__({	\
	uint16_t __addr16 = (uint16_t)(addr);	\
	uint8_t __result;	\
	__asm__	\
	(	\
		"lpm %0, Z" "\n\t"	\
		: "=r" (__result)	\
		: "z" (__addr16)	\
	);	\
	__result;	\
}))

#define __LPM_word_enhanced__(addr)	\
(__extension__({	\
	uint16_t __addr16 = (uint16_t)(addr);	\
	uint16_t __result;	\
	__asm__	\
	(	\
		"lpm %A0, Z+"   "\n\t"	\
		"lpm %B0, Z"    "\n\t"	\
		: "=r" (__result), "=z" (__addr16)	\
		: "1" (__addr16)	\
	);	\
	__result;	\
}))

#define __LPM_dword_enhanced__(addr)	\
(__extension__({	\
	uint16_t __addr16 = (uint16_t)(addr);	\
	uint32_t __result;	\
	__asm__	\
	(	\
		"lpm %A0, Z+"   "\n\t"	\
		"lpm %B0, Z+"   "\n\t"	\
		"lpm %C0, Z+"   "\n\t"	\
		"lpm %D0, Z"    "\n\t"	\
		: "=r" (__result), "=z" (__addr16)	\
		: "1" (__addr16)	\
	);	\
	__result;	\
}))
*/

//#define	__LPM(addr)					__LPM_enhanced__(addr)
//#define	__LPM_word(addr)		__LPM_word_enhanced__(addr)
//#define	__LPM_dword(addr)		__LPM_dword_enhanced__(addr)
//#define	pgm_read_byte_near(address_short)		__LPM((uint16_t)(address_short))
//#define	pgm_read_word_near(address_short)		__LPM_word((uint16_t)(address_short))
//#define	pgm_read_dword_near(address_short)	__LPM_dword((uint16_t)(address_short))
//#define	pgm_read_byte(address_short)	pgm_read_byte_near(address_short)
//#define	pgm_read_word(address_short)	pgm_read_word_near(address_short)
//#define	pgm_read_dword(address_short)	pgm_read_dword_near(address_short)
//
#define	pgm_read_byte(exp)	((unsigned char) *exp)
#define	pgm_read_word(exp)	((unsigned short) *exp)
#define	pgm_read_dword(exp)	((unsigned int) *exp)
