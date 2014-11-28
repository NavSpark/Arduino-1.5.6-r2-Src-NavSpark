/*
  wiring_utoa.c - C file for Arduino digital-to-string functions
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

#ifdef __cplusplus
extern "C" {
#endif

// convert signed 16-bit digits to string
void itoa(short n, char* buffer, unsigned char base)
{
	unsigned short i, j;
	char digit;

	// if value is negative, then turn it to positive and add "-" sign
	if (n < 0) {
		n = 0 - n;
		buffer[0] = '-';
	}
	// add "+" sign
	else { buffer[0] = '+'; }

	// convert digits of "n" into buffer in reverse order
	for (i=1; n>0; i++,n=n/base)
	{
		j = n % base;
		buffer[i] = (j > 9) ? (j + 'A' - 10) : (j + '0');
	}
	buffer[i] = '\0';	// terminator

	// inverse the order of digit again
	for (j=i-1,i=1; j>=(i+1); j--,i++)
	{
		digit = buffer[j]; buffer[j] = buffer[i]; buffer[i] = digit;
	}
}

// convert unsigned 16-bit digits to string
void utoa(unsigned short n, char* buffer, unsigned char base)
{
	unsigned short i, j;
	char digit;

	// convert digits of "n" into buffer in reverse order
	for (i=0; n>0; i++,n=n/base)
	{
		j = n % base;
		buffer[i] = (j > 9) ? (j + 'A' - 10) : (j + '0');
	}
	buffer[i] = '\0';	// terminator

	// inverse the order of digit again
	for (j=i-1,i=0; j>=(i+1); j--,i++)
	{
		digit = buffer[j]; buffer[j] = buffer[i]; buffer[i] = digit;
	}
}

// convert signed 32-bit digits to string
void ltoa(long l, char* buffer, unsigned char base)
{
	unsigned long i, j;
	char digit;

	// if value is negative, then turn it to positive and add "-" sign
	if (l < 0) {
		l = 0 - l;
		buffer[0] = '-';
	}
	// add "+" sign
	else { buffer[0] = '+'; }

	// convert digits of "n" into buffer in reverse order
	for (i=1; l>0; i++,l=l/base)
	{
		j = l % base;
		buffer[i] = (j > 9) ? (j + 'A' - 10) : (j + '0');
	}
	buffer[i] = '\0';	// terminator

	// inverse the order of digit again
	for (j=i-1,i=1; j>=(i+1); j--,i++)
	{
		digit = buffer[j]; buffer[j] = buffer[i]; buffer[i] = digit;
	}
}

// convert unsigned 32-bit digits to string
void ultoa(unsigned long l, char* buffer, unsigned char base)
{
	unsigned long i, j;
	char digit;

	// convert digits of "n" into buffer in reverse order
	for (i=0; l>0; i++,l=l/base)
	{
		j = l % base;
		buffer[i] = (j > 9) ? (j + 'A' - 10) : (j + '0');
	}
	buffer[i] = '\0';	// terminator

	// inverse the order of digit again
	for (j=i-1,i=0; j>=(i+1); j--,i++)
	{
		digit = buffer[j]; buffer[j] = buffer[i]; buffer[i] = digit;
	}
}

#ifdef __cplusplus
}
#endif
