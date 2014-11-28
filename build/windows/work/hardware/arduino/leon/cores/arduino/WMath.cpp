/*
  WMath.cpp - C++ file for Arduino mathematic functions
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

extern "C" {
  #include <stdlib.h>
}

// **********************************************************************
// Description: Re-maps a number from range in [in_min, in_max] to another
//              range in [out_min, out_max].
//
// Syntax: ret = map(x, in_min, in_max, out_min, out_max)
//      -- ret: the number after re-map
//      -- x: the number before re-map
//      -- in_min: lower bound of original range
//      -- in_max: upper bound of original range
//      -- out_min: lower bound of new range
//      -- out_max: upper bound of new range
// **********************************************************************
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// **********************************************************************
// Description: initializes the pseudo-random number generator, causing
//              it to start at an arbitrary point in its random sequence.
//
// Syntax: randomSeed(seed)
//      -- seed: seed of pseudo-random generator
// **********************************************************************
void randomSeed(unsigned int seed)
{
  if (seed != 0) srand(seed);
}

// **********************************************************************
// Description: Function to generate pseudo-random number in the range
//              from 0 to specified number.
//
// Syntax: random(howbig)
//      -- howbig: the upperbound of random number, the maximum number
//                 can be returned is "howbig - 1"
// **********************************************************************
long random(long howbig)
{
  if (howbig == 0) { return 0; }
  else {
  	return map(rand(), 0, RAND_MAX, 0, howbig);
  }
}

// **********************************************************************
// Description: Function to generate pseudo-random number in the range
//              inbetween specified numbers.
//
// Syntax: random(howsmall, howbig)
//      -- howsmall: the lowerbound of random number
//      -- howbig: the upperbound of random number, the maximum number
//                 can be returned is "howbig - 1"
// **********************************************************************
long random(long howsmall, long howbig)
{
  if (howsmall >= howbig) { return howsmall; }
  else {
  	return map(rand(), 0, RAND_MAX, howsmall, howbig);
  }
}
