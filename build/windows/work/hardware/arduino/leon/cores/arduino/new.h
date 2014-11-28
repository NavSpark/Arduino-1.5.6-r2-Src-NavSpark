/*
  new.h - header file for Arduino operator new/delete
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

#ifndef _NEW_H_
#define _NEW_H_

#include <stdlib.h>

#ifdef __cplusplus

/* ***************************************************************** */
// NOTE:
// -- The implementation of C++ operator "new" uses "malloc()" provided
//    in GNU's stdlib.h and heap space, in case of calling "new" without
//    some limitations the heap space may collide with stack space during
//    run-time.
// -- To avoid such case, we implemented a "fix heap" for "new" operation
//    and user can define following directive to enable this mechanism.
//#define VIRTUAL_HEAP_FOR_CPLUSPLUS_OPERATOR_NEW
/* ***************************************************************** */

#ifdef VIRTUAL_HEAP_FOR_CPLUSPLUS_OPERATOR_NEW
// to define constant(s) for HEAP
#define	VIRTUAL_HEAP_BLK_NUM	480
#define	VIRTUAL_HEAP_BLK_SIZ	64
#define	VIRTUAL_HEAP_SIZ			(VIRTUAL_HEAP_BLK_NUM * VIRTUAL_HEAP_BLK_SIZ)
#define	VIRTUAL_HEAP_EMPTY		0xffffffff
// declaration of link-list structure for heap
typedef struct _heap_lnklist
{
	unsigned int	prev;
	unsigned int	next;
	void					*addr;
}	heap_lnklist, *p_heap_lnklist;
#endif // VIRTUAL_HEAP_FOR_CPLUSPLUS_OPERATOR_NEW

void * operator new(size_t size);
void * operator new[](size_t size);
void operator delete(void * ptr);
void operator delete[](void * ptr);

__extension__ typedef int __guard __attribute__((mode (__DI__)));

extern "C" int	__cxa_guard_acquire(__guard *);
extern "C" void __cxa_guard_release(__guard *);
extern "C" void __cxa_guard_abort(__guard *);
extern "C" void __cxa_pure_virtual(void);

#endif // __cplusplus

#endif // _NEW_H_

