/*
  new.cpp - C++ file for Arduino operator new/delete
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

#include "new.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#ifdef VIRTUAL_HEAP_FOR_CPLUSPLUS_OPERATOR_NEW
// global heap variables
static heap_lnklist	heapLnkList[VIRTUAL_HEAP_BLK_NUM];
static unsigned int	headHeapPool;
static unsigned int	tailHeapPool;
static unsigned char heapBuffer[VIRTUAL_HEAP_SIZ] __attribute__ ((aligned (4)));
#endif // VIRTUAL_HEAP_FOR_CPLUSPLUS_OPERATOR_NEW

#ifdef VIRTUAL_HEAP_FOR_CPLUSPLUS_OPERATOR_NEW
// NOTE:
// -- The initialization of user heap must be done before using operator
//    "new", the proper entry point is section "bdinit2" which is called
//    by "C:\opt\sparc-elf-3.4.4-mingw\src\libgloss\sparc_leon\crt0.S"
//    and its weak empty declaration can be seen in "bdinit.S" in same
//    location.
void bdinit2()
{
	unsigned int k;

	// link the blocks
	heapLnkList[0].prev = 0; // point to itself
	heapLnkList[0].next = 1;
	for (k = 1; k < VIRTUAL_HEAP_BLK_NUM - 1; k ++) {
		heapLnkList[k].prev = k - 1;
		heapLnkList[k].next = k + 1;
	}
	heapLnkList[VIRTUAL_HEAP_BLK_NUM-1].prev = VIRTUAL_HEAP_BLK_NUM-2;
	heapLnkList[VIRTUAL_HEAP_BLK_NUM-1].next = VIRTUAL_HEAP_EMPTY; // no next one
	// assign the heap buffer to each block
	for (k = 0; k < VIRTUAL_HEAP_BLK_NUM; k ++) {
		heapLnkList[k].addr = (void *) &heapBuffer[k*VIRTUAL_HEAP_BLK_SIZ];
	}
	// set the head/tail of pool
	headHeapPool = 0;
	tailHeapPool = VIRTUAL_HEAP_BLK_NUM - 1;
#if 0
	dumpHeap();
#endif
}
#endif // VIRTUAL_HEAP_FOR_CPLUSPLUS_OPERATOR_NEW

#ifdef VIRTUAL_HEAP_FOR_CPLUSPLUS_OPERATOR_NEW
void * my_malloc(size_t size)
{
	unsigned int	numBlockNeeded;
	unsigned int	currBlock;
	unsigned int	firstBlock;
	unsigned int	lastBlock;
	bool firstBlockIsHeadOfPool;
	bool lastBlockIsTailOfPool;
	unsigned int	numBlockLocated;

	if ((size == 0) || (headHeapPool == VIRTUAL_HEAP_EMPTY)) return ((void *) 0);

	// calculate how many blocks are needed for "size"
	numBlockNeeded = (size / VIRTUAL_HEAP_BLK_SIZ);
	if (size % VIRTUAL_HEAP_BLK_SIZ) { numBlockNeeded += 1; }

	// search the linklist from headHeapPool
	currBlock = firstBlock = lastBlock = headHeapPool;
	firstBlockIsHeadOfPool = lastBlockIsTailOfPool = false;
	numBlockLocated = 0;

	while (1)
	{
		// enough continuous space found in heap
		if (numBlockLocated == numBlockNeeded)
		{
			if (firstBlock == headHeapPool) firstBlockIsHeadOfPool = true;
			if (lastBlock == tailHeapPool) lastBlockIsTailOfPool = true;

			// update headHeapPool
			if (firstBlockIsHeadOfPool == true) {
				headHeapPool = (lastBlockIsTailOfPool == true)
				? VIRTUAL_HEAP_EMPTY : heapLnkList[lastBlock].next;
			}
			// update tailHeapPool
			if (lastBlockIsTailOfPool) tailHeapPool = VIRTUAL_HEAP_EMPTY;

			// update the link list chain of heap pool
			if (firstBlockIsHeadOfPool == false) {
				currBlock = heapLnkList[firstBlock].prev;
				heapLnkList[currBlock].next = (lastBlockIsTailOfPool == true)
				? VIRTUAL_HEAP_EMPTY : heapLnkList[lastBlock].next;
			}
			if (lastBlockIsTailOfPool == false) {
				currBlock = heapLnkList[lastBlock].next;
				heapLnkList[currBlock].prev = (firstBlockIsHeadOfPool == true)
				? currBlock : heapLnkList[firstBlock].prev;
			}

			// seperate the link list found off the pool
			heapLnkList[firstBlock].prev = firstBlock;
			heapLnkList[lastBlock].next = VIRTUAL_HEAP_EMPTY;
			// return the address
		#if 0
			dumpHeap();
		#endif
			return heapLnkList[firstBlock].addr;
		}
		// go over remaining blocks in pool
		else {
			// in case of adjacent blocks are continuous
			if (heapLnkList[currBlock].next == (currBlock+1)) {
				lastBlock = currBlock;
				numBlockLocated ++;
				currBlock = heapLnkList[currBlock].next;
			}
			// in case of reaching end of link-list
			else if (heapLnkList[currBlock].next == VIRTUAL_HEAP_EMPTY) {
				// no chance to allocate enough space, return NULL
				if (numBlockLocated < (numBlockNeeded - 1)) {
				#if 0
					dumpHeap();
				#endif
					return ((void *) 0);
				}
				// the tail of link-list is the last block needed
				else {
					lastBlock = currBlock;
					numBlockLocated ++;
				}
			}
			// in case of adjacent blocks are not continuous
			else {
				numBlockLocated = 0;
				currBlock = heapLnkList[currBlock].next;
				lastBlock = firstBlock = currBlock;
			}
		} // end of else for "go over remaining blocks"
	} // end of while
}

void my_free(void * ptr)
{
	unsigned int	currBlock;
	unsigned int	firstBlock;	// the first memory block to be frozen
	unsigned int	lastBlock;	// the last memory block to be frozen
	unsigned int	holeTopBlock;
	unsigned int	holeBottomBlock;

	if (ptr == 0) return;

	// find out the block ID of link list associated with "ptr"
	firstBlock = (((unsigned char *) ptr) - ((unsigned char *) &heapBuffer[0])) / VIRTUAL_HEAP_BLK_SIZ;
	currBlock = firstBlock;
	while (heapLnkList[currBlock].next != VIRTUAL_HEAP_EMPTY) {
		// currBlock ++;
		currBlock = heapLnkList[currBlock].next;
	}
	lastBlock = currBlock;

	// find out where to insert into heap pool

	// in case of pool is already empty
	if (headHeapPool == VIRTUAL_HEAP_EMPTY) {
		headHeapPool = firstBlock;
		tailHeapPool = lastBlock;
	}
	// blocks to be returned are inserted in front of head of pool
	else if (headHeapPool > lastBlock) {
		heapLnkList[headHeapPool].prev = lastBlock;
		heapLnkList[lastBlock].next = headHeapPool;
		headHeapPool = firstBlock;
	}
	// blocks to be returned are inserted behind tail of pool
	else if (tailHeapPool < firstBlock) {
		heapLnkList[tailHeapPool].next = firstBlock;
		heapLnkList[firstBlock].prev = tailHeapPool;
		tailHeapPool = lastBlock;
	}
	// blocks to be returned are inserted into middle of pool
	else {
		currBlock = headHeapPool;
		while (1) {
			// the hole in pool matches
			if ((currBlock < firstBlock) && (heapLnkList[currBlock].next > lastBlock)) {
				holeTopBlock = currBlock;
				holeBottomBlock = heapLnkList[currBlock].next;
				heapLnkList[holeTopBlock].next = firstBlock;
				heapLnkList[firstBlock].prev = holeTopBlock;
				heapLnkList[lastBlock].next = holeBottomBlock;
				heapLnkList[holeBottomBlock].prev = lastBlock;
				break;
			}
			// go over remaining blocks
			else {
				currBlock = heapLnkList[currBlock].next;
				if (currBlock == tailHeapPool) {
					// should not happen ... warning message ?
					break;
				}
			}
		} // end of while
	}
#if 0
	dumpHeap();
#endif
}
#endif // VIRTUAL_HEAP_FOR_CPLUSPLUS_OPERATOR_NEW

#ifdef __cplusplus
}
#endif // __cplusplus

void * operator new(size_t size)
{
#ifdef VIRTUAL_HEAP_FOR_CPLUSPLUS_OPERATOR_NEW
	return my_malloc(size);
#else
  return malloc(size);
#endif // VIRTUAL_HEAP_FOR_CPLUSPLUS_OPERATOR_NEW
}

// operator new[] can be called explicitly as a regular function, but in C++,
// new[] is an operator with a very specific behavior: An expression with the
// new operator on an array type, first calls function operator new (i.e.,
// this function) with the size of its array type specifier as first argument
// (plus any array overhead storage to keep track of the size, if any), and if
// this is successful, it then automatically initializes or constructs every
// object in the array (if needed). Finally, the expression evaluates as a
// pointer to the appropriate type pointing to the first element of the array.
//
// example:
//
// int** array = new int*[4];
// array[0] = new int[10];
// array[1] = new int[20];
// array[2] = new int[30];
// array[3] = new int[40];

void * operator new[](size_t size)
{
#ifdef VIRTUAL_HEAP_FOR_CPLUSPLUS_OPERATOR_NEW
	return my_malloc(size);
#else
  return malloc(size);
#endif // VIRTUAL_HEAP_FOR_CPLUSPLUS_OPERATOR_NEW
}

void operator delete(void * ptr)
{
#ifdef VIRTUAL_HEAP_FOR_CPLUSPLUS_OPERATOR_NEW
	my_free(ptr);
#else
  free(ptr);
#endif // VIRTUAL_HEAP_FOR_CPLUSPLUS_OPERATOR_NEW
}

void operator delete[](void * ptr)
{
#ifdef VIRTUAL_HEAP_FOR_CPLUSPLUS_OPERATOR_NEW
	my_free(ptr);
#else
  free(ptr);
#endif // VIRTUAL_HEAP_FOR_CPLUSPLUS_OPERATOR_NEW
}

int		__cxa_guard_acquire(__guard *g)
{
	return !*(char *)(g);
};

void	__cxa_guard_release(__guard *g)
{
	*(char *)g = 1;
};

void	__cxa_guard_abort(__guard *) {};

void	__cxa_pure_virtual(void) {};
