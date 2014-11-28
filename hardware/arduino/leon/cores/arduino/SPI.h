/*
  SPI.h - Header file of library for SPI interface
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

#ifndef _SPI_H_
#define _SPI_H_

#include "Stream.h"

#define	SPI_BUFFER_SIZE	16

#define	SPI_MASTER	1
#define	SPI_SLAVE		0

// for V822A_PACKAGE, CS0 is GPIO pin 28, CS1 is GPIO pin 22
#define	SPI_CS_0	0
#define	SPI_CS_1	1

class SPI
{
	private:
		bool	enabled;

		uint8_t	_spiType;	// 1 for master, 0 for slave
		uint8_t _spiMode; // 0 for mode 0, 1 for mode 1
		uint32_t _clkRate; // valid in master mode
		bool _en_slave_cs1; // true to enable extra CS1
		bool _en_slave_cs2; // true to enable extra CS2
		uint8_t	_slave_cs; // 0 -> GPIO28, 1 -> GPIO22 and 2 -> GPIO6

		bool s2m_buffer_can_fill;
		uint8_t txd[SPI_BUFFER_SIZE];
		uint16_t txdSize;
		uint16_t txdPtr;

		uint8_t rxd[SPI_BUFFER_SIZE];
		uint16_t rxdSize;
		uint16_t rxdPtr;

		void (*userFuncForSlaveAfterHostRead)(void);
		void (*userFuncForSlaveAfterHostWrite)(void);

	public:
		SPI(uint8_t type);
		SPI(void);

		// Set the clock rate and mode of SPI interface
		void config(uint8_t spiMode, uint32_t clkRate, bool en_cs1, bool en_cs2);

		// Init all necessary parameters
		void begin(void);

		// Reset the buffer settings
		void resetTx(void);
		void resetRx(void);

		// Data transfer
		size_t write(uint8_t data);
		size_t write(uint8_t *data, size_t size);
		void slaveSelect(uint8_t slv);
		size_t transfer(void);
		size_t transfer(size_t size);

		// Return how many bytes are received in Rx buffer
		virtual int available(void);

		// Return how many bytes are still in Tx buffer
		int remaining(void);

		// Read out the data received in buffer
		virtual int read(void);

		// Data parsing
		uint8_t pick(size_t offset);

		// Functions for buffer written by host
		void enableBufferForHostWrite(void);

		// Functions for buffer for host read
		bool validBufferForHostRead(void);
		uint8_t copyDataToBufferForHostRead(void);
		void enableBufferForHostRead(void);
		void disableBufferForHostRead(void);

		// hook/un-hook user defined ISR for alsve
		void attachInterrupt(uint8_t type, void (*userFunc)(void));
		void detachInterrupt(uint8_t type);

		// Handler to move data between ring buffers and H/W FIFOs
		void isr(uint8_t type);

		//
		operator bool();
};

extern SPI spiMaster;
extern SPI spiSlave;

#endif // _SPI_H_
