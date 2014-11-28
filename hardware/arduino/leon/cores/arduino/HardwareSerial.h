/*
  HardwareSerial.h - Header file for hardware serial library
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

	Created 25 December 2013 by Ming-Jen Chen

	$Id$
*/

#ifndef _HARDWARESERIAL_H_
#define _HARDWARESERIAL_H_

#include "Stream.h"

#define	SERIAL_RING_BUFFER_SIZE	256

typedef	struct _serial_ring_buffer
{
  unsigned char *buffer;
  volatile unsigned int head;
  volatile unsigned int tail;
  volatile unsigned int numData;
} sSerialRingBuffer, *pSerialRingBuffer;

#define	NMEA_PORT_ID	0
#define	CONSOLE_PORT_ID	1

class HardwareSerial : public Stream
{
	private:
		uint8_t	port_id;
		uint32_t valueBaudRate;
		uint8_t numWordLength;
		uint8_t	numStopBit;
		uint8_t parityCheck;

		sSerialRingBuffer	recv;
		sSerialRingBuffer	xmit;
		pSerialRingBuffer	pRecv;
		pSerialRingBuffer	pXmit;
		bool	enabled;

	public:
		HardwareSerial(uint8_t port);

		// Sets the data format for communication on serial port, see "sti_gnss_lib.h"
		// for legal settings.
		void config(uint8_t word_length, uint8_t stop_bit, uint8_t parity_check);

		// Sets the data rate in bits per second (baudrate) for serial data I/O.
		// The recommended values are one of 4800, 9600, 19200, 38400, 57600,
		// 115200, 230400, 460800 and 921600.
		void begin(void);
		void begin(uint32_t baudrate);

		// Disables serial communication, allowing the RX and TX pins to be used
		// for general input and output. To re-enable serial communication, call
		// Serial.begin().
    void end(void);

    // Get the number of bytes (characters) available for reading from the
    // serial port. This is data that's already arrived and stored in the
    // serial receive buffer (which holds 64 bytes). available() inherits
    // from the Stream utility class.
		size_t available(void); // pure virtual in class Stream

		// Read the first incoming byte from the serial port, or return -1 in case
		// of no valid data.
		int read(void);	// pure virtual in class Stream

		// Returns the next byte (character) of incoming serial data but without
		// removing it from the internal serial buffer. That is, successive calls
		// to peek() will return the same character, as will as the next call to
		// read().
		int peek(void); // pure virtual in class Stream

		// Waits for the transmission of outgoing serial data to complete.
		void flush(void); // pure virtual in class Stream

		// Writes binary data to the serial port. This data is sent as a byte or
		// series of bytes; to send the characters representing the digits of a
		// number use the print() function instead.
		size_t write(uint8_t value); // pure virtual in class Stream
		inline size_t write(uint8_t *data, size_t size); // virtual in class Print
		size_t print(const char str[]);

		// Handler to move data between ring buffers and H/W FIFOs
		void isrRx(void);
		void taskTx(void);

		//
		operator bool();
};

HardwareSerial*	getHardwareSerial(uint8_t port);

extern HardwareSerial Serial;
extern HardwareSerial Serial1;

#endif	// _HARDWARESERIAL_H_
