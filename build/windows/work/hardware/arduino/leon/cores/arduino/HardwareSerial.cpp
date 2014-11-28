/*
  HardwareSerial.cpp - C++ file for hardware serial library
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

#include "stdint.h"
#include "sti_gnss_lib.h"
#include "HardwareSerial.h"

// **********************************************************************
// Description: declaration of functions exported in C naming convention
// **********************************************************************
#ifdef __cplusplus
extern "C" {
#endif
void isrSerialFunc(uint8_t port);
void taskSerialFunc(uint8_t port);
void v8_uart_intr_init(uint8_t port);
void v8_uart_set_txfifo_empty_intr(uint8_t port, uint8_t on);
#ifdef __cplusplus
}
#endif

// NOTE:
// -- In Venus-8, the first UART identified by NMEA_PORT_ID is reserved for NMEA
//    output so we logically assign it as "Serial1" and the second UART identified
//    by CONSOLE_PORT_ID as "Serial" for convenience.
//
HardwareSerial Serial = HardwareSerial(CONSOLE_PORT_ID);
HardwareSerial Serial1 = HardwareSerial(NMEA_PORT_ID);

// **********************************************************************
// Description: Wrapper of UART Tasks
// **********************************************************************
void isrSerialFunc(uint8_t port)
{
	if (port == CONSOLE_PORT_ID) {
		if (Serial) { Serial.isrRx(); }
	}
	if (port == NMEA_PORT_ID) {
		if (Serial1) { Serial1.isrRx(); }
	}
}

void taskSerialFunc(uint8_t port)
{
	if (port == CONSOLE_PORT_ID) {
		if (Serial) { Serial.taskTx(); }
	}
	if (port == NMEA_PORT_ID) {
		if (Serial1) { Serial1.taskTx(); }
	}
}

// **********************************************************************
// Description: Constructor for class "HardwareSerial"
// **********************************************************************
HardwareSerial::HardwareSerial(uint8_t port)
{
	// port identification
	if (port < 2) { port_id = port; }
	// allocate the memory for ring buffers
	recv.buffer = new unsigned char[SERIAL_RING_BUFFER_SIZE];
	xmit.buffer = new unsigned char[SERIAL_RING_BUFFER_SIZE];
	// set pointers of ring buffers
	pRecv = (pSerialRingBuffer) &recv;
	pXmit = (pSerialRingBuffer) &xmit;
	// give the default values
	valueBaudRate = BAUDRATE;
	numWordLength = 8;
	numStopBit = 1;
	parityCheck = 0;
	// set the flag off
	enabled = false;
}

// **********************************************************************
// Description: Destructor for class "HardwareSerial"
// **********************************************************************
//HardwareSerial::~HardwareSerial(void)
//{
//	// release the memory for ring buffers
//	delete(recv.buffer);
//	delete(xmit.buffer);
//}

// **********************************************************************
// Description: Set the data format of serial port, ex. 8-N-1
// **********************************************************************
void HardwareSerial::config(uint8_t word_length, uint8_t stop_bit, uint8_t parity_check)
{
	if ( word_length == STGNSS_UART_5BITS_WORD_LENGTH
		|| word_length == STGNSS_UART_6BITS_WORD_LENGTH
		|| word_length == STGNSS_UART_7BITS_WORD_LENGTH
		|| word_length == STGNSS_UART_8BITS_WORD_LENGTH) {
		numWordLength = word_length;
	}

	if (stop_bit == STGNSS_UART_1STOP_BITS) numStopBit = 1;
	else if (stop_bit == STGNSS_UART_2STOP_BITS) numStopBit = 2;

	if ( parity_check == STGNSS_UART_NOPARITY
		|| parity_check == STGNSS_UART_ODDPARITY
		|| parity_check == STGNSS_UART_EVENPARITY) {
		parityCheck = parity_check;
	}
}

// **********************************************************************
// Description: Set the baudrate of serial port and turn it on
// **********************************************************************
void HardwareSerial::begin(void)
{
	begin(valueBaudRate);
}

void HardwareSerial::begin(uint32_t baudrate)
{
	// reset the pointers
	xmit.head = 0;
	xmit.tail = 0;
	xmit.numData = 0;
	recv.head = 0;
	recv.tail = 0;
	recv.numData = 0;
	// set the baudrate
	if ((baudrate >= 4800) && (baudrate <= 460800)) {
		valueBaudRate = baudrate;
	}
	// set the serial port
	gnss_uart_init(port_id, valueBaudRate, numWordLength, numStopBit, parityCheck);
	// set the interrupt for uart
	v8_uart_intr_init(port_id);
	// set the flag on
	enabled = true;
}

// **********************************************************************
// Description: Turn the serial port off
// **********************************************************************
void HardwareSerial::end(void)
{
	// set the flag off
	enabled = false;
}

// **********************************************************************
// Description: Report how many bytes are received in recv ring buffer
// **********************************************************************
size_t HardwareSerial::available(void)
{
	if (enabled == true) {
		return ((size_t)recv.numData);
	}
	else return 0;
}

// **********************************************************************
// Description: get 1-byte data from receive buffer
// **********************************************************************
int HardwareSerial::read(void)
{
	unsigned char	rxd;

	if (enabled == true) {
		if (recv.numData) {
			// disable interrupt for integrity
			gnss_uart_critical_section_enable(port_id);
			// get 1-byte data from buffer
			rxd = recv.buffer[recv.tail];
			recv.tail = (recv.tail + 1) % SERIAL_RING_BUFFER_SIZE;
			recv.numData--;
			// enable interrupt
			gnss_uart_critical_section_disable(port_id);
			return rxd;
		}
		else return -1;
	}
	else return -1;
}

// **********************************************************************
// Description: pick the data next to current data for peeking
// **********************************************************************
int HardwareSerial::peek(void)
{
	if (enabled == true) {
		if (recv.numData >= 2) {
			return recv.buffer[(recv.tail + 1) % SERIAL_RING_BUFFER_SIZE];
		}
		else return -1;
	}
	else return -1;
}

// **********************************************************************
// Description: waits for xmit buffer is empty
// **********************************************************************
void HardwareSerial::flush(void)
{
#if defined(USE_UART1_FOR_NMEA) && (USE_UART1_FOR_NMEA==1)
	if (port_id == NMEA_PORT_ID) return;
#endif
	if (enabled == true) {
		// disable interrupt for integrity
		gnss_uart_critical_section_enable(port_id);
		// flush all Tx data in ring buffer to H/W FIFO
		while (xmit.numData) {
			if (gnss_uart_tx_status(port_id)==true) {
				gnss_uart_tx_send(port_id, xmit.buffer[xmit.tail]);
				xmit.tail = (xmit.tail + 1) % SERIAL_RING_BUFFER_SIZE;
				xmit.numData--;
			}
		}
		// enable interrupt
		gnss_uart_critical_section_disable(port_id);
	}
	else return;
}

// **********************************************************************
// Description: put 1-byte data into ring buffer
// **********************************************************************
size_t HardwareSerial::write(uint8_t value)
{
#if defined(USE_UART1_FOR_NMEA) && (USE_UART1_FOR_NMEA==1)
	if (port_id == NMEA_PORT_ID) return 0;
#endif
	if (enabled == true) {
		if (xmit.numData < SERIAL_RING_BUFFER_SIZE) {
			// disable interrupt for integrity
			gnss_uart_critical_section_enable(port_id);
			// put 1-byte data into ring buffer
			xmit.buffer[xmit.head] = value;
			xmit.head = (xmit.head + 1) % SERIAL_RING_BUFFER_SIZE;
			xmit.numData++;
			// enable interrupt
			gnss_uart_critical_section_disable(port_id);
			// turn on interrupt for Tx FIFO empty
			v8_uart_set_txfifo_empty_intr(port_id, 1);
			return 1;
		}
		else return 0;
	}
	return 0;
}

// **********************************************************************
// Description: put data in buffer into ring buffer
// **********************************************************************
size_t HardwareSerial::write(uint8_t *data, size_t size)
{
	size_t numWritten = 0;

#if defined(USE_UART1_FOR_NMEA) && (USE_UART1_FOR_NMEA==1)
	if (port_id == NMEA_PORT_ID) return 0;
#endif
	if (enabled == true) {
		// disable interrupt for integrity
		gnss_uart_critical_section_enable(port_id);
		while ((xmit.numData < SERIAL_RING_BUFFER_SIZE) && (numWritten < size)) {
			xmit.buffer[xmit.head] = data[numWritten];
			xmit.head = (xmit.head + 1) % SERIAL_RING_BUFFER_SIZE;
			xmit.numData++;
			numWritten++;
		}
		// enable interrupt
		gnss_uart_critical_section_disable(port_id);
		// turn on interrupt for Tx FIFO empty
		if (numWritten) {
			v8_uart_set_txfifo_empty_intr(port_id, 1);
		}
		// report how many bytes were written
		return numWritten;
	}
	else return 0;
}

size_t HardwareSerial::print(const char str[])
{
	size_t strSize = 0;
	size_t k;

	// check the length of "str"
	for (k = 0; k < SERIAL_RING_BUFFER_SIZE; k++)
	{
		if (str[k] == '\0') {
			strSize = k;
			break;
		}
	}

	// waits for enough free buffer space
	if (strSize) {
		while ((SERIAL_RING_BUFFER_SIZE - xmit.numData) < strSize) {
			for (k = 0; k < 100; k++) { asm("nop"); }
		}
	}

	return write((uint8_t *)str, strSize);
}

// **********************************************************************
// Description:
// **********************************************************************
void HardwareSerial::isrRx(void)
{
	static uint8_t i, k;
#if defined(USE_UART1_FOR_NMEA) && (USE_UART1_FOR_NMEA==1)
	if (port_id == NMEA_PORT_ID) return;
#endif

	// disable interrupt for integrity
	gnss_uart_critical_section_enable(port_id);

	// RX: move data received in H/W FIFO to ring buffer
	for (i = 0; i < gnss_uart_rx_status(port_id); i++) {
		if (recv.numData < SERIAL_RING_BUFFER_SIZE) {
			recv.buffer[recv.head] = gnss_uart_rx_receive(port_id);
			recv.head = (recv.head + 1) % SERIAL_RING_BUFFER_SIZE;
			recv.numData++;
		}
		else {
			// just discard data in FIFO
			k = gnss_uart_rx_receive(port_id);
		}
	}

	// enable interrupt
	gnss_uart_critical_section_disable(port_id);
}

void HardwareSerial::taskTx(void)
{
#if defined(USE_UART1_FOR_NMEA) && (USE_UART1_FOR_NMEA==1)
	if (port_id == NMEA_PORT_ID) return;
#endif

	// disable interrupt for integrity
	gnss_uart_critical_section_enable(port_id);

	// TX: move data in ring buffer to H/W FIFO
	while ((gnss_uart_tx_status(port_id)==true)
			&& (xmit.numData > 0) ) {
		gnss_uart_tx_send(port_id, xmit.buffer[xmit.tail]);
		xmit.tail = (xmit.tail + 1) % SERIAL_RING_BUFFER_SIZE;
		xmit.numData--;
	}
	// turn off interrupt for Tx FIFO empty
	if (xmit.numData == 0) {
		v8_uart_set_txfifo_empty_intr(port_id, 0);
	}

	// enable interrupt
	gnss_uart_critical_section_disable(port_id);
}

// **********************************************************************
// Description:
// **********************************************************************
HardwareSerial::operator bool() {
	return enabled;
}

// **********************************************************************
// Description: create the object for HardwareSerial
// **********************************************************************
HardwareSerial*	getHardwareSerial(uint8_t port) {
	if (port == NMEA_PORT_ID) return &Serial1;
	else if (port == CONSOLE_PORT_ID) return &Serial;
	else return ((HardwareSerial *) 0);
}
