/*
  SPI.cpp - C++ file of library for SPI interface
  Copyright (c) 2014 NavSpark.  All right reserved.

	This library is free software; you can redistribute it under the terms
  of the GNU Lesser General Public License as published by the Free Software
  Foundation; either version 2.1 of the License, or (at your option) any
  later version.

  This library is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
  or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
  License for more details.

	Created 24 Feb. 2014 by Ming-Jen Chen

	$Id$
*/

#include "stdint.h"
#include "Arduino.h"
#include "SPI.h"

// **********************************************************************
// Description: declaration of functions provided in "lib_io.a"
// **********************************************************************
#ifdef __cplusplus
extern "C" {
#endif
void v8_spi_master_init(uint32_t clk_rate, bool en_cs1, bool en_cs2);
void v8_spi_master_config(uint8_t mode, bool intr_en);
uint16_t v8_spi_master_byte_io(uint8_t slv, uint8_t *txd, uint8_t *rxd, uint16_t byteNum);
void v8_spi_slave_init(void);
uint8_t v8_spi_slave_write_s2m_buffer(uint8_t *txd, uint8_t num);
uint8_t v8_spi_slave_read_m2s_buffer(uint8_t *rxd);
void v8_spi_slave_enable_m2s_buffer(void);
void v8_spi_slave_enable_s2m_buffer(void);
void v8_spi_slave_disable_s2m_buffer(void);
#ifdef __cplusplus
}
#endif

// **********************************************************************
// Description: declaration of functions exported in C naming convention
// **********************************************************************
#ifdef __cplusplus
extern "C" {
#endif
void isrSPISlaveFunc(uint8_t type);
#ifdef __cplusplus
}
#endif

// **********************************************************************
// Description: declaration of callback functions
// **********************************************************************


// **********************************************************************
// Description: Instances of SPI master/slave
// **********************************************************************
SPI spiMaster	= SPI(SPI_MASTER);
SPI spiSlave	= SPI(SPI_SLAVE);

// **********************************************************************
// Description: Wrapper of I2C slave ISR
// **********************************************************************
void isrSPISlaveFunc(uint8_t type)
{
	if (spiSlave) spiSlave.isr(type);
}

// **********************************************************************
// Description: Constructor for class "SPI"
// **********************************************************************
SPI::SPI(void)
{
	SPI(SPI_MASTER);
};

SPI::SPI(uint8_t type)
{
	_spiType = type;
	_spiMode = 0;
	_clkRate = 1000000; // default 1MHz
	_en_slave_cs1 = false;
	_en_slave_cs2 = false;
	_slave_cs = 0;
	enabled = false;
};

// **********************************************************************
// Description: Set the clock rate and mode
// **********************************************************************
void SPI::config(uint8_t spiMode, uint32_t clkRate, bool en_cs1, bool en_cs2)
{
	if (_spiType == SPI_MASTER) {
		_clkRate = clkRate;
		_en_slave_cs1 = en_cs1;
		_en_slave_cs2 = en_cs2;
		_slave_cs = 0;
		if (spiMode < 2) _spiMode = spiMode;
	}
}

// **********************************************************************
// Description: Init the SPI object
// **********************************************************************
void SPI::begin(void)
{
	// configure the GPIO pins to proper modes
	pinMode(GPIO29_SPIMS_SCK, SPECIAL);
	pinMode(GPIO30_SPIMS_MOSI, SPECIAL);
	pinMode(GPIO31_SPIMS_MISO, SPECIAL);
	if (_spiType == SPI_MASTER) {
		                           pinMode(GPIO28_SPIMS_CSN, SPECIAL); // always ON
		if (_en_slave_cs1 == true) pinMode(GPIO22_SPISL_MCS1, SPECIAL);
		if (_en_slave_cs2 == true) pinMode(GPIO6_WHEEL_TIC_MEAS, SPECIAL);
	}
	else if (_spiType == SPI_SLAVE) {
		gpioConf[GPIO28_SPIMS_CSN].io_mode = SPECIAL;
	}

	// init the SPI master with specified clock rate
	if (_spiType == SPI_MASTER) {
		v8_spi_master_init(_clkRate, _en_slave_cs1, _en_slave_cs2);
		v8_spi_master_config(_spiMode, false); // no INTR now
		s2m_buffer_can_fill = false;
	}
	else if (_spiType == SPI_SLAVE) {
		v8_spi_slave_init();
		s2m_buffer_can_fill = true;
	}

	resetTx();
	resetRx();
	userFuncForSlaveAfterHostRead = NULL;
	userFuncForSlaveAfterHostWrite = NULL;

	// set the flag on
	enabled = true;
}

// **********************************************************************
// Description:
// **********************************************************************
void SPI::resetTx(void)
{
	txdSize = 0;
	txdPtr = 0;
}

void SPI::resetRx(void)
{
	rxdSize = 0;
	rxdPtr = 0;
}

// **********************************************************************
// Description: Throw data to the Tx buffer
// **********************************************************************
size_t SPI::write(uint8_t data)
{
	if (txdSize < SPI_BUFFER_SIZE) {
		txd[txdSize++] = data;
		return 1;
	}
	else return 0;
}

size_t SPI::write(uint8_t *data, size_t size)
{
	uint16_t sz, k;

	sz = SPI_BUFFER_SIZE - txdSize; /* free space */
	for (k = 0; k < ((size>sz) ? sz : size); k++)
	{
		txd[txdSize++] = data[k];
	}
	return k;
}

// **********************************************************************
// Description: function to select which CS pin for remote slave
// **********************************************************************
void SPI::slaveSelect(uint8_t slv){
	// NavSpark supports 3 CSNs
	if (_spiType == SPI_MASTER) {
		switch (slv) {
			case 0: _slave_cs = 0; break;
			case 1: if (_en_slave_cs1) _slave_cs = 1; break;
			case 2: if (_en_slave_cs2) _slave_cs = 2; break;
			default: _slave_cs = 0;
		}
	}
}

// **********************************************************************
// Description: begin SPI data transfer
// **********************************************************************
size_t SPI::transfer(void)
{
	uint16_t io_num;

	if (_spiType == SPI_MASTER) {
		if (txdSize <= 8) {
			io_num = v8_spi_master_byte_io(_slave_cs, &txd[txdPtr], &rxd[rxdSize], txdSize);
		}
		else {
			io_num = v8_spi_master_byte_io(_slave_cs, &txd[txdPtr], &rxd[rxdSize], 8);
		}
		txdPtr += io_num;
		txdSize -= io_num;
		rxdSize += io_num;
		return io_num;
	}
	else return 0;
}

size_t SPI::transfer(size_t size)
{
	txdSize = size;
	return transfer();
}

// **********************************************************************
// Description: Return how many bytes are received in Rx buffer
// **********************************************************************
int SPI::available(void)
{
	return ((int)rxdSize);
}

// **********************************************************************
// Description: Return how many bytes are still in Tx buffer
// **********************************************************************
int SPI::remaining(void)
{
	return ((int)txdSize);
}

// **********************************************************************
// Description: Read out the data received in buffer
// **********************************************************************
int SPI::read(void)
{
	int value = -1;

	// in case of no more valid data
	if (rxdPtr == SPI_BUFFER_SIZE) return value;

	if (rxdSize) {
		value = (int)rxd[rxdPtr++];
		rxdSize --;
	}
	return value;
}

// **********************************************************************
// Description:
// **********************************************************************
uint8_t SPI::pick(size_t offset)
{
	return rxd[offset];
}

// **********************************************************************
// Description:
// **********************************************************************
void SPI::enableBufferForHostWrite(void)
{
	if (_spiType == SPI_MASTER) return;
	v8_spi_slave_enable_m2s_buffer();
}

// **********************************************************************
// Description:
// **********************************************************************
bool SPI::validBufferForHostRead(void)
{
	return s2m_buffer_can_fill;
}

uint8_t SPI::copyDataToBufferForHostRead(void)
{
	uint8_t s2m_num;

	if (_spiType == SPI_MASTER) return 0;

	if (txdSize && (s2m_buffer_can_fill == true)) {
		s2m_num = v8_spi_slave_write_s2m_buffer(&txd[txdPtr], txdSize);
		txdSize -= s2m_num;
		txdPtr += s2m_num;
		return s2m_num;
	}
	return 0;
}

void SPI::enableBufferForHostRead(void)
{
	if (_spiType == SPI_MASTER) return;
	s2m_buffer_can_fill = false;
	v8_spi_slave_enable_s2m_buffer();
}

void SPI::disableBufferForHostRead()
{
	if (_spiType == SPI_MASTER) return;
	s2m_buffer_can_fill = true;
	v8_spi_slave_disable_s2m_buffer();
}

// **********************************************************************
// Description:
// **********************************************************************
void SPI::attachInterrupt(uint8_t type, void (*userFunc)(void))
{
	if (_spiType == SPI_MASTER) return;

	if (type == IRQ_SPI_SLAVE_HOST_READ_DONE) {
		userFuncForSlaveAfterHostRead = userFunc;
	}
	else if (type == IRQ_SPI_SLAVE_HOST_WRITE_DONE) {
		userFuncForSlaveAfterHostWrite = userFunc;
	}
}

void SPI::detachInterrupt(uint8_t type)
{
	if (_spiType == SPI_MASTER) return;

	if (type == IRQ_SPI_SLAVE_HOST_READ_DONE) {
		userFuncForSlaveAfterHostRead = NULL;
	}
	else if (type == IRQ_SPI_SLAVE_HOST_WRITE_DONE) {
		userFuncForSlaveAfterHostWrite = NULL;
	}
}

// **********************************************************************
// Description:
// **********************************************************************
void SPI::isr(uint8_t type)
{
	if (_spiType == SPI_MASTER) { return; } // master mode
	if (!enabled) { return; } // object not active

	if (type == IRQ_SPI_SLAVE_RESET) { // reset request issued by host
		// add your code here

	}
	if (type == IRQ_SPI_SLAVE_HOST_READ_DONE) { // host read buffer done
		// call user-defined function before processing next read
		if (userFuncForSlaveAfterHostRead) {
			userFuncForSlaveAfterHostRead();
		}
		// allow fill buffer again for next read
		s2m_buffer_can_fill = true;
	}
	if (type == IRQ_SPI_SLAVE_HOST_WRITE_DONE) { // host wrote buffer done
		rxdSize = v8_spi_slave_read_m2s_buffer(rxd);
		rxdPtr = 0;
		// add your code here
		if (userFuncForSlaveAfterHostWrite) {
			userFuncForSlaveAfterHostWrite();
		}
	}
	if (type == IRQ_SPI_SLAVE_CMD_CHECK) { // host wrote command done
		// add your code here

	}
}

// **********************************************************************
// Description:
// **********************************************************************
SPI::operator bool() {
	return enabled;
}
