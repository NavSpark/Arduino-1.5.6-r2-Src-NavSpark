/*
  TwoWire.cpp - C++ file of library for 2-wire interface
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
#include "TwoWire.h"
#include "Arduino.h"

// **********************************************************************
// Description: declaration of functions provided in "lib_io.a"
// **********************************************************************
#ifdef __cplusplus
extern "C" {
#endif
void v8_i2c_master_init(uint32_t clk_rate);
void v8_i2c_master_off(void);
void v8_i2c_master_xmit_stop(void);
bool v8_i2c_master_poll(uint8_t cmd);
uint16_t v8_i2c_master_write(uint8_t addr, uint8_t *txd, uint16_t num, bool xmitStop);
uint16_t v8_i2c_master_read(uint8_t addr, uint8_t *rxd, uint16_t num, bool xmitStop);
void v8_i2c_slave_init(uint8_t addr);
uint8_t v8_i2c_slave_read_reg(uint8_t reg_id);
void v8_i2c_slave_write_reg(uint8_t reg_id, uint8_t wdata);
#ifdef __cplusplus
}
#endif

// **********************************************************************
// Description: declaration of functions exported in C naming convention
// **********************************************************************
#ifdef __cplusplus
extern "C" {
#endif
void isrTwoWireSlaveFunc(void);
#ifdef __cplusplus
}
#endif

// **********************************************************************
// Description: Instances of I2C master/slave
// **********************************************************************
TwoWire twMaster = TwoWire(TWOWIRE_MASTER);
TwoWire twSlave  = TwoWire(TWOWIRE_SLAVE);

// **********************************************************************
// Description: Wrapper of I2C slave ISR
// **********************************************************************
void isrTwoWireSlaveFunc(void)
{
	if (twSlave) twSlave.isr();
}

// **********************************************************************
// Description: Constructor for class "TwoWire"
// **********************************************************************
TwoWire::TwoWire(void)
{
	TwoWire(TWOWIRE_MASTER);
}

TwoWire::TwoWire(uint8_t mode)
{
	_mode = mode;
	_devAddr = 0x3c;
	_txAddr = 0x00;
	_rxAddr = 0x00;
	_clkRate = 400000;
	slvModeTxFilled = 0;
	// allocate the memory for buffers
	recv.buffer = (uint8_t *) malloc(TWOWIRE_BUFFER_SIZE * sizeof(uint8_t));
	xmit.buffer = (uint8_t *) malloc(TWOWIRE_BUFFER_SIZE * sizeof(uint8_t));
	// set pointers of buffers
	pRecv = (p2WireBuffer) &recv;
	pXmit = (p2WireBuffer) &xmit;
	// set buffer sizes
	recvBuffSize = TWOWIRE_BUFFER_SIZE;
	xmitBuffSize = TWOWIRE_BUFFER_SIZE;
	// set the flag off
	enabled = false;
	// clear callback pointer
	user_onReceive = 0;
	user_onRequest = 0;
}

// **********************************************************************
// Description: Set the clock rate of 2-wire master
// **********************************************************************
void TwoWire::config(uint32_t clkRate)
{
	if (_mode == TWOWIRE_MASTER) {
		_clkRate = clkRate;
	}
}

uint32_t TwoWire::getClkRate(void)
{
	return _clkRate;
}

// **********************************************************************
// Description: Reset the buffer settings
// **********************************************************************
bool TwoWire::setBufferSize(uint16_t xmitSize, uint16_t recvSize)
{
	uint8_t *newXmitPtr;
	uint8_t *newRecvPtr;

	newXmitPtr = (uint8_t *) malloc(xmitSize * sizeof(uint8_t));
	newRecvPtr = (uint8_t *) malloc(recvSize * sizeof(uint8_t));
	if (newXmitPtr && newRecvPtr) {
		free(xmit.buffer);
		free(recv.buffer);
		xmit.buffer = newXmitPtr;
		recv.buffer = newRecvPtr;
		this->xmitBuffSize = xmitSize;
		this->recvBuffSize = recvSize;
		return true;
	}
	else {
		if (newXmitPtr) free(newXmitPtr);
		if (newRecvPtr) free(newRecvPtr);
		return false;
	}
}

void TwoWire::reset(void)
{
	this->resetTx();
	this->resetRx();
}

void TwoWire::resetTx(void)
{
	pXmit->wrPtr = 0;
	pXmit->numSpace = xmitBuffSize; // TWOWIRE_BUFFER_SIZE;
	pXmit->rdPtr = 0;
	pXmit->numData = 0;
}

void TwoWire::resetRx(void)
{
	pRecv->wrPtr = 0;
	pRecv->numSpace = recvBuffSize; // TWOWIRE_BUFFER_SIZE;
	pRecv->rdPtr = 0;
	pRecv->numData = 0;
}

// **********************************************************************
// Description: Set the clock rate of 2-wire and the buffer parameters
// **********************************************************************
void TwoWire::begin(void)
{
	// configure the GPIO pins for i2c function
	pinMode(GPIO4_SCL, SPECIAL);
	pinMode(GPIO5_SDA, SPECIAL);

	// init the 2-wire master with specified clock rate
	if (_mode == TWOWIRE_MASTER) {
		v8_i2c_master_init(_clkRate);
	}
	// reset buffer pointer
	this->reset();

	// set the flag on
	enabled = true;
}

void TwoWire::begin(uint8_t addr)
{
	if (_mode == TWOWIRE_SLAVE) {
		// set the device address used in slave mode
		if (addr <= 127) _devAddr = addr;
		v8_i2c_slave_init(_devAddr);
	}
  begin();
}

// **********************************************************************
// Description: Set the address of target device (master mode only)
// **********************************************************************
void TwoWire::setTransmitDeviceAddr(uint8_t addr)
{
	if (_mode == TWOWIRE_MASTER) {
		if (addr <= 127) _txAddr = addr;
	}
}

void TwoWire::setReceiveDeviceAddr(uint8_t addr)
{
	if (_mode == TWOWIRE_MASTER) {
		if (addr <= 127) _rxAddr = addr;
	}
}

// **********************************************************************
// Description: Write data from master to target device
// **********************************************************************
uint16_t TwoWire::endTransmission(bool xmitStop)
{
	uint16_t byteXmit = 0;

	if (!enabled) return 0;

	if (_mode == TWOWIRE_MASTER) {
		byteXmit = v8_i2c_master_write(_txAddr, &pXmit->buffer[pXmit->rdPtr], pXmit->numData, xmitStop);
		pXmit->numData -= byteXmit;
		pXmit->rdPtr += byteXmit;
	}

	return byteXmit;
}

uint16_t TwoWire::endTransmission(void)
{
	return endTransmission(true);
}

// **********************************************************************
// Description: Debug functions for querying private info.
// **********************************************************************



// **********************************************************************
// Description: Read data from target device to master
// **********************************************************************
uint16_t TwoWire::readDeviceFromOffset(uint8_t devOffset, uint16_t quantity, bool xmitStop)
{
	uint8_t	wdata = devOffset;
	uint16_t byteToRead;
	uint16_t byteRecv = 0;

  if (!enabled) return 0;

	if (_mode == TWOWIRE_MASTER) {
		byteToRead = (quantity > pRecv->numSpace) ? pRecv->numSpace : quantity;
		// {S + _rxAddr + W} + ACK + {devOffset} + ACK
		v8_i2c_master_write(_rxAddr, &wdata, 1, false);
		// {DATA} + ACK + {DATA} + ACK + {DATA} + ACK + {DATA} + NACK + P
		byteRecv = v8_i2c_master_read(_rxAddr, &pRecv->buffer[pRecv->wrPtr], byteToRead, xmitStop);
		pRecv->wrPtr += byteRecv;
		pRecv->numSpace -= byteRecv;
		pRecv->numData += byteRecv;
	}
	return byteRecv;
}

uint16_t TwoWire::readDevice(uint16_t quantity, bool xmitStop)
{
	uint16_t byteToRead;
	uint16_t byteRecv = 0;

	if (!enabled) return 0;

	if (_mode == TWOWIRE_MASTER) {
		byteToRead = (quantity > pRecv->numSpace) ? pRecv->numSpace : quantity;
		byteRecv = v8_i2c_master_read(_rxAddr, &pRecv->buffer[pRecv->wrPtr], byteToRead, xmitStop);
		pRecv->wrPtr += byteRecv;
		pRecv->numSpace -= byteRecv;
		pRecv->numData += byteRecv;
	}
	return byteRecv;
}

uint16_t TwoWire::readDevice(uint16_t quantity)
{
	return readDevice(quantity, true);
}

// **********************************************************************
// Description: Throw data to the Tx buffer and start transfer
// **********************************************************************
size_t TwoWire::write(uint8_t data)
{
	if (pXmit->numSpace == 0) return 0;

	pXmit->buffer[pXmit->wrPtr++] = data;
	pXmit->numSpace --;
	pXmit->numData ++;

	if (_mode == TWOWIRE_MASTER) { // master mode
		return endTransmission();
	}
	else { // slave mode
		return 1;
	}
}

size_t TwoWire::write(uint8_t *data, size_t size)
{
	uint32_t	i, k;

	if (pXmit->numSpace == 0) return 0;

	k = (pXmit->numSpace >= size) ? size : pXmit->numSpace;
	for (i = 0; i < k; i++) {
		pXmit->buffer[pXmit->wrPtr++] = data[i];
	}
	pXmit->numSpace -= k;
	pXmit->numData += k;

	if (_mode == TWOWIRE_MASTER) { // master mode
		return endTransmission();
	}
	else { // slave mode
		return k;
	}
}

size_t TwoWire::writeAtOffset(uint8_t devOffset, uint8_t *data, size_t size)
{
	uint32_t i;
	uint16_t k;

	if (_mode == TWOWIRE_MASTER) { // master mode
		if (pXmit->numSpace < (size + 1)) return 0;
		pXmit->buffer[pXmit->wrPtr++] = devOffset;
		for (i = 0; i < size; i++) {
			pXmit->buffer[pXmit->wrPtr++] = data[i];
		}
		pXmit->numSpace -= (size + 1);
		pXmit->numData += (size + 1);
		k = endTransmission();
		if (k > 1) return (k-1);
		else return 0;
	}
	else { // slave mode
		return 0;
	}
}

// **********************************************************************
// Description: Return how many bytes of ingress data received in buffer
// **********************************************************************
size_t TwoWire::available(void)
{
	return ((size_t)pRecv->numData);
}

// **********************************************************************
// Description: Return how many bytes of egress data still in Tx buffer
// **********************************************************************
size_t TwoWire::remaining(void)
{
	return ((size_t)pXmit->numData);
}

// **********************************************************************
// Description: Read out the data received in buffer
// **********************************************************************
int TwoWire::read(void)
{
	int value = -1;
	uint8_t	data;

	if (pRecv->numData) {
		data = pRecv->buffer[pRecv->rdPtr];
		pRecv->rdPtr = (pRecv->rdPtr + 1) % recvBuffSize; // TWOWIRE_BUFFER_SIZE;
		pRecv->numData --;
		pRecv->numSpace ++;
		value = (int)data;
	}
	return value;
}

size_t TwoWire::blk_read(uint8_t *dst, size_t size)
{
  unsigned int k;
  unsigned int num;

  num = (pRecv->numData >= size) ? size : pRecv->numData;
  for (k = 0; k < num; k++) {
  	dst[k] = (uint8_t) read();
  }
  return ((size_t) num);
}

// **********************************************************************
// Description: Pick the data next to current data for peeking
// **********************************************************************
int TwoWire::peek(void)
{
	int value = -1;
	uint8_t	data;

	if (pRecv->numData >= 2) {
//	data = pRecv->buffer[(pRecv->rdPtr+1) % TWOWIRE_BUFFER_SIZE];
		data = pRecv->buffer[(pRecv->rdPtr+1) % recvBuffSize];
		value = (int)data;
	}

	return value;
}

// **********************************************************************
// Description:
// **********************************************************************
void TwoWire::flush(void)
{

}

// **********************************************************************
// Description: Set callback function for data input from master
// **********************************************************************
#if 1
void TwoWire::onReceive( void (*function)(void) )
{
	if (_mode == TWOWIRE_SLAVE) {
		user_onReceive = function;
	}
}
#endif

// **********************************************************************
// Description: Set callback function for data output to master
// **********************************************************************
#if 1
void TwoWire::onRequest( void (*function)(void) )
{
	if (_mode == TWOWIRE_SLAVE) {
  	user_onRequest = function;
	}
}
#endif

// **********************************************************************
// Description: Task routine for handling data transfer in slave mode
// **********************************************************************
//
//						+-----------------+
//						|                 |
//  bit[7:5]  |  s2m_data_cnt   | >> how many bytes were filled by slave
//						|                 |
//						+-----------------+
//						|                 |
//  bit[4:2]  |  m2s_data_cnt   | >> how many bytes were filled by master
//						|                 |
//						+-----------------+
//  bit[1]    |  s2m_fill_done  | >> slave  writes '1' to notify master to read
//						+-----------------+    master writes '0' to notify slave  can write.
//	bit[0]    |  m2s_fill_done  | >> master writes '1' to notify slave  to read.
//						+-----------------+    slave  writes '0' to notify master can write.
//
void TwoWire::isr(void)
{
	uint8_t	i;
	uint8_t tx_num = 0;
	uint8_t rx_num = 0;
	uint8_t regData;
	uint8_t regCtrl;

	if (_mode == TWOWIRE_MASTER) { return; } // master mode
	if (!enabled) { return; } // object not active

	if (user_onRequest) { user_onRequest(); }

	// check s2m_fill_done
	regData = v8_i2c_slave_read_reg(8);
	// Tx is allowed
	if ((regData & 0x02) == 0) {
		// in case of previous Tx was done
		if (slvModeTxFilled) {
			regCtrl = regData & ~((0x1UL<<1) | (0x7UL<<5));
			v8_i2c_slave_write_reg(8, regCtrl); // clear the INTR
			slvModeTxFilled = 0;
		}
		// in case of Tx data is valid
		if (pXmit->numData) {
			tx_num = (pXmit->numData > TWOWIRE_SLAVE_TX_FIFO_SIZE)
			? TWOWIRE_SLAVE_TX_FIFO_SIZE : pXmit->numData;
			for (i = 0; i < tx_num; i++) {
				v8_i2c_slave_write_reg(TWOWIRE_SLAVE_TX_FIFO_BASE+i, pXmit->buffer[pXmit->rdPtr+i]);
			}
			pXmit->numData -= tx_num;
			pXmit->rdPtr += tx_num;
			slvModeTxFilled = 1;
		}
	}

	// Rx was done
	if (regData & 0x01) {
		rx_num = (regData>>2) & 0x7;
		for (i = 0; i < rx_num; i++) {
			if (pRecv->numSpace) {
				pRecv->buffer[pRecv->wrPtr] = v8_i2c_slave_read_reg(TWOWIRE_SLAVE_RX_FIFO_BASE+i);
				pRecv->wrPtr = (pRecv->wrPtr + 1) % recvBuffSize; // TWOWIRE_BUFFER_SIZE;
				pRecv->numData ++;
				pRecv->numSpace --;
			}
			else v8_i2c_slave_read_reg(TWOWIRE_SLAVE_RX_FIFO_BASE+i);
		}
	}

	// clear "m2s_data_cnt" and "m2s_fill_done"
	regCtrl = regData & ~((0x1UL<<0) | (0x7UL<<2));
	// set "s2m_data_cnt" and "s2m_fill_done"
	if (tx_num) {
		regCtrl |= ((tx_num<<5) | 0x02);
	}
	v8_i2c_slave_write_reg(8, regCtrl);

	if (user_onReceive) { user_onReceive(); }
}

// **********************************************************************
// Description:
// **********************************************************************
TwoWire::operator bool() {
	return enabled;
}
