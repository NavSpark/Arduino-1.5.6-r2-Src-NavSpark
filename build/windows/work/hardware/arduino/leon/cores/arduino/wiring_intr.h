/*
  wiring_intr.h - header file for Arduino user-defined interrupt functions
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

#ifndef _WIRING_INTERRUPT_
#define _WIRING_INTERRUPT_

#ifndef _STDINT_H_
#include "stdint.h"
#endif

#ifndef _WIRING_DIGITAL_
#include "wiring_digital.h"
#endif

extern sGPIOConf gpioConf[33];

#ifdef __cplusplus
extern "C" {
#endif

#define	TIMER_ISR_NUMBER	5
#define	GPIO_ISR_NUMBER		6
#define	UART_ISR_NUMBER		7
#define	GPS_ISR_NUMBER		14

#define	IRQ_UART1      0
#define	IRQ_UART2      1
#define	IRQ_SPI_SLAVE  2
#define	IRQ_SPI_MASTER 3
#define	IRQ_I2C_SLAVE  4
#define	IRQ_I2C_MASTER 5

// declarations for SPI slave interrupt
#define	IRQ_SPI_SLAVE_RESET	3
#define	IRQ_SPI_SLAVE_HOST_READ_DONE	2
#define	IRQ_SPI_SLAVE_HOST_WRITE_DONE	1
#define	IRQ_SPI_SLAVE_CMD_CHECK	0

// declarations for public functions
void attachInterrupt(uint8_t pin, void (*taskFunc)(void), int mode);
void detachInterrupt(uint8_t pin);
void interrupts(void);
void noInterrupts(void);
void hwISRFunc(void);

#ifdef __cplusplus
}
#endif

#endif // _WIRING_INTERRUPT_
