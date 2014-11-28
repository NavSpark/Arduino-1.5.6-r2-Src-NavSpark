/*
  wiring_intr.c - C file for Arduino user-defined interrupt functions
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

#include <stdio.h>
#include "stdint.h"
#include "Arduino.h"
#include "sti_gnss_lib.h"
#include "wiring_intr.h"

extern void isrSerialFunc(uint8_t port);
extern void isrTwoWireSlaveFunc(void);
extern void isrSPISlaveFunc(uint8_t type);

extern uint8_t v8_io_irq_query(uint8_t id);
extern uint8_t v8_uart_pisr_query(uint8_t id);
extern void taskSerialFunc(uint8_t port);

extern uint8_t v8_spi_slave_reset_intr_query(void);
extern uint8_t v8_spi_slave_s2m_buffer_empty_intr_query(void);
extern uint8_t v8_spi_slave_m2s_buffer_full_intr_query(void);
extern uint8_t v8_spi_slave_cmd_sts_intr_query(void);
extern void v8_spi_slave_reset_intr_clr(void);
extern void v8_spi_slave_s2m_buffer_empty_intr_clr(void);
extern void v8_spi_slave_m2s_buffer_full_intr_clr(uint8_t m2s_enable);
extern void v8_spi_slave_cmd_sts_intr_clr(void);

#ifdef __cplusplus
extern "C" {
#endif

static uint8_t numOfPinHookOnISR = 0;
static uint32_t	bitMapOfPinHookOnISR = 0;

// **********************************************************************
// Description:
// **********************************************************************
void gpioIntrSrvRoutine(void)
{
	uint8_t	k;
	int	pinVal;

	gnss_disable_irq(GPIO_ISR_NUMBER);
	for (k = 0; k < 32; k++)
	{
		if (gpioConf[k].isrFunc != NULL)
		{
			// check if this pin has an interrupt trig occured
			if (gpioConf[k].intr_mode != 0xff)
			{
				pinVal = digitalRead(k);
				// check the level of pin for matching the interrupt mode
				if (((gpioConf[k].intr_mode == RISING)  && (pinVal == 1)) ||
						((gpioConf[k].intr_mode == FALLING) && (pinVal == 0)))
				{
					// excute the ISR
					gpioConf[k].isrFunc();
				}
			}
		}
	}
	gnss_start_irq(GPIO_ISR_NUMBER);
}

// **********************************************************************
// Description:
// **********************************************************************
void attachInterrupt(uint8_t pin, void (*taskFunc)(void), int mode)
{
	uint8_t	turn_on_ISR = 0;

	// only INPUT pin can trig interrupt
	if (gpioConf[pin].io_mode != INPUT)	return;

	// check if this pin has been hooked
	if (bitMapOfPinHookOnISR & (0x1UL << pin)) return;
	else {
		// keep this pin in bitmap for which pins were hooked
		bitMapOfPinHookOnISR |= (0x1UL << pin);
		numOfPinHookOnISR ++;
		turn_on_ISR = 1;
	}

	// set the ISR for this pin
	gpioConf[pin].intr_mode = mode;
	gpioConf[pin].isrFunc = taskFunc;

  // set the trig way (currently all GPIO pins must share same way ...)
	if      (mode == FALLING) { gnss_gpio_interrupt_property(0, 1); }
	else if (mode == RISING)  { gnss_gpio_interrupt_property(1, 1); }

	// register the service routine entry
	if (turn_on_ISR) {
		// hook "userFunc" to be the GPIO interrupt ISR
		gnss_disable_irq(GPIO_ISR_NUMBER);
		gnss_setup_isr(GPIO_ISR_NUMBER, gpioIntrSrvRoutine);
		gnss_start_irq(GPIO_ISR_NUMBER);
	}

	// unmask the interrupt for this pin
  gnss_gpio_interrupt_enable(pin);
}

// **********************************************************************
// Description:
// **********************************************************************
void detachInterrupt(uint8_t pin)
{
	// check if this pin was already hooked
	if (bitMapOfPinHookOnISR & (0x1UL << pin))
	{
		// mask the interrupt for this pin
		gnss_gpio_interrupt_disable(pin);

		// remove pin from bitmap
		bitMapOfPinHookOnISR &= (~(0x1UL << pin));
		numOfPinHookOnISR --;

		// remove the ISR for this pin
		gpioConf[pin].intr_mode = 0xff;
		gpioConf[pin].isrFunc = NULL;

		// turn off the ISR in case of no GPIO pins are hooked
		if (numOfPinHookOnISR == 0) {
			gnss_disable_irq(GPIO_ISR_NUMBER);
		}
	}
}

// **********************************************************************
// Description:
// **********************************************************************
void interrupts(void)
{
	gnss_enable_irq(GPS_ISR_NUMBER);
	gnss_enable_irq(TIMER_ISR_NUMBER);
	gnss_enable_irq(UART_ISR_NUMBER);
	gnss_enable_irq(GPIO_ISR_NUMBER);
}

// **********************************************************************
// Description:
// **********************************************************************
void noInterrupts(void)
{
	gnss_disable_irq(GPIO_ISR_NUMBER);
	gnss_disable_irq(UART_ISR_NUMBER);
	gnss_disable_irq(TIMER_ISR_NUMBER);
	gnss_disable_irq(GPS_ISR_NUMBER);
}

// **********************************************************************
// Description: The ISR function for UART/I2C/SPI
// **********************************************************************
void hwISRFunc(void)
{
	uint8_t pisr;

#if !defined(USE_UART1_FOR_NMEA) || (USE_UART1_FOR_NMEA==0)
	// Serial1
	if (v8_io_irq_query(IRQ_UART1)) {
		pisr = v8_uart_pisr_query(0);
		if (pisr & ((0x1<<4) | (0x1<<2))) {
			isrSerialFunc(0);
		}
		if (pisr & (0x1<<1)) {
			taskSerialFunc(0);
		}
	}
#endif

	// Serial
	if (v8_io_irq_query(IRQ_UART2)) {
		pisr = v8_uart_pisr_query(1);
		if (pisr & ((0x1<<4) | (0x1<<2))) {
			isrSerialFunc(1);
		}
		if (pisr & (0x1<<1)) {
			taskSerialFunc(1);
		}
	}

	// i2cSlave
	if (v8_io_irq_query(IRQ_I2C_SLAVE)) {
		isrTwoWireSlaveFunc();
	}

	// spiSlave
	if (v8_io_irq_query(IRQ_SPI_SLAVE)) {
		if (v8_spi_slave_reset_intr_query()) {
			v8_spi_slave_reset_intr_clr();
			isrSPISlaveFunc(IRQ_SPI_SLAVE_RESET);
		}
		if (v8_spi_slave_s2m_buffer_empty_intr_query()) {
			v8_spi_slave_s2m_buffer_empty_intr_clr();
			isrSPISlaveFunc(IRQ_SPI_SLAVE_HOST_READ_DONE);
		}
		if (v8_spi_slave_m2s_buffer_full_intr_query()) {
			v8_spi_slave_m2s_buffer_full_intr_clr(0);
			isrSPISlaveFunc(IRQ_SPI_SLAVE_HOST_WRITE_DONE);
		}
		if (v8_spi_slave_cmd_sts_intr_query()) {
			v8_spi_slave_cmd_sts_intr_clr();
			isrSPISlaveFunc(IRQ_SPI_SLAVE_CMD_CHECK);
		}
	}
}

#ifdef __cplusplus
}
#endif
