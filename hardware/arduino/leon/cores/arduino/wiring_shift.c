/*
  wiring_shift.c - C file for Arduino pseudo serial functions
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

#include "Arduino.h"
#include "wiring_digital.h"
#include "wiring_shift.h"

#ifdef __cplusplus
extern "C" {
#endif

// **********************************************************************
// Description: Shifts out a byte of data bit-by-bit with clock toggled
//
// Syntax: shiftOut(dataPin, clockPin, bitOrder, val)
//      -- dataPin: the number of the pin on which the serial data is out
//      -- clockPin: the number of the pin as a clock pin to toggle
//      -- bitOrder: flag to indicate which bit (MSB or LSB) is sent first
//      -- val: the 8-bit data to be sent serially on dataPin
// **********************************************************************
void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val)
{
	uint8_t k;

	// check if dataPin and clockPin are in OUTPUT mode

	// drive the data serially out
	digitalWrite(clockPin, LOW);
	for (k = 0; k < 8; k++) {
		if (bitOrder == LSBFIRST) {
			digitalWrite(dataPin, (val >> k) & 0x01);
		}
		else if (bitOrder == MSBFIRST) {
			digitalWrite(dataPin, (val >> (7-k)) & 0x01);
		}
		digitalWrite(clockPin, HIGH); // rising edge
		digitalWrite(clockPin, LOW);
	}
}

// **********************************************************************
// Description: Shifts in a byte of data bit-by-bit with clock toggled
//
// Syntax: ret = shiftIn(dataPin, clockPin, bitOrder)
//      -- ret: 8-bit data received on dataPin
//      -- dataPin: the number of the pin on which the serial data is received
//      -- clockPin: the number of the pin as a clock pin to toggle
//      -- bitOrder: flag to indicate where to place the 1st received bit
// **********************************************************************
uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder)
{
	uint8_t value = 0;
	uint8_t k;

	// check if dataPin is in INPUT mode and clockPin is in OUTPUT mode

	// sample the data in serially
	digitalWrite(clockPin, LOW);
#ifdef SHIFT_IN_PRE_CLOCK
	digitalWrite(clockPin, HIGH); // rising edge
	digitalWrite(clockPin, LOW);
#endif
	for (k = 0; k < 8; k++) {
		if (bitOrder == LSBFIRST) {
			value |= (digitalRead(dataPin) << k);
		}
		else if (bitOrder == MSBFIRST) {
			value |= (digitalRead(dataPin) << (7-k));
		}
		digitalWrite(clockPin, HIGH); // rising edge
		digitalWrite(clockPin, LOW);
	}
	return value;
}

// **********************************************************************
// Description: Send data/command to the PCD8544 LCD controller
//
// Syntax:
//      -- sce_n: pin number for "chip enable"
//      -- dc_n: pin number for "data/command"
//      -- sclk: pin number for "serial clock input"
//      -- sdin: pin number for "serial data input"
//      -- res_n: pin number for "external reset input"
//      -- ptr: the pointer to data structure for PCD8544
// **********************************************************************
void pcd8544SerialBus(uint8_t sce_n, uint8_t dc_n, uint8_t sclk, uint8_t sdin, uint8_t res_n, pPCD8544Data ptr)
{
	uint16_t i;
	uint8_t	j;
	uint8_t	data;

	digitalWrite(sce_n, LOW);	// chip enable
	for (i = 0; i < ptr->numData; i++)
	{
		if (ptr->type[i] == 2) { // reset
			digitalWrite(res_n, LOW);
			digitalWrite(sclk, HIGH);
			digitalWrite(sclk, LOW);
			digitalWrite(sclk, HIGH);
			digitalWrite(sclk, LOW);
			digitalWrite(res_n, HIGH);
		}
		else {
			data = ptr->data[i];
			for (j = 0; j < 8; j++) {
				(data & (1<<(7-j))) ? digitalWrite(sdin, HIGH) : digitalWrite(sdin, LOW);
				if (j == 7 && ptr->type[i] == 0) digitalWrite(dc_n, LOW);
				digitalWrite(sclk, HIGH);
				digitalWrite(sclk, LOW);
				if (j == 7 && ptr->type[i] == 0) digitalWrite(dc_n, HIGH);
			}
		}
	}
	digitalWrite(sce_n, HIGH); // chip disable
}

// **********************************************************************
// Description: Clean the LCD screen
//
// Syntax: pcd8544CleanScreen()
// **********************************************************************
void pcd8544CleanScreen(void)
{
	uint16_t i;
	uint8_t type = PCD8544_NORMAL_DATA;
	uint8_t data = 0;
	sPCD8544Data digit;

	// clean the RAM
	digit.numData = 1;
	digit.type = &type;
	digit.data = &data;
	for (i = 0; i < 504; i++) {
		pcd8544SerialBus(PCD8544_SCE_PIN, PCD8544_DC_PIN, PCD8544_SCLK_PIN, PCD8544_SDIN_PIN, PCD8544_RES_PIN, &digit);
	}
}

// **********************************************************************
// Description: Init the LCD controller
//
// Syntax: pcd8544Init()
// **********************************************************************
void pcd8544Init(void)
{
	uint8_t type[7];
	uint8_t data[7];
	sPCD8544Data digit;

	digit.numData = 7; digit.type = &type[0]; digit.data = &data[0];
	// apply reset
	type[0] = PCD8544_RESET;
	data[0] = 0x00;

	// function set : PD=0, V=0 and H=1
	type[1] = PCD8544_INSTRUCTION;
	data[1] = 0x21; // 8'b00100001;

	// set Vop (contrast)
	type[2] = PCD8544_INSTRUCTION;
	data[2] = 0xc0;	// 8'b11000000, Vop[6:0] = 64, Vlcd = 3.06 + 64*0.06 = 6.9

	// set temp coeff.
	type[3] = PCD8544_INSTRUCTION;
	data[3] = 0x04;	// 8'b00000100

	// lcd bias mode 1:48
	type[4] = PCD8544_INSTRUCTION;
	data[4] = 0x13; // 8'b00010011

	// function set : PD=0, V=0 and H=0
	type[5] = PCD8544_INSTRUCTION;
	data[5] = 0x20; // 8'b00100000

	// display control : set normal mode (D=1 and E=0)
	type[6] = PCD8544_INSTRUCTION;
	data[6] = 0x0c; // 8'b00001100
	pcd8544SerialBus(PCD8544_SCE_PIN, PCD8544_DC_PIN, PCD8544_SCLK_PIN, PCD8544_SDIN_PIN, PCD8544_RES_PIN, &digit);

	// clean the RAM
	pcd8544CleanScreen();
}

// **********************************************************************
// Description: Set cursor to specified Bank (y) and x
//
// Syntax: pcd8544SetCursor(bank, x)
// **********************************************************************
void pcd8544SetCursor(uint8_t bank, uint8_t x)
{
	uint8_t type[3];
	uint8_t data[3];
	sPCD8544Data digit;

	if (bank > 5 || x > 83) return;

	digit.numData = 3; digit.type = &type[0]; digit.data = &data[0];
	// function set : PD=0, V=0 and H=0
	type[0] = PCD8544_INSTRUCTION;
	data[0] = 0x20; // 8'b00100000
	// set bank of RAM
	type[1] = PCD8544_INSTRUCTION;
	data[1] = 0x40 | (bank & 0x07);
	// set X-address part of RAM
	type[2] = PCD8544_INSTRUCTION;
	data[2] = 0x80 | (x & 0x7f);
	// write instruction to LCD
	pcd8544SerialBus(PCD8544_SCE_PIN, PCD8544_DC_PIN, PCD8544_SCLK_PIN, PCD8544_SDIN_PIN, PCD8544_RES_PIN, &digit);
}

// **********************************************************************
// Description: Demo function to plot digit 0 ~ 9 on screen
//
// Syntax: pcd8544DigitPlot()
// **********************************************************************
sPCD8544Digit pcd8544_digit[] =
{
	{{1, 1, 1, 1, 1}, {0x0, 0x7e, 0x42, 0x7e, 0x0}},// '0'
	{{1, 1, 1, 1, 1}, {0x0, 0x42, 0x7e, 0x40, 0x0}},// '1'
	{{1, 1, 1, 1, 1}, {0x0, 0x72, 0x52, 0x5e, 0x0}},// '2'
	{{1, 1, 1, 1, 1}, {0x0, 0x52, 0x52, 0x7e, 0x0}},// '3'
	{{1, 1, 1, 1, 1}, {0x0, 0x1e, 0x10, 0x7e, 0x0}},// '4'
	{{1, 1, 1, 1, 1}, {0x0, 0x5e, 0x52, 0x72, 0x0}},// '5'
	{{1, 1, 1, 1, 1}, {0x0, 0x7e, 0x52, 0x72, 0x0}},// '6'
	{{1, 1, 1, 1, 1}, {0x0, 0x02, 0x02, 0x7e, 0x0}},// '7'
	{{1, 1, 1, 1, 1}, {0x0, 0x7e, 0x52, 0x7e, 0x0}},// '8'
	{{1, 1, 1, 1, 1}, {0x0, 0x1e, 0x12, 0x7e, 0x0}}	// '9'
};

void pcd8544DigitPlot(void)
{
	uint8_t	i;
	sPCD8544Data digit;

	// init the LCD controller
	pcd8544Init();

	// write data to RAM, start @ Bank 0, X=0
	digit.numData = 5;
	for (i = 0; i < 10; i++) {
		digit.type = &pcd8544_digit[i].type[0];
		digit.data = &pcd8544_digit[i].data[0];
		pcd8544SerialBus(PCD8544_SCE_PIN, PCD8544_DC_PIN, PCD8544_SCLK_PIN, PCD8544_SDIN_PIN, PCD8544_RES_PIN, &digit);
	}

	// write data to RAM, start @ Bank 1, X=10
	pcd8544SetCursor(1, 10);
	digit.numData = 5;
	for (i = 0; i < 10; i++) {
		digit.type = &pcd8544_digit[i].type[0];
		digit.data = &pcd8544_digit[i].data[0];
		pcd8544SerialBus(PCD8544_SCE_PIN, PCD8544_DC_PIN, PCD8544_SCLK_PIN, PCD8544_SDIN_PIN, PCD8544_RES_PIN, &digit);
	}

	// write data to RAM, start @ Bank 2, X=20
	pcd8544SetCursor(2, 20);
	digit.numData = 5;
	for (i = 0; i < 10; i++) {
		digit.type = &pcd8544_digit[i].type[0];
		digit.data = &pcd8544_digit[i].data[0];
		pcd8544SerialBus(PCD8544_SCE_PIN, PCD8544_DC_PIN, PCD8544_SCLK_PIN, PCD8544_SDIN_PIN, PCD8544_RES_PIN, &digit);
	}
}

void pcd8544FractalPlot(void)
{
	uint8_t	type;
	uint8_t	data;
	sPCD8544Data digit;

	uint8_t i, j;
	uint8_t	jj;
	uint8_t i_max = 15;
#if 1
	uint8_t fraimg[84][48];
	uint8_t	img_height = 48;
	uint8_t	img_width = 84;
	uint8_t	iteration;
	float x_min = -1.6;
	float x_max = 0.6;
	float y_min = -1.2;
	float y_max = 1.2;
	float x, y;
	float	v_x, v_y;
	float	z_x, z_y;
	float	k_x, k_y;
	float	dx, dy;
#endif

	// set the structure of data
	digit.numData = 1;
	digit.type = &type;
	digit.data = &data;
	type = PCD8544_NORMAL_DATA;

#if 1
	// clean the content of 2-D array
	for (j = 0; j < img_height; j ++) {
		for (i = 0; i < img_width; i ++) {
			fraimg[i][j] = 0;
		}
	}
	// define the incremental for calculation
	dx = (x_max - x_min) / (img_width - 1);
	dy = (y_max - y_min) / (img_height - 1);
	// calculate the 2-D array
	for (j = 0; j < img_height; j ++) {
		// reset cursor
		pcd8544SetCursor(j/8, 0);
		// calculate the 2-D array alone x-axis
		for (i = 0; i < img_width; i ++) {
			x = x_min + (i+1) * dx;
			y = y_min + (j+1) * dy;
			iteration = 1;
			z_x = 0; z_y = 0;	// z = [0,0]
			v_x = 0; v_y = 0;	// v = [0,0]
			while ((iteration <= i_max) && ((v_x + v_y) <= 4.0)) {
				k_x = z_x;
				k_y = z_y;	// k = z

				z_x = v_x - v_y;
				z_y = 2 * k_x * k_y;

				z_x = z_x + x;
				z_y = z_y + y;

				v_x = z_x * z_x;
				v_y = z_y * z_y;

				iteration ++;
			}
			fraimg[i][j] = (iteration == i_max) ? (i_max + 1) : iteration;

			// plot "dot" in case of fraimg[x][y] is "i_max + 1"
			jj = (j >> 3); // divided by 8
			data = 0;
			data = ((fraimg[i][jj*8+0] == (i_max + 1)) ? (1<<0) : 0)
			     | ((fraimg[i][jj*8+1] == (i_max + 1)) ? (1<<1) : 0)
					 | ((fraimg[i][jj*8+2] == (i_max + 1)) ? (1<<2) : 0)
					 | ((fraimg[i][jj*8+3] == (i_max + 1)) ? (1<<3) : 0)
					 | ((fraimg[i][jj*8+4] == (i_max + 1)) ? (1<<4) : 0)
					 | ((fraimg[i][jj*8+5] == (i_max + 1)) ? (1<<5) : 0)
					 | ((fraimg[i][jj*8+6] == (i_max + 1)) ? (1<<6) : 0)
					 | ((fraimg[i][jj*8+7] == (i_max + 1)) ? (1<<7) : 0);
			pcd8544SerialBus(PCD8544_SCE_PIN, PCD8544_DC_PIN, PCD8544_SCLK_PIN, PCD8544_SDIN_PIN, PCD8544_RES_PIN, &digit);
			// delay for displaying purpose
//		for (ii = 0; ii < 5000; ii++) { asm("nop"); }
		}	// i = 0 ~ img_width-1
	}	// j = 0 ~ img_height-1
#endif
}

#ifdef __cplusplus
}
#endif
