#include "Arduino.h"
#include "wiring_digital.h"
#include "Nokia_5110.h"

#ifdef __cplusplus
extern "C" {
#endif
    
#define _CHECK_MARK 0xf0
uint8_t PCD8544Row;
uint8_t PCD8544Column;

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
	// move cursor back to (0,0)
	PCD8544Row = 0;
	PCD8544Column = 0;
	pcd8544SetCursor(0, 0);
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
	PCD8544Row = 0;
	PCD8544Column = 0;
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
	{{1, 1, 1, 1, 1, 1, 1}, {0x0, 0x7e, 0x42, 0x7e, 0x0, 0x0, 0x0}},// '0'
	{{1, 1, 1, 1, 1, 1, 1}, {0x0, 0x44, 0x7e, 0x40, 0x0, 0x0, 0x0}},// '1'
	{{1, 1, 1, 1, 1, 1, 1}, {0x0, 0x72, 0x52, 0x5e, 0x0, 0x0, 0x0}},// '2'
	{{1, 1, 1, 1, 1, 1, 1}, {0x0, 0x52, 0x52, 0x7e, 0x0, 0x0, 0x0}},// '3'
	{{1, 1, 1, 1, 1, 1, 1}, {0x0, 0x1e, 0x10, 0x7e, 0x0, 0x0, 0x0}},// '4'
	{{1, 1, 1, 1, 1, 1, 1}, {0x0, 0x5e, 0x52, 0x72, 0x0, 0x0, 0x0}},// '5'
	{{1, 1, 1, 1, 1, 1, 1}, {0x0, 0x7e, 0x52, 0x72, 0x0, 0x0, 0x0}},// '6'
	{{1, 1, 1, 1, 1, 1, 1}, {0x0, 0x02, 0x02, 0x7e, 0x0, 0x0, 0x0}},// '7'
	{{1, 1, 1, 1, 1, 1, 1}, {0x0, 0x7e, 0x52, 0x7e, 0x0, 0x0, 0x0}},// '8'
	{{1, 1, 1, 1, 1, 1, 1}, {0x0, 0x1e, 0x12, 0x7e, 0x0, 0x0, 0x0}},// '9'
};


sPCD8544Digit pcd8544_char[] =
{
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x78, 0x14, 0x12, 0x14, 0x78, 0x00}}, //'A'
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x7e, 0x52, 0x52, 0x2c, 0x00, 0x00}}, //'B'
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x3c, 0x42, 0x42, 0x42, 0x00, 0x00}}, //'C'
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x7e, 0x42, 0x42, 0x3c, 0x00, 0x00}}, //'D'
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x7e, 0x4a, 0x4a, 0x4a, 0x00, 0x00}}, //'E'
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x7e, 0x12, 0x12, 0x02, 0x00, 0x00}}, //'F'
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x3c, 0x42, 0x52, 0x34, 0x00, 0x00}}, //'G'
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x7e, 0x10, 0x10, 0x7e, 0x00, 0x00}}, //'H'
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x00, 0x42, 0x7e, 0x42, 0x00, 0x00}}, //'I'
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x20, 0x40, 0x3e, 0x00, 0x00, 0x00}}, //'J'
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x7e, 0x28, 0x24, 0x42, 0x00, 0x00}}, //'K'
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x7e, 0x40, 0x40, 0x40, 0x00, 0x00}}, //'L'
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x7e, 0x04, 0x08, 0x04, 0x7e, 0x00}}, //'M'
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x7e, 0x08, 0x10, 0x7e, 0x00, 0x00}}, //'N'
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x3c, 0x42, 0x42, 0x3c, 0x00, 0x00}}, //'O'
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x7e, 0x12, 0x12, 0x0c, 0x00, 0x00}}, //'P'
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x3c, 0x42, 0x62, 0x7c, 0x00, 0x00}}, //'Q'
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x7e, 0x12, 0x32, 0x4c, 0x00, 0x00}}, //'R'
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x6e, 0x4a, 0x52, 0x76, 0x00, 0x00}}, //'S'
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x02, 0x02, 0x7e, 0x02, 0x02, 0x00}}, //'T'
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x3e, 0x40, 0x40, 0x3e, 0x00, 0x00}}, //'U'
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x06, 0x38, 0x40, 0x38, 0x06, 0x00}}, //'V'
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x3e, 0x40, 0x20, 0x40, 0x3e, 0x00}}, //'W'
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x46, 0x28, 0x10, 0x28, 0x46, 0x00}}, //'X'
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x02, 0x04, 0x78, 0x04, 0x02, 0x00}}, //'Y'
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x42, 0x62, 0x5a, 0x46, 0x42, 0x00}}  //'Z'        
};

sPCD8544Digit pcd8544_symbol[] =
{
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}, // space
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x00, 0x6c, 0x6c, 0x00, 0x00, 0x00}}, // ':'
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x00, 0x60, 0x60, 0x00, 0x00, 0x00}}, // '.'
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x06, 0x00, 0x06, 0x00, 0x00, 0x00}}, // '"'
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00}}, // '''
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x10, 0x10, 0x10, 0x10, 0x10, 0x00}}, // '-'
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x00, 0x00, 0x0e, 0x0a, 0x0e, 0x00}}, // 'Degree'
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x20, 0x40, 0x20, 0x10, 0x08, 0x00}}, // '0xf0 check mark'
        {{1, 1, 1, 1, 1, 1, 1}, {0x00, 0x08, 0x04, 0xa2, 0x14, 0x08, 0x00}} // '?'
};

void printOnLCD(uint8_t ch)
{
    boolean print = true;
    sPCD8544Data digit;
    digit.numData = 7;
    
    if ((ch>='0') && (ch<='9')) { // 0 ~ 9
      digit.type = &pcd8544_digit[ch-'0'].type[0];
      digit.data = &pcd8544_digit[ch-'0'].data[0];
    }
    else if ((ch>='a') && (ch<='z')) { // a ~ z
      digit.type = &pcd8544_char[ch-'a'].type[0];
      digit.data = &pcd8544_char[ch-'a'].data[0];
    }
    else if ((ch>='A') && (ch<='Z')) { // A ~ Z
      digit.type = &pcd8544_char[ch-'A'].type[0];
      digit.data = &pcd8544_char[ch-'A'].data[0]; 
    }
    
    switch(ch)
    {
        case ' ':
            digit.type = &pcd8544_symbol[0].type[0];
            digit.data = &pcd8544_symbol[0].data[0];
            PCD8544Column++;
            break;
            
        case ':':
            digit.type = &pcd8544_symbol[1].type[0];
            digit.data = &pcd8544_symbol[1].data[0];
            PCD8544Column++;
            break;
            
        case '.':
            digit.type = &pcd8544_symbol[2].type[0];
            digit.data = &pcd8544_symbol[2].data[0];
            PCD8544Column++;
            break;
        
        case '"':
            digit.type = &pcd8544_symbol[3].type[0];
            digit.data = &pcd8544_symbol[3].data[0];
            PCD8544Column++;
            break;
        
        case '\'':
            digit.type = &pcd8544_symbol[4].type[0];
            digit.data = &pcd8544_symbol[4].data[0];
            PCD8544Column++;
            break;
        
        case '-':
            digit.type = &pcd8544_symbol[5].type[0];
            digit.data = &pcd8544_symbol[5].data[0];
            PCD8544Column++;
            break;
        
        case 0xf0:
            digit.type = &pcd8544_symbol[7].type[0];
            digit.data = &pcd8544_symbol[7].data[0];
            PCD8544Column++;
            break;
        
        case '?':
            digit.type = &pcd8544_symbol[8].type[0];
            digit.data = &pcd8544_symbol[8].data[0];
            PCD8544Column++;
            break;
            
        case '\r':
            print = false;
            PCD8544Column = 0;
            pcd8544SetCursor(PCD8544Row, PCD8544Column);
            break;
            
        case '\n':
            print = false;
            PCD8544Row = (PCD8544Row + 1) % 6;
            pcd8544SetCursor(PCD8544Row, PCD8544Column);
            break;  
    }
    if (print == true) {
      pcd8544SerialBus(PCD8544_SCE_PIN, PCD8544_DC_PIN, PCD8544_SCLK_PIN, PCD8544_SDIN_PIN, PCD8544_RES_PIN, &digit);
    }
}

void pcd8544DigitPlot(void)
{
	uint8_t	i;
	sPCD8544Data digit;

	// init the LCD controller
	pcd8544Init();

	// write data to RAM, start @ Bank 0, X=0
	digit.numData = 7;
	for (i = 0; i < 12; i++) {
		digit.type = &pcd8544_char[i].type[0];
		digit.data = &pcd8544_char[i].data[0];
		pcd8544SerialBus(PCD8544_SCE_PIN, PCD8544_DC_PIN, PCD8544_SCLK_PIN, PCD8544_SDIN_PIN, PCD8544_RES_PIN, &digit);
	}
        
	// write data to RAM, start @ Bank 1, X=10
	pcd8544SetCursor(1, 0);
	digit.numData = 7;
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

	// set the structure of data
	digit.numData = 1;
	digit.type = &type;
	digit.data = &data;
	type = PCD8544_NORMAL_DATA;

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
}

#ifdef __cplusplus
}
#endif
