#include "stdint.h"

#define	PCD8544_INSTRUCTION	0
#define	PCD8544_NORMAL_DATA	1
#define	PCD8544_RESET				2

typedef	struct _pcd8544_data
{
  uint16_t numData;
  uint8_t *type; // 0: instruction, 1: normal data, 2: reset
  uint8_t *data;
} sPCD8544Data, *pPCD8544Data;

typedef	struct _pcd8544_digit
{
  uint8_t type[7];
  uint8_t data[7];
} sPCD8544Digit, *pPCD8544Digit;

#ifdef __cplusplus
extern "C" {
#endif

// unmark following line in case of one more clock needed before sampling
// 1st bit of data in "shiftIn()"

// defines for pins for PCD8544
#define	PCD8544_LED_PIN		3
#define	PCD8544_SCE_PIN		10
#define	PCD8544_DC_PIN		16	// gpio 22 has bug waiting for fix
#define	PCD8544_SCLK_PIN	30
#define	PCD8544_SDIN_PIN	28
#define	PCD8544_RES_PIN		5

void pcd8544SerialBus(uint8_t sce_n, uint8_t dc_n, uint8_t sclk, uint8_t sdin, uint8_t res_n, pPCD8544Data ptr);
void pcd8544Init(void);
void pcd8544CleanScreen(void);
void pcd8544SetCursor(uint8_t bank, uint8_t x);
void printOnLCD(uint8_t ch);
void pcd8544DigitPlot(void);
void pcd8544FractalPlot(void);

#ifdef __cplusplus
}
#endif
