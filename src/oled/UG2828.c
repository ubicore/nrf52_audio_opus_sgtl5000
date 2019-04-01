/*****************************************************************************
 *
 *   Copyright(C) 2012, Embedded Artists AB
 *   All rights reserved.
 *
 ******************************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * Embedded Artists AB assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. Embedded Artists AB
 * reserves the right to make changes in the software without
 * notification. Embedded Artists AB also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
//#include "lpc_types.h"
#include <zephyr.h>

#include "UG2828.h"
#include <string.h>
#include <stdio.h>


#include "ssd1351.h"
#include "draw.h"
//#include "bsp.h"
#include "oled_spi.h"

/******************************************************************************
 * Typedefs and defines
 *****************************************************************************/

#define UG_DISPLAY_WIDTH  (128)
#define UG_DISPLAY_HEIGHT (128)

/******************************************************************************
 * Local variables
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/

static void pixel(uint16_t x, uint16_t y, uint16_t color)
{
  ssd1351_setColumnAddress(x,x+1);
  ssd1351_setRowAddress(y,y+1);
  ssd1351_setWriteRAM();

  bsp_ssd1351Data((color >> 8) & 0xff);  // color - RRRRRGGG
  bsp_ssd1351Data(color & 0xff);         // color - GGGBBBBB
}

static void window(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  uint16_t x1, y1;

  if (x > UG_DISPLAY_WIDTH || y > UG_DISPLAY_HEIGHT) {
    return;
  }

  x1 = x+w-1;
  y1 = y+h-1;

  if (x1 > UG_DISPLAY_WIDTH-1) {
    x1 = UG_DISPLAY_WIDTH-1;
  }
  if (y1 > UG_DISPLAY_HEIGHT-1) {
    y1 = UG_DISPLAY_HEIGHT-1;
  }

  ssd1351_setColumnAddress(x, x1);
  ssd1351_setRowAddress(y, y1);
  ssd1351_setWriteRAM();
}

static void winPixel(uint16_t color)
{
  bsp_ssd1351Data((color >> 8) & 0xff);  // color - RRRRRGGG
  bsp_ssd1351Data(color & 0xff);         // color - GGGBBBBB
}

static void fill(uint16_t color)
{
  int i = 0;

  ssd1351_setColumnAddress(0, 127);
  ssd1351_setRowAddress(0, 127);
  ssd1351_setWriteRAM();

  for (i = 0; i < 128*128; i++) {
    bsp_ssd1351Data((color >> 8) & 0xff);  // color - RRRRRGGG
    bsp_ssd1351Data(color & 0xff);         // color - GGGBBBBB
  }

}


static void oledInit()
{
  //bsp_display_reset();

  ssd1351_setCommandLock(0x12);			// Unlock Driver IC (0x12/0x16/0xB0/0xB1)
  ssd1351_setCommandLock(0xB1);			// Unlock All Commands (0x12/0x16/0xB0/0xB1)
  ssd1351_setDisplayOnOff(0x00);		// Display Off (0x00/0x01)
  ssd1351_setDisplayClock(0xF1);		// Set Clock as 90 Frames/Sec
  ssd1351_setMultiplexRatio(0x7F);		// 1/128 Duty (0x0F~0x7F)

  ssd1351_setDisplayOffset(0x00);		// Shift Mapping RAM Counter (0x00~0x7F)
  ssd1351_setStartLine(0x00);			// Set Mapping RAM Display Start Line (0x00~0x7F)
  ssd1351_setRemapFormat(0x74);			// Set Horizontal Address Increment
  //     Column Address 0 Mapped to SEG0
  //     Color Sequence D[15:0]=[RRRRR:GGGGGG:BBBBB]
  //     Scan from COM127 to COM0
  //     Enable COM Split Odd Even
  //     65,536 Colors Mode (0x74)
  //     * 262,144 Colors Mode (0xB4)
  ssd1351_setGPIO(0x00);				// Set Low Voltage Level of SEG Pin
  ssd1351_setFunctionSelection(0x01);		// Enable Internal VDD Regulator
  // Select 8-bit Parallel Interface
  ssd1351_setVSL(0xA0);				// Enable External VSL
  ssd1351_setContrastColor(0xC8,0x80,0xC8);	// Set Contrast of Color A (Red)
  // Set Contrast of Color B (Green)
  // Set Contrast of Color C (Blue)
  ssd1351_setMasterCurrent(MAX_MASTER_CURRENT);		// Set Scale Factor of Segment Output Current Control
  ssd1351_setGrayScaleTable();			// Set Pulse Width for Gray Scale Table
  ssd1351_setPhaseLength(0x32);			// Set Phase 1 as 5 Clocks & Phase 2 as 3 Clocks
  ssd1351_setPrechargeVoltage(0x17);		// Set Pre-Charge Voltage Level as 0.50*VCC
  ssd1351_setDisplayEnhancement(0xA4);		// Enhance Display Performance
  ssd1351_setPrechargePeriod(0x01);		// Set Second Pre-Charge Period as 1 Clock
  ssd1351_setVCOMH(0x05);			// Set Common Pins Deselect Voltage Level as 0.82*VCC
  ssd1351_setDisplayMode(0x02);			// Normal Display Mode (0x00/0x01/0x02/0x03)

  fill(/*0x0000*/0xffff); // clear screen

  // powerup external +13V
  ssd1351_setGPIO(0x03);					// Set High Voltage Level of GPIO0 Pin

  k_sleep(K_MSEC(100));

  ssd1351_setDisplayOnOff(0x01);		// Display On (0x00/0x01)
}

/******************************************************************************
 * Public Functions
 *****************************************************************************/

/******************************************************************************
 *
 * Description:
 *   Initialize the display
 *
 * Params:
 *   [out] lcd - this function will initialize the lcd struct with
 *         values/callbacks valid for this display
 *
 *****************************************************************************/

void ug2828_init(draw_lcd_t* lcd)
{
  memset(&lcd, sizeof(draw_lcd_t), 0);

  lcd->width = UG_DISPLAY_WIDTH;
  lcd->height = UG_DISPLAY_HEIGHT;

  lcd->pixel = &pixel;
  lcd->winBound = &window;
  lcd->winPixel = &winPixel;

  oledInit();
}


