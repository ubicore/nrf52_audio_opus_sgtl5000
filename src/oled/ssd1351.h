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
#ifndef __SSD1351_H
#define __SSD1351_H

#define MAX_MASTER_CURRENT  0x0F

void ssd1351_setColumnAddress(unsigned char a, unsigned char b);
void ssd1351_setRowAddress(unsigned char a, unsigned char b);
void ssd1351_setWriteRAM();
void ssd1351_setReadRAM();
void ssd1351_setRemapFormat(unsigned char d);
void ssd1351_setStartLine(unsigned char d);
void ssd1351_setDisplayOffset(unsigned char d);
void ssd1351_setDisplayMode(unsigned char d);
void ssd1351_setFunctionSelection(unsigned char d);
void ssd1351_setDisplayOnOff(unsigned char d);
void ssd1351_setPhaseLength(unsigned char d);
void ssd1351_setDisplayEnhancement(unsigned char d);
void ssd1351_setDisplayClock(unsigned char d);
void ssd1351_setVSL(unsigned char d);
void ssd1351_setGPIO(unsigned char d);
void ssd1351_setPrechargePeriod(unsigned char d);
void ssd1351_setPrechargeVoltage(unsigned char d);
void ssd1351_setVCOMH(unsigned char d);
void ssd1351_setContrastColor(unsigned char a, unsigned char b, unsigned char c);
void ssd1351_setMasterCurrent(unsigned char d);
void ssd1351_setMultiplexRatio(unsigned char d);
void ssd1351_setCommandLock(unsigned char d);
void ssd1351_verticalScroll(uint8_t dir, uint8_t rows, uint8_t interval);
void ssd1351_horizontalScroll(uint8_t dir, uint8_t cols, uint8_t rowStart,
    uint8_t rows, uint8_t interval, uint8_t delay);
void ssd1351_deactivateScroll(void);
void ssd1351_fadeIn(void);
void ssd1351_fadeOut(void);
void ssd1351_setGrayScaleTable(void);

#endif
