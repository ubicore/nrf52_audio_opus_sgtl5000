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
#ifndef __UG_DRAWFONTS_H
#define __UG_DRAWFONTS_H

#include "lpc_fonts.h"


void draw_setFont(const FONT_T* font);

void draw_putChar(draw_lcd_t* lcd, uint16_t x, uint16_t y,
    const char textchar, uint16_t fgColor, uint16_t bgColor);
void draw_putText(draw_lcd_t* lcd, uint16_t x, uint16_t y,
    const char* str, uint16_t fgColor, uint16_t bgColor);



#endif
