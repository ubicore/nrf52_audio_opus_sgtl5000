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
#ifndef __UG_DRAW_H
#define __UG_DRAW_H

#include <stdint.h>

typedef struct {
  /* width of display */
  uint16_t width;

  /* height of display */
  uint16_t height;

  /*
   * Drawing primitives provided by the display controller.
   * The only required function is 'pixel'. All other are optional.
   */

  /* draw a pixel */
  void (*pixel)(uint16_t x, uint16_t y, uint16_t color);

  void(*winBound)(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
  void(*winPixel)(uint16_t color);

} draw_lcd_t;


void draw_pixel(draw_lcd_t* lcd, uint16_t x, uint16_t y, uint16_t color);
void draw_line(draw_lcd_t* lcd, uint16_t x0, uint16_t y0, uint16_t x1,
    uint16_t y1, uint16_t color);
void draw_rectangle(draw_lcd_t* lcd, uint16_t x0, uint16_t y0, uint16_t x1,
    uint16_t y1, uint16_t color);
void draw_fillRectangle(draw_lcd_t* lcd, uint16_t x0, uint16_t y0, uint16_t x1,
    uint16_t y1, uint16_t color);
void draw_circle(draw_lcd_t* lcd, uint16_t x0, uint16_t y0, uint16_t r,
    uint16_t color);
void draw_fillCircle(draw_lcd_t* lcd, uint16_t x0, uint16_t y0, uint16_t r,
    uint16_t color);
void draw_eaImg(draw_lcd_t* lcd, uint16_t x0, uint16_t y0, uint16_t* img);




#endif

