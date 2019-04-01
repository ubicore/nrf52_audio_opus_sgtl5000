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
#include <stdint.h>
#include <stdio.h>

#include "draw.h"

/******************************************************************************
 * Local Functions
 *****************************************************************************/

static void verticalLine(draw_lcd_t* lcd, uint16_t x0, uint16_t y0,
    uint16_t y1, uint16_t color)
{
  uint16_t tmp;

  if (y0 > y1) {
    tmp = y1;
    y1 = y0;
    y0 = tmp;
  }

  if (lcd->winBound != NULL) {
    lcd->winBound(x0, y0, 1, y1-y0+1);
  }

  while (y1 >= y0) {
    if (lcd->winPixel != NULL) {
      lcd->winPixel(color);
    }
    else {
      lcd->pixel(x0, y0, color);
    }
    y0++;
  }
}

static void horizontalLine(draw_lcd_t* lcd, uint16_t x0, uint16_t y0,
    uint16_t x1, uint16_t color)
{
  uint16_t tmp;

  if (x0 > x1) {
    tmp = x1;
    x1 = x0;
    x0 = tmp;
  }

  if (lcd->winBound != NULL) {
    lcd->winBound(x0, y0, x1-x0+1, 1);
  }

  while(x1 >= x0) {
    if (lcd->winPixel != NULL) {
      lcd->winPixel(color);
    }else {
      lcd->pixel(x0, y0, color);
    }

    x0++;
  }
}


/******************************************************************************
 * Public Functions
 *****************************************************************************/

/******************************************************************************
 *
 * Description:
 *   Draw a pixel with specified color at specified location
 *
 * Params:
 *   [in] x - horizontal position
 *   [in] y - vertical position
 *   [in] color - 16-bit color (5:6:5 format)
 *
 *****************************************************************************/
void draw_pixel(draw_lcd_t* lcd, uint16_t x, uint16_t y, uint16_t color)
{
  // check if out of range
  if (x >= lcd->width || y >= lcd->height) {
    return;
  }

  // draw pixel using hw specific function
  lcd->pixel(x, y, color);
}

/******************************************************************************
 *
 * Description:
 *   Draw a line with specified color at specified location
 *
 * Params:
 *   [in] x0 - starting x coordinate
 *   [in] y0 - starting y coordinate
 *   [in] x1 - ending x coordinate
 *   [in] y1 - ending y coordinate
 *   [in] color - 16-bit color (5:6:5 format)
 *
 *****************************************************************************/
void draw_line(draw_lcd_t* lcd, uint16_t x0, uint16_t y0, uint16_t x1,
    uint16_t y1, uint16_t color)
{
  int32_t dx;
  int32_t dy;
  int32_t dx2;
  int32_t dy2;
  int32_t di;

  int8_t xs = 1;
  int8_t ys = 1;

  dx = x1-x0;
  dy = y1-y0;

  // vertical line
  if (dx == 0) {
    verticalLine(lcd, x0, y0, y1, color);
    return;
  }

  // horizontal line
  if (dy == 0) {
    horizontalLine(lcd, x0, y0, x1, color);
    return;
  }

  if (dx < 0) {
    xs = -1;
    dx = -dx;
  }

  if (dy < 0) {
    ys = -1;
    dy = -dy;
  }

  dx2 = dx*2;
  dy2 = dy*2;

  if (dx >= dy) {
    di = dy2 - dx;

    while (x0 != x1) {
      lcd->pixel(x0, y0, color);

      x0 += xs;

      if (di < 0) {
        di += dy2;
      }
      else {
        di += dy2 - dx2;
        y0 += ys;
      }
    }

    lcd->pixel(x0, y0, color);
  }
  else {
    di = dx2 - dy;
    while (y0 != y1) {
      lcd->pixel(x0, y0, color);

      y0 += ys;

      if (di < 0) {
        di += dx2;
      }
      else {
        di += dx2 - dy2;
        x0 += xs;
      }
    }

    lcd->pixel(x0, y0, color);
  }

}

/******************************************************************************
 *
 * Description:
 *   Draw a rectangle with specified color at specified location
 *
 * Params:
 *   [in] x0 - starting x coordinate
 *   [in] y0 - starting y coordinate
 *   [in] x1 - ending x coordinate
 *   [in] y1 - ending y coordinate
 *   [in] color - 16-bit color (5:6:5 format)
 *
 *****************************************************************************/
void draw_rectangle(draw_lcd_t* lcd, uint16_t x0, uint16_t y0, uint16_t x1,
    uint16_t y1, uint16_t color)
{
  horizontalLine(lcd, x0, y0, x1, color);
  horizontalLine(lcd, x0, y1, x1, color);
  verticalLine(lcd, x0, y0, y1, color);
  verticalLine(lcd, x1, y0, y1, color);
}

/******************************************************************************
 *
 * Description:
 *   Fill a rectangle with specified color at specified location
 *
 * Params:
 *   [in] x0 - starting x coordinate
 *   [in] y0 - starting y coordinate
 *   [in] x1 - ending x coordinate
 *   [in] y1 - ending y coordinate
 *   [in] color - 16-bit color (5:6:5 format)
 *
 *****************************************************************************/
void draw_fillRectangle(draw_lcd_t* lcd, uint16_t x0, uint16_t y0, uint16_t x1,
    uint16_t y1, uint16_t color)
{
  uint32_t numPix = 0;

  if (y0 == y1) {
    horizontalLine(lcd, x0, y0, x1, color);
    return;
  }

  if (x0 == x1) {
    verticalLine(lcd, x0, y0, y1, color);
    return;
  }

  if (lcd->winBound != NULL && lcd->winPixel != NULL) {
    lcd->winBound(x0, y0, x1-x0+1, y1-y0+1);
    numPix = (x1-x0+1) * (y1-y0+1);
    while(numPix > 0) {
      lcd->winPixel(color);
      numPix--;
    }
  }

  while (y0 <= y1) {
    horizontalLine(lcd, x0, y0, x1, color);
    y0++;
  }

}

/******************************************************************************
 *
 * Description:
 *   Draw a circle with x0 and y0 as center point.
 *
 * Params:
 *   [in] x0 - x coordinate for center point
 *   [in] y0 - y coordinate for center point
 *   [in] r - circle radius
 *   [in] color - 16-bit color (5:6:5 format)
 *
 *****************************************************************************/
void draw_circle(draw_lcd_t* lcd, uint16_t x0, uint16_t y0, uint16_t r,
    uint16_t color)
{
  int16_t d_x0, d_y0;
  int16_t d_x1, d_y1;
  int16_t d_x2, d_y2;
  int16_t d_x3, d_y3;
  int16_t d_x4, d_y4;
  int16_t d_x5, d_y5;
  int16_t d_x6, d_y6;
  int16_t d_x7, d_y7;
  int16_t xx, yy;
  int16_t di;

  // no radius
  if (r == 0) {
    return;
  }

  d_x0 = d_x1 = x0;
  d_y0 = d_y1 = y0 + r;

  if (d_y0 < lcd->height) {
    lcd->pixel(d_x0, d_y0, color);
  }

  d_x2 = d_x3 = x0;
  d_y2 = d_y3 = y0 - r;

  if (d_y2 >= 0) {
    lcd->pixel(d_x2, d_y2, color);
  }

  d_x4 = d_x6 = x0 + r;
  d_y4 = d_y6 = y0;

  if (d_x4 < lcd->width) {
    lcd->pixel(d_x4, d_y4, color);
  }

  d_x5 = d_x7 = x0 - r;
  d_y5 = d_y7 = y0;

  if (d_x5 >= 0) {
    lcd->pixel(d_x5, d_y5, color);
  }

  if (r == 1) {
    return;
  }

  di = 3 - 2 * r;
  xx = 0;
  yy = r;

  while (xx < yy) {

    if (di < 0) {
      di += 4 * xx + 6;
    }
    else {
      di += 4 * (xx - yy) + 10;
      yy--;
      d_y0--;
      d_y1--;
      d_y2++;
      d_y3++;
      d_x4--;
      d_x5++;
      d_x6--;
      d_x7++;
    }

    xx++;
    d_x0++;
    d_x1--;
    d_x2++;
    d_x3--;
    d_y4++;
    d_y5++;
    d_y6--;
    d_y7--;

    if (d_x0 <= lcd->width && d_y0 >= 0) {
      lcd->pixel(d_x0, d_y0, color);
    }

    if (d_x1 >= 0 && d_y1 >= 0) {
      lcd->pixel(d_x1, d_y1, color);
    }

    if (d_x2 <= lcd->width && d_y2 <= lcd->height) {
      lcd->pixel(d_x2, d_y2, color);
    }

    if (d_x3 >= 0 && d_y3 <= lcd->height) {
      lcd->pixel(d_x3, d_y3, color);
    }

    if (d_x4 <= lcd->height && d_y4 >= 0) {
      lcd->pixel(d_x4, d_y4, color);
    }

    if (d_x5 >= 0 && d_y5 >= 0) {
      lcd->pixel(d_x5, d_y5, color);
    }

    if (d_x6 <= lcd->width && d_y6 <= lcd->height) {
      lcd->pixel(d_x6, d_y6, color);
    }

    if (d_x7 >= 0 && d_y7 <= lcd->height) {
      lcd->pixel(d_x7, d_y7, color);
    }
  }

}

/******************************************************************************
 *
 * Description:
 *   Fill a circle with x0 and y0 as center point.
 *
 * Params:
 *   [in] x0 - x coordinate for center point
 *   [in] y0 - y coordinate for center point
 *   [in] r - circle radius
 *   [in] color - 16-bit color (5:6:5 format)
 *
 *****************************************************************************/
void draw_fillCircle(draw_lcd_t* lcd, uint16_t x0, uint16_t y0, uint16_t r,
    uint16_t color)
{
  int16_t d_x0, d_y0;
  int16_t d_x1, d_y1;
  int16_t d_x2, d_y2;
  int16_t d_x3, d_y3;
  int16_t d_x4, d_y4;
  int16_t d_x5, d_y5;
  int16_t d_x6, d_y6;
  int16_t d_x7, d_y7;
  int16_t fill_x0, fill_y0;
  int16_t fill_x1;
  int16_t xx, yy;
  int16_t di;

  if (r == 0) {
    return;
  }

  d_x0 = d_x1 = x0;
  d_y0 = d_y1 = y0 + r;

  if (d_y0 < lcd->height) {
    lcd->pixel(d_x0, d_y0, color);
  }

  d_x2 = d_x3 = x0;
  d_y2 = d_y3 = y0 - r;

  if (d_y2 >= 0) {
    lcd->pixel(d_x2, d_y2, color);
  }

  d_x4 = d_x6 = x0 + r;
  d_y4 = d_y6 = y0;

  if (d_x4 < lcd->width) {
    lcd->pixel(d_x4, d_y4, color);
    fill_x1 = d_x4;
  }
  else {
    fill_x1 = lcd->width;
  }

  fill_y0 = y0;
  fill_x0 = x0 - r;
  if (fill_x0 < 0) {
    fill_x0 = 0;
  }
  horizontalLine(lcd, fill_x0, fill_y0, fill_x1, color);

  d_x5 = d_x7 = x0 - r;
  d_y5 = d_y7 = y0;

  if (d_x5 >= 0) {
    lcd->pixel(d_x5, d_y5, color);
  }

  if (r == 1) {
    return;
  }

  di = 3 - 2 * r;
  xx = 0;
  yy = r;

  while (xx < yy) {

    if (di < 0) {
      di += 4 * xx + 6;
    }
    else {
      di += 4 * (xx - yy) + 10;
      yy--;
      d_y0--;
      d_y1--;
      d_y2++;
      d_y3++;
      d_x4--;
      d_x5++;
      d_x6--;
      d_x7++;
    }

    xx++;
    d_x0++;
    d_x1--;
    d_x2++;
    d_x3--;
    d_y4++;
    d_y5++;
    d_y6--;
    d_y7--;

    if (d_x0 <= lcd->width && d_y0 >= 0) {
      lcd->pixel(d_x0, d_y0, color);
    }

    if (d_x1 >= 0 && d_y1 >= 0) {
      lcd->pixel(d_x1, d_y1, color);
    }

    if (d_x1 >= 0) {
      fill_x0 = d_x1;
      fill_y0 = d_y1;

      if (fill_y0 > lcd->height) {
        fill_y0 = lcd->height;
      }
      if (fill_y0 < 0) {
        fill_y0 = 0;
      }
      fill_x1 = x0 * 2 - d_x1;
      if (fill_x1 > lcd->width) {
        fill_x1 = lcd->width;
      }

      horizontalLine(lcd, fill_x0, fill_y0, fill_x1, color);
    }

    if (d_x2 <= lcd->width && d_y2 <= lcd->height) {
      lcd->pixel(d_x2, d_y2, color);
    }

    if (d_x3 >= 0 && d_y3 <= lcd->height) {
      lcd->pixel(d_x3, d_y3, color);
    }

    if (d_x3 >= 0) {
      fill_x0 = d_x3;
      fill_y0 = d_y3;

      if (fill_y0 > lcd->height) {
        fill_y0 = lcd->height;
      }
      if (fill_y0 < 0) {
        fill_y0 = 0;
      }
      fill_x1 = x0 * 2 - d_x3;
      if (fill_x1 > lcd->width) {
        fill_x1 = lcd->width;
      }

      horizontalLine(lcd, fill_x0, fill_y0, fill_x1, color);
    }

    if (d_x4 <= lcd->height && d_y4 >= 0) {
      lcd->pixel(d_x4, d_y4, color);
    }

    if (d_x5 >= 0 && d_y5 >= 0) {
      lcd->pixel(d_x5, d_y5, color);
    }

    if (d_x5 >= 0) {
      fill_x0 = d_x5;
      fill_y0 = d_y5;

      if (fill_y0 > lcd->height) {
        fill_y0 = lcd->height;
      }
      if (fill_y0 < 0) {
        fill_y0 = 0;
      }
      fill_x1 = x0 * 2 - d_x5;
      if (fill_x1 > lcd->width) {
        fill_x1 = lcd->width;
      }

      horizontalLine(lcd, fill_x0, fill_y0, fill_x1, color);
    }


    if (d_x6 <= lcd->width && d_y6 <= lcd->height) {
      lcd->pixel(d_x6, d_y6, color);
    }

    if (d_x7 >= 0 && d_y7 <= lcd->height) {
      lcd->pixel(d_x7, d_y7, color);
    }

    if (d_x7 >= 0) {
      fill_x0 = d_x7;
      fill_y0 = d_y7;

      if (fill_y0 > lcd->height) {
        fill_y0 = lcd->height;
      }
      if (fill_y0 < 0) {
        fill_y0 = 0;
      }
      fill_x1 = x0 * 2 - d_x7;
      if (fill_x1 > lcd->width) {
        fill_x1 = lcd->width;
      }

      horizontalLine(lcd, fill_x0, fill_y0, fill_x1, color);
    }

  }

}

/******************************************************************************
 *
 * Description:
 *   Draw an image created with Embedded Artists image converter (img_conv)
 *   utility.
 *
 * Params:
 *   [in] x0 - x coordinate for upper left corner
 *   [in] y0 - y coordinate for upper left corner
 *   [in] img - image data
 *
 *****************************************************************************/
void draw_eaImg(draw_lcd_t* lcd, uint16_t x0, uint16_t y0, uint16_t* img)
{
  int i = 0;
  uint16_t y, x;
  uint16_t width  = img[0];
  uint16_t height = img[1];
  uint16_t comp   = img[2];
  uint16_t esc    = img[3];
  uint32_t len = 0;
  uint32_t n = 0;

  // image doesn't fit on display
  if (x0 + width > lcd->width || y0 + height > lcd->height) {
    return;
  }

  if (lcd->winBound != NULL) {
    lcd->winBound(x0, y0, width, height);
  }

  // skip header
  img += 4;

  // compressed image
  if (comp) {

    len = width * height;
    x = x0;
    y = y0;
    while (len > 0) {

      if (*img == esc) {
        img++;
        n = *img++;
        for (i = 0; i < n; i++) {

          if (lcd->winPixel != NULL) {
            lcd->winPixel(*img);
          }
          else {
            lcd->pixel(x, y, *img);
          }

          x++;
          if (x >= width) {
            x = x0;
            y++;
          }

        }
      }
      else {
        n = 1;

        if (lcd->winPixel != NULL) {
          lcd->winPixel(*img);
        }
        else {
          lcd->pixel(x, y, *img);
        }

        x++;
        if (x >= width) {
          x = x0;
          y++;
        }
      }

      img++;
      len -= n;
    }
  }

  else {
    if (lcd->winBound != NULL) {
      lcd->winBound(x0, y0, width, height);
    }

    for (y = 0; y < height; y++) {
      for (x = 0; x < width; x++ ) {
        if (lcd->winPixel != NULL) {
          lcd->winPixel(*img);
        }
        else {
          lcd->pixel(x+x0, y+y0, *img);
        }
        img++;
      }
    }
  }
}


