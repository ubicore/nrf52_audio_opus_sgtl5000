/******************************************************************************
 *
 * The functions in this file are based on NXP Semiconductors SWIM library.
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
 ******************************************************************************/


/******************************************************************************
 * Includes
 *****************************************************************************/

//#include "lpc_types.h"
#include <stdint.h>
#include <stdio.h>

#include "draw.h"
#include "lpc_fonts.h"

/******************************************************************************
 * Local Variables
 *****************************************************************************/

static FONT_T* sFont = NULL;

/******************************************************************************
 * Local Functions
 *****************************************************************************/

/******************************************************************************
 * Public Functions
 *****************************************************************************/

/******************************************************************************
 *
 * Description:
 *   Set the font to use
 *
 * Params:
 *   [in] font - the font to use
 *
 *****************************************************************************/
void draw_setFont(const FONT_T* font)
{
  sFont = (FONT_T*)font;
}

/******************************************************************************
 *
 * Description:
 *   Put a single character on the display.
 *
 * Params:
 *   [in] x - x coordinate for the character
 *   [in] y - y coordinate for the character
 *   [in] textchar - the character
 *   [in] fgColor - foreground color
 *   [in] bgColor - background color
 *
 *****************************************************************************/
void draw_putChar(draw_lcd_t* lcd, uint16_t x, uint16_t y,
    const char textchar, uint16_t fgColor, uint16_t bgColor)
{
  int32_t i, j;
  int32_t charindex;
  uint16_t *charfields, chardata;
  uint16_t xx, yy;

  // no font has been selected
  if (sFont == NULL) {
    return;
  }

  // Determine index to character data
  charindex = (int32_t) textchar - (int32_t) sFont->first_char;

  // Will the character fit on the display?
  if ((x + (int32_t) sFont->font_width_table [charindex]) > lcd->width)
  {
    // Will not fit, do a newline
    y = y + sFont->font_height;
  }

  // Determine the start of the bitfields for the character
  charfields = sFont->font_table + (charindex * sFont->font_height);

  // Map character to the display
  for (i = 0; i < (int32_t) sFont->font_height; i++)
  {
    xx = x;
    yy = y+i;

    // Get character line mapping data
    chardata = charfields [i];

    for (j = (int32_t)sFont->font_width_table[charindex]; j > 0; j--)
    {
      if ((chardata & 0x8000) != 0)
      {
        lcd->pixel(xx, yy, fgColor);
      }
      else {
        lcd->pixel(xx, yy, bgColor);
      }

      xx++;

      // Next bit in character line
      chardata = chardata << 1;
    }
  }
}

/******************************************************************************
 *
 * Description:
 *   Put a line of text on the display.
 *
 * Params:
 *   [in] x - x coordinate for the string
 *   [in] y - y coordinate for the string
 *   [in] str - the line of text
 *   [in] fgColor - foreground color
 *   [in] bgColor - background color
 *
 *****************************************************************************/
void draw_putText(draw_lcd_t* lcd, uint16_t x, uint16_t y,
    const char* str, uint16_t fgColor, uint16_t bgColor)
{
  int32_t charindex;

  if (str == NULL) {
    return;
  }

  while (*str != '\0') {
    if (*str == '\n') {
      y = y + sFont->font_height;
    }
    else {
      draw_putChar(lcd, x, y, *str, fgColor, bgColor);
    }

    charindex = (int32_t) *str - (int32_t) sFont->first_char;

    str++;
    x += (int32_t) sFont->font_width_table [charindex];
  }


}


