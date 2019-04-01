/**********************************************************************
 * $Id:: lpc_fonts.h 745 2008-05-13 19:59:29Z pdurgesh                 $
 *
 * Project: Fonts selection
 *
 * Description:
 *     This package provides a common font information structure.
 *
 ***********************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
 **********************************************************************/

#ifndef LPC_FONTS_H
#define LPC_FONTS_H

//#include "lpc_types.h"
#include <stdint.h>

#if defined (__cplusplus)
extern "C"
{
#endif

/***********************************************************************
 * Font information structure
 **********************************************************************/

/* Font data structure */
typedef struct
{
  int16_t font_height;
  uint8_t  first_char;
  uint8_t  last_char;
  uint16_t *font_table;
  uint8_t  *font_width_table;
} FONT_T;

#if defined (__cplusplus)
}
#endif /*__cplusplus */

#endif /* LPC_FONTS_H */
