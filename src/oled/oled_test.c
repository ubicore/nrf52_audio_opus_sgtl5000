/*
 * oled_test.c
 *
 *  Created on: 11 févr. 2019
 *      Author: nlantz
 */

#include <stdint.h>
#include "draw.h"
#include "UG2828.h"
#include "img_logo.h"
#include "ssd1351.h"
#include "draw_fonts.h"
#include "lpc_helvr10.h"
#include "oled_spi.h"


/******************************************************************************
 * Local Variables
 *****************************************************************************/

static draw_lcd_t lcd;

/******************************************************************************
 * Local Functions
 *****************************************************************************/

static void rainbow(draw_lcd_t *lcd)
{
  // White => 0~15
  draw_fillRectangle(lcd, 0, 0, 15, 127, 0xffff);

  // Yellow => 16~31
  draw_fillRectangle(lcd, 16, 0, 31, 127, 0xffe0);

  // Purple => 32~47
  draw_fillRectangle(lcd, 32, 0, 47, 127, 0xf81f);

  // Cyan => 48~63
  draw_fillRectangle(lcd, 48, 0, 63, 127, 0x07ff);

  // Red => 64~79
  draw_fillRectangle(lcd, 64, 0, 79, 127, 0xF800);

  // Green => 80~95
  draw_fillRectangle(lcd, 80, 0, 95, 127, 0x07E0);

  // Blue => 96~111
  draw_fillRectangle(lcd, 96, 0, 111, 127, 0x001f);

  // Black => 112~127
  draw_fillRectangle(lcd, 112, 0, 127, 127, 0x0000);
}

/******************************************************************************
 * Main method
 *****************************************************************************/

#include <errno.h>
#include <zephyr.h>
#include <device.h>
#include "nrf_gpio.h"



int oled_test (void)
{
  const char name1[]="Embedded";
  const char name2[]="Artists AB";
  const char info[]="EmbeddedArtists.com";
  //char data[1];

  SPI_PIN_CFG_OUTPUT(OLED_RESET)
  SPI_PIN_CLEAR(OLED_RESET)

  oled_spi_init();

  k_sleep(K_MSEC(100));

  SPI_PIN_SET(OLED_RESET)

  //bsp_init();
  ug2828_init(&lcd);

  while (1) {
    // draw logo
    draw_eaImg(&lcd, 0, 0, (uint16_t*)&_img_logo[0]);

    k_sleep(K_MSEC(3000));


    // Fade In/Out (Full Screen)
    ssd1351_fadeOut();
    ssd1351_fadeIn();
    ssd1351_fadeOut();
    ssd1351_fadeIn();
    k_sleep(K_MSEC(1000));
#if 1
    // Scrolling (Partial or Full Screen)
    ssd1351_verticalScroll(0x00,0x01,0x40);
    // Upward - Full Screen
    k_sleep(K_MSEC(1000));
    ssd1351_verticalScroll(0x01,0x01,0x40);
    // Downward - Full Screen
    k_sleep(K_MSEC(1000));
    ssd1351_deactivateScroll();
    ssd1351_horizontalScroll(0x00,0x01,0x38,0x48,0x01,0x02);
    k_sleep(K_MSEC(2000));
    // Rightward - Partial Screen
    ssd1351_horizontalScroll(0x01,0x01,0x38,0x48,0x01,0x02);
    k_sleep(K_MSEC(2000));
    // Leftward - Partial Screen
    ssd1351_deactivateScroll();

    // Color Bar (Test Pattern)
    rainbow(&lcd);
    k_sleep(K_MSEC(3000));

    // clear screen
    draw_fillRectangle(&lcd, 0, 0, 127, 127, 0x0000);

    draw_rectangle(&lcd, 0, 0, 127, 127, 0xFFFF);
    k_sleep(K_MSEC(1000));
    draw_rectangle(&lcd, 16, 16, 111, 111, 0xf800);
    k_sleep(K_MSEC(1000));
    draw_rectangle(&lcd, 32, 32, 95, 95, 0x07e0);
    k_sleep(K_MSEC(1000));
    draw_rectangle(&lcd, 48, 48, 79, 79, 0x001f);
    k_sleep(K_MSEC(1000));

    // text
    draw_fillRectangle(&lcd, 16, 16, 111, 111, 0x0000);
    draw_setFont(&font_helvr10);
    draw_putText(&lcd, 10, 10, (const char*)&name1, 0xffff, 0x0000);
    draw_putText(&lcd, 10, 20, (const char*)&name2, 0xffff, 0x0000);
    draw_putText(&lcd, 10, 35, (const char*)&info, 0xffff, 0x0000);
    k_sleep(K_MSEC(3000));

    // clear screen
    draw_fillRectangle(&lcd, 0, 0, 127, 127, 0x0000);
#endif
  }



  return 1;
}
