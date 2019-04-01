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
#include <zephyr.h>

//#include "lpc_types.h"
#include <stdint.h>
#include "ssd1351.h"
//#include "bsp.h"
#include "oled_spi.h"

/******************************************************************************
 * Define and typedefs
 *****************************************************************************/



/******************************************************************************
 * Local Functions
 *****************************************************************************/



/******************************************************************************
 * Public Functions
 *****************************************************************************/

/******************************************************************************
 *
 * Description:
 *   Specifies column start address and end address of the display data RAM.
 *
 * Params:
 *   [in] start - start address
 *   [in] end - end address
 *
 *****************************************************************************/
void ssd1351_setColumnAddress(uint8_t start, uint8_t end)
{
  bsp_ssd1351Command(0x15);  // Set Column Address
  bsp_ssd1351Data(start);    //   Default => 0x00 (Start Address)
  bsp_ssd1351Data(end);      //   Default => 0x7F (End Address)
}

/******************************************************************************
 *
 * Description:
 *   Specifies row start address and end address of the display data RAM.
 *
 * Params:
 *   [in] start - start address
 *   [in] end - end address
 *
 *****************************************************************************/
void ssd1351_setRowAddress(uint8_t start, uint8_t end)
{
  bsp_ssd1351Command(0x75);   // Set Row Address
  bsp_ssd1351Data(start);     //   Default => 0x00 (Start Address)
  bsp_ssd1351Data(end);       //   Default => 0x7F (End Address)
}

/******************************************************************************
 *
 * Description:
 *   Enable data to be written to display RAM. This is cancelled when another
 *   command is written to the display.
 *
 *****************************************************************************/
void ssd1351_setWriteRAM(void)
{
  bsp_ssd1351Command(0x5C);      // Enable MCU to Write into RAM
}

/******************************************************************************
 *
 * Description:
 *   Enable data to be read from display RAM. This is cancelled when another
 *   command is written to the display.
 *
 *****************************************************************************/
void ssd1351_setReadRAM(void)
{
  bsp_ssd1351Command(0x5D);      // Enable MCU to Read from RAM
}

/******************************************************************************
 *
 * Description:
 *   Set Re-map & Dual COM Line Mode
 *
 * Params:
 *   [in] cfg: bit pattern that configures the controller
 *        b[0]: 0=horizontal address increment mode, 1=vertical increment mode
 *        b[1]: 0=RAM 0-127 maps to Col 0-127, 1=RAM 0-127 maps to Col 127-0
 *        b[2]: 0=color seq A->B->C, 1=color seq C->B->A
 *        b[4]: 0=Scan up->down, 1=Scan bottom->up
 *        b[5]: Odd even split of COM pins
 *        b[7:6]: Select either 262k, 65k or 256 color mode
 *
 *****************************************************************************/
void ssd1351_setRemapFormat(uint8_t cfg)
{
  bsp_ssd1351Command(0xA0);      // Set Re-Map / Color Depth
  bsp_ssd1351Data(cfg);        //   Default => 0x40
  //     Horizontal Address Increment
  //     Column Address 0 Mapped to SEG0
  //     Color Sequence: A => B => C
  //     Scan from COM0 to COM[N-1]
  //     Disable COM Split Odd Even
  //     65,536 Colors
}

/******************************************************************************
 *
 * Description:
 *   Set display start line register to determine starting address of display
 *   RAM.
 *
 *****************************************************************************/
void ssd1351_setStartLine(uint8_t d)
{
  bsp_ssd1351Command(0xA1);  // Set Vertical Scroll by RAM
  bsp_ssd1351Data(d);        //   Default => 0x00
}

/******************************************************************************
 *
 * Description:
 *   Specifies the mapping of display start line to one of COM0-127
 *
 *****************************************************************************/
void ssd1351_setDisplayOffset(uint8_t d)
{
  bsp_ssd1351Command(0xA2);  // Set Vertical Scroll by Row
  bsp_ssd1351Data(d);        //   Default => 0x60
}

/******************************************************************************
 *
 * Description:
 *   Set display mode
 *
 * Params:
 *   [in]: mode - 0: Display Off
 *                1: Display on, all pixels gray scale "GS63".
 *                2: Normal display
 *                3: Inverse display
 *
 *****************************************************************************/
void ssd1351_setDisplayMode(uint8_t mode)
{
  bsp_ssd1351Command(0xA4|mode);      // Set Display Mode
  //   Default => 0xA6
  //     0xA4 (0x00) => Entire Display Off, All Pixels Turn Off
  //     0xA5 (0x01) => Entire Display On, All Pixels Turn On at GS Level 63
  //     0xA6 (0x02) => Normal Display
  //     0xA7 (0x03) => Inverse Display
}

/******************************************************************************
 *
 * Description:
 *   Enable or disable the VDD regulator
 *
 * Params:
 *   [in]: enable - 1 to enable, 0 to disable
 *
 *****************************************************************************/
void ssd1351_setFunctionSelection(uint8_t enable)
{
  bsp_ssd1351Command(0xAB);      // Function Selection
  bsp_ssd1351Data(enable);        //   Default => 0x01
  //     Enable Internal VDD Regulator
  //     Select 8-bit Parallel Interface
}

/******************************************************************************
 *
 * Description:
 *   Turn OLED panel On/Off
 *
 * Params:
 *   [in]: on - 1 = on, 0 = off
 *
 *****************************************************************************/
void ssd1351_setDisplayOnOff(uint8_t on)
{
  bsp_ssd1351Command(0xAE|on);      // Set Display On/Off
  //   Default => 0xAE
  //     0xAE (0x00) => Display Off (Sleep Mode On)
  //     0xAF (0x01) => Display On (Sleep Mode Off)
}

/******************************************************************************
 *
 * Description:
 *   Set the length of phase 1 and 2 of segment waveform of the driver.
 *
 * Params:
 *   [in]: d - b[3:0] Phase 1: Set the period from 5 to 31 in the unit of 2 DCLKs
 *             b[7:4] Pahse 2: Set the period from 3 to 15 in th eunit of DCLKs
 *
 *****************************************************************************/
void ssd1351_setPhaseLength(uint8_t d)
{
  bsp_ssd1351Command(0xB1);      // Phase 1 (Reset) & Phase 2 (Pre-Charge) Period Adjustment
  bsp_ssd1351Data(d);        //   Default => 0x82 (8 Display Clocks [Phase 2] / 5 Display Clocks [Phase 1])
  //     D[3:0] => Phase 1 Period in 5~31 Display Clocks
  //     D[7:4] => Phase 2 Period in 3~15 Display Clocks
}

/******************************************************************************
 *
 * Description:
 *   Set display enhancement.
 *
 *****************************************************************************/
void ssd1351_setDisplayEnhancement(uint8_t d)
{
  bsp_ssd1351Command(0xB2);      // Display Enhancement
  bsp_ssd1351Data(d);        //   Default => 0x00 (Normal)
  bsp_ssd1351Data(0x00);
  bsp_ssd1351Data(0x00);
}

/******************************************************************************
 *
 * Description:
 *   Set front clock divider / oscillator frequency
 *
 * Params:
 *   [in]: d - b[3:0] clock divide ratio
 *             b[7:4] oscillator frequency
 *
 *****************************************************************************/
void ssd1351_setDisplayClock(uint8_t d)
{
  bsp_ssd1351Command(0xB3);      // Set Display Clock Divider / Oscillator Frequency
  bsp_ssd1351Data(d);        //   Default => 0x00
  //     A[3:0] => Display Clock Divider
  //     A[7:4] => Oscillator Frequency
}

/******************************************************************************
 *
 * Description:
 *   Set VSL
 *
 *****************************************************************************/
void ssd1351_setVSL(uint8_t d)
{
  bsp_ssd1351Command(0xB4);      // Set Segment Low Voltage
  bsp_ssd1351Data(0xA0|d);     //   Default => 0xA0
  //     0xA0 (0x00) => Enable External VSL
  //     0xA2 (0x02) => Enable Internal VSL (Kept VSL Pin N.C.)
  bsp_ssd1351Data(0xB5);
  bsp_ssd1351Data(0x55);
}

/******************************************************************************
 *
 * Description:
 *   Set states if GPIO0 and GPIO1 pins
 *
 *****************************************************************************/
void ssd1351_setGPIO(uint8_t d)
{
  bsp_ssd1351Command(0xB5);      // General Purpose IO
  bsp_ssd1351Data(d);        //   Default => 0x0A (GPIO Pins output Low Level.)
}

/******************************************************************************
 *
 * Description:
 *   Set the phase 3 second pre-charge period
 *
 *****************************************************************************/
void ssd1351_setPrechargePeriod(uint8_t d)
{
  bsp_ssd1351Command(0xB6);      // Set Second Pre-Charge Period
  bsp_ssd1351Data(d);        //   Default => 0x08 (8 Display Clocks)
}

/******************************************************************************
 *
 * Description:
 *   Set look up table for gray scale pulse width
 *
 *****************************************************************************/
void ssd1351_setGrayScaleTable(void)
{
  bsp_ssd1351Command(0xB8);
  bsp_ssd1351Data(0x02);     // Gray Scale Level 1
  bsp_ssd1351Data(0x03);     // Gray Scale Level 2
  bsp_ssd1351Data(0x04);     // Gray Scale Level 3
  bsp_ssd1351Data(0x05);     // Gray Scale Level 4
  bsp_ssd1351Data(0x06);     // Gray Scale Level 5
  bsp_ssd1351Data(0x07);     // Gray Scale Level 6
  bsp_ssd1351Data(0x08);     // Gray Scale Level 7
  bsp_ssd1351Data(0x09);     // Gray Scale Level 8
  bsp_ssd1351Data(0x0A);     // Gray Scale Level 9
  bsp_ssd1351Data(0x0B);     // Gray Scale Level 10
  bsp_ssd1351Data(0x0C);     // Gray Scale Level 11
  bsp_ssd1351Data(0x0D);     // Gray Scale Level 12
  bsp_ssd1351Data(0x0E);     // Gray Scale Level 13
  bsp_ssd1351Data(0x0F);     // Gray Scale Level 14
  bsp_ssd1351Data(0x10);     // Gray Scale Level 15
  bsp_ssd1351Data(0x11);     // Gray Scale Level 16
  bsp_ssd1351Data(0x12);     // Gray Scale Level 17
  bsp_ssd1351Data(0x13);     // Gray Scale Level 18
  bsp_ssd1351Data(0x15);     // Gray Scale Level 19
  bsp_ssd1351Data(0x17);     // Gray Scale Level 20
  bsp_ssd1351Data(0x19);     // Gray Scale Level 21
  bsp_ssd1351Data(0x1B);     // Gray Scale Level 22
  bsp_ssd1351Data(0x1D);     // Gray Scale Level 23
  bsp_ssd1351Data(0x1F);     // Gray Scale Level 24
  bsp_ssd1351Data(0x21);     // Gray Scale Level 25
  bsp_ssd1351Data(0x23);     // Gray Scale Level 26
  bsp_ssd1351Data(0x25);     // Gray Scale Level 27
  bsp_ssd1351Data(0x27);     // Gray Scale Level 28
  bsp_ssd1351Data(0x2A);     // Gray Scale Level 29
  bsp_ssd1351Data(0x2D);     // Gray Scale Level 30
  bsp_ssd1351Data(0x30);     // Gray Scale Level 31
  bsp_ssd1351Data(0x33);     // Gray Scale Level 32
  bsp_ssd1351Data(0x36);     // Gray Scale Level 33
  bsp_ssd1351Data(0x39);     // Gray Scale Level 34
  bsp_ssd1351Data(0x3C);     // Gray Scale Level 35
  bsp_ssd1351Data(0x3F);     // Gray Scale Level 36
  bsp_ssd1351Data(0x42);     // Gray Scale Level 37
  bsp_ssd1351Data(0x45);     // Gray Scale Level 38
  bsp_ssd1351Data(0x48);     // Gray Scale Level 39
  bsp_ssd1351Data(0x4C);     // Gray Scale Level 40
  bsp_ssd1351Data(0x50);     // Gray Scale Level 41
  bsp_ssd1351Data(0x54);     // Gray Scale Level 42
  bsp_ssd1351Data(0x58);     // Gray Scale Level 43
  bsp_ssd1351Data(0x5C);     // Gray Scale Level 44
  bsp_ssd1351Data(0x60);     // Gray Scale Level 45
  bsp_ssd1351Data(0x64);     // Gray Scale Level 46
  bsp_ssd1351Data(0x68);     // Gray Scale Level 47
  bsp_ssd1351Data(0x6C);     // Gray Scale Level 48
  bsp_ssd1351Data(0x70);     // Gray Scale Level 49
  bsp_ssd1351Data(0x74);     // Gray Scale Level 50
  bsp_ssd1351Data(0x78);     // Gray Scale Level 51
  bsp_ssd1351Data(0x7D);     // Gray Scale Level 52
  bsp_ssd1351Data(0x82);     // Gray Scale Level 53
  bsp_ssd1351Data(0x87);     // Gray Scale Level 54
  bsp_ssd1351Data(0x8C);     // Gray Scale Level 55
  bsp_ssd1351Data(0x91);     // Gray Scale Level 56
  bsp_ssd1351Data(0x96);     // Gray Scale Level 57
  bsp_ssd1351Data(0x9B);     // Gray Scale Level 58
  bsp_ssd1351Data(0xA0);     // Gray Scale Level 59
  bsp_ssd1351Data(0xA5);     // Gray Scale Level 60
  bsp_ssd1351Data(0xAA);     // Gray Scale Level 61
  bsp_ssd1351Data(0xAF);     // Gray Scale Level 62
  bsp_ssd1351Data(0xB4);     // Gray Scale Level 63
}

/******************************************************************************
 *
 * Description:
 *   Use built-in Linear LUT
 *
 *****************************************************************************/
void ssd1351_setLinearGrayScaleTable()
{
  bsp_ssd1351Command(0xB9);      // Default
}

/******************************************************************************
 *
 * Description:
 *   Set the first pre-charge voltage (phase 2) level of segment pins
 *
 *****************************************************************************/
void ssd1351_setPrechargeVoltage(uint8_t d)
{
  bsp_ssd1351Command(0xBB);      // Set Pre-Charge Voltage Level
  bsp_ssd1351Data(d);        //   Default => 0x17 (0.50*VCC)
}

/******************************************************************************
 *
 * Description:
 *   Set high voltage level of common pins (VCOMH)
 *
 *****************************************************************************/
void ssd1351_setVCOMH(uint8_t d)
{
  bsp_ssd1351Command(0xBE);      // Set COM Deselect Voltage Level
  bsp_ssd1351Data(d);        //   Default => 0x05 (0.82*VCC)
}

/******************************************************************************
 *
 * Description:
 *   Set contrast setting of the display
 *
 *****************************************************************************/
void ssd1351_setContrastColor(uint8_t a, uint8_t b, uint8_t c)
{
  bsp_ssd1351Command(0xC1);      // Set Contrast Current for Color A, B, C
  bsp_ssd1351Data(a);        //   Default => 0x8A (Color A)
  bsp_ssd1351Data(b);        //   Default => 0x51 (Color B)
  bsp_ssd1351Data(c);        //   Default => 0x8A (Color C)
}

/******************************************************************************
 *
 * Description:
 *   Control the segment output current by a scaling factor
 *
 *****************************************************************************/
void ssd1351_setMasterCurrent(uint8_t d)
{
  bsp_ssd1351Command(0xC7);      // Master Contrast Current Control
  bsp_ssd1351Data(d);        //   Default => 0x0F (Maximum)
}

/******************************************************************************
 *
 * Description:
 *   Switches default 1:128 multiplex mode to any multiplex mode from 16 to
 *   128. For example, when multiplex ration is set to 16, only 16 common pins
 *   are enabled.
 *
 *****************************************************************************/
void ssd1351_setMultiplexRatio(uint8_t d)
{
  bsp_ssd1351Command(0xCA);      // Set Multiplex Ratio
  bsp_ssd1351Data(d);        //   Default => 0x7F (1/128 Duty)
}

/******************************************************************************
 *
 * Description:
 *   Lock the OLED driver IC from accepting any command. Call this function
 *   again to unlock the OLED driver IC.
 *
 *****************************************************************************/
void ssd1351_setCommandLock(unsigned char d)
{
  bsp_ssd1351Command(0xFD);      // Set Command Lock
  bsp_ssd1351Data(d);        //   Default => 0x12
  //     0x12 => Driver IC interface is unlocked from entering command.
  //     0x16 => All Commands are locked except 0xFD.
  //     0xB0 => Command 0xA2, 0xB1, 0xB3, 0xBB & 0xBE are inaccessible.
  //     0xB1 => All Commands are accessible.
}

/******************************************************************************
 *
 * Description:
 *   Vertical scroll of the display.
 *
 * Params:
 *   [in] dir - direction. 0=Up, 1=down
 *   [in] rows - number of row scroll per step
 *   [in] interval - time interval between each scroll step
 *
 *****************************************************************************/
void ssd1351_verticalScroll(uint8_t dir, uint8_t rows, uint8_t interval)
{
  unsigned int i,j;

  switch(dir)
  {
  case 0:
    for(i=0;i<128;i+=rows)
    {
      ssd1351_setStartLine(i);
      for(j=0;j<interval;j++)
      {
        k_sleep(K_MSEC(1));

      }
    }
    break;
  case 1:
    for(i=0;i<128;i+=rows)
    {
      ssd1351_setStartLine(128-i);
      for(j=0;j<interval;j++)
      {
          k_sleep(K_MSEC(1));
      }
    }
    break;
  }
  ssd1351_setStartLine(0x00);
}

/******************************************************************************
 *
 * Description:
 *   Horizontal scroll of the display.
 *
 * Params:
 *   [in] dir - direction. 0=right, 1=left
 *   [in] cols - number of column scroll per step
 *   [in] rowStart - row address of start
 *   [in] rows - number of rows to be scrolled
 *   [in] interval - time interval between each scroll step in terms of
 *        frame frequency
 *   [in] delay - delay time
 *
 *****************************************************************************/
void ssd1351_horizontalScroll(uint8_t dir, uint8_t cols, uint8_t rowStart,
    uint8_t rows, uint8_t interval, uint8_t delay)
{
  bsp_ssd1351Command(0x96);      // Horizontal Scroll Setup
  bsp_ssd1351Data((dir<<7)|cols);
  bsp_ssd1351Data(rowStart);
  bsp_ssd1351Data(rows);
  bsp_ssd1351Data(0x00);
  bsp_ssd1351Data(interval);
  bsp_ssd1351Command(0x9F);      // Activate Horizontal Scroll
//  bsp_delayUs(delay*600);
  k_sleep(K_MSEC(delay*1));

}


/******************************************************************************
 *
 * Description:
 *   Deactivate scroll.
 *
 *****************************************************************************/
void ssd1351_deactivateScroll(void)
{
  bsp_ssd1351Command(0x9E);      // Deactivate Scrolling
}



/******************************************************************************
 *
 * Description:
 *   Fade In display.
 *
 *****************************************************************************/
void ssd1351_fadeIn(void)
{
  unsigned int i;

  ssd1351_setDisplayOnOff(0x01);
  for(i=0;i<(MAX_MASTER_CURRENT+1);i++)
  {
    ssd1351_setMasterCurrent(i);
    k_sleep(K_MSEC(20));

  }
}


/******************************************************************************
 *
 * Description:
 *   Fade Out display.
 *
 *****************************************************************************/
void ssd1351_fadeOut(void)
{
  unsigned int i;

  for(i=(MAX_MASTER_CURRENT+1);i>0;i--)
  {
    ssd1351_setMasterCurrent(i-1);
    k_sleep(K_MSEC(20));
  }
  ssd1351_setDisplayOnOff(0x00);
}

/******************************************************************************
 *
 * Description:
 *   Set sleep mode for display.
 *
 * Params:
 *   [in] mode - 0: enter sleep mode, 1: exit sleep mode
 *
 *****************************************************************************/
void ssd1351_sleep(uint8_t mode)
{
  switch(mode)
  {
  case 0:
    ssd1351_setDisplayOnOff(0x00);
    ssd1351_setDisplayMode(0x01);
    break;
  case 1:
    ssd1351_setDisplayMode(0x02);
    ssd1351_setDisplayOnOff(0x01);
    break;
  }
}

