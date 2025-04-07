The displays use a library called TFT_eSPI. 

This library needs to be configured in the User_Setup_Select.h file for the type of display and the pins used.

Make sure this line is commented out in the setup file 

#include <../Setup453_RP2040_ST7789_240x320.h>  // RP2040, ST7789

Then place the Setup453_RP2040_ST7789_240x320.h in your library directory and edit it for the pins used.

//Pins RP2040

#define TFT_BL     -1 // LED back-light  // 26

#define TFT_MISO   -1   // Not connected

#define TFT_MOSI   3

#define TFT_SCLK   2

#define TFT_CS     5

#define TFT_DC     15

//#define TFT_RST    -1   // Set TFT_RST to -1 if display RESET is connected to RP2040 board RUN

#define TFT_RST    27   // For overclocking, RESET can be connected to pin 27  


