#include "SPI.h"
#include "TFT_eSPI.h"
#include <MIDI.h>
#include "Constants.h"
#include "Parameters.h"
#include "MidiCC.h"
#include "ScreenParams.h"

#include "Hardware.h"

#define MIDI_CHANNEL 1

// Use hardware SPI
TFT_eSPI tft = TFT_eSPI();

unsigned long total = 0;
unsigned long tn = 0;

#define NUM_STEPS 32
#define STEP_HEIGHT 5   // Height of each step in pixels
#define BAR_WIDTH 24    // Width of each bar
#define BAR_SPACING 46  // Distance between the start of each bar

#define DET_STEPS 12        // Number of steps for the detune bar
#define DET_STEP_HEIGHT 13  // Height of each step for the detune bar (210 / 16 ≈ 13)
#define BAR_HEIGHT 160      // Total height for the bar indicators

// Screen dimensions
int screenWidth = 320;
int screenHeight = 140;

//MIDI 5 Pin DIN
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);

int x = 160;
int y = 120;

void setup() {

  tft.init();
  tft.setRotation(1);

  SetupHardware();

  tft.fillScreen(TFT_BLACK);

  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.setHandleControlChange(myConvertControlChange);
  //MIDI.setHandleSystemExclusive(onSysExMessage);
  MIDI.turnThruOn(midi::Thru::Mode::Off);

  renderCurrentPatchPage();
}


int mapValue(int value, int max_value, int scale) {
  return (value * scale) / max_value;
}

void myConvertControlChange(byte channel, byte number, byte value) {

  if (channel == 1) {

    int newvalue = (value << 3);
    myControlChange(channel, number, newvalue);
  }

  if (channel == 2) {

    int newvalue = value;
    myLEDupdate(channel, number, newvalue);
  }
}

void myControlChange(byte channel, byte control, int value) {
  switch (control) {

    case CCKeyTrack:
      panelData[P_keytrack] = value;
      drawBar0(144, panelData[P_keytrack], NUM_STEPS, STEP_HEIGHT);
      break;

    case CCfilterCutoff:
      panelData[P_filterCutoff] = value;
      drawBar0(6, panelData[P_filterCutoff], NUM_STEPS, STEP_HEIGHT);
      break;

    case CCfilterLFO:
      panelData[P_filterLFO] = value;
      drawBar0(192, panelData[P_filterLFO], NUM_STEPS, STEP_HEIGHT);
      break;

    case CCfilterRes:
      panelData[P_filterRes] = value;
      drawBar0(52, panelData[P_filterRes], NUM_STEPS, STEP_HEIGHT);
      break;

    case CCfilterEGlevel:
      panelData[P_filterEGlevel] = value;
      drawBar0(98, panelData[P_filterEGlevel], NUM_STEPS, STEP_HEIGHT);
      break;
  }
}

void myLEDupdate(byte channel, byte control, int value) {
  switch (control) {

    case CCfilterPoleSW:
      if (value == 127) {
        digitalWrite(FILTER_POLE_LED, HIGH);
        panelData[P_filterPoleSW] = 1;
        tft.fillRoundRect(240, 10, 70, 30, 5, TFT_RED);
      } else if (value == 0) {
        digitalWrite(FILTER_POLE_LED, LOW);
        panelData[P_filterPoleSW] = 0;
        tft.fillRoundRect(240, 10, 70, 30, 5, TFT_GREEN);  // Green box for off
      }
      tft.setFreeFont(&FreeSans12pt7b);
      switch (panelData[P_filterPoleSW]) {
        case 1:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(filter01[panelData[P_filterType]])));
          break;
        case 0:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(filter02[panelData[P_filterType]])));
          break;
      }
      // Check if the pointer is valid
      if (str_ptr != nullptr) {
        // Copy the string from program memory to RAM
        strcpy_P(filterDisplay, str_ptr);
      } else {
        // Handle the case where the pointer is NULL (if needed)
      }
      tft.setFreeFont(&FreeSans12pt7b);

      tft.setTextColor(TFT_BLACK);  // Change text color to black for better contrast
      tft.setCursor(260, 33);
      tft.print(panelData[P_filterPoleSW] == 0 ? "Off" : "On");

      // Set range label and value inside a box along the top
      tft.fillRoundRect(10, 10, 200, 30, 5, TFT_YELLOW);  // Background box for range
      tft.setCursor(40, 33);
      tft.setTextColor(TFT_BLACK);
      tft.print(filterDisplay);
      break;

    case CCfilterType:
      panelData[P_filterType] = value;
      switch (panelData[P_filterPoleSW]) {
        case 1:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(filter01[panelData[P_filterType]])));
          break;
        case 0:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(filter02[panelData[P_filterType]])));
          break;
      }
      // Check if the pointer is valid
      if (str_ptr != nullptr) {
        // Copy the string from program memory to RAM
        strcpy_P(filterDisplay, str_ptr);
      } else {
        // Handle the case where the pointer is NULL (if needed)
      }
      tft.setFreeFont(&FreeSans12pt7b);
      if (panelData[P_filterPoleSW] == 0) {
        tft.fillRoundRect(240, 10, 70, 30, 5, TFT_GREEN);  // Green box for off
      } else {
        tft.fillRoundRect(240, 10, 70, 30, 5, TFT_RED);  // Red box for on
      }
      tft.setTextColor(TFT_BLACK);  // Change text color to black for better contrast
      tft.setCursor(260, 33);
      tft.print(panelData[P_filterPoleSW] == 0 ? "Off" : "On");

      // Set range label and value inside a box along the top
      tft.fillRoundRect(10, 10, 200, 30, 5, TFT_YELLOW);  // Background box for range
      tft.setCursor(40, 33);
      tft.setTextColor(TFT_BLACK);
      tft.print(filterDisplay);
      break;

    case CCfilterEGinv:
      if (value == 127) {
        digitalWrite(EG_INVERT_LED, HIGH);
        panelData[P_filterEGinv] = 1;
      } else if (value == 0) {
        digitalWrite(EG_INVERT_LED, LOW);
        panelData[P_filterEGinv] = 0;
      }
      break;
  }
}

// Function to draw the bar
void drawBar0(int x, int value, int steps, int stepHeight) {
  int filledSteps = value / (1023 / steps);  // Calculate the number of steps to fill
  for (int i = 0; i < steps; i++) {
    int y = 210 - (i * stepHeight);
    if (i < filledSteps) {
      tft.fillRoundRect(x, y - stepHeight, BAR_WIDTH, stepHeight - 2, 2, TFT_YELLOW);  // Filled step
    } else {
      tft.fillRoundRect(x, y - stepHeight, BAR_WIDTH, stepHeight - 2, 2, TFT_BLACK);  // Unfilled step
    }
  }
}

void drawPWIndicator0(int x, int value) {
  // Clear the entire bar area by drawing a rectangle with the background color
  tft.fillRoundRect(x, 208 - BAR_HEIGHT, BAR_WIDTH, BAR_HEIGHT, 2, TFT_BLACK);
  tft.drawFastVLine(18, 50, 155, TFT_RED);
  tft.drawFastHLine(6, 50, 24, TFT_RED);
  tft.drawFastHLine(6, 127, 24, TFT_RED);
  tft.drawFastHLine(6, 205, 24, TFT_RED);
  // Calculate the y position for the current value
  int y = 205 - ((value * (BAR_HEIGHT - STEP_HEIGHT)) / 1023);

  // Draw the new bar at the calculated position
  tft.fillRoundRect(x, y - STEP_HEIGHT / 2, BAR_WIDTH, STEP_HEIGHT, 2, TFT_YELLOW);
}

void renderCurrentPatchPage() {
  // ************************ DISPLAY 3 **************************
  tft.fillScreen(TFT_BLACK);  // Fill the screen with black
  tft.setFreeFont(&FreeSans9pt7b);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);

  // Setting text labels at the bottom of each bar
  tft.setCursor(2, 233);
  tft.print("Cut");
  tft.setCursor(48, 233);
  tft.print("Res");
  tft.setCursor(98, 233);
  tft.print("EG");
  tft.setCursor(140, 233);
  tft.print("Key");
  tft.setCursor(190, 233);
  tft.print("TM");

  tft.setFreeFont(&FreeSans12pt7b);
  tft.setTextColor(TFT_BLACK);  // Change text color to black for better contrast
  switch (panelData[P_filterPoleSW]) {
    case 1:
      str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(filter01[panelData[P_filterType]])));
      break;
    case 0:
      str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(filter02[panelData[P_filterType]])));
      break;
  }
  // Check if the pointer is valid
  if (str_ptr != nullptr) {
    // Copy the string from program memory to RAM
    strcpy_P(filterDisplay, str_ptr);
  } else {
    // Handle the case where the pointer is NULL (if needed)
  }
  if (panelData[P_filterPoleSW] == 0) {
    tft.fillRoundRect(240, 10, 70, 30, 5, TFT_GREEN);  // Green box for off
  } else {
    tft.fillRoundRect(240, 10, 70, 30, 5, TFT_RED);  // Red box for on
  }
  tft.setCursor(260, 33);
  tft.print(panelData[P_filterPoleSW] == 0 ? "Off" : "On");

  // Set range label and value inside a box along the top
  tft.fillRoundRect(10, 10, 200, 30, 5, TFT_YELLOW);  // Background box for range
  tft.setCursor(40, 33);
  tft.print(filterDisplay);

  // Drawing the bars
  drawBar0(6, panelData[P_filterCutoff], NUM_STEPS, STEP_HEIGHT);
  drawBar0(52, panelData[P_filterRes], NUM_STEPS, STEP_HEIGHT);
  drawBar0(98, panelData[P_filterEGlevel], NUM_STEPS, STEP_HEIGHT);
  drawBar0(144, panelData[P_keytrack], NUM_STEPS, STEP_HEIGHT);
  drawBar0(192, panelData[P_filterLFO], NUM_STEPS, STEP_HEIGHT);
}

void loop(void) {
  MIDI.read(MIDI_CHANNEL_OMNI);
}

void printnice(int32_t v) {
  char str[32] = { 0 };
  sprintf(str, "%d", v);
  for (char *p = (str + strlen(str)) - 3; p > str; p -= 3) {
    memmove(p + 1, p, strlen(p) + 1);
    *p = ',';
  }
  while (strlen(str) < 7) {
    memmove(str + 1, str, strlen(str) + 1);
    *str = ' ';
  }
  tft.println(str);
}

static inline uint32_t micros_start() __attribute__((always_inline));
static inline uint32_t micros_start() {
  uint8_t oms = millis();
  while ((uint8_t)millis() == oms)
    ;
  return micros();
}

uint32_t testHaD() {
  // pseudo-code for cheesy RLE
  // start with color1
  // while more input data remaining
  //  count =  0nnnnnnn = 1 byte or 1nnnnnnn nnnnnnnn 2 bytes (0 - 32767)
  //  repeat color count times
  //  toggle color1/color2
  static const uint8_t HaD_128x160[] PROGMEM = {
    0x85, 0x91, 0x09, 0x4b, 0x09, 0x24, 0x0a, 0x47, 0x09, 0x27, 0x0a, 0x44, 0x0a, 0x29, 0x0a, 0x42,
    0x0a, 0x2b, 0x0a, 0x41, 0x0a, 0x2c, 0x0a, 0x3e, 0x0b, 0x2f, 0x09, 0x3d, 0x09, 0x32, 0x08, 0x3c,
    0x09, 0x33, 0x09, 0x3b, 0x08, 0x33, 0x0a, 0x3a, 0x0a, 0x31, 0x0b, 0x3a, 0x0c, 0x1d, 0x01, 0x10,
    0x0d, 0x39, 0x0c, 0x1d, 0x01, 0x10, 0x0d, 0x39, 0x0d, 0x0f, 0x01, 0x0c, 0x03, 0x0d, 0x0e, 0x39,
    0x0e, 0x0c, 0x03, 0x0c, 0x04, 0x0b, 0x0f, 0x39, 0x0f, 0x0a, 0x04, 0x0c, 0x05, 0x09, 0x10, 0x39,
    0x10, 0x08, 0x05, 0x0c, 0x06, 0x07, 0x11, 0x39, 0x11, 0x06, 0x06, 0x0d, 0x07, 0x04, 0x13, 0x37,
    0x12, 0x05, 0x07, 0x0d, 0x08, 0x02, 0x15, 0x34, 0x15, 0x03, 0x08, 0x0d, 0x20, 0x32, 0x20, 0x0e,
    0x21, 0x31, 0x20, 0x0f, 0x21, 0x2e, 0x22, 0x10, 0x22, 0x2b, 0x22, 0x12, 0x22, 0x12, 0x05, 0x12,
    0x22, 0x14, 0x22, 0x0c, 0x0f, 0x0c, 0x22, 0x16, 0x22, 0x08, 0x15, 0x08, 0x22, 0x18, 0x22, 0x05,
    0x19, 0x05, 0x21, 0x1c, 0x1f, 0x04, 0x1c, 0x05, 0x1f, 0x1f, 0x1c, 0x04, 0x1e, 0x04, 0x1d, 0x2b,
    0x11, 0x04, 0x21, 0x03, 0x12, 0x36, 0x0f, 0x03, 0x24, 0x03, 0x10, 0x38, 0x0d, 0x03, 0x26, 0x03,
    0x0d, 0x3b, 0x0b, 0x03, 0x28, 0x03, 0x0b, 0x3d, 0x0a, 0x03, 0x29, 0x03, 0x09, 0x40, 0x07, 0x03,
    0x2b, 0x03, 0x07, 0x42, 0x05, 0x03, 0x2c, 0x04, 0x04, 0x45, 0x04, 0x03, 0x2d, 0x03, 0x04, 0x46,
    0x02, 0x03, 0x2e, 0x03, 0x03, 0x48, 0x01, 0x03, 0x2f, 0x03, 0x01, 0x4c, 0x31, 0x4e, 0x32, 0x4e,
    0x33, 0x4c, 0x34, 0x4c, 0x34, 0x4c, 0x35, 0x4b, 0x35, 0x4a, 0x0e, 0x03, 0x14, 0x04, 0x0d, 0x4a,
    0x0b, 0x09, 0x0e, 0x0a, 0x0a, 0x4a, 0x0a, 0x0b, 0x0c, 0x0c, 0x09, 0x4a, 0x09, 0x0d, 0x0a, 0x0e,
    0x09, 0x49, 0x08, 0x0f, 0x09, 0x0e, 0x09, 0x49, 0x08, 0x0f, 0x09, 0x0f, 0x08, 0x49, 0x08, 0x0f,
    0x09, 0x0f, 0x08, 0x49, 0x07, 0x0f, 0x0a, 0x0f, 0x08, 0x49, 0x07, 0x0f, 0x0b, 0x0e, 0x08, 0x49,
    0x07, 0x0d, 0x0e, 0x0d, 0x08, 0x49, 0x07, 0x0b, 0x13, 0x0a, 0x08, 0x49, 0x08, 0x07, 0x18, 0x08,
    0x08, 0x49, 0x08, 0x06, 0x1b, 0x06, 0x08, 0x49, 0x09, 0x04, 0x1c, 0x05, 0x08, 0x4a, 0x09, 0x04,
    0x1d, 0x04, 0x08, 0x4a, 0x0a, 0x03, 0x1d, 0x03, 0x09, 0x4b, 0x19, 0x02, 0x1a, 0x4b, 0x19, 0x03,
    0x19, 0x4b, 0x18, 0x04, 0x18, 0x4d, 0x17, 0x05, 0x17, 0x4a, 0x01, 0x02, 0x17, 0x05, 0x16, 0x4a,
    0x02, 0x02, 0x17, 0x05, 0x16, 0x02, 0x03, 0x44, 0x03, 0x03, 0x16, 0x02, 0x01, 0x02, 0x16, 0x02,
    0x03, 0x43, 0x05, 0x03, 0x15, 0x01, 0x03, 0x01, 0x15, 0x03, 0x04, 0x41, 0x06, 0x03, 0x15, 0x01,
    0x03, 0x01, 0x14, 0x03, 0x07, 0x3d, 0x09, 0x03, 0x2d, 0x03, 0x08, 0x3b, 0x0a, 0x04, 0x2b, 0x03,
    0x0a, 0x39, 0x0c, 0x03, 0x2a, 0x04, 0x0b, 0x37, 0x0e, 0x03, 0x28, 0x03, 0x0e, 0x2e, 0x04, 0x03,
    0x10, 0x03, 0x27, 0x03, 0x10, 0x03, 0x03, 0x24, 0x19, 0x03, 0x26, 0x03, 0x1a, 0x1e, 0x1d, 0x03,
    0x24, 0x03, 0x1e, 0x19, 0x20, 0x04, 0x21, 0x03, 0x20, 0x17, 0x22, 0x04, 0x1f, 0x03, 0x22, 0x15,
    0x22, 0x04, 0x21, 0x04, 0x21, 0x13, 0x22, 0x05, 0x15, 0x01, 0x0b, 0x05, 0x21, 0x12, 0x21, 0x06,
    0x15, 0x01, 0x0b, 0x06, 0x21, 0x10, 0x21, 0x07, 0x0a, 0x01, 0x0a, 0x01, 0x0b, 0x07, 0x21, 0x0e,
    0x20, 0x0a, 0x09, 0x02, 0x09, 0x02, 0x09, 0x09, 0x20, 0x0e, 0x08, 0x02, 0x15, 0x0b, 0x08, 0x03,
    0x08, 0x03, 0x08, 0x0b, 0x15, 0x03, 0x08, 0x0d, 0x07, 0x04, 0x13, 0x0d, 0x06, 0x05, 0x06, 0x06,
    0x05, 0x0d, 0x14, 0x04, 0x07, 0x0c, 0x07, 0x06, 0x11, 0x38, 0x12, 0x06, 0x06, 0x0c, 0x06, 0x08,
    0x10, 0x39, 0x10, 0x08, 0x05, 0x0c, 0x04, 0x0b, 0x0f, 0x39, 0x0f, 0x0a, 0x04, 0x0c, 0x03, 0x0d,
    0x0e, 0x39, 0x0e, 0x0c, 0x03, 0x0c, 0x02, 0x0e, 0x0e, 0x39, 0x0d, 0x0f, 0x01, 0x0c, 0x01, 0x10,
    0x0d, 0x39, 0x0d, 0x0f, 0x01, 0x1e, 0x0c, 0x39, 0x0c, 0x30, 0x0a, 0x3a, 0x0a, 0x33, 0x09, 0x3b,
    0x08, 0x34, 0x09, 0x3b, 0x09, 0x32, 0x09, 0x3c, 0x0a, 0x2f, 0x0a, 0x3e, 0x0a, 0x2d, 0x0b, 0x3f,
    0x0a, 0x2b, 0x0b, 0x41, 0x0a, 0x29, 0x0b, 0x43, 0x0a, 0x27, 0x0a, 0x46, 0x0a, 0x25, 0x0a, 0x49,
    0x09, 0x23, 0x08, 0x4e, 0x08, 0x96, 0x12
  };

  tft.fillScreen(TFT_BLACK);

  uint32_t start = micros_start();

  tft.startWrite();

  for (int i = 0; i < 0x10; i++) {
    //tft.setAddrWindow(0, 0, tft.width(), tft.height());
    tft.setAddrWindow(21, 80, 128, 160);  // for Display 170x320

    uint16_t cnt = 0;
    uint16_t color = tft.color565((i << 4) | i, (i << 4) | i, (i << 4) | i);
    uint16_t curcolor = 0;

    const uint8_t *cmp = &HaD_128x160[0];
    tft.startWrite();
    while (cmp < &HaD_128x160[sizeof(HaD_128x160)]) {
      cnt = pgm_read_byte(cmp++);
      if (cnt & 0x80)
        cnt = ((cnt & 0x7f) << 8) | pgm_read_byte(cmp++);

      tft.pushColor(curcolor, cnt);  // PDQ_GFX has count

      curcolor ^= color;
    }
    tft.endWrite();
  }

  tft.endWrite();

  uint32_t t = micros() - start;

  tft.setTextColor(TFT_YELLOW);
  tft.setCursor(0, 225);
  tft.print(F(" http://hackaday.io/"));
  tft.print(F("    Xark"));

  delay(3 * 1000L);

  return t;
}

uint32_t testFillScreen() {
  uint32_t start = micros_start();
  // Shortened this tedious test!
  tft.fillScreen(TFT_WHITE);
  tft.fillScreen(TFT_RED);
  tft.fillScreen(TFT_GREEN);
  tft.fillScreen(TFT_BLUE);
  tft.fillScreen(TFT_BLACK);

  return (micros() - start) / 5;
}

uint32_t testText() {
  tft.fillScreen(TFT_BLACK);
  uint32_t start = micros_start();
  tft.setCursor(0, 0);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  tft.println(F("Hello World!"));
  tft.setTextSize(2);
  tft.setTextColor(tft.color565(0xff, 0x00, 0x00));
  tft.print(F("RED "));
  tft.setTextColor(tft.color565(0x00, 0xff, 0x00));
  tft.print(F("GREEN "));
  tft.setTextColor(tft.color565(0x00, 0x00, 0xff));
  tft.println(F("BLUE"));
  tft.setTextColor(TFT_YELLOW);
  tft.setTextSize(2);
  tft.println(1234.56);
  tft.setTextColor(TFT_RED);
  tft.setTextSize(3);
  tft.println(0xDEADBEEF, HEX);
  tft.println();
  tft.setTextColor(TFT_GREEN);
  tft.setTextSize(5);
  tft.println(F("Groop"));
  tft.setTextSize(2);
  tft.println(F("I implore thee"));
  tft.setTextColor(TFT_GREEN);
  tft.setTextSize(1);
  tft.println(F("my foonting turlingdromes."));
  tft.println(F("And hooptiously drangle me"));
  tft.println(F("with crinkly bindlewurdles,"));
  tft.println(F("Or I will rend thee"));
  tft.println(F("in the gobberwarts"));
  tft.println(F("with my blurglecruncheon,"));
  tft.println(F("see if I don't!"));
  tft.println(F(""));
  tft.println(F(""));
  tft.setTextColor(TFT_MAGENTA);
  tft.setTextSize(5);
  tft.println(F("Woot!"));
  uint32_t t = micros() - start;
  delay(500);
  return t;
}

uint32_t testPixels() {
  int32_t w = tft.width();
  int32_t h = tft.height();

  uint32_t start = micros_start();
  tft.startWrite();
  for (uint16_t y = 0; y < h; y++) {
    for (uint16_t x = 0; x < w; x++) {
      tft.drawPixel(x, y, tft.color565(x << 3, y << 3, x * y));
    }
  }
  tft.endWrite();
  return micros() - start;
}


uint32_t testLines(uint16_t color) {
  uint32_t start, t;
  int32_t x1, y1, x2, y2;
  int32_t w = tft.width();
  int32_t h = tft.height();

  tft.fillScreen(TFT_BLACK);

  x1 = y1 = 0;
  y2 = h - 1;

  start = micros_start();

  for (x2 = 0; x2 < w; x2 += 6) {
    tft.drawLine(x1, y1, x2, y2, color);
  }

  x2 = w - 1;

  for (y2 = 0; y2 < h; y2 += 6) {
    tft.drawLine(x1, y1, x2, y2, color);
  }

  t = micros() - start;  // fillScreen doesn't count against timing

  tft.fillScreen(TFT_BLACK);

  x1 = w - 1;
  y1 = 0;
  y2 = h - 1;

  start = micros_start();

  for (x2 = 0; x2 < w; x2 += 6) {
    tft.drawLine(x1, y1, x2, y2, color);
  }

  x2 = 0;
  for (y2 = 0; y2 < h; y2 += 6) {
    tft.drawLine(x1, y1, x2, y2, color);
  }

  t += micros() - start;

  tft.fillScreen(TFT_BLACK);

  x1 = 0;
  y1 = h - 1;
  y2 = 0;

  start = micros_start();

  for (x2 = 0; x2 < w; x2 += 6) {
    tft.drawLine(x1, y1, x2, y2, color);
  }
  x2 = w - 1;
  for (y2 = 0; y2 < h; y2 += 6) {
    tft.drawLine(x1, y1, x2, y2, color);
  }
  t += micros() - start;

  tft.fillScreen(TFT_BLACK);

  x1 = w - 1;
  y1 = h - 1;
  y2 = 0;

  start = micros_start();

  for (x2 = 0; x2 < w; x2 += 6) {
    tft.drawLine(x1, y1, x2, y2, color);
  }

  x2 = 0;
  for (y2 = 0; y2 < h; y2 += 6) {
    tft.drawLine(x1, y1, x2, y2, color);
  }

  t += micros() - start;

  return t;
}

uint32_t testFastLines(uint16_t color1, uint16_t color2) {
  uint32_t start;
  int32_t x, y;
  int32_t w = tft.width();
  int32_t h = tft.height();

  tft.fillScreen(TFT_BLACK);

  start = micros_start();

  for (y = 0; y < h; y += 5)
    tft.drawFastHLine(0, y, w, color1);
  for (x = 0; x < w; x += 5)
    tft.drawFastVLine(x, 0, h, color2);

  return micros() - start;
}

uint32_t testRects(uint16_t color) {
  uint32_t start;
  int32_t n, i, i2;
  int32_t cx = tft.width() / 2;
  int32_t cy = tft.height() / 2;

  tft.fillScreen(TFT_BLACK);
  n = min(tft.width(), tft.height());
  start = micros_start();
  for (i = 2; i < n; i += 6) {
    i2 = i / 2;
    tft.drawRect(cx - i2, cy - i2, i, i, color);
  }

  return micros() - start;
}

uint32_t testFilledRects(uint16_t color1, uint16_t color2) {
  uint32_t start, t = 0;
  int32_t n, i, i2;
  int32_t cx = tft.width() / 2 - 1;
  int32_t cy = tft.height() / 2 - 1;

  tft.fillScreen(TFT_BLACK);
  n = min(tft.width(), tft.height());
  for (i = n; i > 0; i -= 6) {
    i2 = i / 2;

    start = micros_start();

    tft.fillRect(cx - i2, cy - i2, i, i, color1);

    t += micros() - start;

    // Outlines are not included in timing results
    tft.drawRect(cx - i2, cy - i2, i, i, color2);
  }

  return t;
}

uint32_t testFilledCircles(uint8_t radius, uint16_t color) {
  uint32_t start;
  int32_t x, y, w = tft.width(), h = tft.height(), r2 = radius * 2;

  tft.fillScreen(TFT_BLACK);

  start = micros_start();

  for (x = radius; x < w; x += r2) {
    for (y = radius; y < h; y += r2) {
      tft.fillCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

uint32_t testCircles(uint8_t radius, uint16_t color) {
  uint32_t start;
  int32_t x, y, r2 = radius * 2;
  int32_t w = tft.width() + radius;
  int32_t h = tft.height() + radius;

  // Screen is not cleared for this one -- this is
  // intentional and does not affect the reported time.
  start = micros_start();

  for (x = 0; x < w; x += r2) {
    for (y = 0; y < h; y += r2) {
      tft.drawCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

uint32_t testTriangles() {
  uint32_t start;
  int32_t n, i;
  int32_t cx = tft.width() / 2 - 1;
  int32_t cy = tft.height() / 2 - 1;

  tft.fillScreen(TFT_BLACK);
  n = min(cx, cy);

  start = micros_start();

  for (i = 0; i < n; i += 5) {
    tft.drawTriangle(
      cx, cy - i,      // peak
      cx - i, cy + i,  // bottom left
      cx + i, cy + i,  // bottom right
      tft.color565(0, 0, i));
  }

  return micros() - start;
}

uint32_t testFilledTriangles() {
  uint32_t start, t = 0;
  int32_t i;
  int32_t cx = tft.width() / 2 - 1;
  int32_t cy = tft.height() / 2 - 1;

  tft.fillScreen(TFT_BLACK);

  start = micros_start();

  for (i = min(cx, cy); i > 10; i -= 5) {
    start = micros_start();
    tft.fillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
                     tft.color565(0, i, i));
    t += micros() - start;
    tft.drawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
                     tft.color565(i, i, 0));
  }

  return t;
}

uint32_t testRoundRects() {
  uint32_t start;
  int32_t w, i, i2;
  int32_t cx = tft.width() / 2 - 1;
  int32_t cy = tft.height() / 2 - 1;

  tft.fillScreen(TFT_BLACK);

  w = min(tft.width(), tft.height());

  start = micros_start();

  for (i = 0; i < w; i += 6) {
    i2 = i / 2;
    tft.drawRoundRect(cx - i2, cy - i2, i, i, i / 8, tft.color565(i, 0, 0));
  }

  return micros() - start;
}

uint32_t testFilledRoundRects() {
  uint32_t start;
  int32_t i, i2;
  int32_t cx = tft.width() / 2 - 1;
  int32_t cy = tft.height() / 2 - 1;

  tft.fillScreen(TFT_BLACK);

  start = micros_start();

  for (i = min(tft.width(), tft.height()); i > 20; i -= 6) {
    i2 = i / 2;
    tft.fillRoundRect(cx - i2, cy - i2, i, i, i / 8, tft.color565(0, i, 0));
  }

  return micros() - start;
}
