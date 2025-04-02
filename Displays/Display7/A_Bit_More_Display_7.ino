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

uint16_t getActiveColor() {
  return upperSW ? TFT_CYAN : TFT_YELLOW;
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

    case CCeffect1:
      panelData[P_effectPot1] = value;
      drawBar0(6, panelData[P_effectPot1], NUM_STEPS, STEP_HEIGHT);
      break;

    case CCeffect2:
      panelData[P_effectPot2] = value;
      drawBar0(52, panelData[P_effectPot2], NUM_STEPS, STEP_HEIGHT);
      break;

    case CCeffect3:
      panelData[P_effectPot3] = value;
      drawBar0(98, panelData[P_effectPot3], NUM_STEPS, STEP_HEIGHT);
      break;

    case CCeffectMix:
      panelData[P_effectsMix] = map(value, 0, readRes, 0, readRes);
      drawPWIndicator7(144, panelData[P_effectsMix]);
      break;
  }
}

void myLEDupdate(byte channel, byte control, int value) {
  switch (control) {

    case CCeffectNumSW:
      panelData[P_effectNum] = value;
      switch (panelData[P_effectBank]) {
        case 0:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name01[panelData[P_effectNum]])));
          break;
        case 1:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name11[panelData[P_effectNum]])));
          break;
        case 2:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name21[panelData[P_effectNum]])));
          break;
        case 3:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name31[panelData[P_effectNum]])));
          break;
      }
      // Check if the pointer is valid
      if (str_ptr != nullptr) {
        // Copy the string from program memory to RAM
        strcpy_P(buf1, str_ptr);
      } else {
        // Handle the case where the pointer is NULL (if needed)
      }

      switch (panelData[P_effectBank]) {
        case 0:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name02[panelData[P_effectNum]])));
          break;
        case 1:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name12[panelData[P_effectNum]])));
          break;
        case 2:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name22[panelData[P_effectNum]])));
          break;
        case 3:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name32[panelData[P_effectNum]])));
          break;
      }
      // Check if the pointer is valid
      if (str_ptr != nullptr) {
        // Copy the string from program memory to RAM
        strcpy_P(buf2, str_ptr);
      } else {
        // Handle the case where the pointer is NULL (if needed)
      }

      switch (panelData[P_effectBank]) {
        case 0:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name03[panelData[P_effectNum]])));
          break;
        case 1:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name13[panelData[P_effectNum]])));
          break;
        case 2:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name23[panelData[P_effectNum]])));
          break;
        case 3:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name33[panelData[P_effectNum]])));
          break;
      }

      // Check if the pointer is valid
      if (str_ptr != nullptr) {
        // Copy the string from program memory to RAM
        strcpy_P(buf3, str_ptr);
      } else {
        // Handle the case where the pointer is NULL (if needed)
      }

      switch (panelData[P_effectBank]) {
        case 0:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name04[panelData[P_effectNum]])));
          break;
        case 1:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name14[panelData[P_effectNum]])));
          break;
        case 2:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name24[panelData[P_effectNum]])));
          break;
        case 3:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name34[panelData[P_effectNum]])));
          break;
      }
      // Check if the pointer is valid
      if (str_ptr != nullptr) {
        // Copy the string from program memory to RAM
        strcpy_P(buf4, str_ptr);
      } else {
        // Handle the case where the pointer is NULL (if needed)
      }

      switch (panelData[P_effectBank]) {
        case 0:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name05[panelData[P_effectNum]])));
          break;
        case 1:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name15[panelData[P_effectNum]])));
          break;
        case 2:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name25[panelData[P_effectNum]])));
          break;
        case 3:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name35[panelData[P_effectNum]])));
          break;
      }
      // Check if the pointer is valid
      if (str_ptr != nullptr) {
        // Copy the string from program memory to RAM
        strcpy_P(buf5, str_ptr);
      } else {
        // Handle the case where the pointer is NULL (if needed)
      }

      tft.fillRoundRect(10, 10, 300, 30, 5, getActiveColor());
      tft.setCursor(50, 33);
      tft.print(buf1);
      tft.setCursor(180, 33);
      tft.print(buf2);


      tft.fillRoundRect(180, 50, 130, 30, 5, getActiveColor());  // Green box for off
      tft.setCursor(200, 73);
      tft.print("Bank:");
      tft.setCursor(280, 73);
      tft.print(panelData[P_effectBank] + 1);

      tft.fillRoundRect(180, 90, 130, 30, 5, getActiveColor());  // Green box for off
      tft.setCursor(200, 113);
      tft.print("Prog:");
      tft.setCursor(280, 113);
      tft.print(panelData[P_effectNum] + 1);

      tft.setFreeFont(&FreeSans9pt7b);

      tft.fillRoundRect(180, 130, 130, 30, 5, TFT_GREEN);  // Green box for off
      // tft.setCursor(200, 138);
      // tft.print("P1");
      tft.setCursor(190, 153);
      tft.print(buf3);

      tft.fillRoundRect(180, 170, 130, 30, 5, TFT_GREEN);  // Green box for off
      // tft.setCursor(200, 178);
      // tft.print("P2");
      tft.setCursor(190, 193);
      tft.print(buf4);

      tft.fillRoundRect(180, 210, 130, 30, 5, TFT_GREEN);  // Green box for off
      // tft.setCursor(200, 218);
      // tft.print("P3");
      tft.setCursor(190, 233);
      tft.print(buf5);
      tft.setFreeFont(&FreeSans12pt7b);
      break;

    case CCeffectBankSW:
      panelData[P_effectBank] = value;

      switch (panelData[P_effectBank]) {
        case 0:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name01[panelData[P_effectNum]])));
          break;
        case 1:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name11[panelData[P_effectNum]])));
          break;
        case 2:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name21[panelData[P_effectNum]])));
          break;
        case 3:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name31[panelData[P_effectNum]])));
          break;
      }
      // Check if the pointer is valid
      if (str_ptr != nullptr) {
        // Copy the string from program memory to RAM
        strcpy_P(buf1, str_ptr);
      } else {
        // Handle the case where the pointer is NULL (if needed)
      }

      switch (panelData[P_effectBank]) {
        case 0:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name02[panelData[P_effectNum]])));
          break;
        case 1:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name12[panelData[P_effectNum]])));
          break;
        case 2:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name22[panelData[P_effectNum]])));
          break;
        case 3:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name32[panelData[P_effectNum]])));
          break;
      }
      // Check if the pointer is valid
      if (str_ptr != nullptr) {
        // Copy the string from program memory to RAM
        strcpy_P(buf2, str_ptr);
      } else {
        // Handle the case where the pointer is NULL (if needed)
      }

      switch (panelData[P_effectBank]) {
        case 0:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name03[panelData[P_effectNum]])));
          break;
        case 1:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name13[panelData[P_effectNum]])));
          break;
        case 2:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name23[panelData[P_effectNum]])));
          break;
        case 3:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name33[panelData[P_effectNum]])));
          break;
      }

      // Check if the pointer is valid
      if (str_ptr != nullptr) {
        // Copy the string from program memory to RAM
        strcpy_P(buf3, str_ptr);
      } else {
        // Handle the case where the pointer is NULL (if needed)
      }

      switch (panelData[P_effectBank]) {
        case 0:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name04[panelData[P_effectNum]])));
          break;
        case 1:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name14[panelData[P_effectNum]])));
          break;
        case 2:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name24[panelData[P_effectNum]])));
          break;
        case 3:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name34[panelData[P_effectNum]])));
          break;
      }
      // Check if the pointer is valid
      if (str_ptr != nullptr) {
        // Copy the string from program memory to RAM
        strcpy_P(buf4, str_ptr);
      } else {
        // Handle the case where the pointer is NULL (if needed)
      }

      switch (panelData[P_effectBank]) {
        case 0:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name05[panelData[P_effectNum]])));
          break;
        case 1:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name15[panelData[P_effectNum]])));
          break;
        case 2:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name25[panelData[P_effectNum]])));
          break;
        case 3:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name35[panelData[P_effectNum]])));
          break;
      }
      // Check if the pointer is valid
      if (str_ptr != nullptr) {
        // Copy the string from program memory to RAM
        strcpy_P(buf5, str_ptr);
      } else {
        // Handle the case where the pointer is NULL (if needed)
      }

      tft.fillRoundRect(10, 10, 300, 30, 5, getActiveColor());
      tft.setCursor(50, 33);
      tft.print(buf1);
      tft.setCursor(180, 33);
      tft.print(buf2);

      tft.fillRoundRect(180, 50, 130, 30, 5, getActiveColor());  // Green box for off
      tft.setCursor(200, 73);
      tft.print("Bank:");
      tft.setCursor(280, 73);
      tft.print(panelData[P_effectBank] + 1);

      tft.fillRoundRect(180, 90, 130, 30, 5, getActiveColor());  // Green box for off
      tft.setCursor(200, 113);
      tft.print("Prog:");
      tft.setCursor(280, 113);
      tft.print(panelData[P_effectNum] + 1);

      tft.setFreeFont(&FreeSans9pt7b);

      tft.fillRoundRect(180, 130, 130, 30, 5, TFT_GREEN);  // Green box for off
      // tft.setCursor(200, 138);
      // tft.print("P1");
      tft.setCursor(190, 150);
      tft.print(buf3);

      tft.fillRoundRect(180, 170, 130, 30, 5, TFT_GREEN);  // Green box for off
      // tft.setCursor(200, 178);
      // tft.print("P2");
      tft.setCursor(190, 190);
      tft.print(buf4);

      tft.fillRoundRect(180, 210, 130, 30, 5, TFT_GREEN);  // Green box for off
      // tft.setCursor(200, 218);
      // tft.print("P3");
      tft.setCursor(190, 230);
      tft.print(buf5);
      tft.setFreeFont(&FreeSans12pt7b);
      break;

    case CCupperSW:
      upperSW = 1;
      lowerSW = 0;
      break;

    case CClowerSW:
      upperSW = 0;
      lowerSW = 1;
      break;
  }
}

// Function to draw the bar
void drawBar0(int x, int value, int steps, int stepHeight) {
  int filledSteps = value / (1023 / steps);  // Calculate the number of steps to fill
  for (int i = 0; i < steps; i++) {
    int y = 210 - (i * stepHeight);
    if (i < filledSteps) {
      tft.fillRoundRect(x, y - stepHeight, BAR_WIDTH, stepHeight - 2, 2, getActiveColor());  // Filled step
    } else {
      tft.fillRoundRect(x, y - stepHeight, BAR_WIDTH, stepHeight - 2, 2, TFT_BLACK);  // Unfilled step
    }
  }
}

void drawPWIndicator7(int x, int value) {
  // Clear the entire bar area by drawing a rectangle with the background color
  tft.fillRoundRect(x, 208 - BAR_HEIGHT, BAR_WIDTH, BAR_HEIGHT, 2, TFT_BLACK);
  tft.drawFastVLine(156, 50, 155, TFT_RED);
  tft.drawFastHLine(144, 50, 24, TFT_RED);
  tft.drawFastHLine(144, 127, 24, TFT_RED);
  tft.drawFastHLine(144, 205, 24, TFT_RED);
  // Calculate the y position for the current value
  int y = 205 - ((value * (BAR_HEIGHT - STEP_HEIGHT)) / 1023);

  // Draw the new bar at the calculated position
  tft.fillRoundRect(x, y - STEP_HEIGHT / 2, BAR_WIDTH, STEP_HEIGHT, 2, getActiveColor());
}

void renderCurrentPatchPage() {
  // ************************ DISPLAY 7 **************************
  tft.fillScreen(TFT_BLACK);  // Fill the screen with black
  tft.setFreeFont(&FreeSans9pt7b);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);

  // Setting text labels at the bottom of each bar
  tft.setCursor(6, 233);
  tft.print("P1");
  tft.setCursor(50, 233);
  tft.print("P2");
  tft.setCursor(98, 233);
  tft.print("P3");
  tft.setCursor(142, 233);
  tft.print("Mix");

  tft.drawFastVLine(156, 50, 155, TFT_RED);
  tft.drawFastHLine(144, 50, 24, TFT_RED);
  tft.drawFastHLine(144, 127, 24, TFT_RED);
  tft.drawFastHLine(144, 205, 24, TFT_RED);

  drawBar0(6, panelData[P_effectPot1], NUM_STEPS, STEP_HEIGHT);
  drawBar0(52, panelData[P_effectPot2], NUM_STEPS, STEP_HEIGHT);
  drawBar0(98, panelData[P_effectPot3], NUM_STEPS, STEP_HEIGHT);
  drawPWIndicator7(144, panelData[P_effectsMix]);

  tft.setFreeFont(&FreeSans12pt7b);
  tft.setTextColor(TFT_BLACK);  // Change text color to black for better contrast

  switch (panelData[P_effectBank]) {
    case 0:
      str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name01[panelData[P_effectNum]])));
      break;
    case 1:
      str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name11[panelData[P_effectNum]])));
      break;
    case 2:
      str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name21[panelData[P_effectNum]])));
      break;
    case 3:
      str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name31[panelData[P_effectNum]])));
      break;
  }
  // Check if the pointer is valid
  if (str_ptr != nullptr) {
    // Copy the string from program memory to RAM
    strcpy_P(buf1, str_ptr);
  } else {
    // Handle the case where the pointer is NULL (if needed)
  }

  switch (panelData[P_effectBank]) {
    case 0:
      str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name02[panelData[P_effectNum]])));
      break;
    case 1:
      str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name12[panelData[P_effectNum]])));
      break;
    case 2:
      str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name22[panelData[P_effectNum]])));
      break;
    case 3:
      str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name32[panelData[P_effectNum]])));
      break;
  }
  // Check if the pointer is valid
  if (str_ptr != nullptr) {
    // Copy the string from program memory to RAM
    strcpy_P(buf2, str_ptr);
  } else {
    // Handle the case where the pointer is NULL (if needed)
  }

  switch (panelData[P_effectBank]) {
    case 0:
      str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name03[panelData[P_effectNum]])));
      break;
    case 1:
      str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name13[panelData[P_effectNum]])));
      break;
    case 2:
      str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name23[panelData[P_effectNum]])));
      break;
    case 3:
      str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name33[panelData[P_effectNum]])));
      break;
  }

  // Check if the pointer is valid
  if (str_ptr != nullptr) {
    // Copy the string from program memory to RAM
    strcpy_P(buf3, str_ptr);
  } else {
    // Handle the case where the pointer is NULL (if needed)
  }

  switch (panelData[P_effectBank]) {
    case 0:
      str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name04[panelData[P_effectNum]])));
      break;
    case 1:
      str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name14[panelData[P_effectNum]])));
      break;
    case 2:
      str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name24[panelData[P_effectNum]])));
      break;
    case 3:
      str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name34[panelData[P_effectNum]])));
      break;
  }
  // Check if the pointer is valid
  if (str_ptr != nullptr) {
    // Copy the string from program memory to RAM
    strcpy_P(buf4, str_ptr);
  } else {
    // Handle the case where the pointer is NULL (if needed)
  }

  switch (panelData[P_effectBank]) {
    case 0:
      str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name05[panelData[P_effectNum]])));
      break;
    case 1:
      str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name15[panelData[P_effectNum]])));
      break;
    case 2:
      str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name25[panelData[P_effectNum]])));
      break;
    case 3:
      str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(name35[panelData[P_effectNum]])));
      break;
  }
  // Check if the pointer is valid
  if (str_ptr != nullptr) {
    // Copy the string from program memory to RAM
    strcpy_P(buf5, str_ptr);
  } else {
    // Handle the case where the pointer is NULL (if needed)
  }

  tft.fillRoundRect(10, 10, 300, 30, 5, getActiveColor());
  tft.setCursor(50, 33);
  tft.print(buf1);
  tft.setCursor(180, 33);
  tft.print(buf2);

  tft.fillRoundRect(180, 50, 130, 30, 5, getActiveColor());  // Green box for off
  tft.setCursor(200, 73);
  tft.print("Bank:");
  tft.setCursor(280, 73);
  tft.print(panelData[P_effectBank] + 1);

  tft.fillRoundRect(180, 90, 130, 30, 5, getActiveColor());  // Green box for off
  tft.setCursor(200, 113);
  tft.print("Prog:");
  tft.setCursor(280, 113);
  tft.print(panelData[P_effectNum] + 1);
  tft.setFreeFont(&FreeSans9pt7b);

  tft.fillRoundRect(180, 130, 130, 30, 5, TFT_GREEN);  // Green box for off
  tft.setCursor(190, 150);
  tft.print(buf3);

  tft.fillRoundRect(180, 170, 130, 30, 5, TFT_GREEN);  // Green box for off
  tft.setCursor(190, 190);
  tft.print(buf4);

  tft.fillRoundRect(180, 210, 130, 30, 5, TFT_GREEN);  // Green box for off
  tft.setCursor(190, 230);
  tft.print(buf5);
  tft.setFreeFont(&FreeSans12pt7b);
}

void loop(void) {
  MIDI.read(MIDI_CHANNEL_OMNI);
}