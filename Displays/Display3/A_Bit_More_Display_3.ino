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

uint16_t getActiveColor() {
  return upperSW ? TFT_CYAN : TFT_YELLOW;
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
      tft.fillRoundRect(10, 10, 200, 30, 5, getActiveColor());  // Background box for range
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
      tft.fillRoundRect(10, 10, 200, 30, 5, getActiveColor());  // Background box for range
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
  tft.fillRoundRect(x, y - STEP_HEIGHT / 2, BAR_WIDTH, STEP_HEIGHT, 2, getActiveColor());
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
  tft.fillRoundRect(10, 10, 200, 30, 5, getActiveColor());  // Background box for range
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