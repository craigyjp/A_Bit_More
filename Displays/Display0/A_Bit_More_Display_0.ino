#include "SPI.h"
#include "TFT_eSPI.h"
#include <MIDI.h>
#include "Constants.h"
#include "Parameters.h"
#include "MidiCC.h"

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

    case CCglideTime:
      panelData[P_glideTime] = value;
      drawBar0(6, panelData[P_glideTime], NUM_STEPS, STEP_HEIGHT);
      break;
  }
}

void myLEDupdate(byte channel, byte control, int value) {
  switch (control) {

    case CCupperSW:
      upperSW = value;
      if (upperSW) {
        digitalWrite(UPPER_LED, HIGH);
        digitalWrite(LOWER_LED, LOW);
        lowerSW = 0;
      }
      // upper/lower indicator box along the top
      tft.setTextColor(TFT_BLACK);  // Change text color to black for better contrast
      tft.setFreeFont(&FreeSans12pt7b);
      if (upperSW == 0) {
        tft.fillRoundRect(185, 90, 125, 30, 5, TFT_GREEN);  // Green box for off
      } else {
        tft.fillRoundRect(185, 90, 125, 30, 5, TFT_RED);  // Green box for off
      }
      tft.setCursor(200, 113);
      tft.print(upperSW == 0 ? "     " : "Upper");

      if (lowerSW == 0) {
        tft.fillRoundRect(50, 90, 125, 30, 5, TFT_GREEN);  // Green box for off
      } else {
        tft.fillRoundRect(50, 90, 125, 30, 5, TFT_RED);  // Green box for off
      }
      tft.setCursor(65, 113);
      tft.print(lowerSW == 0 ? "     " : "Lower");
      //tft.updateScreen();
      memcpy(panelData, upperData, 71);
      //processSysExData();
      break;

    case CClowerSW:
      lowerSW = value;
      if (lowerSW) {
        digitalWrite(UPPER_LED, LOW);
        digitalWrite(LOWER_LED, HIGH);
        upperSW = 0;
      }
      // upper/lower indicator box along the top
      tft.setFreeFont(&FreeSans12pt7b);
      if (upperSW == 0) {
        tft.fillRoundRect(185, 90, 125, 30, 5, TFT_GREEN);  // Green box for off
      } else {
        tft.fillRoundRect(185, 90, 125, 30, 5, TFT_RED);  // Green box for off
      }
      tft.setTextColor(TFT_BLACK);  // Change text color to black for better contrast
      tft.setCursor(200, 113);
      tft.print(upperSW == 0 ? "     " : "Upper");

      if (lowerSW == 0) {
        tft.fillRoundRect(50, 90, 125, 30, 5, TFT_GREEN);  // Green box for off
      } else {
        tft.fillRoundRect(50, 90, 125, 30, 5, TFT_RED);  // Green box for off
      }
      tft.setTextColor(TFT_BLACK);  // Change text color to black for better contrast
      tft.setCursor(60, 113);
      tft.print(lowerSW == 0 ? "     " : "Lower");
      memcpy(panelData, lowerData, 71);
      //processSysExData();
      break;

    case CCNotePriority:
      panelData[P_NotePriority] = value;
      tft.fillRoundRect(50, 130, 125, 30, 5, TFT_CYAN);  // Cyan box
      tft.setCursor(60, 153);
      tft.print("Priority");
      tft.fillRoundRect(185, 130, 125, 30, 5, TFT_YELLOW);  // Yellow box
      switch (panelData[P_NotePriority]) {
        case 0:
          tft.setCursor(200, 153);
          tft.print("Top");
          break;

        case 1:
          tft.setCursor(200, 153);
          tft.print("Bottom");
          break;

        case 2:
          tft.setCursor(200, 153);
          tft.print("Last");
          break;
        
        case 3:
          tft.fillRoundRect(185, 130, 125, 30, 5, TFT_YELLOW);  // Yellow box
          break;
      }
      break;

    case CCglideSW:
      if (value >= 1) {
        panelData[P_glideSW] = 1;
      } else if (value == 0) {
        panelData[P_glideSW] = 0;
      }
      tft.setFreeFont(&FreeSans12pt7b);
      if (panelData[P_glideSW] == 0) {
        tft.fillRoundRect(50, 10, 125, 30, 5, TFT_GREEN);  // Green box for off
        digitalWrite(GLIDE_LED, LOW);
      } else if (panelData[P_glideSW] == 1) {
        tft.fillRoundRect(50, 10, 125, 30, 5, TFT_RED);  // Red box for on
        digitalWrite(GLIDE_LED, HIGH);
      }
      tft.setTextColor(TFT_BLACK);  // Change text color to black for better contrast
      tft.setCursor(60, 33);
      tft.print(panelData[P_glideSW] == 0 ? "Glide Off" : "Glide On");
      break;

    case CCchordHoldSW:
      if (value == 127) {
        chordHoldSW = 1;
      } else if (value == 0) {
        chordHoldSW = 0;
      }
      tft.setFreeFont(&FreeSans12pt7b);
      if (chordHoldSW == 0) {
        tft.fillRoundRect(185, 10, 125, 30, 5, TFT_GREEN);  // Green box for off
        digitalWrite(CHORD_HOLD_LED, LOW);
      } else {
        tft.fillRoundRect(185, 10, 125, 30, 5, TFT_RED);  // Red box for on
        digitalWrite(CHORD_HOLD_LED, HIGH);
      }
      tft.setTextColor(TFT_BLACK);  // Change text color to black for better contrast
      tft.setCursor(200, 33);
      tft.print(chordHoldSW == 0 ? "Hold Off" : "Hold On");
      break;

    case CCplayMode:
      playMode = value;
      tft.fillRoundRect(50, 170, 125, 30, 5, TFT_CYAN);  // Cyan box
      tft.setCursor(60, 193);
      tft.print("Keyboard");
      tft.fillRoundRect(185, 170, 125, 30, 5, TFT_YELLOW);  // Yellow box
      switch (playMode) {
        case 0:
          tft.setCursor(200, 193);
          tft.print("Whole");
          digitalWrite(KEY_MODE_RED_LED, LOW);
          digitalWrite(KEY_MODE_GREEN_LED, LOW);
          break;

        case 1:
          tft.setCursor(200, 193);
          tft.print("Dual");
          digitalWrite(KEY_MODE_RED_LED, LOW);
          digitalWrite(KEY_MODE_GREEN_LED, HIGH);
          break;

        case 2:
          tft.setCursor(200, 193);
          tft.print("Split");
          digitalWrite(KEY_MODE_RED_LED, HIGH);
          digitalWrite(KEY_MODE_GREEN_LED, LOW);
          break;
      }
      break;

    case CCkeyboardMode:
      panelData[P_keyboardMode] = value;
      tft.fillRoundRect(50, 210, 125, 30, 5, TFT_CYAN);  // Cyan box
      tft.setCursor(60, 233);
      tft.print("Key Mode");
      tft.fillRoundRect(185, 210, 125, 30, 5, TFT_YELLOW);  // Yellow box
      switch (panelData[P_keyboardMode]) {
        case 0:
          tft.setCursor(200, 233);
          tft.print("Poly 1");
          digitalWrite(POLY1_LED, HIGH);
          digitalWrite(POLY2_LED, LOW);
          digitalWrite(MONO_LED, LOW);
          digitalWrite(UNISON_LED, LOW);
          break;

        case 1:
          tft.setCursor(200, 233);
          tft.print("Poly 2");
          digitalWrite(POLY1_LED, LOW);
          digitalWrite(POLY2_LED, HIGH);
          digitalWrite(MONO_LED, LOW);
          digitalWrite(UNISON_LED, LOW);
          break;

        case 2:
          tft.setCursor(200, 233);
          tft.print("Mono");
          digitalWrite(POLY1_LED, LOW);
          digitalWrite(POLY2_LED, LOW);
          digitalWrite(MONO_LED, HIGH);
          digitalWrite(UNISON_LED, LOW);
          break;

        case 3:
          tft.setCursor(200, 233);
          tft.print("Unison");
          digitalWrite(POLY1_LED, LOW);
          digitalWrite(POLY2_LED, LOW);
          digitalWrite(MONO_LED, LOW);
          digitalWrite(UNISON_LED, HIGH);
          break;
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

  // ************************ DISPLAY 0 **************************
  tft.fillScreen(TFT_BLACK);  // Fill the screen with black
  tft.setFreeFont(&FreeSans9pt7b);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);

  // Setting text labels at the bottom of each bar
  tft.setCursor(2, 233);
  tft.print("GLD");
  tft.setTextColor(TFT_BLACK);  // Change text color to black for better contrast
  tft.setFreeFont(&FreeSans12pt7b);

  // glide indicator box along the top
  if (panelData[P_glideSW] == 0) {
    tft.fillRoundRect(50, 10, 125, 30, 5, TFT_GREEN);  // Green box for off
  } else {
    tft.fillRoundRect(50, 10, 125, 30, 5, TFT_RED);  // Red box for on
  }
  tft.setCursor(60, 33);
  tft.print(panelData[P_glideSW] == 0 ? "Glide Off" : "Glide On");

  // chord Hold indicator box along the top
  if (chordHoldSW == 0) {
    tft.fillRoundRect(185, 10, 125, 30, 5, TFT_GREEN);  // Green box for off
  } else {
    tft.fillRoundRect(185, 10, 125, 30, 5, TFT_RED);  // Red box for on
  }
  tft.setCursor(200, 33);
  tft.print(chordHoldSW == 0 ? "Hold Off" : "Hold On");

  // upper/lower indicator box along the top
  if (upperSW == 0) {
    tft.fillRoundRect(185, 90, 125, 30, 5, TFT_GREEN);  // Green box for off
  } else {
    tft.fillRoundRect(185, 90, 125, 30, 5, TFT_RED);  // Green box for off
  }
  tft.setCursor(200, 113);
  tft.print(upperSW == 0 ? "     " : "Upper");

  if (lowerSW == 0) {
    tft.fillRoundRect(50, 90, 125, 30, 5, TFT_GREEN);  // Green box for off
  } else {
    tft.fillRoundRect(50, 90, 125, 30, 5, TFT_RED);  // Green box for off
  }
  tft.setCursor(60, 113);
  tft.print(lowerSW == 0 ? "     " : "Lower");

  tft.fillRoundRect(50, 130, 125, 30, 5, TFT_CYAN);  // Cyan box
  tft.setCursor(60, 153);
  tft.print("Priority");
  tft.fillRoundRect(185, 130, 125, 30, 5, TFT_YELLOW);  // Yellow box
  switch (NotePriority) {
    case 0:
      tft.setCursor(200, 153);
      tft.print("Top");
      break;

    case 1:
      tft.setCursor(200, 153);
      tft.print("Bottom");
      break;

    case 2:
      tft.setCursor(200, 153);
      tft.print("Last");
      break;

    case 3:
      tft.fillRoundRect(185, 130, 125, 30, 5, TFT_YELLOW);  // Yellow box
      break;
  }

  tft.fillRoundRect(50, 170, 125, 30, 5, TFT_CYAN);  // Cyan box
  tft.setCursor(60, 193);
  tft.print("Keyboard");
  tft.fillRoundRect(185, 170, 125, 30, 5, TFT_YELLOW);  // Yellow box
  switch (playMode) {
    case 0:
      tft.setCursor(200, 193);
      tft.print("Whole");
      break;

    case 1:
      tft.setCursor(200, 193);
      tft.print("Dual");
      break;

    case 2:
      tft.setCursor(200, 193);
      tft.print("Split");
      break;
  }

  tft.fillRoundRect(50, 210, 125, 30, 5, TFT_CYAN);  // Cyan box
  tft.setCursor(60, 233);
  tft.print("Key Mode");
  tft.fillRoundRect(185, 210, 125, 30, 5, TFT_YELLOW);  // Yellow box
  switch (panelData[P_keyboardMode]) {
    case 0:
      tft.setCursor(200, 233);
      tft.print("Poly 1");
      break;

    case 1:
      tft.setCursor(200, 233);
      tft.print("Poly 2");
      break;

    case 2:
      tft.setCursor(200, 233);
      tft.print("Unison");
      break;

    case 3:
      tft.setCursor(200, 233);
      tft.print("Mono");
      break;
  }


  drawBar0(6, panelData[P_glideTime], NUM_STEPS, STEP_HEIGHT);
}

void loop(void) {
  MIDI.read(MIDI_CHANNEL_OMNI);
}
