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

    case CCampAttack:
      panelData[P_ampAttack] = value;
      panelData[P_oldampAttack] = value;
      drawBar0(6, panelData[P_ampAttack], NUM_STEPS, STEP_HEIGHT);
      break;

    case CCampDecay:
      panelData[P_ampDecay] = value;
      panelData[P_oldampDecay] = value;
      drawBar0(52, panelData[P_ampDecay], NUM_STEPS, STEP_HEIGHT);
      break;

    case CCampSustain:
      panelData[P_ampSustain] = value;
      panelData[P_oldampSustain] = value;
      drawBar0(98, panelData[P_ampSustain], NUM_STEPS, STEP_HEIGHT);
      break;

    case CCampRelease:
      panelData[P_ampRelease] = value;
      panelData[P_oldampRelease] = value;
      drawBar0(144, panelData[P_ampRelease], NUM_STEPS, STEP_HEIGHT);
      break;
  }
}

void myLEDupdate(byte channel, byte control, int value) {
  switch (control) {

    case CCvcaVel:
      panelData[P_vcaVel] = value;
      if (!panelData[P_vcaVel]) {
        tft.fillRoundRect(180, 170, 130, 30, 5, TFT_GREEN);  // Green box for off
      } else {
        tft.fillRoundRect(180, 170, 130, 30, 5, TFT_RED);  // Red box for on
      }
      tft.setCursor(210, 193);
      tft.print(panelData[P_vcaVel] == 0 ? "Vel Off" : "Vel On");
      break;

    case CCvcaGate:
      panelData[P_vcaGate] = value;
      if (panelData[P_vcaGate] == 0) {
        panelData[P_ampAttack] = panelData[P_oldampAttack];
        panelData[P_ampDecay] = panelData[P_oldampDecay];
        panelData[P_ampSustain] = panelData[P_oldampSustain];
        panelData[P_ampRelease] = panelData[P_oldampRelease];
        tft.fillRoundRect(10, 10, 130, 30, 5, TFT_GREEN);  // Green box for off
      } else {
        panelData[P_ampAttack] = 0;
        panelData[P_ampDecay] = 0;
        panelData[P_ampSustain] = 1023;
        panelData[P_ampRelease] = 0;
        tft.fillRoundRect(10, 10, 130, 30, 5, TFT_RED);  // Red box for on
      }
      tft.setCursor(30, 33);
      tft.print(panelData[P_vcaGate] == 0 ? "Gate Off" : "Gate On");

      drawBar0(6, panelData[P_ampAttack], NUM_STEPS, STEP_HEIGHT);
      drawBar0(52, panelData[P_ampDecay], NUM_STEPS, STEP_HEIGHT);
      drawBar0(98, panelData[P_ampSustain], NUM_STEPS, STEP_HEIGHT);
      drawBar0(144, panelData[P_ampRelease], NUM_STEPS, STEP_HEIGHT);

      break;

    case CCampenvLinLogSW:
      panelData[P_ampLogLin] = value;
      if (!panelData[P_ampLogLin]) {
        tft.fillRoundRect(180, 90, 130, 30, 5, TFT_GREEN);  // Green box for off
      } else {
        tft.fillRoundRect(180, 90, 130, 30, 5, TFT_RED);  // Red box for on
      }
      tft.setCursor(210, 113);
      tft.print(panelData[P_ampLogLin] == 0 ? "Env Lin" : "Env Log");
      break;

    case CCAmpLoop:
      panelData[P_vcaLoop] = value;
      tft.fillRoundRect(180, 10, 130, 30, 5, getActiveColor());  // Green box for off
      switch (panelData[P_vcaLoop]) {
        case 0:
          tft.setCursor(195, 33);
          tft.print("Loop Off");
          break;
        case 1:
          tft.setCursor(183, 33);
          tft.print("Loop Gated");
          break;
        case 2:
          tft.setCursor(195, 33);
          tft.print("Loop LFO");
          break;
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
     // ************************ DISPLAY 5 **************************
      tft.fillScreen(TFT_BLACK);
      tft.setFreeFont(&FreeSans9pt7b);
      tft.setTextColor(TFT_WHITE);
      tft.setTextSize(1);
      tft.setCursor(10, 233);
      tft.print("A");
      tft.setCursor(58, 233);
      tft.print("D");
      tft.setCursor(106, 233);
      tft.print("S");
      tft.setCursor(148, 233);
      tft.print("R");
      tft.setFreeFont(&FreeSans12pt7b);
      tft.setTextSize(1);
      tft.setTextColor(TFT_BLACK);  // Change text color to black for better contrast

      if (!panelData[P_vcaVel]) {
        tft.fillRoundRect(180, 170, 130, 30, 5, TFT_GREEN);  // Green box for off
      } else {
        tft.fillRoundRect(180, 170, 130, 30, 5, TFT_RED);  // Red box for on
      }
      tft.setCursor(210, 193);
      tft.print(panelData[P_filterVel] == 0 ? "Vel Off" : "Vel On");

      if (panelData[P_vcaGate] == 0) {
        tft.fillRoundRect(10, 10, 130, 30, 5, TFT_GREEN);  // Green box for off
      } else {
        tft.fillRoundRect(10, 10, 130, 30, 5, TFT_RED);  // Red box for on
      }
      tft.setCursor(30, 33);
      tft.print(panelData[P_vcaGate] == 0 ? "Gate Off" : "Gate On");

      if (!panelData[P_ampLogLin]) {
        tft.fillRoundRect(180, 90, 130, 30, 5, TFT_GREEN);  // Green box for off
      } else {
        tft.fillRoundRect(180, 90, 130, 30, 5, TFT_RED);  // Red box for on
      }
      tft.setCursor(210, 113);
      tft.print(panelData[P_ampLogLin] == 0 ? "Env Lin" : "Env Log");


      tft.fillRoundRect(180, 10, 130, 30, 5, getActiveColor());  // Green box for off
      switch (panelData[P_vcaLoop]) {
        case 0:
          tft.setCursor(195, 33);
          tft.print("Loop Off");
          break;
        case 1:
          tft.setCursor(183, 33);
          tft.print("Loop Gated");
          break;
        case 2:
          tft.setCursor(195, 33);
          tft.print("Loop LFO");
          break;
      }

      // Drawing the bars
      drawBar0(6, panelData[P_ampAttack], NUM_STEPS, STEP_HEIGHT);
      drawBar0(52, panelData[P_ampDecay], NUM_STEPS, STEP_HEIGHT);
      drawBar0(98, panelData[P_ampSustain], NUM_STEPS, STEP_HEIGHT);
      drawBar0(144, panelData[P_ampRelease], NUM_STEPS, STEP_HEIGHT);
}

void loop(void) {
  MIDI.read(MIDI_CHANNEL_OMNI);
}

