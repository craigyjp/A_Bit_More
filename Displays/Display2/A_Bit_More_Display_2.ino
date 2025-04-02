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

    case CCosc2PW:
      panelData[P_osc2PW] = value;
      drawPWIndicator0(6, panelData[P_osc2PW]);
      break;

    case CCosc2PWM:
      panelData[P_osc2PWM] = value;
      drawBar0(52, panelData[P_osc2PWM], NUM_STEPS, STEP_HEIGHT);
      break;

    case CCosc2Detune:
      panelData[P_osc2Detune] = value;
      drawBar0(98, panelData[P_osc2Detune], NUM_STEPS, STEP_HEIGHT);
      break;

    case CCosc2Interval:
      panelData[P_osc2Interval] = value / 8;
      drawBar1(144, panelData[P_osc2Interval], DET_STEPS, DET_STEP_HEIGHT);
      break;

    case CCosc2sawLevel:
      panelData[P_osc2SawLevel] = value;
      drawBar0(192, panelData[P_osc2SawLevel], NUM_STEPS, STEP_HEIGHT);
      break;

    case CCosc2pulseLevel:
      panelData[P_osc2PulseLevel] = value;
      drawBar0(236, panelData[P_osc2PulseLevel], NUM_STEPS, STEP_HEIGHT);
      break;

    case CCosc2triangleLevel:
      panelData[P_osc2TriangleLevel] = value;
      drawBar0(282, panelData[P_osc2TriangleLevel], NUM_STEPS, STEP_HEIGHT);
      break;
  }
}

void myLEDupdate(byte channel, byte control, int value) {
  switch (control) {

    case CCsyncSW:
      panelData[P_sync] = value;
      if (panelData[P_sync]) {
        digitalWrite(SYNC_LED, HIGH);
      } else if (!panelData[P_sync]) {
        digitalWrite(SYNC_LED, LOW);
      }
      if (panelData[P_sync]) {
        tft.fillRoundRect(10, 10, 130, 30, 5, TFT_RED);
      } else {
        tft.fillRoundRect(10, 10, 130, 30, 5, TFT_GREEN);
      }
      tft.setCursor(30, 33);
      tft.print(panelData[P_sync] == 0 ? "Sync Off" : "Sync On");
      break;

    case CCosc2Oct:
      panelData[P_osc2Range] = value;
      tft.setFreeFont(&FreeSans12pt7b);
      // Set range label and value inside a box along the top
      tft.fillRoundRect(180, 10, 130, 30, 5, getActiveColor());  // Background box for range
      tft.setTextColor(TFT_BLACK);
      tft.setCursor(195, 33);
      switch (panelData[P_osc2Range]) {
        case 0:
          tft.print("Range 32");
          break;
        case 1:
          tft.print("Range 16");
          break;
        case 2:
          tft.print("Range 8");
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

void drawBar1(int x, int value, int steps, int stepHeight) {
  value = constrain(value, 0, steps);  // Ensure value stays within the correct range

  for (int i = 0; i < steps; i++) {
    int y = 210 - (i * stepHeight);  // Each step is stacked upwards

    // Draw filled steps only up to `value`
    uint16_t color = (i < value) ? getActiveColor() : TFT_BLACK;
    tft.fillRoundRect(x, y - stepHeight, BAR_WIDTH, stepHeight - 2, 2, color);
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
  //   // ************************ DISPLAY 2 *************************
  tft.fillScreen(TFT_BLACK);  // Fill the screen with black
  tft.setFreeFont(&FreeSans9pt7b);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);

  // Setting text labels at the bottom of each bar
  tft.setCursor(4, 233);
  tft.print("PW");
  tft.setCursor(43, 233);
  tft.print("PWM");
  tft.setCursor(96, 233);
  tft.print("Det");
  tft.setCursor(146, 233);
  tft.print("Int");
  tft.setCursor(184, 233);
  tft.print("Saw");
  tft.setCursor(234, 233);
  tft.print("Sqr");
  tft.setCursor(284, 233);
  tft.print("Tri");

  tft.setFreeFont(&FreeSans12pt7b);
  tft.setTextColor(TFT_BLACK);
  // Set range label and value inside a box along the top
  tft.fillRoundRect(180, 10, 130, 30, 5, getActiveColor());  // Background box for range

  tft.setCursor(195, 33);
  switch (panelData[P_osc2Range]) {
    case 0:
      tft.print("Range 32");
      break;
    case 1:
      tft.print("Range 16");
      break;
    case 2:
      tft.print("Range 8");
      break;
  }

  // Sync indicator box along the top
  if (!panelData[P_sync]) {
    tft.fillRoundRect(10, 10, 130, 30, 5, TFT_GREEN);  // Green box for off
  } else {
    tft.fillRoundRect(10, 10, 130, 30, 5, TFT_RED);  // Red box for on
  }
  tft.setCursor(30, 33);
  tft.print(panelData[P_sync] == 0 ? "Sync Off" : "Sync On");

  // Drawing the PW lines
  tft.drawFastVLine(18, 50, 155, TFT_RED);
  tft.drawFastHLine(6, 50, 24, TFT_RED);
  tft.drawFastHLine(6, 127, 24, TFT_RED);
  tft.drawFastHLine(6, 205, 24, TFT_RED);

  // Drawing the bars
  drawPWIndicator0(6, panelData[P_osc2PW]);
  drawBar0(52, panelData[P_osc2PWM], NUM_STEPS, STEP_HEIGHT);
  drawBar0(98, panelData[P_osc2Detune], NUM_STEPS, STEP_HEIGHT);
  drawBar1(144, panelData[P_osc2Interval], DET_STEPS, DET_STEP_HEIGHT);
  drawBar0(192, panelData[P_osc2SawLevel], NUM_STEPS, STEP_HEIGHT);
  drawBar0(236, panelData[P_osc2PulseLevel], NUM_STEPS, STEP_HEIGHT);
  drawBar0(282, panelData[P_osc2TriangleLevel], NUM_STEPS, STEP_HEIGHT);
}

void loop(void) {
  MIDI.read(MIDI_CHANNEL_OMNI);
}
