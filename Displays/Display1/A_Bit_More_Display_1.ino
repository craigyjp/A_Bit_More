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

    case CCosc1PW:
      panelData[P_osc1PW] = value;
      Serial.println(panelData[P_osc1PW]);
      drawPWIndicator0(6, panelData[P_osc1PW]);
      break;

    case CCosc1PWM:
      panelData[P_osc1PWM] = value;
      drawBar0(52, panelData[P_osc1PWM], NUM_STEPS, STEP_HEIGHT);
      break;

    case CCosc1SawLevel:
      panelData[P_osc1SawLevel] = value;
      drawBar0(192, panelData[P_osc1SawLevel], NUM_STEPS, STEP_HEIGHT);
      break;

    case CCosc1PulseLevel:
      panelData[P_osc1PulseLevel] = value;
      drawBar0(236, panelData[P_osc1PulseLevel], NUM_STEPS, STEP_HEIGHT);
      break;

    case CCosc1SubLevel:
      panelData[P_osc1SubLevel] = value;
      drawBar0(282, panelData[P_osc1SubLevel], NUM_STEPS, STEP_HEIGHT);
      break;

    case CCoscfmDepth:
      panelData[P_fmDepth] = value;
      drawBar0(98, panelData[P_fmDepth], NUM_STEPS, STEP_HEIGHT);
      break;
  }
}

void myLEDupdate(byte channel, byte control, int value) {
  switch (control) {

    case CCNotePriority:
      panelData[P_NotePriority] = value;
      switch (panelData[P_NotePriority]) {
        case 0:
          digitalWrite(PRIORITY_RED_LED, HIGH);
          digitalWrite(PRIORITY_GREEN_LED, LOW);
          break;

        case 1:
          digitalWrite(PRIORITY_RED_LED, LOW);
          digitalWrite(PRIORITY_GREEN_LED, HIGH);
          break;

        case 2:
          digitalWrite(PRIORITY_RED_LED, HIGH);
          digitalWrite(PRIORITY_GREEN_LED, HIGH);
          break;

        case 3:
          digitalWrite(PRIORITY_RED_LED, LOW);
          digitalWrite(PRIORITY_GREEN_LED, LOW);
          break;
      }
      break;

    case CCosc1Oct:
      panelData[P_osc1Range] = value;
      tft.setFreeFont(&FreeSans12pt7b);
      // Set range label and value inside a box along the top
      tft.fillRoundRect(180, 10, 130, 30, 5, getActiveColor());  // Background box for range
      tft.setTextColor(TFT_BLACK);
      tft.setCursor(195, 33);
      switch (panelData[P_osc1Range]) {
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

    case CCKeyTrackSW:
      panelData[P_keytrackSW] = value;
      if (panelData[P_keytrackSW]) {
        digitalWrite(KEYTRACK_LED, HIGH);
      } else if (!panelData[P_keytrackSW]) {
        digitalWrite(KEYTRACK_LED, LOW);
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

        tft.setFreeFont(&FreeSans9pt7b);
        tft.setTextColor(TFT_WHITE);
        tft.setTextSize(1);

        // Setting text labels at the bottom of each bar
        tft.setCursor(4, 233);
        tft.print("PW");
        tft.setCursor(43, 233);
        tft.print("PWM");
        tft.setCursor(96, 233);
        tft.print("FM");

        tft.setCursor(184, 233);
        tft.print("Saw");
        tft.setCursor(234, 233);
        tft.print("Sqr");
        tft.setCursor(280, 233);
        tft.print("Sub");

        tft.setFreeFont(&FreeSans12pt7b);
        tft.setTextColor(TFT_BLACK);
        // Set range label and value inside a box along the top
        tft.fillRoundRect(180, 10, 130, 30, 5, getActiveColor());  // Background box for range
        tft.setCursor(195, 33);
        switch (panelData[P_osc1Range]) {
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
        // Drawing the PW lines
        tft.drawFastVLine(18, 50, 155, TFT_RED);
        tft.drawFastHLine(6, 50, 24, TFT_RED);
        tft.drawFastHLine(6, 127, 24, TFT_RED);
        tft.drawFastHLine(6, 205, 24, TFT_RED);

        // Drawing the bars
        drawPWIndicator0(6, panelData[P_osc1PW]);
        drawBar0(52, panelData[P_osc1PWM], NUM_STEPS, STEP_HEIGHT);
        drawBar0(98, panelData[P_fmDepth], NUM_STEPS, STEP_HEIGHT);

        drawBar0(192, panelData[P_osc1SawLevel], NUM_STEPS, STEP_HEIGHT);
        drawBar0(236, panelData[P_osc1PulseLevel], NUM_STEPS, STEP_HEIGHT);
        drawBar0(282, panelData[P_osc1SubLevel], NUM_STEPS, STEP_HEIGHT);

}

void loop(void) {
  MIDI.read(MIDI_CHANNEL_OMNI);
}