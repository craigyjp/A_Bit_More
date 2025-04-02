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

    case CCLFORate:
      panelData[P_LFORate] = value;
      drawBar0(6, panelData[P_LFORate], NUM_STEPS, STEP_HEIGHT);
      break;

    case CCLFODelay:
      panelData[P_LFODelay] = value;
      drawBar0(52, panelData[P_LFODelay], NUM_STEPS, STEP_HEIGHT);
      break;

    case CCpwLFO:
      panelData[P_pwLFO] = value;
      drawBar0(98, panelData[P_pwLFO], NUM_STEPS, STEP_HEIGHT);
      break;

    case CCnoiseLevel:
      panelData[P_noiseLevel] = value;
      drawPWIndicator8(144, panelData[P_noiseLevel]);
      break;
  }
}

void myLEDupdate(byte channel, byte control, int value) {
  switch (control) {

    case CCLFOWaveform:
      panelData[P_LFOWaveform] = value;
      switch (panelData[P_lfoAlt]) {
        case 1:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(lfo02[panelData[P_LFOWaveform]])));
          break;
        case 0:
          str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(lfo01[panelData[P_LFOWaveform]])));
          break;
      }
      // Check if the pointer is valid
      if (str_ptr != nullptr) {
        // Copy the string from program memory to RAM
        strcpy_P(lfoDisplay, str_ptr);
      }
      tft.setFreeFont(&FreeSans12pt7b);

      // Set range label and value inside a box along the top
      tft.fillRoundRect(10, 10, 300, 30, 5, getActiveColor());  // Background box for range
      tft.setCursor(40, 33);
      tft.setTextColor(TFT_BLACK);
      tft.print(lfoDisplay);
      break;

    case CClfoAlt:
      panelData[P_lfoAlt] = value;
      if (panelData[P_lfoAlt]) {
        digitalWrite(LFO_ALT_LED, HIGH);
      } else if (!panelData[P_lfoAlt]) {
        digitalWrite(LFO_ALT_LED, LOW);
      }
      if (panelData[P_lfoAlt]) {
        tft.fillRoundRect(180, 50, 130, 30, 5, TFT_RED);
      } else {
        tft.fillRoundRect(180, 50, 130, 30, 5, TFT_GREEN);
      }
      tft.setTextColor(TFT_BLACK);  // Change text color to black for better contrast
      tft.setCursor(200, 73);
      tft.print(panelData[P_lfoAlt] == 0 ? "Alt Off" : "Alt On");
      break;

    case CClfoMult:
      panelData[P_lfoMultiplier] = value;
      tft.fillRoundRect(180, 130, 130, 30, 5, getActiveColor());  // Background box for range
      tft.setCursor(200, 153);
      tft.print("Mult");
      tft.setCursor(260, 153);
      switch (panelData[P_lfoMultiplier]) {
        case 0:
          tft.print("x0.5");
          break;
        case 1:
          tft.print("x1.0");
          break;
        case 2:
          tft.print("x1.5");
          break;
        case 3:
          tft.print("x2.0");
          break;
        case 4:
          tft.print("x2.5");
          break;
      }
      break;

    case CCmonoMulti:
      panelData[P_monoMulti] = value;
      if (panelData[P_monoMulti]) {
        digitalWrite(LFO_MULTI_MONO_LED, HIGH);
        tft.fillRoundRect(180, 90, 130, 30, 5, TFT_RED);
      } else {
        digitalWrite(LFO_MULTI_MONO_LED, LOW);
        tft.fillRoundRect(180, 90, 130, 30, 5, TFT_GREEN);
      }
      tft.setCursor(200, 113);
      tft.print(panelData[P_monoMulti] == 0 ? "Trig Off" : "Trig On");
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

void drawPWIndicator8(int x, int value) {
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
  // ************************ DISPLAY 6 **************************
  tft.fillScreen(TFT_BLACK);  // Fill the screen with black
  tft.setFreeFont(&FreeSans9pt7b);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);

  // Setting text labels at the bottom of each bar
  tft.setCursor(0, 233);
  tft.print("LFO");
  tft.setCursor(50, 233);
  tft.print("Dly");
  tft.setCursor(96, 233);
  tft.print("PW");
  tft.setCursor(138, 233);
  tft.print("Noise");

  tft.setFreeFont(&FreeSans12pt7b);
  tft.setTextColor(TFT_BLACK);

  switch (panelData[P_lfoAlt]) {
    case 0:
      str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(lfo02[panelData[P_LFOWaveform]])));
      break;
    case 1:
      str_ptr = reinterpret_cast<const char *>(pgm_read_ptr(&(lfo01[panelData[P_LFOWaveform]])));
      break;
  }
  // Check if the pointer is valid
  if (str_ptr != nullptr) {
    // Copy the string from program memory to RAM
    strcpy_P(lfoDisplay, str_ptr);
  }
  tft.setFreeFont(&FreeSans12pt7b);

  // Set range label and value inside a box along the top
  tft.fillRoundRect(10, 10, 300, 30, 5, getActiveColor());  // Background box for range
  tft.setCursor(40, 33);
  tft.print(lfoDisplay);

  if (panelData[P_lfoAlt]) {
    tft.fillRoundRect(180, 50, 130, 30, 5, TFT_RED);
  } else {
    tft.fillRoundRect(180, 50, 130, 30, 5, TFT_GREEN);
  }
  tft.setCursor(200, 73);
  tft.print(panelData[P_lfoAlt] == 0 ? "Alt Off" : "Alt On");

  if (panelData[P_monoMulti]) {
    tft.fillRoundRect(180, 90, 130, 30, 5, TFT_RED);
  } else {
    tft.fillRoundRect(180, 90, 130, 30, 5, TFT_GREEN);
  }
  tft.setCursor(200, 113);
  tft.print(panelData[P_monoMulti] == 0 ? "Trig Off" : "Trig On");

  // Set range label and value inside a box along the top
  tft.fillRoundRect(180, 130, 130, 30, 5, getActiveColor());  // Background box for range
  tft.setCursor(200, 153);
  tft.print("Mult");
  tft.setCursor(260, 153);
  switch (panelData[P_lfoMultiplier]) {
    case 0:
      tft.print("x0.5");
      break;
    case 1:
      tft.print("x1.0");
      break;
    case 2:
      tft.print("x1.5");
      break;
    case 3:
      tft.print("x2.0");
      break;
    case 4:
      tft.print("x2.5");
      break;
  }

  tft.drawFastVLine(156, 50, 155, TFT_RED);
  tft.drawFastHLine(144, 50, 24, TFT_RED);
  tft.drawFastHLine(144, 127, 24, TFT_RED);
  tft.drawFastHLine(144, 205, 24, TFT_RED);

  // Drawing the bars
  drawBar0(6, panelData[P_LFORate], NUM_STEPS, STEP_HEIGHT);
  drawBar0(52, panelData[P_LFODelay], NUM_STEPS, STEP_HEIGHT);
  drawBar0(98, panelData[P_pwLFO], NUM_STEPS, STEP_HEIGHT);
  drawPWIndicator8(144, panelData[P_noiseLevel]);
}

void loop(void) {
  MIDI.read(MIDI_CHANNEL_OMNI);
}
