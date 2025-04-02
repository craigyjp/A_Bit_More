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

    case CCPM_DCO2:
      panelData[P_pmDCO2] = value;
      drawBar0(6, panelData[P_pmDCO2], NUM_STEPS, STEP_HEIGHT);
      break;

    case CCPM_FilterEnv:
      panelData[P_pmFilterEnv] = value;
      drawBar0(52, panelData[P_pmFilterEnv], NUM_STEPS, STEP_HEIGHT);
      break;

    case CCvolumeControl:
      panelData[P_volumeControl] = value;
      drawBar0(98, panelData[P_volumeControl], NUM_STEPS, STEP_HEIGHT);
      break;

    case CCamDepth:
      panelData[P_amDepth] = value;
      drawBar0(144, panelData[P_amDepth], NUM_STEPS, STEP_HEIGHT);
      break;

    case CCoscATDepth:
      panelData[P_ATDepth] = value;
      drawBar0(192, panelData[P_ATDepth], NUM_STEPS, STEP_HEIGHT);
      break;

    case CCmodWheelDepth:
      panelData[P_modWheelDepth] = value;
      drawBar0(236, panelData[P_modWheelDepth], NUM_STEPS, STEP_HEIGHT);
      break;

    case CCPitchBend:
      panelData[P_PitchBendLevel] = value / 8;
      drawBar1(282, panelData[P_PitchBendLevel], DET_STEPS, DET_STEP_HEIGHT);
      break;

  }
}

void myLEDupdate(byte channel, byte control, int value) {
  switch (control) {

    case CCpmDestDCO1SW:
      panelData[P_pmDestDCO1] = value;
      if (panelData[P_pmDestDCO1]) {
        digitalWrite(PM_DCO1_DEST_LED, HIGH);
      } else if (!panelData[P_pmDestDCO1]) {
        digitalWrite(PM_DCO1_DEST_LED, LOW);
      }
      if (panelData[P_pmDestDCO1]) {
        tft.fillRoundRect(10, 10, 130, 30, 5, TFT_RED);
      } else {
        tft.fillRoundRect(10, 10, 130, 30, 5, TFT_GREEN);
      }
      tft.setCursor(20, 33);
      tft.print(panelData[P_pmDestDCO1] == 0 ? "DCO1 Off" : "DCO1 On");
      break;

    case CCpmDestFilterSW:
      panelData[P_pmDestFilter] = value;
      if (panelData[P_pmDestFilter]) {
        digitalWrite(PM_FILT_ENV_DEST_LED, HIGH);
      } else if (!panelData[P_pmDestFilter]) {
        digitalWrite(PM_FILT_ENV_DEST_LED, LOW);
      }
      if (panelData[P_pmDestFilter]) {
        tft.fillRoundRect(180, 10, 130, 30, 5, TFT_RED);
      } else {
        tft.fillRoundRect(180, 10, 130, 30, 5, TFT_GREEN);
      }
      tft.setCursor(200, 33);
      tft.print(panelData[P_pmDestFilter] == 0 ? "Filter Off" : "Filter On");
      break;

    case CCvcaGate:
      panelData[P_vcaGate] = value;
      if (panelData[P_vcaGate] == 0) {
        digitalWrite(AMP_GATED_LED, LOW);
      } else {
        digitalWrite(AMP_GATED_LED, HIGH);
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

void renderCurrentPatchPage() {
  // ************************ DISPLAY 8 **************************
  tft.fillScreen(TFT_BLACK);  // Fill the screen with black
  tft.setFreeFont(&FreeSans9pt7b);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);

  // Setting text labels at the bottom of each bar
  tft.setCursor(0, 233);
  tft.print("DCO");
  tft.setCursor(48, 233);
  tft.print("Env");
  tft.setCursor(96, 233);
  tft.print("Vol");
  tft.setCursor(142, 233);
  tft.print("AM");
  tft.setCursor(184, 233);
  tft.print("AT");
  tft.setCursor(234, 233);
  tft.print("MW");
  tft.setCursor(280, 233);
  tft.print("PB");

  tft.setFreeFont(&FreeSans12pt7b);
  tft.setTextColor(TFT_BLACK);

  if (panelData[P_pmDestDCO1]) {
    digitalWrite(PM_DCO1_DEST_LED, HIGH);
  } else if (!panelData[P_pmDestDCO1]) {
    digitalWrite(PM_DCO1_DEST_LED, LOW);
  }
  if (panelData[P_pmDestDCO1]) {
    tft.fillRoundRect(10, 10, 130, 30, 5, TFT_RED);
  } else {
    tft.fillRoundRect(10, 10, 130, 30, 5, TFT_GREEN);
  }
  tft.setCursor(20, 33);
  tft.print(panelData[P_pmDestDCO1] == 0 ? "DCO1 Off" : "DCO1 On");

  if (panelData[P_pmDestFilter]) {
    digitalWrite(PM_FILT_ENV_DEST_LED, HIGH);
  } else if (!panelData[P_pmDestFilter]) {
    digitalWrite(PM_FILT_ENV_DEST_LED, LOW);
  }
  if (panelData[P_pmDestFilter]) {
    tft.fillRoundRect(180, 10, 130, 30, 5, TFT_RED);
  } else {
    tft.fillRoundRect(180, 10, 130, 30, 5, TFT_GREEN);
  }
  tft.setCursor(200, 33);
  tft.print(panelData[P_pmDestFilter] == 0 ? "Filter Off" : "Filter On");

  drawBar0(6, panelData[P_pmDCO2], NUM_STEPS, STEP_HEIGHT);
  drawBar0(52, panelData[P_pmFilterEnv], NUM_STEPS, STEP_HEIGHT);
  drawBar0(98, panelData[P_volumeControl], NUM_STEPS, STEP_HEIGHT);
  drawBar0(144, panelData[P_amDepth], NUM_STEPS, STEP_HEIGHT);
  drawBar0(192, panelData[P_ATDepth], NUM_STEPS, STEP_HEIGHT);
  drawBar0(236, panelData[P_modWheelDepth], NUM_STEPS, STEP_HEIGHT);
  drawBar1(282, panelData[P_PitchBendLevel], DET_STEPS, DET_STEP_HEIGHT);
}

void loop(void) {
  MIDI.read(MIDI_CHANNEL_OMNI);
}