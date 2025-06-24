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

void drawThickLine(int x0, int y0, int x1, int y1, uint16_t color, int thickness) {
  int t = thickness / 2;
  for (int dx = -t; dx <= t; dx++) {
    for (int dy = -t; dy <= t; dy++) {
      tft.drawLine(x0 + dx, y0 + dy, x1 + dx, y1 + dy, color);
    }
  }
}

int interpExpo(int start, int end, float t, float curve) {
  float shaped = 1.0 - pow(1.0 - t, curve);  // classic synth curve
  return round(start + (end - start) * shaped);
}

int interpLinear(int start, int end, float t) {
  return round(start + (end - start) * t);
}

// Interpolate for attack curve (convex for exp, linear for curve=1)
int interpAttack(int start, int end, float t, float curve) {
  // curve=1: linear; curve>1: convex (exponential); curve<1: concave (log)
  float shaped = 1.0 - pow(1.0 - t, curve);
  return round(start + (end - start) * shaped);
}

// Interpolate for decay/release (concave for exp, linear for curve=1)
int interpDecayRelease(int start, int end, float t, float curve) {
  float shaped = pow(t, curve);
  return round(start + (end - start) * shaped);
}

void drawThickCurve(int x0, int y0, int x1, int y1, uint16_t color, int thickness, float curve, bool exponential) {
  const int steps = 16;
  for (int i = 0; i < steps; ++i) {
    float t1 = (float)i / steps;
    float t2 = (float)(i + 1) / steps;
    int xa = round(x0 + (x1 - x0) * t1);
    int xb = round(x0 + (x1 - x0) * t2);
    int ya, yb;
    if (exponential) {
      ya = interpExpo(y0, y1, t1, curve);
      yb = interpExpo(y0, y1, t2, curve);
    } else {
      ya = interpLinear(y0, y1, t1);
      yb = interpLinear(y0, y1, t2);
    }
    drawThickLine(xa, ya, xb, yb, color, thickness);
  }
}

void drawEnvADSR(
  int attack, int decay, int sustain, int release,
  int screenW, int baseY, int envHeight, int thickness,
  bool isExponential) {

  float fatt = constrain(attack / 1023.0, 0, 1);
  float fdec = constrain(decay / 1023.0, 0, 1);
  float fsus = constrain(sustain / 1023.0, 0, 1);
  float frel = constrain(release / 1023.0, 0, 1);

  float totalFrac = fatt + fdec + frel + 0.5;
  float attW = (fatt / totalFrac) * (screenW - 1);
  float decW = (fdec / totalFrac) * (screenW - 1);
  float susW = (0.5 / totalFrac) * (screenW - 1);
  float relW = (frel / totalFrac) * (screenW - 1);

  int x0 = 10;  // Now starts at x = 10
  int y0 = baseY;

  int x1 = x0 + attW;
  int y1 = baseY - envHeight;

  int x2 = x1 + decW;
  int y2 = baseY - (fsus * envHeight);

  int x3 = x2 + susW;
  int y3 = y2;

  int x4 = min(x3 + relW, x0 + screenW - 1);
  int y4 = baseY;

  uint16_t envColor = getActiveColor();

  int margin = thickness + 2;
  tft.fillRect(
    x0 - margin,                 // left edge
    baseY - envHeight - margin,  // top edge
    screenW + margin * 2,        // width
    envHeight + margin * 2,      // height
    TFT_BLACK);

  float curve = isExponential ? 2.0 : 1.0;

  drawThickCurve(x0, y0, x1, y1, envColor, thickness, curve, isExponential);
  drawThickCurve(x1, y1, x2, y2, envColor, thickness, curve, isExponential);
  drawThickLine(x2, y2, x3, y3, envColor, thickness);
  drawThickCurve(x3, y3, x4, y4, envColor, thickness, curve, isExponential);
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
      drawEnvADSR(
        panelData[P_ampAttack],
        panelData[P_ampDecay],
        panelData[P_ampSustain],
        panelData[P_ampRelease],
        300, 236, 130, 4,
        panelData[P_ampLogLin] != 0  // true=exponential, false=linear
      );
      break;

    case CCampDecay:
      panelData[P_ampDecay] = value;
      panelData[P_oldampDecay] = value;
      drawEnvADSR(
        panelData[P_ampAttack],
        panelData[P_ampDecay],
        panelData[P_ampSustain],
        panelData[P_ampRelease],
        300, 236, 130, 4,
        panelData[P_ampLogLin] != 0  // true=exponential, false=linear
      );
      break;

    case CCampSustain:
      panelData[P_ampSustain] = value;
      panelData[P_oldampSustain] = value;
      drawEnvADSR(
        panelData[P_ampAttack],
        panelData[P_ampDecay],
        panelData[P_ampSustain],
        panelData[P_ampRelease],
        300, 236, 130, 4,
        panelData[P_ampLogLin] != 0  // true=exponential, false=linear
      );
      break;

    case CCampRelease:
      panelData[P_ampRelease] = value;
      panelData[P_oldampRelease] = value;
      drawEnvADSR(
        panelData[P_ampAttack],
        panelData[P_ampDecay],
        panelData[P_ampSustain],
        panelData[P_ampRelease],
        300, 236, 130, 4,
        panelData[P_ampLogLin] != 0  // true=exponential, false=linear
      );
      break;
  }
}

void myLEDupdate(byte channel, byte control, int value) {
  switch (control) {

    case CCvcaVel:
      panelData[P_vcaVel] = value;
      if (!panelData[P_vcaVel]) {
        tft.fillRoundRect(10, 50, 130, 30, 5, TFT_GREEN);  // Green box for off
      } else {
        tft.fillRoundRect(10, 50, 130, 30, 5, TFT_RED);  // Red box for on
      }
      tft.setCursor(30, 73);
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
      drawEnvADSR(
        panelData[P_ampAttack],
        panelData[P_ampDecay],
        panelData[P_ampSustain],
        panelData[P_ampRelease],
        300, 236, 130, 4,
        panelData[P_ampLogLin] != 0  // true=exponential, false=linear
      );

      break;

    case CCampenvLinLogSW:
      panelData[P_ampLogLin] = value;
      if (!panelData[P_ampLogLin]) {
        tft.fillRoundRect(180, 50, 130, 30, 5, TFT_GREEN);  // Green box for off
      } else {
        tft.fillRoundRect(180, 50, 130, 30, 5, TFT_RED);  // Red box for on
      }
      tft.setCursor(210, 73);
      tft.print(panelData[P_ampLogLin] == 0 ? "Env Lin" : "Env Log");
      drawEnvADSR(
        panelData[P_ampAttack],
        panelData[P_ampDecay],
        panelData[P_ampSustain],
        panelData[P_ampRelease],
        300, 236, 130, 4,
        panelData[P_ampLogLin] != 0  // true=exponential, false=linear
      );
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

  tft.setFreeFont(&FreeSans12pt7b);
  tft.setTextSize(1);
  tft.setTextColor(TFT_BLACK);  // Change text color to black for better contrast

  if (!panelData[P_vcaVel]) {
    tft.fillRoundRect(10, 50, 130, 30, 5, TFT_GREEN);  // Green box for off
  } else {
    tft.fillRoundRect(10, 50, 130, 30, 5, TFT_RED);  // Red box for on
  }
  tft.setCursor(30, 73);
  tft.print(panelData[P_filterVel] == 0 ? "Vel Off" : "Vel On");

  if (panelData[P_vcaGate] == 0) {
    tft.fillRoundRect(10, 10, 130, 30, 5, TFT_GREEN);  // Green box for off
  } else {
    tft.fillRoundRect(10, 10, 130, 30, 5, TFT_RED);  // Red box for on
  }
  tft.setCursor(30, 33);
  tft.print(panelData[P_vcaGate] == 0 ? "Gate Off" : "Gate On");

  if (!panelData[P_ampLogLin]) {
    tft.fillRoundRect(180, 50, 130, 30, 5, TFT_GREEN);  // Green box for off
  } else {
    tft.fillRoundRect(180, 50, 130, 30, 5, TFT_RED);  // Red box for on
  }
  tft.setCursor(210, 73);
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

  drawEnvADSR(
    panelData[P_ampAttack],
    panelData[P_ampDecay],
    panelData[P_ampSustain],
    panelData[P_ampRelease],
    300, 236, 130, 4,
    panelData[P_ampLogLin] != 0  // true=exponential, false=linear
  );
}

void loop(void) {
  MIDI.read(MIDI_CHANNEL_OMNI);
}
