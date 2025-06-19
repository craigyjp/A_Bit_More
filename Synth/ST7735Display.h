// This Teensy3 native optimized version requires specific pins

#define cs 38
#define dc 3
#define rst 9

#define DISPLAYTIMEOUT 1500

#include "TeensyThreads.h"
#include <Adafruit_GFX.h>
#include <ST7735_t3.h>  // Hardware-specific library
#include <ST7789_t3.h>  // Hardware-specific library

#include <Fonts/Org_01.h>
#include "Yeysk16pt7b.h"
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansOblique24pt7b.h>
#include <Fonts/FreeSansBoldOblique24pt7b.h>

#define PULSE 1
#define VAR_TRI 2
#define FILTER_ENV 3
#define AMP_ENV 4

ST7789_t3 tft = ST7789_t3(cs, dc, 26, 27, rst);

extern Performance currentPerformance;
extern CircularBuffer<Performance, PERFORMANCES_LIMIT> performances;

String currentPerfNum = "";
String currentPerfName = "";
int currentUpperPatchNo = 0;
String currentUpperPatchName = "";
int currentLowerPatchNo = 0;
String currentLowerPatchName = "";

String currentParameter = "";
String currentValue = "";
float currentFloatValue = 0.0;
String currentPgmNumU = "";
String currentPgmNumL = "";
String currentPatchNameU = "";
String currentPatchNameL = "";
String newPatchName = "";
const char *currentSettingsOption = "";
const char *currentSettingsValue = "";
int currentSettingsPart = SETTINGS;
int paramType = PARAMETER;

//boolean voiceOn[NO_OF_VOICES] = { false };
boolean MIDIClkSignal = false;

unsigned long timer = 0;

void startTimer() {
  if (state == PARAMETER) {
    timer = millis();
  }
}

void renderBootUpPage() {
  tft.fillScreen(ST7735_BLACK);
  tft.drawRect(42, 30, 46, 11, ST7735_WHITE);
  tft.fillRect(88, 30, 61, 11, ST7735_WHITE);
  tft.setCursor(45, 31);
  tft.setFont(&Org_01);
  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE);
  tft.println("HYBRID");
  tft.setTextColor(ST7735_BLACK);
  tft.setCursor(91, 37);
  tft.println("SYNTHESIZER");
  tft.setTextColor(ST7735_YELLOW);
  tft.setFont(&Yeysk16pt7b);
  tft.setCursor(0, 70);
  tft.setTextSize(1);
  tft.println("A Bit More");
  tft.setTextColor(ST7735_RED);
  tft.setFont(&FreeSans9pt7b);
  tft.setCursor(110, 95);
  tft.println(VERSION);
}

void renderPerformancePage() {
  tft.fillScreen(ST7735_BLACK);
  tft.drawFastHLine(0, 40, tft.width(), ST7735_RED);
  tft.drawFastHLine(0, 140, tft.width(), ST7735_RED);

  tft.setTextColor(ST7735_YELLOW);
  tft.setFont(&FreeSans12pt7b);
  tft.setTextSize(1);
  tft.setCursor(5, 10);
  tft.println("Perf No");

  tft.setCursor(100, 10);
  tft.println("Name");

  tft.setCursor(5, 70);
  tft.setTextSize(3);
  tft.println(currentPerfNum);

  tft.setCursor(100, 75);
  tft.setFont(&FreeSans9pt7b);
  tft.setTextSize(2);
  tft.setTextColor(ST7735_WHITE);
  tft.println(currentPerfName);

  tft.setCursor(5, 160);
  tft.setTextSize(2);
  tft.setTextColor(ST7735_YELLOW);
  tft.println("Upp:");

  tft.setCursor(100, 160);
  tft.setTextColor(ST7735_WHITE);
  tft.println(String(currentUpperPatchNo) + " " + currentUpperPatchName);

  tft.setCursor(5, 200);
  tft.setTextColor(ST7735_YELLOW);
  tft.println("Low:");

  tft.setCursor(100, 200);
  tft.setTextColor(ST7735_WHITE);
  tft.println(String(currentLowerPatchNo) + " " + currentLowerPatchName);
}

void renderCurrentPatchPage() {
  tft.fillScreen(ST7735_BLACK);

  // ─── Section Dividers ──────────────────────
  tft.drawFastHLine(0, 50, tft.width(), ST7735_RED);
  tft.drawFastHLine(0, 140, tft.width(), ST7735_RED);

  // ─── Header Line: Num / Label / Mode ───────
  tft.setTextColor(ST7735_YELLOW);
  tft.setFont(&FreeSans12pt7b);
  tft.setTextSize(1);

  tft.setCursor(5, 20);
  tft.println("Num");

  tft.setCursor(70, 20);  // aligned with parameter page
  if (inPerformanceMode) {
    tft.print("Performance ");
    tft.print(currentPerformance.performanceNo);
  } else {
    tft.println("Patchname");
  }

  tft.setCursor(240, 20);
  switch (playMode) {
    case 0: tft.println("Whole"); break;
    case 1: tft.println("Dual"); break;
    case 2: tft.println("Split"); break;
  }

  // ─── Patch Display ─────────────────────────
  if (!wholemode) {
    // Upper patch block
    tft.setFont(&FreeSans12pt7b);
    tft.setCursor(0, 74);
    tft.setTextSize(3);
    tft.setTextColor(ST7735_YELLOW);
    tft.println(currentPgmNumU);

    tft.setCursor(100, 79);
    tft.setFont(&FreeSans9pt7b);
    tft.setTextSize(2);
    tft.setTextColor(ST7735_WHITE);
    tft.println(currentPatchNameU);
  }

  // Lower patch block (always shown)
  tft.setFont(&FreeSans12pt7b);
  tft.setCursor(0, 170);
  tft.setTextSize(3);
  tft.setTextColor(ST7735_YELLOW);
  tft.println(currentPgmNumL);

  tft.setCursor(100, 175);
  tft.setFont(&FreeSans9pt7b);
  tft.setTextSize(2);
  tft.setTextColor(ST7735_WHITE);
  tft.println(currentPatchNameL);
}

void renderCurrentParameterPage() {
  tft.fillScreen(ST7735_BLACK);

  // ─── Section Dividers ──────────────────────
  tft.drawFastHLine(0, 50, tft.width(), ST7735_RED);
  tft.drawFastHLine(0, 140, tft.width(), ST7735_RED);

  // ─── Header Line: Num / Label / Mode ───────
  tft.setTextColor(ST7735_YELLOW);
  tft.setFont(&FreeSans12pt7b);
  tft.setTextSize(1);

  tft.setCursor(5, 20);
  tft.println("Num");

  tft.setCursor(70, 20);  // moved left
  if (inPerformanceMode) {
    tft.print("Performance ");
    tft.print(currentPerformance.performanceNo);
  } else {
    tft.println("Patchname");
  }

  tft.setCursor(240, 20);  // top-right corner
  switch (playMode) {
    case 0: tft.println("Whole"); break;
    case 1: tft.println("Dual"); break;
    case 2: tft.println("Split"); break;
  }

  // ─── Patch Display ─────────────────────────
  if (!wholemode) {
    // Upper patch
    tft.setFont(&FreeSans12pt7b);
    tft.setCursor(0, 74);
    tft.setTextSize(3);
    tft.setTextColor(ST7735_YELLOW);
    tft.println(currentPgmNumU);

    tft.setCursor(100, 79);
    tft.setFont(&FreeSans9pt7b);
    tft.setTextSize(2);
    tft.setTextColor(ST7735_WHITE);
    tft.println(currentPatchNameU);
  }

  // Lower patch (always shown)
  tft.setFont(&FreeSans12pt7b);
  tft.setCursor(0, 170);
  tft.setTextSize(3);
  tft.setTextColor(ST7735_YELLOW);
  tft.println(currentPgmNumL);

  tft.setCursor(100, 175);
  tft.setFont(&FreeSans9pt7b);
  tft.setTextSize(2);
  tft.setTextColor(ST7735_WHITE);
  tft.println(currentPatchNameL);

  // ─── Parameter Display (erase before draw) ─
  if (upperSW) {
    // Editing upper → show param in LOWER section
    tft.fillRect(0, 160, tft.width(), 80, ST7735_BLACK);  // FIX: covers lower section
    tft.setCursor(0, 170);
    tft.setTextColor(ST7735_YELLOW);
    tft.setTextSize(2);
    tft.println(currentParameter);

    tft.setCursor(0, 200);
    tft.setTextColor(ST7735_WHITE);
    tft.println(currentValue);
  } else {
    // Editing lower → show param in UPPER section
    tft.fillRect(0, 60, tft.width(), 80, ST7735_BLACK);  // FIX: covers upper section fully
    tft.setCursor(0, 70);
    tft.setTextColor(ST7735_YELLOW);
    tft.setTextSize(2);
    tft.println(currentParameter);

    tft.setCursor(0, 100);
    tft.setTextColor(ST7735_WHITE);
    tft.println(currentValue);
  }
}

void renderPerformanceDeletePage() {
  tft.fillScreen(ST7735_BLACK);
  tft.setFont(&FreeSansBold18pt7b);
  tft.setCursor(10, 20);
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(1);
  tft.println("Delete Perf?");
  tft.drawFastHLine(10, 50, tft.width() - 20, ST7735_RED);

  tft.setTextSize(2);
  tft.setFont(&FreeSans9pt7b);
  tft.setCursor(10, 80);
  tft.setTextColor(ST7735_YELLOW);
  tft.println(performances.last().performanceNo);
  tft.setCursor(100, 80);
  tft.setTextColor(ST7735_WHITE);
  tft.println(performances.last().name);

  tft.fillRect(10, 120, tft.width() - 20, 44, ST77XX_RED);

  tft.setCursor(10, 130);
  tft.setTextColor(ST7735_YELLOW);
  tft.println(performances.first().performanceNo);
  tft.setCursor(100, 130);
  tft.setTextColor(ST7735_WHITE);
  tft.println(performances.first().name);

  tft.setCursor(10, 180);
  tft.setTextColor(ST7735_YELLOW);
  performances.size() > 1 ? tft.println(performances[1].performanceNo) : tft.println(performances.last().performanceNo);
  tft.setCursor(100, 180);
  tft.setTextColor(ST7735_WHITE);
  performances.size() > 1 ? tft.println(performances[1].name) : tft.println(performances.last().name);
}

// void renderCurrentPatchPage() {

//   tft.fillScreen(ST7735_BLACK);
//   tft.drawFastHLine(0, 40, tft.width(), ST7735_RED);
//   tft.drawFastHLine(0, 140, tft.width(), ST7735_RED);

//   tft.setTextColor(ST7735_YELLOW);
//   tft.setFont(&FreeSans12pt7b);
//   tft.setTextSize(1);
//   tft.setCursor(5, 10);
//   tft.println("Number");
//   tft.setCursor(100, 10);
//   tft.println(inPerformanceMode ? "Performance" : "Patchname");
//   switch (playMode) {
//     case 0:
//       tft.setCursor(240, 10);
//       tft.println("Whole");
//       break;

//     case 1:
//       tft.setCursor(240, 10);
//       tft.println("Dual");
//       break;

//     case 2:
//       tft.setCursor(240, 10);
//       tft.println("Split");
//       break;
//   }
//   if (wholemode) {

//     tft.setFont(&FreeSans12pt7b);
//     tft.setCursor(0, 70);
//     tft.setTextSize(3);
//     tft.setTextColor(ST7735_YELLOW);
//     tft.println(currentPgmNumL);

//     tft.setCursor(100, 75);
//     tft.setFont(&FreeSans9pt7b);
//     tft.setTextSize(2);
//     tft.setTextColor(ST7735_WHITE);
//     tft.println(currentPatchNameL);

//   } else {

//     tft.setFont(&FreeSans12pt7b);
//     tft.setCursor(0, 70);
//     tft.setTextSize(3);
//     tft.setTextColor(ST7735_YELLOW);
//     tft.println(currentPgmNumU);

//     tft.setCursor(100, 75);
//     tft.setFont(&FreeSans9pt7b);
//     tft.setTextSize(2);
//     tft.setTextColor(ST7735_WHITE);
//     tft.println(currentPatchNameU);

//     tft.setFont(&FreeSans12pt7b);
//     tft.setCursor(0, 170);
//     tft.setTextSize(3);
//     tft.setTextColor(ST7735_YELLOW);
//     tft.println(currentPgmNumL);

//     tft.setCursor(100, 175);
//     tft.setFont(&FreeSans9pt7b);
//     tft.setTextSize(2);
//     tft.setTextColor(ST7735_WHITE);
//     tft.println(currentPatchNameL);
//   }
// }

// void renderCurrentParameterPage() {

//   tft.fillScreen(ST7735_BLACK);
//   tft.drawFastHLine(0, 40, tft.width(), ST7735_RED);
//   tft.drawFastHLine(0, 140, tft.width(), ST7735_RED);

//   tft.setTextColor(ST7735_YELLOW);
//   tft.setFont(&FreeSans12pt7b);
//   tft.setTextSize(1);
//   tft.setCursor(5, 10);
//   tft.println("Number");
//   tft.setCursor(100, 10);
//   tft.println(inPerformanceMode ? "Performance" : "Patchname");
//   switch (playMode) {
//     case 0:
//       tft.setCursor(240, 10);
//       tft.println("Whole");
//       break;

//     case 1:
//       tft.setCursor(240, 10);
//       tft.println("Dual");
//       break;

//     case 2:
//       tft.setCursor(240, 10);
//       tft.println("Split");
//       break;
//   }

//   switch (state) {
//     case PARAMETER:
//       if (upperSW) {
//         tft.setFont(&FreeSans12pt7b);
//         tft.setCursor(0, 70);
//         tft.setTextSize(3);
//         tft.setTextColor(ST7735_YELLOW);
//         tft.println(currentPgmNumU);

//         tft.setCursor(100, 75);
//         tft.setFont(&FreeSans9pt7b);
//         tft.setTextSize(2);
//         tft.setTextColor(ST7735_WHITE);
//         tft.println(currentPatchNameU);

//         // parameter in lower section
//         tft.setCursor(0, 165);
//         tft.setTextColor(ST7735_YELLOW);
//         tft.setTextSize(2);
//         tft.println(currentParameter);

//         tft.setCursor(0, 205);
//         tft.setTextColor(ST7735_WHITE);
//         tft.println(currentValue);

//       } else {
//         if (wholemode) {
//           //upper whole mode patch
//           tft.setFont(&FreeSans12pt7b);
//           tft.setCursor(0, 70);
//           tft.setTextSize(3);
//           tft.setTextColor(ST7735_YELLOW);
//           tft.println(currentPgmNumL);

//           tft.setCursor(100, 75);
//           tft.setFont(&FreeSans9pt7b);
//           tft.setTextSize(2);
//           tft.setTextColor(ST7735_WHITE);
//           tft.println(currentPatchNameL);

//           // parameter in lower section
//           tft.setCursor(0, 165);
//           tft.setTextColor(ST7735_YELLOW);
//           tft.setTextSize(2);
//           tft.println(currentParameter);

//           tft.setCursor(0, 205);
//           tft.setTextColor(ST7735_WHITE);
//           tft.println(currentValue);

//         } else {

//           // lower patch
//           tft.setFont(&FreeSans12pt7b);
//           tft.setCursor(0, 170);
//           tft.setTextSize(3);
//           tft.setTextColor(ST7735_YELLOW);
//           tft.println(currentPgmNumL);

//           tft.setCursor(100, 175);
//           tft.setFont(&FreeSans9pt7b);
//           tft.setTextSize(2);
//           tft.setTextColor(ST7735_WHITE);
//           tft.println(currentPatchNameL);

//           tft.setCursor(0, 65);
//           tft.setTextColor(ST7735_YELLOW);
//           tft.setTextSize(2);
//           tft.println(currentParameter);

//           tft.setCursor(0, 105);
//           tft.setTextColor(ST7735_WHITE);
//           tft.println(currentValue);
//         }
//       }
//       break;
//   }
// }

void renderDeletePatchPage() {
  tft.fillScreen(ST7735_BLACK);
  tft.setFont(&FreeSansBold18pt7b);
  tft.setCursor(10, 20);
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(1);
  tft.println("Delete?");
  tft.drawFastHLine(10, 50, tft.width() - 20, ST7735_RED);

  tft.setTextSize(2);
  tft.setFont(&FreeSans9pt7b);
  tft.setCursor(10, 80);
  tft.setTextColor(ST7735_YELLOW);
  tft.println(patches.last().patchNo);
  tft.setCursor(100, 80);
  tft.setTextColor(ST7735_WHITE);
  tft.println(patches.last().patchName);

  tft.fillRect(10, 120, tft.width() - 20, 44, ST77XX_RED);

  tft.setCursor(10, 130);
  tft.setTextColor(ST7735_YELLOW);
  tft.println(patches.first().patchNo);
  tft.setCursor(100, 130);
  tft.setTextColor(ST7735_WHITE);
  tft.println(patches.first().patchName);
}

void renderDeleteMessagePage() {
  tft.fillScreen(ST7735_BLACK);
  tft.setFont(&FreeSans12pt7b);
  tft.setCursor(10, 20);
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(1);
  tft.println("Renumbering");
  tft.setCursor(10, 80);
  tft.println("SD Card");
}

void renderSavePage() {
  tft.fillScreen(ST7735_BLACK);
  tft.setFont(&FreeSansBold18pt7b);
  tft.setCursor(10, 20);
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(1);
  tft.println("Save?");
  tft.drawFastHLine(10, 50, tft.width() - 20, ST7735_RED);

  tft.setTextSize(2);
  tft.setFont(&FreeSans9pt7b);
  tft.setCursor(10, 80);
  tft.setTextColor(ST7735_YELLOW);
  tft.println(patches[patches.size() - 2].patchNo);
  tft.setCursor(100, 80);
  tft.setTextColor(ST7735_WHITE);
  tft.println(patches[patches.size() - 2].patchName);

  tft.fillRect(10, 120, tft.width() - 20, 44, ST77XX_RED);

  tft.setCursor(10, 130);
  tft.setTextColor(ST7735_YELLOW);
  tft.println(patches.last().patchNo);
  tft.setCursor(100, 130);
  tft.setTextColor(ST7735_WHITE);
  tft.println(patches.last().patchName);
}

void renderReinitialisePage() {
  tft.fillScreen(ST7735_BLACK);
  tft.setFont(&FreeSans12pt7b);
  tft.setCursor(10, 20);
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(1);
  tft.println("Initialise to");
  tft.setCursor(10, 80);
  tft.println("panel settings");
}

void renderPatchNamingPage() {
  tft.fillScreen(ST7735_BLACK);
  tft.setFont(&FreeSansBold18pt7b);
  tft.setCursor(10, 20);
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(1);
  tft.println("Rename Patch");
  tft.drawFastHLine(10, 50, tft.width() - 20, ST7735_RED);

  tft.setTextSize(2);
  tft.setFont(&FreeSans9pt7b);
  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(10, 80);
  tft.println(newPatchName);
}

void renderRecallPage() {
  tft.fillScreen(ST7735_BLACK);
  tft.setFont(&FreeSansBold18pt7b);
  tft.setCursor(10, 20);
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(1);
  tft.println("Recall?");
  tft.drawFastHLine(10, 50, tft.width() - 20, ST7735_RED);

  tft.setTextSize(2);
  tft.setFont(&FreeSans9pt7b);

  // Upper patch display
  tft.setCursor(10, 80);
  tft.setTextColor(ST7735_YELLOW);
  tft.println(currentPgmNumU);
  tft.setCursor(100, 80);
  tft.setTextColor(ST7735_WHITE);
  tft.println(currentPatchNameU);

  // Divider
  tft.fillRect(10, 120, tft.width() - 20, 4, ST77XX_RED);

  // Lower patch display
  tft.setCursor(10, 140);
  tft.setTextColor(ST7735_YELLOW);
  tft.println(currentPgmNumL);
  tft.setCursor(100, 140);
  tft.setTextColor(ST7735_WHITE);
  tft.println(currentPatchNameL);
}

void showRenamingPage(String newName) {
  newPatchName = newName;
}

void renderUpDown(uint16_t x, uint16_t y, uint16_t colour) {
  //Produces up/down indicator glyph at x,y
  tft.setCursor(x, y);
  tft.fillTriangle(x, y, x + 8, y - 8, x + 16, y, colour);
  tft.fillTriangle(x, y + 4, x + 8, y + 12, x + 16, y + 4, colour);
}

void renderPerformanceNamingPage() {
  tft.fillScreen(ST7735_BLACK);
  tft.setFont(&FreeSansBold18pt7b);
  tft.setCursor(10, 20);
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(1);
  tft.println("Rename Perf");
  tft.drawFastHLine(10, 50, tft.width() - 20, ST7735_RED);

  tft.setTextSize(2);
  tft.setFont(&FreeSans9pt7b);
  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(10, 80);
  tft.println(newPatchName);  // reused for performance too
}

void renderSettingsPage() {
  tft.fillScreen(ST7735_BLACK);
  tft.setFont(&FreeSansBold18pt7b);
  tft.setCursor(10, 20);
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(1);
  tft.println("Settings");
  tft.drawFastHLine(10, 50, tft.width() - 20, ST7735_RED);
  tft.setFont(&FreeSans9pt7b);
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(2);
  tft.setCursor(10, 80);
  tft.println(currentSettingsOption);
  if (currentSettingsPart == SETTINGS) renderUpDown(240, 90, ST7735_YELLOW);
  tft.drawFastHLine(10, 125, tft.width() - 20, ST7735_RED);
  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(10, 150);
  tft.println(currentSettingsValue);
  if (currentSettingsPart == SETTINGSVALUE) renderUpDown(240, 160, ST7735_WHITE);
}

void showCurrentParameterPage(const char *param, float val, int pType) {
  currentParameter = param;
  currentValue = String(val);
  currentFloatValue = val;
  paramType = pType;
  startTimer();
}

void showCurrentParameterPage(const char *param, String val, int pType) {
  if (state == SETTINGS || state == SETTINGSVALUE) state = PARAMETER;  //Exit settings page if showing
  currentParameter = param;
  currentValue = val;
  paramType = pType;
  startTimer();
}

void showCurrentParameterPage(const char *param, String val) {
  showCurrentParameterPage(param, val, PARAMETER);
}

void showPatchPage(String numberU, String nameU, String numberL, String nameL) {
  currentPgmNumU = numberU;
  currentPatchNameU = nameU;
  currentPgmNumL = numberL;
  currentPatchNameL = nameL;
}

void showSettingsPage(const char *option, const char *value, int settingsPart) {
  currentSettingsOption = option;
  currentSettingsValue = value;
  currentSettingsPart = settingsPart;
}

void displayThread() {
  threads.delay(2000);  //Give bootup page chance to display
  while (1) {
    switch (state) {
      case PARAMETER:
        if ((millis() - timer) > DISPLAYTIMEOUT) {
          renderCurrentPatchPage();
        } else {
          renderCurrentParameterPage();
        }
        break;
      case RECALL:
        renderRecallPage();
        break;
      case SAVE:
        renderSavePage();
        break;
      case REINITIALISE:
        renderReinitialisePage();
        tft.updateScreen();  //update before delay
        threads.delay(1000);
        state = PARAMETER;
        break;
      case PATCHNAMING:
        renderPatchNamingPage();
        break;
      case PATCH:
        renderCurrentPatchPage();
        break;
      case DELETE:
        renderDeletePatchPage();
        break;
      case DELETEMSG:
        renderDeleteMessagePage();
        break;
      case SETTINGS:
      case SETTINGSVALUE:
        renderSettingsPage();
        break;
      case PERFORMANCE_RECALL:
      case PERFORMANCE_EDIT:
      case PERFORMANCE_SAVE:
        renderPerformancePage();
        break;
      case PERFORMANCE_NAMING:
        renderPerformanceNamingPage();  // see below
        break;
      case PERFORMANCE_DELETE:
        renderPerformanceDeletePage();
        break;
      case PERFORMANCE_DELETEMSG:
        // Handled inside checkSwitches() (already shows a message & delay)
        break;
    }
    tft.updateScreen();
  }
}

void setupDisplay() {
  tft.init(240, 320);
  tft.useFrameBuffer(true);
  tft.setRotation(3);
  tft.invertDisplay(true);
  tft.fillScreen(ST7735_BLACK);
  renderBootUpPage();
  tft.updateScreen();
  threads.addThread(displayThread);
}
