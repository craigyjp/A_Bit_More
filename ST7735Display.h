// This Teensy3 native optimized version requires specific pins

#define cs 38
//#define cs 2
//#define dc 19
#define dc 3
#define rst 9
//#define rst 10
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

//ST7735_t3 tft = ST7735_t3(cs, dc, 26, 27, rst);
ST7789_t3 tft = ST7789_t3(cs, dc, 26, 27, rst);

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
  tft.println("POLYKIT");
  tft.setTextColor(ST7735_RED);
  tft.setFont(&FreeSans9pt7b);
  tft.setCursor(110, 95);
  tft.println(VERSION);
}

void renderCurrentPatchPage() {
  if (wholemode) {
    tft.fillScreen(ST7735_BLACK);
    tft.setFont(&FreeSansBold18pt7b);
    tft.setCursor(5, 29);
    tft.setTextColor(ST7735_YELLOW);
    tft.setTextSize(1);
    tft.println(currentPgmNumL);

    tft.setCursor(80, 19);
    tft.setFont(&FreeSans12pt7b);
    tft.setTextSize(1);
    tft.println("Whole");

    tft.setTextColor(ST7735_BLACK);
    tft.setFont(&Org_01);
    tft.drawFastHLine(10, 63, tft.width() - 20, ST7735_RED);

    tft.setFont(&FreeSans12pt7b);
    tft.setTextColor(ST7735_YELLOW);
    tft.setCursor(1, 55);
    tft.setTextColor(ST7735_WHITE);
    tft.println(currentPatchNameL);

  } else {

    tft.fillScreen(ST7735_BLACK);
    tft.setFont(&FreeSansBold18pt7b);
    tft.setCursor(5, 29);
    tft.setTextColor(ST7735_YELLOW);
    tft.setTextSize(1);
    tft.println(currentPgmNumU);

    tft.setCursor(80, 19);
    tft.setFont(&FreeSans12pt7b);
    tft.setTextSize(1);
    tft.println("Upper");

    tft.setTextColor(ST7735_BLACK);
    tft.setFont(&Org_01);
    tft.drawFastHLine(10, 63, tft.width() - 20, ST7735_RED);

    tft.setFont(&FreeSans12pt7b);
    tft.setTextColor(ST7735_YELLOW);
    tft.setCursor(1, 55);
    tft.setTextColor(ST7735_WHITE);
    tft.println(currentPatchNameU);

    tft.setFont(&FreeSansBold18pt7b);
    tft.setCursor(5, 97);
    tft.setTextColor(ST7735_YELLOW);
    tft.setTextSize(1);
    tft.println(currentPgmNumL);

    tft.setCursor(80, 87);
    tft.setFont(&FreeSans12pt7b);
    tft.setTextSize(1);
    tft.println("Lower");

    tft.setTextColor(ST7735_BLACK);
    tft.setFont(&Org_01);
    tft.drawFastHLine(10, 63, tft.width() - 20, ST7735_RED);

    tft.setFont(&FreeSans12pt7b);
    tft.setTextColor(ST7735_YELLOW);
    tft.setCursor(1, 122);
    tft.setTextColor(ST7735_WHITE);
    tft.println(currentPatchNameL);
  }
}

void renderCurrentParameterPage() {
  switch (state) {
    case PARAMETER:
      if (upperSW) {
        tft.fillScreen(ST7735_BLACK);
        tft.setFont(&FreeSans12pt7b);
        tft.setCursor(0, 29);
        tft.setTextColor(ST7735_YELLOW);
        tft.setTextSize(1);
        tft.println(currentParameter);
        tft.drawFastHLine(10, 63, tft.width() - 20, ST7735_RED);
        tft.setCursor(1, 55);
        tft.setTextColor(ST7735_WHITE);
        tft.println(currentValue);
        // lower patch
        tft.setFont(&FreeSansBold18pt7b);
        tft.setCursor(5, 97);
        tft.setTextColor(ST7735_YELLOW);
        tft.setTextSize(1);
        tft.println(currentPgmNumL);
        tft.setCursor(80, 87);
        tft.setFont(&FreeSans12pt7b);
        tft.setTextSize(1);
        tft.println("Lower");
        tft.setFont(&FreeSans12pt7b);
        tft.setTextColor(ST7735_YELLOW);
        tft.setCursor(1, 122);
        tft.setTextColor(ST7735_WHITE);
        tft.println(currentPatchNameL);
      } else {
        if (wholemode) {
          //upper whole mode patch
          tft.fillScreen(ST7735_BLACK);
          tft.setFont(&FreeSansBold18pt7b);
          tft.setCursor(5, 29);
          tft.setTextColor(ST7735_YELLOW);
          tft.setTextSize(1);
          tft.println(currentPgmNumL);

          tft.setCursor(80, 19);
          tft.setFont(&FreeSans12pt7b);
          tft.setTextSize(1);
          tft.println("Whole");

          tft.setTextColor(ST7735_BLACK);
          tft.setFont(&Org_01);
          tft.drawFastHLine(10, 63, tft.width() - 20, ST7735_RED);

          tft.setFont(&FreeSans12pt7b);
          tft.setTextColor(ST7735_YELLOW);
          tft.setCursor(1, 55);
          tft.setTextColor(ST7735_WHITE);
          tft.println(currentPatchNameL);
          
          // parameter in lower section
          tft.setFont(&FreeSans12pt7b);
          tft.setCursor(0, 97);
          tft.setTextColor(ST7735_YELLOW);
          tft.setTextSize(1);
          tft.println(currentParameter);
          tft.drawFastHLine(10, 63, tft.width() - 20, ST7735_RED);
          tft.setCursor(1, 122);
          tft.setTextColor(ST7735_WHITE);
          tft.println(currentValue);

        } else {
          tft.fillScreen(ST7735_BLACK);
          tft.setFont(&FreeSans12pt7b);
          tft.setCursor(0, 97);
          tft.setTextColor(ST7735_YELLOW);
          tft.setTextSize(1);
          tft.println(currentParameter);
          tft.drawFastHLine(10, 63, tft.width() - 20, ST7735_RED);
          tft.setCursor(1, 122);
          tft.setTextColor(ST7735_WHITE);
          tft.println(currentValue);
          // upper patch
          tft.setFont(&FreeSansBold18pt7b);
          tft.setCursor(5, 29);
          tft.setTextColor(ST7735_YELLOW);
          tft.setTextSize(1);
          tft.println(currentPgmNumU);
          tft.setCursor(80, 19);
          tft.setFont(&FreeSans12pt7b);
          tft.setTextSize(1);
          tft.println("Upper");
          tft.setFont(&FreeSans12pt7b);
          tft.setTextColor(ST7735_YELLOW);
          tft.setCursor(1, 55);
          tft.setTextColor(ST7735_WHITE);
          tft.println(currentPatchNameU);
        }
      }
      break;
  }
}

void renderDeletePatchPage() {
  tft.fillScreen(ST7735_BLACK);
  tft.setFont(&FreeSansBold18pt7b);
  tft.setCursor(5, 53);
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(1);
  tft.println("Delete?");
  tft.drawFastHLine(10, 60, tft.width() - 20, ST7735_RED);
  tft.setFont(&FreeSans9pt7b);
  tft.setCursor(0, 78);
  tft.setTextColor(ST7735_YELLOW);
  tft.println(patches.last().patchNo);
  tft.setCursor(35, 78);
  tft.setTextColor(ST7735_WHITE);
  tft.println(patches.last().patchName);
  tft.fillRect(0, 85, tft.width(), 23, ST7735_RED);
  tft.setCursor(0, 98);
  tft.setTextColor(ST7735_YELLOW);
  tft.println(patches.first().patchNo);
  tft.setCursor(35, 98);
  tft.setTextColor(ST7735_WHITE);
  tft.println(patches.first().patchName);
}

void renderDeleteMessagePage() {
  tft.fillScreen(ST7735_BLACK);
  tft.setFont(&FreeSans12pt7b);
  tft.setCursor(2, 53);
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(1);
  tft.println("Renumbering");
  tft.setCursor(10, 90);
  tft.println("SD Card");
}

void renderSavePage() {
  tft.fillScreen(ST7735_BLACK);
  tft.setFont(&FreeSansBold18pt7b);
  tft.setCursor(5, 53);
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(1);
  tft.println("Save?");
  tft.drawFastHLine(10, 60, tft.width() - 20, ST7735_RED);
  tft.setFont(&FreeSans9pt7b);
  tft.setCursor(0, 78);
  tft.setTextColor(ST7735_YELLOW);
  tft.println(patches[patches.size() - 2].patchNo);
  tft.setCursor(35, 78);
  tft.setTextColor(ST7735_WHITE);
  tft.println(patches[patches.size() - 2].patchName);
  tft.fillRect(0, 85, tft.width(), 23, ST7735_RED);
  tft.setCursor(0, 98);
  tft.setTextColor(ST7735_YELLOW);
  tft.println(patches.last().patchNo);
  tft.setCursor(35, 98);
  tft.setTextColor(ST7735_WHITE);
  tft.println(patches.last().patchName);
}

void renderReinitialisePage() {
  tft.fillScreen(ST7735_BLACK);
  tft.setFont(&FreeSans12pt7b);
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(1);
  tft.setCursor(5, 53);
  tft.println("Initialise to");
  tft.setCursor(5, 90);
  tft.println("panel setting");
}

void renderPatchNamingPage() {
  tft.fillScreen(ST7735_BLACK);
  tft.setFont(&FreeSans12pt7b);
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(1);
  tft.setCursor(0, 53);
  tft.println("Rename Patch");
  tft.drawFastHLine(10, 62, tft.width() - 20, ST7735_RED);
  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(5, 90);
  tft.println(newPatchName);
}

void renderRecallPage() {
  tft.fillScreen(ST7735_BLACK);
  tft.setFont(&FreeSans9pt7b);
  tft.setCursor(0, 45);
  tft.setTextColor(ST7735_YELLOW);
  tft.println(patches.last().patchNo);
  tft.setCursor(35, 45);
  tft.setTextColor(ST7735_WHITE);
  tft.println(patches.last().patchName);

  tft.fillRect(0, 56, tft.width(), 23, 0xA000);
  tft.setCursor(0, 72);
  tft.setTextColor(ST7735_YELLOW);
  tft.println(patches.first().patchNo);
  tft.setCursor(35, 72);
  tft.setTextColor(ST7735_WHITE);
  tft.println(patches.first().patchName);

  tft.setCursor(0, 98);
  tft.setTextColor(ST7735_YELLOW);
  patches.size() > 1 ? tft.println(patches[1].patchNo) : tft.println(patches.last().patchNo);
  tft.setCursor(35, 98);
  tft.setTextColor(ST7735_WHITE);
  patches.size() > 1 ? tft.println(patches[1].patchName) : tft.println(patches.last().patchName);
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


void renderSettingsPage() {
  tft.fillScreen(ST7735_BLACK);
  tft.setFont(&FreeSans12pt7b);
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(1);
  tft.setCursor(0, 53);
  tft.println(currentSettingsOption);
  if (currentSettingsPart == SETTINGS) renderUpDown(140, 42, ST7735_YELLOW);
  tft.drawFastHLine(10, 62, tft.width() - 20, ST7735_RED);
  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(5, 90);
  tft.println(currentSettingsValue);
  if (currentSettingsPart == SETTINGSVALUE) renderUpDown(140, 80, ST7735_WHITE);
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

void showPatchPage(String numberU, String patchNameU, String numberL, String patchNameL) {
  currentPgmNumU = numberU;
  currentPatchNameU = patchNameU;
  currentPgmNumL = numberL;
  currentPatchNameL = patchNameL;
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
    }
    tft.updateScreen();
  }
}

void setupDisplay() {
  tft.init(240, 320);
  tft.useFrameBuffer(true);
  
  //tft.initR(INITR_BLACKTAB);
  tft.setRotation(3);
  tft.invertDisplay(true);
  tft.fillScreen(ST7735_BLACK);
  renderBootUpPage();
  tft.updateScreen();
  threads.addThread(displayThread);
}
