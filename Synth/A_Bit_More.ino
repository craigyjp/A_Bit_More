#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <MIDI.h>
#include <USBHost_t36.h>
#include "MidiCC.h"
#include "Constants.h"
#include "Parameters.h"
#include "PatchMgr.h"
#include "Button.h"
#include "HWControls.h"
#include "EepromMgr.h"
#include "Settings.h"
#include <RoxMux.h>
#include <map>  // Include the map library

std::map<int, int> voiceAssignment;

#define PARAMETER 0      //The main page for displaying the current patch and control (parameter) changes
#define RECALL 1         //Patches list
#define SAVE 2           //Save patch page
#define REINITIALISE 3   // Reinitialise message
#define PATCH 4          // Show current patch bypassing PARAMETER
#define PATCHNAMING 5    // Patch naming page
#define DELETE 6         //Delete patch page
#define DELETEMSG 7      //Delete patch message page
#define SETTINGS 8       //Settings page
#define SETTINGSVALUE 9  //Settings page
#define PERFORMANCE_RECALL 10
#define PERFORMANCE_SAVE 11
#define PERFORMANCE_EDIT 12
#define PERFORMANCE_NAMING 13
#define PERFORMANCE_DELETE 14
#define PERFORMANCE_DELETEMSG 15

unsigned int state = PARAMETER;

uint32_t int_ref_on_flexible_mode = 0b00001001000010100000000000000000;  // { 0000 , 1001 , 0000 , 1010000000000000 , 0000 }

uint32_t sample_data1 = 0b00000000000000000000000000000000;
uint32_t sample_data2 = 0b00000000000000000000000000000000;
uint32_t sample_data3 = 0b00000000000000000000000000000000;
uint32_t sample_data4 = 0b00000000000000000000000000000000;
uint32_t channel_a = 0b00000010000000000000000000000000;
uint32_t channel_b = 0b00000010000100000000000000000000;
uint32_t channel_c = 0b00000010001000000000000000000000;
uint32_t channel_d = 0b00000010001100000000000000000000;
uint32_t channel_e = 0b00000010010000000000000000000000;
uint32_t channel_f = 0b00000010010100000000000000000000;
uint32_t channel_g = 0b00000010011000000000000000000000;
uint32_t channel_h = 0b00000010011100000000000000000000;

enum PlayMode {
  WHOLE = 0,
  DUAL = 1,
  SPLIT = 2
};

struct Performance {
  int performanceNo;
  int upperPatchNo;
  int lowerPatchNo;
  String name;
  PlayMode mode;  // ← Back to enum type!
};

#include "ST7735Display.h"

boolean cardStatus = false;

struct VoiceAndNote {
  int note;
  int velocity;
  unsigned long timeOn;
  bool sustained;  // Sustain flag
  bool keyDown;
  double noteFreq;  // Note frequency
  int position;
  bool noteOn;
};

struct VoiceAndNote voices[NO_OF_VOICES] = {
  { -1, -1, 0, false, false, 0, -1, false },
  { -1, -1, 0, false, false, 0, -1, false },
  { -1, -1, 0, false, false, 0, -1, false },
  { -1, -1, 0, false, false, 0, -1, false },
  { -1, -1, 0, false, false, 0, -1, false },
  { -1, -1, 0, false, false, 0, -1, false },
  { -1, -1, 0, false, false, 0, -1, false },
  { -1, -1, 0, false, false, 0, -1, false }
};

// Tracks exactly which note each voice currently plays
int voiceToNoteLower[4] = { -1, -1, -1, -1 };
int voiceToNoteUpper[4] = { -1, -1, -1, -1 };


boolean voiceOn[NO_OF_VOICES] = { false, false, false, false, false, false, false, false };
int prevNote = 0;  //Initialised to middle value
bool notes[128] = { 0 }, initial_loop = 1;
int8_t noteOrder[40] = { 0 }, orderIndx = { 0 };

bool notesWhole[128], notesLower[128], notesUpper[128];
byte noteOrderWhole[40], noteOrderLower[40], noteOrderUpper[40];
int orderIndxWhole = 0, orderIndxLower = 0, orderIndxUpper = 0;

int voiceAssignmentLower[128];
int voiceAssignmentUpper[128];

CircularBuffer<Performance, PERFORMANCES_LIMIT> performances;
Performance currentPerformance;


//USB HOST MIDI Class Compliant
USBHost myusb;
USBHub hub1(myusb);
USBHub hub2(myusb);
MIDIDevice midi1(myusb);


//MIDI 5 Pin DIN
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);   // main MIDI in and out
MIDI_CREATE_INSTANCE(HardwareSerial, Serial6, MIDI6);  // MIDI out to voices
MIDI_CREATE_INSTANCE(HardwareSerial, Serial7, MIDI7);  // MIDI out to display (not connected)

#define SRP_TOTAL 8
Rox74HC595<SRP_TOTAL> srp;

// pins for 74HC595
#define LED_DATA 6   // pin 14 on 74HC595 (DATA)
#define LED_CLK 7    // pin 11 on 74HC595 (CLK)
#define LED_LATCH 8  // pin 12 on 74HC595 (LATCH)
#define LED_PWM -1   // pin 13 on 74HC595

#define OCTO_TOTAL 4
#define BTN_DEBOUNCE 50
RoxOctoswitch<OCTO_TOTAL, BTN_DEBOUNCE> octoswitch;

// pins for 74HC165
#define PIN_DATA 49  // pin 9 on 74HC165 (DATA)
#define PIN_CLK 50   // pin 2 on 74HC165 (CLK))
#define PIN_LOAD 51  // pin 1 on 74HC165 (LOAD)

RoxButton button;

int count = 0;  //For MIDI Clk Sync
int DelayForSH3 = 50;
int patchNo = 0;
int patchNoU = 0;
int patchNoL = 0;
int voiceToReturn = -1;                 //Initialise
unsigned long earliestTime = millis();  //For voice allocation - initialise to now
unsigned long buttonDebounce = 0;

void pollAllMCPs();

void initRotaryEncoders();

void initButtons();

int getEncoderSpeed(int id);

void setup() {
  SPI.begin();
  Wire.begin();           // Join the I2C bus as Master
  Wire.setClock(400000);  // Set I2C speed to 400 kHz

  mcp1.begin(0);
  delay(10);
  mcp2.begin(1);
  delay(10);
  mcp3.begin(2);
  delay(10);
  mcp4.begin(3);
  delay(10);
  mcp5.begin(4);
  delay(10);
  mcp6.begin(5);
  delay(10);
  mcp7.begin(6);
  delay(10);

  //groupEncoders();
  initRotaryEncoders();
  initButtons();

  mcp1.pinMode(7, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp1.pinMode(15, OUTPUT);  // pin 15 = GPB7 of MCP2301X

  mcp2.pinMode(7, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp2.pinMode(15, OUTPUT);  // pin 15 = GPB7 of MCP2301X

  mcp3.pinMode(7, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp3.pinMode(15, OUTPUT);  // pin 15 = GPB7 of MCP2301X

  mcp4.pinMode(7, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp4.pinMode(15, OUTPUT);  // pin 15 = GPB7 of MCP2301X

  mcp5.pinMode(7, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp5.pinMode(15, OUTPUT);  // pin 15 = GPB7 of MCP2301X

  mcp6.pinMode(6, OUTPUT);   // pin 6 = GPA7 of MCP2301X
  mcp6.pinMode(7, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp6.pinMode(14, OUTPUT);  // pin 14 = GPA7 of MCP2301X
  mcp6.pinMode(15, OUTPUT);  // pin 15 = GPB7 of MCP2301X

  mcp7.pinMode(6, OUTPUT);   // pin 6 = GPA6 of MCP2301X
  mcp7.pinMode(15, OUTPUT);  // pin 15 = GPB7 of MCP2301X

  setupDisplay();
  setUpSettings();
  setupHardware();

  octoswitch.begin(PIN_DATA, PIN_LOAD, PIN_CLK);
  octoswitch.setCallback(onButtonPress);

  srp.begin(LED_DATA, LED_LATCH, LED_CLK, LED_PWM);

  button.begin();
  button.setDoublePressThreshold(300);

  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE1));
  digitalWrite(DAC_CS1, LOW);
  delayMicroseconds(1);
  SPI.transfer32(int_ref_on_flexible_mode);
  digitalWrite(DAC_CS1, HIGH);
  SPI.endTransaction();

  for (int i = 0; i < 128; i++) {
    voiceAssignmentLower[i] = -1;
    voiceAssignmentUpper[i] = -1;
  }

  for (int i = 0; i < 4; i++) {
    voiceToNoteLower[i] = -1;
    voiceToNoteUpper[i] = -1;
  }


  cardStatus = SD.begin(BUILTIN_SDCARD);
  if (cardStatus) {
    Serial.println("SD card is connected");
    loadPatches();
    if (patches.size() == 0) {
      //save an initialised patch to SD card
      savePatch("1", INITPATCH);
      loadPatches();
    }
    loadPerformances();
    if (performances.size() == 0 && patches.size() > 0) {
      Performance defaultPerf = {
        1,
        patches.first().patchNo,
        patches.first().patchNo,
        "Default"
      };
      performances.push(defaultPerf);
      savePerformance("perf001", defaultPerf);
      loadPerformances();  // reload to ensure it's in the buffer
    }
  } else {
    Serial.println("SD card is not connected or unusable");
    reinitialiseToPanel();
    showPatchPage("No SD", "conn'd / usable", "", "");
  }

  //Read MIDI Channel from EEPROM
  midiChannel = getMIDIChannel();
  Serial.println("MIDI Ch:" + String(midiChannel) + " (0 is Omni On)");

  //USB HOST MIDI Class Compliant
  delay(400);  //Wait to turn on USB Host
  myusb.begin();
  midi1.setHandleControlChange(editControlChange);
  midi1.setHandleNoteOff(myNoteOff);
  midi1.setHandleNoteOn(myNoteOn);
  midi1.setHandlePitchChange(DinHandlePitchBend);
  midi1.setHandleAfterTouch(myAfterTouch);
  Serial.println("USB HOST MIDI Class Compliant Listening");

  //USB Client MIDI
  usbMIDI.setHandleControlChange(editControlChange);
  usbMIDI.setHandleProgramChange(myProgramChange);
  usbMIDI.setHandleAfterTouchChannel(myAfterTouch);
  usbMIDI.setHandlePitchChange(DinHandlePitchBend);
  usbMIDI.setHandleNoteOn(myNoteOn);
  usbMIDI.setHandleNoteOff(myNoteOff);
  Serial.println("USB Client MIDI Listening");

  //MIDI 5 Pin DIN
  MIDI.begin();
  MIDI.setHandleControlChange(editControlChange);
  MIDI.setHandleProgramChange(myProgramChange);
  MIDI.setHandleAfterTouchChannel(myAfterTouch);
  MIDI.setHandlePitchBend(DinHandlePitchBend);
  MIDI.setHandleNoteOn(myNoteOn);
  MIDI.setHandleNoteOff(myNoteOff);
  MIDI.turnThruOn(midi::Thru::Mode::Off);
  Serial.println("MIDI In DIN Listening");

  MIDI7.begin();
  MIDI7.setHandleControlChange(panelControlChange);
  MIDI7.turnThruOn(midi::Thru::Mode::Off);

  MIDI6.begin();
  MIDI6.turnThruOn(midi::Thru::Mode::Off);

  //Read Aftertouch from EEPROM, this can be set individually by each patch.
  upperData[P_AfterTouchDest] = getAfterTouchU();
  lowerData[P_AfterTouchDest] = getAfterTouchL();

  splitPoint = getSplitPoint();
  splitPoint = (splitPoint + 36);

  splitTrans = getSplitTrans();
  setTranspose(splitTrans);

  //Read Encoder Direction from EEPROM
  encCW = getEncoderDir();

  // Read the encoders accelerate
  accelerate = getEncoderAccelerate();
  Serial.print("Accelerate ");
  Serial.println(accelerate);

  //setupDisplay();
  delay(500);


  for (int i = 0; i < 8; i++) {
    int noteon = 60;
    MIDI6.sendNoteOn(noteon, 64, 1);
    delayMicroseconds(DelayForSH3);
    MIDI6.sendNoteOn(noteon, 64, 2);
    delay(1);
    MIDI6.sendNoteOff(noteon, 64, 1);
    delayMicroseconds(DelayForSH3);
    MIDI6.sendNoteOff(noteon, 64, 2);
    noteon++;
  }
  delay(200);

  patchNoU = 1;
  patchNoL = 1;
  upperSW = false;
  lowerSW = true;
  updatekeyboardMode(0);
  updateplayMode(0);
  recallPatch(patchNoL);  //Load first patch
}

void pollAllMCPs() {

  for (int j = 0; j < numMCPs; j++) {
    uint16_t gpioAB = allMCPs[j]->readGPIOAB();
    for (int i = 0; i < numEncoders; i++) {
      if (rotaryEncoders[i].getMCP() == allMCPs[j])
        rotaryEncoders[i].feedInput(gpioAB);
    }

    for (auto &button : allButtons) {
      if (button->getMcp() == allMCPs[j]) {
        button->feedInput(gpioAB);
      }
    }
  }
}

void initRotaryEncoders() {
  for (auto &rotaryEncoder : rotaryEncoders) {
    rotaryEncoder.init();
  }
}

void initButtons() {
  for (auto &button : allButtons) {
    button->begin();
  }
}

int getEncoderSpeed(int id) {
  if (id < 1 || id > numEncoders) return 1;

  unsigned long now = millis();
  unsigned long revolutionTime = now - lastTransition[id];

  int speed = 1;
  if (revolutionTime < 50) {
    speed = 10;
  } else if (revolutionTime < 125) {
    speed = 5;
  } else if (revolutionTime < 250) {
    speed = 2;
  }

  lastTransition[id] = now;
  return speed;
}

void RotaryEncoderChanged(bool clockwise, int id) {

  if (!accelerate) {
    speed = 1;
  } else {
    speed = getEncoderSpeed(id);
  }

  if (!clockwise) {
    speed = -speed;
  }

  switch (id) {
    case 1:
      if (upperSW) {
        upperData[P_glideTime] = (upperData[P_glideTime] + speed);
        upperData[P_glideTime] = constrain(upperData[P_glideTime], 0, 127);
        glideTimestr = LINEAR[upperData[P_glideTime]];
      } else {
        lowerData[P_glideTime] = (lowerData[P_glideTime] + speed);
        lowerData[P_glideTime] = constrain(lowerData[P_glideTime], 0, 127);
        glideTimestr = LINEAR[lowerData[P_glideTime]];
        if (wholemode) {
          upperData[P_glideTime] = lowerData[P_glideTime];
        }
      }

      updateglideTime(1);
      break;

    case 2:
      if (upperSW) {
        upperData[P_osc1PW] = (upperData[P_osc1PW] + speed);
        upperData[P_osc1PW] = constrain(upperData[P_osc1PW], 0, 127);
        osc1PWstr = PULSEWIDTH[upperData[P_osc1PW]];
      } else {
        lowerData[P_osc1PW] = (lowerData[P_osc1PW] + speed);
        lowerData[P_osc1PW] = constrain(lowerData[P_osc1PW], 0, 127);
        osc1PWstr = PULSEWIDTH[lowerData[P_osc1PW]];
        if (wholemode) {
          upperData[P_osc1PW] = lowerData[P_osc1PW];
        }
      }

      updateosc1PW(1);
      break;

    case 3:
      if (upperSW) {
        upperData[P_osc1PWM] = (upperData[P_osc1PWM] + speed);
        upperData[P_osc1PWM] = constrain(upperData[P_osc1PWM], 0, 127);
        osc1PWMstr = upperData[P_osc1PWM];
      } else {
        lowerData[P_osc1PWM] = (lowerData[P_osc1PWM] + speed);
        lowerData[P_osc1PWM] = constrain(lowerData[P_osc1PWM], 0, 127);
        osc1PWMstr = lowerData[P_osc1PWM];
        if (wholemode) {
          upperData[P_osc1PWM] = lowerData[P_osc1PWM];
        }
      }

      updateosc1PWM(1);
      break;

    case 4:
      if (upperSW) {
        upperData[P_osc1SawLevel] = (upperData[P_osc1SawLevel] + speed);
        upperData[P_osc1SawLevel] = constrain(upperData[P_osc1SawLevel], 0, 127);
        osc1SawLevelstr = upperData[P_osc1SawLevel];
      } else {
        lowerData[P_osc1SawLevel] = (lowerData[P_osc1SawLevel] + speed);
        lowerData[P_osc1SawLevel] = constrain(lowerData[P_osc1SawLevel], 0, 127);
        osc1SawLevelstr = lowerData[P_osc1SawLevel];
        if (wholemode) {
          upperData[P_osc1SawLevel] = lowerData[P_osc1SawLevel];
        }
      }

      updateOsc1SawLevel(1);
      break;

    case 5:
      if (upperSW) {
        upperData[P_osc1PulseLevel] = (upperData[P_osc1PulseLevel] + speed);
        upperData[P_osc1PulseLevel] = constrain(upperData[P_osc1PulseLevel], 0, 127);
        osc1PulseLevelstr = upperData[P_osc1PulseLevel];
      } else {
        lowerData[P_osc1PulseLevel] = (lowerData[P_osc1PulseLevel] + speed);
        lowerData[P_osc1PulseLevel] = constrain(lowerData[P_osc1PulseLevel], 0, 127);
        osc1PulseLevelstr = lowerData[P_osc1PulseLevel];
        if (wholemode) {
          upperData[P_osc1PulseLevel] = lowerData[P_osc1PulseLevel];
        }
      }

      updateOsc1PulseLevel(1);
      break;

    case 6:
      if (upperSW) {
        upperData[P_osc1SubLevel] = (upperData[P_osc1SubLevel] + speed);
        upperData[P_osc1SubLevel] = constrain(upperData[P_osc1SubLevel], 0, 127);
        osc1SubLevelstr = upperData[P_osc1SubLevel];
      } else {
        lowerData[P_osc1SubLevel] = (lowerData[P_osc1SubLevel] + speed);
        lowerData[P_osc1SubLevel] = constrain(lowerData[P_osc1SubLevel], 0, 127);
        osc1SubLevelstr = lowerData[P_osc1SubLevel];
        if (wholemode) {
          upperData[P_osc1SubLevel] = lowerData[P_osc1SubLevel];
        }
      }

      updateOsc1SubLevel(1);
      break;

    case 7:
      if (upperSW) {
        upperData[P_fmDepth] = (upperData[P_fmDepth] + speed);
        upperData[P_fmDepth] = constrain(upperData[P_fmDepth], 0, 127);
        fmDepthstr = upperData[P_fmDepth];
      } else {
        lowerData[P_fmDepth] = (lowerData[P_fmDepth] + speed);
        lowerData[P_fmDepth] = constrain(lowerData[P_fmDepth], 0, 127);
        fmDepthstr = lowerData[P_fmDepth];
        if (wholemode) {
          upperData[P_fmDepth] = lowerData[P_fmDepth];
        }
      }

      updatefmDepth(1);
      break;

    case 8:
      if (upperSW) {
        upperData[P_osc2PW] = (upperData[P_osc2PW] + speed);
        upperData[P_osc2PW] = constrain(upperData[P_osc2PW], 0, 127);
        osc2PWstr = PULSEWIDTH[upperData[P_osc2PW]];
      } else {
        lowerData[P_osc2PW] = (lowerData[P_osc2PW] + speed);
        lowerData[P_osc2PW] = constrain(lowerData[P_osc2PW], 0, 127);
        osc2PWstr = PULSEWIDTH[lowerData[P_osc2PW]];
        if (wholemode) {
          upperData[P_osc2PW] = lowerData[P_osc2PW];
        }
      }

      updateosc2PW(1);
      break;

    case 9:
      if (upperSW) {
        upperData[P_osc2PWM] = (upperData[P_osc2PWM] + speed);
        upperData[P_osc2PWM] = constrain(upperData[P_osc2PWM], 0, 127);
        osc2PWMstr = upperData[P_osc2PWM];
      } else {
        lowerData[P_osc2PWM] = (lowerData[P_osc2PWM] + speed);
        lowerData[P_osc2PWM] = constrain(lowerData[P_osc2PWM], 0, 127);
        osc2PWMstr = lowerData[P_osc2PWM];
        if (wholemode) {
          upperData[P_osc2PWM] = lowerData[P_osc2PWM];
        }
      }

      updateosc2PWM(1);
      break;

    case 10:
      if (upperSW) {
        upperData[P_osc2SawLevel] = (upperData[P_osc2SawLevel] + speed);
        upperData[P_osc2SawLevel] = constrain(upperData[P_osc2SawLevel], 0, 127);
        osc2SawLevelstr = upperData[P_osc2SawLevel];
      } else {
        lowerData[P_osc2SawLevel] = (lowerData[P_osc2SawLevel] + speed);
        lowerData[P_osc2SawLevel] = constrain(lowerData[P_osc2SawLevel], 0, 127);
        osc2SawLevelstr = lowerData[P_osc2SawLevel];
        if (wholemode) {
          upperData[P_osc2SawLevel] = lowerData[P_osc2SawLevel];
        }
      }

      updateOsc2SawLevel(1);
      break;

    case 11:
      if (upperSW) {
        upperData[P_osc2PulseLevel] = (upperData[P_osc2PulseLevel] + speed);
        upperData[P_osc2PulseLevel] = constrain(upperData[P_osc2PulseLevel], 0, 127);
        osc2PulseLevelstr = upperData[P_osc2PulseLevel];
      } else {
        lowerData[P_osc2PulseLevel] = (lowerData[P_osc2PulseLevel] + speed);
        lowerData[P_osc2PulseLevel] = constrain(lowerData[P_osc2PulseLevel], 0, 127);
        osc2PulseLevelstr = lowerData[P_osc2PulseLevel];
        if (wholemode) {
          upperData[P_osc2PulseLevel] = lowerData[P_osc2PulseLevel];
        }
      }

      updateOsc2PulseLevel(1);
      break;

    case 12:
      if (upperSW) {
        upperData[P_osc2TriangleLevel] = (upperData[P_osc2TriangleLevel] + speed);
        upperData[P_osc2TriangleLevel] = constrain(upperData[P_osc2TriangleLevel], 0, 127);
        osc2TriangleLevelstr = upperData[P_osc2TriangleLevel];
      } else {
        lowerData[P_osc2TriangleLevel] = (lowerData[P_osc2TriangleLevel] + speed);
        lowerData[P_osc2TriangleLevel] = constrain(lowerData[P_osc2TriangleLevel], 0, 127);
        osc2TriangleLevelstr = lowerData[P_osc2TriangleLevel];
        if (wholemode) {
          upperData[P_osc2TriangleLevel] = lowerData[P_osc2TriangleLevel];
        }
      }

      updateOsc2TriangleLevel(1);
      break;

    case 13:
      if (upperSW) {
        upperData[P_filterCutoff] = (upperData[P_filterCutoff] + speed);
        upperData[P_filterCutoff] = constrain(upperData[P_filterCutoff], 0, 127);
        filterCutoffstr = FILTERCUTOFF[upperData[P_filterCutoff]];
      } else {
        lowerData[P_filterCutoff] = (lowerData[P_filterCutoff] + speed);
        lowerData[P_filterCutoff] = constrain(lowerData[P_filterCutoff], 0, 127);
        filterCutoffstr = FILTERCUTOFF[lowerData[P_filterCutoff]];
        if (wholemode) {
          upperData[P_filterCutoff] = lowerData[P_filterCutoff];
        }
      }

      updateFilterCutoff(1);
      break;

    case 14:
      if (upperSW) {
        upperData[P_filterRes] = (upperData[P_filterRes] + speed);
        upperData[P_filterRes] = constrain(upperData[P_filterRes], 0, 127);
        filterResstr = upperData[P_filterRes];
      } else {
        lowerData[P_filterRes] = (lowerData[P_filterRes] + speed);
        lowerData[P_filterRes] = constrain(lowerData[P_filterRes], 0, 127);
        filterResstr = lowerData[P_filterRes];
        if (wholemode) {
          upperData[P_filterRes] = lowerData[P_filterRes];
        }
      }

      updatefilterRes(1);
      break;

    case 15:
      if (upperSW) {
        upperData[P_filterEGlevel] = (upperData[P_filterEGlevel] + speed);
        upperData[P_filterEGlevel] = constrain(upperData[P_filterEGlevel], 0, 127);
        filterEGlevelstr = upperData[P_filterEGlevel];
      } else {
        lowerData[P_filterEGlevel] = (lowerData[P_filterEGlevel] + speed);
        lowerData[P_filterEGlevel] = constrain(lowerData[P_filterEGlevel], 0, 127);
        filterEGlevelstr = lowerData[P_filterEGlevel];
        if (wholemode) {
          upperData[P_filterEGlevel] = lowerData[P_filterEGlevel];
        }
      }

      updatefilterEGlevel(1);
      break;

    case 16:
      if (upperSW) {
        upperData[P_keytrack] = (upperData[P_keytrack] + speed);
        upperData[P_keytrack] = constrain(upperData[P_keytrack], 0, 127);
        keytrackstr = upperData[P_keytrack];
      } else {
        lowerData[P_keytrack] = (lowerData[P_keytrack] + speed);
        lowerData[P_keytrack] = constrain(lowerData[P_keytrack], 0, 127);
        keytrackstr = lowerData[P_keytrack];
        if (wholemode) {
          upperData[P_keytrack] = lowerData[P_keytrack];
        }
      }

      updatekeytrack(1);
      break;

    case 17:
      if (upperSW) {
        upperData[P_filterLFO] = (upperData[P_filterLFO] + speed);
        upperData[P_filterLFO] = constrain(upperData[P_filterLFO], 0, 127);
        filterLFOstr = upperData[P_filterLFO];
      } else {
        lowerData[P_filterLFO] = (lowerData[P_filterLFO] + speed);
        lowerData[P_filterLFO] = constrain(lowerData[P_filterLFO], 0, 127);
        filterLFOstr = lowerData[P_filterLFO];
        if (wholemode) {
          upperData[P_filterLFO] = lowerData[P_filterLFO];
        }
      }

      updatefilterLFO(1);
      break;

    case 18:
      if (upperSW) {
        upperData[P_filterAttack] = (upperData[P_filterAttack] + speed);
        upperData[P_filterAttack] = constrain(upperData[P_filterAttack], 0, 127);
        filterAttackstr = ENVTIMES[upperData[P_filterAttack]];
      } else {
        lowerData[P_filterAttack] = (lowerData[P_filterAttack] + speed);
        lowerData[P_filterAttack] = constrain(lowerData[P_filterAttack], 0, 127);
        filterAttackstr = ENVTIMES[lowerData[P_filterAttack]];
        if (wholemode) {
          upperData[P_filterAttack] = lowerData[P_filterAttack];
        }
      }

      updatefilterAttack(1);
      break;

    case 19:
      if (upperSW) {
        upperData[P_filterDecay] = (upperData[P_filterDecay] + speed);
        upperData[P_filterDecay] = constrain(upperData[P_filterDecay], 0, 127);
        filterDecaystr = ENVTIMES[upperData[P_filterDecay]];
      } else {
        lowerData[P_filterDecay] = (lowerData[P_filterDecay] + speed);
        lowerData[P_filterDecay] = constrain(lowerData[P_filterDecay], 0, 127);
        filterDecaystr = ENVTIMES[lowerData[P_filterDecay]];
        if (wholemode) {
          upperData[P_filterDecay] = lowerData[P_filterDecay];
        }
      }

      updatefilterDecay(1);
      break;

    case 20:
      if (upperSW) {
        upperData[P_filterSustain] = (upperData[P_filterSustain] + speed);
        upperData[P_filterSustain] = constrain(upperData[P_filterSustain], 0, 127);
        filterSustainstr = LINEAR_FILTERMIXERSTR[upperData[P_filterSustain]];
      } else {
        lowerData[P_filterSustain] = (lowerData[P_filterSustain] + speed);
        lowerData[P_filterSustain] = constrain(lowerData[P_filterSustain], 0, 127);
        filterSustainstr = LINEAR_FILTERMIXERSTR[lowerData[P_filterSustain]];
        if (wholemode) {
          upperData[P_filterSustain] = lowerData[P_filterSustain];
        }
      }

      updatefilterSustain(1);
      break;

    case 21:
      if (upperSW) {
        upperData[P_filterRelease] = (upperData[P_filterRelease] + speed);
        upperData[P_filterRelease] = constrain(upperData[P_filterRelease], 0, 127);
        filterReleasestr = ENVTIMES[upperData[P_filterRelease]];
      } else {
        lowerData[P_filterRelease] = (lowerData[P_filterRelease] + speed);
        lowerData[P_filterRelease] = constrain(lowerData[P_filterRelease], 0, 127);
        filterReleasestr = ENVTIMES[lowerData[P_filterRelease]];
        if (wholemode) {
          upperData[P_filterRelease] = lowerData[P_filterRelease];
        }
      }

      updatefilterRelease(1);
      break;

    case 22:
      if (upperSW) {
        upperData[P_osc2Detune] = (upperData[P_osc2Detune] + speed);
        upperData[P_osc2Detune] = constrain(upperData[P_osc2Detune], 0, 127);
        osc2Detunestr = upperData[P_osc2Detune];
      } else {
        lowerData[P_osc2Detune] = (lowerData[P_osc2Detune] + speed);
        lowerData[P_osc2Detune] = constrain(lowerData[P_osc2Detune], 0, 127);
        osc2Detunestr = lowerData[P_osc2Detune];
        if (wholemode) {
          upperData[P_osc2Detune] = lowerData[P_osc2Detune];
        }
      }

      updateosc2Detune(1);
      break;

    case 23:
      if (upperSW) {
        upperData[P_osc2Interval] = (upperData[P_osc2Interval] + speed);
        upperData[P_osc2Interval] = constrain(upperData[P_osc2Interval], 0, 12);
        osc2Intervalstr = upperData[P_osc2Interval];
      } else {
        lowerData[P_osc2Interval] = (lowerData[P_osc2Interval] + speed);
        lowerData[P_osc2Interval] = constrain(lowerData[P_osc2Interval], 0, 12);
        osc2Intervalstr = lowerData[P_osc2Interval];
        if (wholemode) {
          upperData[P_osc2Interval] = lowerData[P_osc2Interval];
        }
      }

      updateosc2Interval(1);
      break;

    case 24:
      if (upperSW) {
        upperData[P_ampAttack] = (upperData[P_ampAttack] + speed);
        upperData[P_ampAttack] = constrain(upperData[P_ampAttack], 0, 127);
        ampAttackstr = ENVTIMES[upperData[P_ampAttack]];
      } else {
        lowerData[P_ampAttack] = (lowerData[P_ampAttack] + speed);
        lowerData[P_ampAttack] = constrain(lowerData[P_ampAttack], 0, 127);
        ampAttackstr = ENVTIMES[lowerData[P_ampAttack]];
        if (wholemode) {
          upperData[P_ampAttack] = lowerData[P_ampAttack];
        }
      }

      updateampAttack(1);
      break;

    case 25:
      if (upperSW) {
        upperData[P_ampDecay] = (upperData[P_ampDecay] + speed);
        upperData[P_ampDecay] = constrain(upperData[P_ampDecay], 0, 127);
        ampDecaystr = ENVTIMES[upperData[P_ampDecay]];
      } else {
        lowerData[P_ampDecay] = (lowerData[P_ampDecay] + speed);
        lowerData[P_ampDecay] = constrain(lowerData[P_ampDecay], 0, 127);
        ampDecaystr = ENVTIMES[lowerData[P_ampDecay]];
        if (wholemode) {
          upperData[P_ampDecay] = lowerData[P_ampDecay];
        }
      }

      updateampDecay(1);
      break;

    case 26:
      if (upperSW) {
        upperData[P_ampSustain] = (upperData[P_ampSustain] + speed);
        upperData[P_ampSustain] = constrain(upperData[P_ampSustain], 0, 127);
        ampSustainstr = LINEAR_FILTERMIXERSTR[upperData[P_ampSustain]];
      } else {
        lowerData[P_ampSustain] = (lowerData[P_ampSustain] + speed);
        lowerData[P_ampSustain] = constrain(lowerData[P_ampSustain], 0, 127);
        ampSustainstr = LINEAR_FILTERMIXERSTR[lowerData[P_ampSustain]];
        if (wholemode) {
          upperData[P_ampSustain] = lowerData[P_ampSustain];
        }
      }

      updateampSustain(1);
      break;

    case 27:
      if (upperSW) {
        upperData[P_ampRelease] = (upperData[P_ampRelease] + speed);
        upperData[P_ampRelease] = constrain(upperData[P_ampRelease], 0, 127);
        ampReleasestr = ENVTIMES[upperData[P_ampRelease]];
      } else {
        lowerData[P_ampRelease] = (lowerData[P_ampRelease] + speed);
        lowerData[P_ampRelease] = constrain(lowerData[P_ampRelease], 0, 127);
        ampReleasestr = ENVTIMES[lowerData[P_ampRelease]];
        if (wholemode) {
          upperData[P_ampRelease] = lowerData[P_ampRelease];
        }
      }

      updateampRelease(1);
      break;

    case 28:
      if (upperSW) {
        upperData[P_LFORate] = (upperData[P_LFORate] + speed);
        upperData[P_LFORate] = constrain(upperData[P_LFORate], 0, 127);
        LFORatestr = LFOTEMPO[upperData[P_LFORate]];
      } else {
        lowerData[P_LFORate] = (lowerData[P_LFORate] + speed);
        lowerData[P_LFORate] = constrain(lowerData[P_LFORate], 0, 127);
        LFORatestr = LFOTEMPO[lowerData[P_LFORate]];
        if (wholemode) {
          upperData[P_LFORate] = lowerData[P_LFORate];
        }
      }

      updateLFORate(1);
      break;

    case 29:
      if (upperSW) {
        upperData[P_LFODelay] = (upperData[P_LFODelay] + speed);
        upperData[P_LFODelay] = constrain(upperData[P_LFODelay], 0, 127);
        LFODelaystr = upperData[P_LFODelay];
      } else {
        lowerData[P_LFODelay] = (lowerData[P_LFODelay] + speed);
        lowerData[P_LFODelay] = constrain(lowerData[P_LFODelay], 0, 127);
        LFODelaystr = lowerData[P_LFODelay];
        if (wholemode) {
          upperData[P_LFODelay] = lowerData[P_LFODelay];
        }
      }

      updateLFODelay(1);
      break;

    case 30:
      if (upperSW) {
        upperData[P_modWheelDepth] = (upperData[P_modWheelDepth] + speed);
        upperData[P_modWheelDepth] = constrain(upperData[P_modWheelDepth], 0, 127);
        modWheelDepthstr = upperData[P_modWheelDepth];
      } else {
        lowerData[P_modWheelDepth] = (lowerData[P_modWheelDepth] + speed);
        lowerData[P_modWheelDepth] = constrain(lowerData[P_modWheelDepth], 0, 127);
        modWheelDepthstr = lowerData[P_modWheelDepth];
        if (wholemode) {
          upperData[P_modWheelDepth] = lowerData[P_modWheelDepth];
        }
      }

      updatemodWheelDepth(1);
      break;

    case 31:
      if (upperSW) {
        upperData[P_pwLFO] = (upperData[P_pwLFO] + speed);
        upperData[P_pwLFO] = constrain(upperData[P_pwLFO], 0, 127);
        pwLFOstr = LFOTEMPO[upperData[P_pwLFO]];
      } else {
        lowerData[P_pwLFO] = (lowerData[P_pwLFO] + speed);
        lowerData[P_pwLFO] = constrain(lowerData[P_pwLFO], 0, 127);
        pwLFOstr = LFOTEMPO[lowerData[P_pwLFO]];
        if (wholemode) {
          upperData[P_pwLFO] = lowerData[P_pwLFO];
        }
      }

      updatepwLFO(1);
      break;

    case 32:
      if (upperSW) {
        upperData[P_PitchBendLevel] = (upperData[P_PitchBendLevel] + speed);
        upperData[P_PitchBendLevel] = constrain(upperData[P_PitchBendLevel], 0, 12);
        PitchBendLevelstr = upperData[P_PitchBendLevel];
      } else {
        lowerData[P_PitchBendLevel] = (lowerData[P_PitchBendLevel] + speed);
        lowerData[P_PitchBendLevel] = constrain(lowerData[P_PitchBendLevel], 0, 12);
        PitchBendLevelstr = lowerData[P_PitchBendLevel];
        if (wholemode) {
          upperData[P_PitchBendLevel] = lowerData[P_PitchBendLevel];
        }
      }

      updatePitchBendDepth(1);
      break;

    case 33:
      if (upperSW) {
        upperData[P_noiseLevel] = (upperData[P_noiseLevel] + speed);
        upperData[P_noiseLevel] = constrain(upperData[P_noiseLevel], 0, 127);
        noiseLevelstr = LINEARCENTREZERO[upperData[P_noiseLevel]];
      } else {
        lowerData[P_noiseLevel] = (lowerData[P_noiseLevel] + speed);
        lowerData[P_noiseLevel] = constrain(lowerData[P_noiseLevel], 0, 127);
        noiseLevelstr = LINEARCENTREZERO[lowerData[P_noiseLevel]];
        if (wholemode) {
          upperData[P_noiseLevel] = lowerData[P_noiseLevel];
        }
      }

      updatenoiseLevel(1);
      break;

    case 34:
      if (upperSW) {
        upperData[P_ATDepth] = (upperData[P_ATDepth] + speed);
        upperData[P_ATDepth] = constrain(upperData[P_ATDepth], 0, 127);
        ATDepthstr = upperData[P_ATDepth];
      } else {
        lowerData[P_ATDepth] = (lowerData[P_ATDepth] + speed);
        lowerData[P_ATDepth] = constrain(lowerData[P_ATDepth], 0, 127);
        ATDepthstr = lowerData[P_ATDepth];
        if (wholemode) {
          upperData[P_ATDepth] = lowerData[P_ATDepth];
        }
      }

      updateATDepth(1);
      break;

    case 35:
      if (upperSW) {
        upperData[P_effectsMix] = (upperData[P_effectsMix] + speed);
        upperData[P_effectsMix] = constrain(upperData[P_effectsMix], 0, 127);
        effectsMixstr = upperData[P_effectsMix];
      } else {
        lowerData[P_effectsMix] = (lowerData[P_effectsMix] + speed);
        lowerData[P_effectsMix] = constrain(lowerData[P_effectsMix], 0, 127);
        effectsMixstr = lowerData[P_effectsMix];
        if (wholemode) {
          upperData[P_effectsMix] = lowerData[P_effectsMix];
        }
      }

      updateeffectsMix(1);
      break;

    case 36:
      if (upperSW) {
        upperData[P_volumeControl] = (upperData[P_volumeControl] + speed);
        upperData[P_volumeControl] = constrain(upperData[P_volumeControl], 0, 127);
        volumeControlstr = upperData[P_volumeControl];
      } else {
        lowerData[P_volumeControl] = (lowerData[P_volumeControl] + speed);
        lowerData[P_volumeControl] = constrain(lowerData[P_volumeControl], 0, 127);
        volumeControlstr = lowerData[P_volumeControl];
        if (wholemode) {
          upperData[P_volumeControl] = lowerData[P_volumeControl];
        }
      }

      updatevolumeControl(1);
      break;

    case 37:
      if (upperSW) {
        upperData[P_effectPot1] = (upperData[P_effectPot1] + speed);
        upperData[P_effectPot1] = constrain(upperData[P_effectPot1], 0, 127);
        effectPot1str = upperData[P_effectPot1];
      } else {
        lowerData[P_effectPot1] = (lowerData[P_effectPot1] + speed);
        lowerData[P_effectPot1] = constrain(lowerData[P_effectPot1], 0, 127);
        effectPot1str = lowerData[P_effectPot1];
        if (wholemode) {
          upperData[P_effectPot1] = lowerData[P_effectPot1];
        }
      }

      updateeffectPot1(1);
      break;

    case 38:
      if (upperSW) {
        upperData[P_effectPot2] = (upperData[P_effectPot2] + speed);
        upperData[P_effectPot2] = constrain(upperData[P_effectPot2], 0, 127);
        effectPot2str = upperData[P_effectPot2];
      } else {
        lowerData[P_effectPot2] = (lowerData[P_effectPot2] + speed);
        lowerData[P_effectPot2] = constrain(lowerData[P_effectPot2], 0, 127);
        effectPot2str = lowerData[P_effectPot2];
        if (wholemode) {
          upperData[P_effectPot2] = lowerData[P_effectPot2];
        }
      }

      updateeffectPot2(1);
      break;

    case 39:
      if (upperSW) {
        upperData[P_effectPot3] = (upperData[P_effectPot3] + speed);
        upperData[P_effectPot3] = constrain(upperData[P_effectPot3], 0, 127);
        effectPot3str = upperData[P_effectPot3];
      } else {
        lowerData[P_effectPot3] = (lowerData[P_effectPot3] + speed);
        lowerData[P_effectPot3] = constrain(lowerData[P_effectPot3], 0, 127);
        effectPot3str = lowerData[P_effectPot3];
        if (wholemode) {
          upperData[P_effectPot3] = lowerData[P_effectPot3];
        }
      }

      updateeffectPot3(1);
      break;

    case 40:
      if (upperSW) {
        upperData[P_pmDCO2] = (upperData[P_pmDCO2] + speed);
        upperData[P_pmDCO2] = constrain(upperData[P_pmDCO2], 0, 127);
        pmDCO2str = upperData[P_pmDCO2];
      } else {
        lowerData[P_pmDCO2] = (lowerData[P_pmDCO2] + speed);
        lowerData[P_pmDCO2] = constrain(lowerData[P_pmDCO2], 0, 127);
        pmDCO2str = lowerData[P_pmDCO2];
        if (wholemode) {
          upperData[P_pmDCO2] = lowerData[P_pmDCO2];
        }
      }

      updatePM_DCO2(1);
      break;

    case 41:
      if (upperSW) {
        upperData[P_pmFilterEnv] = (upperData[P_pmFilterEnv] + speed);
        upperData[P_pmFilterEnv] = constrain(upperData[P_pmFilterEnv], 0, 127);
        pmFilterEnvstr = upperData[P_pmFilterEnv];
      } else {
        lowerData[P_pmFilterEnv] = (lowerData[P_pmFilterEnv] + speed);
        lowerData[P_pmFilterEnv] = constrain(lowerData[P_pmFilterEnv], 0, 127);
        pmFilterEnvstr = lowerData[P_pmFilterEnv];
        if (wholemode) {
          upperData[P_pmFilterEnv] = lowerData[P_pmFilterEnv];
        }
      }

      updatePM_FilterEnv(1);
      break;

    case 42:
      if (upperSW) {
        upperData[P_amDepth] = (upperData[P_amDepth] + speed);
        upperData[P_amDepth] = constrain(upperData[P_amDepth], 0, 127);
        amDepthstr = upperData[P_amDepth];
      } else {
        lowerData[P_amDepth] = (lowerData[P_amDepth] + speed);
        lowerData[P_amDepth] = constrain(lowerData[P_amDepth], 0, 127);
        amDepthstr = lowerData[P_amDepth];
        if (wholemode) {
          upperData[P_amDepth] = lowerData[P_amDepth];
        }
      }

      updateamDepth(1);
      break;
  }


  //rotaryEncoderChanged(id, clockwise, speed);
}

void mainButtonChanged(Button *btn, bool released) {

  switch (btn->id) {
    // case OSC1_PW_BUTTON:
    //   if (!released) {
    //     osc1_octave = osc1_octave + 1;
    //     if (osc1_octave > 2) {
    //       osc1_octave = 0;
    //     }
    //     myControlChange(midiChannel, CCosc1_octave, osc1_octave);
    //   }
    //   break;

    // case OSC1_WAVE_BUTTON:
    //   if (!released) {
    //     osc1_wave = osc1_wave + 1;
    //     if (osc1_wave > 3) {
    //       osc1_wave = 0;
    //     }
    //     myControlChange(midiChannel, CCosc1_wave, osc1_wave);
    //   }
    //   break;

    // case OSC1_SUB_BUTTON:
    //   if (!released) {
    //     osc1_sub = !osc1_sub;
    //     myControlChange(midiChannel, CCosc1_sub, osc1_sub);
    //   }
    //   break;

    // case OSC2_WAVE_BUTTON:
    //   if (!released) {
    //     osc2_wave = osc2_wave + 1;
    //     if (osc2_wave > 3) {
    //       osc2_wave = 0;
    //     }
    //     myControlChange(midiChannel, CCosc2_wave, osc2_wave);
    //   }
    //   break;

    // case OSC2_XMOD_BUTTON:
    //   if (!released) {
    //     osc2_xmod = osc2_xmod + 1;
    //     if (osc2_xmod > 2) {
    //       osc2_xmod = 0;
    //     }
    //     myControlChange(midiChannel, CCosc2_xmod, osc2_xmod);
    //   }
    //   break;

    // case OSC2_EG_BUTTON:
    //   if (!released) {
    //     osc2_eg_select = osc2_eg_select + 1;
    //     if (osc2_eg_select > 1) {
    //       osc2_eg_select = 0;
    //     }
    //     myControlChange(midiChannel, CCosc2_eg_select, osc2_eg_select);
    //   }
    //   break;

    // case LFO1_WAVE_BUTTON:
    //   if (!released) {
    //     lfo1_wave = lfo1_wave + 1;
    //     if (lfo1_wave > 3) {
    //       lfo1_wave = 0;
    //     }
    //     myControlChange(midiChannel, CClfo1_wave, lfo1_wave);
    //   }
    //   break;

    // case LFO2_WAVE_BUTTON:
    //   if (!released) {
    //     lfo2_wave = lfo2_wave + 1;
    //     if (lfo2_wave > 3) {
    //       lfo2_wave = 0;
    //     }
    //     myControlChange(midiChannel, CClfo2_wave, lfo2_wave);
    //   }
    //   break;

    // case LFO3_WAVE_BUTTON:
    //   if (!released) {
    //     lfo3_wave = lfo3_wave + 1;
    //     if (lfo3_wave > 3) {
    //       lfo3_wave = 0;
    //     }
    //     myControlChange(midiChannel, CClfo3_wave, lfo3_wave);
    //   }
    //   break;

    // case ENV_SEL_BUTTON:
    //   if (!released) {
    //     eg_select = eg_select + 1;
    //     if (eg_select > 2) {
    //       eg_select = 0;
    //     }
    //     myControlChange(midiChannel, CCeg_select, eg_select);
    //   }
    //   break;

    // case LFO_SEL_BUTTON:
    //   if (!released) {
    //     lfo_select = lfo_select + 1;
    //     if (lfo_select > 2) {
    //       lfo_select = 0;
    //     }
    //     myControlChange(midiChannel, CClfo_select, lfo_select);
    //   }
    //   break;

    // case OSC1_LEV_SW:
    //   if (!released) {
    //     osc1_level = 0;
    //     myControlChange(midiChannel, CCosc1_level, osc1_level);
    //   }
    //   break;

    // case OSC2_DET_SW:
    //   if (!released) {
    //     osc2_detune = 50;
    //     myControlChange(midiChannel, CCosc2_detune, osc2_detune);
    //   }
    //   break;

    // case OSC2_LEV_SW:
    //   if (!released) {
    //     osc2_level = 0;
    //     myControlChange(midiChannel, CCosc2_level, osc2_level);
    //   }
    //   break;

    // case OSC2_EG_SW:
    //   if (!released) {
    //     osc2_eg_depth = 50;
    //     myControlChange(midiChannel, CCosc2_eg_depth, osc2_eg_depth);
    //   }
    //   break;

    // case VCF_EG_SW:
    //   if (!released) {
    //     vcf_eg_depth = 50;
    //     myControlChange(midiChannel, CCvcf_eg_depth, vcf_eg_depth);
    //   }
    //   break;

    // case VCF_KEYF_SW:
    //   if (!released) {
    //     vcf_key_follow = 0;
    //     myControlChange(midiChannel, CCvcf_key_follow, vcf_key_follow);
    //   }
    //   break;

    // case VCF_VEL_SW:
    //   if (!released) {
    //     vcf_key_velocity = 0;
    //     myControlChange(midiChannel, CCvcf_key_velocity, vcf_key_velocity);
    //   }
    //   break;

    // case VCA_VEL_SW:
    //   if (!released) {
    //     vca_key_velocity = 0;
    //     myControlChange(midiChannel, CCvca_key_velocity, vca_key_velocity);
    //   }
    //   break;
  }
}

void recallPerformance(const Performance &perf) {
  currentPerformance = perf;
  playMode = perf.mode;

  switch (playMode) {
    case WHOLE:
      recallPatch(perf.lowerPatchNo);
      patchNo = perf.lowerPatchNo;
      refreshPatchDisplayFromState();
      break;
    case DUAL:
    case SPLIT:
      recallPatch(perf.upperPatchNo);
      recallPatch(perf.lowerPatchNo);
      patchNo = perf.lowerPatchNo;
      refreshPatchDisplayFromState();
      break;
  }
}

void refreshPatchDisplayFromState() {
  showPatchPage(
    currentPgmNumU,
    currentPatchNameU,
    currentPgmNumL,
    currentPatchNameL);
}

String getModeName(PlayMode mode) {
  switch (mode) {
    case WHOLE: return "Whole";
    case DUAL: return "Dual";
    case SPLIT: return "Split";
    default: return "-";
  }
}


void loadPerformances() {
  performances.clear();
  File dir = SD.open("/performances");

  if (!dir || !dir.isDirectory()) {
    Serial.println("/performances not found or is not a directory");
    return;
  }

  while (true) {
    File file = dir.openNextFile();
    if (!file) break;

    if (file.isDirectory()) {
      file.close();
      continue;
    }

    String dataLine = file.readStringUntil('\n');
    file.close();

    if (dataLine.length() > 0) {
      int comma1 = dataLine.indexOf(',');
      int comma2 = dataLine.indexOf(',', comma1 + 1);
      int comma3 = dataLine.indexOf(',', comma2 + 1);

      if (comma1 == -1 || comma2 == -1 || comma3 == -1) continue;

      int upper = dataLine.substring(0, comma1).toInt();
      int lower = dataLine.substring(comma1 + 1, comma2).toInt();
      String name = dataLine.substring(comma2 + 1, comma3);
      int mode = dataLine.substring(comma3 + 1).toInt();

      int perfNo = performances.size() + 1;
      performances.push({ perfNo, upper, lower, name, (PlayMode)mode });
    }
  }

  if (performances.size() == 0) {
    Performance defaultPerf = { 1, 1, 1, "Default", WHOLE };
    savePerformance("perf001", defaultPerf);
    loadPerformances();  // try again
  }
}

void savePerformance(const char *fileName, const Performance &perf) {
  String path = "/performances/" + String(fileName);

  if (SD.exists(path.c_str())) {
    SD.remove(path.c_str());
  }

  File file = SD.open(path.c_str(), FILE_WRITE);
  if (file) {
    file.print(perf.upperPatchNo);
    file.print(",");
    file.print(perf.lowerPatchNo);
    file.print(",");
    file.print(perf.name);
    file.print(",");
    file.println((int)perf.mode);  // Save playMode as an integer (0, 1, 2)
    file.close();
  } else {
    Serial.print("Failed to save performance: ");
    Serial.println(path);
  }
}

void editControlChange(byte channel, byte control, byte value) {
  int newvalue = (value << 3);
  myControlChange(channel, control, newvalue);
}

void panelControlChange(byte channel, byte control, byte value) {
  int newvalue = value;
  myControlChange(channel, control, newvalue);
}

int mod(int a, int b) {
  int r = a % b;
  return r < 0 ? r + b : r;
}

void setTranspose(int splitTrans) {
  switch (splitTrans) {
    case 0:
      lowerTranspose = -24;
      oldsplitTrans = splitTrans;
      break;

    case 1:
      lowerTranspose = -12;
      oldsplitTrans = splitTrans;
      break;

    case 2:
      lowerTranspose = 0;
      oldsplitTrans = splitTrans;
      break;

    case 3:
      lowerTranspose = 12;
      oldsplitTrans = splitTrans;
      break;

    case 4:
      lowerTranspose = 24;
      oldsplitTrans = splitTrans;
      break;
  }
}

void LFODelayHandle() {
  // LFO Delay code
  getDelayTime();

  unsigned long currentMillisU = millis();
  if (upperData[P_monoMulti] && !upperData[P_LFODelayGo]) {
    if (oldnumberOfNotesU < numberOfNotesU) {
      previousMillisU = currentMillisU;
      oldnumberOfNotesU = numberOfNotesU;
    }
  }
  if (numberOfNotesU > 0) {
    if (currentMillisU - previousMillisU >= intervalU) {
      upperData[P_LFODelayGo] = 1;
    } else {
      upperData[P_LFODelayGo] = 0;
    }
  } else {
    upperData[P_LFODelayGo] = 1;
    previousMillisU = currentMillisU;  //reset timer so its ready for the next time
  }

  unsigned long currentMillisL = millis();
  if (lowerData[P_monoMulti] && !lowerData[P_LFODelayGo]) {
    if (oldnumberOfNotesL < numberOfNotesL) {
      previousMillisL = currentMillisL;
      oldnumberOfNotesL = numberOfNotesL;
    }
  }
  if (numberOfNotesL > 0) {
    if (currentMillisL - previousMillisL >= intervalL) {
      lowerData[P_LFODelayGo] = 1;
    } else {
      lowerData[P_LFODelayGo] = 0;
    }
  } else {
    lowerData[P_LFODelayGo] = 1;
    previousMillisL = currentMillisL;  //reset timer so its ready for the next time
  }
}

// Mono lower & uppper

void commandTopNoteLower() {
  int topNote = -1;
  for (int i = 0; i < 128; i++)
    if (notesLower[i]) topNote = i;

  if (topNote >= 0)
    assignVoice(topNote, noteVel, 0);
  else
    releaseVoice(noteMsg, 0);
}

void commandBottomNoteLower() {
  int bottomNote = -1;
  for (int i = 127; i >= 0; i--)
    if (notesLower[i]) bottomNote = i;

  if (bottomNote >= 0)
    assignVoice(bottomNote, noteVel, 0);
  else
    releaseVoice(noteMsg, 0);
}

void commandLastNoteLower() {
  for (int i = 0; i < 40; i++) {
    int8_t idx = noteOrderLower[mod(orderIndxLower - i, 40)];
    if (notesLower[idx]) {
      assignVoice(idx, noteVel, 0);
      return;
    }
  }
  releaseVoice(noteMsg, 0);
}

void commandTopNoteUpper() {
  int topNote = -1;
  for (int i = 0; i < 128; i++)
    if (notesUpper[i]) topNote = i;

  if (topNote >= 0)
    assignVoice(topNote, noteVel, 4);
  else
    releaseVoice(noteMsg, 4);
}

void commandBottomNoteUpper() {
  int bottomNote = -1;
  for (int i = 127; i >= 0; i--)
    if (notesUpper[i]) bottomNote = i;

  if (bottomNote >= 0)
    assignVoice(bottomNote, noteVel, 4);
  else
    releaseVoice(noteMsg, 4);
}

void commandLastNoteUpper() {
  for (int i = 0; i < 40; i++) {
    int8_t idx = noteOrderUpper[mod(orderIndxUpper - i, 40)];
    if (notesUpper[idx]) {
      assignVoice(idx, noteVel, 4);
      return;
    }
  }
  releaseVoice(noteMsg, 4);
}

// Unison lower and upper

void commandTopNoteUniLower() {
  int topNote = -1;
  for (int i = 0; i < 128; i++)
    if (notesLower[i]) topNote = i;

  if (topNote >= 0)
    for (int v = 0; v < 4; v++) assignVoice(topNote, noteVel, v);
  else
    for (int v = 0; v < 4; v++) releaseVoice(noteMsg, v);
}

void commandBottomNoteUniLower() {
  int bottomNote = -1;
  for (int i = 127; i >= 0; i--)
    if (notesLower[i]) bottomNote = i;

  if (bottomNote >= 0)
    for (int v = 0; v < 4; v++) assignVoice(bottomNote, noteVel, v);
  else
    for (int v = 0; v < 4; v++) releaseVoice(noteMsg, v);
}

void commandLastNoteUniLower() {
  for (int i = 0; i < 40; i++) {
    int8_t idx = noteOrderLower[mod(orderIndxLower - i, 40)];
    if (notesLower[idx]) {
      for (int v = 0; v < 4; v++) assignVoice(idx, noteVel, v);
      return;
    }
  }
  for (int v = 0; v < 4; v++) releaseVoice(noteMsg, v);
}

void commandTopNoteUniUpper() {
  int topNote = -1;
  for (int i = 0; i < 128; i++)
    if (notesUpper[i]) topNote = i;

  if (topNote >= 0)
    for (int v = 4; v < 8; v++) assignVoice(topNote, noteVel, v);
  else
    for (int v = 4; v < 8; v++) releaseVoice(noteMsg, v);
}

void commandBottomNoteUniUpper() {
  int bottomNote = -1;
  for (int i = 127; i >= 0; i--)
    if (notesUpper[i]) bottomNote = i;

  if (bottomNote >= 0)
    for (int v = 4; v < 8; v++) assignVoice(bottomNote, noteVel, v);
  else
    for (int v = 4; v < 8; v++) releaseVoice(noteMsg, v);
}

void commandLastNoteUniUpper() {
  for (int i = 0; i < 40; i++) {
    int8_t idx = noteOrderUpper[mod(orderIndxUpper - i, 40)];
    if (notesUpper[idx]) {
      for (int v = 4; v < 8; v++) assignVoice(idx, noteVel, v);
      return;
    }
  }
  for (int v = 4; v < 8; v++) releaseVoice(noteMsg, v);
}


void myNoteOn(byte channel, byte note, byte velocity) {

  if (isAutotuning) return;

  numberOfNotesU++;
  numberOfNotesL++;

  prevNote = note;

  int voiceNum = -1;

  switch (playMode) {

    // WHOLE MODE (No changes needed if currently working)
    case 0:
      switch (lowerData[P_keyboardMode]) {
        case 0:
          voiceNum = getVoiceNo(-1) - 1;
          assignVoice(note, velocity, voiceNum);
          break;  // Poly1
        case 1:
          voiceNum = getVoiceNoPoly2(-1) - 1;
          assignVoice(note, velocity, voiceNum);
          break;                                             // Poly2
        case 2: commandMonoNoteOn(note, velocity); break;    // Mono
        case 3: commandUnisonNoteOn(note, velocity); break;  // Unison
      }
      voiceAssignment[note] = voiceNum;
      break;

    // DUAL MODE (Explicitly corrected, place this clearly here):
    case 1:
      {
        // Lower Split
        if (lowerData[P_keyboardMode] == 1) {  // Poly2 Lower
          int lowerVoice = getLowerSplitVoicePoly2(note);
          int oldNote = voiceToNoteLower[lowerVoice];
          if (oldNote >= 0) {
            releaseVoice(oldNote, lowerVoice);
            voiceAssignmentLower[oldNote] = -1;
          }
          assignVoice(note, velocity, lowerVoice);
          voiceAssignmentLower[note] = lowerVoice;
          voiceToNoteLower[lowerVoice] = note;
        } else if (lowerData[P_keyboardMode] == 0) {  // Poly1 Lower
          int lowerVoice = getLowerSplitVoice(note);
          assignVoice(note, velocity, lowerVoice);
          voiceAssignmentLower[note] = lowerVoice;
          voiceToNoteLower[lowerVoice] = note;
        } else if (lowerData[P_keyboardMode] == 2) {
          commandMonoNoteOnLower(note, velocity, lowerData[P_NotePriority]);
        } else if (lowerData[P_keyboardMode] == 3) {
          commandUnisonNoteOnLower(note, velocity, lowerData[P_NotePriority]);
        }

        // Upper Split
        if (upperData[P_keyboardMode] == 1) {  // Poly2 Upper
          int upperVoice = getUpperSplitVoicePoly2(note);
          int oldNote = voiceToNoteUpper[upperVoice - 4];
          if (oldNote >= 0) {
            releaseVoice(oldNote, upperVoice);
            voiceAssignmentUpper[oldNote] = -1;
          }
          assignVoice(note, velocity, upperVoice);
          voiceAssignmentUpper[note] = upperVoice;
          voiceToNoteUpper[upperVoice - 4] = note;
        } else if (upperData[P_keyboardMode] == 0) {  // Poly1 Upper
          int upperVoice = getUpperSplitVoice(note);
          assignVoice(note, velocity, upperVoice);
          voiceAssignmentUpper[note] = upperVoice;
          voiceToNoteUpper[upperVoice - 4] = note;
        } else if (upperData[P_keyboardMode] == 2) {
          commandMonoNoteOnUpper(note, velocity, upperData[P_NotePriority]);
        } else if (upperData[P_keyboardMode] == 3) {
          commandUnisonNoteOnUpper(note, velocity, upperData[P_NotePriority]);
        }
      }
      break;

      // SPLIT MODE (Also explicitly corrected, place here clearly):
    case 2:  // SPLIT MODE explicitly confirmed (note-on):
      if (note < splitPoint) {
        switch (lowerData[P_keyboardMode]) {
          case 0:
            voiceNum = getLowerSplitVoice(note);
            assignVoice(note, velocity, voiceNum);
            voiceAssignmentLower[note] = voiceNum;
            voiceToNoteLower[voiceNum] = note;
            break;
          case 1:
            voiceNum = getLowerSplitVoicePoly2(note);
            assignVoice(note, velocity, voiceNum);
            voiceAssignmentLower[note] = voiceNum;
            voiceToNoteLower[voiceNum] = note;
            break;
          case 2:
            commandMonoNoteOnLower(note, velocity, lowerData[P_NotePriority]);
            break;
          case 3:
            commandUnisonNoteOnLower(note, velocity, lowerData[P_NotePriority]);
            break;
        }
      } else {
        switch (upperData[P_keyboardMode]) {
          case 0:
            voiceNum = getUpperSplitVoice(note);
            assignVoice(note, velocity, voiceNum);
            voiceAssignmentUpper[note] = voiceNum;
            voiceToNoteUpper[voiceNum - 4] = note;
            break;
          case 1:
            voiceNum = getUpperSplitVoicePoly2(note);
            assignVoice(note, velocity, voiceNum);
            voiceAssignmentUpper[note] = voiceNum;
            voiceToNoteUpper[voiceNum - 4] = note;
            break;
          case 2:
            commandMonoNoteOnUpper(note, velocity, upperData[P_NotePriority]);
            break;
          case 3:
            commandUnisonNoteOnUpper(note, velocity, upperData[P_NotePriority]);
            break;
        }
      }
      break;
  }
}

void myNoteOff(byte channel, byte note, byte velocity) {

  if (isAutotuning) return;

  numberOfNotesU--;
  numberOfNotesL--;

  int assignedVoice = voiceAssignment[note];

  switch (playMode) {

    // WHOLE MODE corrected explicitly
    case 0:
      switch (lowerData[P_keyboardMode]) {
        case 0:
          assignedVoice = getVoiceNo(note) - 1;
          releaseVoice(note, assignedVoice);
          break;
        case 1:
          assignedVoice = getVoiceNoPoly2(note) - 1;
          releaseVoice(note, assignedVoice);
          break;
        case 2: commandMonoNoteOff(note); break;
        case 3: commandUnisonNoteOff(note); break;
      }
      break;

      // DUAL MODE corrected explicitly
    case 1:  // DUAL MODE Poly2 fix explicitly (note-off):
      {
        // Lower Split
        if (lowerData[P_keyboardMode] == 2) commandMonoNoteOffLower(note);
        else if (lowerData[P_keyboardMode] == 3) commandUnisonNoteOffLower(note);
        else {
          int lowerVoice = voiceAssignmentLower[note];
          if (lowerVoice >= 0 && lowerVoice <= 3 && voiceToNoteLower[lowerVoice] == note) {
            releaseVoice(note, lowerVoice);
            voiceAssignmentLower[note] = -1;
            voiceToNoteLower[lowerVoice] = -1;
          }
        }

        // Upper Split
        if (upperData[P_keyboardMode] == 2) commandMonoNoteOffUpper(note);
        else if (upperData[P_keyboardMode] == 3) commandUnisonNoteOffUpper(note);
        else {
          int upperVoice = voiceAssignmentUpper[note];
          if (upperVoice >= 4 && upperVoice <= 7 && voiceToNoteUpper[upperVoice - 4] == note) {
            releaseVoice(note, upperVoice);
            voiceAssignmentUpper[note] = -1;
            voiceToNoteUpper[upperVoice - 4] = -1;
          }
        }
      }
      break;

      // SPLIT MODE corrected explicitly
    case 2:  // SPLIT MODE explicitly corrected (note-off):
      {
        if (note < splitPoint) {
          if (lowerData[P_keyboardMode] == 2) {
            commandMonoNoteOffLower(note);
          } else if (lowerData[P_keyboardMode] == 3) {
            commandUnisonNoteOffLower(note);
          } else {
            int lowerVoice = voiceAssignmentLower[note];
            if (lowerVoice >= 0 && lowerVoice <= 3 && voiceToNoteLower[lowerVoice] == note) {
              releaseVoice(note, lowerVoice);
              voiceAssignmentLower[note] = -1;
              voiceToNoteLower[lowerVoice] = -1;
            }
          }
        } else {
          if (upperData[P_keyboardMode] == 2) {
            commandMonoNoteOffUpper(note);
          } else if (upperData[P_keyboardMode] == 3) {
            commandUnisonNoteOffUpper(note);
          } else {
            int upperVoice = voiceAssignmentUpper[note];
            if (upperVoice >= 4 && upperVoice <= 7 && voiceToNoteUpper[upperVoice - 4] == note) {
              releaseVoice(note, upperVoice);
              voiceAssignmentUpper[note] = -1;
              voiceToNoteUpper[upperVoice - 4] = -1;
            }
          }
        }
      }
      break;
  }
}

void commandMonoNoteOn(byte note, byte velocity) {
  notesWhole[note] = true;
  noteMsg = note;
  noteVel = velocity;
  orderIndxWhole = (orderIndxWhole + 1) % 40;
  noteOrderWhole[orderIndxWhole] = note;

  if (lowerData[P_NotePriority] == 0) commandTopNoteWhole();
  else if (lowerData[P_NotePriority] == 1) commandBottomNoteWhole();
  else commandLastNoteWhole();
}

void commandMonoNoteOff(byte note) {
  notesWhole[note] = false;
  noteMsg = note;
  commandLastNoteWhole();
}

void commandTopNoteWhole() {
  int topNote = -1;
  for (int i = 0; i < 128; i++)
    if (notesWhole[i]) topNote = i;

  if (topNote >= 0) assignVoice(topNote, noteVel, 0);
  else releaseVoice(noteMsg, 0);
}

void commandBottomNoteWhole() {
  int bottomNote = -1;
  for (int i = 127; i >= 0; i--)
    if (notesWhole[i]) bottomNote = i;

  if (bottomNote >= 0) assignVoice(bottomNote, noteVel, 0);
  else releaseVoice(noteMsg, 0);
}

void commandLastNoteWhole() {
  for (int i = 0; i < 40; i++) {
    int8_t idx = noteOrderWhole[mod(orderIndxWhole - i, 40)];
    if (notesWhole[idx]) {
      assignVoice(idx, noteVel, 0);
      return;
    }
  }
  releaseVoice(noteMsg, 0);
}

void commandUnisonNoteOn(byte note, byte velocity) {
  notesWhole[note] = true;
  noteMsg = note;
  noteVel = velocity;
  orderIndxWhole = (orderIndxWhole + 1) % 40;
  noteOrderWhole[orderIndxWhole] = note;

  if (lowerData[P_NotePriority] == 0) commandTopNoteUniWhole();
  else if (lowerData[P_NotePriority] == 1) commandBottomNoteUniWhole();
  else commandLastNoteUniWhole();
}

void commandUnisonNoteOff(byte note) {
  notesWhole[note] = false;
  noteMsg = note;
  commandLastNoteUniWhole();
}

void commandTopNoteUniWhole() {
  int topNote = -1;
  for (int i = 0; i < 128; i++)
    if (notesWhole[i]) topNote = i;
  if (topNote >= 0)
    for (int v = 0; v < 8; v++) assignVoice(topNote, noteVel, v);
  else
    for (int v = 0; v < 8; v++) releaseVoice(noteMsg, v);
}

void commandBottomNoteUniWhole() {
  int bottomNote = -1;
  for (int i = 127; i >= 0; i--)
    if (notesWhole[i]) bottomNote = i;
  if (bottomNote >= 0)
    for (int v = 0; v < 8; v++) assignVoice(bottomNote, noteVel, v);
  else
    for (int v = 0; v < 8; v++) releaseVoice(noteMsg, v);
}

void commandLastNoteUniWhole() {
  for (int i = 0; i < 40; i++) {
    int8_t idx = noteOrderWhole[mod(orderIndxWhole - i, 40)];
    if (notesWhole[idx]) {
      for (int v = 0; v < 8; v++) assignVoice(idx, noteVel, v);
      return;
    }
  }
  for (int v = 0; v < 8; v++) releaseVoice(noteMsg, v);
}


void commandMonoNoteOnUpper(byte note, byte velocity, byte priority) {
  notesUpper[note] = true;
  noteMsg = note;
  noteVel = velocity;
  orderIndxUpper = (orderIndxUpper + 1) % 40;
  noteOrderUpper[orderIndxUpper] = note;
  if (priority == 0) commandTopNoteUpper();
  else if (priority == 1) commandBottomNoteUpper();
  else commandLastNoteUpper();
}

void commandMonoNoteOffUpper(byte note) {
  notesUpper[note] = false;
  noteMsg = note;
  commandLastNoteUpper();
}

void commandMonoNoteOnLower(byte note, byte velocity, byte priority) {
  notesLower[note] = true;
  noteMsg = note;
  noteVel = velocity;
  orderIndxLower = (orderIndxLower + 1) % 40;
  noteOrderLower[orderIndxLower] = note;

  if (priority == 0) commandTopNoteLower();
  else if (priority == 1) commandBottomNoteLower();
  else commandLastNoteLower();
}

void commandMonoNoteOffLower(byte note) {
  notesLower[note] = false;
  noteMsg = note;
  commandLastNoteLower();
}

void commandUnisonNoteOnUpper(byte note, byte velocity, byte priority) {
  notesUpper[note] = true;
  noteMsg = note;                                       // explicitly set here
  noteVel = velocity;                                   // explicitly set here
  if (priority == 0) commandTopNoteUniUpper();          // Highest priority
  else if (priority == 1) commandBottomNoteUniUpper();  // Lowest priority
  else commandLastNoteUniUpper();                       // Last note priority
}

void commandUnisonNoteOffUpper(byte note) {
  notesUpper[note] = false;
  noteMsg = note;  // explicitly set here
  commandLastNoteUniUpper();
}

void commandUnisonNoteOnLower(byte note, byte velocity, byte priority) {
  notesLower[note] = true;
  noteMsg = note;                                       // explicitly set here
  noteVel = velocity;                                   // explicitly set here
  if (priority == 0) commandTopNoteUniLower();          // Highest priority
  else if (priority == 1) commandBottomNoteUniLower();  // Lowest priority
  else commandLastNoteUniLower();                       // Last note priority
}

void commandUnisonNoteOffLower(byte note) {
  notesLower[note] = false;
  noteMsg = note;  // explicitly set here
  commandLastNoteUniLower();
}

int getUpperSplitVoice(byte note) {
  for (int i = 0; i < 4; i++) {
    int idx = 4 + (upperSplitVoicePointer + i) % 4;
    if (!voiceOn[idx]) {
      upperSplitVoicePointer = (idx + 1) % 4;
      return idx;
    }
  }
  // fallback oldest (poly2 style if no voice free)
  int oldest = 4;
  unsigned long oldestTime = voices[4].timeOn;
  for (int i = 5; i < 8; i++)
    if (voices[i].timeOn < oldestTime) {
      oldest = i;
      oldestTime = voices[i].timeOn;
    }
  upperSplitVoicePointer = ((oldest - 4) + 1) % 4;
  return oldest;
}

int getLowerSplitVoice(byte note) {
  for (int i = 0; i < 4; i++) {
    int idx = (lowerSplitVoicePointer + i) % 4;
    if (!voiceOn[idx]) {
      lowerSplitVoicePointer = (idx + 1) % 4;
      return idx;
    }
  }
  int oldest = 0;
  unsigned long oldestTime = voices[0].timeOn;
  for (int i = 1; i < 4; i++)
    if (voices[i].timeOn < oldestTime) {
      oldest = i;
      oldestTime = voices[i].timeOn;
    }
  lowerSplitVoicePointer = (oldest + 1) % 4;
  return oldest;
}

int getLowerSplitVoicePoly2(byte note) {
  for (int i = 0; i < 4; i++)
    if (!voiceOn[i]) return i;

  int oldest = 0;
  unsigned long oldestTime = voices[0].timeOn;

  for (int i = 1; i < 4; i++) {
    if (voices[i].timeOn < oldestTime) {
      oldest = i;
      oldestTime = voices[i].timeOn;
    }
  }
  return oldest;
}

int getUpperSplitVoicePoly2(byte note) {
  for (int i = 4; i < 8; i++)
    if (!voiceOn[i]) return i;

  int oldest = 4;
  unsigned long oldestTime = voices[4].timeOn;

  for (int i = 5; i < 8; i++) {
    if (voices[i].timeOn < oldestTime) {
      oldest = i;
      oldestTime = voices[i].timeOn;
    }
  }
  return oldest;
}


// Leave these functions as-is
void assignVoice(byte note, byte velocity, int voiceIdx) {
  if (voiceIdx >= 0 && voiceIdx < 8) {
    voices[voiceIdx].note = note;
    voices[voiceIdx].velocity = velocity;
    voices[voiceIdx].timeOn = millis();
    MIDI6.sendNoteOn(note, velocity, voiceIdx + 1);
    voiceOn[voiceIdx] = true;
  }
}

void releaseVoice(byte note, int voiceIdx) {
  if (voiceIdx >= 0 && voiceIdx < 8 && voices[voiceIdx].note == note) {
    MIDI6.sendNoteOn(note, 0, voiceIdx + 1);
    voices[voiceIdx].note = -1;
    voiceOn[voiceIdx] = false;

    if (voiceIdx < 4) {
      voiceAssignmentLower[note] = -1;
      voiceToNoteLower[voiceIdx] = -1;
    } else {
      voiceAssignmentUpper[note] = -1;
      voiceToNoteUpper[voiceIdx - 4] = -1;
    }
  }
}

int getVoiceNoPoly2(int note) {
  voiceToReturn = -1;       // Initialize to 'null'
  earliestTime = millis();  // Initialize to now

  if (note == -1) {
    // NoteOn() - Get the oldest free voice (recent voices may still be on the release stage)
    if (voices[lastUsedVoice].note == -1) {
      return lastUsedVoice + 1;
    }

    // If the last used voice is not free or doesn't exist, check if the first voice is free
    if (voices[0].note == -1) {
      return 1;
    }

    // Find the lowest available voice for the new note
    for (int i = 0; i < NO_OF_VOICES; i++) {
      if (voices[i].note == -1) {
        return i + 1;
      }
    }

    // If no voice is available, release the oldest note
    int oldestVoice = 0;
    for (int i = 1; i < NO_OF_VOICES; i++) {
      if (voices[i].timeOn < voices[oldestVoice].timeOn) {
        oldestVoice = i;
      }
    }
    return oldestVoice + 1;
  } else {
    // NoteOff() - Get the voice number from the note
    for (int i = 0; i < NO_OF_VOICES; i++) {
      if (voices[i].note == note) {
        return i + 1;
      }
    }
  }

  // Shouldn't get here, return voice 1
  return 1;
}


int getVoiceNo(int note) {
  voiceToReturn = -1;       //Initialise to 'null'
  earliestTime = millis();  //Initialise to now
  if (note == -1) {
    //NoteOn() - Get the oldest free voice (recent voices may be still on release stage)
    for (int i = 0; i < NO_OF_VOICES; i++) {
      if (voices[i].note == -1) {
        if (voices[i].timeOn < earliestTime) {
          earliestTime = voices[i].timeOn;
          voiceToReturn = i;
        }
      }
    }
    if (voiceToReturn == -1) {
      //No free voices, need to steal oldest sounding voice
      earliestTime = millis();  //Reinitialise
      for (int i = 0; i < NO_OF_VOICES; i++) {
        if (voices[i].timeOn < earliestTime) {
          earliestTime = voices[i].timeOn;
          voiceToReturn = i;
        }
      }
    }
    return voiceToReturn + 1;
  } else {
    //NoteOff() - Get voice number from note
    for (int i = 0; i < NO_OF_VOICES; i++) {
      if (voices[i].note == note) {
        return i + 1;
      }
    }
  }
  //Shouldn't get here, return voice 1
  return 1;
}

void DinHandlePitchBend(byte channel, int pitch) {
  if (wholemode) {
    MIDI6.sendPitchBend(pitch, 1);
    MIDI6.sendPitchBend(pitch, 2);
  }
  if (dualmode) {
    MIDI6.sendPitchBend(pitch, 1);
    MIDI6.sendPitchBend(pitch, 2);
  }
  if (splitmode) {
    MIDI6.sendPitchBend(pitch, 1);
    MIDI6.sendPitchBend(pitch, 2);
  }
}

void getDelayTime() {
  delaytimeL = (lowerData[P_LFODelay]);
  if (delaytimeL <= 0) {
    delaytimeL = 0.1;
  }
  intervalL = (delaytimeL * 10);

  delaytimeU = (upperData[P_LFODelay]);
  if (delaytimeU <= 0) {
    delaytimeU = 0.1;
  }
  intervalU = (delaytimeU * 10);
}

void allNotesOff() {
  midiCCOut61(WSallNotesOff, 127);
  midiCCOut62(WSallNotesOff, 127);
}

void updatepwLFO(boolean announce) {

  if (announce) {
    showCurrentParameterPage("PWM Rate", int(pwLFOstr));
  }
  if (upperSW) {
    midiCCOut(CCpwLFO, upperData[P_pwLFO]);
    midiCCOut71(CCpwLFO, upperData[P_pwLFO]);
  } else {
    midiCCOut(CCpwLFO, lowerData[P_pwLFO]);
    midiCCOut71(CCpwLFO, lowerData[P_pwLFO]);
  }
}

void updatefmDepth(boolean announce) {
  if (announce) {
    showCurrentParameterPage("FM Depth", int(fmDepthstr));
  }
  if (upperSW) {
    midiCCOut62(WSFMDepth, upperData[P_fmDepth]);
    midiCCOut(CCfmDepth, upperData[P_fmDepth]);
    midiCCOut71(CCfmDepth, upperData[P_fmDepth]);
  } else {
    midiCCOut61(WSFMDepth, lowerData[P_fmDepth]);
    midiCCOut(CCfmDepth, lowerData[P_fmDepth]);
    midiCCOut71(CCfmDepth, lowerData[P_fmDepth]);
    if (wholemode) {
      midiCCOut62(WSFMDepth, upperData[P_fmDepth]);
    }
  }
}

void updateATDepth(boolean announce) {
  if (announce) {
    showCurrentParameterPage("AT Depth", int(ATDepthstr));
  }
  if (upperSW) {
    midiCCOut62(WSATmodDepth, upperData[P_ATDepth]);
    midiCCOut(CCATDepth, upperData[P_ATDepth]);
    midiCCOut71(CCATDepth, upperData[P_ATDepth]);
  } else {
    midiCCOut61(WSATmodDepth, lowerData[P_ATDepth]);
    midiCCOut(CCATDepth, lowerData[P_ATDepth]);
    midiCCOut71(CCATDepth, lowerData[P_ATDepth]);
    if (wholemode) {
      midiCCOut62(WSATmodDepth, upperData[P_ATDepth]);
    }
  }
}

void updateosc2PW(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 PW", String(osc2PWstr) + " %");
  }
  if (upperSW) {
    midiCCOut62(WSosc2PW, upperData[P_osc2PW]);
    midiCCOut(CCosc2PW, upperData[P_osc2PW]);
    midiCCOut71(CCosc2PW, upperData[P_osc2PW]);
  } else {
    midiCCOut61(WSosc2PW, lowerData[P_osc2PW]);
    midiCCOut(CCosc2PW, lowerData[P_osc2PW]);
    midiCCOut71(CCosc2PW, lowerData[P_osc2PW]);
    if (wholemode) {
      midiCCOut62(WSosc2PW, upperData[P_osc2PW]);
    }
  }
}

void updateosc2PWM(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 PWM", int(osc2PWMstr));
  }
  if (upperSW) {
    midiCCOut62(WSosc2PWM, upperData[P_osc2PWM]);
    midiCCOut(CCosc2PWM, upperData[P_osc2PWM]);
    midiCCOut71(CCosc2PWM, upperData[P_osc2PWM]);
  } else {
    midiCCOut61(WSosc2PWM, lowerData[P_osc2PWM]);
    midiCCOut(CCosc2PWM, lowerData[P_osc2PWM]);
    midiCCOut71(CCosc2PWM, lowerData[P_osc2PWM]);
    if (wholemode) {
      midiCCOut62(WSosc2PWM, upperData[P_osc2PWM]);
    }
  }
}

void updateosc1PW(boolean announce) {

  if (announce) {
    showCurrentParameterPage("OSC1 PW", String(osc1PWstr) + " %");
  }
  if (upperSW) {
    midiCCOut62(WSosc1PW, upperData[P_osc1PW]);
    midiCCOut(CCosc1PW, upperData[P_osc1PW]);
    midiCCOut71(CCosc1PW, upperData[P_osc1PW]);
  } else {
    midiCCOut61(WSosc1PW, lowerData[P_osc1PW]);
    midiCCOut(CCosc1PW, lowerData[P_osc1PW]);
    midiCCOut71(CCosc1PW, lowerData[P_osc1PW]);
    if (wholemode) {
      midiCCOut62(WSosc1PW, upperData[P_osc1PW]);
    }
  }
}

void updateosc1PWM(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC1 PWM", int(osc1PWMstr));
  }
  if (upperSW) {
    midiCCOut62(WSosc1PWM, upperData[P_osc1PWM]);
    midiCCOut(CCosc1PWM, upperData[P_osc1PWM]);
    midiCCOut71(CCosc1PWM, upperData[P_osc1PWM]);
  } else {
    midiCCOut61(WSosc1PWM, lowerData[P_osc1PWM]);
    midiCCOut(CCosc1PWM, lowerData[P_osc1PWM]);
    midiCCOut71(CCosc1PWM, lowerData[P_osc1PWM]);
    if (wholemode) {
      midiCCOut62(WSosc1PWM, upperData[P_osc1PWM]);
    }
  }
}

void updateosc1Range(boolean announce) {
  if (upperSW) {
    panelData[P_osc1Range] = upperData[P_osc1Range];
    if (upperData[P_osc1Range] == 2) {
      if (announce) {
        showCurrentParameterPage("Osc1 Range", String("8"));
      }
      midiCCOut(CCosc1Oct, 2);
      midiCCOut62(WSosc1oct, 127);
      midiCCOut72(CCosc1Oct, 2);
    } else if (upperData[P_osc1Range] == 1) {
      if (announce) {
        showCurrentParameterPage("Osc1 Range", String("16"));
      }
      midiCCOut(CCosc1Oct, 1);
      midiCCOut62(WSosc1oct, 64);
      midiCCOut72(CCosc1Oct, 1);
    } else {
      if (announce) {
        showCurrentParameterPage("Osc1 Range", String("32"));
      }
      midiCCOut(CCosc1Oct, 0);
      midiCCOut62(WSosc1oct, 0);
      midiCCOut72(CCosc1Oct, 0);
    }
  } else {
    panelData[P_osc1Range] = lowerData[P_osc1Range];
    if (lowerData[P_osc1Range] == 2) {
      if (announce) {
        showCurrentParameterPage("Osc1 Range", String("8"));
      }
      midiCCOut(CCosc1Oct, 2);
      midiCCOut61(WSosc1oct, 127);
      midiCCOut72(CCosc1Oct, 2);
      if (wholemode) {
        midiCCOut62(WSosc1oct, 127);
      }
    } else if (lowerData[P_osc1Range] == 1) {
      if (announce) {
        showCurrentParameterPage("Osc1 Range", String("16"));
      }
      midiCCOut(CCosc1Oct, 1);
      midiCCOut61(WSosc1oct, 64);
      midiCCOut72(CCosc1Oct, 1);
      if (wholemode) {
        midiCCOut62(WSosc1oct, 64);
      }
    } else {
      if (announce) {
        showCurrentParameterPage("Osc1 Range", String("32"));
      }
      midiCCOut(CCosc1Oct, 0);
      midiCCOut61(WSosc1oct, 0);
      midiCCOut72(CCosc1Oct, 0);
      if (wholemode) {
        midiCCOut62(WSosc1oct, 0);
      }
    }
  }
}

void updateosc2Range(boolean announce) {
  if (upperSW) {
    panelData[P_osc2Range] = upperData[P_osc2Range];
    if (upperData[P_osc2Range] == 2) {
      if (announce) {
        showCurrentParameterPage("Osc2 Range", String("8"));
      }
      midiCCOut62(WSosc2oct, 127);
      midiCCOut72(CCosc2Oct, 2);
      midiCCOut(CCosc2Oct, 2);
    } else if (upperData[P_osc2Range] == 1) {
      if (announce) {
        showCurrentParameterPage("Osc2 Range", String("16"));
      }
      midiCCOut62(WSosc2oct, 64);
      midiCCOut72(CCosc2Oct, 1);
      midiCCOut(CCosc2Oct, 1);
    } else {
      if (announce) {
        showCurrentParameterPage("Osc2 Range", String("32"));
      }
      midiCCOut(CCosc2Oct, 0);
      midiCCOut62(WSosc2oct, 0);
      midiCCOut72(CCosc2Oct, 0);
    }
  } else {
    panelData[P_osc2Range] = lowerData[P_osc2Range];
    if (lowerData[P_osc2Range] == 2) {
      if (announce) {
        showCurrentParameterPage("Osc2 Range", String("8"));
      }
      midiCCOut(CCosc2Oct, 2);
      midiCCOut61(WSosc2oct, 127);
      midiCCOut72(CCosc2Oct, 2);
      if (wholemode) {
        midiCCOut62(WSosc2oct, 127);
      }
    } else if (lowerData[P_osc2Range] == 1) {
      if (announce) {
        showCurrentParameterPage("Osc2 Range", String("16"));
      }
      midiCCOut(CCosc2Oct, 1);
      midiCCOut61(WSosc2oct, 64);
      midiCCOut72(CCosc2Oct, 1);
      if (wholemode) {
        midiCCOut62(WSosc2oct, 64);
      }
    } else {
      if (announce) {
        showCurrentParameterPage("Osc2 Range", String("32"));
      }
      midiCCOut(CCosc2Oct, 0);
      midiCCOut61(WSosc2oct, 0);
      midiCCOut72(CCosc2Oct, 0);
      if (wholemode) {
        midiCCOut62(WSosc2oct, 0);
      }
    }
  }
}

void updateglideTime(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Glide Time", String(glideTimestr * 10) + " Seconds");
  }
  if (upperSW) {
    midiCCOut62(WSglideTime, upperData[P_glideTime]);
    midiCCOut(CCglideTime, upperData[P_glideTime]);
    midiCCOut71(CCglideTime, upperData[P_glideTime]);
  } else {
    midiCCOut61(WSglideTime, lowerData[P_glideTime]);
    midiCCOut(CCglideTime, lowerData[P_glideTime]);
    midiCCOut71(CCglideTime, lowerData[P_glideTime]);
    if (wholemode) {
      midiCCOut62(WSglideTime, upperData[P_glideTime]);
    }
  }
}

void updateosc2Detune(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 Detune", String(osc2Detunestr));
  }
  if (upperSW) {
    midiCCOut62(WSdetune, upperData[P_osc2Detune]);
    midiCCOut(CCosc2Detune, upperData[P_osc2Detune]);
    midiCCOut71(CCosc2Detune, upperData[P_osc2Detune]);
  } else {
    midiCCOut61(WSdetune, lowerData[P_osc2Detune]);
    midiCCOut(CCosc2Detune, lowerData[P_osc2Detune]);
    midiCCOut71(CCosc2Detune, lowerData[P_osc2Detune]);
    if (wholemode) {
      midiCCOut62(WSdetune, upperData[P_osc2Detune]);
    }
  }
}

void updateosc2Interval(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 Interval", String(osc2Intervalstr));
  }
  if (upperSW) {
    midiCCOut62(WSinterval, upperData[P_osc2Interval]);
    midiCCOut(CCosc2Interval, upperData[P_osc2Interval]);
    midiCCOut71(CCosc2Interval, upperData[P_osc2Interval]);
  } else {
    midiCCOut61(WSinterval, lowerData[P_osc2Interval]);
    midiCCOut(CCosc2Interval, lowerData[P_osc2Interval]);
    midiCCOut71(CCosc2Interval, lowerData[P_osc2Interval]);
    if (wholemode) {
      midiCCOut62(WSinterval, upperData[P_osc2Interval]);
    }
  }
}

void updatenoiseLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Noise Level", String(noiseLevelstr));
  }
  if (upperSW) {
    midiCCOut(CCnoiseLevel, upperData[P_noiseLevel]);
    midiCCOut71(CCnoiseLevel, upperData[P_noiseLevel]);
  } else {
    midiCCOut(CCnoiseLevel, lowerData[P_noiseLevel]);
    midiCCOut71(CCnoiseLevel, lowerData[P_noiseLevel]);
  }
}

void updateOsc2SawLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 Saw", int(osc2SawLevelstr));
  }
  if (upperSW) {
    midiCCOut(CCosc2SawLevel, upperData[P_osc2SawLevel]);
    midiCCOut71(CCosc2SawLevel, upperData[P_osc2SawLevel]);
  } else {
    midiCCOut(CCosc2SawLevel, lowerData[P_osc2SawLevel]);
    midiCCOut71(CCosc2SawLevel, lowerData[P_osc2SawLevel]);
  }
}

void updateOsc1SawLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC1 Saw", int(osc1SawLevelstr));
  }
  if (upperSW) {
    midiCCOut(CCosc1SawLevel, upperData[P_osc1SawLevel]);
    midiCCOut71(CCosc1SawLevel, upperData[P_osc1SawLevel]);
  } else {
    midiCCOut(CCosc1SawLevel, lowerData[P_osc1SawLevel]);
    midiCCOut71(CCosc1SawLevel, lowerData[P_osc1SawLevel]);
  }
}

void updateOsc2PulseLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 Pulse", int(osc2PulseLevelstr));
  }
  if (upperSW) {
    midiCCOut(CCosc2PulseLevel, upperData[P_osc2PulseLevel]);
    midiCCOut71(CCosc2PulseLevel, upperData[P_osc2PulseLevel]);
  } else {
    midiCCOut(CCosc2PulseLevel, lowerData[P_osc2PulseLevel]);
    midiCCOut71(CCosc2PulseLevel, lowerData[P_osc2PulseLevel]);
  }
}

void updateOsc1PulseLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC1 Pulse", int(osc1PulseLevelstr));
  }
  if (upperSW) {
    midiCCOut(CCosc1PulseLevel, upperData[P_osc1PulseLevel]);
    midiCCOut71(CCosc1PulseLevel, upperData[P_osc1PulseLevel]);
  } else {
    midiCCOut(CCosc1PulseLevel, lowerData[P_osc1PulseLevel]);
    midiCCOut71(CCosc1PulseLevel, lowerData[P_osc1PulseLevel]);
  }
}

void updateOsc2TriangleLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 Triangle", int(osc2TriangleLevelstr));
  }
  if (upperSW) {
    midiCCOut(CCosc2TriangleLevel, upperData[P_osc2TriangleLevel]);
    midiCCOut71(CCosc2TriangleLevel, upperData[P_osc2TriangleLevel]);
  } else {
    midiCCOut(CCosc2TriangleLevel, lowerData[P_osc2TriangleLevel]);
    midiCCOut71(CCosc2TriangleLevel, lowerData[P_osc2TriangleLevel]);
  }
}

void updateOsc1SubLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC1 Sub", int(osc1SubLevelstr));
  }
  if (upperSW) {
    midiCCOut(CCosc1SubLevel, upperData[P_osc1SubLevel]);
    midiCCOut71(CCosc1SubLevel, upperData[P_osc1SubLevel]);
  } else {
    midiCCOut(CCosc1SubLevel, lowerData[P_osc1SubLevel]);
    midiCCOut71(CCosc1SubLevel, lowerData[P_osc1SubLevel]);
  }
}

void updateamDepth(boolean announce) {
  if (announce) {
    showCurrentParameterPage("AM Depth", int(amDepthstr));
  }
  if (upperSW) {
    midiCCOut(CCamDepth, upperData[P_amDepth]);
    midiCCOut71(CCamDepth, upperData[P_amDepth]);
  } else {
    midiCCOut(CCamDepth, lowerData[P_amDepth]);
    midiCCOut71(CCamDepth, lowerData[P_amDepth]);
  }
}

void updateFilterCutoff(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Cutoff", String(filterCutoffstr) + " Hz");
  }
  if (upperSW) {
    midiCCOut(CCfilterCutoff, upperData[P_filterCutoff]);
    midiCCOut71(CCfilterCutoff, upperData[P_filterCutoff]);
  } else {
    midiCCOut(CCfilterCutoff, lowerData[P_filterCutoff]);
    midiCCOut71(CCfilterCutoff, lowerData[P_filterCutoff]);
  }
}

void updatefilterLFO(boolean announce) {
  if (announce) {
    showCurrentParameterPage("TM depth", int(filterLFOstr));
  }
  if (upperSW) {
    midiCCOut(CCfilterLFO, upperData[P_filterLFO]);
    midiCCOut71(CCfilterLFO, upperData[P_filterLFO]);
  } else {
    midiCCOut(CCfilterLFO, lowerData[P_filterLFO]);
    midiCCOut71(CCfilterLFO, lowerData[P_filterLFO]);
  }
}

void updatefilterRes(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Resonance", int(filterResstr));
  }
  if (upperSW) {
    midiCCOut(CCfilterRes, upperData[P_filterRes]);
    midiCCOut71(CCfilterRes, upperData[P_filterRes]);
  } else {
    midiCCOut(CCfilterRes, lowerData[P_filterRes]);
    midiCCOut71(CCfilterRes, lowerData[P_filterRes]);
  }
}

void updateFilterType(boolean announce) {
  if (upperSW) {
    switch (upperData[P_filterType]) {
      case 0:
        if (upperData[P_filterPoleSW] == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("3P LowPass"));
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("4P LowPass"));
          }
        }
        midiCCOut72(CCfilterType, 0);
        midiCCOut(CCfilterType, 0);
        srp.writePin(FILTERA_UPPER, LOW);
        srp.writePin(FILTERB_UPPER, LOW);
        srp.writePin(FILTERC_UPPER, LOW);
        break;

      case 1:
        if (upperData[P_filterPoleSW] == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("1P LowPass"));
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P LowPass"));
          }
        }
        midiCCOut72(CCfilterType, 1);
        midiCCOut(CCfilterType, 1);
        srp.writePin(FILTERA_UPPER, HIGH);
        srp.writePin(FILTERB_UPPER, LOW);
        srp.writePin(FILTERC_UPPER, LOW);
        break;

      case 2:
        if (upperData[P_filterPoleSW] == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("3P HP + 1P LP"));
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("4P HighPass"));
          }
        }
        midiCCOut72(CCfilterType, 2);
        midiCCOut(CCfilterType, 2);
        srp.writePin(FILTERA_UPPER, LOW);
        srp.writePin(FILTERB_UPPER, HIGH);
        srp.writePin(FILTERC_UPPER, LOW);
        break;

      case 3:
        if (upperData[P_filterPoleSW] == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("1P HP + 1P LP"));
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P HighPass"));
          }
        }
        midiCCOut72(CCfilterType, 3);
        midiCCOut(CCfilterType, 3);
        srp.writePin(FILTERA_UPPER, HIGH);
        srp.writePin(FILTERB_UPPER, HIGH);
        srp.writePin(FILTERC_UPPER, LOW);
        break;

      case 4:
        if (upperData[P_filterPoleSW] == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P HP + 1P LP"));
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("4P BandPass"));
          }
        }
        midiCCOut72(CCfilterType, 4);
        midiCCOut(CCfilterType, 4);
        srp.writePin(FILTERA_UPPER, LOW);
        srp.writePin(FILTERB_UPPER, LOW);
        srp.writePin(FILTERC_UPPER, HIGH);
        break;

      case 5:
        if (upperData[P_filterPoleSW] == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P BP + 1P LP"));
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P BandPass"));
          }
        }
        midiCCOut72(CCfilterType, 5);
        midiCCOut(CCfilterType, 5);
        srp.writePin(FILTERA_UPPER, HIGH);
        srp.writePin(FILTERB_UPPER, LOW);
        srp.writePin(FILTERC_UPPER, HIGH);
        break;

      case 6:
        if (upperData[P_filterPoleSW] == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("3P AP + 1P LP"));
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("3P AllPass"));
          }
        }
        midiCCOut72(CCfilterType, 6);
        midiCCOut(CCfilterType, 6);
        srp.writePin(FILTERA_UPPER, LOW);
        srp.writePin(FILTERB_UPPER, HIGH);
        srp.writePin(FILTERC_UPPER, HIGH);
        break;

      case 7:
        if (upperData[P_filterPoleSW] == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P Notch + LP"));
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("Notch"));
          }
        }
        midiCCOut72(CCfilterType, 7);
        midiCCOut(CCfilterType, 7);
        srp.writePin(FILTERA_UPPER, HIGH);
        srp.writePin(FILTERB_UPPER, HIGH);
        srp.writePin(FILTERC_UPPER, HIGH);
        break;
    }
  } else {
    switch (lowerData[P_filterType]) {
      case 0:
        if (lowerData[P_filterPoleSW] == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("3P LowPass"));
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("4P LowPass"));
          }
        }
        midiCCOut72(CCfilterType, 0);
        midiCCOut(CCfilterType, 0);
        srp.writePin(FILTERA_LOWER, LOW);
        srp.writePin(FILTERB_LOWER, LOW);
        srp.writePin(FILTERC_LOWER, LOW);
        if (wholemode) {
          srp.writePin(FILTERA_UPPER, LOW);
          srp.writePin(FILTERB_UPPER, LOW);
          srp.writePin(FILTERC_UPPER, LOW);
        }
        break;

      case 1:
        if (lowerData[P_filterPoleSW] == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("1P LowPass"));
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P LowPass"));
          }
        }
        midiCCOut72(CCfilterType, 1);
        midiCCOut(CCfilterType, 1);
        srp.writePin(FILTERA_LOWER, HIGH);
        srp.writePin(FILTERB_LOWER, LOW);
        srp.writePin(FILTERC_LOWER, LOW);
        if (wholemode) {
          srp.writePin(FILTERA_UPPER, HIGH);
          srp.writePin(FILTERB_UPPER, LOW);
          srp.writePin(FILTERC_UPPER, LOW);
        }
        break;

      case 2:
        if (lowerData[P_filterPoleSW] == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("3P HP + 1P LP"));
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("4P HighPass"));
          }
        }
        midiCCOut72(CCfilterType, 2);
        midiCCOut(CCfilterType, 2);
        srp.writePin(FILTERA_LOWER, LOW);
        srp.writePin(FILTERB_LOWER, HIGH);
        srp.writePin(FILTERC_LOWER, LOW);
        if (wholemode) {
          srp.writePin(FILTERA_UPPER, LOW);
          srp.writePin(FILTERB_UPPER, HIGH);
          srp.writePin(FILTERC_UPPER, LOW);
        }
        break;

      case 3:
        if (lowerData[P_filterPoleSW] == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("1P HP + 1P LP"));
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P HighPass"));
          }
        }
        midiCCOut72(CCfilterType, 3);
        midiCCOut(CCfilterType, 3);
        srp.writePin(FILTERA_LOWER, HIGH);
        srp.writePin(FILTERB_LOWER, HIGH);
        srp.writePin(FILTERC_LOWER, LOW);
        if (wholemode) {
          srp.writePin(FILTERA_UPPER, HIGH);
          srp.writePin(FILTERB_UPPER, HIGH);
          srp.writePin(FILTERC_UPPER, LOW);
        }
        break;

      case 4:
        if (lowerData[P_filterPoleSW] == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P HP + 1P LP"));
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("4P BandPass"));
          }
        }
        midiCCOut72(CCfilterType, 4);
        midiCCOut(CCfilterType, 4);
        srp.writePin(FILTERA_LOWER, LOW);
        srp.writePin(FILTERB_LOWER, LOW);
        srp.writePin(FILTERC_LOWER, HIGH);
        if (wholemode) {
          srp.writePin(FILTERA_UPPER, LOW);
          srp.writePin(FILTERB_UPPER, LOW);
          srp.writePin(FILTERC_UPPER, HIGH);
        }
        break;

      case 5:
        if (lowerData[P_filterPoleSW] == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P BP + 1P LP"));
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P BandPass"));
          }
        }
        midiCCOut72(CCfilterType, 5);
        midiCCOut(CCfilterType, 5);
        srp.writePin(FILTERA_LOWER, HIGH);
        srp.writePin(FILTERB_LOWER, LOW);
        srp.writePin(FILTERC_LOWER, HIGH);
        if (wholemode) {
          srp.writePin(FILTERA_UPPER, HIGH);
          srp.writePin(FILTERB_UPPER, LOW);
          srp.writePin(FILTERC_UPPER, HIGH);
        }
        break;


      case 6:
        if (lowerData[P_filterPoleSW] == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("3P AP + 1P LP"));
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("3P AllPass"));
          }
        }
        midiCCOut72(CCfilterType, 6);
        midiCCOut(CCfilterType, 6);
        srp.writePin(FILTERA_LOWER, LOW);
        srp.writePin(FILTERB_LOWER, HIGH);
        srp.writePin(FILTERC_LOWER, HIGH);
        if (wholemode) {
          srp.writePin(FILTERA_UPPER, LOW);
          srp.writePin(FILTERB_UPPER, HIGH);
          srp.writePin(FILTERC_UPPER, HIGH);
        }
        break;

      case 7:
        if (lowerData[P_filterPoleSW] == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P Notch + LP"));
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("Notch"));
          }
        }
        midiCCOut72(CCfilterType, 7);
        midiCCOut(CCfilterType, 7);
        srp.writePin(FILTERA_LOWER, HIGH);
        srp.writePin(FILTERB_LOWER, HIGH);
        srp.writePin(FILTERC_LOWER, HIGH);
        if (wholemode) {
          srp.writePin(FILTERA_UPPER, HIGH);
          srp.writePin(FILTERB_UPPER, HIGH);
          srp.writePin(FILTERC_UPPER, HIGH);
        }
        break;
    }
  }
}

void updatefilterEGlevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("EG Depth", int(filterEGlevelstr));
  }
  if (upperSW) {
    midiCCOut(CCfilterEGlevel, upperData[P_filterEGlevel]);
    midiCCOut71(CCfilterEGlevel, upperData[P_filterEGlevel]);
  } else {
    midiCCOut(CCfilterEGlevel, lowerData[P_filterEGlevel]);
    midiCCOut71(CCfilterEGlevel, lowerData[P_filterEGlevel]);
  }
}

void updatekeytrack(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Keytrack", int(keytrackstr));
  }
  if (upperSW) {
    midiCCOut62(WSkeytrack, upperData[P_keytrack]);
    midiCCOut(CCkeyTrack, upperData[P_keytrack]);
    midiCCOut71(CCkeyTrack, upperData[P_keytrack]);
  } else {
    midiCCOut61(WSkeytrack, lowerData[P_keytrack]);
    midiCCOut(CCkeyTrack, lowerData[P_keytrack]);
    midiCCOut71(CCkeyTrack, lowerData[P_keytrack]);
    if (wholemode) {
      midiCCOut62(WSkeytrack, upperData[P_keytrack]);
    }
  }
}

void updateLFORate(boolean announce) {

  if (announce) {
    showCurrentParameterPage("LFO Rate", String(LFORatestr) + " Hz");
  }
  if (upperSW) {
    midiCCOut(CCLFORate, upperData[P_LFORate]);
    midiCCOut71(CCLFORate, upperData[P_LFORate]);
  } else {
    midiCCOut(CCLFORate, lowerData[P_LFORate]);
    midiCCOut71(CCLFORate, lowerData[P_LFORate]);
  }
}

void updateLFODelay(boolean announce) {
  if (announce) {
    showCurrentParameterPage("LFO Delay", String(LFODelaystr));
  }
  if (upperSW) {
    midiCCOut(CCLFODelay, upperData[P_LFODelay]);
    midiCCOut71(CCLFODelay, upperData[P_LFODelay]);
  } else {
    midiCCOut(CCLFODelay, lowerData[P_LFODelay]);
    midiCCOut71(CCLFODelay, lowerData[P_LFODelay]);
  }
}

void updatemodWheelDepth(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Mod Wheel Depth", String(modWheelDepthstr));
  }
  if (upperSW) {
    midiCCOut62(WSmodDepth, upperData[P_modWheelDepth]);
    midiCCOut(CCmodWheelDepth, upperData[P_modWheelDepth]);
    midiCCOut71(CCmodWheelDepth, upperData[P_modWheelDepth]);
  } else {
    midiCCOut61(WSmodDepth, lowerData[P_modWheelDepth]);
    midiCCOut(CCmodWheelDepth, lowerData[P_modWheelDepth]);
    midiCCOut71(CCmodWheelDepth, lowerData[P_modWheelDepth]);
    if (wholemode) {
      midiCCOut62(WSmodDepth, upperData[P_modWheelDepth]);
    }
  }
}

void updatePitchBendDepth(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Pitch Bend Depth", String(PitchBendLevelstr));
  }
  if (upperSW) {
    midiCCOut62(WSbendRange, upperData[P_PitchBendLevel]);
    midiCCOut71(CCPitchBend, upperData[P_PitchBendLevel]);
  } else {
    midiCCOut61(WSbendRange, lowerData[P_PitchBendLevel]);
    midiCCOut71(CCPitchBend, lowerData[P_PitchBendLevel]);
    if (wholemode) {
      midiCCOut62(WSbendRange, upperData[P_PitchBendLevel]);
    }
  }
}

void updateeffectPot1(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Effect Pot 1", String(effectPot1str));
  }
  if (upperSW) {
    midiCCOut(CCeffectPot1, upperData[P_effectPot1]);
    midiCCOut71(CCeffectPot1, upperData[P_effectPot1]);
  } else {
    midiCCOut(CCeffectPot1, lowerData[P_effectPot1]);
    midiCCOut71(CCeffectPot1, lowerData[P_effectPot1]);
  }
}

void updateeffectPot2(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Effect Pot 2", String(effectPot2str));
  }
  if (upperSW) {
    midiCCOut(CCeffectPot2, upperData[P_effectPot2]);
    midiCCOut71(CCeffectPot2, upperData[P_effectPot2]);
  } else {
    midiCCOut(CCeffectPot2, lowerData[P_effectPot2]);
    midiCCOut71(CCeffectPot2, lowerData[P_effectPot2]);
  }
}

void updateeffectPot3(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Effect Pot 3", String(effectPot3str));
  }
  if (upperSW) {
    midiCCOut(CCeffectPot3, upperData[P_effectPot3]);
    midiCCOut71(CCeffectPot3, upperData[P_effectPot3]);
  } else {
    midiCCOut(CCeffectPot3, lowerData[P_effectPot3]);
    midiCCOut71(CCeffectPot3, lowerData[P_effectPot3]);
  }
}

void updateeffectsMix(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Effects Mix", String(effectsMixstr));
  }
  if (upperSW) {
    midiCCOut(CCeffectsMix, upperData[P_effectsMix]);
    midiCCOut71(CCeffectsMix, upperData[P_effectsMix]);
  } else {
    midiCCOut(CCeffectsMix, lowerData[P_effectsMix]);
    midiCCOut71(CCeffectsMix, lowerData[P_effectsMix]);
  }
}

void updateStratusLFOWaveform(boolean announce) {

  if (upperSW) {
    panelData[P_LFOWaveform] = upperData[P_LFOWaveform];
    panelData[P_lfoAlt] = upperData[P_lfoAlt];
  } else {
    panelData[P_LFOWaveform] = lowerData[P_LFOWaveform];
    panelData[P_lfoAlt] = lowerData[P_lfoAlt];
  }

  if (panelData[P_lfoAlt]) {
    switch (panelData[P_LFOWaveform]) {
      case 0:
        StratusLFOWaveform = "Sawtooth Up";
        LFOWaveCV = 10;
        midiCCOut72(CCLFOWaveform, 0);
        break;

      case 1:
        StratusLFOWaveform = "Sawtooth Down";
        LFOWaveCV = 160;
        midiCCOut72(CCLFOWaveform, 1);
        break;

      case 2:
        StratusLFOWaveform = "Squarewave";
        LFOWaveCV = 280;
        midiCCOut72(CCLFOWaveform, 2);
        break;

      case 3:
        StratusLFOWaveform = "Triangle";
        LFOWaveCV = 400;
        midiCCOut72(CCLFOWaveform, 3);
        break;

      case 4:
        StratusLFOWaveform = "Sinewave";
        LFOWaveCV = 592;
        midiCCOut72(CCLFOWaveform, 4);
        break;

      case 5:
        StratusLFOWaveform = "Sweeps";
        LFOWaveCV = 720;
        midiCCOut72(CCLFOWaveform, 5);
        break;

      case 6:
        StratusLFOWaveform = "Lumps";
        LFOWaveCV = 840;
        midiCCOut72(CCLFOWaveform, 6);
        break;

      case 7:
        StratusLFOWaveform = "Sample & Hold";
        LFOWaveCV = 968;
        midiCCOut72(CCLFOWaveform, 7);
        break;
    }
  } else {
    switch (panelData[P_LFOWaveform]) {
      case 0:
        StratusLFOWaveform = "Saw +Oct";
        LFOWaveCV = 10;
        midiCCOut72(CCLFOWaveform, 0);
        break;

      case 1:
        StratusLFOWaveform = "Quad Saw";
        LFOWaveCV = 160;
        midiCCOut72(CCLFOWaveform, 1);
        break;

      case 2:
        StratusLFOWaveform = "Quad Pulse";
        LFOWaveCV = 280;
        midiCCOut72(CCLFOWaveform, 2);
        break;

      case 3:
        StratusLFOWaveform = "Tri Step";
        LFOWaveCV = 400;
        midiCCOut72(CCLFOWaveform, 3);
        break;

      case 4:
        StratusLFOWaveform = "Sine +Oct";
        LFOWaveCV = 592;
        midiCCOut72(CCLFOWaveform, 4);
        break;

      case 5:
        StratusLFOWaveform = "Sine +3rd";
        LFOWaveCV = 720;
        midiCCOut72(CCLFOWaveform, 5);
        break;

      case 6:
        StratusLFOWaveform = "Sine +4th";
        LFOWaveCV = 840;
        midiCCOut72(CCLFOWaveform, 6);
        break;

      case 7:
        StratusLFOWaveform = "Rand Slopes";
        LFOWaveCV = 968;
        midiCCOut72(CCLFOWaveform, 7);
        break;
    }
  }
  if (announce) {
    showCurrentParameterPage("LFO Wave", StratusLFOWaveform);
  }
  if (upperSW) {
    LFOWaveCVupper = LFOWaveCV;
  } else {
    LFOWaveCVlower = LFOWaveCV;
    if (wholemode) {
      LFOWaveCVupper = LFOWaveCV;
    }
  }
}

void updatefilterAttack(boolean announce) {
  if (announce) {
    if (filterAttackstr < 1000) {
      showCurrentParameterPage("VCF Attack", String(int(filterAttackstr)) + " ms", FILTER_ENV);
    } else {
      showCurrentParameterPage("VCF Attack", String(filterAttackstr * 0.001) + " s", FILTER_ENV);
    }
  }
  if (upperSW) {
    midiCCOut(CCfilterAttack, upperData[P_filterAttack]);
    midiCCOut71(CCfilterAttack, upperData[P_filterAttack]);
  } else {
    midiCCOut(CCfilterAttack, lowerData[P_filterAttack]);
    midiCCOut71(CCfilterAttack, lowerData[P_filterAttack]);
  }
}

void updatefilterDecay(boolean announce) {
  if (announce) {
    if (filterDecaystr < 1000) {
      showCurrentParameterPage("VCF Decay", String(int(filterDecaystr)) + " ms", FILTER_ENV);
    } else {
      showCurrentParameterPage("VCF Decay", String(filterDecaystr * 0.001) + " s", FILTER_ENV);
    }
  }
  if (upperSW) {
    midiCCOut(CCfilterDecay, upperData[P_filterDecay]);
    midiCCOut71(CCfilterDecay, upperData[P_filterDecay]);
  } else {
    midiCCOut(CCfilterDecay, lowerData[P_filterDecay]);
    midiCCOut71(CCfilterDecay, lowerData[P_filterDecay]);
  }
}

void updatefilterSustain(boolean announce) {
  if (announce) {
    showCurrentParameterPage("VCF Sustain", String(filterSustainstr), FILTER_ENV);
  }
  if (upperSW) {
    midiCCOut(CCfilterSustain, upperData[P_filterSustain]);
    midiCCOut71(CCfilterSustain, upperData[P_filterSustain]);
  } else {
    midiCCOut(CCfilterSustain, lowerData[P_filterSustain]);
    midiCCOut71(CCfilterSustain, lowerData[P_filterSustain]);
  }
}

void updatefilterRelease(boolean announce) {
  if (announce) {
    if (filterReleasestr < 1000) {
      showCurrentParameterPage("VCF Release", String(int(filterReleasestr)) + " ms", FILTER_ENV);
    } else {
      showCurrentParameterPage("VCF Release", String(filterReleasestr * 0.001) + " s", FILTER_ENV);
    }
  }
  if (upperSW) {
    midiCCOut(CCfilterRelease, upperData[P_filterRelease]);
    midiCCOut71(CCfilterRelease, upperData[P_filterRelease]);
  } else {
    midiCCOut(CCfilterRelease, lowerData[P_filterRelease]);
    midiCCOut71(CCfilterRelease, lowerData[P_filterRelease]);
  }
}

void updateampAttack(boolean announce) {
  if (announce) {
    if (ampAttackstr < 1000) {
      showCurrentParameterPage("VCA Attack", String(int(ampAttackstr)) + " ms", AMP_ENV);
    } else {
      showCurrentParameterPage("VCA Attack", String(ampAttackstr * 0.001) + " s", AMP_ENV);
    }
  }
  if (upperSW) {
    midiCCOut(CCampAttack, upperData[P_ampAttack]);
    midiCCOut71(CCampAttack, upperData[P_ampAttack]);
  } else {
    midiCCOut(CCampAttack, lowerData[P_ampAttack]);
    midiCCOut71(CCampAttack, lowerData[P_ampAttack]);
  }
}

void updateampDecay(boolean announce) {
  if (announce) {
    if (ampDecaystr < 1000) {
      showCurrentParameterPage("VCA Decay", String(int(ampDecaystr)) + " ms", AMP_ENV);
    } else {
      showCurrentParameterPage("VCA Decay", String(ampDecaystr * 0.001) + " s", AMP_ENV);
    }
  }
  if (upperSW) {
    midiCCOut(CCampDecay, upperData[P_ampDecay]);
    midiCCOut71(CCampDecay, upperData[P_ampDecay]);
  } else {
    midiCCOut(CCampDecay, lowerData[P_ampDecay]);
    midiCCOut71(CCampDecay, lowerData[P_ampDecay]);
  }
}

void updateampSustain(boolean announce) {
  if (announce) {
    showCurrentParameterPage("VCA Sustain", String(ampSustainstr), AMP_ENV);
  }
  if (upperSW) {
    midiCCOut(CCampSustain, upperData[P_ampSustain]);
    midiCCOut71(CCampSustain, upperData[P_ampSustain]);
  } else {
    midiCCOut(CCampSustain, lowerData[P_ampSustain]);
    midiCCOut71(CCampSustain, lowerData[P_ampSustain]);
  }
}

void updateampRelease(boolean announce) {
  if (announce) {
    if (ampReleasestr < 1000) {
      showCurrentParameterPage("VCA Release", String(int(ampReleasestr)) + " ms", AMP_ENV);
    } else {
      showCurrentParameterPage("VCA Release", String(ampReleasestr * 0.001) + " s", AMP_ENV);
    }
  }
  if (upperSW) {
    midiCCOut(CCampRelease, upperData[P_ampRelease]);
    midiCCOut71(CCampRelease, upperData[P_ampRelease]);
  } else {
    midiCCOut(CCampRelease, lowerData[P_ampRelease]);
    midiCCOut71(CCampRelease, lowerData[P_ampRelease]);
  }
}

void updatevolumeControl(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Volume", int(volumeControlstr));
  }
  if (upperSW) {
    midiCCOut(CCvolumeControl, upperData[P_volumeControl]);
    midiCCOut71(CCvolumeControl, upperData[P_volumeControl]);
  } else {
    midiCCOut(CCvolumeControl, lowerData[P_volumeControl]);
    midiCCOut71(CCvolumeControl, lowerData[P_volumeControl]);
  }
}

void updatePM_DCO2(boolean announce) {
  if (announce) {
    showCurrentParameterPage("PolyMod DCO2", int(pmDCO2str));
  }
  if (upperSW) {
    midiCCOut(CCPM_DCO2, upperData[P_pmDCO2]);
    midiCCOut71(CCPM_DCO2, upperData[P_pmDCO2]);
  } else {
    midiCCOut(CCPM_DCO2, lowerData[P_pmDCO2]);
    midiCCOut71(CCPM_DCO2, lowerData[P_pmDCO2]);
  }
}

void updatePM_FilterEnv(boolean announce) {
  if (announce) {
    showCurrentParameterPage("PolyMod Filter Env", int(pmFilterEnvstr));
  }
  if (upperSW) {
    midiCCOut(CCPM_FilterEnv, upperData[P_pmFilterEnv]);
    midiCCOut71(CCPM_FilterEnv, upperData[P_pmFilterEnv]);
  } else {
    midiCCOut(CCPM_FilterEnv, lowerData[P_pmFilterEnv]);
    midiCCOut71(CCPM_FilterEnv, lowerData[P_pmFilterEnv]);
  }
}

// ////////////////////////////////////////////////////////////////

void updatechordHoldSW(boolean announce) {
  if (upperSW) {
    if (chordHoldU == 0) {
      if (announce) {
        showCurrentParameterPage("Chord Hold", "Off");
      }
      midiCCOut(CCchordHoldSW, 0);
      midiCCOut72(CCchordHoldSW, 0);
    } else {
      if (announce) {
        showCurrentParameterPage("Chord Hold", "On");
      }
      midiCCOut(CCchordHoldSW, 127);
      midiCCOut72(CCchordHoldSW, 127);
    }
  } else {
    if (chordHoldL == 0) {
      if (announce) {
        showCurrentParameterPage("Chord Hold", "Off");
      }
      midiCCOut(CCchordHoldSW, 0);
      midiCCOut72(CCchordHoldSW, 0);
    } else {
      if (announce) {
        showCurrentParameterPage("Chord Hold", "On");
      }
      midiCCOut(CCchordHoldSW, 127);
      midiCCOut72(CCchordHoldSW, 127);
    }
  }
}

void updateplayMode(boolean announce) {
  if (playMode == 0) {
    if (announce) {
      showCurrentParameterPage("Key Mode", "Whole");
    }
    midiCCOut72(CCplayMode, 0);
    midiCCOut(CCplayMode, 0);
    srp.writePin(UPPER_RELAY_2, HIGH);
    srp.writePin(UPPER_RELAY_3, HIGH);
    wholemode = true;
    dualmode = false;
    splitmode = false;
    lowerSW = true;
    upperSW = false;
    updatelowerSW(0);
    lowerParamsToDisplay();
    setAllButtons();

  } else if (playMode == 1) {
    if (announce) {
      showCurrentParameterPage("Key Mode", "Dual");
    }
    midiCCOut72(CCplayMode, 1);
    midiCCOut(CCplayMode, 1);
    srp.writePin(UPPER_RELAY_2, LOW);
    srp.writePin(UPPER_RELAY_3, LOW);
    wholemode = false;
    dualmode = true;
    splitmode = false;
  } else if (playMode == 2) {
    if (announce) {
      showCurrentParameterPage("Key Mode", "Split");
    }
    midiCCOut72(CCplayMode, 2);
    midiCCOut(CCplayMode, 2);
    srp.writePin(UPPER_RELAY_2, LOW);
    srp.writePin(UPPER_RELAY_3, LOW);
    wholemode = false;
    dualmode = false;
    splitmode = true;
  }
}

void updatekeyboardMode(boolean announce) {
  if (upperSW) {
    if (dualmode) {
      lowerData[P_keyboardMode] = upperData[P_keyboardMode];
    }
    if (upperData[P_keyboardMode] == 0) {
      if (announce) {
        showCurrentParameterPage("Keyboard Mode", "Poly 1");
      }
      midiCCOut72(CCkeyboardMode, 0);
      midiCCOut(CCkeyboardMode, 0);
    } else if (upperData[P_keyboardMode] == 1) {
      if (announce) {
        showCurrentParameterPage("Keyboard Mode", "Poly 2");
      }
      midiCCOut72(CCkeyboardMode, 1);
      midiCCOut(CCkeyboardMode, 1);
    } else if (upperData[P_keyboardMode] == 2) {
      if (announce) {
        showCurrentParameterPage("Keyboard Mode", "Mono");
      }
      midiCCOut72(CCkeyboardMode, 2);
      midiCCOut(CCkeyboardMode, 2);
    } else if (upperData[P_keyboardMode] == 3) {
      if (announce) {
        showCurrentParameterPage("Keyboard Mode", "Unison");
      }
      midiCCOut72(CCkeyboardMode, 3);
      midiCCOut(CCkeyboardMode, 3);
    }
  } else {
    if (dualmode) {
      upperData[P_keyboardMode] = lowerData[P_keyboardMode];
    }
    if (lowerData[P_keyboardMode] == 0) {
      if (announce) {
        showCurrentParameterPage("Keyboard Mode", "Poly 1");
      }
      midiCCOut72(CCkeyboardMode, 0);
      midiCCOut(CCkeyboardMode, 0);
    } else if (lowerData[P_keyboardMode] == 1) {
      if (announce) {
        showCurrentParameterPage("Keyboard Mode", "Poly 2");
      }
      midiCCOut72(CCkeyboardMode, 1);
      midiCCOut(CCkeyboardMode, 1);
    } else if (lowerData[P_keyboardMode] == 2) {
      if (announce) {
        showCurrentParameterPage("Keyboard Mode", "Mono");
      }
      midiCCOut72(CCkeyboardMode, 2);
      midiCCOut(CCkeyboardMode, 2);
    } else if (lowerData[P_keyboardMode] == 3) {
      if (announce) {
        showCurrentParameterPage("Keyboard Mode", "Unison");
      }
      midiCCOut72(CCkeyboardMode, 3);
      midiCCOut(CCkeyboardMode, 3);
    }
  }
}

void updateeffectNumSW(boolean announce) {
  if (upperSW) {
    if (upperData[P_effectNum] == 0) {
      if (announce) {
        showCurrentParameterPage("Effect", "1");
      }
      srp.writePin(EFFECT_0_UPPER, LOW);
      srp.writePin(EFFECT_1_UPPER, LOW);
      srp.writePin(EFFECT_2_UPPER, LOW);
      midiCCOut72(CCeffectNumSW, 0);
      midiCCOut(CCeffectNumSW, 0);

    } else if (upperData[P_effectNum] == 1) {
      if (announce) {
        showCurrentParameterPage("Effect", "2");
      }
      srp.writePin(EFFECT_0_UPPER, HIGH);
      srp.writePin(EFFECT_1_UPPER, LOW);
      srp.writePin(EFFECT_2_UPPER, LOW);
      midiCCOut72(CCeffectNumSW, 1);
      midiCCOut(CCeffectNumSW, 1);

    } else if (upperData[P_effectNum] == 2) {
      if (announce) {
        showCurrentParameterPage("Effect", "3");
      }
      srp.writePin(EFFECT_0_UPPER, LOW);
      srp.writePin(EFFECT_1_UPPER, HIGH);
      srp.writePin(EFFECT_2_UPPER, LOW);
      midiCCOut72(CCeffectNumSW, 2);
      midiCCOut(CCeffectNumSW, 2);

    } else if (upperData[P_effectNum] == 3) {
      if (announce) {
        showCurrentParameterPage("Effect", "4");
      }
      srp.writePin(EFFECT_0_UPPER, HIGH);
      srp.writePin(EFFECT_1_UPPER, HIGH);
      srp.writePin(EFFECT_2_UPPER, LOW);
      midiCCOut72(CCeffectNumSW, 3);
      midiCCOut(CCeffectNumSW, 3);

    } else if (upperData[P_effectNum] == 4) {
      if (announce) {
        showCurrentParameterPage("Effect", "5");
      }
      srp.writePin(EFFECT_0_UPPER, LOW);
      srp.writePin(EFFECT_1_UPPER, LOW);
      srp.writePin(EFFECT_2_UPPER, HIGH);
      midiCCOut72(CCeffectNumSW, 4);
      midiCCOut(CCeffectNumSW, 4);

    } else if (upperData[P_effectNum] == 5) {
      if (announce) {
        showCurrentParameterPage("Effect", "6");
      }
      srp.writePin(EFFECT_0_UPPER, HIGH);
      srp.writePin(EFFECT_1_UPPER, LOW);
      srp.writePin(EFFECT_2_UPPER, HIGH);
      midiCCOut72(CCeffectNumSW, 5);
      midiCCOut(CCeffectNumSW, 5);

    } else if (upperData[P_effectNum] == 6) {
      if (announce) {
        showCurrentParameterPage("Effect", "7");
      }
      srp.writePin(EFFECT_0_UPPER, LOW);
      srp.writePin(EFFECT_1_UPPER, HIGH);
      srp.writePin(EFFECT_2_UPPER, HIGH);
      midiCCOut72(CCeffectNumSW, 6);
      midiCCOut(CCeffectNumSW, 6);

    } else if (upperData[P_effectNum] == 7) {
      if (announce) {
        showCurrentParameterPage("Effect", "8");
      }
      srp.writePin(EFFECT_0_UPPER, HIGH);
      srp.writePin(EFFECT_1_UPPER, HIGH);
      srp.writePin(EFFECT_2_UPPER, HIGH);
      midiCCOut72(CCeffectNumSW, 7);
      midiCCOut(CCeffectNumSW, 7);
    }

  } else {
    if (lowerData[P_effectNum] == 0) {
      if (announce) {
        showCurrentParameterPage("Effect", "1");
      }
      srp.writePin(EFFECT_0_LOWER, LOW);
      srp.writePin(EFFECT_1_LOWER, LOW);
      srp.writePin(EFFECT_2_LOWER, LOW);
      if (wholemode) {
        srp.writePin(EFFECT_0_UPPER, LOW);
        srp.writePin(EFFECT_1_UPPER, LOW);
        srp.writePin(EFFECT_2_UPPER, LOW);
      }
      midiCCOut72(CCeffectNumSW, 0);
      midiCCOut(CCeffectNumSW, 0);

    } else if (lowerData[P_effectNum] == 1) {
      if (announce) {
        showCurrentParameterPage("Effect", "2");
      }
      srp.writePin(EFFECT_0_LOWER, HIGH);
      srp.writePin(EFFECT_1_LOWER, LOW);
      srp.writePin(EFFECT_2_LOWER, LOW);
      if (wholemode) {
        srp.writePin(EFFECT_0_UPPER, HIGH);
        srp.writePin(EFFECT_1_UPPER, LOW);
        srp.writePin(EFFECT_2_UPPER, LOW);
      }
      midiCCOut72(CCeffectNumSW, 1);
      midiCCOut(CCeffectNumSW, 1);

    } else if (lowerData[P_effectNum] == 2) {
      if (announce) {
        showCurrentParameterPage("Effect", "3");
      }
      srp.writePin(EFFECT_0_LOWER, LOW);
      srp.writePin(EFFECT_1_LOWER, HIGH);
      srp.writePin(EFFECT_2_LOWER, LOW);
      if (wholemode) {
        srp.writePin(EFFECT_0_UPPER, LOW);
        srp.writePin(EFFECT_1_UPPER, HIGH);
        srp.writePin(EFFECT_2_UPPER, LOW);
      }
      midiCCOut72(CCeffectNumSW, 2);
      midiCCOut(CCeffectNumSW, 2);

    } else if (lowerData[P_effectNum] == 3) {
      if (announce) {
        showCurrentParameterPage("Effect", "4");
      }
      srp.writePin(EFFECT_0_LOWER, HIGH);
      srp.writePin(EFFECT_1_LOWER, HIGH);
      srp.writePin(EFFECT_2_LOWER, LOW);
      if (wholemode) {
        srp.writePin(EFFECT_0_UPPER, HIGH);
        srp.writePin(EFFECT_1_UPPER, HIGH);
        srp.writePin(EFFECT_2_UPPER, LOW);
      }
      midiCCOut72(CCeffectNumSW, 3);
      midiCCOut(CCeffectNumSW, 3);

    } else if (lowerData[P_effectNum] == 4) {
      if (announce) {
        showCurrentParameterPage("Effect", "5");
      }
      srp.writePin(EFFECT_0_LOWER, LOW);
      srp.writePin(EFFECT_1_LOWER, LOW);
      srp.writePin(EFFECT_2_LOWER, HIGH);
      if (wholemode) {
        srp.writePin(EFFECT_0_UPPER, LOW);
        srp.writePin(EFFECT_1_UPPER, LOW);
        srp.writePin(EFFECT_2_UPPER, HIGH);
      }
      midiCCOut72(CCeffectNumSW, 4);
      midiCCOut(CCeffectNumSW, 4);

    } else if (lowerData[P_effectNum] == 5) {
      if (announce) {
        showCurrentParameterPage("Effect", "6");
      }
      srp.writePin(EFFECT_0_LOWER, HIGH);
      srp.writePin(EFFECT_1_LOWER, LOW);
      srp.writePin(EFFECT_2_LOWER, HIGH);
      if (wholemode) {
        srp.writePin(EFFECT_0_UPPER, HIGH);
        srp.writePin(EFFECT_1_UPPER, LOW);
        srp.writePin(EFFECT_2_UPPER, HIGH);
      }
      midiCCOut72(CCeffectNumSW, 5);
      midiCCOut(CCeffectNumSW, 5);

    } else if (lowerData[P_effectNum] == 6) {
      if (announce) {
        showCurrentParameterPage("Effect", "7");
      }
      srp.writePin(EFFECT_0_LOWER, LOW);
      srp.writePin(EFFECT_1_LOWER, HIGH);
      srp.writePin(EFFECT_2_LOWER, HIGH);
      if (wholemode) {
        srp.writePin(EFFECT_0_UPPER, LOW);
        srp.writePin(EFFECT_1_UPPER, HIGH);
        srp.writePin(EFFECT_2_UPPER, HIGH);
      }
      midiCCOut72(CCeffectNumSW, 6);
      midiCCOut(CCeffectNumSW, 6);

    } else if (lowerData[P_effectNum] == 7) {
      if (announce) {
        showCurrentParameterPage("Effect", "8");
      }
      srp.writePin(EFFECT_0_LOWER, HIGH);
      srp.writePin(EFFECT_1_LOWER, HIGH);
      srp.writePin(EFFECT_2_LOWER, HIGH);
      if (wholemode) {
        srp.writePin(EFFECT_0_UPPER, HIGH);
        srp.writePin(EFFECT_1_UPPER, HIGH);
        srp.writePin(EFFECT_2_UPPER, HIGH);
      }
      midiCCOut72(CCeffectNumSW, 7);
      midiCCOut(CCeffectNumSW, 7);
    }
  }
}

void updateeffectBankSW(boolean announce) {
  int bank = upperSW ? upperData[P_effectBank] : lowerData[P_effectBank];

  if (announce) {
    showCurrentParameterPage("Effects", "Bank " + String(bank + 1));
  }

  if (upperSW) {
    // Step 1: Enter external mode
    srp.writePin(EFFECT_INTERNAL_UPPER, HIGH);

    // Step 2: Reset all CS lines
    srp.writePin(EFFECT_BANK_1_UPPER, HIGH);
    srp.writePin(EFFECT_BANK_2_UPPER, HIGH);
    srp.writePin(EFFECT_BANK_3_UPPER, HIGH);
    srp.update();

    if (bank == 0) {
      // Internal ROM selected
      srp.writePin(EFFECT_INTERNAL_UPPER, LOW);
      srp.update();
    } else {
      // Select only the chosen EEPROM
      if (bank == 1) srp.writePin(EFFECT_BANK_1_UPPER, LOW);
      else if (bank == 2) srp.writePin(EFFECT_BANK_2_UPPER, LOW);
      else if (bank == 3) srp.writePin(EFFECT_BANK_3_UPPER, LOW);

      srp.update();  // or srp.latch(), or whatever your library uses
      srp.writePin(EFFECT_INTERNAL_UPPER, LOW);
      srp.update();
      srp.writePin(EFFECT_INTERNAL_UPPER, HIGH);
      srp.update();
    }

  } else {
    // Step 1: Enter external mode
    srp.writePin(EFFECT_INTERNAL_LOWER, HIGH);

    // Step 2: Reset all CS lines
    srp.writePin(EFFECT_BANK_1_LOWER, HIGH);
    srp.writePin(EFFECT_BANK_2_LOWER, HIGH);
    srp.writePin(EFFECT_BANK_3_LOWER, HIGH);
    srp.update();

    if (bank == 0) {
      srp.writePin(EFFECT_INTERNAL_LOWER, LOW);
      srp.update();
      if (wholemode) {
        srp.writePin(EFFECT_INTERNAL_UPPER, LOW);
        srp.writePin(EFFECT_BANK_1_UPPER, HIGH);
        srp.writePin(EFFECT_BANK_2_UPPER, HIGH);
        srp.writePin(EFFECT_BANK_3_UPPER, HIGH);
        srp.update();
      }

    } else {
      if (bank == 1) srp.writePin(EFFECT_BANK_1_LOWER, LOW);
      else if (bank == 2) srp.writePin(EFFECT_BANK_2_LOWER, LOW);
      else if (bank == 3) srp.writePin(EFFECT_BANK_3_LOWER, LOW);

      srp.update();
      srp.writePin(EFFECT_INTERNAL_LOWER, LOW);
      srp.update();
      srp.writePin(EFFECT_INTERNAL_LOWER, HIGH);
      srp.update();

      if (wholemode) {
        srp.writePin(EFFECT_INTERNAL_UPPER, HIGH);
        srp.writePin(EFFECT_BANK_1_UPPER, HIGH);
        srp.writePin(EFFECT_BANK_2_UPPER, HIGH);
        srp.writePin(EFFECT_BANK_3_UPPER, HIGH);
        srp.update();

        if (bank == 1) srp.writePin(EFFECT_BANK_1_UPPER, LOW);
        else if (bank == 2) srp.writePin(EFFECT_BANK_2_UPPER, LOW);
        else if (bank == 3) srp.writePin(EFFECT_BANK_3_UPPER, LOW);

        srp.update();
        srp.writePin(EFFECT_INTERNAL_UPPER, LOW);
        srp.update();
        srp.writePin(EFFECT_INTERNAL_UPPER, HIGH);
        srp.update();
      }
    }
  }

  // Send MIDI
  midiCCOut72(CCeffectBankSW, bank);
  midiCCOut(CCeffectBankSW, bank);
}

void updatelfoMultiplier(boolean announce) {
  if (upperSW) {
    if (upperData[P_lfoMultiplier] == 0) {
      if (announce) {
        showCurrentParameterPage("LFO Multiplier", "x0.5");
      }
      srp.writePin(LFO_MULTI_BIT0_UPPER, LOW);
      srp.writePin(LFO_MULTI_BIT1_UPPER, LOW);
      srp.writePin(LFO_MULTI_BIT2_UPPER, LOW);
      midiCCOut72(CClfoMult, 0);
      midiCCOut(CClfoMult, 0);
    } else if (upperData[P_lfoMultiplier] == 1) {
      if (announce) {
        showCurrentParameterPage("LFO Multiplier", "x1.0");
      }
      srp.writePin(LFO_MULTI_BIT0_UPPER, HIGH);
      srp.writePin(LFO_MULTI_BIT1_UPPER, LOW);
      srp.writePin(LFO_MULTI_BIT2_UPPER, LOW);
      midiCCOut72(CClfoMult, 1);
      midiCCOut(CClfoMult, 1);
    } else if (upperData[P_lfoMultiplier] == 2) {
      if (announce) {
        showCurrentParameterPage("LFO Multiplier", "x1.5");
      }
      srp.writePin(LFO_MULTI_BIT0_UPPER, LOW);
      srp.writePin(LFO_MULTI_BIT1_UPPER, HIGH);
      srp.writePin(LFO_MULTI_BIT2_UPPER, LOW);
      midiCCOut72(CClfoMult, 2);
      midiCCOut(CClfoMult, 2);
    } else if (upperData[P_lfoMultiplier] == 3) {
      if (announce) {
        showCurrentParameterPage("LFO Multiplier", "x2.0");
      }
      srp.writePin(LFO_MULTI_BIT0_UPPER, HIGH);
      srp.writePin(LFO_MULTI_BIT1_UPPER, HIGH);
      srp.writePin(LFO_MULTI_BIT2_UPPER, LOW);
      midiCCOut72(CClfoMult, 3);
      midiCCOut(CClfoMult, 3);
    } else if (upperData[P_lfoMultiplier] == 4) {
      if (announce) {
        showCurrentParameterPage("LFO Multiplier", "x2.5");
      }
      srp.writePin(LFO_MULTI_BIT0_UPPER, LOW);
      srp.writePin(LFO_MULTI_BIT1_UPPER, LOW);
      srp.writePin(LFO_MULTI_BIT2_UPPER, HIGH);
      midiCCOut72(CClfoMult, 4);
      midiCCOut(CClfoMult, 4);
    }
  } else {
    if (lowerData[P_lfoMultiplier] == 0) {
      if (announce) {
        showCurrentParameterPage("LFO Multiplier", "x0.5");
      }
      srp.writePin(LFO_MULTI_BIT0_LOWER, LOW);
      srp.writePin(LFO_MULTI_BIT1_LOWER, LOW);
      srp.writePin(LFO_MULTI_BIT2_LOWER, LOW);
      if (wholemode) {
        srp.writePin(LFO_MULTI_BIT0_UPPER, LOW);
        srp.writePin(LFO_MULTI_BIT1_UPPER, LOW);
        srp.writePin(LFO_MULTI_BIT2_UPPER, LOW);
      }
      midiCCOut72(CClfoMult, 0);
      midiCCOut(CClfoMult, 0);
    } else if (lowerData[P_lfoMultiplier] == 1) {
      if (announce) {
        showCurrentParameterPage("LFO Multiplier", "x1.0");
      }
      srp.writePin(LFO_MULTI_BIT0_LOWER, HIGH);
      srp.writePin(LFO_MULTI_BIT1_LOWER, LOW);
      srp.writePin(LFO_MULTI_BIT2_LOWER, LOW);
      if (wholemode) {
        srp.writePin(LFO_MULTI_BIT0_UPPER, HIGH);
        srp.writePin(LFO_MULTI_BIT1_UPPER, LOW);
        srp.writePin(LFO_MULTI_BIT2_UPPER, LOW);
      }
      midiCCOut72(CClfoMult, 1);
      midiCCOut(CClfoMult, 1);
    } else if (lowerData[P_lfoMultiplier] == 2) {
      if (announce) {
        showCurrentParameterPage("LFO Multiplier", "x1.5");
      }
      srp.writePin(LFO_MULTI_BIT0_LOWER, LOW);
      srp.writePin(LFO_MULTI_BIT1_LOWER, HIGH);
      srp.writePin(LFO_MULTI_BIT2_LOWER, LOW);
      if (wholemode) {
        srp.writePin(LFO_MULTI_BIT0_UPPER, LOW);
        srp.writePin(LFO_MULTI_BIT1_UPPER, HIGH);
        srp.writePin(LFO_MULTI_BIT2_UPPER, LOW);
      }
      midiCCOut72(CClfoMult, 2);
      midiCCOut(CClfoMult, 2);
    } else if (lowerData[P_lfoMultiplier] == 3) {
      if (announce) {
        showCurrentParameterPage("LFO Multiplier", "x2.0");
      }
      srp.writePin(LFO_MULTI_BIT0_LOWER, HIGH);
      srp.writePin(LFO_MULTI_BIT1_LOWER, HIGH);
      srp.writePin(LFO_MULTI_BIT2_LOWER, LOW);
      if (wholemode) {
        srp.writePin(LFO_MULTI_BIT0_UPPER, HIGH);
        srp.writePin(LFO_MULTI_BIT1_UPPER, HIGH);
        srp.writePin(LFO_MULTI_BIT2_UPPER, LOW);
      }
      midiCCOut72(CClfoMult, 3);
      midiCCOut(CClfoMult, 3);
    } else if (lowerData[P_lfoMultiplier] == 4) {
      if (announce) {
        showCurrentParameterPage("LFO Multiplier", "x2.5");
      }
      srp.writePin(LFO_MULTI_BIT0_LOWER, LOW);
      srp.writePin(LFO_MULTI_BIT1_LOWER, LOW);
      srp.writePin(LFO_MULTI_BIT2_LOWER, HIGH);
      if (wholemode) {
        srp.writePin(LFO_MULTI_BIT0_UPPER, LOW);
        srp.writePin(LFO_MULTI_BIT1_UPPER, LOW);
        srp.writePin(LFO_MULTI_BIT2_UPPER, HIGH);
      }
      midiCCOut72(CClfoMult, 4);
      midiCCOut(CClfoMult, 4);
    }
  }
}

void updateglideSW(boolean announce) {
  if (upperSW) {
    if (upperData[P_glideSW] == 0) {
      if (announce) {
        showCurrentParameterPage("Glide", "Off");
      }
      midiCCOut62(CCglideSW, 0);
      midiCCOut72(CCglideSW, 0);
    } else {
      if (announce) {
        showCurrentParameterPage("Glide", "On");
      }
      midiCCOut62(CCglideTime, upperData[P_glideTime]);
      midiCCOut62(CCglideSW, 127);
      midiCCOut71(CCglideTime, upperData[P_glideTime]);
      midiCCOut72(CCglideSW, 1);
    }
  } else {
    if (lowerData[P_glideSW] == 0) {
      if (announce) {
        showCurrentParameterPage("Glide", "Off");
      }
      midiCCOut61(CCglideSW, 0);
      midiCCOut72(CCglideSW, 0);
      if (wholemode) {
        midiCCOut62(CCglideSW, 0);
      }
    } else {
      if (announce) {
        showCurrentParameterPage("Glide", "On");
      }
      midiCCOut61(CCglideTime, lowerData[P_glideTime]);
      midiCCOut61(CCglideSW, 127);

      midiCCOut71(CCglideTime, lowerData[P_glideTime]);
      midiCCOut72(CCglideSW, 1);
      if (wholemode) {
        midiCCOut62(CCglideTime, upperData[P_glideTime]);
        midiCCOut62(CCglideSW, 127);
      }
    }
  }
}

void updatefilterPoleSwitch(boolean announce) {
  if (upperSW) {
    if (upperData[P_filterPoleSW] == 1) {
      if (announce) {
        //showCurrentParameterPage("VCF Pole", "On");
        updateFilterType(1);
      }
      midiCCOut(CCfilterPoleSW, 127);
      midiCCOut72(CCfilterPoleSW, 127);
      srp.writePin(FILTER_POLE_UPPER, HIGH);
    } else {
      if (announce) {
        //showCurrentParameterPage("VCF Pole", "Off");
        updateFilterType(1);
      }
      midiCCOut(CCfilterPoleSW, 0);
      midiCCOut72(CCfilterPoleSW, 0);
      srp.writePin(FILTER_POLE_UPPER, LOW);
    }
  } else {
    if (lowerData[P_filterPoleSW] == 1) {
      if (announce) {
        //showCurrentParameterPage("VCF Pole", "On");
        updateFilterType(1);
      }
      midiCCOut(CCfilterPoleSW, 127);
      midiCCOut72(CCfilterPoleSW, 127);
      srp.writePin(FILTER_POLE_LOWER, HIGH);
      if (wholemode) {
        srp.writePin(FILTER_POLE_UPPER, HIGH);
      }
    } else {
      if (announce) {
        //showCurrentParameterPage("VCF Pole", "Off");
        updateFilterType(1);
      }
      midiCCOut(CCfilterPoleSW, 0);
      midiCCOut72(CCfilterPoleSW, 0);
      srp.writePin(FILTER_POLE_LOWER, LOW);
      if (wholemode) {
        srp.writePin(FILTER_POLE_UPPER, LOW);
      }
    }
  }
}

void updatefilterLoop(boolean announce) {
  if (upperSW) {
    switch (upperData[P_filterLoop]) {
      case 0:
        if (announce) {
          showCurrentParameterPage("VCF Key Loop", "Off");
        }
        midiCCOut72(CCFilterLoop, 0);
        midiCCOut(CCFilterLoop, 0);
        srp.writePin(FILTER_MODE_BIT0_UPPER, LOW);
        srp.writePin(FILTER_MODE_BIT1_UPPER, LOW);
        break;

      case 1:
        if (announce) {
          showCurrentParameterPage("VCF LFO Loop", "Gated");
        }
        midiCCOut72(CCFilterLoop, 1);
        midiCCOut(CCFilterLoop, 63);
        srp.writePin(FILTER_MODE_BIT0_UPPER, HIGH);
        srp.writePin(FILTER_MODE_BIT1_UPPER, LOW);
        break;

      case 2:
        if (announce) {
          showCurrentParameterPage("VCF Looping", "LFO");
        }
        midiCCOut72(CCFilterLoop, 2);
        midiCCOut(CCFilterLoop, 127);
        srp.writePin(FILTER_MODE_BIT0_UPPER, HIGH);
        srp.writePin(FILTER_MODE_BIT1_UPPER, HIGH);
        break;
    }
  } else {
    switch (lowerData[P_filterLoop]) {
      case 0:
        if (announce) {
          showCurrentParameterPage("VCF Key Loop", "Off");
        }
        midiCCOut72(CCFilterLoop, 0);
        midiCCOut(CCFilterLoop, 0);
        srp.writePin(FILTER_MODE_BIT0_LOWER, LOW);
        srp.writePin(FILTER_MODE_BIT1_LOWER, LOW);
        if (wholemode) {
          srp.writePin(FILTER_MODE_BIT0_UPPER, LOW);
          srp.writePin(FILTER_MODE_BIT1_UPPER, LOW);
        }
        break;

      case 1:
        if (announce) {
          showCurrentParameterPage("VCF LFO Loop", "Gated");
        }
        midiCCOut72(CCFilterLoop, 1);
        midiCCOut(CCFilterLoop, 63);
        srp.writePin(FILTER_MODE_BIT0_LOWER, HIGH);
        srp.writePin(FILTER_MODE_BIT1_LOWER, LOW);
        if (wholemode) {
          srp.writePin(FILTER_MODE_BIT0_UPPER, HIGH);
          srp.writePin(FILTER_MODE_BIT1_UPPER, LOW);
        }
        break;

      case 2:
        if (announce) {
          showCurrentParameterPage("VCF Looping", "LFO");
        }
        midiCCOut72(CCFilterLoop, 2);
        midiCCOut(CCFilterLoop, 127);
        srp.writePin(FILTER_MODE_BIT0_LOWER, HIGH);
        srp.writePin(FILTER_MODE_BIT1_LOWER, HIGH);
        if (wholemode) {
          srp.writePin(FILTER_MODE_BIT0_UPPER, HIGH);
          srp.writePin(FILTER_MODE_BIT1_UPPER, HIGH);
        }
        break;
    }
  }
}

void updatefilterEGinv(boolean announce) {
  if (upperSW) {
    if (upperData[P_filterEGinv] == 0) {
      if (announce) {
        showCurrentParameterPage("Filter Env", "Positive");
      }
      midiCCOut(CCfilterEGinv, 0);
      midiCCOut72(CCfilterEGinv, 0);
      srp.writePin(FILTER_EG_INV_UPPER, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Filter Env", "Negative");
      }
      midiCCOut(CCfilterEGinv, 127);
      midiCCOut72(CCfilterEGinv, 127);
      // sr.set(FILTERINV_LED, HIGH);  // LED on
      srp.writePin(FILTER_EG_INV_UPPER, HIGH);
    }
  } else {
    if (lowerData[P_filterEGinv] == 0) {
      if (announce) {
        showCurrentParameterPage("Filter Env", "Positive");
      }
      midiCCOut(CCfilterEGinv, 0);
      midiCCOut72(CCfilterEGinv, 0);
      srp.writePin(FILTER_EG_INV_LOWER, LOW);
      if (wholemode) {
        srp.writePin(FILTER_EG_INV_UPPER, LOW);
      }
    } else {
      if (announce) {
        showCurrentParameterPage("Filter Env", "Negative");
      }
      midiCCOut(CCfilterEGinv, 127);
      midiCCOut72(CCfilterEGinv, 127);
      srp.writePin(FILTER_EG_INV_LOWER, HIGH);
      if (wholemode) {
        srp.writePin(FILTER_EG_INV_UPPER, HIGH);
      }
    }
  }
}

void updatepmDestDCO1(boolean announce) {
  if (upperSW) {
    if (!upperData[P_pmDestDCO1]) {
      if (announce) {
        showCurrentParameterPage("PolyMod DCO1", "Off");
      }
      midiCCOut(CCpmDestDCO1SW, 0);
      midiCCOut72(CCpmDestDCO1SW, 0);
      srp.writePin(POLYMOD_DEST_DCO1_UPPER, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("PolyMod DCO1", "On");
      }
      midiCCOut(CCpmDestDCO1SW, 127);
      midiCCOut72(CCpmDestDCO1SW, 1);
      srp.writePin(POLYMOD_DEST_DCO1_UPPER, HIGH);
    }
  } else {
    if (!lowerData[P_pmDestDCO1]) {
      if (announce) {
        showCurrentParameterPage("PolyMod DCO1", "Off");
      }
      midiCCOut(CCpmDestDCO1SW, 0);
      midiCCOut72(CCpmDestDCO1SW, 0);
      srp.writePin(POLYMOD_DEST_DCO1_LOWER, LOW);
      if (wholemode) {
        srp.writePin(POLYMOD_DEST_DCO1_UPPER, LOW);
      }
    } else {
      if (announce) {
        showCurrentParameterPage("PolyMod DCO1", "On");
      }
      midiCCOut(CCpmDestDCO1SW, 127);
      midiCCOut72(CCpmDestDCO1SW, 1);
      srp.writePin(POLYMOD_DEST_DCO1_LOWER, HIGH);
      if (wholemode) {
        srp.writePin(POLYMOD_DEST_DCO1_UPPER, HIGH);
      }
    }
  }
}

void updatepmDestFilter(boolean announce) {
  if (upperSW) {
    if (!upperData[P_pmDestFilter]) {
      if (announce) {
        showCurrentParameterPage("PolyMod Filter", "Off");
      }
      midiCCOut(CCpmDestFilterSW, 0);
      midiCCOut72(CCpmDestFilterSW, 0);
      srp.writePin(POLYMOD_DEST_FILTER_UPPER, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("PolyMod Filter", "On");
      }
      midiCCOut(CCpmDestFilterSW, 127);
      midiCCOut72(CCpmDestFilterSW, 1);
      srp.writePin(POLYMOD_DEST_FILTER_UPPER, HIGH);
    }
  } else {
    if (!lowerData[P_pmDestFilter]) {
      if (announce) {
        showCurrentParameterPage("PolyMod Filter", "Off");
      }
      midiCCOut(CCpmDestFilterSW, 0);
      midiCCOut72(CCpmDestFilterSW, 0);
      srp.writePin(POLYMOD_DEST_FILTER_LOWER, LOW);
      if (wholemode) {
        srp.writePin(POLYMOD_DEST_FILTER_UPPER, LOW);
      }
    } else {
      if (announce) {
        showCurrentParameterPage("PolyMod Filter", "On");
      }
      midiCCOut(CCpmDestFilterSW, 127);
      midiCCOut72(CCpmDestFilterSW, 1);
      srp.writePin(POLYMOD_DEST_FILTER_LOWER, HIGH);
      if (wholemode) {
        srp.writePin(POLYMOD_DEST_FILTER_UPPER, HIGH);
      }
    }
  }
}

void updatekeyTrackSW(boolean announce) {
  if (upperSW) {
    if (!upperData[P_keytrackSW]) {
      if (announce) {
        showCurrentParameterPage("Keytrack", "Off");
      }
      midiCCOut62(WSkeytrackSW, 0);
      midiCCOut(CCkeyTrackSW, 0);
      midiCCOut72(CCkeyTrackSW, 0);
    } else {
      if (announce) {
        showCurrentParameterPage("Keytrack", "On");
      }
      midiCCOut62(WSkeytrackSW, 127);
      midiCCOut(CCkeyTrackSW, 127);
      midiCCOut72(CCkeyTrackSW, 1);
    }
  } else {
    if (!lowerData[P_keytrackSW]) {
      if (announce) {
        showCurrentParameterPage("Keytrack", "Off");
      }
      midiCCOut61(WSkeytrackSW, 0);
      midiCCOut(CCkeyTrackSW, 0);
      midiCCOut72(CCkeyTrackSW, 0);
      if (wholemode) {
        midiCCOut62(WSkeytrackSW, 0);
      }
    } else {
      if (announce) {
        showCurrentParameterPage("Keytrack", "On");
      }
      midiCCOut61(WSkeytrackSW, 127);
      midiCCOut(CCkeyTrackSW, 127);
      midiCCOut72(CCkeyTrackSW, 1);
      if (wholemode) {
        midiCCOut62(WSkeytrackSW, 127);
      }
    }
  }
}

void updatesyncSW(boolean announce) {
  if (upperSW) {
    if (!upperData[P_sync]) {
      if (announce) {
        showCurrentParameterPage("Sync", "Off");
      }
      midiCCOut62(WSsyncW, 0);
      midiCCOut(CCsyncSW, 0);
      midiCCOut72(CCsyncSW, 0);
      srp.writePin(SYNC_UPPER, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Sync", "On");
      }
      midiCCOut62(WSsyncW, 127);
      midiCCOut(CCsyncSW, 127);
      midiCCOut72(CCsyncSW, 1);
      srp.writePin(SYNC_UPPER, HIGH);
    }
  } else {
    if (!lowerData[P_sync]) {
      if (announce) {
        showCurrentParameterPage("Sync", "Off");
      }
      midiCCOut61(WSsyncW, 0);
      midiCCOut(CCsyncSW, 0);
      midiCCOut72(CCsyncSW, 0);
      srp.writePin(SYNC_LOWER, LOW);
      if (wholemode) {
        midiCCOut62(WSsyncW, 0);
        srp.writePin(SYNC_UPPER, LOW);
      }
    } else {
      if (announce) {
        showCurrentParameterPage("Sync", "On");
      }
      midiCCOut61(WSsyncW, 127);
      midiCCOut(CCsyncSW, 127);
      midiCCOut72(CCsyncSW, 1);
      srp.writePin(SYNC_LOWER, HIGH);
      if (wholemode) {
        midiCCOut62(WSsyncW, 127);
        srp.writePin(SYNC_UPPER, HIGH);
      }
    }
  }
}

void updatefootSwitch() {

  if (upperSW) {
    if (upperData[P_effectPot3] < 2047) {
      upperslowpot3 = upperData[P_effectPot3];
      upperfast = true;
      upperslow = false;
    }
    if (upperData[P_effectPot3] > 2047) {
      upperfastpot3 = upperData[P_effectPot3];
      upperfast = false;
      upperslow = true;
    }
  } else {
    if (lowerData[P_effectPot3] < 2047) {
      lowerslowpot3 = lowerData[P_effectPot3];
      lowerfast = true;
      lowerslow = false;
    }
    if (lowerData[P_effectPot3] > 2047) {
      lowerfastpot3 = lowerData[P_effectPot3];
      lowerfast = false;
      lowerslow = true;
    }
    if (wholemode) {
      if (upperData[P_effectPot3] < 2047) {
        upperslowpot3 = upperData[P_effectPot3];
        upperfast = true;
        upperslow = false;
      }
      if (upperData[P_effectPot3] > 2047) {
        upperfastpot3 = upperData[P_effectPot3];
        upperfast = false;
        upperslow = true;
      }
    }
  }
}

void changeSpeed() {
  if (upperfootPedal) {
    if (upperslow) {
      upperData[P_effectPot3] -= 3;
      if (upperData[P_effectPot3] <= upperslowpot3) {
        upperData[P_effectPot3] = upperslowpot3;
        midiCCOut71(CCeffectPot3, upperData[P_effectPot3]);
        upperLastSentPot3 = upperData[P_effectPot3];
        upperfootPedal = false;
        upperslow = false;
      } else if (abs(upperData[P_effectPot3] - upperLastSentPot3) >= 32) {
        midiCCOut71(CCeffectPot3, upperData[P_effectPot3]);
        upperLastSentPot3 = upperData[P_effectPot3];
      }
    } else if (upperfast) {
      upperData[P_effectPot3] += 3;
      if (upperData[P_effectPot3] >= upperfastpot3) {
        upperData[P_effectPot3] = upperfastpot3;
        midiCCOut71(CCeffectPot3, upperData[P_effectPot3]);
        upperLastSentPot3 = upperData[P_effectPot3];
        upperfootPedal = false;
        upperfast = false;
      } else if (abs(upperData[P_effectPot3] - upperLastSentPot3) >= 32) {
        midiCCOut71(CCeffectPot3, upperData[P_effectPot3]);
        upperLastSentPot3 = upperData[P_effectPot3];
      }
    }
  }

  if (lowerfootPedal) {
    if (lowerslow) {
      lowerData[P_effectPot3] -= 3;
      if (lowerData[P_effectPot3] <= lowerslowpot3) {
        lowerData[P_effectPot3] = lowerslowpot3;
        midiCCOut71(CCeffectPot3, lowerData[P_effectPot3]);
        lowerLastSentPot3 = lowerData[P_effectPot3];
        lowerfootPedal = false;
        lowerslow = false;
      } else if (abs(lowerData[P_effectPot3] - lowerLastSentPot3) >= 32) {
        midiCCOut71(CCeffectPot3, lowerData[P_effectPot3]);
        lowerLastSentPot3 = lowerData[P_effectPot3];
      }
    } else if (lowerfast) {
      lowerData[P_effectPot3] += 3;
      if (lowerData[P_effectPot3] >= lowerfastpot3) {
        lowerData[P_effectPot3] = lowerfastpot3;
        midiCCOut71(CCeffectPot3, lowerData[P_effectPot3]);
        lowerLastSentPot3 = lowerData[P_effectPot3];
        lowerfootPedal = false;
        lowerfast = false;
      } else if (abs(lowerData[P_effectPot3] - lowerLastSentPot3) >= 32) {
        midiCCOut71(CCeffectPot3, lowerData[P_effectPot3]);
        lowerLastSentPot3 = lowerData[P_effectPot3];
      }
    }
  }
}

void updatefilterenvLogLin(boolean announce) {

  if (upperSW) {
    if (!upperData[P_filterLogLin]) {
      if (announce) {
        showCurrentParameterPage("Filter Env", "Linear");
      }
      midiCCOut(CCfilterenvLinLogSW, 0);
      midiCCOut72(CCfilterenvLinLogSW, 0);
      srp.writePin(FILTER_LIN_LOG_UPPER, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Filter Env", "Log");
      }
      midiCCOut(CCfilterenvLinLogSW, 127);
      midiCCOut72(CCfilterenvLinLogSW, 1);
      srp.writePin(FILTER_LIN_LOG_UPPER, HIGH);
    }
  } else {
    if (!lowerData[P_filterLogLin]) {
      if (announce) {
        showCurrentParameterPage("Filter Env", "Linear");
      }
      midiCCOut(CCfilterenvLinLogSW, 0);
      midiCCOut72(CCfilterenvLinLogSW, 0);
      srp.writePin(FILTER_LIN_LOG_LOWER, LOW);
      if (wholemode) {
        srp.writePin(FILTER_LIN_LOG_UPPER, LOW);
      }
    } else {
      if (announce) {
        showCurrentParameterPage("Filter Env", "Log");
      }
      midiCCOut(CCfilterenvLinLogSW, 127);
      midiCCOut72(CCfilterenvLinLogSW, 1);
      srp.writePin(FILTER_LIN_LOG_LOWER, HIGH);
      if (wholemode) {
        srp.writePin(FILTER_LIN_LOG_UPPER, HIGH);
      }
    }
  }
}

void updateampenvLogLin(boolean announce) {
  if (upperSW) {
    if (!upperData[P_ampLogLin]) {
      if (announce) {
        showCurrentParameterPage("Amp Env", "Linear");
      }
      midiCCOut(CCampenvLinLogSW, 0);
      midiCCOut72(CCampenvLinLogSW, 0);
      srp.writePin(AMP_LIN_LOG_UPPER, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Amp Env", "Log");
      }
      midiCCOut(CCampenvLinLogSW, 127);
      midiCCOut72(CCampenvLinLogSW, 1);
      srp.writePin(AMP_LIN_LOG_UPPER, HIGH);
    }
  } else {
    if (!lowerData[P_ampLogLin]) {
      if (announce) {
        showCurrentParameterPage("Amp Env", "Linear");
      }
      midiCCOut(CCampenvLinLogSW, 0);
      midiCCOut72(CCampenvLinLogSW, 0);
      srp.writePin(AMP_LIN_LOG_LOWER, LOW);
      if (wholemode) {
        srp.writePin(AMP_LIN_LOG_UPPER, LOW);
      }
    } else {
      if (announce) {
        showCurrentParameterPage("Amp Env", "Log");
      }
      midiCCOut(CCampenvLinLogSW, 127);
      midiCCOut72(CCampenvLinLogSW, 1);
      srp.writePin(AMP_LIN_LOG_LOWER, HIGH);
      if (wholemode) {
        srp.writePin(AMP_LIN_LOG_UPPER, HIGH);
      }
    }
  }
}

void updatefilterVel(boolean announce) {
  if (upperSW) {
    if (upperData[P_filterVel] == 0) {
      if (announce) {
        showCurrentParameterPage("VCF Velocity", "Off");
      }
      midiCCOut72(CCfilterVel, 0);
      midiCCOut(CCfilterVel, 0);
      srp.writePin(FILTER_VELOCITY_UPPER, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("VCF Velocity", "On");
      }
      midiCCOut72(CCfilterVel, 1);
      midiCCOut(CCfilterVel, 127);
      srp.writePin(FILTER_VELOCITY_UPPER, HIGH);
    }
  } else {
    if (lowerData[P_filterVel] == 0) {
      if (announce) {
        showCurrentParameterPage("VCF Velocity", "Off");
      }
      midiCCOut72(CCfilterVel, 0);
      midiCCOut(CCfilterVel, 0);
      srp.writePin(FILTER_VELOCITY_LOWER, LOW);
      if (wholemode) {
        srp.writePin(FILTER_VELOCITY_UPPER, LOW);
      }
    } else {
      if (announce) {
        showCurrentParameterPage("VCF Velocity", "On");
      }
      midiCCOut72(CCfilterVel, 1);
      midiCCOut(CCfilterVel, 127);
      srp.writePin(FILTER_VELOCITY_LOWER, HIGH);
      if (wholemode) {
        srp.writePin(FILTER_VELOCITY_UPPER, HIGH);
      }
    }
  }
}

void updateNotePriority(boolean announce) {
  if (upperSW) {
    if (dualmode) {
      lowerData[P_NotePriority] = upperData[P_NotePriority];
    }
    switch (upperData[P_NotePriority]) {
      case 0:
        if (announce) {
          showCurrentParameterPage("Note Priority", "Top");
        }
        midiCCOut72(CCNotePriority, 0);
        midiCCOut(CCNotePriority, 0);
        break;

      case 1:
        if (announce) {
          showCurrentParameterPage("Note Priority", "Bottom");
        }
        midiCCOut72(CCNotePriority, 1);
        midiCCOut(CCNotePriority, 63);
        break;

      case 2:
        if (announce) {
          showCurrentParameterPage("Note Priority", "Last");
        }
        midiCCOut72(CCNotePriority, 2);
        midiCCOut(CCNotePriority, 127);
        break;
    }
  } else {
    if (dualmode) {
      upperData[P_NotePriority] = lowerData[P_NotePriority];
    }
    switch (lowerData[P_NotePriority]) {
      case 0:
        if (announce) {
          showCurrentParameterPage("Note Priority", "Top");
        }
        midiCCOut72(CCNotePriority, 0);
        midiCCOut(CCNotePriority, 0);
        break;

      case 1:
        if (announce) {
          showCurrentParameterPage("Note Priority", "Bottom");
        }
        midiCCOut72(CCNotePriority, 1);
        midiCCOut(CCNotePriority, 63);
        break;

      case 2:
        if (announce) {
          showCurrentParameterPage("Note Priority", "Last");
        }
        midiCCOut72(CCNotePriority, 2);
        midiCCOut(CCNotePriority, 127);
        break;
    }
  }
}

void updatevcaLoop(boolean announce) {
  if (upperSW) {
    switch (upperData[P_vcaLoop]) {
      case 0:
        if (announce) {
          showCurrentParameterPage("VCA Loop", "Off");
        }
        midiCCOut72(CCAmpLoop, 0);
        midiCCOut(CCAmpLoop, 0);
        srp.writePin(AMP_MODE_BIT0_UPPER, LOW);
        srp.writePin(AMP_MODE_BIT1_UPPER, LOW);
        break;

      case 1:
        if (announce) {
          showCurrentParameterPage("VCA Loop", "Gated");
        }
        midiCCOut72(CCAmpLoop, 1);
        midiCCOut(CCAmpLoop, 63);
        srp.writePin(AMP_MODE_BIT0_UPPER, HIGH);
        srp.writePin(AMP_MODE_BIT1_UPPER, LOW);
        break;

      case 2:
        if (announce) {
          showCurrentParameterPage("VCA Loop", "LFO");
        }
        midiCCOut72(CCAmpLoop, 2);
        midiCCOut(CCAmpLoop, 127);
        srp.writePin(AMP_MODE_BIT0_UPPER, HIGH);
        srp.writePin(AMP_MODE_BIT1_UPPER, HIGH);
        break;
    }
  } else {
    switch (lowerData[P_vcaLoop]) {
      case 0:
        if (announce) {
          showCurrentParameterPage("VCA Loop", "Off");
        }
        midiCCOut72(CCAmpLoop, 0);
        midiCCOut(CCAmpLoop, 0);
        srp.writePin(AMP_MODE_BIT0_LOWER, LOW);
        srp.writePin(AMP_MODE_BIT1_LOWER, LOW);
        if (wholemode) {
          srp.writePin(AMP_MODE_BIT0_UPPER, LOW);
          srp.writePin(AMP_MODE_BIT1_UPPER, LOW);
        }
        break;

      case 1:
        if (announce) {
          showCurrentParameterPage("VCA Loop", "Gated");
        }
        midiCCOut72(CCAmpLoop, 1);
        midiCCOut(CCAmpLoop, 63);
        srp.writePin(AMP_MODE_BIT0_LOWER, HIGH);
        srp.writePin(AMP_MODE_BIT1_LOWER, LOW);
        if (wholemode) {
          srp.writePin(AMP_MODE_BIT0_UPPER, HIGH);
          srp.writePin(AMP_MODE_BIT1_UPPER, LOW);
        }
        break;

      case 2:
        if (announce) {
          showCurrentParameterPage("VCA Loop", "LFO");
        }
        midiCCOut72(CCAmpLoop, 2);
        midiCCOut(CCAmpLoop, 127);
        srp.writePin(AMP_MODE_BIT0_LOWER, HIGH);
        srp.writePin(AMP_MODE_BIT1_LOWER, HIGH);
        if (wholemode) {
          srp.writePin(AMP_MODE_BIT0_UPPER, HIGH);
          srp.writePin(AMP_MODE_BIT1_UPPER, HIGH);
        }
        break;
    }
  }
}

void updatevcaVel(boolean announce) {
  if (upperSW) {
    if (upperData[P_vcaVel] == 0) {
      if (announce) {
        showCurrentParameterPage("VCA Velocity", "Off");
      }
      midiCCOut72(CCvcaVel, 0);
      midiCCOut(CCvcaVel, 0);
      srp.writePin(AMP_VELOCITY_UPPER, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("VCA Velocity", "On");
      }
      midiCCOut72(CCvcaVel, 1);
      midiCCOut(CCvcaVel, 127);
      srp.writePin(AMP_VELOCITY_UPPER, HIGH);
    }
  } else {
    if (lowerData[P_vcaVel] == 0) {
      if (announce) {
        showCurrentParameterPage("VCA Velocity", "Off");
      }
      midiCCOut72(CCvcaVel, 0);
      midiCCOut(CCvcaVel, 0);
      srp.writePin(AMP_VELOCITY_LOWER, LOW);
      if (wholemode) {
        srp.writePin(AMP_VELOCITY_UPPER, LOW);
      }
    } else {
      if (announce) {
        showCurrentParameterPage("VCA Velocity", "On");
      }
      midiCCOut72(CCvcaVel, 1);
      midiCCOut(CCvcaVel, 127);
      srp.writePin(AMP_VELOCITY_LOWER, HIGH);
      if (wholemode) {
        srp.writePin(AMP_VELOCITY_UPPER, HIGH);
      }
    }
  }
}

void updatevcaGate(boolean announce) {
  if (upperSW) {
    if (!upperData[P_vcaGate]) {
      if (announce) {
        showCurrentParameterPage("VCA Gate", "Off");
      }
      midiCCOut(CCvcaGate, 0);
      midiCCOut72(CCvcaGate, 0);
      upperData[P_ampAttack] = upperData[P_oldampAttack];
      upperData[P_ampDecay] = upperData[P_oldampDecay];
      upperData[P_ampSustain] = upperData[P_oldampSustain];
      upperData[P_ampRelease] = upperData[P_oldampRelease];
    } else {
      if (announce) {
        showCurrentParameterPage("VCA Gate", "On");
      }
      midiCCOut(CCvcaGate, 127);
      midiCCOut72(CCvcaGate, 1);
      upperData[P_ampAttack] = 0;
      upperData[P_ampDecay] = 0;
      upperData[P_ampSustain] = 127;
      upperData[P_ampRelease] = 0;
    }
  } else {
    if (!lowerData[P_vcaGate]) {
      if (announce) {
        showCurrentParameterPage("VCA Gate", "Off");
      }
      midiCCOut(CCvcaGate, 0);
      midiCCOut72(CCvcaGate, 0);
      lowerData[P_ampAttack] = lowerData[P_oldampAttack];
      lowerData[P_ampDecay] = lowerData[P_oldampDecay];
      lowerData[P_ampSustain] = lowerData[P_oldampSustain];
      lowerData[P_ampRelease] = lowerData[P_oldampRelease];
      if (wholemode) {
        upperData[P_ampAttack] = upperData[P_oldampAttack];
        upperData[P_ampDecay] = upperData[P_oldampDecay];
        upperData[P_ampSustain] = upperData[P_oldampSustain];
        upperData[P_ampRelease] = upperData[P_oldampRelease];
      }
    } else {
      if (announce) {
        showCurrentParameterPage("VCA Gate", "On");
      }
      midiCCOut(CCvcaGate, 127);
      midiCCOut72(CCvcaGate, 1);
      lowerData[P_ampAttack] = 0;
      lowerData[P_ampDecay] = 0;
      lowerData[P_ampSustain] = 127;
      lowerData[P_ampRelease] = 0;
      if (wholemode) {
        upperData[P_ampAttack] = 0;
        upperData[P_ampDecay] = 0;
        upperData[P_ampSustain] = 127;
        upperData[P_ampRelease] = 0;
      }
    }
  }
}

void updatelfoAlt(boolean announce) {
  bool isUpper = upperSW;
  bool lfoAltState = isUpper ? upperData[P_lfoAlt] : lowerData[P_lfoAlt];

  // Send MIDI CC messages
  midiCCOut(CClfoAlt, lfoAltState ? 127 : 0);
  midiCCOut72(CClfoAlt, lfoAltState ? 1 : 0);

  // Update LFO waveform
  if (announce) {
    updateStratusLFOWaveform(1);
  } else {
    updateStratusLFOWaveform(0);
  }

  // Set pin states
  if (isUpper) {
    srp.writePin(LFO_ALT_UPPER, lfoAltState ? LOW : HIGH);
  } else {
    srp.writePin(LFO_ALT_LOWER, lfoAltState ? LOW : HIGH);
    if (wholemode) {
      srp.writePin(LFO_ALT_UPPER, lfoAltState ? LOW : HIGH);
    }
  }
}

void updateupperSW(boolean announce) {
  if (!wholemode) {
    if (upperSW) {
      midiCCOut72(CClowerSW, 0);
      midiCCOut72(CCupperSW, 1);
      upperParamsToDisplay();
      setAllButtons();
      srp.writePin(UPPER_RELAY_1, HIGH);
    }
  }
}

void updatelowerSW(boolean announce) {
  if (lowerSW) {
    midiCCOut72(CCupperSW, 0);
    midiCCOut72(CClowerSW, 1);
    lowerParamsToDisplay();
    setAllButtons();
    srp.writePin(UPPER_RELAY_1, LOW);
  }
}

void updateMonoMulti(boolean announce) {
  if (upperSW) {
    if (!upperData[P_monoMulti]) {
      if (announce) {
        showCurrentParameterPage("LFO Retrigger", "Off");
      }
      midiCCOut(CCmonoMulti, 0);
      midiCCOut72(CCmonoMulti, 0);
    } else {
      if (announce) {
        showCurrentParameterPage("LFO Retrigger", "On");
      }
      midiCCOut(CCmonoMulti, 127);
      midiCCOut72(CCmonoMulti, 1);
    }
  } else {
    if (!lowerData[P_monoMulti]) {
      if (announce) {
        showCurrentParameterPage("LFO Retrigger", "Off");
      }
      midiCCOut(CCmonoMulti, 0);
      midiCCOut72(CCmonoMulti, 0);
      if (wholemode) {
        upperData[P_monoMulti] = lowerData[P_monoMulti];
      }
    } else {
      if (announce) {
        showCurrentParameterPage("LFO Retrigger", "On");
      }
      midiCCOut(CCmonoMulti, 127);
      midiCCOut72(CCmonoMulti, 1);
      if (wholemode) {
        upperData[P_monoMulti] = lowerData[P_monoMulti];
      }
    }
  }
}

void updatePatchname() {
  refreshPatchDisplayFromState();
}

void myControlChange(byte channel, byte control, int value) {

  switch (control) {

    case CCglideTime:
      if (upperSW) {
        upperData[P_glideTime] = value;
      } else {
        lowerData[P_glideTime] = value;
        if (wholemode) {
          upperData[P_glideTime] = value;
        }
      }
      glideTimestr = LINEAR[value];
      updateglideTime(1);
      break;

    case CCpwLFO:
      if (upperSW) {
        upperData[P_pwLFO] = value;
      } else {
        lowerData[P_pwLFO] = value;
        if (wholemode) {
          upperData[P_pwLFO] = value;
        }
      }
      pwLFOstr = LFOTEMPO[value];  // for display
      updatepwLFO(1);
      break;

    case CCfmDepth:
      if (upperSW) {
        upperData[P_fmDepth] = value;
      } else {
        lowerData[P_fmDepth] = value;
        if (wholemode) {
          upperData[P_fmDepth] = value;
        }
      }
      fmDepthstr = value;
      updatefmDepth(1);
      break;

    case CCosc2PW:
      if (upperSW) {
        upperData[P_osc2PW] = value;
      } else {
        lowerData[P_osc2PW] = value;
        if (wholemode) {
          upperData[P_osc2PW] = value;
        }
      }
      osc2PWstr = PULSEWIDTH[value];
      updateosc2PW(1);
      break;

    case CCosc2PWM:
      if (upperSW) {
        upperData[P_osc2PWM] = value;
      } else {
        lowerData[P_osc2PWM] = value;
        if (wholemode) {
          upperData[P_osc2PWM] = value;
        }
      }
      osc2PWMstr = value;
      updateosc2PWM(1);
      break;

    case CCosc1PW:
      if (upperSW) {
        upperData[P_osc1PW] = value;
      } else {
        lowerData[P_osc1PW] = value;
        if (wholemode) {
          upperData[P_osc1PW] = value;
        }
      }
      osc1PWstr = PULSEWIDTH[value];
      updateosc1PW(1);
      break;

    case CCosc1PWM:
      if (upperSW) {
        upperData[P_osc1PWM] = value;
      } else {
        lowerData[P_osc1PWM] = value;
        if (wholemode) {
          upperData[P_osc1PWM] = value;
        }
      }
      osc1PWMstr = value;
      updateosc1PWM(1);
      break;

    case CCosc1Oct:
      if (upperSW) {
        upperData[P_osc1Range] = value;
      } else {
        lowerData[P_osc1Range] = value;
        if (wholemode) {
          upperData[P_osc1Range] = value;
        }
      }
      updateosc1Range(1);
      break;

    case CCosc2Oct:
      if (upperSW) {
        upperData[P_osc2Range] = value;
      } else {
        lowerData[P_osc2Range] = value;
        if (wholemode) {
          upperData[P_osc2Range] = value;
        }
      }
      updateosc2Range(1);
      break;

    case CCosc2Detune:
      if (upperSW) {
        upperData[P_osc2Detune] = value;
      } else {
        lowerData[P_osc2Detune] = value;
        if (wholemode) {
          upperData[P_osc2Detune] = value;
        }
      }
      osc2Detunestr = PULSEWIDTH[value];
      updateosc2Detune(1);
      break;

    case CCosc2Interval:
      if (upperSW) {
        upperData[P_osc2Interval] = value;
      } else {
        lowerData[P_osc2Interval] = value;
        if (wholemode) {
          upperData[P_osc2Interval] = value;
        }
      }
      osc2Intervalstr = value;
      updateosc2Interval(1);
      break;

    case CCATDepth:
      if (upperSW) {
        upperData[P_ATDepth] = value;
      } else {
        lowerData[P_ATDepth] = value;
        if (wholemode) {
          upperData[P_ATDepth] = value;
        }
      }
      ATDepthstr = value;
      updateATDepth(1);
      break;

    case CCnoiseLevel:
      if (upperSW) {
        upperData[P_noiseLevel] = value;
      } else {
        lowerData[P_noiseLevel] = value;
        if (wholemode) {
          upperData[P_noiseLevel] = value;
        }
      }
      noiseLevelstr = LINEARCENTREZERO[value];
      updatenoiseLevel(1);
      break;

    case CCosc2SawLevel:
      if (upperSW) {
        upperData[P_osc2SawLevel] = value;
      } else {
        lowerData[P_osc2SawLevel] = value;
        if (wholemode) {
          upperData[P_osc2SawLevel] = value;
        }
      }
      osc2SawLevelstr = value;  // for display
      updateOsc2SawLevel(1);
      break;

    case CCosc1SawLevel:
      if (upperSW) {
        upperData[P_osc1SawLevel] = value;
      } else {
        lowerData[P_osc1SawLevel] = value;
        if (wholemode) {
          upperData[P_osc1SawLevel] = value;
        }
      }
      osc1SawLevelstr = value;  // for display
      updateOsc1SawLevel(1);
      break;

    case CCosc2PulseLevel:
      if (upperSW) {
        upperData[P_osc2PulseLevel] = value;
      } else {
        lowerData[P_osc2PulseLevel] = value;
        if (wholemode) {
          upperData[P_osc2PulseLevel] = value;
        }
      }
      osc2PulseLevelstr = value;  // for display
      updateOsc2PulseLevel(1);
      break;

    case CCosc1PulseLevel:
      if (upperSW) {
        upperData[P_osc1PulseLevel] = value;
      } else {
        lowerData[P_osc1PulseLevel] = value;
        if (wholemode) {
          upperData[P_osc1PulseLevel] = value;
        }
      }
      osc1PulseLevelstr = value;  // for display
      updateOsc1PulseLevel(1);
      break;

    case CCosc2TriangleLevel:
      if (upperSW) {
        upperData[P_osc2TriangleLevel] = value;
      } else {
        lowerData[P_osc2TriangleLevel] = value;
        if (wholemode) {
          upperData[P_osc2TriangleLevel] = value;
        }
      }
      osc2TriangleLevelstr = value;  // for display
      updateOsc2TriangleLevel(1);
      break;

    case CCosc1SubLevel:
      if (upperSW) {
        upperData[P_osc1SubLevel] = value;
      } else {
        lowerData[P_osc1SubLevel] = value;
        if (wholemode) {
          upperData[P_osc1SubLevel] = value;
        }
      }
      osc1SubLevelstr = value;  // for display
      updateOsc1SubLevel(1);
      break;

    case CCLFODelay:
      if (upperSW) {
        upperData[P_LFODelay] = value;
      } else {
        lowerData[P_LFODelay] = value;
        if (wholemode) {
          upperData[P_LFODelay] = value;
        }
      }
      LFODelaystr = value;  // for display
      updateLFODelay(1);
      break;

    case CCfilterCutoff:
      if (upperSW) {
        upperData[P_filterCutoff] = value;
        oldfilterCutoffU = value;
      } else {
        lowerData[P_filterCutoff] = value;
        oldfilterCutoffL = value;
        if (wholemode) {
          upperData[P_filterCutoff] = value;
          oldfilterCutoffU = value;
        }
      }
      filterCutoffstr = FILTERCUTOFF[value];
      updateFilterCutoff(1);
      break;

    case CCfilterLFO:
      if (upperSW) {
        upperData[P_filterLFO] = value;
      } else {
        lowerData[P_filterLFO] = value;
        if (wholemode) {
          upperData[P_filterLFO] = value;
        }
      }
      filterLFOstr = value;
      updatefilterLFO(1);
      break;

    case CCfilterRes:
      if (upperSW) {
        upperData[P_filterRes] = value;
      } else {
        lowerData[P_filterRes] = value;
        if (wholemode) {
          upperData[P_filterRes] = value;
        }
      }
      filterResstr = int(value);
      updatefilterRes(1);
      break;

    case CCfilterType:
      if (upperSW) {
        upperData[P_filterType] = value;
      } else {
        lowerData[P_filterType] = value;
        if (wholemode) {
          upperData[P_filterType] = value;
        }
      }
      updateFilterType(1);
      break;

    case CCfilterEGlevel:
      if (upperSW) {
        upperData[P_filterEGlevel] = value;
      } else {
        lowerData[P_filterEGlevel] = value;
        if (wholemode) {
          upperData[P_filterEGlevel] = value;
        }
      }
      filterEGlevelstr = int(value);
      updatefilterEGlevel(1);
      break;

    case CCLFORate:
      if (upperSW) {
        upperData[P_LFORate] = value;
      } else {
        lowerData[P_LFORate] = value;
        if (wholemode) {
          upperData[P_LFORate] = value;
        }
      }
      LFORatestr = LFOTEMPO[value];  // for display
      updateLFORate(1);
      break;

    case CCmodWheelDepth:
      if (upperSW) {
        upperData[P_modWheelDepth] = value;
      } else {
        lowerData[P_modWheelDepth] = value;
        if (wholemode) {
          upperData[P_modWheelDepth] = value;
        }
      }
      modWheelDepthstr = value;  // for display
      updatemodWheelDepth(1);
      break;

    case CCPitchBend:
      if (upperSW) {
        upperData[P_PitchBendLevel] = value;
      } else {
        lowerData[P_PitchBendLevel] = value;
        if (wholemode) {
          upperData[P_PitchBendLevel] = value;
        }
      }
      PitchBendLevelstr = value;  // for display
      updatePitchBendDepth(1);
      break;

    case CCeffectPot1:
      if (upperSW) {
        upperData[P_effectPot1] = value;
      } else {
        lowerData[P_effectPot1] = value;
        if (wholemode) {
          upperData[P_effectPot1] = value;
        }
      }
      effectPot1str = value;  // for display
      updateeffectPot1(1);
      break;

    case CCeffectPot2:
      if (upperSW) {
        upperData[P_effectPot2] = value;
      } else {
        lowerData[P_effectPot2] = value;
        if (wholemode) {
          upperData[P_effectPot2] = value;
        }
      }
      effectPot2str = value;  // for display
      updateeffectPot2(1);
      break;

    case CCeffectPot3:
      if (upperSW) {
        upperData[P_effectPot3] = value;
      } else {
        lowerData[P_effectPot3] = value;
        if (wholemode) {
          upperData[P_effectPot3] = value;
        }
      }
      effectPot3str = value;  // for display
      updateeffectPot3(1);
      break;

    case CCeffectsMix:
      if (upperSW) {
        upperData[P_effectsMix] = value;
      } else {
        lowerData[P_effectsMix] = value;
        if (wholemode) {
          upperData[P_effectsMix] = value;
        }
      }
      effectsMixstr = value;  // for display
      updateeffectsMix(1);
      break;

    case CCfilterAttack:
      if (upperSW) {
        upperData[P_filterAttack] = value;
      } else {
        lowerData[P_filterAttack] = value;
        if (wholemode) {
          upperData[P_filterAttack] = value;
        }
      }
      filterAttackstr = ENVTIMES[value];
      updatefilterAttack(1);
      break;

    case CCfilterDecay:
      if (upperSW) {
        upperData[P_filterDecay] = value;
      } else {
        lowerData[P_filterDecay] = value;
        if (wholemode) {
          upperData[P_filterDecay] = value;
        }
      }
      filterDecaystr = ENVTIMES[value];
      updatefilterDecay(1);
      break;

    case CCfilterSustain:
      if (upperSW) {
        upperData[P_filterSustain] = value;
      } else {
        lowerData[P_filterSustain] = value;
        if (wholemode) {
          upperData[P_filterSustain] = value;
        }
      }
      filterSustainstr = LINEAR_FILTERMIXERSTR[value];
      updatefilterSustain(1);
      break;

    case CCfilterRelease:
      if (upperSW) {
        upperData[P_filterRelease] = value;
      } else {
        lowerData[P_filterRelease] = value;
        if (wholemode) {
          upperData[P_filterRelease] = value;
        }
      }
      filterReleasestr = ENVTIMES[value];
      updatefilterRelease(1);
      break;

    case CCampAttack:
      if (upperSW) {
        upperData[P_ampAttack] = value;
        upperData[P_oldampAttack] = value;
      } else {
        lowerData[P_ampAttack] = value;
        lowerData[P_oldampAttack] = value;
        if (wholemode) {
          upperData[P_ampAttack] = value;
          upperData[P_oldampAttack] = value;
        }
      }
      ampAttackstr = ENVTIMES[value];
      updateampAttack(1);
      break;

    case CCampDecay:
      if (upperSW) {
        upperData[P_ampDecay] = value;
        upperData[P_oldampDecay] = value;
      } else {
        lowerData[P_ampDecay] = value;
        lowerData[P_oldampDecay] = value;
        if (wholemode) {
          upperData[P_ampDecay] = value;
          upperData[P_oldampDecay] = value;
        }
      }
      ampDecaystr = ENVTIMES[value];
      updateampDecay(1);
      break;

    case CCampSustain:
      if (upperSW) {
        upperData[P_ampSustain] = value;
        upperData[P_oldampSustain] = value;
      } else {
        lowerData[P_ampSustain] = value;
        lowerData[P_oldampSustain] = value;
        if (wholemode) {
          upperData[P_ampSustain] = value;
          upperData[P_oldampSustain] = value;
        }
      }
      ampSustainstr = LINEAR_FILTERMIXERSTR[value];
      updateampSustain(1);
      break;

    case CCampRelease:
      if (upperSW) {
        upperData[P_ampRelease] = value;
        upperData[P_oldampRelease] = value;
      } else {
        lowerData[P_ampRelease] = value;
        lowerData[P_oldampRelease] = value;
        if (wholemode) {
          upperData[P_ampRelease] = value;
          upperData[P_oldampRelease] = value;
        }
      }
      ampReleasestr = ENVTIMES[value];
      updateampRelease(1);
      break;

    case CCvolumeControl:
      if (upperSW) {
        upperData[P_volumeControl] = value;
      } else {
        lowerData[P_volumeControl] = value;
        if (wholemode) {
          upperData[P_volumeControl] = value;
        }
      }
      volumeControlstr = value;
      updatevolumeControl(1);
      break;

    case CCPM_DCO2:
      if (upperSW) {
        upperData[P_pmDCO2] = value;
      } else {
        lowerData[P_pmDCO2] = value;
        if (wholemode) {
          upperData[P_pmDCO2] = value;
        }
      }
      pmDCO2str = value;
      updatePM_DCO2(1);
      break;

    case CCPM_FilterEnv:
      if (upperSW) {
        upperData[P_pmFilterEnv] = value;
      } else {
        lowerData[P_pmFilterEnv] = value;
        if (wholemode) {
          upperData[P_pmFilterEnv] = value;
        }
      }
      pmFilterEnvstr = value;
      updatePM_FilterEnv(1);
      break;

    case CCkeyTrack:
      if (upperSW) {
        upperData[P_keytrack] = value;
      } else {
        lowerData[P_keytrack] = value;
        if (wholemode) {
          upperData[P_keytrack] = value;
        }
      }
      keytrackstr = value;
      updatekeytrack(1);
      break;


    case CCamDepth:
      if (upperSW) {
        upperData[P_amDepth] = value;
      } else {
        lowerData[P_amDepth] = value;
        if (wholemode) {
          upperData[P_amDepth] = value;
        }
      }
      amDepthstr = value;
      updateamDepth(1);
      break;

      //   ////////////////////////////////////////////////

    case CCplayMode:
      updateplayMode(1);
      break;

    case CCNotePriority:
      if (upperData[P_keyboardMode] >= 2) {
        if (upperSW) {
          upperData[P_NotePriority] = value;
        }
        updateNotePriority(1);
      }
      if (lowerData[P_keyboardMode] >= 2) {
        if (lowerSW) {
          lowerData[P_NotePriority] = value;
        }
        updateNotePriority(1);
      }
      break;

    case CCkeyboardMode:
      if (upperSW) {
        upperData[P_keyboardMode] = panelData[P_keyboardMode];
      } else {
        lowerData[P_keyboardMode] = panelData[P_keyboardMode];
      }
      updatekeyboardMode(1);
      break;

    case CCglideSW:
      if (upperSW) {
        upperData[P_glideSW] = !upperData[P_glideSW];
      } else {
        lowerData[P_glideSW] = !lowerData[P_glideSW];
      }
      updateglideSW(1);
      break;

    case CCfilterPoleSW:
      if (upperSW) {
        upperData[P_filterPoleSW] = value;
      } else {
        lowerData[P_filterPoleSW] = value;
      }
      updatefilterPoleSwitch(1);
      break;

    case CCfilterVel:
      if (upperSW) {
        upperData[P_filterVel] = !upperData[P_filterVel];
      } else {
        lowerData[P_filterVel] = !lowerData[P_filterVel];
      }
      updatefilterVel(1);
      break;

    case CCfilterEGinv:
      if (upperSW) {
        upperData[P_filterEGinv] = !upperData[P_filterEGinv];
      } else {
        lowerData[P_filterEGinv] = !lowerData[P_filterEGinv];
      }
      updatefilterEGinv(1);
      break;

    case CCsyncSW:
      if (upperSW) {
        upperData[P_sync] = !upperData[P_sync];
      } else {
        lowerData[P_sync] = !lowerData[P_sync];
      }
      updatesyncSW(1);
      break;

    case CCeffectparam3:
      if (value > 63) {
        if (upperSW) {
          upperfootPedal = true;
        } else {
          lowerfootPedal = true;
        }
        updatefootSwitch();
      }
      break;

    case CCkeyTrackSW:
      if (upperSW) {
        upperData[P_keytrackSW] = !upperData[P_keytrackSW];
      } else {
        lowerData[P_keytrackSW] = !lowerData[P_keytrackSW];
      }
      updatekeyTrackSW(1);
      break;

    case CCpmDestDCO1SW:
      if (upperSW) {
        upperData[P_pmDestDCO1] = !upperData[P_pmDestDCO1];
      } else {
        lowerData[P_pmDestDCO1] = !lowerData[P_pmDestDCO1];
      }
      updatepmDestDCO1(1);
      break;

    case CCpmDestFilterSW:
      if (upperSW) {
        upperData[P_pmDestFilter] = !upperData[P_pmDestFilter];
      } else {
        lowerData[P_pmDestFilter] = !lowerData[P_pmDestFilter];
      }
      updatepmDestFilter(1);
      break;

    case CCfilterenvLinLogSW:
      if (upperSW) {
        upperData[P_filterLogLin] = !upperData[P_filterLogLin];
      } else {
        lowerData[P_filterLogLin] = !lowerData[P_filterLogLin];
      }
      updatefilterenvLogLin(1);
      break;

    case CCampenvLinLogSW:
      if (upperSW) {
        upperData[P_ampLogLin] = !upperData[P_ampLogLin];
      } else {
        lowerData[P_ampLogLin] = !lowerData[P_ampLogLin];
      }
      updateampenvLogLin(1);
      break;

    case CCFilterLoop:
      if (upperSW) {
        upperData[P_filterLoop] = value;
      } else {
        lowerData[P_filterLoop] = value;
      }
      updatefilterLoop(1);
      break;

    case CCAmpLoop:
      if (upperSW) {
        upperData[P_vcaLoop] = value;
      } else {
        lowerData[P_vcaLoop] = value;
      }
      updatevcaLoop(1);
      break;

    case CCchordHoldSW:
      if (upperSW) {
        chordHoldU = !chordHoldU;
      } else {
        chordHoldL = !chordHoldL;
      }
      updatechordHoldSW(1);
      break;

    case CCvcaVel:
      if (upperSW) {
        upperData[P_vcaVel] = !upperData[P_vcaVel];
      } else {
        lowerData[P_vcaVel] = !lowerData[P_vcaVel];
      }
      updatevcaVel(1);
      break;

    case CCeffectBankSW:
      if (upperSW) {
        upperData[P_effectBank] = value;
      } else {
        lowerData[P_effectBank] = value;
      }
      updateeffectBankSW(1);
      break;

    case CClfoMult:
      if (upperSW) {
        upperData[P_lfoMultiplier] = value;
      } else {
        lowerData[P_lfoMultiplier] = value;
      }
      updatelfoMultiplier(1);
      break;

    case CCeffectNumSW:
      if (upperSW) {
        upperData[P_effectNum] = value;
      } else {
        lowerData[P_effectNum] = value;
      }
      updateeffectNumSW(1);
      break;

    case CCvcaGate:
      if (upperSW) {
        upperData[P_vcaGate] = !upperData[P_vcaGate];
      } else {
        lowerData[P_vcaGate] = !lowerData[P_vcaGate];
      }
      updatevcaGate(1);
      break;

    case CCmonoMulti:
      if (upperSW) {
        upperData[P_monoMulti] = !upperData[P_monoMulti];
      } else {
        lowerData[P_monoMulti] = !lowerData[P_monoMulti];
      }
      updateMonoMulti(1);
      break;

    case CClfoAlt:
      if (upperSW) {
        upperData[P_lfoAlt] = !upperData[P_lfoAlt];
      } else {
        lowerData[P_lfoAlt] = !lowerData[P_lfoAlt];
      }
      updatelfoAlt(1);
      break;

    case CCLFOWaveform:
      if (upperSW) {
        upperData[P_LFOWaveform] = value;
      } else {
        lowerData[P_LFOWaveform] = value;
        if (wholemode) {
          upperData[P_LFOWaveform] = value;
        }
      }
      updateStratusLFOWaveform(1);
      break;

    case CCupperSW:
      upperSW = true;
      lowerSW = false;
      updateupperSW(1);
      break;

    case CClowerSW:
      lowerSW = true;
      upperSW = false;
      updatelowerSW(1);
      break;

    case CCmodwheel:
      if (upperSW) {
        midiCCOut62(WSmodwheel, value / 8);  // divided by 8 because the convert bumps it up to 1023
      } else {
        midiCCOut61(WSmodwheel, value / 8);
        if (wholemode) {
          midiCCOut62(WSmodwheel, value / 8);
        }
      }
      break;

    case CCallnotesoff:
      allNotesOff();
      break;
  }
}

void myProgramChange(byte channel, byte program) {
  if (inPerformanceMode) {
    if (program < performances.size()) {
      performanceIndex = program;
      currentPerformance = performances[performanceIndex];

      // Update playmode and patch indices
      playMode = currentPerformance.mode;
      wholemode = (playMode == WHOLE);
      updateplayMode(0);

      // Set patch indices
      for (int i = 0; i < patches.size(); i++) {
        if (patches[i].patchNo == currentPerformance.upperPatchNo) upperPatchIndex = i;
        if (patches[i].patchNo == currentPerformance.lowerPatchNo) lowerPatchIndex = i;
      }

      // Recall both patches
      upperSW = true;
      recallPatch(currentPerformance.upperPatchNo);
      upperSW = false;
      recallPatch(currentPerformance.lowerPatchNo);

      refreshPatchDisplayFromState();
    }
  } else {
    // Normal patch recall
    state = PATCH;
    patchNo = program + 1;
    recallPatch(patchNo);
    state = PARAMETER;
  }
}

void myAfterTouch(byte channel, byte value) {

  afterTouch = (value * 1023) / 127;  // Exact scaling, range 1023
  afterTouchU = (afterTouch * upperData[P_ATDepth]) / 1023;
  afterTouchL = (afterTouch * lowerData[P_ATDepth]) / 1023;

  switch (upperData[P_AfterTouchDest]) {
    case 1:
      MIDI6.sendAfterTouch(value, 2);
      break;
    case 2:
      upperData[P_filterCutoff] = (oldfilterCutoffU + afterTouchU);
      if (afterTouchU < 10) {
        upperData[P_filterCutoff] = oldfilterCutoffU;
      }
      if (upperData[P_filterCutoff] > 1023) {
        upperData[P_filterCutoff] = 1023;
      }
      break;
    case 3:
      upperData[P_filterLFO] = afterTouchU;
      break;
    case 4:
      upperData[P_amDepth] = afterTouchU;
      break;
  }
  switch (lowerData[P_AfterTouchDest]) {
    case 1:
      MIDI6.sendAfterTouch(value, 1);
      if (wholemode) {
        MIDI6.sendAfterTouch(value, 2);
      }
      break;
    case 2:
      lowerData[P_filterCutoff] = (oldfilterCutoffL + afterTouchL);
      if (afterTouchL < 10) {
        lowerData[P_filterCutoff] = oldfilterCutoffL;
      }
      if (lowerData[P_filterCutoff] > 1023) {
        lowerData[P_filterCutoff] = 1023;
      }
      break;
    case 3:
      lowerData[P_filterLFO] = afterTouchL;
      break;
    case 4:
      lowerData[P_amDepth] = afterTouchL;
      break;
  }
}

void recallPatch(int patchNo) {
  allNotesOff();

  File patchFile = SD.open(String(patchNo).c_str());
  if (!patchFile) {
    Serial.println("File not found");
  } else {
    String data[NO_OF_PARAMS];
    recallPatchData(patchFile, data);
    patchFile.close();

    // Find matching patch in the circular buffer to set name and number
    for (int i = 0; i < patches.size(); i++) {

      if (patches[i].patchNo == patchNo) {
        if (upperSW) {
          upperPatchIndex = i;
          currentPgmNumU = String(patches[i].patchNo);
          currentPatchNameU = patches[i].patchName;
          //storeLastPatchU(currentPgmNumU)
        } else {
          lowerPatchIndex = i;
          currentPgmNumL = String(patches[i].patchNo);
          currentPatchNameL = patches[i].patchName;
          //storeLastPatchL(currentPgmNumL)
        }

        break;
      }
    }

    setCurrentPatchData(data);
  }
}

void setCurrentPatchData(String data[]) {
  int tempData[75];  // Temporary array for converted integers

  // Convert data from String to int once
  for (int i = 1; i <= 74; i++) {
    tempData[i] = data[i].toInt();
  }

  if (upperSW) {
    patchNameU = data[0];
    tempData[0] = 1;
    memcpy(upperData, tempData, sizeof(tempData));

    oldfilterCutoffU = upperData[P_filterCutoff];
    upperParamsToDisplay();
    setAllButtons();
  } else {
    patchNameL = data[0];
    tempData[0] = 1;
    memcpy(lowerData, tempData, sizeof(tempData));

    oldfilterCutoffL = lowerData[P_filterCutoff];
    lowerParamsToDisplay();
    setAllButtons();

    if (wholemode) {

      // Update previous values and pick-up flags
      for (int i = 1; i <= 74; i++) {
        upperData[i] = lowerData[i];  // Store previous value
      }

      oldfilterCutoffU = upperData[P_filterCutoff];
      upperParamsToDisplay();
      setAllButtons();
    }
  }

  updatePatchname();
}

void upperParamsToDisplay() {

  updateglideTime(0);
  updateosc1PW(0);
  updateosc1PWM(0);
  updateOsc1SawLevel(0);
  updateOsc1PulseLevel(0);
  updateOsc1SubLevel(0);
  updatefmDepth(0);
  updateosc2PW(0);
  updateosc2PWM(0);
  updateOsc2SawLevel(0);
  updateOsc2PulseLevel(0);
  updateOsc2TriangleLevel(0);
  updateosc2Detune(0);
  updateosc2Interval(0);
  updateFilterCutoff(0);
  updatefilterRes(0);
  updatefilterEGlevel(0);
  updatekeytrack(0);
  updatefilterLFO(0);
  updatefilterAttack(0);
  updatefilterDecay(0);
  updatefilterSustain(0);
  updatefilterRelease(0);
  updateampAttack(0);
  updateampDecay(0);
  updateampSustain(0);
  updateampRelease(0);
  updateLFORate(0);
  updateLFODelay(0);
  updatepwLFO(0);
  updateeffectPot1(0);
  updateeffectPot2(0);
  updateeffectPot3(0);
  updateeffectsMix(0);
  updatenoiseLevel(0);
  updatemodWheelDepth(0);
  updatePitchBendDepth(0);
  updatevolumeControl(0);
  updatePM_DCO2(0);
  updatePM_FilterEnv(0);
  updateATDepth(0);
  updateamDepth(0);
  updateosc1Range(0);
  updateosc2Range(0);
  updateFilterType(0);
  updatelfoAlt(0);
  updateStratusLFOWaveform(0);
  updatefilterenvLogLin(0);
  updateampenvLogLin(0);
  updatefilterVel(0);
  updatevcaVel(0);
  updatefilterLoop(0);
  updatevcaLoop(0);
  updatelfoMultiplier(0);
  updateeffectBankSW(0);
  updateeffectNumSW(0);
}

void lowerParamsToDisplay() {

  updateglideTime(0);
  updateosc1PW(0);
  updateosc1PWM(0);
  updateOsc1SawLevel(0);
  updateOsc1PulseLevel(0);
  updateOsc1SubLevel(0);
  updatefmDepth(0);
  updateosc2PW(0);
  updateosc2PWM(0);
  updateOsc2SawLevel(0);
  updateOsc2PulseLevel(0);
  updateOsc2TriangleLevel(0);
  updateosc2Detune(0);
  updateosc2Interval(0);
  updateFilterCutoff(0);
  updatefilterRes(0);
  updatefilterEGlevel(0);
  updatekeytrack(0);
  updatefilterLFO(0);
  updatefilterAttack(0);
  updatefilterDecay(0);
  updatefilterSustain(0);
  updatefilterRelease(0);
  updateampAttack(0);
  updateampDecay(0);
  updateampSustain(0);
  updateampRelease(0);
  updateLFORate(0);
  updateLFODelay(0);
  updatepwLFO(0);
  updateeffectPot1(0);
  updateeffectPot2(0);
  updateeffectPot3(0);
  updateeffectsMix(0);
  updatenoiseLevel(0);
  updatemodWheelDepth(0);
  updatePitchBendDepth(0);
  updatevolumeControl(0);
  updatePM_DCO2(0);
  updatePM_FilterEnv(0);
  updateamDepth(0);
  updateATDepth(0);
  updateosc1Range(0);
  updateosc2Range(0);
  updateFilterType(0);
  updatelfoAlt(0);
  updateStratusLFOWaveform(0);
  updatefilterenvLogLin(0);
  updateampenvLogLin(0);
  updatefilterVel(0);
  updatevcaVel(0);
  updatefilterLoop(0);
  updatevcaLoop(0);
  updatelfoMultiplier(0);
  updateeffectBankSW(0);
  updateeffectNumSW(0);
}

void setAllButtons() {
  updatekeyboardMode(0);
  updateNotePriority(0);
  updateglideSW(0);
  updatesyncSW(0);
  updatefilterPoleSwitch(0);
  updatefilterEGinv(0);
  updatevcaGate(0);
  updatelfoAlt(0);
  updatepmDestDCO1(0);
  updatepmDestFilter(0);
  updatekeyTrackSW(0);
}

String getCurrentPatchData() {
  if (upperSW) {
    return patchNameU + "," + String(upperData[P_pwLFO]) + "," + String(upperData[P_fmDepth]) + "," + String(upperData[P_osc2PW]) + "," + String(upperData[P_osc2PWM])
           + "," + String(upperData[P_osc1PW]) + "," + String(upperData[P_osc1PWM]) + "," + String(upperData[P_osc1Range]) + "," + String(upperData[P_osc2Range]) + "," + String(upperData[P_osc2Interval])
           + "," + String(upperData[P_glideTime]) + "," + String(upperData[P_osc2Detune]) + "," + String(upperData[P_noiseLevel]) + "," + String(upperData[P_osc2SawLevel])
           + "," + String(upperData[P_osc1SawLevel]) + "," + String(upperData[P_osc2PulseLevel]) + "," + String(upperData[P_osc1PulseLevel]) + "," + String(upperData[P_filterCutoff])
           + "," + String(upperData[P_filterLFO]) + "," + String(upperData[P_filterRes]) + "," + String(upperData[P_filterType]) + "," + String(upperData[P_modWheelDepth])
           + "," + String(upperData[P_effectsMix]) + "," + String(upperData[P_LFODelayGo]) + "," + String(upperData[P_filterEGlevel]) + "," + String(upperData[P_LFORate])
           + "," + String(upperData[P_LFOWaveform]) + "," + String(upperData[P_filterAttack]) + "," + String(upperData[P_filterDecay]) + "," + String(upperData[P_filterSustain])
           + "," + String(upperData[P_filterRelease]) + "," + String(upperData[P_ampAttack]) + "," + String(upperData[P_ampDecay]) + "," + String(upperData[P_ampSustain])
           + "," + String(upperData[P_ampRelease]) + "," + String(upperData[P_volumeControl]) + "," + String(upperData[P_glideSW]) + "," + String(upperData[P_keytrack])
           + "," + String(upperData[P_filterPoleSW]) + "," + String(upperData[P_filterLoop]) + "," + String(upperData[P_filterEGinv]) + "," + String(upperData[P_filterVel])
           + "," + String(upperData[P_vcaLoop]) + "," + String(upperData[P_vcaVel]) + "," + String(upperData[P_vcaGate]) + "," + String(upperData[P_lfoAlt]) + "," + String(upperData[P_pmDCO2])
           + "," + String(upperData[P_pmFilterEnv]) + "," + String(upperData[P_monoMulti]) + "," + String(upperData[P_modWheelLevel]) + "," + String(upperData[P_PitchBendLevel])
           + "," + String(upperData[P_amDepth]) + "," + String(upperData[P_sync]) + "," + String(upperData[P_effectPot1]) + "," + String(upperData[P_effectPot2]) + "," + String(upperData[P_effectPot3])
           + "," + String(upperData[P_oldampAttack]) + "," + String(upperData[P_oldampDecay]) + "," + String(upperData[P_oldampSustain]) + "," + String(upperData[P_oldampRelease])
           + "," + String(upperData[P_AfterTouchDest]) + "," + String(upperData[P_filterLogLin]) + "," + String(upperData[P_ampLogLin]) + "," + String(upperData[P_osc2TriangleLevel])
           + "," + String(upperData[P_osc1SubLevel]) + "," + String(upperData[P_keyboardMode]) + "," + String(upperData[P_LFODelay]) + "," + String(upperData[P_effectNum]) + "," + String(upperData[P_effectBank])
           + "," + String(upperData[P_pmDestDCO1]) + "," + String(upperData[P_pmDestFilter]) + "," + String(upperData[P_lfoMultiplier]) + "," + String(upperData[P_NotePriority]) + "," + String(upperData[P_keytrackSW])
           + "," + String(upperData[P_ATDepth]);
  } else {
    return patchNameL + "," + String(lowerData[P_pwLFO]) + "," + String(lowerData[P_fmDepth]) + "," + String(lowerData[P_osc2PW]) + "," + String(lowerData[P_osc2PWM])
           + "," + String(lowerData[P_osc1PW]) + "," + String(lowerData[P_osc1PWM]) + "," + String(lowerData[P_osc1Range]) + "," + String(lowerData[P_osc2Range]) + "," + String(lowerData[P_osc2Interval])
           + "," + String(lowerData[P_glideTime]) + "," + String(lowerData[P_osc2Detune]) + "," + String(lowerData[P_noiseLevel]) + "," + String(lowerData[P_osc2SawLevel])
           + "," + String(lowerData[P_osc1SawLevel]) + "," + String(lowerData[P_osc2PulseLevel]) + "," + String(lowerData[P_osc1PulseLevel]) + "," + String(lowerData[P_filterCutoff])
           + "," + String(lowerData[P_filterLFO]) + "," + String(lowerData[P_filterRes]) + "," + String(lowerData[P_filterType]) + "," + String(lowerData[P_modWheelDepth])
           + "," + String(lowerData[P_effectsMix]) + "," + String(lowerData[P_LFODelayGo]) + "," + String(lowerData[P_filterEGlevel]) + "," + String(lowerData[P_LFORate])
           + "," + String(lowerData[P_LFOWaveform]) + "," + String(lowerData[P_filterAttack]) + "," + String(lowerData[P_filterDecay]) + "," + String(lowerData[P_filterSustain])
           + "," + String(lowerData[P_filterRelease]) + "," + String(lowerData[P_ampAttack]) + "," + String(lowerData[P_ampDecay]) + "," + String(lowerData[P_ampSustain])
           + "," + String(lowerData[P_ampRelease]) + "," + String(lowerData[P_volumeControl]) + "," + String(lowerData[P_glideSW]) + "," + String(lowerData[P_keytrack])
           + "," + String(lowerData[P_filterPoleSW]) + "," + String(lowerData[P_filterLoop]) + "," + String(lowerData[P_filterEGinv]) + "," + String(lowerData[P_filterVel])
           + "," + String(lowerData[P_vcaLoop]) + "," + String(lowerData[P_vcaVel]) + "," + String(lowerData[P_vcaGate]) + "," + String(lowerData[P_lfoAlt]) + "," + String(lowerData[P_pmDCO2])
           + "," + String(lowerData[P_pmFilterEnv]) + "," + String(lowerData[P_monoMulti]) + "," + String(lowerData[P_modWheelLevel]) + "," + String(lowerData[P_PitchBendLevel])
           + "," + String(lowerData[P_amDepth]) + "," + String(lowerData[P_sync]) + "," + String(lowerData[P_effectPot1]) + "," + String(lowerData[P_effectPot2]) + "," + String(lowerData[P_effectPot3])
           + "," + String(lowerData[P_oldampAttack]) + "," + String(lowerData[P_oldampDecay]) + "," + String(lowerData[P_oldampSustain]) + "," + String(lowerData[P_oldampRelease])
           + "," + String(lowerData[P_AfterTouchDest]) + "," + String(lowerData[P_filterLogLin]) + "," + String(lowerData[P_ampLogLin]) + "," + String(lowerData[P_osc2TriangleLevel])
           + "," + String(lowerData[P_osc1SubLevel]) + "," + String(lowerData[P_keyboardMode]) + "," + String(lowerData[P_LFODelay]) + "," + String(lowerData[P_effectNum]) + "," + String(lowerData[P_effectBank])
           + "," + String(lowerData[P_pmDestDCO1]) + "," + String(lowerData[P_pmDestFilter]) + "," + String(lowerData[P_lfoMultiplier]) + "," + String(lowerData[P_NotePriority]) + "," + String(lowerData[P_keytrackSW])
           + "," + String(lowerData[P_ATDepth]);
  }
}

void midiCCOut(byte cc, byte value) {
  MIDI.sendControlChange(cc, value, midiChannel);  //MIDI DIN main out
}

void midiCCOut71(byte cc, byte value) {
  MIDI7.sendControlChange(cc, value, 1);  //MIDI DIN to panel for display bars
}

void midiCCOut72(byte cc, byte value) {
  MIDI7.sendControlChange(cc, value, 2);  //MIDI DIN to panel for switches
}

void midiCCOut61(byte cc, byte value) {
  MIDI6.sendControlChange(cc, value, 1);  //MIDI DIN to synth board lower
}

void midiCCOut62(byte cc, byte value) {
  MIDI6.sendControlChange(cc, value, 2);  //MIDI DIN to synth board upper
}

void outputDAC(int CHIP_SELECT, uint32_t sample_data1, uint32_t sample_data2, uint32_t sample_data3, uint32_t sample_data4) {
  SPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE1));
  digitalWriteFast(CHIP_SELECT, LOW);
  SPI.transfer32(sample_data1);
  digitalWriteFast(CHIP_SELECT, HIGH);
  digitalWriteFast(CHIP_SELECT, LOW);
  SPI.transfer32(sample_data2);
  digitalWriteFast(CHIP_SELECT, HIGH);
  digitalWriteFast(CHIP_SELECT, LOW);
  SPI.transfer32(sample_data3);
  digitalWriteFast(CHIP_SELECT, HIGH);
  digitalWriteFast(CHIP_SELECT, LOW);
  SPI.transfer32(sample_data4);
  digitalWriteFast(CHIP_SELECT, HIGH);
  SPI.endTransaction();
  delayMicroseconds(2);
}

void writeDemux() {

  switch (muxOutput) {

    case 0:
      sample_data1 = (channel_a & 0xFFF0000F) | (((int(upperData[P_noiseLevel] * MULT2V)) & 0xFFFF) << 4);
      sample_data2 = (channel_c & 0xFFF0000F) | (((int(lowerData[P_noiseLevel] * MULT2V)) & 0xFFFF) << 4);

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(upperData[P_filterAttack] * MULT5V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(lowerData[P_filterAttack] * MULT5V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 1:
      sample_data1 = (channel_a & 0xFFF0000F) | (((int(upperData[P_osc1SawLevel] * MULT1_2V)) & 0xFFFF) << 4);
      sample_data2 = (channel_c & 0xFFF0000F) | (((int(lowerData[P_osc1SawLevel] * MULT1_2V)) & 0xFFFF) << 4);

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(upperData[P_filterDecay] * MULT5V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(lowerData[P_filterDecay] * MULT5V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 2:
      sample_data1 = (channel_a & 0xFFF0000F) | (((int(upperData[P_osc1PulseLevel] * MULT1_2V)) & 0xFFFF) << 4);
      sample_data2 = (channel_c & 0xFFF0000F) | (((int(lowerData[P_osc1PulseLevel] * MULT1_2V)) & 0xFFFF) << 4);

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(upperData[P_filterSustain] * MULT5V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(lowerData[P_filterSustain] * MULT5V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 3:
      sample_data1 = (channel_a & 0xFFF0000F) | (((int(upperData[P_osc1SubLevel] * MULT1_2V)) & 0xFFFF) << 4);
      sample_data2 = (channel_c & 0xFFF0000F) | (((int(lowerData[P_osc1SubLevel] * MULT1_2V)) & 0xFFFF) << 4);

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(upperData[P_filterRelease] * MULT5V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(lowerData[P_filterRelease] * MULT5V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 4:
      sample_data1 = (channel_a & 0xFFF0000F) | (((int(upperData[P_pmDCO2] * MULT2V)) & 0xFFFF) << 4);
      sample_data2 = (channel_c & 0xFFF0000F) | (((int(lowerData[P_pmDCO2] * MULT2V)) & 0xFFFF) << 4);

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(upperData[P_ampAttack] * MULT5V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(lowerData[P_ampAttack] * MULT5V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 5:
      sample_data1 = (channel_a & 0xFFF0000F) | (((int(upperData[P_pmFilterEnv] * MULT2V)) & 0xFFFF) << 4);
      sample_data2 = (channel_c & 0xFFF0000F) | (((int(lowerData[P_pmFilterEnv] * MULT2V)) & 0xFFFF) << 4);

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(upperData[P_ampDecay] * MULT5V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(lowerData[P_ampDecay] * MULT5V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 6:
      sample_data1 = (channel_a & 0xFFF0000F) | (((int(upperData[P_osc2SawLevel] * MULT1_2V)) & 0xFFFF) << 4);
      sample_data2 = (channel_c & 0xFFF0000F) | (((int(lowerData[P_osc2SawLevel] * MULT1_2V)) & 0xFFFF) << 4);

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(upperData[P_ampSustain] * MULT5V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(lowerData[P_ampSustain] * MULT5V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 7:
      sample_data1 = (channel_a & 0xFFF0000F) | (((int(upperData[P_osc2PulseLevel] * MULT1_2V)) & 0xFFFF) << 4);
      sample_data2 = (channel_c & 0xFFF0000F) | (((int(lowerData[P_osc2PulseLevel] * MULT1_2V)) & 0xFFFF) << 4);

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(upperData[P_ampRelease] * MULT5V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(lowerData[P_ampRelease] * MULT5V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 8:
      sample_data1 = (channel_a & 0xFFF0000F) | (((int(upperData[P_osc2TriangleLevel] * MULT1_2V)) & 0xFFFF) << 4);
      sample_data2 = (channel_c & 0xFFF0000F) | (((int(lowerData[P_osc2TriangleLevel] * MULT1_2V)) & 0xFFFF) << 4);

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(upperData[P_filterEGlevel] * MULT5V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(lowerData[P_filterEGlevel] * MULT5V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 9:
      sample_data1 = (channel_a & 0xFFF0000F) | (((int(upperData[P_volumeControl] * MULT2V)) & 0xFFFF) << 4);
      sample_data2 = (channel_c & 0xFFF0000F) | (((int(lowerData[P_volumeControl] * MULT2V)) & 0xFFFF) << 4);

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(upperData[P_filterCutoff] * MULT5V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(lowerData[P_filterCutoff] * MULT5V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 10:
      scaled = int(upperData[P_effectsMix] * MULT3V);
      if (scaled > CLAMP2V) scaled = CLAMP2V;
      sample_data1 = (channel_a & 0xFFF0000F) | ((scaled & 0xFFFF) << 4);
      scaled = int(lowerData[P_effectsMix] * MULT3V);
      if (scaled > CLAMP2V) scaled = CLAMP2V;
      sample_data2 = (channel_c & 0xFFF0000F) | ((scaled & 0xFFFF) << 4);

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(upperData[P_filterRes] * MULT2V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(lowerData[P_filterRes] * MULT2V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 11:
      switch (upperData[P_LFODelayGo]) {
        case 1:
          //sample_data1 = (channel_a & 0xFFF0000F) | (((int(upperData[P_fmDepth] * MULT2V)) & 0xFFFF) << 4);
          sample_data1 = (channel_a & 0xFFF0000F) | (((int(1023 * MULT2V)) & 0xFFFF) << 4);
          break;

        case 0:
          sample_data1 = (channel_a & 0xFFF0000F) | ((0 & 0xFFFF) << 4);
          break;
      }
      switch (lowerData[P_LFODelayGo]) {
        case 1:
          //sample_data2 = (channel_c & 0xFFF0000F) | (((int(lowerData[P_fmDepth] * MULT2V)) & 0xFFFF) << 4);
          sample_data2 = (channel_c & 0xFFF0000F) | (((int(1023 * MULT2V)) & 0xFFFF) << 4);
          break;

        case 0:
          sample_data2 = (channel_c & 0xFFF0000F) | ((0 & 0xFFFF) << 4);
          break;
      }

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(upperData[P_LFORate] * MULT5V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(lowerData[P_LFORate] * MULT5V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 12:
      switch (upperData[P_LFODelayGo]) {
        case 1:
          sample_data1 = (channel_a & 0xFFF0000F) | (((int(upperData[P_filterLFO] * MULT2V)) & 0xFFFF) << 4);
          break;

        case 0:
          sample_data1 = (channel_a & 0xFFF0000F) | ((0 & 0xFFFF) << 4);
          break;
      }
      switch (lowerData[P_LFODelayGo]) {
        case 1:
          sample_data2 = (channel_c & 0xFFF0000F) | (((int(lowerData[P_filterLFO] * MULT2V)) & 0xFFFF) << 4);
          break;

        case 0:
          sample_data2 = (channel_c & 0xFFF0000F) | ((0 & 0xFFFF) << 4);
          break;
      }

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(LFOWaveCVupper * MULT5V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(LFOWaveCVlower * MULT5V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 13:
      switch (upperData[P_LFODelayGo]) {
        case 1:
          sample_data1 = (channel_a & 0xFFF0000F) | (((int(upperData[P_amDepth] * MULT2V)) & 0xFFFF) << 4);
          break;

        case 0:
          sample_data1 = (channel_a & 0xFFF0000F) | ((0 & 0xFFFF) << 4);
          break;
      }
      switch (lowerData[P_LFODelayGo]) {
        case 1:
          sample_data2 = (channel_c & 0xFFF0000F) | (((int(lowerData[P_amDepth] * MULT2V)) & 0xFFFF) << 4);
          break;

        case 0:
          sample_data2 = (channel_c & 0xFFF0000F) | ((0 & 0xFFFF) << 4);
          break;
      }

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(upperData[P_effectPot1] * MULT33V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(lowerData[P_effectPot1] * MULT33V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 14:
      sample_data1 = (channel_a & 0xFFF0000F) | ((0 & 0xFFFF) << 4);
      sample_data2 = (channel_c & 0xFFF0000F) | ((0 & 0xFFFF) << 4);

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(upperData[P_effectPot2] * MULT33V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(lowerData[P_effectPot2] * MULT33V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 15:
      sample_data1 = (channel_a & 0xFFF0000F) | (((upperData[P_pwLFO] * MULT5V) & 0xFFFF) << 4);
      sample_data2 = (channel_c & 0xFFF0000F) | (((lowerData[P_pwLFO] * MULT5V) & 0xFFFF) << 4);

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(upperData[P_effectPot3] * MULT33V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(lowerData[P_effectPot3] * MULT33V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;
  }
  delayMicroseconds(10);
  digitalWriteFast(DEMUX_EN_1, HIGH);
  //delayMicroseconds(100);

  muxOutput++;
  if (muxOutput >= DEMUXCHANNELS)

    muxOutput = 0;

  digitalWriteFast(DEMUX_0, muxOutput & B0001);
  digitalWriteFast(DEMUX_1, muxOutput & B0010);
  digitalWriteFast(DEMUX_2, muxOutput & B0100);
  digitalWriteFast(DEMUX_3, muxOutput & B1000);
}

void showSettingsPage() {
  showSettingsPage(settings::current_setting(), settings::current_setting_value(), state);
}

void showPerformancePage(String perfNum, String name, int upperNo, String upperName, int lowerNo, String lowerName) {
  currentPerfNum = perfNum;
  currentPerfName = name;
  currentUpperPatchNo = upperNo;
  currentUpperPatchName = upperName;
  currentLowerPatchNo = lowerNo;
  currentLowerPatchName = lowerName;
}

void reinitialiseToPanel() {
  if (upperSW) {
    for (int i = 1; i < 77; i++) {
      upperData[i] = 0;
    }
    upperData[P_osc1SawLevel] = 127;
    upperData[P_osc2SawLevel] = 127;
    upperData[P_osc2Detune] = 8;
    upperData[P_filterCutoff] = 127;
    upperData[P_ampSustain] = 127;
    upperData[P_volumeControl] = 127;
    upperData[P_noiseLevel] = 63;
    upperData[P_osc1PW] = 63;
    upperData[P_osc2PW] = 63;
    upperParamsToDisplay();
    setAllButtons();
  } else {
    for (int i = 1; i < 77; i++) {
      lowerData[i] = 0;
    }
    lowerData[P_osc1SawLevel] = 127;
    lowerData[P_osc2SawLevel] = 127;
    lowerData[P_osc2Detune] = 8;
    lowerData[P_filterCutoff] = 127;
    lowerData[P_ampSustain] = 127;
    lowerData[P_volumeControl] = 127;
    lowerData[P_noiseLevel] = 63;
    lowerData[P_osc1PW] = 63;
    lowerData[P_osc2PW] = 63;
    lowerParamsToDisplay();
    setAllButtons();
    if (wholemode) {
      for (int i = 1; i < 77; i++) {
        upperData[i] = 0;
      }
      upperData[P_osc1SawLevel] = 127;
      upperData[P_osc2SawLevel] = 127;
      upperData[P_osc2Detune] = 8;
      upperData[P_filterCutoff] = 127;
      upperData[P_ampSustain] = 127;
      upperData[P_volumeControl] = 127;
      upperData[P_noiseLevel] = 63;
      upperData[P_osc1PW] = 63;
      upperData[P_osc2PW] = 63;
      upperParamsToDisplay();
      setAllButtons();
    }
  }
}

void deletePerformance(int perfNo) {
  char filename[32];
  snprintf(filename, sizeof(filename), "/performances/perf%03d", perfNo);
  if (SD.exists(filename)) {
    SD.remove(filename);
    Serial.print("[DELETE] Removed performance: ");
    Serial.println(filename);
  }
}

void renumberPerformancesOnSD() {
  char filename[32];
  for (int i = 0; i < performances.size(); i++) {
    Performance p = performances[i];
    p.performanceNo = i + 1;
    performances[i] = p;

    snprintf(filename, sizeof(filename), "/performances/perf%03d", p.performanceNo);
    savePerformance(filename, p);
  }
}

void checkSwitches() {
  button.update(digitalRead(TUNE_BUTTON), 50, LOW);
  if (button.held()) {
    midiCCOut61(WSresetAutotune, 127);
    midiCCOut62(WSresetAutotune, 127);
    showCurrentParameterPage("Autotune", String("Reset"));
  } else if (button.released(true)) {
    midiCCOut61(WSautotune, 127);
    midiCCOut62(WSautotune, 127);
    showCurrentParameterPage("Autotune", String("Started"));
  }

  saveButton.update();
  if (saveButton.held()) {
    if (inPerformanceMode && (state == PARAMETER || state == PATCH)) {
      state = PERFORMANCE_DELETE;
    } else if (state == PARAMETER || state == PATCH) {
      state = DELETE;
    }
  } else if (saveButton.numClicks() == 1) {
    switch (state) {
      case SAVE:
        {
          if (renamedPatch.length() == 0) {
            renamedPatch = INITPATCHNAME;  // fallback if no rename occurred
          }

          // Update patch name depending on upper or lower
          if (upperSW) {
            patchNameU = renamedPatch;
            currentPatchNameU = renamedPatch;
            currentPgmNumU = String(patches.last().patchNo);
          } else {
            patchNameL = renamedPatch;
            currentPatchNameL = renamedPatch;
            currentPgmNumL = String(patches.last().patchNo);
          }

          // ✅ Update last patch in the buffer before saving
          patches.last().patchName = renamedPatch;

          // ✅ Save updated patch data
          String patchData = getCurrentPatchData();
          savePatch(String(patches.last().patchNo).c_str(), patchData);

          // ✅ Reload and reorder patches explicitly
          loadPatches();
          setPatchesOrdering(patches.last().patchNo);

          // ✅ Correctly update patch index for immediate display
          for (int i = 0; i < patches.size(); i++) {
            if (patches[i].patchNo == patches.last().patchNo) {
              if (upperSW) upperPatchIndex = i;
              else lowerPatchIndex = i;
              break;
            }
          }

          // ✅ Immediately refresh display with updated data
          refreshPatchDisplayFromState();

          renamedPatch = "";
          state = PARAMETER;
        }
        break;


      case PATCHNAMING:
        {
          //Serial.println("renamedPatch BEFORE SAVING: " + renamedPatch);

          if (renamedPatch.length() == 0) {
            renamedPatch = patches.last().patchName;  // fallback to existing name
          }

          // Update correct upper/lower patch name based on current layer
          if (upperSW) {
            patchNameU = renamedPatch;
            currentPatchNameU = renamedPatch;  // Update immediately
            currentPgmNumU = String(patches.last().patchNo);
          } else {
            patchNameL = renamedPatch;
            currentPatchNameL = renamedPatch;  // Update immediately
            currentPgmNumL = String(patches.last().patchNo);
          }

          // Update last patch in the patches buffer
          patches.last().patchName = renamedPatch;

          // Save patch data (with the correct name included)
          String patchData = getCurrentPatchData();
          savePatch(String(patches.last().patchNo).c_str(), patchData);

          loadPatches();                   // Refresh patches list from SD card
          refreshPatchDisplayFromState();  // immediately update the display
          setPatchesOrdering(patches.last().patchNo);

          renamedPatch = "";
          state = PARAMETER;
        }
        break;


      case PARAMETER:
        if (inPerformanceMode) {
          if (performances.size() < PERFORMANCES_LIMIT) {
            int newPerfNo = performances.size() + 1;
            Performance newPerf = {
              newPerfNo,
              patches[upperPatchIndex].patchNo,
              patches[lowerPatchIndex].patchNo,
              INITPATCHNAME,
              (PlayMode)playMode
            };
            currentPerformance = newPerf;
            performances.push(newPerf);
            performanceIndex = performances.size() - 1;

            showPerformancePage(
              String(newPerf.performanceNo),
              newPerf.name,
              newPerf.upperPatchNo,
              getPatchName(newPerf.upperPatchNo),
              newPerf.lowerPatchNo,
              getPatchName(newPerf.lowerPatchNo));

            state = PERFORMANCE_SAVE;
          }
        } else {
          // 🛠 PATCH SAVE FLOW
          if (patches.size() < PATCHES_LIMIT) {
            resetPatchesOrdering();  // start from patch 1
            patches.push({ patches.size() + 1, INITPATCHNAME });
            state = SAVE;
          }
        }
        break;

      case PERFORMANCE_SAVE:
        currentPerformance = performances[performanceIndex];
        state = PERFORMANCE_NAMING;
        renamedPatch = currentPerformance.name;
        charIndex = 0;
        currentCharacter = CHARACTERS[charIndex];
        startedRenaming = false;
        showRenamingPage(renamedPatch);
        break;

      case PERFORMANCE_NAMING:
        if (saveButton.numClicks() == 1) {
          if (renamedPatch.length() > 0) {
            currentPerformance.name = renamedPatch;
          }

          upperSW = true;
          savePatch(String(currentPerformance.upperPatchNo).c_str(), getCurrentPatchData());

          upperSW = false;
          savePatch(String(currentPerformance.lowerPatchNo).c_str(), getCurrentPatchData());

          upperSW = true;

          // Update full performance data
          currentPerformance.upperPatchNo = patches[upperPatchIndex].patchNo;
          currentPerformance.lowerPatchNo = patches[lowerPatchIndex].patchNo;
          currentPerformance.mode = (PlayMode)playMode;

          for (int i = 0; i < performances.size(); i++) {
            if (performances[i].performanceNo == currentPerformance.performanceNo) {
              performances[i] = currentPerformance;
              break;
            }
          }

          char filename[16];
          snprintf(filename, sizeof(filename), "perf%03d", currentPerformance.performanceNo);

          savePerformance(filename, currentPerformance);
          loadPerformances();

          renamedPatch = "";
          charIndex = 0;
          currentCharacter = CHARACTERS[0];
          startedRenaming = false;
          state = PARAMETER;
        } else if (recallButton.numClicks() == 1) {
          if (renamedPatch.length() < 12) {
            renamedPatch.concat(String(currentCharacter));
            charIndex = 0;
            currentCharacter = CHARACTERS[charIndex];
            showRenamingPage(renamedPatch);
          }
        } else if (backButton.numClicks() == 1) {
          renamedPatch = "";
          charIndex = 0;
          startedRenaming = false;
          state = PARAMETER;
          if (performances.size() > 0 && performances.last().name == INITPATCHNAME) {
            performances.pop();
          }
        }
        break;
    }
  }

  settingsButton.update();
  if (settingsButton.held()) {
    //If recall held, set current patch to match current hardware state
    //Reinitialise all hardware values to force them to be re-read if different
    state = REINITIALISE;
    reinitialiseToPanel();
  } else if (settingsButton.numClicks() == 1) {
    switch (state) {
      case PARAMETER:
        state = SETTINGS;
        showSettingsPage();
        break;
      case SETTINGS:
        showSettingsPage();
      case SETTINGSVALUE:
        settings::save_current_value();
        state = SETTINGS;
        showSettingsPage();
        break;
    }
  }

  backButton.update();
  if (backButton.held()) {
    //If Back button held, Panic - all notes off
  } else if (backButton.numClicks() == 1) {
    switch (state) {
      case RECALL:
        setPatchesOrdering(patchNo);
        state = PARAMETER;
        break;
      case SAVE:
        renamedPatch = "";
        state = PARAMETER;
        loadPatches();  //Remove patch that was to be saved
        setPatchesOrdering(patchNo);
        break;
      case PATCHNAMING:
        charIndex = 0;
        renamedPatch = "";
        state = SAVE;
        break;
      case DELETE:
        setPatchesOrdering(patchNo);
        state = PARAMETER;
        break;
      case SETTINGS:
        state = PARAMETER;
        break;
      case SETTINGSVALUE:
        state = SETTINGS;
        showSettingsPage();
        break;
      case PERFORMANCE_NAMING:
        renamedPatch = "";
        charIndex = 0;
        state = PARAMETER;
        // Optionally remove the unsaved performance from the buffer:
        if (performances.size() > 0 && performances.last().name == INITPATCHNAME) {
          performances.pop();
        }
        break;
      case PERFORMANCE_DELETE:
        setPerformancesOrdering(currentPerformance.performanceNo);
        state = PARAMETER;
        break;
    }
  }

  // Encoder switch
  recallButton.update();
  if (recallButton.held()) {
    if (!recallHeldToggleLatch) {
      inPerformanceMode = !inPerformanceMode;
      recallHeldToggleLatch = true;

      //Serial.print("[MODE] Switched to ");
      //Serial.println(inPerformanceMode ? "Performance Mode" : "Patch Mode");

      showCurrentParameterPage("Mode", inPerformanceMode ? "Performance" : "Patch");

      if (inPerformanceMode && performances.size() > 0) {
        // Entering Performance Mode
        performanceIndex = 0;
        currentPerformance = performances[performanceIndex];

        showPerformancePage(
          String(currentPerformance.performanceNo),
          currentPerformance.name,
          currentPerformance.upperPatchNo,
          getPatchName(currentPerformance.upperPatchNo),
          currentPerformance.lowerPatchNo,
          getPatchName(currentPerformance.lowerPatchNo));

      } else {
        // Returning to Patch Mode
        refreshPatchDisplayFromState();
      }
    }
  } else {
    recallHeldToggleLatch = false;
  }
  if (recallButton.numClicks() == 1) {
    switch (state) {
      case RECALL:
        //Serial.println("[INFO] Ignored default RECALL to avoid overwriting performance recall.");
        state = PARAMETER;
        break;
      case SAVE:
        showRenamingPage(patches.last().patchName);
        patchName = patches.last().patchName;
        state = PATCHNAMING;
        break;
      case PATCHNAMING:
        if (renamedPatch.length() < 12)  //actually 12 chars
        {
          renamedPatch.concat(String(currentCharacter));
          charIndex = 0;
          currentCharacter = CHARACTERS[charIndex];
          showRenamingPage(renamedPatch);
        }
        break;
      case DELETE:
        //Don't delete final patch
        if (patches.size() > 1) {
          state = DELETEMSG;
          patchNo = patches.first().patchNo;     //PatchNo to delete from SD card
          patches.shift();                       //Remove patch from circular buffer
          deletePatch(String(patchNo).c_str());  //Delete from SD card
          loadPatches();                         //Repopulate circular buffer to start from lowest Patch No
          renumberPatchesOnSD();
          loadPatches();                      //Repopulate circular buffer again after delete
          patchNo = patches.first().patchNo;  //Go back to 1
          recallPatch(patchNo);               //Load first patch
        }
        state = PARAMETER;
        break;
      case SETTINGS:
        state = SETTINGSVALUE;
        showSettingsPage();
        break;
      case SETTINGSVALUE:
        settings::save_current_value();
        state = SETTINGS;
        showSettingsPage();
        break;

      case PARAMETER:
        // Enter performance recall
        if (performances.size() > 0) {
          currentPerformance = performances.first();
          showPerformancePage(
            String(currentPerformance.performanceNo),
            currentPerformance.name,
            currentPerformance.upperPatchNo,
            getPatchName(currentPerformance.upperPatchNo),
            currentPerformance.lowerPatchNo,
            getPatchName(currentPerformance.lowerPatchNo));
          state = PERFORMANCE_RECALL;
        }
        break;

      case PERFORMANCE_RECALL:
        for (int i = 0; i < patches.size(); i++) {
          if (patches[i].patchNo == currentPerformance.upperPatchNo) {
            upperPatchIndex = i;
          }
          if (patches[i].patchNo == currentPerformance.lowerPatchNo) {
            lowerPatchIndex = i;
          }
        }

        playMode = currentPerformance.mode;
        wholemode = (playMode == WHOLE);
        updateplayMode(0);

        upperSW = true;
        recallPatch(currentPerformance.upperPatchNo);

        upperSW = false;
        recallPatch(currentPerformance.lowerPatchNo);

        refreshPatchDisplayFromState();

        state = PARAMETER;
        patchNo = 0;  // ✅ Clear global patchNo to avoid accidental reuse
        return;

      case PERFORMANCE_NAMING:
        if (renamedPatch.length() < 12) {
          renamedPatch.concat(String(currentCharacter));
          charIndex = 0;
          currentCharacter = CHARACTERS[charIndex];
          showRenamingPage(renamedPatch);
        }
        break;

      case PERFORMANCE_DELETE:
        if (performances.size() > 0) {
          state = PERFORMANCE_DELETEMSG;

          int deletedNo = performances.first().performanceNo;
          performances.shift();          // Remove from buffer
          deletePerformance(deletedNo);  // Delete file
          loadPerformances();            // Refresh buffer
          renumberPerformancesOnSD();    // Reorder files
          loadPerformances();            // Reload to apply new order

          currentPerformance = performances.first();
          recallPerformance(currentPerformance);
        }
        state = PARAMETER;
        return;


      case PERFORMANCE_DELETEMSG:
        // Show deletion complete screen briefly
        tft.fillScreen(ST7735_BLACK);
        tft.setFont(&FreeSans12pt7b);
        tft.setTextColor(ST7735_YELLOW);
        tft.setCursor(10, 60);
        tft.println("Renumbering");
        tft.setCursor(10, 100);
        tft.println("Performances...");
        tft.updateScreen();
        delay(1000);
        state = PARAMETER;
        break;
    }
  }
}

// Updated checkEncoder() with upperPatchIndex and lowerPatchIndex
void checkEncoder() {
  long encRead = encoder.read();
  bool moved = false;

  if ((encCW && encRead > encPrevious + 3) || (!encCW && encRead < encPrevious - 3)) {
    moved = true;

    switch (state) {

      case PERFORMANCE_DELETE:
        if (encCW) {
          performances.push(performances.shift());
        } else {
          performances.unshift(performances.pop());
        }
        break;

      case PERFORMANCE_SAVE:
        performanceIndex++;
        if (performanceIndex >= performances.size()) performanceIndex = 0;
        currentPerformance = performances[performanceIndex];
        showPerformancePage(
          String(currentPerformance.performanceNo),
          currentPerformance.name,
          currentPerformance.upperPatchNo,
          getPatchName(currentPerformance.upperPatchNo),
          currentPerformance.lowerPatchNo,
          getPatchName(currentPerformance.lowerPatchNo));
        break;

      case PERFORMANCE_RECALL:
        performanceIndex++;
        if (performanceIndex >= performances.size()) performanceIndex = 0;
        currentPerformance = performances[performanceIndex];
        showPerformancePage(
          String(currentPerformance.performanceNo),
          currentPerformance.name,
          currentPerformance.upperPatchNo,
          getPatchName(currentPerformance.upperPatchNo),
          currentPerformance.lowerPatchNo,
          getPatchName(currentPerformance.lowerPatchNo));
        break;

      case PERFORMANCE_NAMING:
        if (!startedRenaming) {
          renamedPatch = "";
          startedRenaming = true;
        }

        charIndex++;
        if (charIndex >= TOTALCHARS) charIndex = 0;
        currentCharacter = CHARACTERS[charIndex];
        showRenamingPage(renamedPatch + currentCharacter);
        break;

      case PARAMETER:
        if (inPerformanceMode) {
          performanceIndex++;
          if (performanceIndex >= performances.size()) performanceIndex = 0;
          currentPerformance = performances[performanceIndex];

          for (int i = 0; i < patches.size(); i++) {
            if (patches[i].patchNo == currentPerformance.upperPatchNo) upperPatchIndex = i;
            if (patches[i].patchNo == currentPerformance.lowerPatchNo) lowerPatchIndex = i;
          }

          playMode = currentPerformance.mode;
          wholemode = (playMode == WHOLE);
          updateplayMode(0);

          upperSW = true;
          recallPatch(currentPerformance.upperPatchNo);
          upperSW = false;
          recallPatch(currentPerformance.lowerPatchNo);
        } else {
          if (upperSW) {
            upperPatchIndex++;
            if (upperPatchIndex >= patches.size()) upperPatchIndex = 0;
            patchNo = patches[upperPatchIndex].patchNo;
            recallPatch(patchNo);
          } else {
            lowerPatchIndex++;
            if (lowerPatchIndex >= patches.size()) lowerPatchIndex = 0;
            patchNo = patches[lowerPatchIndex].patchNo;
            recallPatch(patchNo);
          }
        }
        refreshPatchDisplayFromState();
        break;

      case RECALL:
      case SAVE:
      case DELETE:
        patches.push(patches.shift());
        break;

      case PATCHNAMING:
        if (charIndex == TOTALCHARS) charIndex = 0;
        currentCharacter = CHARACTERS[charIndex++];
        showRenamingPage(renamedPatch + currentCharacter);
        break;

      case SETTINGS:
        settings::increment_setting();
        showSettingsPage();
        break;

      case SETTINGSVALUE:
        settings::increment_setting_value();
        showSettingsPage();
        break;
    }
  } else if ((encCW && encRead < encPrevious - 3) || (!encCW && encRead > encPrevious + 3)) {
    moved = true;

    switch (state) {

      case PERFORMANCE_DELETE:
        if (encCW) {
          performances.push(performances.shift());
        } else {
          performances.unshift(performances.pop());
        }
        break;

      case PERFORMANCE_SAVE:
        performanceIndex--;
        if (performanceIndex < 0) performanceIndex = performances.size() - 1;
        currentPerformance = performances[performanceIndex];
        showPerformancePage(
          String(currentPerformance.performanceNo),
          currentPerformance.name,
          currentPerformance.upperPatchNo,
          getPatchName(currentPerformance.upperPatchNo),
          currentPerformance.lowerPatchNo,
          getPatchName(currentPerformance.lowerPatchNo));
        break;

      case PERFORMANCE_RECALL:
        performanceIndex--;
        if (performanceIndex < 0) performanceIndex = performances.size() - 1;
        currentPerformance = performances[performanceIndex];
        showPerformancePage(
          String(currentPerformance.performanceNo),
          currentPerformance.name,
          currentPerformance.upperPatchNo,
          getPatchName(currentPerformance.upperPatchNo),
          currentPerformance.lowerPatchNo,
          getPatchName(currentPerformance.lowerPatchNo));
        break;

      case PERFORMANCE_NAMING:
        if (!startedRenaming) {
          renamedPatch = "";
          startedRenaming = true;
        }

        charIndex--;
        if (charIndex < 0) charIndex = TOTALCHARS - 1;
        currentCharacter = CHARACTERS[charIndex];
        showRenamingPage(renamedPatch + currentCharacter);
        break;

      case PARAMETER:
        if (inPerformanceMode) {
          performanceIndex--;
          if (performanceIndex < 0) performanceIndex = performances.size() - 1;
          currentPerformance = performances[performanceIndex];

          for (int i = 0; i < patches.size(); i++) {
            if (patches[i].patchNo == currentPerformance.upperPatchNo) upperPatchIndex = i;
            if (patches[i].patchNo == currentPerformance.lowerPatchNo) lowerPatchIndex = i;
          }

          playMode = currentPerformance.mode;
          wholemode = (playMode == WHOLE);
          updateplayMode(0);

          upperSW = true;
          recallPatch(currentPerformance.upperPatchNo);
          upperSW = false;
          recallPatch(currentPerformance.lowerPatchNo);
        } else {
          if (upperSW) {
            upperPatchIndex--;
            if (upperPatchIndex < 0) upperPatchIndex = patches.size() - 1;
            patchNo = patches[upperPatchIndex].patchNo;
            recallPatch(patchNo);
          } else {
            lowerPatchIndex--;
            if (lowerPatchIndex < 0) lowerPatchIndex = patches.size() - 1;
            patchNo = patches[lowerPatchIndex].patchNo;
            recallPatch(patchNo);
          }
        }
        refreshPatchDisplayFromState();
        break;


      case RECALL:
      case SAVE:
      case DELETE:
        patches.unshift(patches.pop());
        break;

      case PATCHNAMING:
        if (charIndex == -1) charIndex = TOTALCHARS - 1;
        currentCharacter = CHARACTERS[charIndex--];
        showRenamingPage(renamedPatch + currentCharacter);
        break;

      case SETTINGS:
        settings::decrement_setting();
        showSettingsPage();
        break;

      case SETTINGSVALUE:
        settings::decrement_setting_value();
        showSettingsPage();
        break;
    }
  }

  if (moved) {
    encPrevious = encRead;
  }
}

String getPatchName(int patchNo) {
  for (int i = 0; i < patches.size(); i++) {
    if (patches[i].patchNo == patchNo) return patches[i].patchName;
  }
  return "-";
}

void setPerformancesOrdering(int no) {
  if (performances.size() < 2) return;
  while (performances.first().performanceNo != no) {
    performances.push(performances.shift());
  }
}

void onButtonPress(uint16_t btnIndex, uint8_t btnType) {

  if (btnIndex == GLIDE_SW && btnType == ROX_PRESSED) {
    panelData[P_glideSW] = !panelData[P_glideSW];
    myControlChange(midiChannel, CCglideSW, panelData[P_glideSW]);
  }

  if (btnIndex == FILTER_POLE_SW && btnType == ROX_PRESSED) {
    panelData[P_filterPoleSW] = !panelData[P_filterPoleSW];
    myControlChange(midiChannel, CCfilterPoleSW, panelData[P_filterPoleSW]);
  }

  if (btnIndex == EG_INVERT_SW && btnType == ROX_PRESSED) {
    panelData[P_filterEGinv] = !panelData[P_filterEGinv];
    myControlChange(midiChannel, CCfilterEGinv, panelData[P_filterEGinv]);
  }

  if (btnIndex == DCO1_OCT_SW && btnType == ROX_PRESSED) {
    panelData[P_osc1Range] = panelData[P_osc1Range] + 1;
    if (panelData[P_osc1Range] > 2) {
      panelData[P_osc1Range] = 0;
    }
    myControlChange(midiChannel, CCosc1Oct, panelData[P_osc1Range]);
  }

  if (btnIndex == DCO2_OCT_SW && btnType == ROX_PRESSED) {
    panelData[P_osc2Range] = panelData[P_osc2Range] + 1;
    if (panelData[P_osc2Range] > 2) {
      panelData[P_osc2Range] = 0;
    }
    myControlChange(midiChannel, CCosc2Oct, panelData[P_osc2Range]);
  }

  if (btnIndex == FILTER_TYPE_SW && btnType == ROX_PRESSED) {
    panelData[P_filterType] = panelData[P_filterType] + 1;
    if (panelData[P_filterType] > 7) {
      panelData[P_filterType] = 0;
    }
    myControlChange(midiChannel, CCfilterType, panelData[P_filterType]);
  }

  if (btnIndex == LFO_ALT_SW && btnType == ROX_PRESSED) {
    panelData[P_lfoAlt] = !panelData[P_lfoAlt];
    myControlChange(midiChannel, CClfoAlt, panelData[P_lfoAlt]);
  }

  if (btnIndex == LFO_MULT_SW && btnType == ROX_PRESSED) {
    panelData[P_lfoMultiplier] = panelData[P_lfoMultiplier] + 1;
    if (panelData[P_lfoMultiplier] > 4) {
      panelData[P_lfoMultiplier] = 0;
    }
    myControlChange(midiChannel, CClfoMult, panelData[P_lfoMultiplier]);
  }

  if (btnIndex == LFO_WAVEFORM_SW && btnType == ROX_PRESSED) {
    panelData[P_LFOWaveform] = panelData[P_LFOWaveform] + 1;
    if (panelData[P_LFOWaveform] > 7) {
      panelData[P_LFOWaveform] = 0;
    }
    myControlChange(midiChannel, CCLFOWaveform, panelData[P_LFOWaveform]);
  }

  if (btnIndex == FILTER_ENV_VELOCITY_SW && btnType == ROX_PRESSED) {
    panelData[P_filterVel] = !panelData[P_filterVel];
    myControlChange(midiChannel, CCfilterVel, panelData[P_filterVel]);
  }

  if (btnIndex == AMP_ENV_VELOCITY_SW && btnType == ROX_PRESSED) {
    panelData[P_vcaVel] = !panelData[P_vcaVel];
    myControlChange(midiChannel, CCvcaVel, panelData[P_vcaVel]);
  }

  if (btnIndex == FILTER_ENV_LOOP_SW && btnType == ROX_PRESSED) {
    panelData[P_filterLoop] = panelData[P_filterLoop] + 1;
    if (panelData[P_filterLoop] > 2) {
      panelData[P_filterLoop] = 0;
    }
    myControlChange(midiChannel, CCFilterLoop, panelData[P_filterLoop]);
  }

  if (btnIndex == AMP_ENV_LOOP_SW && btnType == ROX_PRESSED) {
    panelData[P_vcaLoop] = panelData[P_vcaLoop] + 1;
    if (panelData[P_vcaLoop] > 2) {
      panelData[P_vcaLoop] = 0;
    }
    myControlChange(midiChannel, CCAmpLoop, panelData[P_vcaLoop]);
  }

  if (btnIndex == AMP_GATED_SW && btnType == ROX_PRESSED) {
    panelData[P_vcaGate] = !panelData[P_vcaGate];
    myControlChange(midiChannel, CCvcaGate, panelData[P_vcaGate]);
  }

  if (btnIndex == EFFECT_NUMBER_SW && btnType == ROX_PRESSED) {
    panelData[P_effectNum] = panelData[P_effectNum] + 1;
    if (panelData[P_effectNum] > 7) {
      panelData[P_effectNum] = 0;
    }
    myControlChange(midiChannel, CCeffectNumSW, panelData[P_effectNum]);
  }

  if (btnIndex == EFFECT_BANK_SW && btnType == ROX_PRESSED) {
    panelData[P_effectBank] = panelData[P_effectBank] + 1;
    if (panelData[P_effectBank] > 3) {
      panelData[P_effectBank] = 0;
    }
    myControlChange(midiChannel, CCeffectBankSW, panelData[P_effectBank]);
  }

  if (btnIndex == FILTER_ENV_LIN_LOG_SW && btnType == ROX_PRESSED) {
    panelData[P_filterLogLin] = !panelData[P_filterLogLin];
    myControlChange(midiChannel, CCfilterenvLinLogSW, panelData[P_filterLogLin]);
  }

  if (btnIndex == AMP_ENV_LIN_LOG_SW && btnType == ROX_PRESSED) {
    panelData[P_ampLogLin] = !panelData[P_ampLogLin];
    myControlChange(midiChannel, CCampenvLinLogSW, panelData[P_ampLogLin]);
  }

  if (btnIndex == POLY1_SW && btnType == ROX_PRESSED) {
    panelData[P_keyboardMode] = 0;
    myControlChange(midiChannel, CCkeyboardMode, panelData[P_keyboardMode]);
  }

  if (btnIndex == POLY2_SW && btnType == ROX_PRESSED) {
    panelData[P_keyboardMode] = 1;
    myControlChange(midiChannel, CCkeyboardMode, panelData[P_keyboardMode]);
  }

  if (btnIndex == UNISON_SW && btnType == ROX_PRESSED) {
    panelData[P_keyboardMode] = 2;
    myControlChange(midiChannel, CCkeyboardMode, panelData[P_keyboardMode]);
  }

  if (btnIndex == MONO_SW && btnType == ROX_PRESSED) {
    panelData[P_keyboardMode] = 3;
    myControlChange(midiChannel, CCkeyboardMode, panelData[P_keyboardMode]);
  }

  if (btnIndex == KEYBOARD_SW && btnType == ROX_PRESSED) {
    playMode = playMode + 1;
    if (playMode > 2) {
      playMode = 0;
    }
    myControlChange(midiChannel, CCplayMode, playMode);
  }

  if (btnIndex == PRIORITY_SW && btnType == ROX_PRESSED) {
    panelData[P_NotePriority] = panelData[P_NotePriority] + 1;
    if (panelData[P_NotePriority] > 2) {
      panelData[P_NotePriority] = 0;
    }
    myControlChange(midiChannel, CCNotePriority, panelData[P_NotePriority]);
  }

  if (btnIndex == LFO_MULTI_MONO_SW && btnType == ROX_PRESSED) {
    panelData[P_monoMulti] = !panelData[P_monoMulti];
    myControlChange(midiChannel, CCmonoMulti, panelData[P_monoMulti]);
  }

  if (btnIndex == CHORD_HOLD_SW && btnType == ROX_PRESSED) {
    chordHoldSW = !chordHoldSW;
    myControlChange(midiChannel, CCchordHoldSW, chordHoldSW);
  }

  if (btnIndex == SYNC_SW && btnType == ROX_PRESSED) {
    panelData[P_sync] = !panelData[P_sync];
    myControlChange(midiChannel, CCsyncSW, panelData[P_sync]);
  }

  if (btnIndex == KEYTRACK_SW && btnType == ROX_PRESSED) {
    panelData[P_keytrackSW] = !panelData[P_keytrackSW];
    myControlChange(midiChannel, CCkeyTrackSW, panelData[P_keytrackSW]);
  }

  if (btnIndex == LOWER_SW && btnType == ROX_PRESSED) {
    lowerSW = true;
    upperSW = false;
    myControlChange(midiChannel, CClowerSW, lowerSW);
  }

  if (btnIndex == UPPER_SW && btnType == ROX_PRESSED) {
    lowerSW = false;
    upperSW = true;
    myControlChange(midiChannel, CCupperSW, upperSW);
  }

  if (btnIndex == PM_DCO1_DEST_SW && btnType == ROX_PRESSED) {
    panelData[P_pmDestDCO1] = !panelData[P_pmDestDCO1];
    myControlChange(midiChannel, CCpmDestDCO1SW, panelData[P_pmDestDCO1]);
  }

  if (btnIndex == PM_FILT_ENV_DEST_SW && btnType == ROX_PRESSED) {
    panelData[P_pmDestFilter] = !panelData[P_pmDestFilter];
    myControlChange(midiChannel, CCpmDestFilterSW, panelData[P_pmDestFilter]);
  }
}

void loop() {

  if (digitalRead(AUTOTUNE_INPUT) == HIGH) {
    isAutotuning = true;
    digitalWrite(TUNE_LED, HIGH);

    while (digitalRead(AUTOTUNE_INPUT) == HIGH) {
      while (midi1.read()) {}
      while (MIDI.read()) {}
      while (MIDI6.read()) {}
      while (MIDI7.read()) {}
      while (usbMIDI.read()) {}
      writeDemux();
      delay(1);
    }

    digitalWrite(TUNE_LED, LOW);
    isAutotuning = false;

    while (midi1.read()) {}
    while (MIDI.read()) {}
    while (MIDI6.read()) {}
    while (MIDI7.read()) {}
    while (usbMIDI.read()) {}

    allNotesOff();
  } else {

    checkSwitches();
    writeDemux();
    pollAllMCPs();
    checkEncoder();
    midi1.read(midiChannel);  //USB HOST MIDI Class Compliant
    MIDI.read(midiChannel);
    MIDI6.read(midiChannel);
    MIDI7.read();
    usbMIDI.read(midiChannel);
    octoswitch.update();  // read all the buttons for the Synth
    srp.update();         // update all the LEDs in the buttons
    LFODelayHandle();
    changeSpeed();
  }
}
