/*
Version 1.3
*/

#include <Arduino.h>
#include <i2c_driver.h>
#include <i2c_driver_wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <MIDI.h>
#include <USBHost_t36.h>
#include "MidiCC.h"
#include "Constants.h"
#include "Parameters.h"
#include "PatchMgr.h"
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

#include "ST7735Display.h"

boolean cardStatus = false;

struct VoiceAndNote {
  int note;
  int velocity;
  long timeOn;
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

boolean voiceOn[NO_OF_VOICES] = { false, false, false, false, false, false, false, false };
int prevNote = 0;  //Initialised to middle value
bool notes[128] = { 0 }, initial_loop = 1;
int8_t noteOrder[40] = { 0 }, orderIndx = { 0 };

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
#define PIN_DATA 17  // pin 9 on 74HC165 (DATA)
#define PIN_CLK 41   // pin 2 on 74HC165 (CLK))
#define PIN_LOAD 40  // pin 1 on 74HC165 (LOAD)

int count = 0;  //For MIDI Clk Sync
int DelayForSH3 = 50;
int midioutfrig = 3;
int patchNo = 0;
int patchNoU = 0;
int patchNoL = 0;
int voiceToReturn = -1;        //Initialise
long earliestTime = millis();  //For voice allocation - initialise to now
unsigned long buttonDebounce = 0;

// create a global shift register object
// parameters: <number of shift registers> (data pin, clock pin, latch pin)

void setup() {
  SPI.begin();
  Wire.begin();           // Join the I2C bus as Master
  Wire.setClock(400000);  // Set I2C speed to 400 kHz
  setupDisplay();
  setUpSettings();
  setupHardware();

  octoswitch.begin(PIN_DATA, PIN_LOAD, PIN_CLK);
  octoswitch.setCallback(onButtonPress);

  srp.begin(LED_DATA, LED_LATCH, LED_CLK, LED_PWM);

  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));
  digitalWrite(DAC_CS1, LOW);
  delayMicroseconds(1);
  SPI.transfer32(int_ref_on_flexible_mode);
  digitalWrite(DAC_CS1, HIGH);
  SPI.endTransaction();


  cardStatus = SD.begin(BUILTIN_SDCARD);
  if (cardStatus) {
    //Serial.println("SD card is connected");
    //Get patch numbers and names from SD card
    loadPatches();
    if (patches.size() == 0) {
      //save an initialised patch to SD card
      savePatch("1", INITPATCH);
      loadPatches();
    }
  } else {
    //Serial.println("SD card is not connected or unusable");
    reinitialiseToPanel();
    showPatchPage("No SD", "conn'd / usable", "", "");
  }

  //Read MIDI Channel from EEPROM
  midiChannel = getMIDIChannel();
  //Serial.println("MIDI Ch:" + String(midiChannel) + " (0 is Omni On)");

  //USB Client MIDI
  usbMIDI.setHandleControlChange(editControlChange);
  usbMIDI.setHandleProgramChange(myProgramChange);
  usbMIDI.setHandleAfterTouchChannel(myAfterTouch);
  usbMIDI.setHandlePitchChange(DinHandlePitchBend);
  usbMIDI.setHandleNoteOn(myNoteOn);
  usbMIDI.setHandleNoteOff(myNoteOff);
  //Serial.println("USB Client MIDI Listening");

  //MIDI 5 Pin DIN
  MIDI.begin();
  MIDI.setHandleControlChange(editControlChange);
  MIDI.setHandleProgramChange(myProgramChange);
  MIDI.setHandleAfterTouchChannel(myAfterTouch);
  MIDI.setHandlePitchBend(DinHandlePitchBend);
  MIDI.setHandleNoteOn(myNoteOn);
  MIDI.setHandleNoteOff(myNoteOff);
  MIDI.turnThruOn(midi::Thru::Mode::Off);
  //Serial.println("MIDI In DIN Listening");

  MIDI7.begin();
  MIDI7.setHandleControlChange(panelControlChange);
  MIDI7.turnThruOn(midi::Thru::Mode::Off);

  MIDI6.begin();
  MIDI6.turnThruOn(midi::Thru::Mode::Off);

  //Read Aftertouch from EEPROM, this can be set individually by each patch.
  upperData[P_AfterTouchDest] = getAfterTouchU();
  oldAfterTouchDestU = upperData[P_AfterTouchDest];
  lowerData[P_AfterTouchDest] = getAfterTouchL();
  oldAfterTouchDestL = lowerData[P_AfterTouchDest];

  newsplitPoint = getSplitPoint();

  splitTrans = getSplitTrans();
  setTranspose(splitTrans);

  //Read Encoder Direction from EEPROM
  encCW = getEncoderDir();

  //setupDisplay();
  delay(1000);


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

  // patchNoU = getLastPatchU();
  // patchNoL = getLastPatchL();
  patchNoU = 1;
  patchNoL = 1;
  upperSW = 1;
  lowerSW = 0;
  recallPatch(patchNoU);
  upperSW = 0;
  lowerSW = 0;
  recallPatch(patchNoL);  //Load first patch
  //updatewholemode();
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

void commandTopNote() {
  int topNote = 0;
  bool noteActive = false;

  for (int i = 0; i < 128; i++) {
    if (notes[i]) {
      topNote = i;
      noteActive = true;
    }
  }

  if (noteActive)
    commandNote(topNote);
  else  // All notes are off, turn off gate
    MIDI6.sendNoteOff(noteMsg, 0, 1);
}

void commandBottomNote() {
  int bottomNote = 0;
  bool noteActive = false;

  for (int i = 127; i >= 0; i--) {
    if (notes[i]) {
      bottomNote = i;
      noteActive = true;
    }
  }

  if (noteActive)
    commandNote(bottomNote);
  else  // All notes are off, turn off gate
    MIDI6.sendNoteOff(noteMsg, 0, 1);
}

void commandLastNote() {

  int8_t noteIndx;

  for (int i = 0; i < 40; i++) {
    noteIndx = noteOrder[mod(orderIndx - i, 40)];
    if (notes[noteIndx]) {
      commandNote(noteIndx);
      return;
    }
  }
  MIDI6.sendNoteOff(noteMsg, 0, 1);
}

void commandNote(int noteMsg) {
  MIDI6.sendNoteOn(noteMsg, noteVel, 1);
}

void commandTopNoteUni() {
  int topNote = 0;
  bool noteActive = false;

  for (int i = 0; i < 128; i++) {
    if (notes[i]) {
      topNote = i;
      noteActive = true;
    }
  }

  if (noteActive) {
    commandNoteUni(topNote);
  } else {  // All notes are off, turn off gate
    MIDI6.sendNoteOff(noteMsg, 0, 1);
    MIDI6.sendNoteOff(noteMsg, 0, 2);
    MIDI6.sendNoteOff(noteMsg, 0, 3);
    MIDI6.sendNoteOff(noteMsg, 0, 4);
    MIDI6.sendNoteOff(noteMsg, 0, 5);
    MIDI6.sendNoteOff(noteMsg, 0, 6);
    MIDI6.sendNoteOff(noteMsg, 0, 7);
    MIDI6.sendNoteOff(noteMsg, 0, 8);
  }
}

void commandBottomNoteUni() {
  int bottomNote = 0;
  bool noteActive = false;

  for (int i = 127; i >= 0; i--) {
    if (notes[i]) {
      bottomNote = i;
      noteActive = true;
    }
  }

  if (noteActive) {
    commandNoteUni(bottomNote);
  } else {  // All notes are off, turn off gate
    MIDI6.sendNoteOff(noteMsg, 0, 1);
    MIDI6.sendNoteOff(noteMsg, 0, 2);
    MIDI6.sendNoteOff(noteMsg, 0, 3);
    MIDI6.sendNoteOff(noteMsg, 0, 4);
    MIDI6.sendNoteOff(noteMsg, 0, 5);
    MIDI6.sendNoteOff(noteMsg, 0, 6);
    MIDI6.sendNoteOff(noteMsg, 0, 7);
    MIDI6.sendNoteOff(noteMsg, 0, 8);
  }
}

void commandLastNoteUni() {

  int8_t noteIndx;

  for (int i = 0; i < 40; i++) {
    noteIndx = noteOrder[mod(orderIndx - i, 40)];
    if (notes[noteIndx]) {
      commandNoteUni(noteIndx);
      return;
    }
  }
  MIDI6.sendNoteOff(noteMsg, 0, 1);
  MIDI6.sendNoteOff(noteIndx, 0, 2);
  MIDI6.sendNoteOff(noteIndx, 0, 3);
  MIDI6.sendNoteOff(noteIndx, 0, 4);
  MIDI6.sendNoteOff(noteIndx, 0, 5);
  MIDI6.sendNoteOff(noteIndx, 0, 6);
  MIDI6.sendNoteOff(noteIndx, 0, 7);
  MIDI6.sendNoteOff(noteIndx, 0, 8);
}

void commandNoteUni(int noteMsg) {

  MIDI6.sendNoteOn(noteMsg, noteVel, 1);
  MIDI6.sendNoteOn(noteMsg, noteVel, 2);
  MIDI6.sendNoteOn(noteMsg, noteVel, 3);
  MIDI6.sendNoteOn(noteMsg, noteVel, 4);
  MIDI6.sendNoteOn(noteMsg, noteVel, 5);
  MIDI6.sendNoteOn(noteMsg, noteVel, 6);
  MIDI6.sendNoteOn(noteMsg, noteVel, 7);
  MIDI6.sendNoteOn(noteMsg, noteVel, 8);
}

void myNoteOn(byte channel, byte note, byte velocity) {

  // for lfo multi trigger
  numberOfNotes = numberOfNotes + 1;

  //Check for out of range notes
  if (note >= 0 && note <= 127) {
    voiceAssignment[note] = getVoiceNoPoly2(note) - 1;
  }

  prevNote = note;
  switch (panelData[P_keyboardMode]) {
    case 0:
      switch (getVoiceNo(-1)) {
        case 1:
          voices[0].note = note;
          voices[0].velocity = velocity;
          voices[0].timeOn = millis();
          updateVoice1();
          MIDI6.sendNoteOn(voices[0].note, voices[0].velocity, 1);
          //trig.writePin(GATE_NOTE1, HIGH);
          voiceOn[0] = true;
          break;
        case 2:
          voices[1].note = note;
          voices[1].velocity = velocity;
          voices[1].timeOn = millis();
          updateVoice2();
          MIDI6.sendNoteOn(voices[1].note, voices[1].velocity, 2);
          //trig.writePin(GATE_NOTE2, HIGH);
          voiceOn[1] = true;
          break;
        case 3:
          voices[2].note = note;
          voices[2].velocity = velocity;
          voices[2].timeOn = millis();
          updateVoice3();
          MIDI6.sendNoteOn(voices[2].note, voices[2].velocity, 3);
          //trig.writePin(GATE_NOTE3, HIGH);
          voiceOn[2] = true;
          break;
        case 4:
          voices[3].note = note;
          voices[3].velocity = velocity;
          voices[3].timeOn = millis();
          updateVoice4();
          MIDI6.sendNoteOn(voices[3].note, voices[3].velocity, 4);
          //trig.writePin(GATE_NOTE4, HIGH);
          voiceOn[3] = true;
          break;
        case 5:
          voices[4].note = note;
          voices[4].velocity = velocity;
          voices[4].timeOn = millis();
          updateVoice5();
          MIDI6.sendNoteOn(voices[4].note, voices[4].velocity, 5);
          //trig.writePin(GATE_NOTE5, HIGH);
          voiceOn[4] = true;
          break;
        case 6:
          voices[5].note = note;
          voices[5].velocity = velocity;
          voices[5].timeOn = millis();
          updateVoice6();
          MIDI6.sendNoteOn(voices[5].note, voices[5].velocity, 6);
          //trig.writePin(GATE_NOTE6, HIGH);
          voiceOn[5] = true;
          break;
        case 7:
          voices[6].note = note;
          voices[6].velocity = velocity;
          voices[6].timeOn = millis();
          updateVoice7();
          MIDI6.sendNoteOn(voices[6].note, voices[6].velocity, 7);
          //trig.writePin(GATE_NOTE7, HIGH);
          voiceOn[6] = true;
          break;
        case 8:
          voices[7].note = note;
          voices[7].velocity = velocity;
          voices[7].timeOn = millis();
          updateVoice8();
          MIDI6.sendNoteOn(voices[7].note, voices[7].velocity, 8);
          //trig.writePin(GATE_NOTE8, HIGH);
          voiceOn[7] = true;
          break;
      }
      break;

    case 1:
      switch (getVoiceNoPoly2(-1)) {
        case 1:
          voices[0].note = note;
          voices[0].velocity = velocity;
          voices[0].timeOn = millis();
          updateVoice1();
          MIDI6.sendNoteOn(voices[0].note, voices[0].velocity, 1);
          //trig.writePin(GATE_NOTE1, HIGH);
          voiceOn[0] = true;
          break;
        case 2:
          voices[1].note = note;
          voices[1].velocity = velocity;
          voices[1].timeOn = millis();
          updateVoice2();
          MIDI6.sendNoteOn(voices[1].note, voices[1].velocity, 2);
          //trig.writePin(GATE_NOTE2, HIGH);
          voiceOn[1] = true;
          break;
        case 3:
          voices[2].note = note;
          voices[2].velocity = velocity;
          voices[2].timeOn = millis();
          updateVoice3();
          MIDI6.sendNoteOn(voices[2].note, voices[2].velocity, 3);
          //trig.writePin(GATE_NOTE3, HIGH);
          voiceOn[2] = true;
          break;
        case 4:
          voices[3].note = note;
          voices[3].velocity = velocity;
          voices[3].timeOn = millis();
          updateVoice4();
          MIDI6.sendNoteOn(voices[3].note, voices[3].velocity, 4);
          //trig.writePin(GATE_NOTE4, HIGH);
          voiceOn[3] = true;
          break;
        case 5:
          voices[4].note = note;
          voices[4].velocity = velocity;
          voices[4].timeOn = millis();
          updateVoice5();
          MIDI6.sendNoteOn(voices[4].note, voices[4].velocity, 5);
          //trig.writePin(GATE_NOTE5, HIGH);
          voiceOn[4] = true;
          break;
        case 6:
          voices[5].note = note;
          voices[5].velocity = velocity;
          voices[5].timeOn = millis();
          updateVoice6();
          MIDI6.sendNoteOn(voices[5].note, voices[5].velocity, 6);
          //trig.writePin(GATE_NOTE6, HIGH);
          voiceOn[5] = true;
          break;
        case 7:
          voices[6].note = note;
          voices[6].velocity = velocity;
          voices[6].timeOn = millis();
          updateVoice7();
          MIDI6.sendNoteOn(voices[6].note, voices[6].velocity, 7);
          //trig.writePin(GATE_NOTE7, HIGH);
          voiceOn[6] = true;
          break;
        case 8:
          voices[7].note = note;
          voices[7].velocity = velocity;
          voices[7].timeOn = millis();
          updateVoice8();
          MIDI6.sendNoteOn(voices[7].note, voices[7].velocity, 8);
          //trig.writePin(GATE_NOTE8, HIGH);
          voiceOn[7] = true;
          break;
      }
      break;

    case 2:
      noteMsg = note;
      noteVel = velocity;

      if (velocity == 0) {
        notes[noteMsg] = false;
      } else {
        notes[noteMsg] = true;
      }

      if (panelData[P_NotePriority] == 0) {  // Highest note priority
        commandTopNote();
      } else if (panelData[P_NotePriority] == 1) {  // Lowest note priority
        commandBottomNote();
      } else {                 // Last note priority
        if (notes[noteMsg]) {  // If note is on and using last note priority, add to ordered list
          orderIndx = (orderIndx + 1) % 40;
          noteOrder[orderIndx] = noteMsg;
        }
        commandLastNote();
      }
      break;

    case 3:
      noteMsg = note;
      noteVel = velocity;

      if (velocity == 0) {
        notes[noteMsg] = false;
      } else {
        notes[noteMsg] = true;
      }

      if (panelData[P_NotePriority] == 0) {  // Highest note priority
        commandTopNoteUni();
      } else if (panelData[P_NotePriority] == 1) {  // Lowest note priority
        commandBottomNoteUni();
      } else {                 // Last note priority
        if (notes[noteMsg]) {  // If note is on and using last note priority, add to ordered list
          orderIndx = (orderIndx + 1) % 40;
          noteOrder[orderIndx] = noteMsg;
        }
        commandLastNoteUni();
      }
      break;
  }
}

void myNoteOff(byte channel, byte note, byte velocity) {

  numberOfNotes = numberOfNotes - 1;
  oldnumberOfNotes = oldnumberOfNotes - 1;

  switch (panelData[P_keyboardMode]) {
    case 0:
      switch (getVoiceNo(note)) {
        case 1:
          MIDI6.sendNoteOn(voices[0].note, 0, 1);
          voices[0].note = -1;
          voiceOn[0] = false;
          break;
        case 2:
          MIDI6.sendNoteOn(voices[1].note, 0, 2);
          voices[1].note = -1;
          voiceOn[1] = false;
          break;
        case 3:
          MIDI6.sendNoteOn(voices[2].note, 0, 3);
          voices[2].note = -1;
          voiceOn[2] = false;
          break;
        case 4:
          MIDI6.sendNoteOn(voices[3].note, 0, 4);
          voices[3].note = -1;
          voiceOn[3] = false;
          break;
        case 5:
          MIDI6.sendNoteOn(voices[4].note, 0, 5);
          voices[4].note = -1;
          voiceOn[4] = false;
          break;
        case 6:
          MIDI6.sendNoteOn(voices[5].note, 0, 6);
          voices[5].note = -1;
          voiceOn[5] = false;
          break;
        case 7:
          MIDI6.sendNoteOn(voices[6].note, 0, 7);
          voices[6].note = -1;
          voiceOn[6] = false;
          break;
        case 8:
          MIDI6.sendNoteOn(voices[7].note, 0, 8);
          voices[7].note = -1;
          voiceOn[7] = false;
          break;
      }
      break;

    case 1:
      switch (getVoiceNoPoly2(note)) {
        case 1:
          MIDI6.sendNoteOn(voices[0].note, 0, 1);
          voices[0].note = -1;
          voiceOn[0] = false;
          break;
        case 2:
          MIDI6.sendNoteOn(voices[1].note, 0, 2);
          voices[1].note = -1;
          voiceOn[1] = false;
          break;
        case 3:
          MIDI6.sendNoteOn(voices[2].note, 0, 3);
          voices[2].note = -1;
          voiceOn[2] = false;
          break;
        case 4:
          MIDI6.sendNoteOn(voices[3].note, 0, 4);
          voices[3].note = -1;
          voiceOn[3] = false;
          break;
        case 5:
          MIDI6.sendNoteOn(voices[4].note, 0, 5);
          voices[4].note = -1;
          voiceOn[4] = false;
          break;
        case 6:
          MIDI6.sendNoteOn(voices[5].note, 0, 6);
          voices[5].note = -1;
          voiceOn[5] = false;
          break;
        case 7:
          MIDI6.sendNoteOn(voices[6].note, 0, 7);
          voices[6].note = -1;
          voiceOn[6] = false;
          break;
        case 8:
          MIDI6.sendNoteOn(voices[7].note, 0, 8);
          voices[7].note = -1;
          voiceOn[7] = false;
          break;
      }
      break;

    case 2:
      noteMsg = note;

      if (velocity == 0 || velocity == 64) {
        notes[noteMsg] = false;
      } else {
        notes[noteMsg] = true;
      }

      // Pins NP_SEL1 and NP_SEL2 indictate note priority
      // velmV = ((unsigned int)((float)velocity) * VEL_SF);
      // sample_data = (channel_a & 0xFFF0000F) | (((int(velmV)) & 0xFFFF) << 4);
      // outputDAC(DAC_NOTE2, sample_data);
      if (panelData[P_NotePriority] == 0) {  // Highest note priority
        commandTopNote();
      } else if (panelData[P_NotePriority] == 1) {  // Lowest note priority
        commandBottomNote();
      } else {                 // Last note priority
        if (notes[noteMsg]) {  // If note is on and using last note priority, add to ordered list
          orderIndx = (orderIndx + 1) % 40;
          noteOrder[orderIndx] = noteMsg;
        }
        commandLastNote();
      }
      break;

    case 3:
      noteMsg = note;

      if (velocity == 0 || velocity == 64) {
        notes[noteMsg] = false;
      } else {
        notes[noteMsg] = true;
      }

      // velmV = ((unsigned int)((float)velocity) * VEL_SF);
      // sample_data = (channel_a & 0xFFF0000F) | (((int(velmV)) & 0xFFFF) << 4);
      // outputDAC(DAC_NOTE2, sample_data);
      // sample_data = (channel_b & 0xFFF0000F) | (((int(velmV)) & 0xFFFF) << 4);
      // outputDAC(DAC_NOTE2, sample_data);
      // sample_data = (channel_c & 0xFFF0000F) | (((int(velmV)) & 0xFFFF) << 4);
      // outputDAC(DAC_NOTE2, sample_data);
      // sample_data = (channel_d & 0xFFF0000F) | (((int(velmV)) & 0xFFFF) << 4);
      // outputDAC(DAC_NOTE2, sample_data);
      // sample_data = (channel_e & 0xFFF0000F) | (((int(velmV)) & 0xFFFF) << 4);
      // outputDAC(DAC_NOTE2, sample_data);
      // sample_data = (channel_f & 0xFFF0000F) | (((int(velmV)) & 0xFFFF) << 4);
      // outputDAC(DAC_NOTE2, sample_data);
      // sample_data = (channel_g & 0xFFF0000F) | (((int(velmV)) & 0xFFFF) << 4);
      // outputDAC(DAC_NOTE2, sample_data);
      // sample_data = (channel_h & 0xFFF0000F) | (((int(velmV)) & 0xFFFF) << 4);
      // outputDAC(DAC_NOTE2, sample_data);
      if (panelData[P_NotePriority] == 0) {  // Highest note priority
        commandTopNoteUni();
      } else if (panelData[P_NotePriority] == 1) {  // Lowest note priority
        commandBottomNoteUni();
      } else {                 // Last note priority
        if (notes[noteMsg]) {  // If note is on and using last note priority, add to ordered list
          orderIndx = (orderIndx + 1) % 40;
          noteOrder[orderIndx] = noteMsg;
        }
        commandLastNoteUni();
      }
      break;
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

void updateVoice1() {
  // unsigned int mV = (unsigned int)((float)(voices[0].note + realoctave) * NOTE_SF * sfAdj[0] + 0.5);
  // mV = mV * keyTrackMult;
  // sample_data = (channel_a & 0xFFF0000F) | (((int(mV)) & 0xFFFF) << 4);
  // outputDAC(DAC_NOTE1, sample_data);
  // velmV = ((unsigned int)((float)voices[0].velocity) * VEL_SF);
  // sample_data = (channel_a & 0xFFF0000F) | (((int(velmV)) & 0xFFFF) << 4);
  // outputDAC(DAC_NOTE2, sample_data);
}

void updateVoice2() {
  // unsigned int mV = (unsigned int)((float)(voices[1].note + realoctave) * NOTE_SF * sfAdj[1] + 0.5);
  // mV = mV * keyTrackMult;
  // sample_data = (channel_b & 0xFFF0000F) | (((int(mV)) & 0xFFFF) << 4);
  // outputDAC(DAC_NOTE1, sample_data);
  // velmV = ((unsigned int)((float)voices[1].velocity) * VEL_SF);
  // sample_data = (channel_b & 0xFFF0000F) | (((int(velmV)) & 0xFFFF) << 4);
  // outputDAC(DAC_NOTE2, sample_data);
}

void updateVoice3() {
  // unsigned int mV = (unsigned int)((float)(voices[2].note + realoctave) * NOTE_SF * sfAdj[2] + 0.5);
  // mV = mV * keyTrackMult;
  // sample_data = (channel_c & 0xFFF0000F) | (((int(mV)) & 0xFFFF) << 4);
  // outputDAC(DAC_NOTE1, sample_data);
  // velmV = ((unsigned int)((float)voices[2].velocity) * VEL_SF);
  // sample_data = (channel_c & 0xFFF0000F) | (((int(velmV)) & 0xFFFF) << 4);
  // outputDAC(DAC_NOTE2, sample_data);
}

void updateVoice4() {
  // unsigned int mV = (unsigned int)((float)(voices[3].note + realoctave) * NOTE_SF * sfAdj[3] + 0.5);
  // mV = mV * keyTrackMult;
  // sample_data = (channel_d & 0xFFF0000F) | (((int(mV)) & 0xFFFF) << 4);
  // outputDAC(DAC_NOTE1, sample_data);
  // velmV = ((unsigned int)((float)voices[3].velocity) * VEL_SF);
  // sample_data = (channel_d & 0xFFF0000F) | (((int(velmV)) & 0xFFFF) << 4);
  // outputDAC(DAC_NOTE2, sample_data);
}

void updateVoice5() {
  // unsigned int mV = (unsigned int)((float)(voices[4].note + realoctave) * NOTE_SF * sfAdj[4] + 0.5);
  // mV = mV * keyTrackMult;
  // sample_data = (channel_e & 0xFFF0000F) | (((int(mV)) & 0xFFFF) << 4);
  // outputDAC(DAC_NOTE1, sample_data);
  // velmV = ((unsigned int)((float)voices[4].velocity) * VEL_SF);
  // sample_data = (channel_e & 0xFFF0000F) | (((int(velmV)) & 0xFFFF) << 4);
  // outputDAC(DAC_NOTE2, sample_data);
}

void updateVoice6() {
  // unsigned int mV = (unsigned int)((float)(voices[5].note + realoctave) * NOTE_SF * sfAdj[5] + 0.5);
  // mV = mV * keyTrackMult;
  // sample_data = (channel_f & 0xFFF0000F) | (((int(mV)) & 0xFFFF) << 4);
  // outputDAC(DAC_NOTE1, sample_data);
  // velmV = ((unsigned int)((float)voices[5].velocity) * VEL_SF);
  // sample_data = (channel_f & 0xFFF0000F) | (((int(velmV)) & 0xFFFF) << 4);
  // outputDAC(DAC_NOTE2, sample_data);
}

void updateVoice7() {
  // unsigned int mV = (unsigned int)((float)(voices[6].note + realoctave) * NOTE_SF * sfAdj[6] + 0.5);
  // mV = mV * keyTrackMult;
  // sample_data = (channel_g & 0xFFF0000F) | (((int(mV)) & 0xFFFF) << 4);
  // outputDAC(DAC_NOTE1, sample_data);
  // velmV = ((unsigned int)((float)voices[6].velocity) * VEL_SF);
  // sample_data = (channel_g & 0xFFF0000F) | (((int(velmV)) & 0xFFFF) << 4);
  // outputDAC(DAC_NOTE2, sample_data);
}

void updateVoice8() {
  // unsigned int mV = (unsigned int)((float)(voices[7].note + realoctave) * NOTE_SF * sfAdj[7] + 0.5);
  // mV = mV * keyTrackMult;
  // sample_data = (channel_h & 0xFFF0000F) | (((int(mV)) & 0xFFFF) << 4);
  // outputDAC(DAC_NOTE1, sample_data);
  // velmV = ((unsigned int)((float)voices[7].velocity) * VEL_SF);
  // sample_data = (channel_h & 0xFFF0000F) | (((int(velmV)) & 0xFFFF) << 4);
  // outputDAC(DAC_NOTE2, sample_data);
}


// void DinHandleNoteOn(byte channel, byte note, byte velocity) {
//   numberOfNotesU = numberOfNotesU + 1;
//   numberOfNotesL = numberOfNotesL + 1;

//   if (wholemode) {
//     if (note < 0 || note > 127) return;
//     switch (getVoiceNo(-1)) {
//       case 1:
//         voices[0].note = note;
//         voices[0].velocity = velocity;
//         voices[0].timeOn = millis();
//         MIDI.sendNoteOn(note, velocity, 1);
//         voiceOn[0] = true;
//         break;
//       case 2:
//         voices[1].note = note;
//         voices[1].velocity = velocity;
//         voices[1].timeOn = millis();
//         MIDI.sendNoteOn(note, velocity, 2);
//         voiceOn[1] = true;
//         break;
//       case 3:
//         voices[2].note = note;
//         voices[2].velocity = velocity;
//         voices[2].timeOn = millis();
//         MIDI.sendNoteOn(note, velocity, 1);
//         voiceOn[2] = true;
//         break;
//       case 4:
//         voices[3].note = note;
//         voices[3].velocity = velocity;
//         voices[3].timeOn = millis();
//         MIDI.sendNoteOn(note, velocity, 2);
//         voiceOn[3] = true;
//         break;
//       case 5:
//         voices[4].note = note;
//         voices[4].velocity = velocity;
//         voices[4].timeOn = millis();
//         MIDI.sendNoteOn(note, velocity, 1);
//         voiceOn[4] = true;
//         break;
//       case 6:
//         voices[5].note = note;
//         voices[5].velocity = velocity;
//         voices[5].timeOn = millis();
//         MIDI.sendNoteOn(note, velocity, 2);
//         voiceOn[5] = true;
//         break;
//       case 7:
//         voices[6].note = note;
//         voices[6].velocity = velocity;
//         voices[6].timeOn = millis();
//         MIDI.sendNoteOn(note, velocity, 1);
//         voiceOn[6] = true;
//         break;
//       case 8:
//         voices[7].note = note;
//         voices[7].velocity = velocity;
//         voices[7].timeOn = millis();
//         MIDI.sendNoteOn(note, velocity, 2);
//         voiceOn[7] = true;
//         break;
//     }
//   }
//   if (dualmode) {
//     MIDI.sendNoteOn(note, velocity, 1);
//     MIDI.sendNoteOn(note, velocity, 2);
//   }
//   if (splitmode) {
//     if (note < (newsplitPoint + 36)) {
//       MIDI.sendNoteOn((note + lowerTranspose), velocity, 1);
//     } else {
//       MIDI.sendNoteOn(note, velocity, 2);
//     }
//   }
// }

// void DinHandleNoteOff(byte channel, byte note, byte velocity) {
//   numberOfNotesU = numberOfNotesU - 1;
//   oldnumberOfNotesU = oldnumberOfNotesU - 1;
//   numberOfNotesL = numberOfNotesL - 1;
//   oldnumberOfNotesL = oldnumberOfNotesL - 1;

//   if (wholemode) {
//     switch (getVoiceNo(note)) {
//       case 1:
//         MIDI.sendNoteOff(note, velocity, 1);
//         voices[0].note = -1;
//         voiceOn[0] = false;
//         break;
//       case 2:
//         MIDI.sendNoteOff(note, velocity, 2);
//         voices[1].note = -1;
//         voiceOn[1] = false;
//         break;
//       case 3:
//         MIDI.sendNoteOff(note, velocity, 1);
//         voices[2].note = -1;
//         voiceOn[2] = false;
//         break;
//       case 4:
//         MIDI.sendNoteOff(note, velocity, 2);
//         voices[3].note = -1;
//         voiceOn[3] = false;
//         break;
//       case 5:
//         MIDI.sendNoteOff(note, velocity, 1);
//         voices[4].note = -1;
//         voiceOn[4] = false;
//         break;
//       case 6:
//         MIDI.sendNoteOff(note, velocity, 2);
//         voices[5].note = -1;
//         voiceOn[5] = false;
//         break;
//       case 7:
//         MIDI.sendNoteOff(note, velocity, 1);
//         voices[6].note = -1;
//         voiceOn[6] = false;
//         break;
//       case 8:
//         MIDI.sendNoteOff(note, velocity, 2);
//         voices[7].note = -1;
//         voiceOn[7] = false;
//         break;
//     }
//   }
//   if (dualmode) {
//     MIDI.sendNoteOff(note, velocity, 1);
//     MIDI.sendNoteOff(note, velocity, 2);
//   }
//   if (splitmode) {
//     if (note < (newsplitPoint + 36)) {
//       MIDI.sendNoteOff((note + lowerTranspose), velocity, 1);
//     } else {
//       MIDI.sendNoteOff(note, velocity, 2);
//     }
//   }
// }

// int getVoiceNo(int note) {
//   voiceToReturn = -1;       //Initialise to 'null'
//   earliestTime = millis();  //Initialise to now
//   if (note == -1) {
//     //NoteOn() - Get the oldest free voice (recent voices may be still on release stage)
//     for (int i = 0; i < NO_OF_VOICES; i++) {
//       if (voices[i].note == -1) {
//         if (voices[i].timeOn < earliestTime) {
//           earliestTime = voices[i].timeOn;
//           voiceToReturn = i;
//         }
//       }
//     }
//     if (voiceToReturn == -1) {
//       //No free voices, need to steal oldest sounding voice
//       earliestTime = millis();  //Reinitialise
//       for (int i = 0; i < NO_OF_VOICES; i++) {
//         if (voices[i].timeOn < earliestTime) {
//           earliestTime = voices[i].timeOn;
//           voiceToReturn = i;
//         }
//       }
//     }
//     return voiceToReturn + 1;
//   } else {
//     //NoteOff() - Get voice number from note
//     for (int i = 0; i < NO_OF_VOICES; i++) {
//       if (voices[i].note == note) {
//         return i + 1;
//       }
//     }
//   }
//   //Shouldn't get here, return voice 1
//   return 1;
// }

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
  delaytimeL = lowerData[P_LFODelay];
  if (delaytimeL <= 0) {
    delaytimeL = 0.1;
  }
  intervalL = (delaytimeL * 10);

  delaytimeU = upperData[P_LFODelay];
  if (delaytimeU <= 0) {
    delaytimeU = 0.1;
  }
  intervalU = (delaytimeU * 10);
}

void allNotesOff() {
  //midiCCOut(CCallnotesoff, 127);
}

void updatepwLFO(boolean announce) {
  if (announce) {
    showCurrentParameterPage("PWM Rate", int(pwLFOstr));
  }
  if (upperSW) {
    midiCCOut(CCpwLFO, upperData[P_pwLFO] >> midioutfrig);
    midiCCOut71(CCpwLFO, upperData[P_pwLFO] >> midioutfrig);
  } else {
    midiCCOut(CCpwLFO, lowerData[P_pwLFO] >> midioutfrig);
    midiCCOut71(CCpwLFO, lowerData[P_pwLFO] >> midioutfrig);
  }
}

void updatefmDepth(boolean announce) {
  if (announce) {
    showCurrentParameterPage("FM Depth", int(fmDepthstr));
  }
  if (upperSW) {
    midiCCOut(CCfmDepth, upperData[P_fmDepth] >> midioutfrig);
    midiCCOut71(CCfmDepth, upperData[P_fmDepth] >> midioutfrig);
  } else {
    midiCCOut(CCfmDepth, lowerData[P_fmDepth] >> midioutfrig);
    midiCCOut71(CCfmDepth, lowerData[P_fmDepth] >> midioutfrig);
  }
}

void updateosc2PW(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 PW", String(osc2PWstr) + " %");
  }
  if (upperSW) {
    midiCCOut62(WSosc2PW, upperData[P_osc2PW] >> midioutfrig);
    midiCCOut(CCosc2PW, upperData[P_osc2PW] >> midioutfrig);
    midiCCOut71(CCosc2PW, upperData[P_osc2PW] >> midioutfrig);
  } else {
    midiCCOut61(WSosc2PW, lowerData[P_osc2PW] >> midioutfrig);
    midiCCOut(CCosc2PW, lowerData[P_osc2PW] >> midioutfrig);
    midiCCOut71(CCosc2PW, lowerData[P_osc2PW] >> midioutfrig);
    if (wholemode) {
      midiCCOut62(WSosc2PW, upperData[P_osc2PW] >> midioutfrig);
    }
  }
}

void updateosc2PWM(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 PWM", int(osc2PWMstr));
  }
  if (upperSW) {
    midiCCOut62(WSosc2PWM, upperData[P_osc2PWM] >> midioutfrig);
    midiCCOut(CCosc2PWM, upperData[P_osc2PWM] >> midioutfrig);
    midiCCOut71(CCosc2PWM, upperData[P_osc2PWM] >> midioutfrig);
  } else {
    midiCCOut61(WSosc2PWM, lowerData[P_osc2PWM] >> midioutfrig);
    midiCCOut(CCosc2PWM, lowerData[P_osc2PWM] >> midioutfrig);
    midiCCOut71(CCosc2PWM, lowerData[P_osc2PWM] >> midioutfrig);
    if (wholemode) {
      midiCCOut62(WSosc2PWM, upperData[P_osc2PWM] >> midioutfrig);
    }
  }
}

void updateosc1PW(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC1 PW", String(osc1PWstr) + " %");
  }
  if (upperSW) {
    midiCCOut62(WSosc1PW, upperData[P_osc1PW] >> midioutfrig);
    midiCCOut(CCosc1PW, upperData[P_osc1PW] >> midioutfrig);
    midiCCOut71(CCosc1PW, upperData[P_osc1PW] >> midioutfrig);
  } else {
    midiCCOut61(WSosc1PW, lowerData[P_osc1PW] >> midioutfrig);
    midiCCOut(CCosc1PW, lowerData[P_osc1PW] >> midioutfrig);
    midiCCOut71(CCosc1PW, lowerData[P_osc1PW] >> midioutfrig);
    if (wholemode) {
      midiCCOut62(WSosc1PW, upperData[P_osc1PW] >> midioutfrig);
    }
  }
}

void updateosc1PWM(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC1 PWM", int(osc1PWMstr));
  }
  if (upperSW) {
    midiCCOut62(WSosc1PWM, upperData[P_osc1PWM] >> midioutfrig);
    midiCCOut(CCosc1PWM, upperData[P_osc1PWM] >> midioutfrig);
    midiCCOut71(CCosc1PWM, upperData[P_osc1PWM] >> midioutfrig);
  } else {
    midiCCOut61(WSosc1PWM, upperData[P_osc1PWM] >> midioutfrig);
    midiCCOut(CCosc1PWM, lowerData[P_osc1PWM] >> midioutfrig);
    midiCCOut71(CCosc1PWM, lowerData[P_osc1PWM] >> midioutfrig);
    if (wholemode) {
      midiCCOut62(WSosc1PWM, upperData[P_osc1PWM] >> midioutfrig);
    }
  }
}

void updateosc1Range(boolean announce) {
  if (upperSW) {
    if (upperData[P_osc1Range] == 2) {
      if (announce) {
        showCurrentParameterPage("Osc1 Range", String("8"));
      }
      midiCCOut(CCosc1Oct, 2);
      midiCCOut61(WSosc1oct, 127);
      midiCCOut72(CCosc1Oct, 2);
    } else if (upperData[P_osc1Range] == 1) {
      if (announce) {
        showCurrentParameterPage("Osc1 Range", String("16"));
      }
      midiCCOut(CCosc1Oct, 1);
      midiCCOut61(WSosc1oct, 63);
      midiCCOut72(CCosc1Oct, 1);
    } else {
      if (announce) {
        showCurrentParameterPage("Osc1 Range", String("32"));
      }
      midiCCOut(CCosc1Oct, 0);
      midiCCOut61(WSosc1oct, 0);
      midiCCOut72(CCosc1Oct, 0);
    }
  } else {
    if (lowerData[P_osc1Range] == 2) {
      if (announce) {
        showCurrentParameterPage("Osc1 Range", String("8"));
      }
      midiCCOut(CCosc1Oct, 2);
      midiCCOut62(WSosc1oct, 127);
      midiCCOut72(CCosc1Oct, 2);
      if (wholemode) {
        midiCCOut61(WSosc1oct, 127);
      }
    } else if (lowerData[P_osc1Range] == 1) {
      if (announce) {
        showCurrentParameterPage("Osc1 Range", String("16"));
      }
      midiCCOut(CCosc1Oct, 1);
      midiCCOut62(WSosc1oct, 63);
      midiCCOut72(CCosc1Oct, 1);
      if (wholemode) {
        midiCCOut61(WSosc1oct, 63);
      }
    } else {
      if (announce) {
        showCurrentParameterPage("Osc1 Range", String("32"));
      }
      midiCCOut(CCosc1Oct, 0);
      midiCCOut62(WSosc1oct, 0);
      midiCCOut72(CCosc1Oct, 0);
      if (wholemode) {
        midiCCOut61(WSosc1oct, 0);
      }
    }
  }
}

void updateosc2Range(boolean announce) {
  if (upperSW) {
    if (upperData[P_osc2Range] == 2) {
      if (announce) {
        showCurrentParameterPage("Osc2 Range", String("8"));
      }
      midiCCOut61(WSosc2oct, 127);
      midiCCOut72(CCosc2Oct, 2);
      midiCCOut(CCosc2Oct, 2);
    } else if (upperData[P_osc2Range] == 1) {
      if (announce) {
        showCurrentParameterPage("Osc2 Range", String("16"));
      }
      midiCCOut61(WSosc2oct, 63);
      midiCCOut72(CCosc2Oct, 1);
      midiCCOut(CCosc2Oct, 1);
    } else {
      if (announce) {
        showCurrentParameterPage("Osc2 Range", String("32"));
      }
      midiCCOut(CCosc2Oct, 0);
      midiCCOut61(WSosc2oct, 0);
      midiCCOut72(CCosc2Oct, 0);
    }
  } else {
    if (lowerData[P_osc2Range] == 2) {
      if (announce) {
        showCurrentParameterPage("Osc2 Range", String("8"));
      }
      midiCCOut(CCosc2Oct, 2);
      midiCCOut62(WSosc2oct, 127);
      midiCCOut72(CCosc2Oct, 2);
      if (wholemode) {
        midiCCOut61(WSosc2oct, 127);
        //midiCCOut72(CCosc2Oct, 2);
      }
    } else if (lowerData[P_osc2Range] == 1) {
      if (announce) {
        showCurrentParameterPage("Osc2 Range", String("16"));
      }
      midiCCOut(CCosc2Oct, 1);
      midiCCOut62(WSosc2oct, 63);
      midiCCOut72(CCosc2Oct, 1);
      if (wholemode) {
        midiCCOut61(WSosc2oct, 63);
        //midiCCOut72(CCosc2Oct, 1);
      }
    } else {
      if (announce) {
        showCurrentParameterPage("Osc2 Range", String("32"));
      }
      midiCCOut(CCosc2Oct, 0);
      midiCCOut62(WSosc2oct, 0);
      midiCCOut72(CCosc2Oct, 0);
      if (wholemode) {
        midiCCOut61(WSosc2oct, 0);
        //midiCCOut72(CCosc2Oct, 0);
      }
    }
  }
}

void updateglideTime(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Glide Time", String(glideTimestr * 10) + " Seconds");
  }
  if (upperSW) {
    midiCCOut62(WSglideTime, upperData[P_glideTime] >> midioutfrig);
    midiCCOut(CCglideTime, upperData[P_glideTime] >> midioutfrig);
    midiCCOut71(CCglideTime, upperData[P_glideTime] >> midioutfrig);
  } else {
    midiCCOut61(WSglideTime, upperData[P_glideTime] >> midioutfrig);
    midiCCOut(CCglideTime, lowerData[P_glideTime] >> midioutfrig);
    midiCCOut71(CCglideTime, lowerData[P_glideTime] >> midioutfrig);
    if (wholemode) {
      midiCCOut62(WSglideTime, upperData[P_glideTime] >> midioutfrig);
      //midiCCOut71(CCglideTime, upperData[P_glideTime] >> midioutfrig);
    }
  }
}

void updateosc2Detune(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 Detune", String(osc2Detunestr));
  }
  if (upperSW) {
    midiCCOut62(WSdetune, upperData[P_osc2Detune] >> midioutfrig);
    midiCCOut(CCosc2Detune, upperData[P_osc2Detune] >> midioutfrig);
    midiCCOut71(CCosc2Detune, upperData[P_osc2Detune] >> midioutfrig);
  } else {
    midiCCOut61(WSdetune, lowerData[P_osc2Detune] >> midioutfrig);
    midiCCOut(CCosc2Detune, lowerData[P_osc2Detune] >> midioutfrig);
    midiCCOut71(CCosc2Detune, lowerData[P_osc2Detune] >> midioutfrig);
    if (wholemode) {
      midiCCOut62(WSdetune, upperData[P_osc2Detune] >> midioutfrig);
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
    midiCCOut71(CCosc2Interval, map(upperData[P_osc2Interval], 0, 12, 0, 127));
  } else {
    midiCCOut61(WSinterval, lowerData[P_osc2Interval]);
    midiCCOut(CCosc2Interval, lowerData[P_osc2Interval] >> midioutfrig);
    midiCCOut71(CCosc2Interval, map(lowerData[P_osc2Interval], 0, 12, 0, 127));
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
    midiCCOut(CCnoiseLevel, upperData[P_noiseLevel] >> midioutfrig);
    midiCCOut71(CCnoiseLevel, upperData[P_noiseLevel] >> midioutfrig);
  } else {
    midiCCOut(CCnoiseLevel, lowerData[P_noiseLevel] >> midioutfrig);
    midiCCOut71(CCnoiseLevel, lowerData[P_noiseLevel] >> midioutfrig);
  }
}

void updateOsc2SawLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 Saw", int(osc2SawLevelstr));
  }
  if (upperSW) {
    midiCCOut(CCosc2SawLevel, upperData[P_osc2SawLevel] >> midioutfrig);
    midiCCOut71(CCosc2SawLevel, upperData[P_osc2SawLevel] >> midioutfrig);
  } else {
    midiCCOut(CCosc2SawLevel, lowerData[P_osc2SawLevel] >> midioutfrig);
    midiCCOut71(CCosc2SawLevel, lowerData[P_osc2SawLevel] >> midioutfrig);
  }
}

void updateOsc1SawLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC1 Saw", int(osc1SawLevelstr));
  }
  if (upperSW) {
    midiCCOut(CCosc1SawLevel, upperData[P_osc1SawLevel] >> midioutfrig);
    midiCCOut71(CCosc1SawLevel, upperData[P_osc1SawLevel] >> midioutfrig);
  } else {
    midiCCOut(CCosc1SawLevel, lowerData[P_osc1SawLevel] >> midioutfrig);
    midiCCOut71(CCosc1SawLevel, lowerData[P_osc1SawLevel] >> midioutfrig);
  }
}

void updateOsc2PulseLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 Pulse", int(osc2PulseLevelstr));
  }
  if (upperSW) {
    midiCCOut(CCosc2PulseLevel, upperData[P_osc2PulseLevel] >> midioutfrig);
    midiCCOut71(CCosc2PulseLevel, upperData[P_osc2PulseLevel] >> midioutfrig);
  } else {
    midiCCOut(CCosc2PulseLevel, lowerData[P_osc2PulseLevel] >> midioutfrig);
    midiCCOut71(CCosc2PulseLevel, lowerData[P_osc2PulseLevel] >> midioutfrig);
  }
}

void updateOsc1PulseLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC1 Pulse", int(osc1PulseLevelstr));
  }
  if (upperSW) {
    midiCCOut(CCosc1PulseLevel, upperData[P_osc1PulseLevel] >> midioutfrig);
    midiCCOut71(CCosc1PulseLevel, upperData[P_osc1PulseLevel] >> midioutfrig);
  } else {
    midiCCOut(CCosc1PulseLevel, lowerData[P_osc1PulseLevel] >> midioutfrig);
    midiCCOut71(CCosc1PulseLevel, lowerData[P_osc1PulseLevel] >> midioutfrig);
  }
}

void updateOsc2TriangleLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 Triangle", int(osc2TriangleLevelstr));
  }
  if (upperSW) {
    midiCCOut(CCosc2TriangleLevel, upperData[P_osc2TriangleLevel] >> midioutfrig);
    midiCCOut71(CCosc2TriangleLevel, upperData[P_osc2TriangleLevel] >> midioutfrig);
  } else {
    midiCCOut(CCosc2TriangleLevel, lowerData[P_osc2TriangleLevel] >> midioutfrig);
    midiCCOut71(CCosc2TriangleLevel, lowerData[P_osc2TriangleLevel] >> midioutfrig);
  }
}

void updateOsc1SubLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC1 Sub", int(osc1SubLevelstr));
  }
  if (upperSW) {
    midiCCOut(CCosc1SubLevel, upperData[P_osc1SubLevel] >> midioutfrig);
    midiCCOut71(CCosc1SubLevel, upperData[P_osc1SubLevel] >> midioutfrig);
  } else {
    midiCCOut(CCosc1SubLevel, lowerData[P_osc1SubLevel] >> midioutfrig);
    midiCCOut71(CCosc1SubLevel, lowerData[P_osc1SubLevel] >> midioutfrig);
  }
}

void updateamDepth(boolean announce) {
  if (announce) {
    showCurrentParameterPage("AM Depth", int(amDepthstr));
  }
  if (upperSW) {
    midiCCOut(CCamDepth, upperData[P_amDepth] >> midioutfrig);
    midiCCOut71(CCamDepth, upperData[P_amDepth] >> midioutfrig);
  } else {
    midiCCOut(CCamDepth, lowerData[P_amDepth] >> midioutfrig);
    midiCCOut71(CCamDepth, lowerData[P_amDepth] >> midioutfrig);
  }
}

void updateFilterCutoff(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Cutoff", String(filterCutoffstr) + " Hz");
  }
  if (upperSW) {
    midiCCOut(CCfilterCutoff, upperData[P_filterCutoff] >> midioutfrig);
    midiCCOut71(CCfilterCutoff, upperData[P_filterCutoff] >> midioutfrig);
  } else {
    midiCCOut(CCfilterCutoff, lowerData[P_filterCutoff] >> midioutfrig);
    midiCCOut71(CCfilterCutoff, lowerData[P_filterCutoff] >> midioutfrig);
  }
}

void updatefilterLFO(boolean announce) {
  if (announce) {
    showCurrentParameterPage("TM depth", int(filterLFOstr));
  }
  if (upperSW) {
    midiCCOut(CCfilterLFO, upperData[P_filterLFO] >> midioutfrig);
    midiCCOut71(CCfilterLFO, upperData[P_filterLFO] >> midioutfrig);
  } else {
    midiCCOut(CCfilterLFO, lowerData[P_filterLFO] >> midioutfrig);
    midiCCOut71(CCfilterLFO, lowerData[P_filterLFO] >> midioutfrig);
  }
}

void updatefilterRes(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Resonance", int(filterResstr));
  }
  if (upperSW) {
    midiCCOut(CCfilterRes, upperData[P_filterRes] >> midioutfrig);
    midiCCOut71(CCfilterRes, upperData[P_filterRes] >> midioutfrig);
  } else {
    midiCCOut(CCfilterRes, lowerData[P_filterRes] >> midioutfrig);
    midiCCOut71(CCfilterRes, lowerData[P_filterRes] >> midioutfrig);
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
    midiCCOut(CCfilterEGlevel, upperData[P_filterEGlevel] >> midioutfrig);
    midiCCOut71(CCfilterEGlevel, upperData[P_filterEGlevel] >> midioutfrig);
  } else {
    midiCCOut(CCfilterEGlevel, lowerData[P_filterEGlevel] >> midioutfrig);
    midiCCOut71(CCfilterEGlevel, lowerData[P_filterEGlevel] >> midioutfrig);
  }
}

void updatekeytrack(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Keytrack", int(keytrackstr));
  }
  if (upperSW) {
    midiCCOut62(WSkeytrack, upperData[P_keytrack] >> midioutfrig);
    midiCCOut(CCkeyTrack, upperData[P_keytrack] >> midioutfrig);
    midiCCOut71(CCkeyTrack, upperData[P_keytrack] >> midioutfrig);
  } else {
    midiCCOut61(WSkeytrack, lowerData[P_keytrack] >> midioutfrig);
    midiCCOut(CCkeyTrack, lowerData[P_keytrack] >> midioutfrig);
    midiCCOut71(CCkeyTrack, lowerData[P_keytrack] >> midioutfrig);
    if (wholemode) {
      midiCCOut62(WSkeytrack, upperData[P_keytrack] >> midioutfrig);
    }
  }
}

void updateLFORate(boolean announce) {
  if (announce) {
    showCurrentParameterPage("LFO Rate", String(LFORatestr) + " Hz");
  }
  if (upperSW) {
    midiCCOut(CCLFORate, upperData[P_LFORate] >> midioutfrig);
    midiCCOut71(CCLFORate, upperData[P_LFORate] >> midioutfrig);
  } else {
    midiCCOut(CCLFORate, lowerData[P_LFORate] >> midioutfrig);
    midiCCOut71(CCLFORate, lowerData[P_LFORate] >> midioutfrig);
  }
}

void updateLFODelay(boolean announce) {
  if (announce) {
    showCurrentParameterPage("LFO Delay", String(LFODelaystr));
  }
  if (upperSW) {
    midiCCOut(CCLFODelay, upperData[P_LFODelay] >> midioutfrig);
    midiCCOut71(CCLFODelay, upperData[P_LFODelay] >> midioutfrig);
  } else {
    midiCCOut(CCLFODelay, lowerData[P_LFODelay] >> midioutfrig);
    midiCCOut71(CCLFODelay, lowerData[P_LFODelay] >> midioutfrig);
  }
}

void updatemodWheelDepth(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Mod Wheel Depth", String(modWheelDepthstr));
  }
  if (upperSW) {
    midiCCOut(CCmodWheelDepth, upperData[P_modWheelDepth] >> midioutfrig);
    midiCCOut71(CCmodWheelDepth, upperData[P_modWheelDepth] >> midioutfrig);
  } else {
    midiCCOut(CCmodWheelDepth, lowerData[P_modWheelDepth] >> midioutfrig);
    midiCCOut71(CCmodWheelDepth, lowerData[P_modWheelDepth] >> midioutfrig);
  }
}

void updatePitchBendDepth(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Pitch Bend Depth", String(PitchBendLevelstr));
  }
  if (upperSW) {
    midiCCOut62(WSbendRange, upperData[P_PitchBendLevel]);
    midiCCOut(CCPitchBend, upperData[P_PitchBendLevel] >> midioutfrig);
    midiCCOut71(CCPitchBend, map(upperData[P_PitchBendLevel], 0, 12, 0, 127));
  } else {
    midiCCOut61(WSbendRange, lowerData[P_PitchBendLevel]);
    midiCCOut(CCPitchBend, lowerData[P_PitchBendLevel] >> midioutfrig);
    midiCCOut71(CCPitchBend, map(upperData[P_PitchBendLevel], 0, 12, 0, 127));
  }
}

void updateeffectPot1(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Effect Pot 1", String(effectPot1str));
  }
  if (upperSW) {
    midiCCOut(CCeffectPot1, upperData[P_effectPot1] >> midioutfrig);
    midiCCOut71(CCeffectPot1, upperData[P_effectPot1] >> midioutfrig);
  } else {
    midiCCOut(CCeffectPot1, lowerData[P_effectPot1] >> midioutfrig);
    midiCCOut71(CCeffectPot1, lowerData[P_effectPot1] >> midioutfrig);
  }
}

void updateeffectPot2(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Effect Pot 2", String(effectPot2str));
  }
  if (upperSW) {
    midiCCOut(CCeffectPot2, upperData[P_effectPot2] >> midioutfrig);
    midiCCOut71(CCeffectPot2, upperData[P_effectPot2] >> midioutfrig);
  } else {
    midiCCOut(CCeffectPot2, lowerData[P_effectPot2] >> midioutfrig);
    midiCCOut71(CCeffectPot2, lowerData[P_effectPot2] >> midioutfrig);
  }
}

void updateeffectPot3(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Effect Pot 3", String(effectPot3str));
  }
  if (upperSW) {
    midiCCOut(CCeffectPot3, upperData[P_effectPot3] >> midioutfrig);
    midiCCOut71(CCeffectPot3, upperData[P_effectPot3] >> midioutfrig);
  } else {
    midiCCOut(CCeffectPot3, lowerData[P_effectPot3] >> midioutfrig);
    midiCCOut71(CCeffectPot3, lowerData[P_effectPot3] >> midioutfrig);
  }
}

void updateeffectsMix(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Effects Mix", String(effectsMixstr));
  }
  if (upperSW) {
    midiCCOut(CCeffectsMix, upperData[P_effectsMix] >> midioutfrig);
    midiCCOut71(CCeffectsMix, upperData[P_effectsMix] >> midioutfrig);
  } else {
    midiCCOut(CCeffectsMix, lowerData[P_effectsMix] >> midioutfrig);
    midiCCOut71(CCeffectsMix, lowerData[P_effectsMix] >> midioutfrig);
  }
}

void updateStratusLFOWaveform(boolean announce) {
  if (!panelData[P_lfoAlt]) {
    switch (panelData[P_LFOWaveform]) {
      case 0:
        StratusLFOWaveform = "Sawtooth Up";
        LFOWaveCV = 40;
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
        LFOWaveCV = 40;
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
    upperData[P_LFOWaveform] = LFOWaveCV;
  } else {
    lowerData[P_LFOWaveform] = LFOWaveCV;
    if (wholemode) {
      upperData[P_LFOWaveform] = LFOWaveCV;
    }
  }
}

void updatefilterAttack(boolean announce) {

  if (filterAttackstr < 1000) {
    if (announce) {
      showCurrentParameterPage("VCF Attack", String(int(filterAttackstr)) + " ms", FILTER_ENV);
    } else {
      showCurrentParameterPage("VCF Attack", String(filterAttackstr * 0.001) + " s", FILTER_ENV);
    }
  }
  if (upperSW) {
    midiCCOut(CCfilterAttack, upperData[P_filterAttack] >> midioutfrig);
    midiCCOut71(CCfilterAttack, upperData[P_filterAttack] >> midioutfrig);
  } else {
    midiCCOut(CCfilterAttack, lowerData[P_filterAttack] >> midioutfrig);
    midiCCOut71(CCfilterAttack, lowerData[P_filterAttack] >> midioutfrig);
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
    midiCCOut(CCfilterDecay, upperData[P_filterDecay] >> midioutfrig);
    midiCCOut71(CCfilterDecay, upperData[P_filterDecay] >> midioutfrig);
  } else {
    midiCCOut(CCfilterDecay, lowerData[P_filterDecay] >> midioutfrig);
    midiCCOut71(CCfilterDecay, lowerData[P_filterDecay] >> midioutfrig);
  }
}

void updatefilterSustain(boolean announce) {
  if (announce) {
    showCurrentParameterPage("VCF Sustain", String(filterSustainstr), FILTER_ENV);
  }
  if (upperSW) {
    midiCCOut(CCfilterSustain, upperData[P_filterSustain] >> midioutfrig);
    midiCCOut71(CCfilterSustain, upperData[P_filterSustain] >> midioutfrig);
  } else {
    midiCCOut(CCfilterSustain, lowerData[P_filterSustain] >> midioutfrig);
    midiCCOut71(CCfilterSustain, lowerData[P_filterSustain] >> midioutfrig);
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
    midiCCOut(CCfilterRelease, upperData[P_filterRelease] >> midioutfrig);
    midiCCOut71(CCfilterRelease, upperData[P_filterRelease] >> midioutfrig);
  } else {
    midiCCOut(CCfilterRelease, lowerData[P_filterRelease] >> midioutfrig);
    midiCCOut71(CCfilterRelease, lowerData[P_filterRelease] >> midioutfrig);
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
    midiCCOut(CCampAttack, upperData[P_ampAttack] >> midioutfrig);
    midiCCOut71(CCampAttack, upperData[P_ampAttack] >> midioutfrig);
  } else {
    midiCCOut(CCampAttack, lowerData[P_ampAttack] >> midioutfrig);
    midiCCOut71(CCampAttack, lowerData[P_ampAttack] >> midioutfrig);
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
    midiCCOut(CCampDecay, upperData[P_ampDecay] >> midioutfrig);
    midiCCOut71(CCampDecay, upperData[P_ampDecay] >> midioutfrig);
  } else {
    midiCCOut(CCampDecay, lowerData[P_ampDecay] >> midioutfrig);
    midiCCOut71(CCampDecay, lowerData[P_ampDecay] >> midioutfrig);
  }
}

void updateampSustain(boolean announce) {
  if (announce) {
    showCurrentParameterPage("VCA Sustain", String(ampSustainstr), AMP_ENV);
  }
  if (upperSW) {
    midiCCOut(CCampSustain, upperData[P_ampSustain] >> midioutfrig);
    midiCCOut71(CCampSustain, upperData[P_ampSustain] >> midioutfrig);
  } else {
    midiCCOut(CCampSustain, lowerData[P_ampSustain] >> midioutfrig);
    midiCCOut71(CCampSustain, lowerData[P_ampSustain] >> midioutfrig);
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
    midiCCOut(CCampRelease, upperData[P_ampRelease] >> midioutfrig);
    midiCCOut71(CCampRelease, upperData[P_ampRelease] >> midioutfrig);
  } else {
    midiCCOut(CCampRelease, lowerData[P_ampRelease] >> midioutfrig);
    midiCCOut71(CCampRelease, lowerData[P_ampRelease] >> midioutfrig);
  }
}

void updatevolumeControl(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Volume", int(volumeControlstr));
  }
  if (upperSW) {
    midiCCOut(CCvolumeControl, upperData[P_volumeControl] >> midioutfrig);
    midiCCOut71(CCvolumeControl, upperData[P_volumeControl] >> midioutfrig);
  } else {
    midiCCOut(CCvolumeControl, lowerData[P_volumeControl] >> midioutfrig);
    midiCCOut71(CCvolumeControl, lowerData[P_volumeControl] >> midioutfrig);
  }
}

void updatePM_DCO2(boolean announce) {
  if (announce) {
    showCurrentParameterPage("PolyMod DCO2", int(pmDCO2str));
  }
  if (upperSW) {
    midiCCOut(CCPM_DCO2, upperData[P_pmDCO2] >> midioutfrig);
    midiCCOut71(CCPM_DCO2, upperData[P_pmDCO2] >> midioutfrig);
  } else {
    midiCCOut(CCPM_DCO2, lowerData[P_pmDCO2] >> midioutfrig);
    midiCCOut71(CCPM_DCO2, lowerData[P_pmDCO2] >> midioutfrig);
  }
}

void updatePM_FilterEnv(boolean announce) {
  if (announce) {
    showCurrentParameterPage("PolyMod Filter Env", int(pmFilterEnvstr));
  }
  if (upperSW) {
    midiCCOut(CCPM_FilterEnv, upperData[P_pmFilterEnv] >> midioutfrig);
    midiCCOut71(CCPM_FilterEnv, upperData[P_pmFilterEnv] >> midioutfrig);
  } else {
    midiCCOut(CCPM_FilterEnv, lowerData[P_pmFilterEnv] >> midioutfrig);
    midiCCOut71(CCPM_FilterEnv, lowerData[P_pmFilterEnv] >> midioutfrig);
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
  } else if (playMode == 1) {
    if (announce) {
      showCurrentParameterPage("Key Mode", "Dual");
    }
    midiCCOut72(CCplayMode, 1);
    midiCCOut(CCplayMode, 1);
  } else if (playMode == 2) {
    if (announce) {
      showCurrentParameterPage("Key Mode", "Split");
    }
    midiCCOut72(CCplayMode, 2);
    midiCCOut(CCplayMode, 2);
  }
}

void updatekeyboardMode(boolean announce) {
  if (upperSW) {
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
        showCurrentParameterPage("Keyboard Mode", "Unison");
      }
      midiCCOut72(CCkeyboardMode, 2);
      midiCCOut(CCkeyboardMode, 2);
    } else if (upperData[P_keyboardMode] == 3) {
      if (announce) {
        showCurrentParameterPage("Keyboard Mode", "Mono");
      }
      midiCCOut72(CCkeyboardMode, 3);
      midiCCOut(CCkeyboardMode, 3);
    }
  } else {
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
        showCurrentParameterPage("Keyboard Mode", "Unison");
      }
      midiCCOut72(CCkeyboardMode, 2);
      midiCCOut(CCkeyboardMode, 2);
    } else if (lowerData[P_keyboardMode] == 3) {
      if (announce) {
        showCurrentParameterPage("Keyboard Mode", "Mono");
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
      srp.writePin(EFFECT_O_UPPER, LOW);
      srp.writePin(EFFECT_1_UPPER, LOW);
      srp.writePin(EFFECT_2_UPPER, LOW);
      midiCCOut72(CCeffectNumSW, 0);
      midiCCOut(CCeffectNumSW, 0);

    } else if (upperData[P_effectNum] == 1) {
      if (announce) {
        showCurrentParameterPage("Effect", "2");
      }
      srp.writePin(EFFECT_O_UPPER, HIGH);
      srp.writePin(EFFECT_1_UPPER, LOW);
      srp.writePin(EFFECT_2_UPPER, LOW);
      midiCCOut72(CCeffectNumSW, 1);
      midiCCOut(CCeffectNumSW, 1);

    } else if (upperData[P_effectNum] == 2) {
      if (announce) {
        showCurrentParameterPage("Effect", "3");
      }
      srp.writePin(EFFECT_O_UPPER, LOW);
      srp.writePin(EFFECT_1_UPPER, HIGH);
      srp.writePin(EFFECT_2_UPPER, LOW);
      midiCCOut72(CCeffectNumSW, 2);
      midiCCOut(CCeffectNumSW, 2);

    } else if (upperData[P_effectNum] == 3) {
      if (announce) {
        showCurrentParameterPage("Effect", "4");
      }
      srp.writePin(EFFECT_O_UPPER, HIGH);
      srp.writePin(EFFECT_1_UPPER, HIGH);
      srp.writePin(EFFECT_2_UPPER, LOW);
      midiCCOut72(CCeffectNumSW, 3);
      midiCCOut(CCeffectNumSW, 3);

    } else if (upperData[P_effectNum] == 4) {
      if (announce) {
        showCurrentParameterPage("Effect", "5");
      }
      srp.writePin(EFFECT_O_UPPER, LOW);
      srp.writePin(EFFECT_1_UPPER, LOW);
      srp.writePin(EFFECT_2_UPPER, HIGH);
      midiCCOut72(CCeffectNumSW, 4);
      midiCCOut(CCeffectNumSW, 4);

    } else if (upperData[P_effectNum] == 5) {
      if (announce) {
        showCurrentParameterPage("Effect", "6");
      }
      srp.writePin(EFFECT_O_UPPER, HIGH);
      srp.writePin(EFFECT_1_UPPER, LOW);
      srp.writePin(EFFECT_2_UPPER, HIGH);
      midiCCOut72(CCeffectNumSW, 5);
      midiCCOut(CCeffectNumSW, 5);

    } else if (upperData[P_effectNum] == 6) {
      if (announce) {
        showCurrentParameterPage("Effect", "7");
      }
      srp.writePin(EFFECT_O_UPPER, LOW);
      srp.writePin(EFFECT_1_UPPER, HIGH);
      srp.writePin(EFFECT_2_UPPER, HIGH);
      midiCCOut72(CCeffectNumSW, 6);
      midiCCOut(CCeffectNumSW, 6);

    } else if (upperData[P_effectNum] == 7) {
      if (announce) {
        showCurrentParameterPage("Effect", "8");
      }
      srp.writePin(EFFECT_O_UPPER, HIGH);
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
        srp.writePin(EFFECT_O_UPPER, LOW);
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
        srp.writePin(EFFECT_O_UPPER, HIGH);
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
        srp.writePin(EFFECT_O_UPPER, LOW);
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
        srp.writePin(EFFECT_O_UPPER, HIGH);
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
        srp.writePin(EFFECT_O_UPPER, LOW);
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
        srp.writePin(EFFECT_O_UPPER, HIGH);
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
        srp.writePin(EFFECT_O_UPPER, LOW);
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
        srp.writePin(EFFECT_O_UPPER, HIGH);
        srp.writePin(EFFECT_1_UPPER, HIGH);
        srp.writePin(EFFECT_2_UPPER, HIGH);
      }
      midiCCOut72(CCeffectNumSW, 7);
      midiCCOut(CCeffectNumSW, 7);
    }
  }
}

void updateeffectBankSW(boolean announce) {
  if (upperSW) {
    if (upperData[P_effectBank] == 0) {
      if (announce) {
        showCurrentParameterPage("Effects", "Bank 1");
      }
      srp.writePin(EFFECT_BANK_1_UPPER, LOW);
      srp.writePin(EFFECT_BANK_2_UPPER, LOW);
      srp.writePin(EFFECT_BANK_3_UPPER, LOW);
      midiCCOut72(CCeffectBankSW, 0);
      midiCCOut(CCeffectBankSW, 0);
    } else if (upperData[P_effectBank] == 1) {
      if (announce) {
        showCurrentParameterPage("Effects", "Bank 2");
      }
      srp.writePin(EFFECT_BANK_1_UPPER, HIGH);
      srp.writePin(EFFECT_BANK_2_UPPER, LOW);
      srp.writePin(EFFECT_BANK_3_UPPER, LOW);
      midiCCOut72(CCeffectBankSW, 1);
      midiCCOut(CCeffectBankSW, 1);
    } else if (upperData[P_effectBank] == 2) {
      if (announce) {
        showCurrentParameterPage("Effects", "Bank 3");
      }
      srp.writePin(EFFECT_BANK_1_UPPER, LOW);
      srp.writePin(EFFECT_BANK_2_UPPER, HIGH);
      srp.writePin(EFFECT_BANK_3_UPPER, LOW);
      midiCCOut72(CCeffectBankSW, 2);
      midiCCOut(CCeffectBankSW, 2);
    } else if (upperData[P_effectBank] == 3) {
      if (announce) {
        showCurrentParameterPage("Effects", "Bank 4");
      }
      srp.writePin(EFFECT_BANK_1_UPPER, HIGH);
      srp.writePin(EFFECT_BANK_2_UPPER, HIGH);
      srp.writePin(EFFECT_BANK_3_UPPER, LOW);
      midiCCOut72(CCeffectBankSW, 3);
      midiCCOut(CCeffectBankSW, 3);
    }
  } else {
    if (lowerData[P_effectBank] == 0) {
      if (announce) {
        showCurrentParameterPage("Effects", "Bank 1");
      }
      srp.writePin(EFFECT_BANK_1_LOWER, LOW);
      srp.writePin(EFFECT_BANK_2_LOWER, LOW);
      srp.writePin(EFFECT_BANK_3_LOWER, LOW);
      if (wholemode) {
        srp.writePin(EFFECT_BANK_1_UPPER, LOW);
        srp.writePin(EFFECT_BANK_2_UPPER, LOW);
        srp.writePin(EFFECT_BANK_3_UPPER, LOW);
      }
      midiCCOut72(CCeffectBankSW, 0);
      midiCCOut(CCeffectBankSW, 0);
    } else if (lowerData[P_effectBank] == 1) {
      if (announce) {
        showCurrentParameterPage("Effects", "Bank 2");
      }
      srp.writePin(EFFECT_BANK_1_LOWER, HIGH);
      srp.writePin(EFFECT_BANK_2_LOWER, LOW);
      srp.writePin(EFFECT_BANK_3_LOWER, LOW);
      if (wholemode) {
        srp.writePin(EFFECT_BANK_1_UPPER, HIGH);
        srp.writePin(EFFECT_BANK_2_UPPER, LOW);
        srp.writePin(EFFECT_BANK_3_UPPER, LOW);
      }
      midiCCOut72(CCeffectBankSW, 1);
      midiCCOut(CCeffectBankSW, 1);
    } else if (lowerData[P_effectBank] == 2) {
      if (announce) {
        showCurrentParameterPage("Effects", "Bank 3");
      }
      srp.writePin(EFFECT_BANK_1_LOWER, LOW);
      srp.writePin(EFFECT_BANK_2_LOWER, HIGH);
      srp.writePin(EFFECT_BANK_3_LOWER, LOW);
      if (wholemode) {
        srp.writePin(EFFECT_BANK_1_UPPER, LOW);
        srp.writePin(EFFECT_BANK_2_UPPER, HIGH);
        srp.writePin(EFFECT_BANK_3_UPPER, LOW);
      }
      midiCCOut72(CCeffectBankSW, 2);
      midiCCOut(CCeffectBankSW, 2);
    } else if (lowerData[P_effectBank] == 3) {
      if (announce) {
        showCurrentParameterPage("Effects", "Bank 4");
      }
      srp.writePin(EFFECT_BANK_1_LOWER, HIGH);
      srp.writePin(EFFECT_BANK_2_LOWER, HIGH);
      srp.writePin(EFFECT_BANK_3_LOWER, LOW);
      if (wholemode) {
        srp.writePin(EFFECT_BANK_1_UPPER, HIGH);
        srp.writePin(EFFECT_BANK_2_UPPER, HIGH);
        srp.writePin(EFFECT_BANK_3_UPPER, LOW);
      }
      midiCCOut72(CCeffectBankSW, 3);
      midiCCOut(CCeffectBankSW, 3);
    }
  }
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
      midiCCOut62(CCglideTime, upperData[P_glideTime] >> midioutfrig);
      midiCCOut62(CCglideSW, 127);
      midiCCOut71(CCglideTime, upperData[P_glideTime] >> midioutfrig);
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
      midiCCOut61(CCglideTime, lowerData[P_glideTime] >> midioutfrig);
      midiCCOut61(CCglideSW, 127);
      midiCCOut71(CCglideTime, lowerData[P_glideTime] >> midioutfrig);
      midiCCOut72(CCglideSW, 1);
      if (wholemode) {
        midiCCOut62(CCglideTime, upperData[P_glideTime] >> midioutfrig);
        midiCCOut62(CCglideSW, 1);
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

void updatekeytrackSW(boolean announce) {
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
      upperData[P_ampSustain] = 1023;
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
      lowerData[P_ampSustain] = 1023;
      lowerData[P_ampRelease] = 0;
      if (wholemode) {
        upperData[P_ampAttack] = 0;
        upperData[P_ampDecay] = 0;
        upperData[P_ampSustain] = 1023;
        upperData[P_ampRelease] = 0;
      }
    }
  }
}

void updatelfoAlt(boolean announce) {
  if (upperSW) {
    if (!upperData[P_lfoAlt]) {
      midiCCOut(CClfoAlt, 0);
      midiCCOut72(CClfoAlt, 0);
      updateStratusLFOWaveform(1);
      srp.writePin(LFO_ALT_UPPER, HIGH);
    } else {
      midiCCOut(CClfoAlt, 127);
      midiCCOut72(CClfoAlt, 1);
      updateStratusLFOWaveform(1);
      srp.writePin(LFO_ALT_UPPER, LOW);
    }
  } else {
    if (!lowerData[P_lfoAlt]) {
      midiCCOut(CClfoAlt, 0);
      midiCCOut72(CClfoAlt, 0);
      updateStratusLFOWaveform(1);
      srp.writePin(LFO_ALT_LOWER, HIGH);
      if (wholemode) {
        srp.writePin(LFO_ALT_UPPER, HIGH);
      }
    } else {
      midiCCOut(CClfoAlt, 127);
      midiCCOut72(CClfoAlt, 1);
      updateStratusLFOWaveform(1);
      srp.writePin(LFO_ALT_LOWER, LOW);
      if (wholemode) {
        srp.writePin(LFO_ALT_UPPER, LOW);
      }
    }
  }
}

void updateupperSW() {
  if (upperSW) {
    showCurrentParameterPage("Upper", "On");
    setAllButtons();
    midiCCOut72(CCupperSW, 1);
  } else {
    upperSW = 0;
  }
}

void updatelowerSW() {
  if (lowerSW) {
    showCurrentParameterPage("Lower", "On");
    setAllButtons();
    midiCCOut72(CClowerSW, 1);
  } else {
    lowerSW = 0;
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
    } else {
      if (announce) {
        showCurrentParameterPage("LFO Retrigger", "On");
      }
      midiCCOut(CCmonoMulti, 127);
      midiCCOut72(CCmonoMulti, 1);
    }
  }
}

void updatePatchname() {
  showPatchPage(String(patchNoU), patchNameU, String(patchNoL), patchNameL);
}

void myControlChange(byte channel, byte control, int value) {

  switch (control) {
    case CCpwLFO:
      if (upperSW) {
        upperData[P_pwLFO] = value;
      } else {
        lowerData[P_pwLFO] = value;
        if (wholemode) {
          upperData[P_pwLFO] = value;
        }
      }
      pwLFOstr = value >> midioutfrig;  // for display
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
      fmDepthstr = value >> midioutfrig;
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
      osc2PWstr = PULSEWIDTH[value >> midioutfrig];
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
      osc2PWMstr = value >> midioutfrig;
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
      osc1PWstr = PULSEWIDTH[value >> midioutfrig];
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
      osc1PWMstr = value >> midioutfrig;
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

    case CCglideTime:
      if (upperSW) {
        upperData[P_glideTime] = value;
      } else {
        lowerData[P_glideTime] = value;
        if (wholemode) {
          upperData[P_glideTime] = value;
        }
      }
      glideTimestr = LINEAR[value >> midioutfrig];
      updateglideTime(1);
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
      osc2Detunestr = PULSEWIDTH[value >> midioutfrig];
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

    case CCnoiseLevel:
      if (upperSW) {
        upperData[P_noiseLevel] = value;
      } else {
        lowerData[P_noiseLevel] = value;
        if (wholemode) {
          upperData[P_noiseLevel] = value;
        }
      }
      noiseLevelstr = LINEARCENTREZERO[value >> midioutfrig];
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
      osc2SawLevelstr = value >> midioutfrig;  // for display
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
      osc1SawLevelstr = value >> midioutfrig;  // for display
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
      osc2PulseLevelstr = value >> midioutfrig;  // for display
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
      osc1PulseLevelstr = value >> midioutfrig;  // for display
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
      osc2TriangleLevelstr = value >> midioutfrig;  // for display
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
      osc1SubLevelstr = value >> midioutfrig;  // for display
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
      LFODelaystr = value >> midioutfrig;  // for display
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
      filterCutoffstr = FILTERCUTOFF[value >> midioutfrig];
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
      filterLFOstr = value >> midioutfrig;
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
      filterResstr = int(value >> midioutfrig);
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
      filterEGlevelstr = int(value >> midioutfrig);
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
      LFORatestr = LFOTEMPO[value >> midioutfrig];  // for display
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
      modWheelDepthstr = value >> midioutfrig;  // for display
      updatemodWheelDepth(1);
      break;

    case CCPitchBend:
      Serial.println(value);
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
      effectPot1str = value >> midioutfrig;  // for display
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
      effectPot2str = value >> midioutfrig;  // for display
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
      effectPot3str = value >> midioutfrig;  // for display
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
      effectsMixstr = value >> midioutfrig;  // for display
      updateeffectsMix(1);
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

    case CCfilterAttack:
      if (upperSW) {
        upperData[P_filterAttack] = value;
      } else {
        lowerData[P_filterAttack] = value;
        if (wholemode) {
          upperData[P_filterAttack] = value;
        }
      }
      filterAttackstr = ENVTIMES[value >> midioutfrig];
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
      filterDecaystr = ENVTIMES[value >> midioutfrig];
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
      filterSustainstr = LINEAR_FILTERMIXERSTR[value >> midioutfrig];
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
      filterReleasestr = ENVTIMES[value >> midioutfrig];
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
      ampAttackstr = ENVTIMES[value >> midioutfrig];
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
      ampDecaystr = ENVTIMES[value >> midioutfrig];
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
      ampSustainstr = LINEAR_FILTERMIXERSTR[value >> midioutfrig];
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
      ampReleasestr = ENVTIMES[value >> midioutfrig];
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
      volumeControlstr = value >> midioutfrig;
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
      pmDCO2str = value >> midioutfrig;
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
      pmFilterEnvstr = value >> midioutfrig;
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
      keytrackstr = value >> midioutfrig;
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
      amDepthstr = value >> midioutfrig;
      updateamDepth(1);
      break;

      //   ////////////////////////////////////////////////

    case CCplayMode:
      updateplayMode(1);
      break;

    case CCNotePriority:
      if (upperSW) {
        upperData[P_NotePriority] = value;
      } else {
        lowerData[P_NotePriority] = value;
      }
      updateNotePriority(1);
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

    case CCkeyTrackSW:
      if (upperSW) {
        upperData[P_keytrackSW] = !upperData[P_keytrackSW];
      } else {
        lowerData[P_keytrackSW] = !lowerData[P_keytrackSW];
      }
      updatekeytrackSW(1);
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

    case CCupperSW:
      upperSW = value;
      lowerSW = false;
      updateupperSW();
      break;

    case CClowerSW:
      lowerSW = value;
      upperSW = false;
      updatelowerSW();
      break;

      // case CCmodwheel:
      //   value = (value * MIDICCTOPOT);
      //   switch (modWheelDepth) {
      //     case 1:
      //       modWheelLevel = ((value) / 5);
      //       upperData[P_fmDepth] = int(modWheelLevel);
      //       lowerData[P_fmDepth] = int(modWheelLevel);
      //       break;

      //     case 2:
      //       modWheelLevel = ((value) / 4);
      //       upperData[P_fmDepth] = int(modWheelLevel);
      //       lowerData[P_fmDepth] = int(modWheelLevel);
      //       break;

      //     case 3:
      //       modWheelLevel = ((value) / 3.5);
      //       upperData[P_fmDepth] = int(modWheelLevel);
      //       lowerData[P_fmDepth] = int(modWheelLevel);
      //       break;

      //     case 4:
      //       modWheelLevel = ((value) / 3);
      //       upperData[P_fmDepth] = int(modWheelLevel);
      //       lowerData[P_fmDepth] = int(modWheelLevel);
      //       break;

      //     case 5:
      //       modWheelLevel = ((value) / 2.5);
      //       upperData[P_fmDepth] = int(modWheelLevel);
      //       lowerData[P_fmDepth] = int(modWheelLevel);
      //       break;

      //     case 6:
      //       modWheelLevel = ((value) / 2);
      //       upperData[P_fmDepth] = int(modWheelLevel);
      //       lowerData[P_fmDepth] = int(modWheelLevel);
      //       break;

      //     case 7:
      //       modWheelLevel = ((value) / 1.75);
      //       upperData[P_fmDepth] = int(modWheelLevel);
      //       lowerData[P_fmDepth] = int(modWheelLevel);
      //       break;

      //     case 8:
      //       modWheelLevel = ((value) / 1.5);
      //       upperData[P_fmDepth] = int(modWheelLevel);
      //       lowerData[P_fmDepth] = int(modWheelLevel);
      //       break;

      //     case 9:
      //       modWheelLevel = ((value) / 1.25);
      //       upperData[P_fmDepth] = int(modWheelLevel);
      //       lowerData[P_fmDepth] = int(modWheelLevel);
      //       break;

      //     case 10:
      //       modWheelLevel = (value);
      //       upperData[P_fmDepth] = int(modWheelLevel);
      //       lowerData[P_fmDepth] = int(modWheelLevel);
      //       break;
      //   }
      //   break;

      // case CCallnotesoff:
      //   allNotesOff();
      //   break;
  }
}

void myProgramChange(byte channel, byte program) {
  state = PATCH;
  patchNo = program + 1;
  recallPatch(patchNo);
  state = PARAMETER;
}

void myAfterTouch(byte channel, byte value) {
  // afterTouch = int(value * MIDICCTOPOT);
  // switch (upperData[P_AfterTouchDest]) {
  //   case 1:
  //     upperData[P_fmDepth] = afterTouch;
  //     break;
  //   case 2:
  //     upperData[P_filterCutoff] = (oldfilterCutoffU + afterTouch);
  //     if (afterTouch < 10) {
  //       upperData[P_filterCutoff] = oldfilterCutoffU;
  //     }
  //     if (upperData[P_filterCutoff] > 1023) {
  //       upperData[P_filterCutoff] = 1023;
  //     }
  //     break;
  //   case 3:
  //     upperData[P_filterLFO] = afterTouch;
  //     break;
  //   case 4:
  //     upperData[P_amDepth] = afterTouch;
  //     break;
  // }
  // switch (lowerData[P_AfterTouchDest]) {
  //   case 1:
  //     lowerData[P_fmDepth] = afterTouch;
  //     break;
  //   case 2:
  //     lowerData[P_filterCutoff] = (oldfilterCutoffL + afterTouch);
  //     if (afterTouch < 10) {
  //       lowerData[P_filterCutoff] = oldfilterCutoffL;
  //     }
  //     if (lowerData[P_filterCutoff] > 1023) {
  //       lowerData[P_filterCutoff] = 1023;
  //     }
  //     break;
  //   case 3:
  //     lowerData[P_filterLFO] = afterTouch;
  //     break;
  //   case 4:
  //     lowerData[P_amDepth] = afterTouch;
  //     break;
  // }
}

void recallPatch(int patchNo) {
  allNotesOff();
  File patchFile = SD.open(String(patchNo).c_str());
  if (!patchFile) {
    //Serial.println("File not found");
  } else {
    String data[NO_OF_PARAMS];  //Array of data read in
    recallPatchData(patchFile, data);
    setCurrentPatchData(data);
    patchFile.close();
    if (upperSW) {
      storeLastPatchU(patchNoU);
      upperpatchtag = patchNoU;
    } else {
      storeLastPatchL(patchNoL);
      lowerpatchtag = patchNoL;
    }
  }
}

void setCurrentPatchData(String data[]) {
  if (upperSW) {
    patchNameU = data[0];
    upperData[0] = 1;
    upperData[P_pwLFO] = data[1].toInt();               // pwLFOU
    upperData[P_fmDepth] = data[2].toInt();             // fmDepthU
    upperData[P_osc2PW] = data[3].toInt();              // osc2PWU
    upperData[P_osc2PWM] = data[4].toInt();             // osc2PWMU
    upperData[P_osc1PW] = data[5].toInt();              // osc1PWU
    upperData[P_osc1PWM] = data[6].toInt();             // osc1PWMU
    upperData[P_osc1Range] = data[7].toInt();           // osc1RangeU
    upperData[P_osc2Range] = data[8].toInt();           // osc2RangeU
    upperData[P_osc2Interval] = data[9].toInt();        // osc2IntervalU
    upperData[P_glideTime] = data[10].toInt();          // glideTimeU
    upperData[P_osc2Detune] = data[11].toInt();         // osc2DetuneU
    upperData[P_noiseLevel] = data[12].toInt();         // noiseLevelU
    upperData[P_osc2SawLevel] = data[13].toInt();       // osc2SawLevelU
    upperData[P_osc1SawLevel] = data[14].toInt();       // osc1SawLevelU
    upperData[P_osc2PulseLevel] = data[15].toInt();     // osc2PulseLevelU
    upperData[P_osc1PulseLevel] = data[16].toInt();     // osc1PulseLevelU
    upperData[P_filterCutoff] = data[17].toInt();       // filterCutoffU
    upperData[P_filterLFO] = data[18].toInt();          // filterLFOU
    upperData[P_filterRes] = data[19].toInt();          // filterResU
    upperData[P_filterType] = data[20].toInt();         // filterTypeU
    upperData[P_modWheelDepth] = data[21].toInt();      // P_modWheelDepthU
    upperData[P_effectsMix] = data[22].toInt();         // effectsMixU
    upperData[P_LFODelayGo] = data[23].toInt();         // LFODelayGoU
    upperData[P_filterEGlevel] = data[24].toInt();      // filterEGlevelU
    upperData[P_LFORate] = data[25].toInt();            // LFORateU
    upperData[P_LFOWaveform] = data[26].toInt();        // LFOWaveformU
    upperData[P_filterAttack] = data[27].toInt();       // filterAttackU
    upperData[P_filterDecay] = data[28].toInt();        // filterDecayU
    upperData[P_filterSustain] = data[29].toInt();      // filterSustainU
    upperData[P_filterRelease] = data[30].toInt();      // filterReleaseU
    upperData[P_ampAttack] = data[31].toInt();          // ampAttackU
    upperData[P_ampDecay] = data[32].toInt();           // ampDecayU
    upperData[P_ampSustain] = data[33].toInt();         // ampSustainU
    upperData[P_ampRelease] = data[34].toInt();         // ampReleaseU
    upperData[P_volumeControl] = data[35].toInt();      // volumeControlU
    upperData[P_glideSW] = data[36].toInt();            // glideSWU
    upperData[P_keytrack] = data[37].toInt();           // keytrackU
    upperData[P_filterPoleSW] = data[38].toInt();       // filterPoleSWU
    upperData[P_filterLoop] = data[39].toInt();         // filterLoopU
    upperData[P_filterEGinv] = data[40].toInt();        // filterEGinvU
    upperData[P_filterVel] = data[41].toInt();          // filterVelU
    upperData[P_vcaLoop] = data[42].toInt();            // vcaLoopU
    upperData[P_vcaVel] = data[43].toInt();             // vcaVelU
    upperData[P_vcaGate] = data[44].toInt();            // vcaGateU
    upperData[P_lfoAlt] = data[45].toInt();             // lfoAltU
    upperData[P_pmDCO2] = data[46].toInt();             // pmDCO2U
    upperData[P_pmFilterEnv] = data[47].toInt();        // pmFilterEnvU
    upperData[P_monoMulti] = data[48].toInt();          // monoMultiU
    upperData[P_modWheelLevel] = data[49].toInt();      // modWheelLevelU
    upperData[P_PitchBendLevel] = data[50].toInt();     // PitchBendLevelU
    upperData[P_amDepth] = data[51].toInt();            // amDepthU
    upperData[P_sync] = data[52].toInt();               // syncU
    upperData[P_effectPot1] = data[53].toInt();         // effectPot1U
    upperData[P_effectPot2] = data[54].toInt();         // effectPot2U
    upperData[P_effectPot3] = data[55].toInt();         // effectPot3U
    upperData[P_oldampAttack] = data[56].toInt();       // oldampAttackU
    upperData[P_oldampDecay] = data[57].toInt();        // oldampDecayU
    upperData[P_oldampSustain] = data[58].toInt();      // oldampSustainU
    upperData[P_oldampRelease] = data[59].toInt();      // oldampReleaseU
    upperData[P_AfterTouchDest] = data[60].toInt();     // AfterTouchDestU
    upperData[P_filterLogLin] = data[61].toInt();       // filterLogLinU
    upperData[P_ampLogLin] = data[62].toInt();          // ampLogLinU
    upperData[P_osc2TriangleLevel] = data[63].toInt();  // osc2TriangleLevelU
    upperData[P_osc1SubLevel] = data[64].toInt();       // osc1SubLevelU
    upperData[P_keyboardMode] = data[65].toInt();       // keyTrackSWU
    upperData[P_LFODelay] = data[66].toInt();           // LFODelayU
    upperData[P_effectNum] = data[67].toInt();          // effectNumU
    upperData[P_effectBank] = data[68].toInt();         // effectBankU
    upperData[P_pmDestDCO1] = data[69].toInt();         // pmDestDCO1U
    upperData[P_pmDestFilter] = data[70].toInt();       // pmDestFilterU
    upperData[P_lfoMultiplier] = data[71].toInt();      // lfoMultiplierU
    upperData[P_NotePriority] = data[72].toInt();       // NotePriorityU
    upperData[P_keytrackSW] = data[73].toInt();         // keyTrackSWU

    oldfilterCutoffU = upperData[P_filterCutoff];

  } else {
    patchNameL = data[0];
    lowerData[0] = 0;
    lowerData[P_pwLFO] = data[1].toInt();               // pwLFOL
    lowerData[P_fmDepth] = data[2].toInt();             // fmDepthL
    lowerData[P_osc2PW] = data[3].toInt();              // osc2PWL
    lowerData[P_osc2PWM] = data[4].toInt();             // osc2PWML
    lowerData[P_osc1PW] = data[5].toInt();              // osc1PWL
    lowerData[P_osc1PWM] = data[6].toInt();             // osc1PWML
    lowerData[P_osc1Range] = data[7].toInt();           // osc1RangeL
    lowerData[P_osc2Range] = data[8].toInt();           // osc2RangeL
    lowerData[P_osc2Interval] = data[9].toInt();        // osc2IntervalL
    lowerData[P_glideTime] = data[10].toInt();          // glideTimeL
    lowerData[P_osc2Detune] = data[11].toInt();         // osc2DetuneL
    lowerData[P_noiseLevel] = data[12].toInt();         // noiseLevelL
    lowerData[P_osc2SawLevel] = data[13].toInt();       // osc2SawLevelL
    lowerData[P_osc1SawLevel] = data[14].toInt();       // osc1SawLevelL
    lowerData[P_osc2PulseLevel] = data[15].toInt();     // osc2PulseLevelL
    lowerData[P_osc1PulseLevel] = data[16].toInt();     // osc1PulseLevelL
    lowerData[P_filterCutoff] = data[17].toInt();       // filterCutoffL
    lowerData[P_filterLFO] = data[18].toInt();          // filterLFOL
    lowerData[P_filterRes] = data[19].toInt();          // filterResL
    lowerData[P_filterType] = data[20].toInt();         // filterTypeL
    lowerData[P_modWheelDepth] = data[21].toInt();      // modWheelDepthL
    lowerData[P_effectsMix] = data[22].toInt();         // effectsMixL
    lowerData[P_LFODelayGo] = data[23].toInt();         // LFODelayGoL
    lowerData[P_filterEGlevel] = data[24].toInt();      // filterEGlevelL
    lowerData[P_LFORate] = data[25].toInt();            // LFORateL
    lowerData[P_LFOWaveform] = data[26].toInt();        // LFOWaveformL
    lowerData[P_filterAttack] = data[27].toInt();       // filterAttackL
    lowerData[P_filterDecay] = data[28].toInt();        // filterDecayL
    lowerData[P_filterSustain] = data[29].toInt();      // filterSustainL
    lowerData[P_filterRelease] = data[30].toInt();      // filterReleaseL
    lowerData[P_ampAttack] = data[31].toInt();          // ampAttackL
    lowerData[P_ampDecay] = data[32].toInt();           // ampDecayL
    lowerData[P_ampSustain] = data[33].toInt();         // ampSustainL
    lowerData[P_ampRelease] = data[34].toInt();         // ampReleaseL
    lowerData[P_volumeControl] = data[35].toInt();      // volumeControlL
    lowerData[P_glideSW] = data[36].toInt();            // glideSWL
    lowerData[P_keytrack] = data[37].toInt();           // keytrackL
    lowerData[P_filterPoleSW] = data[38].toInt();       // filterPoleSWL
    lowerData[P_filterLoop] = data[39].toInt();         // filterLoopL
    lowerData[P_filterEGinv] = data[40].toInt();        // filterEGinvL
    lowerData[P_filterVel] = data[41].toInt();          // filterVelL
    lowerData[P_vcaLoop] = data[42].toInt();            // vcaLoopL
    lowerData[P_vcaVel] = data[43].toInt();             // vcaVelL
    lowerData[P_vcaGate] = data[44].toInt();            // vcaGateL
    lowerData[P_lfoAlt] = data[45].toInt();             // lfoAltL
    lowerData[P_pmDCO2] = data[46].toInt();             // pmDCO2L
    lowerData[P_pmFilterEnv] = data[47].toInt();        // pmFilterEnvL
    lowerData[P_monoMulti] = data[48].toInt();          // monoMultiL
    lowerData[P_modWheelLevel] = data[49].toInt();      // modWheelLevelL
    lowerData[P_PitchBendLevel] = data[50].toInt();     // PitchBendLevelL
    lowerData[P_amDepth] = data[51].toInt();            // amDepthL
    lowerData[P_sync] = data[52].toInt();               // syncL
    lowerData[P_effectPot1] = data[53].toInt();         // effectPot1L
    lowerData[P_effectPot2] = data[54].toInt();         // effectPot2L
    lowerData[P_effectPot3] = data[55].toInt();         // effectPot3L
    lowerData[P_oldampAttack] = data[56].toInt();       // oldampAttackL
    lowerData[P_oldampDecay] = data[57].toInt();        // oldampDecayL
    lowerData[P_oldampSustain] = data[58].toInt();      // oldampSustainL
    lowerData[P_oldampRelease] = data[59].toInt();      // oldampReleaseL
    lowerData[P_AfterTouchDest] = data[60].toInt();     // AfterTouchDestL
    lowerData[P_filterLogLin] = data[61].toInt();       // filterLogLinL
    lowerData[P_ampLogLin] = data[62].toInt();          // ampLogLinL
    lowerData[P_osc2TriangleLevel] = data[63].toInt();  // osc2TriangleLevelL
    lowerData[P_osc1SubLevel] = data[64].toInt();       // osc1SubLevelL
    lowerData[P_keyboardMode] = data[65].toInt();       // keyTrackSWL
    lowerData[P_LFODelay] = data[66].toInt();           // LFODelayL
    lowerData[P_effectNum] = data[67].toInt();          // effectNumL
    lowerData[P_effectBank] = data[68].toInt();         // effectBankL
    lowerData[P_pmDestDCO1] = data[69].toInt();         // pmDestDCO1L
    lowerData[P_pmDestFilter] = data[70].toInt();       // pmDestFilterL
    lowerData[P_lfoMultiplier] = data[71].toInt();      // lfoMultiplierL
    lowerData[P_NotePriority] = data[72].toInt();       // NotePriorityL
    lowerData[P_keytrackSW] = data[73].toInt();         // keyTrackSWL

    oldfilterCutoffL = lowerData[P_filterCutoff];

    if (wholemode) {
      patchNameU = data[0];
      upperData[0] = 1;
      upperData[P_pwLFO] = data[1].toInt();               //
      upperData[P_fmDepth] = data[2].toInt();             // fmDepthU
      upperData[P_osc2PW] = data[3].toInt();              // osc2PWU
      upperData[P_osc2PWM] = data[4].toInt();             // osc2PWMU
      upperData[P_osc1PW] = data[5].toInt();              // osc1PWU
      upperData[P_osc1PWM] = data[6].toInt();             // osc1PWMU
      upperData[P_osc1Range] = data[7].toInt();           // osc1RangeU
      upperData[P_osc2Range] = data[8].toInt();           // osc2RangeU
      upperData[P_osc2Interval] = data[9].toInt();        // osc2IntervalU
      upperData[P_glideTime] = data[10].toInt();          // glideTimeU
      upperData[P_osc2Detune] = data[11].toInt();         // osc2DetuneU
      upperData[P_noiseLevel] = data[12].toInt();         // noiseLevelU
      upperData[P_osc2SawLevel] = data[13].toInt();       // osc2SawLevelU
      upperData[P_osc1SawLevel] = data[14].toInt();       // osc1SawLevelU
      upperData[P_osc2PulseLevel] = data[15].toInt();     // osc2PulseLevelU
      upperData[P_osc1PulseLevel] = data[16].toInt();     // osc1PulseLevelU
      upperData[P_filterCutoff] = data[17].toInt();       // upperData[17]
      upperData[P_filterLFO] = data[18].toInt();          // filterLFOU
      upperData[P_filterRes] = data[19].toInt();          // filterResU
      upperData[P_filterType] = data[20].toInt();         // filterTypeU
      upperData[P_modWheelDepth] = data[21].toInt();      // modWheelDepthU
      upperData[P_effectsMix] = data[22].toInt();         // effectsMixU
      upperData[P_LFODelayGo] = data[23].toInt();         // LFODelayGoU
      upperData[P_filterEGlevel] = data[24].toInt();      // filterEGlevelU
      upperData[P_LFORate] = data[25].toInt();            // LFORateU
      upperData[P_LFOWaveform] = data[26].toInt();        // LFOWaveformU
      upperData[P_filterAttack] = data[27].toInt();       // filterAttackU
      upperData[P_filterDecay] = data[28].toInt();        // filterDecayU
      upperData[P_filterSustain] = data[29].toInt();      // filterSustainU
      upperData[P_filterRelease] = data[30].toInt();      // filterReleaseU
      upperData[P_ampAttack] = data[31].toInt();          // ampAttackU
      upperData[P_ampDecay] = data[32].toInt();           // ampDecayU
      upperData[P_ampSustain] = data[33].toInt();         // ampSustainU
      upperData[P_ampRelease] = data[34].toInt();         // ampReleaseU
      upperData[P_volumeControl] = data[35].toInt();      // volumeControlU
      upperData[P_glideSW] = data[36].toInt();            // glideSWU
      upperData[P_keytrack] = data[37].toInt();           // keytrackU
      upperData[P_filterPoleSW] = data[38].toInt();       // filterPoleSWU
      upperData[P_filterLoop] = data[39].toInt();         // filterLoopU
      upperData[P_filterEGinv] = data[40].toInt();        // filterEGinvU
      upperData[P_filterVel] = data[41].toInt();          // filterVelU
      upperData[P_vcaLoop] = data[42].toInt();            // vcaLoopU
      upperData[P_vcaVel] = data[43].toInt();             // vcaVelU
      upperData[P_vcaGate] = data[44].toInt();            // vcaGateU
      upperData[P_lfoAlt] = data[45].toInt();             // lfoAltU
      upperData[P_pmDCO2] = data[46].toInt();             // pmDCO2U
      upperData[P_pmFilterEnv] = data[47].toInt();        // pmFilterEnvU
      upperData[P_monoMulti] = data[48].toInt();          // monoMultiU
      upperData[P_modWheelLevel] = data[49].toInt();      // modWheelLevelU
      upperData[P_PitchBendLevel] = data[50].toInt();     // PitchBendLevelU
      upperData[P_amDepth] = data[51].toInt();            // amDepthU
      upperData[P_sync] = data[52].toInt();               // syncU
      upperData[P_effectPot1] = data[53].toInt();         // effectPot1U
      upperData[P_effectPot2] = data[54].toInt();         // effectPot2U
      upperData[P_effectPot3] = data[55].toInt();         // effectPot3U
      upperData[P_oldampAttack] = data[56].toInt();       // oldampAttackU
      upperData[P_oldampDecay] = data[57].toInt();        // oldampDecayU
      upperData[P_oldampSustain] = data[58].toInt();      // oldampSustainU
      upperData[P_oldampRelease] = data[59].toInt();      // oldampReleaseU
      upperData[P_AfterTouchDest] = data[60].toInt();     // AfterTouchDestU
      upperData[P_filterLogLin] = data[61].toInt();       // filterLogLinU
      upperData[P_ampLogLin] = data[62].toInt();          // ampLogLinU
      upperData[P_osc2TriangleLevel] = data[63].toInt();  // osc2TriangleLevelU
      upperData[P_osc1SubLevel] = data[64].toInt();       // osc1SubLevelU
      upperData[P_keyboardMode] = data[65].toInt();       // keyTrackSWU
      upperData[P_LFODelay] = data[66].toInt();           // LFODelayU
      upperData[P_effectNum] = data[67].toInt();          // effectNumU
      upperData[P_effectBank] = data[68].toInt();         // effectBankU
      upperData[P_pmDestDCO1] = data[69].toInt();         // pmDestDCO1U
      upperData[P_pmDestFilter] = data[70].toInt();       // pmDestFilterU
      upperData[P_lfoMultiplier] = data[71].toInt();      // lfoMultiplierU
      upperData[P_NotePriority] = data[72].toInt();       // NotePriorityU
      upperData[P_keytrackSW] = data[73].toInt();         // keyTrackSWU

      oldfilterCutoffU = upperData[P_filterCutoff];
    }
  }
  // Convert and send the SysEx message
  // convertData();
  // sendSysExMessage();
  sendi2cMessage();
  updatePatchname();
}

void sendi2cMessage() {

  if (upperSW) {

    Wire.beginTransmission(8);
    for (int i = 0; i < 76; i++) {
      Wire.write((uint8_t)(upperData[i] >> 8));    // High byte
      Wire.write((uint8_t)(upperData[i] & 0xFF));  // Low byte
    }
    Wire.endTransmission();

  } else {

    Wire.beginTransmission(8);
    for (int i = 0; i < 76; i++) {
      Wire.write((uint8_t)(lowerData[i] >> 8));    // High byte
      Wire.write((uint8_t)(lowerData[i] & 0xFF));  // Low byte
    }
    Wire.endTransmission();
  }
}

void sendSysExMessage() {

  // SysEx start byte

  byte startByte = 0xF0;
  // SysEx end byte
  byte endByte = 0xF7;
  // Manufacturer ID (3 bytes for non-real-time messages)
  byte manufacturerID[] = { 0x7D, 0x00, 0x00 };  // 0x7D is reserved for educational use

  // Begin the SysEx message
  //MIDI7.sendSysEx(startByte + sizeof(manufacturerID) + sysexDataLength + endByte, sysexData, true);

  // Alternatively, you can construct the message manually:
  Serial7.write(startByte);
  Serial7.write(manufacturerID, sizeof(manufacturerID));
  Serial7.write(sysexData, sysexDataLength);
  Serial7.write(endByte);

  Serial.println("SysEx message sent.");
}

void convertData() {

  if (upperSW) {

    sysexData[0] = upperData[0];

    for (int i = 0; i < originalDataLength; i++) {

      sysexData[2 * i] = (upperData[i] >> 8) & 0xFF;
      Serial.print(sysexData[2 * i]);
      Serial.print(" ");
      // Low byte contains the lower 7 bits (7-0)
      sysexData[2 * i + 1] = upperData[i] & 0xFF;
      Serial.print(sysexData[2 * i + 1]);
      Serial.print(" ");
    }

  } else {

    sysexData[0] = lowerData[0];

    for (int i = 0; i < originalDataLength; i++) {
      sysexData[2 * i] = (lowerData[i] >> 7) & 0x7F;
      Serial.print(sysexData[2 * i]);
      Serial.print(" ");
      // Low byte contains the lower 7 bits (7-0)
      sysexData[2 * i + 1] = lowerData[i] & 0x7F;
      Serial.print(sysexData[2 * i + 1]);
      Serial.print(" ");
    }
  }
}

void setAllButtons() {
  // updatefilterPoleSwitch(0);
  // updatefilterLoop(0);
  // updatefilterEGinv(0);
  // updatefilterVel(0);
  // updatevcaLoop(0);
  // updatevcaVel(0);
  // updatevcaGate(0);
  // updatelfoAlt(0);
  // updateglideSW(0);
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
           + "," + String(upperData[P_pmDestDCO1]) + "," + String(upperData[P_pmDestFilter]) + "," + String(upperData[P_lfoMultiplier]) + "," + String(upperData[P_NotePriority]) + "," + String(upperData[P_keytrackSW]);
  } else {
    return patchNameL + "," + String(upperData[P_pwLFO]) + "," + String(lowerData[P_fmDepth]) + "," + String(lowerData[P_osc2PW]) + "," + String(lowerData[P_osc2PWM])
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
           + "," + String(lowerData[P_pmDestDCO1]) + "," + String(lowerData[P_pmDestFilter]) + "," + String(lowerData[P_lfoMultiplier]) + "," + String(lowerData[P_NotePriority]) + "," + String(lowerData[P_keytrackSW]);
  }
}

void checkMux() {

  mux1Read = adc->adc1->analogRead(MUX1_S);
  mux2Read = adc->adc1->analogRead(MUX2_S);
  mux3Read = adc->adc1->analogRead(MUX3_S);

  if (mux1Read > (mux1ValuesPrev[muxInput] + QUANTISE_FACTOR) || mux1Read < (mux1ValuesPrev[muxInput] - QUANTISE_FACTOR)) {
    mux1ValuesPrev[muxInput] = mux1Read;
    switch (muxInput) {
      case MUX1_glideTime:
        myControlChange(midiChannel, CCglideTime, mux1Read);
        break;
      case MUX1_osc1SawLevel:
        myControlChange(midiChannel, CCosc1SawLevel, mux1Read);
        break;
      case MUX1_osc1PulseLevel:
        myControlChange(midiChannel, CCosc1PulseLevel, mux1Read);
        break;
      case MUX1_osc1PW:
        myControlChange(midiChannel, CCosc1PW, mux1Read);
        break;
      case MUX1_osc1PWM:
        myControlChange(midiChannel, CCosc1PWM, mux1Read);
        break;
      case MUX1_osc2Detune:
        myControlChange(midiChannel, CCosc2Detune, mux1Read);
        break;
      case MUX1_osc2interval:
        mux1Read = map(mux1Read, 0, 1023, 0, 12);
        myControlChange(midiChannel, CCosc2Interval, mux1Read);
        break;
      case MUX1_fmDepth:
        myControlChange(midiChannel, CCfmDepth, mux1Read);
        break;
      case MUX1_osc1SubLevel:
        myControlChange(midiChannel, CCosc1SubLevel, mux1Read);
        break;
      case MUX1_osc2SawLevel:
        myControlChange(midiChannel, CCosc2SawLevel, mux1Read);
        break;
      case MUX1_osc2PulseLevel:
        myControlChange(midiChannel, CCosc2PulseLevel, mux1Read);
        break;
      case MUX1_osc2TriangleLevel:
        myControlChange(midiChannel, CCosc2TriangleLevel, mux1Read);
        break;
      case MUX1_osc2PW:
        myControlChange(midiChannel, CCosc2PW, mux1Read);
        break;
      case MUX1_osc2PWM:
        myControlChange(midiChannel, CCosc2PWM, mux1Read);
        break;
    }
  }

  if (mux2Read > (mux2ValuesPrev[muxInput] + QUANTISE_FACTOR) || mux2Read < (mux2ValuesPrev[muxInput] - QUANTISE_FACTOR)) {
    mux2ValuesPrev[muxInput] = mux2Read;
    switch (muxInput) {
      case MUX2_filterAttack:
        myControlChange(midiChannel, CCfilterAttack, mux2Read);
        break;
      case MUX2_filterDecay:
        myControlChange(midiChannel, CCfilterDecay, mux2Read);
        break;
      case MUX2_filterSustain:
        myControlChange(midiChannel, CCfilterSustain, mux2Read);
        break;
      case MUX2_filterRelease:
        myControlChange(midiChannel, CCfilterRelease, mux2Read);
        break;
      case MUX2_ampAttack:
        myControlChange(midiChannel, CCampAttack, mux2Read);
        break;
      case MUX2_ampDecay:
        myControlChange(midiChannel, CCampDecay, mux2Read);
        break;
      case MUX2_ampSustain:
        myControlChange(midiChannel, CCampSustain, mux2Read);
        break;
      case MUX2_ampRelease:
        myControlChange(midiChannel, CCampRelease, mux2Read);
        break;
      case MUX2_filterLFO:
        myControlChange(midiChannel, CCfilterLFO, mux2Read);
        break;
      case MUX2_keyTrack:
        myControlChange(midiChannel, CCkeyTrack, mux2Read);
        break;
      case MUX2_filterCutoff:
        myControlChange(midiChannel, CCfilterCutoff, mux2Read);
        break;
      case MUX2_filterRes:
        myControlChange(midiChannel, CCfilterRes, mux2Read);
        break;
      case MUX2_filterEGlevel:
        myControlChange(midiChannel, CCfilterEGlevel, mux2Read);
        break;
    }
  }

  if (mux3Read > (mux3ValuesPrev[muxInput] + QUANTISE_FACTOR) || mux3Read < (mux3ValuesPrev[muxInput] - QUANTISE_FACTOR)) {
    mux3ValuesPrev[muxInput] = mux3Read;
    switch (muxInput) {
      case MUX3_pitchBendDepth:
        mux3Read = map(mux3Read, 0, 1023, 0, 12);
        myControlChange(midiChannel, CCPitchBend, mux3Read);
        break;
      case MUX3_effectMix:
        myControlChange(midiChannel, CCeffectsMix, mux3Read);
        break;
      case MUX3_volumeControl:
        myControlChange(midiChannel, CCvolumeControl, mux3Read);
        break;
      case MUX3_amplifierLFO:
        myControlChange(midiChannel, CCamDepth, mux3Read);
        break;
      case MUX3_noiseLevel:
        myControlChange(midiChannel, CCnoiseLevel, mux3Read);
        break;
      case MUX3_pwLFO:
        myControlChange(midiChannel, CCpwLFO, mux3Read);
        break;
      case MUX3_LFORate:
        myControlChange(midiChannel, CCLFORate, mux3Read);
        break;
      case MUX3_LFODelay:
        myControlChange(midiChannel, CCLFODelay, mux3Read);
        break;
      case MUX3_modWheelDepth:
        myControlChange(midiChannel, CCmodWheelDepth, mux3Read);
        break;
      case MUX3_effectPot1:
        myControlChange(midiChannel, CCeffectPot1, mux3Read);
        break;
      case MUX3_effectPot2:
        myControlChange(midiChannel, CCeffectPot2, mux3Read);
        break;
      case MUX3_effectPot3:
        myControlChange(midiChannel, CCeffectPot3, mux3Read);
        break;
      case MUX3_PM_DCO2:
        myControlChange(midiChannel, CCPM_DCO2, mux3Read);
        break;
      case MUX3_PM_FilterEnv:
        myControlChange(midiChannel, CCPM_FilterEnv, mux3Read);
        break;
    }
  }

  muxInput++;
  if (muxInput >= MUXCHANNELS)
    muxInput = 0;

  digitalWriteFast(MUX_0, muxInput & B0001);
  digitalWriteFast(MUX_1, muxInput & B0010);
  digitalWriteFast(MUX_2, muxInput & B0100);
  digitalWriteFast(MUX_3, muxInput & B1000);
}

void midiCCOut(byte cc, byte value) {
  MIDI.sendControlChange(cc, value, midiChannel);  //MIDI DIN is set to Out
}

void midiCCOut71(byte cc, byte value) {
  Serial.print("Sent on channel 1 from the controller ");
  Serial.println(value);
  MIDI7.sendControlChange(cc, value, 1);  //MIDI DIN is set to Out
}

void midiCCOut72(byte cc, byte value) {
  Serial.print("Sent on channel 2 from the controller ");
  Serial.println(value);
  MIDI7.sendControlChange(cc, value, 2);  //MIDI DIN is set to Out
}

void midiCCOut73(byte cc, byte value) {
  // Serial.print("Sent on channel 3 from the controller ");
  // Serial.println(value);
  MIDI7.sendControlChange(cc, value, 3);  //MIDI DIN is set to Out
}

void midiCCOut61(byte cc, byte value) {
  MIDI6.sendControlChange(cc, value, 1);  //MIDI DIN is set to Out channel 1
}

void midiCCOut62(byte cc, byte value) {
  MIDI6.sendControlChange(cc, value, 2);  //MIDI DIN is set to Out channel 2
}

void outputDAC(int CHIP_SELECT, uint32_t sample_data1, uint32_t sample_data2, uint32_t sample_data3, uint32_t sample_data4) {
  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE1));
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
      sample_data1 = (channel_a & 0xFFF0000F) | (((int(upperData[P_osc1SawLevel] * MULT2V)) & 0xFFFF) << 4);
      sample_data2 = (channel_c & 0xFFF0000F) | (((int(lowerData[P_osc1SawLevel] * MULT2V)) & 0xFFFF) << 4);

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(upperData[P_filterDecay] * MULT5V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(lowerData[P_filterDecay] * MULT5V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 2:
      sample_data1 = (channel_a & 0xFFF0000F) | (((int(upperData[P_osc1PulseLevel] * MULT2V)) & 0xFFFF) << 4);
      sample_data2 = (channel_c & 0xFFF0000F) | (((int(lowerData[P_osc1PulseLevel] * MULT2V)) & 0xFFFF) << 4);

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(upperData[P_filterSustain] * MULT5V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(lowerData[P_filterSustain] * MULT5V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 3:
      sample_data1 = (channel_a & 0xFFF0000F) | (((int(upperData[P_osc1SubLevel] * MULT2V)) & 0xFFFF) << 4);
      sample_data2 = (channel_c & 0xFFF0000F) | (((int(lowerData[P_osc1SubLevel] * MULT2V)) & 0xFFFF) << 4);

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
      sample_data1 = (channel_a & 0xFFF0000F) | (((int(upperData[P_osc2SawLevel] * MULT2V)) & 0xFFFF) << 4);
      sample_data2 = (channel_c & 0xFFF0000F) | (((int(lowerData[P_osc2SawLevel] * MULT2V)) & 0xFFFF) << 4);

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(upperData[P_ampSustain] * MULT5V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(lowerData[P_ampSustain] * MULT5V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 7:
      sample_data1 = (channel_a & 0xFFF0000F) | (((int(upperData[P_osc2PulseLevel] * MULT2V)) & 0xFFFF) << 4);
      sample_data2 = (channel_c & 0xFFF0000F) | (((int(lowerData[P_osc2PulseLevel] * MULT2V)) & 0xFFFF) << 4);

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(upperData[P_ampRelease] * MULT5V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(lowerData[P_ampRelease] * MULT5V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 8:
      sample_data1 = (channel_a & 0xFFF0000F) | (((int(upperData[P_osc2TriangleLevel] * MULT2V)) & 0xFFFF) << 4);
      sample_data2 = (channel_c & 0xFFF0000F) | (((int(lowerData[P_osc2TriangleLevel] * MULT2V)) & 0xFFFF) << 4);

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
      sample_data1 = (channel_a & 0xFFF0000F) | (((int(upperData[P_effectsMix] * MULT2V)) & 0xFFFF) << 4);
      sample_data2 = (channel_c & 0xFFF0000F) | (((int(lowerData[P_effectsMix] * MULT2V)) & 0xFFFF) << 4);

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(upperData[P_filterRes] * MULT5V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(lowerData[P_filterRes] * MULT5V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 11:
      switch (upperData[P_LFODelayGo]) {
        case 1:
          sample_data1 = (channel_a & 0xFFF0000F) | (((int(upperData[P_fmDepth] * MULT2V)) & 0xFFFF) << 4);
          break;

        case 0:
          sample_data1 = (channel_a & 0xFFF0000F) | ((0 & 0xFFFF) << 4);
          break;
      }
      switch (lowerData[P_LFODelayGo]) {
        case 1:
          sample_data2 = (channel_c & 0xFFF0000F) | (((int(lowerData[P_fmDepth] * MULT2V)) & 0xFFFF) << 4);
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

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(upperData[P_LFOWaveform] * MULT5V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(lowerData[P_LFOWaveform] * MULT5V)) & 0xFFFF) << 4);
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
      sample_data2 = (channel_c & 0xFFF0000F) | (((upperData[P_pwLFO] * MULT5V) & 0xFFFF) << 4);

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(upperData[P_effectPot3] * MULT33V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(lowerData[P_effectPot3] * MULT33V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;
  }
  delayMicroseconds(800);
  digitalWriteFast(DEMUX_EN_1, HIGH);

  muxOutput++;
  if (muxOutput >= DEMUXCHANNELS)

    muxOutput = 0;

  digitalWriteFast(DEMUX_0, muxOutput & B0001);
  digitalWriteFast(DEMUX_1, muxOutput & B0010);
  digitalWriteFast(DEMUX_2, muxOutput & B0100);
  digitalWriteFast(DEMUX_3, muxOutput & B1000);
}

void checkEeprom() {

  // if (oldsplitTrans != splitTrans) {
  //   setTranspose(splitTrans);
  // }

  // if (oldAfterTouchDestU != upperData[P_AfterTouchDest]) {
  //   oldAfterTouchDestU = upperData[P_AfterTouchDest];
  // }

  // if (oldAfterTouchDestL != lowerData[P_AfterTouchDest]) {
  //   oldAfterTouchDestL = lowerData[P_AfterTouchDest];
  // }
}

void showSettingsPage() {
  showSettingsPage(settings::current_setting(), settings::current_setting_value(), state);
}

void checkSwitches() {

  saveButton.update();
  if (saveButton.held()) {
    switch (state) {
      case PARAMETER:
      case PATCH:
        state = DELETE;
        break;
    }
  } else if (saveButton.numClicks() == 1) {
    switch (state) {
      case PARAMETER:
        if (patches.size() < PATCHES_LIMIT) {
          resetPatchesOrdering();  //Reset order of patches from first patch
          patches.push({ patches.size() + 1, INITPATCHNAME });
          state = SAVE;
        }
        break;
      case SAVE:
        //Save as new patch with INITIALPATCH name or overwrite existing keeping name - bypassing patch renaming
        patchName = patches.last().patchName;
        state = PATCH;
        savePatch(String(patches.last().patchNo).c_str(), getCurrentPatchData());
        //showPatchPage(patches.last().patchNo, patches.last().patchName);
        showPatchPage(patches.last().patchNo, patches.last().patchName, "", "");
        patchNo = patches.last().patchNo;
        loadPatches();  //Get rid of pushed patch if it wasn't saved
        setPatchesOrdering(patchNo);
        renamedPatch = "";
        state = PARAMETER;
        break;
      case PATCHNAMING:
        if (renamedPatch.length() > 0) patchName = renamedPatch;  //Prevent empty strings
        state = PATCH;
        savePatch(String(patches.last().patchNo).c_str(), getCurrentPatchData());
        showPatchPage(patches.last().patchNo, patches.last().patchName, "", "");
        //showPatchPage(patches.last().patchNo, patchName);
        patchNo = patches.last().patchNo;
        loadPatches();  //Get rid of pushed patch if it wasn't saved
        setPatchesOrdering(patchNo);
        renamedPatch = "";
        state = PARAMETER;
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
    }
  }

  //Encoder switch
  recallButton.update();
  if (recallButton.held()) {
    //If Recall button held, return to current patch setting
    //which clears any changes made
    state = PATCH;
    //Recall the current patch
    patchNo = patches.first().patchNo;
    recallPatch(patchNo);
    state = PARAMETER;
  } else if (recallButton.numClicks() == 1) {
    switch (state) {
      case PARAMETER:
        state = RECALL;  //show patch list
        break;
      case RECALL:
        state = PATCH;
        //Recall the current patch
        patchNo = patches.first().patchNo;
        recallPatch(patchNo);
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
    }
  }
}

void reinitialiseToPanel() {
  //This sets the current patch to be the same as the current hardware panel state - all the pots
  //The four button controls stay the same state
  //This reinialises the previous hardware values to force a re-read
  muxInput = 0;
  for (int i = 0; i < MUXCHANNELS; i++) {
    mux1ValuesPrev[i] = RE_READ;
    mux2ValuesPrev[i] = RE_READ;
    mux3ValuesPrev[i] = RE_READ;
  }
  patchName = INITPATCHNAME;
  showPatchPage("Initial", "Panel Settings", "", "");
}

void checkEncoder() {
  //Encoder works with relative inc and dec values
  //Detent encoder goes up in 4 steps, hence +/-3

  long encRead = encoder.read();
  if ((encCW && encRead > encPrevious + 3) || (!encCW && encRead < encPrevious - 3)) {
    switch (state) {
      case PARAMETER:
        state = PATCH;
        if (upperSW) {
          patches.push(patches.shift());
          patchNoU = patches.first().patchNo;
          recallPatch(patchNoU);
        } else {
          patches.push(patches.shift());
          patchNoL = patches.first().patchNo;
          recallPatch(patchNoL);
        }
        state = PARAMETER;
        break;
      case RECALL:
        patches.push(patches.shift());
        break;
      case SAVE:
        patches.push(patches.shift());
        break;
      case PATCHNAMING:
        if (charIndex == TOTALCHARS) charIndex = 0;  //Wrap around
        currentCharacter = CHARACTERS[charIndex++];
        showRenamingPage(renamedPatch + currentCharacter);
        break;
      case DELETE:
        patches.push(patches.shift());
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
    encPrevious = encRead;
  } else if ((encCW && encRead < encPrevious - 3) || (!encCW && encRead > encPrevious + 3)) {
    switch (state) {
      case PARAMETER:
        state = PATCH;
        if (upperSW) {
          patches.unshift(patches.pop());
          patchNoU = patches.first().patchNo;
          recallPatch(patchNoU);
        } else {
          patches.unshift(patches.pop());
          patchNoL = patches.first().patchNo;
          recallPatch(patchNoL);
        }
        state = PARAMETER;
        break;
      case RECALL:
        patches.unshift(patches.pop());
        break;
      case SAVE:
        patches.unshift(patches.pop());
        break;
      case PATCHNAMING:
        if (charIndex == -1)
          charIndex = TOTALCHARS - 1;
        currentCharacter = CHARACTERS[charIndex--];
        showRenamingPage(renamedPatch + currentCharacter);
        break;
      case DELETE:
        patches.unshift(patches.pop());
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
    encPrevious = encRead;
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
    lowerSW = 1;
    upperSW = 0;
    myControlChange(midiChannel, CClowerSW, lowerSW);
  }

  if (btnIndex == UPPER_SW && btnType == ROX_PRESSED) {
    lowerSW = 0;
    upperSW = 1;
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
  checkSwitches();
  checkEeprom();
  writeDemux();
  checkMux();
  checkEncoder();
  MIDI.read(midiChannel);
  MIDI6.read(midiChannel);
  MIDI7.read();
  usbMIDI.read(midiChannel);
  octoswitch.update();  // read all the buttons for the Synth
  srp.update();         // update all the LEDs in the buttons
  LFODelayHandle();
}