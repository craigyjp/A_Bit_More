/*
  PolyKit 16 MUX - Firmware Rev 1.4

  Includes code by:
    Dave Benn - Handling MUXs, a few other bits and original inspiration  https://www.notesandvolts.com/2019/01/teensy-synth-part-10-hardware.html

  Arduino IDE
  Tools Settings:
  Board: "Teensy4.1"
  USB Type: "Serial + MIDI"
  CPU Speed: "600"
  Optimize: "Fastest"

  Additional libraries:
    Agileware CircularBuffer available in Arduino libraries manager
    Replacement files are in the Modified Libraries folder and need to be placed in the teensy Audio folder.
*/

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
#include "HWControls.h"
#include "EepromMgr.h"
#include "Settings.h"
#include <ShiftRegister74HC595.h>
#include <RoxMux.h>


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
};

struct VoiceAndNote voices[NO_OF_VOICES] = {
  { -1, -1, 0 },
  { -1, -1, 0 },
  { -1, -1, 0 },
  { -1, -1, 0 },
  { -1, -1, 0 },
  { -1, -1, 0 },
  { -1, -1, 0 },
  { -1, -1, 0 }
};

boolean voiceOn[NO_OF_VOICES] = { false, false, false, false, false, false, false, false };
int prevNote = 0;  //Initialised to middle value
bool notes[88] = { 0 }, initial_loop = 1;
int8_t noteOrder[40] = { 0 }, orderIndx = { 0 };

//USB HOST MIDI Class Compliant
USBHost myusb;
USBHub hub1(myusb);
USBHub hub2(myusb);
MIDIDevice midi1(myusb);


//MIDI 5 Pin DIN
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);  //RX - Pin 0
MIDI_CREATE_INSTANCE(HardwareSerial, Serial6, MIDI6);
MIDI_CREATE_INSTANCE(HardwareSerial, Serial5, MIDI5);


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

ShiftRegister74HC595<8> srp(6, 7, 8);

void setup() {
  SPI.begin();
  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
  setupDisplay();
  setUpSettings();
  setupHardware();

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
  usbMIDI.setHandleNoteOn(DinHandleNoteOn);
  usbMIDI.setHandleNoteOff(DinHandleNoteOff);
  //Serial.println("USB Client MIDI Listening");

  //MIDI 5 Pin DIN
  MIDI.begin();
  MIDI.setHandleControlChange(editControlChange);
  MIDI.setHandleProgramChange(myProgramChange);
  MIDI.setHandleAfterTouchChannel(myAfterTouch);
  MIDI.setHandlePitchBend(DinHandlePitchBend);
  MIDI.setHandleNoteOn(DinHandleNoteOn);
  MIDI.setHandleNoteOff(DinHandleNoteOff);
  MIDI.turnThruOn(midi::Thru::Mode::Off);
  //Serial.println("MIDI In DIN Listening");


  MIDI5.begin();
  MIDI5.setHandleControlChange(panelControlChange);
  MIDI5.turnThruOn(midi::Thru::Mode::Off);

  MIDI6.begin();
  MIDI6.turnThruOn(midi::Thru::Mode::Off);

  //Read Aftertouch from EEPROM, this can be set individually by each patch.
  AfterTouchDestU = getAfterTouchU();
  oldAfterTouchDestU = AfterTouchDestU;
  AfterTouchDestL = getAfterTouchL();
  oldAfterTouchDestL = AfterTouchDestL;

  newsplitPoint = getSplitPoint();

  splitTrans = getSplitTrans();
  setTranspose(splitTrans);

  //Read Pitch Bend Range from EEPROM
  pitchBendRange = getPitchBendRange();

  //Read Mod Wheel Depth from EEPROM
  modWheelDepth = getModWheelDepth();

  //Read Encoder Direction from EEPROM
  encCW = getEncoderDir();
  monoMultiL = getMonoMultiL();
  oldmonoMultiL = monoMultiL;
  monoMultiU = getMonoMultiU();
  oldmonoMultiU = monoMultiU;
  filterLogLinU = getFilterEnvU();
  oldfilterLogLinU = filterLogLinU;
  filterLogLinL = getFilterEnvL();
  oldfilterLogLinL = filterLogLinL;
  ampLogLinU = getAmpEnvU();
  oldampLogLinU = ampLogLinU;
  ampLogLinL = getAmpEnvL();
  oldampLogLinL = ampLogLinL;
  keyTrackSWU = getKeyTrackU();
  oldkeyTrackSWU = keyTrackSWU;
  keyTrackSWL = getKeyTrackL();
  oldkeyTrackSWL = keyTrackSWL;

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

  patchNoU = getLastPatchU();
  patchNoL = getLastPatchL();
  upperSW = 1;
  recallPatch(patchNoU);
  upperSW = 0;
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
  if (monoMultiU && !LFODelayGoU) {
    if (oldnumberOfNotesU < numberOfNotesU) {
      previousMillisU = currentMillisU;
      oldnumberOfNotesU = numberOfNotesU;
    }
  }
  if (numberOfNotesU > 0) {
    if (currentMillisU - previousMillisU >= intervalU) {
      LFODelayGoU = 1;
    } else {
      LFODelayGoU = 0;
    }
  } else {
    LFODelayGoU = 1;
    previousMillisU = currentMillisU;  //reset timer so its ready for the next time
  }

  unsigned long currentMillisL = millis();
  if (monoMultiL && !LFODelayGoL) {
    if (oldnumberOfNotesL < numberOfNotesL) {
      previousMillisL = currentMillisL;
      oldnumberOfNotesL = numberOfNotesL;
    }
  }
  if (numberOfNotesL > 0) {
    if (currentMillisL - previousMillisL >= intervalL) {
      LFODelayGoL = 1;
    } else {
      LFODelayGoL = 0;
    }
  } else {
    LFODelayGoL = 1;
    previousMillisL = currentMillisL;  //reset timer so its ready for the next time
  }
}

void DinHandleNoteOn(byte channel, byte note, byte velocity) {
  numberOfNotesU = numberOfNotesU + 1;
  numberOfNotesL = numberOfNotesL + 1;

  if (wholemode) {
    if (note < 0 || note > 127) return;
    switch (getVoiceNo(-1)) {
      case 1:
        voices[0].note = note;
        voices[0].velocity = velocity;
        voices[0].timeOn = millis();
        MIDI.sendNoteOn(note, velocity, 1);
        voiceOn[0] = true;
        break;
      case 2:
        voices[1].note = note;
        voices[1].velocity = velocity;
        voices[1].timeOn = millis();
        MIDI.sendNoteOn(note, velocity, 2);
        voiceOn[1] = true;
        break;
      case 3:
        voices[2].note = note;
        voices[2].velocity = velocity;
        voices[2].timeOn = millis();
        MIDI.sendNoteOn(note, velocity, 1);
        voiceOn[2] = true;
        break;
      case 4:
        voices[3].note = note;
        voices[3].velocity = velocity;
        voices[3].timeOn = millis();
        MIDI.sendNoteOn(note, velocity, 2);
        voiceOn[3] = true;
        break;
      case 5:
        voices[4].note = note;
        voices[4].velocity = velocity;
        voices[4].timeOn = millis();
        MIDI.sendNoteOn(note, velocity, 1);
        voiceOn[4] = true;
        break;
      case 6:
        voices[5].note = note;
        voices[5].velocity = velocity;
        voices[5].timeOn = millis();
        MIDI.sendNoteOn(note, velocity, 2);
        voiceOn[5] = true;
        break;
      case 7:
        voices[6].note = note;
        voices[6].velocity = velocity;
        voices[6].timeOn = millis();
        MIDI.sendNoteOn(note, velocity, 1);
        voiceOn[6] = true;
        break;
      case 8:
        voices[7].note = note;
        voices[7].velocity = velocity;
        voices[7].timeOn = millis();
        MIDI.sendNoteOn(note, velocity, 2);
        voiceOn[7] = true;
        break;
    }
  }
  if (dualmode) {
    MIDI.sendNoteOn(note, velocity, 1);
    MIDI.sendNoteOn(note, velocity, 2);
  }
  if (splitmode) {
    if (note < (newsplitPoint + 36)) {
      MIDI.sendNoteOn((note + lowerTranspose), velocity, 1);
    } else {
      MIDI.sendNoteOn(note, velocity, 2);
    }
  }
}

void DinHandleNoteOff(byte channel, byte note, byte velocity) {
  numberOfNotesU = numberOfNotesU - 1;
  oldnumberOfNotesU = oldnumberOfNotesU - 1;
  numberOfNotesL = numberOfNotesL - 1;
  oldnumberOfNotesL = oldnumberOfNotesL - 1;

  if (wholemode) {
    switch (getVoiceNo(note)) {
      case 1:
        MIDI.sendNoteOff(note, velocity, 1);
        voices[0].note = -1;
        voiceOn[0] = false;
        break;
      case 2:
        MIDI.sendNoteOff(note, velocity, 2);
        voices[1].note = -1;
        voiceOn[1] = false;
        break;
      case 3:
        MIDI.sendNoteOff(note, velocity, 1);
        voices[2].note = -1;
        voiceOn[2] = false;
        break;
      case 4:
        MIDI.sendNoteOff(note, velocity, 2);
        voices[3].note = -1;
        voiceOn[3] = false;
        break;
      case 5:
        MIDI.sendNoteOff(note, velocity, 1);
        voices[4].note = -1;
        voiceOn[4] = false;
        break;
      case 6:
        MIDI.sendNoteOff(note, velocity, 2);
        voices[5].note = -1;
        voiceOn[5] = false;
        break;
      case 7:
        MIDI.sendNoteOff(note, velocity, 1);
        voices[6].note = -1;
        voiceOn[6] = false;
        break;
      case 8:
        MIDI.sendNoteOff(note, velocity, 2);
        voices[7].note = -1;
        voiceOn[7] = false;
        break;
    }
  }
  if (dualmode) {
    MIDI.sendNoteOff(note, velocity, 1);
    MIDI.sendNoteOff(note, velocity, 2);
  }
  if (splitmode) {
    if (note < (newsplitPoint + 36)) {
      MIDI.sendNoteOff((note + lowerTranspose), velocity, 1);
    } else {
      MIDI.sendNoteOff(note, velocity, 2);
    }
  }
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
    MIDI.sendPitchBend(pitch, 1);
    MIDI.sendPitchBend(pitch, 2);
  }
  if (dualmode) {
    MIDI.sendPitchBend(pitch, 1);
    MIDI.sendPitchBend(pitch, 2);
  }
  if (splitmode) {
    MIDI.sendPitchBend(pitch, 1);
    MIDI.sendPitchBend(pitch, 2);
  }
}

void getDelayTime() {
  delaytimeL = LFODelayL;
  if (delaytimeL <= 0) {
    delaytimeL = 0.1;
  }
  intervalL = (delaytimeL * 10);

  delaytimeU = LFODelayU;
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
    midiCCOut(CCpwLFO, pwLFOU >> midioutfrig);
    midiCCOut51(CCpwLFO, pwLFOU >> midioutfrig);
  } else {
    midiCCOut(CCpwLFO, pwLFOL >> midioutfrig);
    midiCCOut51(CCpwLFO, pwLFOL >> midioutfrig);
  }
}

void updatefmDepth(boolean announce) {
  if (announce) {
    showCurrentParameterPage("FM Depth", int(fmDepthstr));
  }
  if (upperSW) {
    midiCCOut(CCfmDepth, fmDepthU >> midioutfrig);
    midiCCOut51(CCfmDepth, fmDepthU >> midioutfrig);
  } else {
    midiCCOut(CCfmDepth, fmDepthL >> midioutfrig);
    midiCCOut51(CCfmDepth, fmDepthL >> midioutfrig);
  }
}

void updateosc2PW(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 PW", String(osc2PWstr) + " %");
  }
  if (upperSW) {
    midiCCOut(CCosc2PW, osc2PWU >> midioutfrig);
    midiCCOut51(CCosc2PW, osc2PWU >> midioutfrig);
  } else {
    midiCCOut(CCosc2PW, osc2PWL >> midioutfrig);
    midiCCOut51(CCosc2PW, osc2PWL >> midioutfrig);
  }
}

void updateosc2PWM(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 PWM", int(osc2PWMstr));
  }
  if (upperSW) {
    midiCCOut(CCosc2PWM, osc2PWMU >> midioutfrig);
    midiCCOut51(CCosc2PWM, osc2PWMU >> midioutfrig);
  } else {
    midiCCOut(CCosc2PWM, osc2PWML >> midioutfrig);
    midiCCOut51(CCosc2PWM, osc2PWML >> midioutfrig);
  }
}

void updateosc1PW(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC1 PW", String(osc1PWstr) + " %");
  }
  if (upperSW) {
    midiCCOut(CCosc1PW, osc1PWU >> midioutfrig);
    midiCCOut51(CCosc1PW, osc1PWU >> midioutfrig);
  } else {
    midiCCOut(CCosc1PW, osc1PWL >> midioutfrig);
    midiCCOut51(CCosc1PW, osc1PWL >> midioutfrig);
  }
}

void updateosc1PWM(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC1 PWM", int(osc1PWMstr));
  }
  if (upperSW) {
    midiCCOut(CCosc1PWM, osc1PWMU >> midioutfrig);
    midiCCOut51(CCosc1PWM, osc1PWMU >> midioutfrig);
  } else {
    midiCCOut(CCosc1PWM, osc1PWML >> midioutfrig);
    midiCCOut51(CCosc1PWM, osc1PWML >> midioutfrig);
  }
}

void updateosc1Range(boolean announce) {
  if (upperSW) {
    if (osc1Rangestr == 2) {
      if (announce) {
        showCurrentParameterPage("Osc1 Range", String("8"));
      }
      midiCCOut(CCosc1Oct, 2);
      midiCCOut61(CCosc1Oct, 127);
      midiCCOut52(CCosc1Oct, 2);
    } else if (osc1Rangestr == 1) {
      if (announce) {
        showCurrentParameterPage("Osc1 Range", String("16"));
      }
      midiCCOut(CCosc1Oct, 1);
      midiCCOut61(CCosc1Oct, 64);
      midiCCOut52(CCosc1Oct, 1);
    } else {
      if (announce) {
        showCurrentParameterPage("Osc1 Range", String("32"));
      }
      midiCCOut(CCosc1Oct, 0);
      midiCCOut61(CCosc1Oct, 0);
      midiCCOut52(CCosc1Oct, 0);
    }
  } else {
    if (osc1Rangestr == 2) {
      if (announce) {
        showCurrentParameterPage("Osc1 Range", String("8"));
      }
      midiCCOut(CCosc1Oct, 2);
      midiCCOut62(CCosc1Oct, 127);
      midiCCOut52(CCosc1Oct, 2);
      if (wholemode) {
        midiCCOut61(CCosc1Oct, 127);
        midiCCOut52(CCosc1Oct, 2);
      }
    } else if (osc1Rangestr == 1) {
      if (announce) {
        showCurrentParameterPage("Osc1 Range", String("16"));
      }
      midiCCOut(CCosc1Oct, 1);
      midiCCOut62(CCosc1Oct, 64);
      midiCCOut52(CCosc1Oct, 1);
      if (wholemode) {
        midiCCOut61(CCosc1Oct, 64);
        midiCCOut52(CCosc1Oct, 1);
      }
    } else {
      if (announce) {
        showCurrentParameterPage("Osc1 Range", String("32"));
      }
      midiCCOut(CCosc1Oct, 0);
      midiCCOut62(CCosc1Oct, 0);
      midiCCOut52(CCosc1Oct, 0);
      if (wholemode) {
        midiCCOut61(CCosc1Oct, 0);
        midiCCOut52(CCosc1Oct, 0);
      }
    }
  }
}

void updateosc2Range(boolean announce) {
  if (upperSW) {
    if (osc2Rangestr == 2) {
      if (announce) {
        showCurrentParameterPage("Osc2 Range", String("8"));
      }
      midiCCOut61(CCosc2Oct, 127);
      midiCCOut52(CCosc2Oct, 2);
      midiCCOut(CCosc2Oct, 2);
    } else if (osc2Rangestr == 1) {
      if (announce) {
        showCurrentParameterPage("Osc2 Range", String("16"));
      }
      midiCCOut61(CCosc2Oct, 64);
      midiCCOut52(CCosc2Oct, 1);
      midiCCOut(CCosc2Oct, 1);
    } else {
      if (announce) {
        showCurrentParameterPage("Osc2 Range", String("32"));
      }
      midiCCOut(CCosc2Oct, 0);
      midiCCOut61(CCosc2Oct, 0);
      midiCCOut52(CCosc2Oct, 0);
    }
  } else {
    if (osc2Rangestr == 2) {
      if (announce) {
        showCurrentParameterPage("Osc2 Range", String("8"));
      }
      midiCCOut(CCosc2Oct, 2);
      midiCCOut62(CCosc2Oct, 127);
      midiCCOut52(CCosc2Oct, 2);
      if (wholemode) {
        midiCCOut61(CCosc2Oct, 127);
        midiCCOut52(CCosc2Oct, 2);
      }
    } else if (osc2Rangestr == 1) {
      if (announce) {
        showCurrentParameterPage("Osc2 Range", String("16"));
      }
      midiCCOut(CCosc2Oct, 1);
      midiCCOut62(CCosc2Oct, 64);
      midiCCOut52(CCosc2Oct, 1);
      if (wholemode) {
        midiCCOut61(CCosc2Oct, 64);
        midiCCOut52(CCosc2Oct, 1);
      }
    } else {
      if (announce) {
        showCurrentParameterPage("Osc2 Range", String("32"));
      }
      midiCCOut(CCosc2Oct, 0);
      midiCCOut62(CCosc2Oct, 0);
      midiCCOut52(CCosc2Oct, 0);
      if (wholemode) {
        midiCCOut61(CCosc2Oct, 0);
        midiCCOut52(CCosc2Oct, 0);
      }
    }
  }
}

void updateglideTime(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Glide Time", String(glideTimestr * 10) + " Seconds");
  }
  if (upperSW) {
    midiCCOut(CCglideTime, glideTimeU >> midioutfrig);
    midiCCOut51(CCglideTime, glideTimeU >> midioutfrig);
  } else {
    midiCCOut(CCglideTime, glideTimeL >> midioutfrig);
    midiCCOut51(CCglideTime, glideTimeL >> midioutfrig);
  }
}

void updateosc2Detune(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 Detune", String(osc2Detunestr));
  }
  if (upperSW) {
    midiCCOut(CCosc2Detune, osc2DetuneU >> midioutfrig);
    midiCCOut51(CCosc2Detune, osc2DetuneU >> midioutfrig);
  } else {
    midiCCOut(CCosc2Detune, osc2DetuneL >> midioutfrig);
    midiCCOut51(CCosc2Detune, osc2DetuneL >> midioutfrig);
  }
}

void updateosc2Interval(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 Interval", String(osc2Intervalstr));
  }
  if (upperSW) {
    midiCCOut(CCosc2Interval, osc2IntervalU >> midioutfrig);
    midiCCOut51(CCosc2Interval, osc2IntervalU >> midioutfrig);
  } else {
    midiCCOut(CCosc2Interval, osc2IntervalL >> midioutfrig);
    midiCCOut51(CCosc2Interval, osc2IntervalL >> midioutfrig);
  }
}

void updatenoiseLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Noise Level", String(noiseLevelstr));
  }
  if (upperSW) {
    midiCCOut(CCnoiseLevel, noiseLevelU >> midioutfrig);
    midiCCOut51(CCnoiseLevel, noiseLevelU >> midioutfrig);
  } else {
    midiCCOut(CCnoiseLevel, noiseLevelL >> midioutfrig);
    midiCCOut51(CCnoiseLevel, noiseLevelL >> midioutfrig);
  }
}

void updateOsc2SawLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 Saw", int(osc2SawLevelstr));
  }
  if (upperSW) {
    midiCCOut(CCosc2SawLevel, osc2SawLevelU >> midioutfrig);
    midiCCOut51(CCosc2SawLevel, osc2SawLevelU >> midioutfrig);
  } else {
    midiCCOut(CCosc2SawLevel, osc2SawLevelL >> midioutfrig);
    midiCCOut51(CCosc2SawLevel, osc2SawLevelL >> midioutfrig);
  }
}

void updateOsc1SawLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC1 Saw", int(osc1SawLevelstr));
  }
  if (upperSW) {
    midiCCOut(CCosc1SawLevel, osc1SawLevelU >> midioutfrig);
    midiCCOut51(CCosc1SawLevel, osc1SawLevelU >> midioutfrig);
  } else {
    midiCCOut(CCosc1SawLevel, osc1SawLevelL >> midioutfrig);
    midiCCOut51(CCosc1SawLevel, osc1SawLevelL >> midioutfrig);
  }
}

void updateOsc2PulseLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 Pulse", int(osc2PulseLevelstr));
  }
  if (upperSW) {
    midiCCOut(CCosc2PulseLevel, osc2PulseLevelU >> midioutfrig);
    midiCCOut51(CCosc2PulseLevel, osc2PulseLevelU >> midioutfrig);
  } else {
    midiCCOut(CCosc2PulseLevel, osc2PulseLevelL >> midioutfrig);
    midiCCOut51(CCosc2PulseLevel, osc2PulseLevelL >> midioutfrig);
  }
}

void updateOsc1PulseLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC1 Pulse", int(osc1PulseLevelstr));
  }
  if (upperSW) {
    midiCCOut(CCosc1PulseLevel, osc1PulseLevelU >> midioutfrig);
    midiCCOut51(CCosc1PulseLevel, osc1PulseLevelU >> midioutfrig);
  } else {
    midiCCOut(CCosc1PulseLevel, osc1PulseLevelL >> midioutfrig);
    midiCCOut51(CCosc1PulseLevel, osc1PulseLevelL >> midioutfrig);
  }
}

void updateOsc2TriangleLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 Triangle", int(osc2TriangleLevelstr));
  }
  if (upperSW) {
    midiCCOut(CCosc2TriangleLevel, osc2TriangleLevelU >> midioutfrig);
    midiCCOut51(CCosc2TriangleLevel, osc2TriangleLevelU >> midioutfrig);
  } else {
    midiCCOut(CCosc2TriangleLevel, osc2TriangleLevelL >> midioutfrig);
    midiCCOut51(CCosc2TriangleLevel, osc2TriangleLevelL >> midioutfrig);
  }
}

void updateOsc1SubLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC1 Sub", int(osc1SubLevelstr));
  }
  if (upperSW) {
    midiCCOut(CCosc1SubLevel, osc1SubLevelU >> midioutfrig);
    midiCCOut51(CCosc1SubLevel, osc1SubLevelU >> midioutfrig);
  } else {
    midiCCOut(CCosc1SubLevel, osc1SubLevelL >> midioutfrig);
    midiCCOut51(CCosc1SubLevel, osc1SubLevelL >> midioutfrig);
  }
}

void updateamDepth(boolean announce) {
  if (announce) {
    showCurrentParameterPage("AM Depth", int(amDepthstr));
  }
  if (upperSW) {
    midiCCOut(CCamDepth, amDepthU >> midioutfrig);
    midiCCOut51(CCamDepth, amDepthU >> midioutfrig);
  } else {
    midiCCOut(CCamDepth, amDepthL >> midioutfrig);
    midiCCOut51(CCamDepth, amDepthL >> midioutfrig);
  }
}

void updateFilterCutoff(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Cutoff", String(filterCutoffstr) + " Hz");
  }
  if (upperSW) {
    midiCCOut(CCfilterCutoff, filterCutoffU >> midioutfrig);
    midiCCOut51(CCfilterCutoff, filterCutoffU >> midioutfrig);
  } else {
    midiCCOut(CCfilterCutoff, filterCutoffL >> midioutfrig);
    midiCCOut51(CCfilterCutoff, filterCutoffL >> midioutfrig);
  }
}

void updatefilterLFO(boolean announce) {
  if (announce) {
    showCurrentParameterPage("TM depth", int(filterLFOstr));
  }
  if (upperSW) {
    midiCCOut(CCfilterLFO, filterLFOU >> midioutfrig);
    midiCCOut51(CCfilterLFO, filterLFOU >> midioutfrig);
  } else {
    midiCCOut(CCfilterLFO, filterLFOL >> midioutfrig);
    midiCCOut51(CCfilterLFO, filterLFOL >> midioutfrig);
  }
}

void updatefilterRes(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Resonance", int(filterResstr));
  }
  if (upperSW) {
    midiCCOut(CCfilterRes, filterResU >> midioutfrig);
    midiCCOut51(CCfilterRes, filterResU >> midioutfrig);
  } else {
    midiCCOut(CCfilterRes, filterResL >> midioutfrig);
    midiCCOut51(CCfilterRes, filterResL >> midioutfrig);
  }
}

void updateFilterType(boolean announce) {
  if (upperSW) {
    switch (filterTypeU) {
      case 0:
        if (filterPoleSWU == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("3P LowPass"));
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("4P LowPass"));
          }
        }
        srp.set(FILTERA_UPPER, LOW);
        srp.set(FILTERB_UPPER, LOW);
        srp.set(FILTERC_UPPER, LOW);
        break;

      case 1:
        if (filterPoleSWU == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("1P LowPass"));
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P LowPass"));
          }
        }
        srp.set(FILTERA_UPPER, HIGH);
        srp.set(FILTERB_UPPER, LOW);
        srp.set(FILTERC_UPPER, LOW);
        break;

      case 2:
        if (filterPoleSWU == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("3P HP + 1P LP"));
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("4P HighPass"));
          }
        }
        srp.set(FILTERA_UPPER, LOW);
        srp.set(FILTERB_UPPER, HIGH);
        srp.set(FILTERC_UPPER, LOW);
        break;

      case 3:
        if (filterPoleSWU == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("1P HP + 1P LP"));
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P HighPass"));
          }
        }
        srp.set(FILTERA_UPPER, HIGH);
        srp.set(FILTERB_UPPER, HIGH);
        srp.set(FILTERC_UPPER, LOW);
        break;

      case 4:
        if (filterPoleSWU == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P HP + 1P LP"));
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("4P BandPass"));
          }
        }
        srp.set(FILTERA_UPPER, LOW);
        srp.set(FILTERB_UPPER, LOW);
        srp.set(FILTERC_UPPER, HIGH);
        break;

      case 5:
        if (filterPoleSWU == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P BP + 1P LP"));
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P BandPass"));
          }
        }
        srp.set(FILTERA_UPPER, HIGH);
        srp.set(FILTERB_UPPER, LOW);
        srp.set(FILTERC_UPPER, HIGH);
        break;

      case 6:
        if (filterPoleSWU == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("3P AP + 1P LP"));
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("3P AllPass"));
          }
        }
        srp.set(FILTERA_UPPER, LOW);
        srp.set(FILTERB_UPPER, HIGH);
        srp.set(FILTERC_UPPER, HIGH);
        break;

      case 7:
        if (filterPoleSWU == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P Notch + LP"));
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("Notch"));
          }
        }
        srp.set(FILTERA_UPPER, HIGH);
        srp.set(FILTERB_UPPER, HIGH);
        srp.set(FILTERC_UPPER, HIGH);
        break;
    }
  } else {
    switch (filterTypeL) {
      case 0:
        if (filterPoleSWL == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("3P LowPass"));
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("4P LowPass"));
          }
        }
        srp.set(FILTERA_LOWER, LOW);
        srp.set(FILTERB_LOWER, LOW);
        srp.set(FILTERC_LOWER, LOW);
        if (wholemode) {
          srp.set(FILTERA_UPPER, LOW);
          srp.set(FILTERB_UPPER, LOW);
          srp.set(FILTERC_UPPER, LOW);
        }
        break;

      case 1:
        if (filterPoleSWL == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("1P LowPass"));
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P LowPass"));
          }
        }
        srp.set(FILTERA_LOWER, HIGH);
        srp.set(FILTERB_LOWER, LOW);
        srp.set(FILTERC_LOWER, LOW);
        if (wholemode) {
          srp.set(FILTERA_UPPER, HIGH);
          srp.set(FILTERB_UPPER, LOW);
          srp.set(FILTERC_UPPER, LOW);
        }
        break;

      case 2:
        if (filterPoleSWL == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("3P HP + 1P LP"));
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("4P HighPass"));
          }
        }
        srp.set(FILTERA_LOWER, LOW);
        srp.set(FILTERB_LOWER, HIGH);
        srp.set(FILTERC_LOWER, LOW);
        if (wholemode) {
          srp.set(FILTERA_UPPER, LOW);
          srp.set(FILTERB_UPPER, HIGH);
          srp.set(FILTERC_UPPER, LOW);
        }
        break;

      case 3:
        if (filterPoleSWL == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("1P HP + 1P LP"));
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P HighPass"));
          }
        }
        srp.set(FILTERA_LOWER, HIGH);
        srp.set(FILTERB_LOWER, HIGH);
        srp.set(FILTERC_LOWER, LOW);
        if (wholemode) {
          srp.set(FILTERA_UPPER, HIGH);
          srp.set(FILTERB_UPPER, HIGH);
          srp.set(FILTERC_UPPER, LOW);
        }
        break;

      case 4:
        if (filterPoleSWL == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P HP + 1P LP"));
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("4P BandPass"));
          }
        }
        srp.set(FILTERA_LOWER, LOW);
        srp.set(FILTERB_LOWER, LOW);
        srp.set(FILTERC_LOWER, HIGH);
        if (wholemode) {
          srp.set(FILTERA_UPPER, LOW);
          srp.set(FILTERB_UPPER, LOW);
          srp.set(FILTERC_UPPER, HIGH);
        }
        break;

      case 5:
        if (filterPoleSWL == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P BP + 1P LP"));
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P BandPass"));
          }
        }
        srp.set(FILTERA_LOWER, HIGH);
        srp.set(FILTERB_LOWER, LOW);
        srp.set(FILTERC_LOWER, HIGH);
        if (wholemode) {
          srp.set(FILTERA_UPPER, HIGH);
          srp.set(FILTERB_UPPER, LOW);
          srp.set(FILTERC_UPPER, HIGH);
        }
        break;


      case 6:
        if (filterPoleSWL == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("3P AP + 1P LP"));
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("3P AllPass"));
          }
        }
        srp.set(FILTERA_LOWER, LOW);
        srp.set(FILTERB_LOWER, HIGH);
        srp.set(FILTERC_LOWER, HIGH);
        if (wholemode) {
          srp.set(FILTERA_UPPER, LOW);
          srp.set(FILTERB_UPPER, HIGH);
          srp.set(FILTERC_UPPER, HIGH);
        }
        break;

      case 7:
        if (filterPoleSWL == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P Notch + LP"));
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("Notch"));
          }
        }
        srp.set(FILTERA_LOWER, HIGH);
        srp.set(FILTERB_LOWER, HIGH);
        srp.set(FILTERC_LOWER, HIGH);
        if (wholemode) {
          srp.set(FILTERA_UPPER, HIGH);
          srp.set(FILTERB_UPPER, HIGH);
          srp.set(FILTERC_UPPER, HIGH);
        }
        break;
    }
  }
  midiCCOut52(CCfilterType, filterType);
}

void updatefilterEGlevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("EG Depth", int(filterEGlevelstr));
  }
  if (upperSW) {
    midiCCOut(CCfilterEGlevel, filterEGlevelU >> midioutfrig);
    midiCCOut51(CCfilterEGlevel, filterEGlevelU >> midioutfrig);
  } else {
    midiCCOut(CCfilterEGlevel, filterEGlevelL >> midioutfrig);
    midiCCOut51(CCfilterEGlevel, filterEGlevelL >> midioutfrig);
  }
}

void updatekeytrack(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Keytrack", int(keytrackstr));
  }
  if (upperSW) {
    midiCCOut(CCkeyTrack, keytrackU >> midioutfrig);
    midiCCOut51(CCkeyTrack, keytrackU >> midioutfrig);
  } else {
    midiCCOut(CCkeyTrack, keytrackL >> midioutfrig);
    midiCCOut51(CCkeyTrack, keytrackL >> midioutfrig);
  }
}

void updateLFORate(boolean announce) {
  if (announce) {
    showCurrentParameterPage("LFO Rate", String(LFORatestr) + " Hz");
  }
  if (upperSW) {
    midiCCOut(CCLFORate, LFORateU >> midioutfrig);
    midiCCOut51(CCLFORate, LFORateU >> midioutfrig);
  } else {
    midiCCOut(CCLFORate, LFORateL >> midioutfrig);
    midiCCOut51(CCLFORate, LFORateL >> midioutfrig);
  }
}

void updateLFODelay(boolean announce) {
  if (announce) {
    showCurrentParameterPage("LFO Delay", String(LFODelaystr));
  }
  if (upperSW) {
    midiCCOut(CCLFODelay, LFODelayU >> midioutfrig);
    midiCCOut51(CCLFODelay, LFODelayU >> midioutfrig);
  } else {
    midiCCOut(CCLFODelay, LFODelayL >> midioutfrig);
    midiCCOut51(CCLFODelay, LFODelayL >> midioutfrig);
  }
}

void updatemodWheelDepth(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Mod Wheel Depth", String(modWheelDepthstr));
  }
  if (upperSW) {
    midiCCOut(CCmodWheelDepth, modWheelDepthU >> midioutfrig);
    midiCCOut51(CCmodWheelDepth, modWheelDepthU >> midioutfrig);
  } else {
    midiCCOut(CCmodWheelDepth, modWheelDepthL >> midioutfrig);
    midiCCOut51(CCmodWheelDepth, modWheelDepthL >> midioutfrig);
  }
}

void updateeffectPot1(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Effect Pot 1", String(effectPot1str));
  }
  if (upperSW) {
    midiCCOut(CCeffectPot1, effectPot1U >> midioutfrig);
    midiCCOut51(CCeffectPot1, effectPot1U >> midioutfrig);
  } else {
    midiCCOut(CCeffectPot1, effectPot1L >> midioutfrig);
    midiCCOut51(CCeffectPot1, effectPot1L >> midioutfrig);
  }
}

void updateeffectPot2(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Effect Pot 2", String(effectPot2str));
  }
  if (upperSW) {
    midiCCOut(CCeffectPot2, effectPot2U >> midioutfrig);
    midiCCOut51(CCeffectPot2, effectPot2U >> midioutfrig);
  } else {
    midiCCOut(CCeffectPot2, effectPot2L >> midioutfrig);
    midiCCOut51(CCeffectPot2, effectPot2L >> midioutfrig);
  }
}

void updateeffectPot3(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Effect Pot 3", String(effectPot3str));
  }
  if (upperSW) {
    midiCCOut(CCeffectPot3, effectPot3U >> midioutfrig);
    midiCCOut51(CCeffectPot3, effectPot3U >> midioutfrig);
  } else {
    midiCCOut(CCeffectPot3, effectPot3L >> midioutfrig);
    midiCCOut51(CCeffectPot3, effectPot3L >> midioutfrig);
  }
}

void updateeffectsMix(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Effects Mix", String(effectsMixstr));
  }
  if (upperSW) {
    midiCCOut(CCeffectsMix, effectsMixU >> midioutfrig);
    midiCCOut51(CCeffectsMix, effectsMixU >> midioutfrig);
  } else {
    midiCCOut(CCeffectsMix, effectsMixL >> midioutfrig);
    midiCCOut51(CCeffectsMix, effectsMixL >> midioutfrig);
  }
}

void updateStratusLFOWaveform(boolean announce) {
  if (lfoAlt == 0) {
    switch (LFOWaveform) {
      case 0:
        StratusLFOWaveform = "Sawtooth Up";
        LFOWaveCV = 40;
        midiCCOut52(CCLFOWaveform, 0);
        break;

      case 1:
        StratusLFOWaveform = "Sawtooth Down";
        LFOWaveCV = 160;
        midiCCOut52(CCLFOWaveform, 1);
        break;

      case 2:
        StratusLFOWaveform = "Squarewave";
        LFOWaveCV = 280;
        midiCCOut52(CCLFOWaveform, 2);
        break;

      case 3:
        StratusLFOWaveform = "Triangle";
        LFOWaveCV = 400;
        midiCCOut52(CCLFOWaveform, 3);
        break;

      case 4:
        StratusLFOWaveform = "Sinewave";
        LFOWaveCV = 592;
        midiCCOut52(CCLFOWaveform, 4);
        break;

      case 5:
        StratusLFOWaveform = "Sweeps";
        LFOWaveCV = 720;
        midiCCOut52(CCLFOWaveform, 5);
        break;

      case 6:
        StratusLFOWaveform = "Lumps";
        LFOWaveCV = 840;
        midiCCOut52(CCLFOWaveform, 6);
        break;

      case 7:
        StratusLFOWaveform = "Sample & Hold";
        LFOWaveCV = 968;
        midiCCOut52(CCLFOWaveform, 7);
        break;
    }
  } else {
    switch (LFOWaveform) {
      case 0:
        StratusLFOWaveform = "Saw +Oct";
        LFOWaveCV = 40;
        midiCCOut52(CCLFOWaveform, 0);
        break;

      case 1:
        StratusLFOWaveform = "Quad Saw";
        LFOWaveCV = 160;
        midiCCOut52(CCLFOWaveform, 1);
        break;

      case 2:
        StratusLFOWaveform = "Quad Pulse";
        LFOWaveCV = 280;
        midiCCOut52(CCLFOWaveform, 2);
        break;

      case 3:
        StratusLFOWaveform = "Tri Step";
        LFOWaveCV = 400;
        midiCCOut52(CCLFOWaveform, 3);
        break;

      case 4:
        StratusLFOWaveform = "Sine +Oct";
        LFOWaveCV = 592;
        midiCCOut52(CCLFOWaveform, 4);
        break;

      case 5:
        StratusLFOWaveform = "Sine +3rd";
        LFOWaveCV = 720;
        midiCCOut52(CCLFOWaveform, 5);
        break;

      case 6:
        StratusLFOWaveform = "Sine +4th";
        LFOWaveCV = 840;
        midiCCOut52(CCLFOWaveform, 6);
        break;

      case 7:
        StratusLFOWaveform = "Rand Slopes";
        LFOWaveCV = 968;
        midiCCOut52(CCLFOWaveform, 7);
        break;
    }
  }
  if (announce) {
    showCurrentParameterPage("LFO Wave", StratusLFOWaveform);
  }
  if (upperSW) {
    LFOWaveformU = LFOWaveCV;
  } else {
    LFOWaveformL = LFOWaveCV;
    if (wholemode) {
      LFOWaveformU = LFOWaveCV;
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
    midiCCOut(CCfilterAttack, filterAttackU >> midioutfrig);
    midiCCOut51(CCfilterAttack, filterAttackU >> midioutfrig);
  } else {
    midiCCOut(CCfilterAttack, filterAttackL >> midioutfrig);
    midiCCOut51(CCfilterAttack, filterAttackL >> midioutfrig);
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
    midiCCOut(CCfilterDecay, filterDecayU >> midioutfrig);
    midiCCOut51(CCfilterDecay, filterDecayU >> midioutfrig);
  } else {
    midiCCOut(CCfilterDecay, filterDecayL >> midioutfrig);
    midiCCOut51(CCfilterDecay, filterDecayL >> midioutfrig);
  }
}

void updatefilterSustain(boolean announce) {
  if (announce) {
    showCurrentParameterPage("VCF Sustain", String(filterSustainstr), FILTER_ENV);
  }
  if (upperSW) {
    midiCCOut(CCfilterSustain, filterSustainU >> midioutfrig);
    midiCCOut51(CCfilterSustain, filterSustainU >> midioutfrig);
  } else {
    midiCCOut(CCfilterSustain, filterSustainL >> midioutfrig);
    midiCCOut51(CCfilterSustain, filterSustainL >> midioutfrig);
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
    midiCCOut(CCfilterRelease, filterReleaseU >> midioutfrig);
    midiCCOut51(CCfilterRelease, filterReleaseU >> midioutfrig);
  } else {
    midiCCOut(CCfilterRelease, filterReleaseL >> midioutfrig);
    midiCCOut51(CCfilterRelease, filterReleaseL >> midioutfrig);
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
    midiCCOut(CCampAttack, ampAttackU >> midioutfrig);
    midiCCOut51(CCampAttack, ampAttackU >> midioutfrig);
  } else {
    midiCCOut(CCampAttack, ampAttackL >> midioutfrig);
    midiCCOut51(CCampAttack, ampAttackL >> midioutfrig);
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
    midiCCOut(CCampDecay, ampDecayU >> midioutfrig);
    midiCCOut51(CCampDecay, ampDecayU >> midioutfrig);
  } else {
    midiCCOut(CCampDecay, ampDecayL >> midioutfrig);
    midiCCOut51(CCampDecay, ampDecayL >> midioutfrig);
  }
}

void updateampSustain(boolean announce) {
  if (announce) {
    showCurrentParameterPage("VCA Sustain", String(ampSustainstr), AMP_ENV);
  }
  if (upperSW) {
    midiCCOut(CCampSustain, ampSustainU >> midioutfrig);
    midiCCOut51(CCampSustain, ampSustainU >> midioutfrig);
  } else {
    midiCCOut(CCampSustain, ampSustainL >> midioutfrig);
    midiCCOut51(CCampSustain, ampSustainL >> midioutfrig);
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
    midiCCOut(CCampRelease, ampReleaseU >> midioutfrig);
    midiCCOut51(CCampRelease, ampReleaseU >> midioutfrig);
  } else {
    midiCCOut(CCampRelease, ampReleaseL >> midioutfrig);
    midiCCOut51(CCampRelease, ampReleaseL >> midioutfrig);
  }
}

void updatevolumeControl(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Volume", int(volumeControlstr));
  }
  if (upperSW) {
    midiCCOut(CCvolumeControl, volumeControlU >> midioutfrig);
    midiCCOut51(CCvolumeControl, volumeControlU >> midioutfrig);
  } else {
    midiCCOut(CCvolumeControl, volumeControlL >> midioutfrig);
    midiCCOut51(CCvolumeControl, volumeControlL >> midioutfrig);
  }
}

void updatePM_DCO2(boolean announce) {
  if (announce) {
    showCurrentParameterPage("PolyMod DCO2", int(pmDCO2str));
  }
  if (upperSW) {
    midiCCOut(CCPM_DCO2, pmDCO2U >> midioutfrig);
    midiCCOut51(CCPM_DCO2, pmDCO2U >> midioutfrig);
  } else {
    midiCCOut(CCPM_DCO2, pmDCO2L >> midioutfrig);
    midiCCOut51(CCPM_DCO2, pmDCO2L >> midioutfrig);
  }
}

void updatePM_FilterEnv(boolean announce) {
  if (announce) {
    showCurrentParameterPage("PolyMod Filter Env", int(pmFilterEnvstr));
  }
  if (upperSW) {
    midiCCOut(CCPM_FilterEnv, pmFilterEnvU >> midioutfrig);
    midiCCOut51(CCPM_FilterEnv, pmFilterEnvU >> midioutfrig);
  } else {
    midiCCOut(CCPM_FilterEnv, pmFilterEnvL >> midioutfrig);
    midiCCOut51(CCPM_FilterEnv, pmFilterEnvL >> midioutfrig);
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
      midiCCOut52(CCchordHoldSW, 0);
    } else {
      if (announce) {
        showCurrentParameterPage("Chord Hold", "On");
      }
      midiCCOut(CCchordHoldSW, 127);
      midiCCOut52(CCchordHoldSW, 127);
    }
  } else {
    if (chordHoldL == 0) {
      if (announce) {
        showCurrentParameterPage("Chord Hold", "Off");
      }
      midiCCOut(CCchordHoldSW, 0);
      midiCCOut52(CCchordHoldSW, 0);
    } else {
      if (announce) {
        showCurrentParameterPage("Chord Hold", "On");
      }
      midiCCOut(CCchordHoldSW, 127);
      midiCCOut52(CCchordHoldSW, 127);
    }
  }
}

void updateglideSW(boolean announce) {
  if (upperSW) {
    if (glideSWU == 0) {
      if (announce) {
        showCurrentParameterPage("Glide", "Off");
      }
      midiCCOut52(CCglideSW, 0);
      delay(1);
      midiCCOut(CCglideTime, glideTimeU >> midioutfrig);
      midiCCOut51(CCglideTime, glideTimeU >> midioutfrig);
    } else {
      if (announce) {
        showCurrentParameterPage("Glide", "On");
      }
      midiCCOut(CCglideTime, glideTimeU >> midioutfrig);
      midiCCOut51(CCglideTime, glideTimeU >> midioutfrig);
      delay(1);
      midiCCOut52(CCglideSW, 127);
    }
  } else {
    if (glideSWL == 0) {
      if (announce) {
        showCurrentParameterPage("Glide", "Off");
      }
      midiCCOut52(CCglideSW, 0);
      delay(1);
      midiCCOut(CCglideTime, glideTimeL >> midioutfrig);
      midiCCOut51(CCglideTime, glideTimeL >> midioutfrig);
    } else {
      if (announce) {
        showCurrentParameterPage("Glide", "On");
      }
      midiCCOut(CCglideTime, glideTimeL >> midioutfrig);
      midiCCOut51(CCglideTime, glideTimeL >> midioutfrig);
      delay(1);
      midiCCOut52(CCglideSW, 127);
    }
  }
}

void updatefilterPoleSwitch(boolean announce) {
  if (upperSW) {
    if (filterPoleSWU == 1) {
      if (announce) {
        //showCurrentParameterPage("VCF Pole", "On");
        updateFilterType(1);
      }
      midiCCOut(CCfilterPoleSW, 127);
      midiCCOut52(CCfilterPoleSW, 127);
      srp.set(FILTER_POLE_UPPER, HIGH);
    } else {
      if (announce) {
        //showCurrentParameterPage("VCF Pole", "Off");
        updateFilterType(1);
      }
      midiCCOut(CCfilterPoleSW, 0);
      midiCCOut52(CCfilterPoleSW, 0);
      srp.set(FILTER_POLE_UPPER, LOW);
    }
  } else {
    if (filterPoleSWL == 1) {
      if (announce) {
        //showCurrentParameterPage("VCF Pole", "On");
        updateFilterType(1);
      }
      midiCCOut(CCfilterPoleSW, 127);
      midiCCOut52(CCfilterPoleSW, 127);
      srp.set(FILTER_POLE_LOWER, HIGH);
      if (wholemode) {
        srp.set(FILTER_POLE_UPPER, HIGH);
      }
    } else {
      if (announce) {
        //showCurrentParameterPage("VCF Pole", "Off");
        updateFilterType(1);
      }
      midiCCOut(CCfilterPoleSW, 0);
      midiCCOut52(CCfilterPoleSW, 0);
      srp.set(FILTER_POLE_LOWER, LOW);
      if (wholemode) {
        srp.set(FILTER_POLE_UPPER, LOW);
      }
    }
  }
}

// void updatefilterLoop(boolean announce) {
//   if (upperSW) {
//     switch (statefilterLoopU) {
//       case 1:
//         if (announce) {
//           showCurrentParameterPage("VCF Key Loop", "On");
//           midiCCOut(CCfilterLoop, 127);
//         }
//         // sr.set(FILTERLOOP_LED, HIGH);        // LED on
//         // sr.set(FILTERLOOP_DOUBLE_LED, LOW);  // LED on
//         srp.set(FILTER_MODE_BIT0_UPPER, LOW);
//         srp.set(FILTER_MODE_BIT1_UPPER, HIGH);
//         oldfilterLoop = statefilterLoopU;
//         break;

//       case 2:
//         if (announce) {
//           showCurrentParameterPage("VCF LFO Loop", "On");
//           midiCCOut(CCfilterDoubleLoop, 127);
//         }
//         // sr.set(FILTERLOOP_DOUBLE_LED, HIGH);  // LED on
//         // sr.set(FILTERLOOP_LED, LOW);
//         srp.set(FILTER_MODE_BIT0_UPPER, HIGH);
//         srp.set(FILTER_MODE_BIT1_UPPER, HIGH);
//         oldfilterLoop = statefilterLoopU;
//         break;

//       default:
//         if (announce) {
//           showCurrentParameterPage("VCF Looping", "Off");
//           midiCCOut(CCfilterLoop, 1);
//         }
//         // sr.set(FILTERLOOP_LED, LOW);         // LED off
//         // sr.set(FILTERLOOP_DOUBLE_LED, LOW);  // LED on
//         srp.set(FILTER_MODE_BIT0_UPPER, LOW);
//         srp.set(FILTER_MODE_BIT1_UPPER, LOW);
//         oldfilterLoop = 0;
//         break;
//     }
//   } else {
//     switch (statefilterLoopL) {
//       case 1:
//         if (announce) {
//           showCurrentParameterPage("VCF Key Loop", "On");
//           midiCCOut(CCfilterLoop, 127);
//         }
//         // sr.set(FILTERLOOP_LED, HIGH);        // LED on
//         // sr.set(FILTERLOOP_DOUBLE_LED, LOW);  // LED on
//         srp.set(FILTER_MODE_BIT0_LOWER, LOW);
//         srp.set(FILTER_MODE_BIT1_LOWER, HIGH);
//         if (wholemode) {
//           srp.set(FILTER_MODE_BIT0_UPPER, LOW);
//           srp.set(FILTER_MODE_BIT1_UPPER, HIGH);
//         }
//         oldfilterLoop = statefilterLoopL;
//         break;

//       case 2:
//         if (announce) {
//           showCurrentParameterPage("VCF LFO Loop", "On");
//           midiCCOut(CCfilterDoubleLoop, 127);
//         }
//         // sr.set(FILTERLOOP_DOUBLE_LED, HIGH);  // LED on
//         // sr.set(FILTERLOOP_LED, LOW);
//         srp.set(FILTER_MODE_BIT0_LOWER, HIGH);
//         srp.set(FILTER_MODE_BIT1_LOWER, HIGH);
//         if (wholemode) {
//           srp.set(FILTER_MODE_BIT0_UPPER, LOW);
//           srp.set(FILTER_MODE_BIT1_UPPER, LOW);
//         }
//         oldfilterLoop = statefilterLoopL;
//         break;

//       default:
//         if (announce) {
//           showCurrentParameterPage("VCF Looping", "Off");
//           midiCCOut(CCfilterLoop, 1);
//         }
//         // sr.set(FILTERLOOP_LED, LOW);         // LED off
//         // sr.set(FILTERLOOP_DOUBLE_LED, LOW);  // LED on
//         srp.set(FILTER_MODE_BIT0_LOWER, LOW);
//         srp.set(FILTER_MODE_BIT1_LOWER, LOW);
//         if (wholemode) {
//           srp.set(FILTER_MODE_BIT0_UPPER, LOW);
//           srp.set(FILTER_MODE_BIT1_UPPER, LOW);
//         }
//         oldfilterLoop = 0;
//         break;
//     }
//   }
// }

void updatefilterEGinv(boolean announce) {
  if (upperSW) {
    if (filterEGinvU == 0) {
      if (announce) {
        showCurrentParameterPage("Filter Env", "Positive");
      }
      midiCCOut(CCfilterEGinv, 0);
      midiCCOut52(CCfilterEGinv, 0);
      srp.set(FILTER_EG_INV_UPPER, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Filter Env", "Negative");
      }
      midiCCOut(CCfilterEGinv, 127);
      midiCCOut52(CCfilterEGinv, 127);
      // sr.set(FILTERINV_LED, HIGH);  // LED on
      srp.set(FILTER_EG_INV_UPPER, HIGH);
    }
  } else {
    if (filterEGinvL == 0) {
      if (announce) {
        showCurrentParameterPage("Filter Env", "Positive");
      }
      midiCCOut(CCfilterEGinv, 0);
      midiCCOut52(CCfilterEGinv, 0);
      srp.set(FILTER_EG_INV_LOWER, LOW);
      if (wholemode) {
        srp.set(FILTER_EG_INV_UPPER, LOW);
      }
    } else {
      if (announce) {
        showCurrentParameterPage("Filter Env", "Negative");
      }
      midiCCOut(CCfilterEGinv, 127);
      midiCCOut52(CCfilterEGinv, 127);
      srp.set(FILTER_EG_INV_LOWER, HIGH);
      if (wholemode) {
        srp.set(FILTER_EG_INV_UPPER, HIGH);
      }
    }
  }
}

void updatesyncSW(boolean announce) {
  if (upperSW) {
    if (!syncU) {
      if (announce) {
        showCurrentParameterPage("Sync", "Off");
      }
      midiCCOut(CCsyncSW, 0);
      midiCCOut52(CCsyncSW, 0);
      srp.set(SYNC_UPPER, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Sync", "On");
      }
      midiCCOut(CCsyncSW, 127);
      midiCCOut52(CCsyncSW, 127);
      srp.set(SYNC_UPPER, HIGH);
    }
  } else {
    if (!syncL) {
      if (announce) {
        showCurrentParameterPage("Sync", "Off");
      }
      midiCCOut(CCsyncSW, 0);
      midiCCOut52(CCsyncSW, 0);
      srp.set(SYNC_LOWER, LOW);
      if (wholemode) {
        srp.set(SYNC_UPPER, LOW);
      }
    } else {
      if (announce) {
        showCurrentParameterPage("Sync", "On");
      }
      midiCCOut(CCsyncSW, 127);
      midiCCOut52(CCsyncSW, 127);
      srp.set(SYNC_LOWER, HIGH);
      if (wholemode) {
        srp.set(SYNC_UPPER, HIGH);
      }
    }
  }
}

// void updatefilterVel(boolean announce) {
//   if (upperSW) {
//     if (filterVelU == 0) {
//       if (announce) {
//         showCurrentParameterPage("VCF Velocity", "Off");
//         midiCCOut(CCfilterVel, 1);
//       }
//       // sr.set(FILTERVEL_LED, LOW);  // LED off
//       srp.set(FILTER_VELOCITY_UPPER, LOW);
//     } else {
//       if (announce) {
//         showCurrentParameterPage("VCF Velocity", "On");
//         midiCCOut(CCfilterVel, 127);
//       }
//       // sr.set(FILTERVEL_LED, HIGH);  // LED on
//       srp.set(FILTER_VELOCITY_UPPER, HIGH);
//     }
//   } else {
//     if (filterVelL == 0) {
//       if (announce) {
//         showCurrentParameterPage("VCF Velocity", "Off");
//         midiCCOut(CCfilterVel, 1);
//       }
//       // sr.set(FILTERVEL_LED, LOW);  // LED off
//       srp.set(FILTER_VELOCITY_LOWER, LOW);
//       if (wholemode) {
//         srp.set(FILTER_VELOCITY_UPPER, LOW);
//       }
//     } else {
//       if (announce) {
//         showCurrentParameterPage("VCF Velocity", "On");
//         midiCCOut(CCfilterVel, 127);
//       }
//       // sr.set(FILTERVEL_LED, HIGH);  // LED on
//       srp.set(FILTER_VELOCITY_LOWER, HIGH);
//       if (wholemode) {
//         srp.set(FILTER_VELOCITY_UPPER, HIGH);
//       }
//     }
//   }
// }

// void updatevcaLoop(boolean announce) {
//   if (upperSW) {
//     switch (statevcaLoopU) {
//       case 1:
//         if (announce) {
//           showCurrentParameterPage("VCA Key Loop", "On");
//           midiCCOut(CCvcaLoop, 127);
//         }
//         // sr.set(VCALOOP_LED, HIGH);        // LED on
//         // sr.set(VCALOOP_DOUBLE_LED, LOW);  // LED on
//         srp.set(AMP_MODE_BIT0_UPPER, LOW);
//         srp.set(AMP_MODE_BIT1_UPPER, HIGH);
//         oldvcaLoop = statevcaLoopU;
//         break;

//       case 2:
//         if (announce) {
//           showCurrentParameterPage("VCA LFO Loop", "On");
//           midiCCOut(CCvcaDoubleLoop, 127);
//         }
//         // sr.set(VCALOOP_DOUBLE_LED, HIGH);  // LED on
//         // sr.set(VCALOOP_LED, LOW);
//         srp.set(AMP_MODE_BIT0_UPPER, HIGH);
//         srp.set(AMP_MODE_BIT1_UPPER, HIGH);
//         oldvcaLoop = statevcaLoopU;
//         break;

//       default:
//         if (announce) {
//           showCurrentParameterPage("VCA Looping", "Off");
//           midiCCOut(CCvcaLoop, 1);
//         }
//         // sr.set(VCALOOP_LED, LOW);         // LED off
//         // sr.set(VCALOOP_DOUBLE_LED, LOW);  // LED on
//         srp.set(AMP_MODE_BIT0_UPPER, LOW);
//         srp.set(AMP_MODE_BIT1_UPPER, LOW);
//         oldvcaLoop = 0;
//         break;
//     }
//   } else {
//     switch (statevcaLoopL) {
//       case 1:
//         if (announce) {
//           showCurrentParameterPage("VCA Key Loop", "On");
//           midiCCOut(CCvcaLoop, 127);
//         }
//         // sr.set(VCALOOP_LED, HIGH);        // LED on
//         // sr.set(VCALOOP_DOUBLE_LED, LOW);  // LED on
//         srp.set(AMP_MODE_BIT0_LOWER, LOW);
//         srp.set(AMP_MODE_BIT1_LOWER, HIGH);
//         if (wholemode) {
//           srp.set(AMP_MODE_BIT0_UPPER, LOW);
//           srp.set(AMP_MODE_BIT1_UPPER, HIGH);
//         }
//         oldvcaLoop = statevcaLoopL;
//         break;

//       case 2:
//         if (announce) {
//           showCurrentParameterPage("VCA LFO Loop", "On");
//           midiCCOut(CCvcaDoubleLoop, 127);
//         }
//         // sr.set(VCALOOP_DOUBLE_LED, HIGH);  // LED on
//         // sr.set(VCALOOP_LED, LOW);
//         srp.set(AMP_MODE_BIT0_LOWER, HIGH);
//         srp.set(AMP_MODE_BIT1_LOWER, HIGH);
//         if (wholemode) {
//           srp.set(AMP_MODE_BIT0_UPPER, LOW);
//           srp.set(AMP_MODE_BIT1_UPPER, LOW);
//         }
//         oldvcaLoop = statevcaLoopL;
//         break;

//       default:
//         if (announce) {
//           showCurrentParameterPage("VCA Looping", "Off");
//           midiCCOut(CCvcaLoop, 1);
//         }
//         // sr.set(VCALOOP_LED, LOW);         // LED off
//         // sr.set(VCALOOP_DOUBLE_LED, LOW);  // LED on
//         srp.set(AMP_MODE_BIT0_LOWER, LOW);
//         srp.set(AMP_MODE_BIT1_LOWER, LOW);
//         if (wholemode) {
//           srp.set(AMP_MODE_BIT0_UPPER, LOW);
//           srp.set(AMP_MODE_BIT1_UPPER, LOW);
//         }
//         oldvcaLoop = 0;
//         break;
//     }
//   }
// }

// void updatevcaVel(boolean announce) {
//   if (upperSW) {
//     if (vcaVelU == 0) {
//       if (announce) {
//         showCurrentParameterPage("VCA Velocity", "Off");
//         midiCCOut(CCvcaVel, 1);
//       }
//       // sr.set(VCAVEL_LED, LOW);  // LED off
//       srp.set(AMP_VELOCITY_UPPER, LOW);
//     } else {
//       if (announce) {
//         showCurrentParameterPage("VCA Velocity", "On");
//         midiCCOut(CCvcaVel, 127);
//       }
//       // sr.set(VCAVEL_LED, HIGH);  // LED on
//       srp.set(AMP_VELOCITY_UPPER, HIGH);
//     }
//   } else {
//     if (vcaVelL == 0) {
//       if (announce) {
//         showCurrentParameterPage("VCA Velocity", "Off");
//         midiCCOut(CCvcaVel, 1);
//       }
//       // sr.set(VCAVEL_LED, LOW);  // LED off
//       srp.set(AMP_VELOCITY_LOWER, LOW);
//       if (wholemode) {
//         srp.set(AMP_VELOCITY_UPPER, LOW);
//       }
//     } else {
//       if (announce) {
//         showCurrentParameterPage("VCA Velocity", "On");
//         midiCCOut(CCvcaVel, 127);
//       }
//       // sr.set(VCAVEL_LED, HIGH);  // LED on
//       srp.set(AMP_VELOCITY_LOWER, HIGH);
//       if (wholemode) {
//         srp.set(AMP_VELOCITY_UPPER, HIGH);
//       }
//     }
//   }
// }


// void updatevcaGate(boolean announce) {
//   if (upperSW) {
//     if (vcaGateU == 0) {
//       if (announce) {
//         showCurrentParameterPage("VCA Gate", "Off");
//         midiCCOut(CCvcaGate, 1);
//       }
//       // sr.set(VCAGATE_LED, LOW);  // LED off
//       ampAttackU = oldampAttackU;
//       ampDecayU = oldampDecayU;
//       ampSustainU = oldampSustainU;
//       ampReleaseU = oldampReleaseU;
//     } else {
//       if (announce) {
//         showCurrentParameterPage("VCA Gate", "On");
//         midiCCOut(CCvcaGate, 127);
//       }
//       // sr.set(VCAGATE_LED, HIGH);  // LED on
//       ampAttackU = 0;
//       ampDecayU = 0;
//       ampSustainU = 1023;
//       ampReleaseU = 0;
//     }
//   } else {
//     if (vcaGateL == 0) {
//       if (announce) {
//         showCurrentParameterPage("VCA Gate", "Off");
//         midiCCOut(CCvcaGate, 1);
//       }
//       // sr.set(VCAGATE_LED, LOW);  // LED off
//       ampAttackL = oldampAttackL;
//       ampDecayL = oldampDecayL;
//       ampSustainL = oldampSustainL;
//       ampReleaseL = oldampReleaseL;
//       if (wholemode) {
//         ampAttackU = oldampAttackU;
//         ampDecayU = oldampDecayU;
//         ampSustainU = oldampSustainU;
//         ampReleaseU = oldampReleaseU;
//       }
//     } else {
//       if (announce) {
//         showCurrentParameterPage("VCA Gate", "On");
//         midiCCOut(CCvcaGate, 127);
//       }
//       // sr.set(VCAGATE_LED, HIGH);  // LED on
//       ampAttackL = 0;
//       ampDecayL = 0;
//       ampSustainL = 1023;
//       ampReleaseL = 0;
//       if (wholemode) {
//         ampAttackU = 0;
//         ampDecayU = 0;
//         ampSustainU = 1023;
//         ampReleaseU = 0;
//       }
//     }
//   }
// }

void updatelfoAlt(boolean announce) {
  if (upperSW) {
    if (lfoAltU == 0) {
      lfoAlt = 0;
      midiCCOut(CClfoAlt, 0);
      midiCCOut52(CClfoAlt, 0);
      updateStratusLFOWaveform(1);
      srp.set(LFO_ALT_UPPER, HIGH);
    } else {
      lfoAlt = 127;
      midiCCOut(CClfoAlt, 127);
      midiCCOut52(CClfoAlt, 127);
      updateStratusLFOWaveform(1);
      srp.set(LFO_ALT_UPPER, LOW);
    }
  } else {
    if (lfoAltL == 0) {
      lfoAlt = 0;
      midiCCOut(CClfoAlt, 0);
      midiCCOut52(CClfoAlt, 0);
      updateStratusLFOWaveform(1);
      srp.set(LFO_ALT_LOWER, HIGH);
      if (wholemode) {
        srp.set(LFO_ALT_UPPER, HIGH);
      }
    } else {
      lfoAlt = 127;
      midiCCOut(CClfoAlt, 127);
      midiCCOut52(CClfoAlt, 127);
      updateStratusLFOWaveform(1);
      srp.set(LFO_ALT_LOWER, LOW);
      if (wholemode) {
        srp.set(LFO_ALT_UPPER, LOW);
      }
    }
  }
}

// void updatekeyTrackSW(boolean announce) {
//   if (upperSW) {
//     if (keyTrackSWU == 0) {
//       srp.set(FILTER_KEYTRACK_UPPER, LOW);
//     } else {
//       srp.set(FILTER_KEYTRACK_UPPER, HIGH);
//     }
//   } else {
//     if (keyTrackSWL == 0) {
//       srp.set(FILTER_KEYTRACK_LOWER, LOW);
//       if (wholemode) {
//         srp.set(FILTER_KEYTRACK_UPPER, LOW);
//       }
//     } else {
//       srp.set(FILTER_KEYTRACK_LOWER, HIGH);
//       if (wholemode) {
//         srp.set(FILTER_KEYTRACK_UPPER, HIGH);
//       }
//     }
//   }
// }

void updateupperSW() {
  if (upperSW) {
    showCurrentParameterPage("Upper", "On");
    setAllButtons();
    midiCCOut52(CCupperSW, 127);
    midiCCOut52(CClowerSW, 0);
  } else {
    upperSW = 0;
  }
}

void updatelowerSW() {
  if (lowerSW) {
    showCurrentParameterPage("Lower", "On");
    setAllButtons();
    midiCCOut52(CCupperSW, 0);
    midiCCOut52(CClowerSW, 127);
  } else {
    lowerSW = 0;
  }
}

// void updatewholemode() {
//   allNotesOff();
//   showCurrentParameterPage("Mode", String("Whole"));
//   // sr.set(WHOLE_LED, HIGH);  // LED off
//   // sr.set(DUAL_LED, LOW);    // LED off
//   // sr.set(SPLIT_LED, LOW);   // LED off
//   // sr.set(UPPER_LED, LOW);   // LED off
//   // srp.set(UPPER_RELAY_1, HIGH);
//   // srp.set(UPPER_RELAY_2, HIGH);
//   upperSW = 0;
//   setAllButtons();
//   dualmode = 0;
//   splitmode = 0;
// }

// void updatedualmode() {
//   allNotesOff();
//   showCurrentParameterPage("Mode", String("Dual"));
//   // sr.set(DUAL_LED, HIGH);  // LED off
//   // sr.set(WHOLE_LED, LOW);  // LED off
//   // sr.set(SPLIT_LED, LOW);  // LED off
//   srp.set(UPPER2, LOW);
//   wholemode = 0;
//   splitmode = 0;
// }

// void updatesplitmode() {
//   allNotesOff();
//   showCurrentParameterPage("Mode", String("Split"));
//   // sr.set(SPLIT_LED, HIGH);  // LED off
//   // sr.set(WHOLE_LED, LOW);   // LED off
//   // sr.set(DUAL_LED, LOW);    // LED off
//   srp.set(UPPER2, LOW);
//   wholemode = 0;
//   dualmode = 0;
// }

// void updateFilterEnv(boolean announce) {
//   if (filterLogLinU == 0) {
//     srp.set(FILTER_LIN_LOG_UPPER, HIGH);
//   } else {
//     srp.set(FILTER_LIN_LOG_UPPER, LOW);
//   }
//   if (filterLogLinL == 0) {
//     srp.set(FILTER_LIN_LOG_LOWER, HIGH);
//     if (wholemode) {
//       srp.set(FILTER_LIN_LOG_UPPER, HIGH);
//     }
//   } else {
//     srp.set(FILTER_LIN_LOG_LOWER, LOW);
//     if (wholemode) {
//       srp.set(FILTER_LIN_LOG_UPPER, LOW);
//     }
//   }
// }

// void updateAmpEnv(boolean announce) {
//   if (ampLogLinU == 0) {
//     srp.set(AMP_LIN_LOG_UPPER, LOW);
//   } else {
//     srp.set(AMP_LIN_LOG_UPPER, HIGH);
//   }
//   if (ampLogLinL == 0) {
//     srp.set(AMP_LIN_LOG_LOWER, LOW);
//     if (wholemode) {
//       srp.set(AMP_LIN_LOG_UPPER, LOW);
//     }
//   } else {
//     srp.set(AMP_LIN_LOG_LOWER, HIGH);
//     if (wholemode) {
//       srp.set(AMP_LIN_LOG_UPPER, HIGH);
//     }
//   }
// }

// void updateMonoMulti(boolean announce) {
//   if (upperSW) {
//     if (monoMultiU == 0) {
//       if (announce) {
//         showCurrentParameterPage("LFO Retrigger", "Off");
//       }
//     } else {
//       if (announce) {
//         showCurrentParameterPage("LFO Retrigger", "On");
//       }
//     }
//   } else {
//     if (monoMultiL == 0) {
//       if (announce) {
//         showCurrentParameterPage("LFO Retrigger", "Off");
//       }
//     } else {
//       if (announce) {
//         showCurrentParameterPage("LFO Retrigger", "On");
//       }
//     }
//   }
// }

void updatePitchBend() {
  showCurrentParameterPage("Bender Range", int(PitchBendLevelstr));
}

void updatemodWheel() {
  showCurrentParameterPage("Mod Range", int(modWheelLevelstr));
}

void updatePatchname() {
  showPatchPage(String(patchNoU), patchNameU, String(patchNoL), patchNameL);
}

void myControlChange(byte channel, byte control, int value) {

  switch (control) {
    case CCpwLFO:
      if (upperSW) {
        pwLFOU = value;
      } else {
        pwLFOL = value;
        if (wholemode) {
          pwLFOU = value;
        }
      }
      pwLFOstr = value >> midioutfrig;  // for display
      updatepwLFO(1);
      break;

    case CCfmDepth:
      if (upperSW) {
        fmDepthU = value;
      } else {
        fmDepthL = value;
        if (wholemode) {
          fmDepthU = value;
        }
      }
      fmDepthstr = value >> midioutfrig;
      updatefmDepth(1);
      break;

    case CCosc2PW:
      if (upperSW) {
        osc2PWU = value;
      } else {
        osc2PWL = value;
        if (wholemode) {
          osc2PWU = value;
        }
      }
      osc2PWstr = PULSEWIDTH[value >> midioutfrig];
      updateosc2PW(1);
      break;

    case CCosc2PWM:
      if (upperSW) {
        osc2PWMU = value;
      } else {
        osc2PWML = value;
        if (wholemode) {
          osc2PWMU = value;
        }
      }
      osc2PWMstr = value >> midioutfrig;
      updateosc2PWM(1);
      break;

    case CCosc1PW:
      if (upperSW) {
        osc1PWU = value;
      } else {
        osc1PWL = value;
        if (wholemode) {
          osc1PWU = value;
        }
      }
      osc1PWstr = PULSEWIDTH[value >> midioutfrig];
      updateosc1PW(1);
      break;

    case CCosc1PWM:
      if (upperSW) {
        osc1PWMU = value;
      } else {
        osc1PWML = value;
        if (wholemode) {
          osc1PWMU = value;
        }
      }
      osc1PWMstr = value >> midioutfrig;
      updateosc1PWM(1);
      break;

    case CCosc1Oct:
      if (upperSW) {
        osc1RangeU = value;
      } else {
        osc1RangeL = value;
        if (wholemode) {
          osc1RangeU = value;
        }
      }
      osc1Rangestr = map(value, 0, 127, 0, 2);
      updateosc1Range(1);
      break;

    case CCosc2Oct:
      if (upperSW) {
        osc2RangeU = value;
      } else {
        osc2RangeL = value;
        if (wholemode) {
          osc2RangeU = value;
        }
      }
      osc2Rangestr = map(value, 0, 127, 0, 2);
      updateosc2Range(1);
      break;

    case CCglideTime:
      if (upperSW) {
        glideTimeU = value;
      } else {
        glideTimeL = value;
        if (wholemode) {
          glideTimeU = value;
        }
      }
      glideTimestr = LINEAR[value >> midioutfrig];
      updateglideTime(1);
      break;

    case CCosc2Detune:
      if (upperSW) {
        osc2DetuneU = value;
      } else {
        osc2DetuneL = value;
        if (wholemode) {
          osc2DetuneU = value;
        }
      }
      osc2Detunestr = PULSEWIDTH[value >> midioutfrig];
      updateosc2Detune(1);
      break;

    case CCosc2Interval:
      if (upperSW) {
        osc2IntervalU = value;
      } else {
        osc2IntervalL = value;
        if (wholemode) {
          osc2IntervalU = value;
        }
      }
      osc2Intervalstr = value >> midioutfrig;
      updateosc2Interval(1);
      break;

    case CCnoiseLevel:
      if (upperSW) {
        noiseLevelU = value;
      } else {
        noiseLevelL = value;
        if (wholemode) {
          noiseLevelU = value;
        }
      }
      noiseLevelstr = LINEARCENTREZERO[value >> midioutfrig];
      updatenoiseLevel(1);
      break;

    case CCosc2SawLevel:
      if (upperSW) {
        osc2SawLevelU = value;
      } else {
        osc2SawLevelL = value;
        if (wholemode) {
          osc2SawLevelU = value;
        }
      }
      osc2SawLevelstr = value >> midioutfrig;  // for display
      updateOsc2SawLevel(1);
      break;

    case CCosc1SawLevel:
      if (upperSW) {
        osc1SawLevelU = value;
      } else {
        osc1SawLevelL = value;
        if (wholemode) {
          osc1SawLevelU = value;
        }
      }
      osc1SawLevelstr = value >> midioutfrig;  // for display
      updateOsc1SawLevel(1);
      break;

    case CCosc2PulseLevel:
      if (upperSW) {
        osc2PulseLevelU = value;
      } else {
        osc2PulseLevelL = value;
        if (wholemode) {
          osc2PulseLevelU = value;
        }
      }
      osc2PulseLevelstr = value >> midioutfrig;  // for display
      updateOsc2PulseLevel(1);
      break;

    case CCosc1PulseLevel:
      if (upperSW) {
        osc1PulseLevelU = value;
      } else {
        osc1PulseLevelL = value;
        if (wholemode) {
          osc1PulseLevelU = value;
        }
      }
      osc1PulseLevelstr = value >> midioutfrig;  // for display
      updateOsc1PulseLevel(1);
      break;

    case CCosc2TriangleLevel:
      if (upperSW) {
        osc2TriangleLevelU = value;
      } else {
        osc2TriangleLevelL = value;
        if (wholemode) {
          osc2TriangleLevelU = value;
        }
      }
      osc2TriangleLevelstr = value >> midioutfrig;  // for display
      updateOsc2TriangleLevel(1);
      break;

    case CCosc1SubLevel:
      if (upperSW) {
        osc1SubLevelU = value;
      } else {
        osc1SubLevelL = value;
        if (wholemode) {
          osc1SubLevelU = value;
        }
      }
      osc1SubLevelstr = value >> midioutfrig;  // for display
      updateOsc1SubLevel(1);
      break;

    case CCLFODelay:
      if (upperSW) {
        LFODelayU = value;
      } else {
        LFODelayL = value;
        if (wholemode) {
          LFODelayU = value;
        }
      }
      LFODelaystr = value >> midioutfrig;  // for display
      updateLFODelay(1);
      break;

    case CCfilterCutoff:
      if (upperSW) {
        filterCutoffU = value;
        oldfilterCutoffU = value;
      } else {
        filterCutoffL = value;
        oldfilterCutoffL = value;
        if (wholemode) {
          filterCutoffU = value;
          oldfilterCutoffU = value;
        }
      }
      filterCutoffstr = FILTERCUTOFF[value >> midioutfrig];
      updateFilterCutoff(1);
      break;

    case CCfilterLFO:
      if (upperSW) {
        filterLFOU = value;
      } else {
        filterLFOL = value;
        if (wholemode) {
          filterLFOU = value;
        }
      }
      filterLFOstr = value >> midioutfrig;
      updatefilterLFO(1);
      break;

    case CCfilterRes:
      if (upperSW) {
        filterResU = value;
      } else {
        filterResL = value;
        if (wholemode) {
          filterResU = value;
        }
      }
      filterResstr = int(value >> midioutfrig);
      updatefilterRes(1);
      break;

    case CCfilterType:
      filterType = value;
      if (upperSW) {
        filterTypeU = value;
      } else {
        filterTypeL = value;
        if (wholemode) {
          filterTypeU = value;
        }
      }
      updateFilterType(1);
      break;

    case CCfilterEGlevel:
      if (upperSW) {
        filterEGlevelU = value;
      } else {
        filterEGlevelL = value;
        if (wholemode) {
          filterEGlevelU = value;
        }
      }
      filterEGlevelstr = int(value >> midioutfrig);
      updatefilterEGlevel(1);
      break;

    case CCLFORate:
      if (upperSW) {
        LFORateU = value;
      } else {
        LFORateL = value;
        if (wholemode) {
          LFORateU = value;
        }
      }
      LFORatestr = LFOTEMPO[value >> midioutfrig];  // for display
      updateLFORate(1);
      break;

    case CCmodWheelDepth:
      if (upperSW) {
        modWheelDepthU = value;
      } else {
        modWheelDepthL = value;
        if (wholemode) {
          modWheelDepthU = value;
        }
      }
      modWheelDepthstr = value >> midioutfrig;  // for display
      updatemodWheelDepth(1);
      break;

    case CCeffectPot1:
      if (upperSW) {
        effectPot1U = value;
      } else {
        effectPot1L = value;
        if (wholemode) {
          effectPot1U = value;
        }
      }
      effectPot1str = value >> midioutfrig;  // for display
      updateeffectPot1(1);
      break;

    case CCeffectPot2:
      if (upperSW) {
        effectPot2U = value;
      } else {
        effectPot2L = value;
        if (wholemode) {
          effectPot2U = value;
        }
      }
      effectPot2str = value >> midioutfrig;  // for display
      updateeffectPot2(1);
      break;

    case CCeffectPot3:
      if (upperSW) {
        effectPot3U = value;
      } else {
        effectPot3L = value;
        if (wholemode) {
          effectPot3U = value;
        }
      }
      effectPot3str = value >> midioutfrig;  // for display
      updateeffectPot3(1);
      break;

    case CCeffectsMix:
      if (upperSW) {
        effectsMixU = value;
      } else {
        effectsMixL = value;
        if (wholemode) {
          effectsMixU = value;
        }
      }
      effectsMixstr = value >> midioutfrig;  // for display
      updateeffectsMix(1);
      break;

    case CCLFOWaveform:
      if (upperSW) {
        LFOWaveformU = value;
      } else {
        LFOWaveformL = value;
        if (wholemode) {
          LFOWaveformU = value;
        }
      }
      LFOWaveform = value;
      updateStratusLFOWaveform(1);
      break;

    case CCfilterAttack:
      if (upperSW) {
        filterAttackU = value;
      } else {
        filterAttackL = value;
        if (wholemode) {
          filterAttackU = value;
        }
      }
      filterAttackstr = ENVTIMES[value >> midioutfrig];
      updatefilterAttack(1);
      break;

    case CCfilterDecay:
      if (upperSW) {
        filterDecayU = value;
      } else {
        filterDecayL = value;
        if (wholemode) {
          filterDecayU = value;
        }
      }
      filterDecaystr = ENVTIMES[value >> midioutfrig];
      updatefilterDecay(1);
      break;

    case CCfilterSustain:
      if (upperSW) {
        filterSustainU = value;
      } else {
        filterSustainL = value;
        if (wholemode) {
          filterSustainU = value;
        }
      }
      filterSustainstr = LINEAR_FILTERMIXERSTR[value >> midioutfrig];
      updatefilterSustain(1);
      break;

    case CCfilterRelease:
      if (upperSW) {
        filterReleaseU = value;
      } else {
        filterReleaseL = value;
        if (wholemode) {
          filterReleaseU = value;
        }
      }
      filterReleasestr = ENVTIMES[value >> midioutfrig];
      updatefilterRelease(1);
      break;

    case CCampAttack:
      if (upperSW) {
        ampAttackU = value;
        oldampAttackU = value;
      } else {
        ampAttackL = value;
        oldampAttackL = value;
        if (wholemode) {
          ampAttackU = value;
          oldampAttackU = value;
        }
      }
      ampAttackstr = ENVTIMES[value >> midioutfrig];
      updateampAttack(1);
      break;

    case CCampDecay:
      if (upperSW) {
        ampDecayU = value;
        oldampDecayU = value;
      } else {
        ampDecayL = value;
        oldampDecayL = value;
        if (wholemode) {
          ampDecayU = value;
          oldampDecayU = value;
        }
      }
      ampDecaystr = ENVTIMES[value >> midioutfrig];
      updateampDecay(1);
      break;

    case CCampSustain:
      if (upperSW) {
        ampSustainU = value;
        oldampSustainU = value;
      } else {
        ampSustainL = value;
        oldampSustainL = value;
        if (wholemode) {
          ampSustainU = value;
          oldampSustainU = value;
        }
      }
      ampSustainstr = LINEAR_FILTERMIXERSTR[value >> midioutfrig];
      updateampSustain(1);
      break;

    case CCampRelease:
      if (upperSW) {
        ampReleaseU = value;
        oldampReleaseU = value;
      } else {
        ampReleaseL = value;
        oldampReleaseL = value;
        if (wholemode) {
          ampReleaseU = value;
          oldampReleaseU = value;
        }
      }
      ampReleasestr = ENVTIMES[value >> midioutfrig];
      updateampRelease(1);
      break;

    case CCvolumeControl:
      if (upperSW) {
        volumeControlU = value;
      } else {
        volumeControlL = value;
        if (wholemode) {
          volumeControlU = value;
        }
      }
      volumeControlstr = value >> midioutfrig;
      updatevolumeControl(1);
      break;

    case CCPM_DCO2:
      if (upperSW) {
        pmDCO2U = value;
      } else {
        pmDCO2L = value;
        if (wholemode) {
          pmDCO2U = value;
        }
      }
      pmDCO2str = value >> midioutfrig;
      updatePM_DCO2(1);
      break;

    case CCPM_FilterEnv:
      if (upperSW) {
        pmFilterEnvU = value;
      } else {
        pmFilterEnvL = value;
        if (wholemode) {
          pmFilterEnvU = value;
        }
      }
      pmFilterEnvstr = value >> midioutfrig;
      updatePM_FilterEnv(1);
      break;

    case CCkeyTrack:
      if (upperSW) {
        keytrackU = value;
      } else {
        keytrackL = value;
        if (wholemode) {
          keytrackU = value;
        }
      }
      keytrackstr = value >> midioutfrig;
      updatekeytrack(1);
      break;


    case CCamDepth:
      if (upperSW) {
        amDepthU = value;
      } else {
        amDepthL = value;
        if (wholemode) {
          amDepthU = value;
        }
      }
      amDepth = value;
      amDepthstr = value >> midioutfrig;
      updateamDepth(1);
      break;

      //   ////////////////////////////////////////////////

    case CCglideSW:
      if (upperSW) {
        glideSWU = !glideSWU;
      } else {
        glideSWL = !glideSWL;
      }
      updateglideSW(1);
      break;

    case CCfilterPoleSW:
      if (upperSW) {
        filterPoleSWU = !filterPoleSWU;
      } else {
        filterPoleSWL = !filterPoleSWL;
      }
      updatefilterPoleSwitch(1);
      break;

      // case CCfilterVel:
      //   if (upperSW) {
      //     filterVelU = !filterVelU;
      //   } else {
      //     filterVelL = !filterVelL;
      //   }
      //   updatefilterVel(1);
      //   break;

    case CCfilterEGinv:
      if (upperSW) {
        filterEGinvU = !filterEGinvU;
      } else {
        filterEGinvL = !filterEGinvL;
      }
      updatefilterEGinv(1);
      break;

    case CCsyncSW:
      if (upperSW) {
        syncU = !syncU;
      } else {
        syncL = !syncL;
      }
      updatesyncSW(1);
      break;

      // case CCfilterLoop:
      //   if (upperSW) {
      //     statefilterLoopU = statefilterLoop;
      //   } else {
      //     statefilterLoopL = statefilterLoop;
      //   }
      //   updatefilterLoop(1);
      //   break;

      // case CCvcaLoop:
      //   if (upperSW) {
      //     statevcaLoopU = statevcaLoop;
      //   } else {
      //     statevcaLoopL = statevcaLoop;
      //   }
      //   updatevcaLoop(1);
      //   break;

    case CCchordHoldSW:
      if (upperSW) {
        chordHoldU = !chordHoldU;
      } else {
        chordHoldL = !chordHoldL;
      }
      updatechordHoldSW(1);
      break;

      // case CCvcaVel:
      //   if (upperSW) {
      //     vcaVelU = !vcaVelU;
      //   } else {
      //     vcaVelL = !vcaVelL;
      //   }
      //   updatevcaVel(1);
      //   break;

      // case CCvcaGate:
      //   if (upperSW) {
      //     vcaGateU = !vcaGateU;
      //   } else {
      //     vcaGateL = !vcaGateL;
      //   }
      //   updatevcaGate(1);
      //   break;

      // case CCwholemode:
      //   wholemode = 1;
      //   updatewholemode();
      //   break;

      // case CCdualmode:
      //   dualmode = 1;
      //   updatedualmode();
      //   break;

      // case CCsplitmode:
      //   splitmode = 1;
      //   updatesplitmode();
      //   break;


      // case CCmonoMulti:
      //   value > 0 ? monoMulti = 1 : monoMulti = 0;
      //   updateMonoMulti(1);
      //   break;

      //   // case CCPBDepth:
      //   //   PitchBendLevel = value;
      //   //   PitchBendLevelstr = PITCHBEND[value / midioutfrig];  // for display
      //   //   updatePitchBend();
      //   //   break;

    case CClfoAlt:
      if (upperSW) {
        lfoAltU = value;
      } else {
        lfoAltL = value;
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
      //       fmDepthU = int(modWheelLevel);
      //       fmDepthL = int(modWheelLevel);
      //       break;

      //     case 2:
      //       modWheelLevel = ((value) / 4);
      //       fmDepthU = int(modWheelLevel);
      //       fmDepthL = int(modWheelLevel);
      //       break;

      //     case 3:
      //       modWheelLevel = ((value) / 3.5);
      //       fmDepthU = int(modWheelLevel);
      //       fmDepthL = int(modWheelLevel);
      //       break;

      //     case 4:
      //       modWheelLevel = ((value) / 3);
      //       fmDepthU = int(modWheelLevel);
      //       fmDepthL = int(modWheelLevel);
      //       break;

      //     case 5:
      //       modWheelLevel = ((value) / 2.5);
      //       fmDepthU = int(modWheelLevel);
      //       fmDepthL = int(modWheelLevel);
      //       break;

      //     case 6:
      //       modWheelLevel = ((value) / 2);
      //       fmDepthU = int(modWheelLevel);
      //       fmDepthL = int(modWheelLevel);
      //       break;

      //     case 7:
      //       modWheelLevel = ((value) / 1.75);
      //       fmDepthU = int(modWheelLevel);
      //       fmDepthL = int(modWheelLevel);
      //       break;

      //     case 8:
      //       modWheelLevel = ((value) / 1.5);
      //       fmDepthU = int(modWheelLevel);
      //       fmDepthL = int(modWheelLevel);
      //       break;

      //     case 9:
      //       modWheelLevel = ((value) / 1.25);
      //       fmDepthU = int(modWheelLevel);
      //       fmDepthL = int(modWheelLevel);
      //       break;

      //     case 10:
      //       modWheelLevel = (value);
      //       fmDepthU = int(modWheelLevel);
      //       fmDepthL = int(modWheelLevel);
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
  // switch (AfterTouchDestU) {
  //   case 1:
  //     fmDepthU = afterTouch;
  //     break;
  //   case 2:
  //     filterCutoffU = (oldfilterCutoffU + afterTouch);
  //     if (afterTouch < 10) {
  //       filterCutoffU = oldfilterCutoffU;
  //     }
  //     if (filterCutoffU > 1023) {
  //       filterCutoffU = 1023;
  //     }
  //     break;
  //   case 3:
  //     filterLFOU = afterTouch;
  //     break;
  //   case 4:
  //     amDepthU = afterTouch;
  //     break;
  // }
  // switch (AfterTouchDestL) {
  //   case 1:
  //     fmDepthL = afterTouch;
  //     break;
  //   case 2:
  //     filterCutoffL = (oldfilterCutoffL + afterTouch);
  //     if (afterTouch < 10) {
  //       filterCutoffL = oldfilterCutoffL;
  //     }
  //     if (filterCutoffL > 1023) {
  //       filterCutoffL = 1023;
  //     }
  //     break;
  //   case 3:
  //     filterLFOL = afterTouch;
  //     break;
  //   case 4:
  //     amDepthL = afterTouch;
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
    pwLFOU = data[1].toInt();
    fmDepthU = data[2].toInt();
    osc2PWU = data[3].toInt();
    osc2PWMU = data[4].toInt();
    osc1PWU = data[5].toInt();
    osc1PWMU = data[6].toInt();
    osc1RangeU = data[7].toInt();
    osc2RangeU = data[8].toInt();
    osc2IntervalU = data[9].toInt();
    glideTimeU = data[10].toInt();
    osc2DetuneU = data[11].toInt();
    noiseLevelU = data[12].toInt();
    osc2SawLevelU = data[13].toInt();
    osc1SawLevelU = data[14].toInt();
    osc2PulseLevelU = data[15].toInt();
    osc1PulseLevelU = data[16].toInt();
    filterCutoffU = data[17].toInt();
    filterLFOU = data[18].toInt();
    filterResU = data[19].toInt();
    filterTypeU = data[20].toInt();
    filterdoubleLoopU = data[21].toInt();
    vcadoubleLoopU = data[22].toInt();
    LFODelayGoU = data[23].toInt();
    filterEGlevelU = data[24].toInt();
    LFORateU = data[25].toInt();
    LFOWaveformU = data[26].toInt();
    filterAttackU = data[27].toInt();
    filterDecayU = data[28].toInt();
    filterSustainU = data[29].toInt();
    filterReleaseU = data[30].toInt();
    ampAttackU = data[31].toInt();
    ampDecayU = data[32].toInt();
    ampSustainU = data[33].toInt();
    ampReleaseU = data[34].toInt();
    volumeControlU = data[35].toInt();
    glideSWU = data[36].toInt();
    keytrackU = data[37].toInt();
    filterPoleSWU = data[38].toInt();
    filterLoopU = data[39].toInt();
    filterEGinvU = data[40].toInt();
    filterVelU = data[41].toInt();
    vcaLoopU = data[42].toInt();
    vcaVelU = data[43].toInt();
    vcaGateU = data[44].toInt();
    lfoAltU = data[45].toInt();
    chorus1U = data[46].toInt();
    chorus2U = data[47].toInt();
    monoMultiU = data[48].toInt();
    modWheelLevelU = data[49].toInt();
    PitchBendLevelU = data[50].toInt();
    amDepthU = data[51].toInt();
    syncU = data[52].toInt();
    effectPot1U = data[53].toInt();
    effectPot2U = data[54].toInt();
    effectPot3U = data[55].toInt();
    oldampAttackU = data[56].toInt();
    oldampDecayU = data[57].toInt();
    oldampSustainU = data[58].toInt();
    oldampReleaseU = data[59].toInt();
    AfterTouchDestU = data[60].toInt();
    filterLogLinU = data[61].toInt();
    ampLogLinU = data[62].toInt();
    osc2TriangleLevelU = data[63].toInt();
    osc1SubLevelU = data[64].toInt();
    keyTrackSWU = data[65].toInt();
    LFODelayU = data[66].toInt();

    oldfilterCutoffU = filterCutoffU;

  } else {
    patchNameL = data[0];
    pwLFOL = data[1].toInt();
    fmDepthL = data[2].toInt();
    osc2PWL = data[3].toInt();
    osc2PWML = data[4].toInt();
    osc1PWL = data[5].toInt();
    osc1PWML = data[6].toInt();
    osc1RangeL = data[7].toInt();
    osc2RangeL = data[8].toInt();
    osc2IntervalL = data[9].toInt();
    glideTimeL = data[10].toInt();
    osc2DetuneL = data[11].toInt();
    noiseLevelL = data[12].toInt();
    osc2SawLevelL = data[13].toInt();
    osc1SawLevelL = data[14].toInt();
    osc2PulseLevelL = data[15].toInt();
    osc1PulseLevelL = data[16].toInt();
    filterCutoffL = data[17].toInt();
    filterLFOL = data[18].toInt();
    filterResL = data[19].toInt();
    filterTypeL = data[20].toInt();
    filterdoubleLoopL = data[21].toInt();
    vcadoubleLoopL = data[22].toInt();
    LFODelayGoL = data[23].toInt();
    filterEGlevelL = data[24].toInt();
    LFORateL = data[25].toInt();
    LFOWaveformL = data[26].toInt();
    filterAttackL = data[27].toInt();
    filterDecayL = data[28].toInt();
    filterSustainL = data[29].toInt();
    filterReleaseL = data[30].toInt();
    ampAttackL = data[31].toInt();
    ampDecayL = data[32].toInt();
    ampSustainL = data[33].toInt();
    ampReleaseL = data[34].toInt();
    volumeControlL = data[35].toInt();
    glideSWL = data[36].toInt();
    keytrackL = data[37].toInt();
    filterPoleSWL = data[38].toInt();
    filterLoopL = data[39].toInt();
    filterEGinvL = data[40].toInt();
    filterVelL = data[41].toInt();
    vcaLoopL = data[42].toInt();
    vcaVelL = data[43].toInt();
    vcaGateL = data[44].toInt();
    lfoAltL = data[45].toInt();
    chorus1L = data[46].toInt();
    chorus2L = data[47].toInt();
    monoMultiL = data[48].toInt();
    modWheelLevelL = data[49].toInt();
    PitchBendLevelL = data[50].toInt();
    amDepthL = data[51].toInt();
    syncL = data[52].toInt();
    effectPot1L = data[53].toInt();
    effectPot2L = data[54].toInt();
    effectPot3L = data[55].toInt();
    oldampAttackL = data[56].toInt();
    oldampDecayL = data[57].toInt();
    oldampSustainL = data[58].toInt();
    oldampReleaseL = data[59].toInt();
    AfterTouchDestL = data[60].toInt();
    filterLogLinL = data[61].toInt();
    ampLogLinL = data[62].toInt();
    osc2TriangleLevelL = data[63].toInt();
    osc1SubLevelL = data[64].toInt();
    keyTrackSWL = data[65].toInt();
    LFODelayL = data[66].toInt();

    oldfilterCutoffL = filterCutoffL;

    if (wholemode) {
      patchNameU = data[0];
      pwLFOU = data[1].toInt();
      fmDepthU = data[2].toInt();
      osc2PWU = data[3].toInt();
      osc2PWMU = data[4].toInt();
      osc1PWU = data[5].toInt();
      osc1PWMU = data[6].toInt();
      osc1RangeU = data[7].toInt();
      osc2RangeU = data[8].toInt();
      osc2IntervalU = data[9].toInt();
      glideTimeU = data[10].toInt();
      osc2DetuneU = data[11].toInt();
      noiseLevelU = data[12].toInt();
      osc2SawLevelU = data[13].toInt();
      osc1SawLevelU = data[14].toInt();
      osc2PulseLevelU = data[15].toInt();
      osc1PulseLevelU = data[16].toInt();
      filterCutoffU = data[17].toInt();
      filterLFOU = data[18].toInt();
      filterResU = data[19].toInt();
      filterTypeU = data[20].toInt();
      filterdoubleLoopU = data[21].toInt();
      vcadoubleLoopU = data[22].toInt();
      LFODelayGoU = data[23].toInt();
      filterEGlevelU = data[24].toInt();
      LFORateU = data[25].toInt();
      LFOWaveformU = data[26].toInt();
      filterAttackU = data[27].toInt();
      filterDecayU = data[28].toInt();
      filterSustainU = data[29].toInt();
      filterReleaseU = data[30].toInt();
      ampAttackU = data[31].toInt();
      ampDecayU = data[32].toInt();
      ampSustainU = data[33].toInt();
      ampReleaseU = data[34].toInt();
      volumeControlU = data[35].toInt();
      glideSWU = data[36].toInt();
      keytrackU = data[37].toInt();
      filterPoleSWU = data[38].toInt();
      filterLoopU = data[39].toInt();
      filterEGinvU = data[40].toInt();
      filterVelU = data[41].toInt();
      vcaLoopU = data[42].toInt();
      vcaVelU = data[43].toInt();
      vcaGateU = data[44].toInt();
      lfoAltU = data[45].toInt();
      chorus1U = data[46].toInt();
      chorus2U = data[47].toInt();
      monoMultiU = data[48].toInt();
      modWheelLevelU = data[49].toInt();
      PitchBendLevelU = data[50].toInt();
      amDepthU = data[51].toInt();
      syncU = data[52].toInt();
      effectPot1U = data[53].toInt();
      effectPot2U = data[54].toInt();
      effectPot3U = data[55].toInt();
      oldampAttackU = data[56].toInt();
      oldampDecayU = data[57].toInt();
      oldampSustainU = data[58].toInt();
      oldampReleaseU = data[59].toInt();
      AfterTouchDestU = data[60].toInt();
      filterLogLinU = data[61].toInt();
      ampLogLinU = data[62].toInt();
      osc2TriangleLevelU = data[63].toInt();
      osc1SubLevelU = data[64].toInt();
      keyTrackSWU = data[65].toInt();
      LFODelayU = data[66].toInt();

      oldfilterCutoffU = filterCutoffU;
    }
  }

  if (wholemode) {
    //Switches
    oldupperSW = upperSW;
    //for (upperSW = 0; upperSW < 2; upperSW++) {
    // midiCCOut52(CCupperSW, upperSW);
    // delay(1);
    // midiCCOut52(CClowerSW, lowerSW);
    // delay(1);
    midiCCOut53(CCdumpStartedSW, 1);
    delay(1);
    updateglideTime(0);
    updateglideSW(0);
    // updatefilterLoop(0);
    // updatefilterVel(0);
    // updatevcaLoop(0);
    // updatevcaVel(0);
    // updatevcaGate(0);
    updateosc1Range(0);
    updatefmDepth(0);
    // updateMonoMulti(0);
    // updateFilterEnv(0);
    // updateAmpEnv(0);
    updatesyncSW(0);
    updateOsc1SawLevel(0);
    updateOsc1PulseLevel(0);
    updateOsc1SubLevel(0);
    updateosc1PW(0);
    updateosc1PWM(0);
    updateosc2Range(0);
    updateosc2PW(0);
    updateosc2PWM(0);
    updateOsc2SawLevel(0);
    updateOsc2PulseLevel(0);
    updateOsc2TriangleLevel(0);
    updateosc2Detune(0);
    updateosc2Interval(0);
    updateFilterCutoff(0);
    updatefilterPoleSwitch(0);
    updatefilterRes(0);
    updatekeytrack(0);
    updatefilterEGlevel(0);
    updatefilterLFO(0);
    updateFilterType(0);
    updatefilterAttack(0);
    updatefilterDecay(0);
    updatefilterSustain(0);
    updatefilterRelease(0);
    updatefilterEGinv(0);
    updateampAttack(0);
    updateampDecay(0);
    updateampSustain(0);
    updateampRelease(0);
    updateLFORate(0);
    updateLFODelay(0);
    updatepwLFO(0);
    updatelfoAlt(0);
    updateStratusLFOWaveform(0);
    updateeffectPot1(0);
    updateeffectPot2(0);
    updateeffectPot3(0);
    updateeffectsMix(0);
    updatevolumeControl(0);
    updateamDepth(0);
    updatePM_DCO2(0);
    updatePM_FilterEnv(0);
    delay(1);
    midiCCOut53(CCdumpCompleteSW, 0);
    //}
    upperSW = oldupperSW;
  }

  // else {
  //   midiCCOut52(CClowerSW, 1);
  //   delay(1);
  //   midiCCOut52(CCupperSW, 0);
  //   delay(1);
  // midiCCOut53(CCdumpStartedSW, 1);
  // delay(1);
  // updateglideTime(0);
  // updateglideSW(0);
  // // updatefilterLoop(0);
  // // updatefilterVel(0);
  // // updatevcaLoop(0);
  // // updatevcaVel(0);
  // // updatevcaGate(0);
  // // updatelfoAlt(0);
  // updateosc1Range(0);
  // updatefmDepth(0);
  // // updateMonoMulti(0);
  // // updateFilterEnv(0);
  // // updateAmpEnv(0);
  // updatesyncSW(0);
  // updateOsc1SawLevel(0);
  // updateOsc1PulseLevel(0);
  // updateOsc1SubLevel(0);
  // updateosc1PW(0);
  // updateosc1PWM(0);
  // updateosc2Range(0);
  // updateosc2PW(0);
  // updateosc2PWM(0);
  // updateOsc2SawLevel(0);
  // updateOsc2PulseLevel(0);
  // updateOsc2TriangleLevel(0);
  // updateosc2Detune(0);
  // updateosc2Interval(0);
  // updateFilterCutoff(0);
  // updatefilterPoleSwitch(0);
  // updatefilterRes(0);
  // updatekeytrack(0);
  // updatefilterEGlevel(0);
  // updatefilterLFO(0);
  // updateFilterType(0);
  // updatefilterAttack(0);
  // updatefilterDecay(0);
  // updatefilterSustain(0);
  // updatefilterRelease(0);
  // updatefilterEGinv(0);
  // updateampAttack(0);
  // updateampDecay(0);
  // updateampSustain(0);
  // updateampRelease(0);
  // updateLFORate(0);
  // updateLFODelay(0);
  // updatepwLFO(0);
  // updateeffectPot1(0);
  // updateeffectPot2(0);
  // updateeffectPot3(0);
  // updateeffectsMix(0);
  // updatevolumeControl(0);
  // updateamDepth(0);
  // updatePM_DCO2(0);
  // updatePM_FilterEnv(0);
  // delay(1);
  // midiCCOut53(CCdumpCompleteSW, 0);
  // }

  //Patchname
  updatePatchname();
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
    return patchNameU + "," + String(pwLFOU) + "," + String(fmDepthU) + "," + String(osc2PWU) + "," + String(osc2PWMU) + "," + String(osc1PWU) + "," + String(osc1PWMU) + "," + String(osc1RangeU) + "," + String(osc2RangeU) + "," + String(osc2IntervalU) + "," + String(glideTimeU) + "," + String(osc2DetuneU) + "," + String(noiseLevelU) + "," + String(osc2SawLevelU) + "," + String(osc1SawLevelU) + "," + String(osc2PulseLevelU) + "," + String(osc1PulseLevelU) + "," + String(filterCutoffU) + "," + String(filterLFOU) + "," + String(filterResU) + "," + String(filterTypeU) + "," + String(filterdoubleLoopU) + "," + String(vcadoubleLoopU) + "," + String(LFODelayGoU) + "," + String(filterEGlevelU) + "," + String(LFORateU) + "," + String(LFOWaveformU) + "," + String(filterAttackU) + "," + String(filterDecayU) + "," + String(filterSustainU) + "," + String(filterReleaseU) + "," + String(ampAttackU) + "," + String(ampDecayU) + "," + String(ampSustainU) + "," + String(ampReleaseU) + "," + String(volumeControlU) + "," + String(glideSWU) + "," + String(keytrackU) + "," + String(filterPoleSWU) + "," + String(filterLoopU) + "," + String(filterEGinvU) + "," + String(filterVelU) + "," + String(vcaLoopU) + "," + String(vcaVelU) + "," + String(vcaGateU) + "," + String(lfoAltU) + "," + String(chorus1U) + "," + String(chorus2U) + "," + String(monoMultiU) + "," + String(modWheelLevelU) + "," + String(PitchBendLevelU) + "," + String(amDepthU) + "," + String(syncU) + "," + String(effectPot1U) + "," + String(effectPot2U) + "," + String(effectPot3U) + "," + String(oldampAttackU) + "," + String(oldampDecayU) + "," + String(oldampSustainU) + "," + String(oldampReleaseU) + "," + String(AfterTouchDestU) + "," + String(filterLogLinU) + "," + String(ampLogLinU) + "," + String(osc2TriangleLevelU) + "," + String(osc1SubLevelU) + "," + String(keyTrackSWU) + "," + String(LFODelayU);
  } else {
    return patchNameL + "," + String(pwLFOL) + "," + String(fmDepthL) + "," + String(osc2PWL) + "," + String(osc2PWML) + "," + String(osc1PWL) + "," + String(osc1PWML) + "," + String(osc1RangeL) + "," + String(osc2RangeL) + "," + String(osc2IntervalL) + "," + String(glideTimeL) + "," + String(osc2DetuneL) + "," + String(noiseLevelL) + "," + String(osc2SawLevelL) + "," + String(osc1SawLevelL) + "," + String(osc2PulseLevelL) + "," + String(osc1PulseLevelL) + "," + String(filterCutoffL) + "," + String(filterLFOL) + "," + String(filterResL) + "," + String(filterTypeL) + "," + String(filterdoubleLoopL) + "," + String(vcadoubleLoopL) + "," + String(LFODelayGoL) + "," + String(filterEGlevelL) + "," + String(LFORateL) + "," + String(LFOWaveformL) + "," + String(filterAttackL) + "," + String(filterDecayL) + "," + String(filterSustainL) + "," + String(filterReleaseL) + "," + String(ampAttackL) + "," + String(ampDecayL) + "," + String(ampSustainL) + "," + String(ampReleaseL) + "," + String(volumeControlL) + "," + String(glideSWL) + "," + String(keytrackL) + "," + String(filterPoleSWL) + "," + String(filterLoopL) + "," + String(filterEGinvL) + "," + String(filterVelL) + "," + String(vcaLoopL) + "," + String(vcaVelL) + "," + String(vcaGateL) + "," + String(lfoAltL) + "," + String(chorus1L) + "," + String(chorus2L) + "," + String(monoMultiL) + "," + String(modWheelLevelL) + "," + String(PitchBendLevelL) + "," + String(amDepthL) + "," + String(syncL) + "," + String(effectPot1L) + "," + String(effectPot2L) + "," + String(effectPot3L) + "," + String(oldampAttackL) + "," + String(oldampDecayL) + "," + String(oldampSustainL) + "," + String(oldampReleaseL) + "," + String(AfterTouchDestL) + "," + String(filterLogLinL) + "," + String(ampLogLinL) + "," + String(osc2TriangleLevelL) + "," + String(osc1SubLevelL) + "," + String(keyTrackSWL) + "," + String(LFODelayL);
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
      case MUX3_spare0:
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

void midiCCOut51(byte cc, byte value) {
  Serial.print("Sent on channel 1 from the controller ");
  Serial.println(value);
  MIDI5.sendControlChange(cc, value, 1);  //MIDI DIN is set to Out
}

void midiCCOut52(byte cc, byte value) {
  Serial.print("Sent on channel 2 from the controller ");
  Serial.println(value);
  MIDI5.sendControlChange(cc, value, 2);  //MIDI DIN is set to Out
}

void midiCCOut53(byte cc, byte value) {
  // Serial.print("Sent on channel 3 from the controller ");
  // Serial.println(value);
  MIDI5.sendControlChange(cc, value, 3);  //MIDI DIN is set to Out
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
      sample_data1 = (channel_a & 0xFFF0000F) | (((int(noiseLevelU * MULT2V)) & 0xFFFF) << 4);
      sample_data2 = (channel_c & 0xFFF0000F) | (((int(noiseLevelL * MULT2V)) & 0xFFFF) << 4);

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(filterAttackU * MULT5V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(filterAttackL * MULT5V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 1:
      sample_data1 = (channel_a & 0xFFF0000F) | (((int(osc1SawLevelU * MULT2V)) & 0xFFFF) << 4);
      sample_data2 = (channel_c & 0xFFF0000F) | (((int(osc1SawLevelL * MULT2V)) & 0xFFFF) << 4);

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(filterDecayU * MULT5V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(filterDecayL * MULT5V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 2:
      sample_data1 = (channel_a & 0xFFF0000F) | (((int(osc1PulseLevelU * MULT2V)) & 0xFFFF) << 4);
      sample_data2 = (channel_c & 0xFFF0000F) | (((int(osc1PulseLevelL * MULT2V)) & 0xFFFF) << 4);

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(filterSustainU * MULT5V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(filterSustainL * MULT5V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 3:
      sample_data1 = (channel_a & 0xFFF0000F) | (((int(osc1SubLevelU * MULT2V)) & 0xFFFF) << 4);
      sample_data2 = (channel_c & 0xFFF0000F) | (((int(osc1SubLevelL * MULT2V)) & 0xFFFF) << 4);

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(filterReleaseU * MULT5V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(filterReleaseL * MULT5V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 4:
      sample_data1 = (channel_a & 0xFFF0000F) | (((int(pmDCO2U * MULT2V)) & 0xFFFF) << 4);
      sample_data2 = (channel_c & 0xFFF0000F) | (((int(pmDCO2L * MULT2V)) & 0xFFFF) << 4);

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(ampAttackU * MULT5V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(ampAttackL * MULT5V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 5:
      sample_data1 = (channel_a & 0xFFF0000F) | (((int(pmFilterEnvU * MULT2V)) & 0xFFFF) << 4);
      sample_data2 = (channel_c & 0xFFF0000F) | (((int(pmFilterEnvL * MULT2V)) & 0xFFFF) << 4);

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(ampDecayU * MULT5V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(ampDecayL * MULT5V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 6:
      sample_data1 = (channel_a & 0xFFF0000F) | (((int(osc2SawLevelU * MULT2V)) & 0xFFFF) << 4);
      sample_data2 = (channel_c & 0xFFF0000F) | (((int(osc2SawLevelL * MULT2V)) & 0xFFFF) << 4);

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(ampSustainU * MULT5V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(ampSustainL * MULT5V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 7:
      sample_data1 = (channel_a & 0xFFF0000F) | (((int(osc2PulseLevelU * MULT2V)) & 0xFFFF) << 4);
      sample_data2 = (channel_c & 0xFFF0000F) | (((int(osc2PulseLevelL * MULT2V)) & 0xFFFF) << 4);

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(ampReleaseU * MULT5V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(ampReleaseL * MULT5V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 8:
      sample_data1 = (channel_a & 0xFFF0000F) | (((int(osc2TriangleLevelU * MULT2V)) & 0xFFFF) << 4);
      sample_data2 = (channel_c & 0xFFF0000F) | (((int(osc2TriangleLevelL * MULT2V)) & 0xFFFF) << 4);

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(filterEGlevelU * MULT5V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(filterEGlevelL * MULT5V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 9:
      sample_data1 = (channel_a & 0xFFF0000F) | (((int(volumeControlU * MULT2V)) & 0xFFFF) << 4);
      sample_data2 = (channel_c & 0xFFF0000F) | (((int(volumeControlL * MULT2V)) & 0xFFFF) << 4);

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(filterCutoffU * MULT5V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(filterCutoffL * MULT5V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 10:
      sample_data1 = (channel_a & 0xFFF0000F) | (((int(effectsMixU * MULT2V)) & 0xFFFF) << 4);
      sample_data2 = (channel_c & 0xFFF0000F) | (((int(effectsMixL * MULT2V)) & 0xFFFF) << 4);

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(filterResU * MULT5V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(filterResL * MULT5V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 11:
      switch (LFODelayGoU) {
        case 1:
          sample_data1 = (channel_a & 0xFFF0000F) | (((int(fmDepthU * MULT2V)) & 0xFFFF) << 4);
          break;

        case 0:
          sample_data1 = (channel_a & 0xFFF0000F) | ((0 & 0xFFFF) << 4);
          break;
      }
      switch (LFODelayGoL) {
        case 1:
          sample_data2 = (channel_c & 0xFFF0000F) | (((int(fmDepthL * MULT2V)) & 0xFFFF) << 4);
          break;

        case 0:
          sample_data2 = (channel_c & 0xFFF0000F) | ((0 & 0xFFFF) << 4);
          break;
      }

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(LFORateU * MULT5V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(LFORateL * MULT5V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 12:
      switch (LFODelayGoU) {
        case 1:
          sample_data1 = (channel_a & 0xFFF0000F) | (((int(filterLFOU * MULT2V)) & 0xFFFF) << 4);
          break;

        case 0:
          sample_data1 = (channel_a & 0xFFF0000F) | ((0 & 0xFFFF) << 4);
          break;
      }
      switch (LFODelayGoL) {
        case 1:
          sample_data2 = (channel_c & 0xFFF0000F) | (((int(filterLFOL * MULT2V)) & 0xFFFF) << 4);
          break;

        case 0:
          sample_data2 = (channel_c & 0xFFF0000F) | ((0 & 0xFFFF) << 4);
          break;
      }

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(LFOWaveformU * MULT5V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(LFOWaveformL * MULT5V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 13:
      switch (LFODelayGoU) {
        case 1:
          sample_data1 = (channel_a & 0xFFF0000F) | (((int(amDepthU * MULT2V)) & 0xFFFF) << 4);
          break;

        case 0:
          sample_data1 = (channel_a & 0xFFF0000F) | ((0 & 0xFFFF) << 4);
          break;
      }
      switch (LFODelayGoL) {
        case 1:
          sample_data2 = (channel_c & 0xFFF0000F) | (((int(amDepthL * MULT2V)) & 0xFFFF) << 4);
          break;

        case 0:
          sample_data2 = (channel_c & 0xFFF0000F) | ((0 & 0xFFFF) << 4);
          break;
      }

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(effectPot1U * MULT33V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(effectPot1L * MULT33V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 14:
      sample_data1 = (channel_a & 0xFFF0000F) | ((0 & 0xFFFF) << 4);
      sample_data2 = (channel_c & 0xFFF0000F) | ((0 & 0xFFFF) << 4);

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(effectPot2U * MULT33V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(effectPot2L * MULT33V)) & 0xFFFF) << 4);
      outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
      digitalWriteFast(DEMUX_EN_1, LOW);
      break;

    case 15:
      sample_data1 = (channel_a & 0xFFF0000F) | (((pwLFOU * MULT5V) & 0xFFFF) << 4);
      sample_data2 = (channel_c & 0xFFF0000F) | (((pwLFOL * MULT5V) & 0xFFFF) << 4);

      sample_data3 = (channel_b & 0xFFF0000F) | (((int(effectPot3U * MULT33V)) & 0xFFFF) << 4);
      sample_data4 = (channel_d & 0xFFF0000F) | (((int(effectPot3L * MULT33V)) & 0xFFFF) << 4);
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

// void writeDemux() {

//   switch (muxOutput) {
//     // case 0:
//     //   switch (LFODelayGoU) {
//     //     case 1:
//     //       sample_data1 = (channel_a & 0xFFF0000F) | (((int(fmDepthU * DACMULT)) & 0xFFFF) << 4);
//     //       break;

//     //     case 0:
//     //       sample_data1 = (channel_a & 0xFFF0000F) | ((0 & 0xFFFF) << 4);
//     //       break;
//     //   }
//     //   switch (LFODelayGoL) {
//     //     case 1:
//     //       sample_data2 = (channel_c & 0xFFF0000F) | (((int(fmDepthL * DACMULT)) & 0xFFFF) << 4);
//     //       break;

//     //     case 0:
//     //       sample_data2 = (channel_c & 0xFFF0000F) | ((0 & 0xFFFF) << 4);
//     //       break;
//     //   }

//     //   sample_data3 = (channel_b & 0xFFF0000F) | (((int(filterAttackU * DACMULT)) & 0xFFFF) << 4);
//     //   sample_data3 = (channel_d & 0xFFF0000F) | (((int(filterAttackL * DACMULT)) & 0xFFFF) << 4);
//     //   outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
//     //   digitalWriteFast(DEMUX_EN_1, LOW);
//     //   break;

//     // case 1:
//     //   sample_data1 = (channel_a & 0xFFF0000F) | (((int(osc2PWMU * DACMULT)) & 0xFFFF) << 4);
//     //   sample_data2 = (channel_c & 0xFFF0000F) | (((int(osc2PWML * DACMULT)) & 0xFFFF) << 4);

//     //   sample_data3 = (channel_b & 0xFFF0000F) | (((int(filterDecayU * DACMULT)) & 0xFFFF) << 4);
//     //   sample_data4 = (channel_d & 0xFFF0000F) | (((int(filterDecayL * DACMULT)) & 0xFFFF) << 4);
//     //   outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
//     //   digitalWriteFast(DEMUX_EN_1, LOW);
//     //   break;

//     // case 2:
//     //   sample_data1 = (channel_a & 0xFFF0000F) | (((int(osc1PWMU * DACMULT)) & 0xFFFF) << 4);
//     //   sample_data2 = (channel_c & 0xFFF0000F) | (((int(osc1PWML * DACMULT)) & 0xFFFF) << 4);

//     //   sample_data3 = (channel_b & 0xFFF0000F) | (((int(filterSustainU * DACMULT)) & 0xFFFF) << 4);
//     //   sample_data4 = (channel_d & 0xFFF0000F) | (((int(filterSustainL * DACMULT)) & 0xFFFF) << 4);
//     //   outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
//     //   digitalWriteFast(DEMUX_EN_1, LOW);
//     //   break;

//     // case 3:
//     //   sample_data1 = (channel_a & 0xFFF0000F) | (((int(stackU * DACMULT)) & 0xFFFF) << 4);
//     //   sample_data2 = (channel_c & 0xFFF0000F) | (((int(stackL * DACMULT)) & 0xFFFF) << 4);

//     //   sample_data3 = (channel_b & 0xFFF0000F) | (((int(filterReleaseU * DACMULT)) & 0xFFFF) << 4);
//     //   sample_data4 = (channel_d & 0xFFF0000F) | (((int(filterReleaseL * DACMULT)) & 0xFFFF) << 4);
//     //   outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
//     //   digitalWriteFast(DEMUX_EN_1, LOW);
//     //   break;

//     // case 4:
//     //   sample_data1 = (channel_a & 0xFFF0000F) | (((int(osc2DetuneU * DACMULT)) & 0xFFFF) << 4);
//     //   sample_data2 = (channel_c & 0xFFF0000F) | (((int(osc2DetuneL * DACMULT)) & 0xFFFF) << 4);

//     //   sample_data3 = (channel_b & 0xFFF0000F) | (((int(ampAttackU * DACMULT)) & 0xFFFF) << 4);
//     //   sample_data4 = (channel_d & 0xFFF0000F) | (((int(ampAttackL * DACMULT)) & 0xFFFF) << 4);
//     //   outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
//     //   digitalWriteFast(DEMUX_EN_1, LOW);
//     //   break;

//     // case 5:
//     //   sample_data1 = (channel_a & 0xFFF0000F) | (((int(noiseLevelU * DACMULT)) & 0xFFFF) << 4);
//     //   sample_data2 = (channel_c & 0xFFF0000F) | (((int(noiseLevelL * DACMULT)) & 0xFFFF) << 4);

//     //   sample_data3 = (channel_b & 0xFFF0000F) | (((int(ampDecayU * DACMULT)) & 0xFFFF) << 4);
//     //   sample_data4 = (channel_d & 0xFFF0000F) | (((int(ampDecayL * DACMULT)) & 0xFFFF) << 4);
//     //   outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
//     //   digitalWriteFast(DEMUX_EN_1, LOW);
//     //   break;

//     // case 6:
//     //   switch (LFODelayGoU) {
//     //     case 1:
//     //       sample_data1 = (channel_a & 0xFFF0000F) | (((int(filterLFOU * DACMULT)) & 0xFFFF) << 4);
//     //       break;

//     //     case 0:
//     //       sample_data1 = (channel_a & 0xFFF0000F) | ((0 & 0xFFFF) << 4);
//     //       break;
//     //   }
//     //   switch (LFODelayGoL) {
//     //     case 1:
//     //       sample_data2 = (channel_c & 0xFFF0000F) | (((int(filterLFOL * DACMULT)) & 0xFFFF) << 4);
//     //       break;

//     //     case 0:
//     //       sample_data2 = (channel_c & 0xFFF0000F) | ((0 & 0xFFFF) << 4);
//     //       break;
//     //   }

//     //   sample_data3 = (channel_b & 0xFFF0000F) | (((int(ampSustainU * DACMULT)) & 0xFFFF) << 4);
//     //   sample_data4 = (channel_d & 0xFFF0000F) | (((int(ampSustainL * DACMULT)) & 0xFFFF) << 4);
//     //   outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
//     //   digitalWriteFast(DEMUX_EN_1, LOW);
//     //   break;

//     // case 7:
//     //   sample_data1 = (channel_a & 0xFFF0000F) | (((int(volumeControlU * DACMULT)) & 0xFFFF) << 4);
//     //   sample_data2 = (channel_c & 0xFFF0000F) | (((int(volumeControlL * DACMULT)) & 0xFFFF) << 4);

//     //   sample_data3 = (channel_b & 0xFFF0000F) | (((int(ampReleaseU * DACMULT)) & 0xFFFF) << 4);
//     //   sample_data4 = (channel_d & 0xFFF0000F) | (((int(ampReleaseL * DACMULT)) & 0xFFFF) << 4);
//     //   outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
//     //   digitalWriteFast(DEMUX_EN_1, LOW);
//     //   break;

//     // case 8:
//     //   sample_data1 = (channel_a & 0xFFF0000F) | (((int(osc1SawLevelU * DACMULT)) & 0xFFFF) << 4);
//     //   sample_data2 = (channel_c & 0xFFF0000F) | (((int(osc1SawLevelL * DACMULT)) & 0xFFFF) << 4);

//     //   sample_data3 = (channel_b & 0xFFF0000F) | (((int(pwLFOU * DACMULT)) & 0xFFFF) << 4);
//     //   sample_data4 = (channel_d & 0xFFF0000F) | (((int(pwLFOL * DACMULT)) & 0xFFFF) << 4);
//     //   outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
//     //   digitalWriteFast(DEMUX_EN_1, LOW);
//     //   break;

//     // case 9:
//     //   sample_data1 = (channel_a & 0xFFF0000F) | (((int(osc1PulseLevelU * DACMULT)) & 0xFFFF) << 4);
//     //   sample_data2 = (channel_c & 0xFFF0000F) | (((int(osc1PulseLevelL * DACMULT)) & 0xFFFF) << 4);

//     //   sample_data3 = (channel_b & 0xFFF0000F) | (((int(LFORateU * DACMULT)) & 0xFFFF) << 4);
//     //   sample_data4 = (channel_d & 0xFFF0000F) | (((int(LFORateL * DACMULT)) & 0xFFFF) << 4);
//     //   outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
//     //   digitalWriteFast(DEMUX_EN_1, LOW);
//     //   break;

//     // case 10:
//     //   sample_data1 = (channel_a & 0xFFF0000F) | (((int(osc2SawLevelU * DACMULT)) & 0xFFFF) << 4);
//     //   sample_data2 = (channel_c & 0xFFF0000F) | (((int(osc2SawLevelL * DACMULT)) & 0xFFFF) << 4);

//     //   sample_data3 = (channel_b & 0xFFF0000F) | (((int(LFOWaveformU * DACMULT)) & 0xFFFF) << 4);
//     //   sample_data4 = (channel_d & 0xFFF0000F) | (((int(LFOWaveformL * DACMULT)) & 0xFFFF) << 4);
//     //   outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
//     //   digitalWriteFast(DEMUX_EN_1, LOW);
//     //   break;

//     case 11:
//       sample_data1 = (channel_a & 0xFFF0000F) | (((int(fmDepthU * MULT2V)) & 0xFFFF) << 4);
//       sample_data2 = (channel_c & 0xFFF0000F) | (((int(fmDepthL * MULT2V)) & 0xFFFF) << 4);

//       sample_data3 = (channel_b & 0xFFF0000F) | (((int(LFORateU * MULT5V)) & 0xFFFF) << 4);
//       sample_data4 = (channel_d & 0xFFF0000F) | (((int(LFORateL * MULT5V)) & 0xFFFF) << 4);
//       outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
//       digitalWriteFast(DEMUX_EN_1, LOW);
//       break;

//     // case 12:
//     //   sample_data1 = (channel_a & 0xFFF0000F) | (((int(keytrackU * 12)) & 0xFFFF) << 4);
//     //   sample_data2 = (channel_c & 0xFFF0000F) | (((int(keytrackL * 12)) & 0xFFFF) << 4);

//     //   sample_data3 = (channel_b & 0xFFF0000F) | (((int(filterCutoffU * DACMULT)) & 0xFFFF) << 4);
//     //   sample_data4 = (channel_d & 0xFFF0000F) | (((int(filterCutoffL * DACMULT)) & 0xFFFF) << 4);
//     //   outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
//     //   digitalWriteFast(DEMUX_EN_1, LOW);
//     //   break;

//     case 13:
//       sample_data1 = (channel_a & 0xFFF0000F) | (((int(amDepthU * MULT2V)) & 0xFFFF) << 4);
//       sample_data2 = (channel_c & 0xFFF0000F) | (((int(amDepthU * MULT2V)) & 0xFFFF) << 4);

//       sample_data3 = (channel_b & 0xFFF0000F) | (((int(effectPot1U * MULT33V)) & 0xFFFF) << 4);
//       sample_data4 = (channel_d & 0xFFF0000F) | (((int(effectPot1L * MULT33V)) & 0xFFFF) << 4);
//       outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
//       digitalWriteFast(DEMUX_EN_1, LOW);
//       break;

//     case 14:
//       sample_data1 = (channel_a & 0xFFF0000F) | ((0 & 0xFFFF) << 4);
//       sample_data2 = (channel_c & 0xFFF0000F) | ((0 & 0xFFFF) << 4);

//       sample_data3 = (channel_b & 0xFFF0000F) | (((int(effectPot2U * MULT33V)) & 0xFFFF) << 4);
//       sample_data4 = (channel_d & 0xFFF0000F) | (((int(effectPot2L * MULT33V)) & 0xFFFF) << 4);
//       outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
//       digitalWriteFast(DEMUX_EN_1, LOW);
//       break;

//     // case 15:
//     //   switch (LFODelayGoU) {
//     //     case 1:
//     //       sample_data1 = (channel_a & 0xFFF0000F) | (((int(amDepthU * DACMULT)) & 0xFFFF) << 4);
//     //       break;

//     //     case 0:
//     //       sample_data1 = (channel_a & 0xFFF0000F) | ((0 & 0xFFFF) << 4);
//     //       break;
//     //   }
//     //   switch (LFODelayGoL) {
//     //     case 1:
//     //       sample_data2 = (channel_c & 0xFFF0000F) | (((int(amDepthL * DACMULT)) & 0xFFFF) << 4);
//     //       break;

//     //     case 0:
//     //       sample_data2 = (channel_c & 0xFFF0000F) | ((0 & 0xFFFF) << 4);
//     //       break;
//     //   }

//     //   sample_data3 = (channel_b & 0xFFF0000F) | (((int(osc2TriangleLevelU * DACMULT)) & 0xFFFF) << 4);
//     //   sample_data4 = (channel_d & 0xFFF0000F) | (((int(osc2TriangleLevelL * DACMULT)) & 0xFFFF) << 4);
//     //   outputDAC(DAC_CS1, sample_data1, sample_data2, sample_data3, sample_data4);
//     //   digitalWriteFast(DEMUX_EN_1, LOW);
//     //   break;
//   }
//   delayMicroseconds(800);
//   digitalWriteFast(DEMUX_EN_1, HIGH);

//   muxOutput++;
//   if (muxOutput >= DEMUXCHANNELS)

//     muxOutput = 0;

//   digitalWriteFast(DEMUX_0, muxOutput & B0001);
//   digitalWriteFast(DEMUX_1, muxOutput & B0010);
//   digitalWriteFast(DEMUX_2, muxOutput & B0100);
//   digitalWriteFast(DEMUX_3, muxOutput & B1000);
// }

void checkEeprom() {

  // if (oldsplitTrans != splitTrans) {
  //   setTranspose(splitTrans);
  // }

  // if (oldfilterLogLinU != filterLogLinU) {
  //   updateFilterEnv(0);
  //   oldfilterLogLinU = filterLogLinU;
  // }

  // if (oldfilterLogLinL != filterLogLinL) {
  //   updateFilterEnv(0);
  //   oldfilterLogLinL = filterLogLinL;
  // }

  // if (oldampLogLinU != ampLogLinU) {
  //   updateAmpEnv(0);
  //   oldampLogLinU = ampLogLinU;
  // }

  // if (oldampLogLinL != ampLogLinL) {
  //   updateAmpEnv(0);
  //   oldampLogLinL = ampLogLinL;
  // }

  // if (oldkeyTrackSWU != keyTrackSWU) {
  //   updatekeyTrackSW(0);
  //   oldkeyTrackSWU = keyTrackSWU;
  // }

  // if (oldkeyTrackSWL != keyTrackSWL) {
  //   updatekeyTrackSW(0);
  //   oldkeyTrackSWL = keyTrackSWL;
  // }

  // if (oldmonoMultiU != monoMultiU) {
  //   updateMonoMulti(0);
  //   oldmonoMultiU = monoMultiU;
  // }

  // if (oldmonoMultiL != monoMultiL) {
  //   updateMonoMulti(0);
  //   oldmonoMultiL = monoMultiL;
  // }

  // if (oldAfterTouchDestU != AfterTouchDestU) {
  //   oldAfterTouchDestU = AfterTouchDestU;
  // }

  // if (oldAfterTouchDestL != AfterTouchDestL) {
  //   oldAfterTouchDestL = AfterTouchDestL;
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

void loop() {
  checkSwitches();
  checkEeprom();
  writeDemux();
  checkMux();
  checkEncoder();
  MIDI.read(midiChannel);
  MIDI6.read(midiChannel);
  MIDI5.read();
  usbMIDI.read(midiChannel);
  LFODelayHandle();
}