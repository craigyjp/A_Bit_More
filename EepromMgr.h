#include <EEPROM.h>

#define EEPROM_MIDI_CH 0
#define EEPROM_SPLITTRANS 1
#define EEPROM_ENCODER_DIR 2
#define EEPROM_MODWHEEL_DEPTH 3
#define EEPROM_FILTERENV_U 4
#define EEPROM_FILTERENV_L 5
#define EEPROM_AMPENV_U 6
#define EEPROM_AMPENV_L 7
#define EEPROM_LAST_PATCHU 8
#define EEPROM_LAST_PATCHL 9
#define EEPROM_AFTERTOUCH_U 10
#define EEPROM_AFTERTOUCH_L 11
#define EEPROM_SPLITPOINT 12
#define EEPROM_KEYTRACK_U 13
#define EEPROM_KEYTRACK_L 14
#define EEPROM_PITCHBEND 15
#define EEPROM_MONOMULTI_L 16
#define EEPROM_MONOMULTI_U 17

int getMIDIChannel() {
  byte midiChannel = EEPROM.read(EEPROM_MIDI_CH);
  if (midiChannel < 0 || midiChannel > 16) midiChannel = MIDI_CHANNEL_OMNI;  //If EEPROM has no MIDI channel stored
  return midiChannel;
}

void storeMidiChannel(byte channel) {
  EEPROM.update(EEPROM_MIDI_CH, channel);
}

float getSplitPoint() {
  byte sp = EEPROM.read(EEPROM_SPLITPOINT);
  if (sp < 0 || sp > 24) sp = 12;
  return sp;
}

void storeSplitPoint(byte type) {
  EEPROM.update(EEPROM_SPLITPOINT, type);
}

float getSplitTrans() {
  int st = EEPROM.read(EEPROM_SPLITTRANS);
  if (st < 0 || st > 4) st = 2;
  return st;  //If EEPROM has no key tracking stored
}

void storeSplitTrans(byte type) {
  EEPROM.update(EEPROM_SPLITTRANS, type);
}

float getAfterTouchU() {
  upperData[60] = EEPROM.read(EEPROM_AFTERTOUCH_U);
  if (upperData[60] < 0 || upperData[60] > 4) upperData[60] = 0;
  return upperData[60];  //If EEPROM has no key tracking stored
}

void storeAfterTouchU(byte AfterTouchDestL) {
  EEPROM.update(EEPROM_AFTERTOUCH_U, AfterTouchDestL);
}

float getAfterTouchL() {
  byte AfterTouchDestL = EEPROM.read(EEPROM_AFTERTOUCH_L);
  if (AfterTouchDestL < 0 || AfterTouchDestL > 4) AfterTouchDestL = 0;
  return AfterTouchDestL;  //If EEPROM has no key tracking stored
}

void storeAfterTouchL(byte AfterTouchDestL) {
  EEPROM.update(EEPROM_AFTERTOUCH_L, AfterTouchDestL);
}

boolean getEncoderDir() {
  byte ed = EEPROM.read(EEPROM_ENCODER_DIR);
  if (ed < 0 || ed > 1) return true;  //If EEPROM has no encoder direction stored
  return ed == 1 ? true : false;
}

void storeEncoderDir(byte encoderDir) {
  EEPROM.update(EEPROM_ENCODER_DIR, encoderDir);
}

int getLastPatchU() {
  int lastPatchNumberU = EEPROM.read(EEPROM_LAST_PATCHU);
  if (lastPatchNumberU < 1 || lastPatchNumberU > 999) lastPatchNumberU = 1;
  return lastPatchNumberU;
}

int getLastPatchL() {
  int lastPatchNumberL = EEPROM.read(EEPROM_LAST_PATCHL);
  if (lastPatchNumberL < 1 || lastPatchNumberL > 999) lastPatchNumberL = 1;
  return lastPatchNumberL;
}

void storeLastPatchU(int lastPatchNumber) {
  EEPROM.update(EEPROM_LAST_PATCHU, lastPatchNumber);
}

void storeLastPatchL(int lastPatchNumber) {
  EEPROM.update(EEPROM_LAST_PATCHL, lastPatchNumber);
}
