#include "SettingsService.h"

void settingsMIDICh();
void settingsSplitPoint();
void settingsSplitTrans();
void settingsAfterTouchU();
void settingsAfterTouchL();
void settingsPitchBend();
void settingsEncoderDir();

int currentIndexMIDICh();
int currentIndexSplitPoint();
int currentIndexSplitTrans();
int currentIndexAfterTouchU();
int currentIndexAfterTouchL();
int currentIndexPitchBend();
int currentIndexEncoderDir();

void settingsSplitPoint(int index, const char *value) {
  if (strcmp(value, "36") == 0) newsplitPoint = 0;
  if (strcmp(value, "37") == 0) newsplitPoint = 1;
  if (strcmp(value, "38") == 0) newsplitPoint = 2;
  if (strcmp(value, "39") == 0) newsplitPoint = 3;
  if (strcmp(value, "40") == 0) newsplitPoint = 4;
  if (strcmp(value, "41") == 0) newsplitPoint = 5;
  if (strcmp(value, "42") == 0) newsplitPoint = 6;
  if (strcmp(value, "43") == 0) newsplitPoint = 7;
  if (strcmp(value, "44") == 0) newsplitPoint = 8;
  if (strcmp(value, "45") == 0) newsplitPoint = 9;
  if (strcmp(value, "46") == 0) newsplitPoint = 10;
  if (strcmp(value, "47") == 0) newsplitPoint = 11;
  if (strcmp(value, "48") == 0) newsplitPoint = 12;
  if (strcmp(value, "49") == 0) newsplitPoint = 13;
  if (strcmp(value, "50") == 0) newsplitPoint = 14;
  if (strcmp(value, "51") == 0) newsplitPoint = 15;
  if (strcmp(value, "52") == 0) newsplitPoint = 16;
  if (strcmp(value, "53") == 0) newsplitPoint = 17;
  if (strcmp(value, "54") == 0) newsplitPoint = 18;
  if (strcmp(value, "55") == 0) newsplitPoint = 19;
  if (strcmp(value, "56") == 0) newsplitPoint = 20;
  if (strcmp(value, "57") == 0) newsplitPoint = 21;
  if (strcmp(value, "58") == 0) newsplitPoint = 22;
  if (strcmp(value, "59") == 0) newsplitPoint = 23;
  if (strcmp(value, "60") == 0) newsplitPoint = 24;
  storeSplitPoint(newsplitPoint);
}

void settingsSplitTrans(int index, const char *value) {
  if (strcmp(value, "-2 Octave") == 0) splitTrans = 0;
  if (strcmp(value, "-1 Octave") == 0) splitTrans = 1;
  if (strcmp(value, "Original") == 0) splitTrans = 2;
  if (strcmp(value, "+1 Octave") == 0) splitTrans = 3;
  if (strcmp(value, "+2 Octave") == 0) splitTrans = 4;
  storeSplitTrans(splitTrans);
}

void settingsMIDICh(int index, const char *value) {
  if (strcmp(value, "ALL") == 0) {
    midiChannel = MIDI_CHANNEL_OMNI;
  } else {
    midiChannel = atoi(value);
  }
  storeMidiChannel(midiChannel);
}

void settingsAfterTouchU(int index, const char *value) {
  if (strcmp(value, "Off") == 0) upperData[60] = 0;
  if (strcmp(value, "DCO Mod") == 0) upperData[60] = 1;
  if (strcmp(value, "CutOff Freq") == 0) upperData[60] = 2;
  if (strcmp(value, "VCF Mod") == 0) upperData[60] = 3;
  if (strcmp(value, "VCA Mod") == 0) upperData[60] = 4;
  storeAfterTouchU(AfterTouchDestU);
}

void settingsAfterTouchL(int index, const char *value) {
  if (strcmp(value, "Off") == 0) AfterTouchDestL = 0;
  if (strcmp(value, "DCO Mod") == 0) AfterTouchDestL = 1;
  if (strcmp(value, "CutOff Freq") == 0) AfterTouchDestL = 2;
  if (strcmp(value, "VCF Mod") == 0) AfterTouchDestL = 3;
  if (strcmp(value, "VCA Mod") == 0) AfterTouchDestL = 4;
  storeAfterTouchL(AfterTouchDestL);
}

void settingsPitchBend(int index, const char *value) {
  pitchBendRange = atoi(value);
  storePitchBendRange(pitchBendRange);
}

void settingsEncoderDir(int index, const char *value) {
  if (strcmp(value, "Type 1") == 0) {
    encCW = true;
  } else {
    encCW = false;
  }
  storeEncoderDir(encCW ? 1 : 0);
}

int currentIndexSplitTrans() {
  return getSplitTrans();
}

int currentIndexMIDICh() {
  return getMIDIChannel();
}

int currentIndexSplitPoint() {
  return getSplitPoint();
}

int currentIndexAfterTouchU() {
  return getAfterTouchU();
}

int currentIndexAfterTouchL() {
  return getAfterTouchL();
}

int currentIndexPitchBend() {
  return getPitchBendRange() - 1;
}

int currentIndexEncoderDir() {
  return getEncoderDir() ? 0 : 1;
}


// add settings to the circular buffer
void setUpSettings() {
  settings::append(settings::SettingsOption{ "MIDI Ch.", { "All", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "16", "\0" }, settingsMIDICh, currentIndexMIDICh });
  settings::append(settings::SettingsOption{ "Split Point", { "36", "37", "38", "39", "40", "41", "42", "43", "44", "45", "46", "47", "48", "49", "50", "51", "52", "53", "54", "55", "56", "57", "58", "59", "60", "\0" }, settingsSplitPoint, currentIndexSplitPoint });
  settings::append(settings::SettingsOption{ "Split Trans", { "-2 Octave", "-1 Octave", "Original", "+1 Octave", "+2 Octave", "\0" }, settingsSplitTrans, currentIndexSplitTrans });
  settings::append(settings::SettingsOption{ "Pitch Bend", { "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "\0" }, settingsPitchBend, currentIndexPitchBend });
  settings::append(settings::SettingsOption{ "AfterTouch U", { "Off", "DCO Mod", "CutOff Freq", "VCF Mod", "VCA Mod", "\0" }, settingsAfterTouchU, currentIndexAfterTouchU });
  settings::append(settings::SettingsOption{ "AfterTouch L", { "Off", "DCO Mod", "CutOff Freq", "VCF Mod", "VCA Mod", "\0" }, settingsAfterTouchL, currentIndexAfterTouchL });
  settings::append(settings::SettingsOption{ "Encoder", { "Type 1", "Type 2", "\0" }, settingsEncoderDir, currentIndexEncoderDir });
}