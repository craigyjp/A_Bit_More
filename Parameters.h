//Values below are just for initialising and will be changed when synth is initialised to current panel controls & EEPROM settings
byte midiChannel = 1; //(EEPROM)
String patchNameU = INITPATCHNAME;
String patchNameL = INITPATCHNAME;
String patchName = INITPATCHNAME;
int upperpatchtag = 0;
int lowerpatchtag = 1;
byte splitPoint = 0;
byte oldsplitPoint = 0;
byte newsplitPoint = 0;
byte splitTrans = 0;
byte oldsplitTrans = 0;
int lowerTranspose = 0;

int noteMsg;
int noteVel;
int lastPlayedNote = -1;  // Track the last note played
int lastPlayedVoice = 0;  // Track the voice of the last note played
int lastUsedVoice = 0; // Global variable to store the last used voice

int upperData[76];
int lowerData[76];
int panelData[76];

const int originalDataLength = 76;
const int sysexDataLength = originalDataLength * 2;
byte sysexData[sysexDataLength];

#define P_sysex 0
#define P_pwLFO 1
#define P_fmDepth 2
#define P_osc2PW 3
#define P_osc2PWM 4
#define P_osc1PW 5
#define P_osc1PWM 6
#define P_osc1Range 7 
#define P_osc2Range 8 
#define P_osc2Interval 9
#define P_glideTime 10 
#define P_osc2Detune 11
#define P_noiseLevel 12
#define P_osc2SawLevel 13
#define P_osc1SawLevel 14
#define P_osc2PulseLevel 15
#define P_osc1PulseLevel 16
#define P_filterCutoff 17
#define P_filterLFO 18
#define P_filterRes 19
#define P_filterType 20
#define P_modWheelDepth 21
#define P_effectsMix 22
#define P_LFODelayGo 23
#define P_filterEGlevel 24
#define P_LFORate 25
#define P_LFOWaveform 26
#define P_filterAttack 27
#define P_filterDecay 28
#define P_filterSustain 29
#define P_filterRelease 30
#define P_ampAttack 31
#define P_ampDecay 32
#define P_ampSustain 33
#define P_ampRelease 34
#define P_volumeControl 35
#define P_glideSW 36
#define P_keytrack 37
#define P_filterPoleSW 38
#define P_filterLoop 39
#define P_filterEGinv 40
#define P_filterVel 41
#define P_vcaLoop 42
#define P_vcaVel 43
#define P_vcaGate 44
#define P_lfoAlt 45
#define P_pmDCO2 46
#define P_pmFilterEnv 47
#define P_monoMulti 48
#define P_modWheelLevel 49
#define P_PitchBendLevel 50
#define P_amDepth 51
#define P_sync 52
#define P_effectPot1 53
#define P_effectPot2 54
#define P_effectPot3 55
#define P_oldampAttack 56
#define P_oldampDecay 57
#define P_oldampSustain 58
#define P_oldampRelease 59
#define P_AfterTouchDest 60
#define P_filterLogLin 61
#define P_ampLogLin 62
#define P_osc2TriangleLevel 63
#define P_osc1SubLevel 64
#define P_keyboardMode 65
#define P_LFODelay 66
#define P_effectNum 67
#define P_effectBank 68
#define P_pmDestDCO1 69
#define P_pmDestFilter 70
#define P_lfoMultiplier 71
#define P_NotePriority 72
#define P_keytrackSW 73

int playMode = 0;

//Delayed LFO
int numberOfNotes = 0;
int oldnumberOfNotes = 0;
int numberOfNotesU = 0;
int oldnumberOfNotesU = 0;
int numberOfNotesL = 0;
int oldnumberOfNotesL = 0;
unsigned long previousMillisL = 0;
unsigned long intervalL = 1; //10 seconds
long delaytimeL  = 0;
unsigned long previousMillisU = 0;
unsigned long intervalU = 1; //10 seconds
long delaytimeU  = 0;

boolean encCW = true;//This is to set the encoder to increment when turned CW - Settings Option
boolean announce = true;
// polykit parameters in order of mux

String StratusLFOWaveform = "                ";

int oldfilterCutoff = 0;
int oldfilterCutoffU = 0;
int oldfilterCutoffL = 0;



int upperSW = 0;
int oldupperSW = 0;
int lowerSW = 0;
int oldlowerSW = 0;
int chordHoldSW = 0;
int chordHoldU = 0;
int chordHoldL = 0;

float afterTouch = 0;
int AfterTouchDest = 0;
int AfterTouchDestU = 0;
int AfterTouchDestL = 0;
int oldAfterTouchDestU = 0;
int oldAfterTouchDestL = 0;
float pwLFOstr = 0;
float fmDepthstr = 0;
float osc2PWstr = 0;
float osc2PWMstr = 0;
float osc1PWstr = 0;
float osc1PWMstr = 0;
float glideTimestr = 0;
float osc2Detunestr = 0;
int osc2Intervalstr = 0;
float noiseLevelstr = 0;
float osc2SawLevelstr = 0;
float osc1SawLevelstr = 0;
float osc2PulseLevelstr = 0;
float osc1PulseLevelstr = 0;
float osc2TriangleLevelstr = 0;
float osc1SubLevelstr = 0;
float filterCutoffstr = 0;
float filterLFOstr = 0;
float filterResstr = 0;
float filterEGlevelstr = 0;
float LFORatestr = 0;
float LFODelaystr = 0;
int LFOWaveformstr = 0;
float filterAttackstr = 0;
float filterDecaystr = 0;
float filterSustainstr = 0;
float filterReleasestr = 0;
float amDepthstr = 0; 
float volumeControlstr = 0;
float ampReleasestr = 0;
float ampSustainstr = 0;
float ampDecaystr = 0;
float ampAttackstr = 0;
float effectPot1str = 0;
float effectPot2str = 0;
float effectPot3str = 0;
float effectsMixstr = 0;
float pmDCO2str = 0;
float pmFilterEnvstr = 0;
float keytrackstr = 0;
float modWheelDepthstr = 0;
int modWheelLevelstr = 0;
int PitchBendLevelstr = 0; // for display

int wholemode = 1;
int dualmode = 0;
int splitmode = 0;
int LFOWaveCV = 0;

int returnvalue = 0;
