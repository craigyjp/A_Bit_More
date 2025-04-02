const int sysexDataLength = 154;
byte receivedSysExData[sysexDataLength];
bool isSysExProcessing = false;  // Flag to indicate SysEx processing state
unsigned long lastSysExByteTime = 0;
const unsigned long sysExTimeout = 100; // Timeout for SysEx processing (in milliseconds)

int i2cByteNumber = 0;

unsigned long timerStart = 0;
bool timerRunning = false;
const unsigned long timerDuration = 500; // 500 milliseconds

int maxSectionWidth = 0;

char filterDisplay[30];
char lfoDisplay[30];
char buf1[30];
char buf2[30];
char buf3[30];
char buf4[30];
char buf5[30];

int playMode = 0;
int readRes = 1023;
int parameterGroup = 0;

//Values below are just for initialising and will be changed when synth is initialised to current panel controls & EEPROM settings
int i = 0;

int resolutionFrig = 3;
boolean recallPatchFlag = false;
int setCursorPos = 0;

String patchName = INITPATCHNAME;

int NotePriority = 0;
int ClockSource = 0;
int chordHoldSW = 0;
int upperSW = 0;
int lowerSW = 1;

int returnvalue = 0;

int upperData[76];
int lowerData[76];
int panelData[76];

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

