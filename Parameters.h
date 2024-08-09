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
float keytrackingAmount = 0.5;

int upperData[70];
int lowerData[70];
int panelData[70];

const int originalDataLength = 70;
const int sysexDataLength = originalDataLength;
byte sysexData[sysexDataLength];

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
#define P_keyTrackSW 65
#define P_LFODelay 66

boolean syncSW = false;
boolean filterenvLinLogSW = false;
boolean ampenvLinLogSW = false;
int lfoMult = 0;
int effectBankSW = 0;
int effectNumSW = 0;
boolean pmDestDCO1SW = false;
boolean pmDestFilterSW = false;
boolean monoMultiSW = false;
int pwLFOwaveformSW = 0;

int keyboardMode = 0;
int playMode = 0;
int NotePriority = 0;

//Delayed LFO
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

int pwLFO = 0;
float pwLFOstr = 0; // for display
int fmDepth = 0;

float fmDepthstr = 0;
int osc2PW = 0;
float osc2PWstr = 0;
int osc2PWM = 0;
float osc2PWMstr = 0;
int osc1PW = 0;
float osc1PWstr = 0;
int osc1PWM = 0;
float osc1PWMstr = 0;
int osc1Range = 0;
float osc1Rangestr = 0;
int osc2Range = 0;
float osc2Rangestr = 0;
int glideTime = 0;
float glideTimestr = 0;
int osc2Detune = 0;
float osc2Detunestr = 0;
int osc2Interval = 0;
float osc2Intervalstr = 0;
int modWheelDepth = 0;
float modWheelDepthstr = 0;
int noiseLevel = 0;
float noiseLevelstr = 0;
float osc2SawLevelstr = 0;
int osc2SawLevel = 0;
int osc1SawLevel = 0;
float osc1SawLevelstr = 0;
int osc2PulseLevel = 0;
float osc2PulseLevelstr = 0;
int osc1PulseLevel = 0;
float osc1PulseLevelstr = 0;
int osc2TriangleLevel = 0;
float osc2TriangleLevelstr = 0; // for display
int osc1SubLevel = 0;
float osc1SubLevelstr = 0; // for display

int filterCutoff = 0;
int oldfilterCutoff = 0;
int oldfilterCutoffU = 0;
int oldfilterCutoffL = 0;
float filterCutoffstr = 0; // for display
int filterLFO = 0;
float filterLFOstr = 0; // for display
int filterRes = 0;
float filterResstr = 0;
int filterType = 0;
int filterEGlevel = 0;
float filterEGlevelstr = 0;
int LFORate = 0;
float LFORatestr = 0; //for display
int LFODelay = 0;
float LFODelaystr = 0; //for display
String StratusLFOWaveform = "                ";
int LFOWaveformstr = 0;
int LFOWaveform = 0;
int LFOWaveCV = 0;
int filterAttack = 0;
float filterAttackstr = 0;
int filterDecay = 0;
float filterDecaystr = 0;
int filterSustain = 0;
float filterSustainstr = 0;
int filterRelease = 0;
float filterReleasestr = 0;
int ampAttack = 0;
int oldampAttack = 0;
float ampAttackstr = 0;
int ampDecay = 0;
int oldampDecay = 0;
int oldampDecayU = 0;
int oldampDecayL = 0;
float ampDecaystr = 0;
int ampSustain = 0;
int oldampSustain = 0;
int oldampSustainU = 0;
int oldampSustainL = 0;
float ampSustainstr = 0;
int ampRelease = 0;
int oldampRelease = 0;
int oldampReleaseU = 0;
int oldampReleaseL  = 0;
float ampReleasestr = 0;
int volumeControl = 0;
float volumeControlstr = 0; // for display
int amDepth = 0;;
float amDepthstr = 0; // for display
int keytrack = 0;
float keytrackstr = 0;
int keyTrackSW = 0;

int effectPot1 = 0;
float effectPot1str = 0;
int effectPot2 = 0;
float effectPot2str = 0;
int effectPot3 = 0;
float effectPot3str = 0;
int effectsMix= 0;
float effectsMixstr = 0;

int pmDCO2 = 0;
float pmDCO2str = 0;
int pmFilterEnv = 0;
float pmFilterEnvstr = 0;

int pitchBendRange = 0;
int PitchBendLevel = 0;
int PitchBendLevelstr = 0; // for display
int modWheelLevel = 0;
float modWheelLevelstr = 0;

int glideSW = 0;
int vcaLoop = 0;
int statevcaLoop = 0;
int oldvcaLoop = 0;
int statevcaLoopU = 0;
int oldvcaLoopU = 0;
int statevcaLoopL = 0;
int oldvcaLoopL = 0;
int vcadoubleLoop = 0;
int vcaVel = 0;
int vcaGate = 0;
int chorus1 = 0;
int chorus2 = 0;
int lfoAlt = 0;
int monoMulti = 0;

int oldmonoMultiU = 0;
int oldmonoMultiL = 0;
int filterPoleSW = 0;
int filterVel = 0;
int filterLoop = 0;
int statefilterLoop = 0;
int oldfilterLoop = 0;
int statefilterLoopU = 0;
int oldfilterLoopU = 0;
int statefilterLoopL = 0;
int oldfilterLoopL = 0;
int filterdoubleLoop = 0;
int filterEGinv = 0;
int upperSW = 0;
int oldupperSW = 0;
int lowerSW = 0;
int oldlowerSW = 0;
int filterLogLin = 0;
int ampLogLin = 0;
int sync = 0;
int chordHoldSW = 0;
int chordHoldU = 0;
int chordHoldL = 0;

float afterTouch = 0;
int AfterTouchDest = 0;
int AfterTouchDestU = 0;
int AfterTouchDestL = 0;
int oldAfterTouchDestU = 0;
int oldAfterTouchDestL = 0;
int oldfilterLogLinU;
int oldfilterLogLinL;
int oldampLogLinU;
int oldampLogLinL;
int oldkeyTrackSWU;
int oldkeyTrackSWL;

int wholemode = 1;
int dualmode = 0;
int splitmode = 0;


int returnvalue = 0;
