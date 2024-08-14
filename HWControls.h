// This optional setting causes Encoder to use more optimized code,
// It must be defined before Encoder.h is included.
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <Bounce.h>
#include "TButton.h"
#include <ADC.h>
#include <ADC_util.h>

ADC *adc = new ADC();

//Teensy 3.6 - Mux Pins
#define MUX_0 37
#define MUX_1 30
#define MUX_2 31
#define MUX_3 32

#define MUX1_S A0
#define MUX2_S A1
#define MUX3_S A2

#define DEMUX_0 36
#define DEMUX_1 35
#define DEMUX_2 34
#define DEMUX_3 33

#define DEMUX_EN_1 2


//Note DAC
#define MULT2V 25.9
#define MULT5V 32
#define MULT33V 21.3
#define DACMULT 25.9
#define MIDICCTOPOT 8.62

#define DAC_CS1 10

//Mux 1 Connections

#define MUX1_glideTime 0
#define MUX1_osc1SawLevel 1
#define MUX1_osc1PulseLevel 2
#define MUX1_osc1PW 3
#define MUX1_osc1PWM 4
#define MUX1_osc2Detune 5
#define MUX1_osc2interval 6
#define MUX1_fmDepth 7
#define MUX1_osc1SubLevel 8
#define MUX1_osc2SawLevel 9
#define MUX1_osc2PulseLevel 10
#define MUX1_osc2TriangleLevel 11
#define MUX1_osc2PW 12
#define MUX1_osc2PWM 13
#define MUX1_spare14 14
#define MUX1_spare15 15

//Mux 2 Connections

#define MUX2_filterAttack 0
#define MUX2_filterDecay 1
#define MUX2_filterSustain 2
#define MUX2_filterRelease 3
#define MUX2_ampAttack 4
#define MUX2_ampDecay 5	
#define MUX2_ampSustain 6
#define MUX2_ampRelease 7	
#define MUX2_filterLFO 8
#define MUX2_keyTrack 9
#define MUX2_filterCutoff 10
#define MUX2_filterRes 11
#define MUX2_filterEGlevel 12
#define MUX2_spare13 13
#define MUX2_spare14 14
#define MUX2_spare15 15

//Mux 3 Connections

#define MUX3_spare0 0
#define MUX3_effectMix 1
#define MUX3_volumeControl 2
#define MUX3_amplifierLFO 3
#define MUX3_spare4 4
#define MUX3_spare5 5
#define MUX3_noiseLevel 6 
#define MUX3_pwLFO 7
#define MUX3_LFORate 8
#define MUX3_LFODelay 9
#define MUX3_modWheelDepth 10
#define MUX3_effectPot1 11
#define MUX3_effectPot2 12
#define MUX3_effectPot3 13
#define MUX3_PM_DCO2 14
#define MUX3_PM_FilterEnv 15


// New DeMux 1 Connections A 

#define DEMUX1_noiseLevel_Upper 0           // 0-2v
#define DEMUX1_osc1SawLevel_Upper 1         // 0-2v
#define DEMUX1_osc1PulseLevel_Upper 2       // 0-2v
#define DEMUX1_osc1SubLevel_Upper 3         // 0-2v
#define DEMUX1_osc1PM_DCO1_level_Upper 4    // 0-2v
#define DEMUX1_osc1PM_Env_level_Upper 5     // 0-2v
#define DEMUX1_osc2SawLevel_Upper 6         // 0-2v
#define DEMUX1_osc2PulseLevel_Upper 7       // 0-2v
#define DEMUX1_osc2TriLevel_Upper 8         // 0-2v
#define DEMUX1_volumeControl_Upper 9        // 0-2v
#define DEMUX1_effectMix_Upper 10           // 0-2v
#define DEMUX1_FM_LFO_Depth_Upper 11        // 0-2v
#define DEMUX1_TM_LFO_Depth_Upper 12        // 0-2v
#define DEMUX1_AM_LFO_Depth_Upper 13        // 0-2v
#define DEMUX1_spare 14                     // 0-2v
#define DEMUX1_PW_LFO_Rate_Upper 15         // 0-5v

//DeMux 2 Connections B
#define DEMUX2_filterAttack_Upper 0         // 0-5v
#define DEMUX2_filterDecay_Upper 1          // 0-5v
#define DEMUX2_filterSustain_Upper 2        // 0-5v
#define DEMUX2_filterRelease_Upper 3        // 0-5v
#define DEMUX2_ampAttack_Upper 4            // 0-5v
#define DEMUX2_ampDecay_Upper 5             // 0-5v
#define DEMUX2_amp_Sustain_Upper 6          // 0-5v
#define DEMUX2_ampRelease_Upper 7           // 0-5v
#define DEMUX2_egDepth_upper 8              // 0-5v
#define DEMUX2_filterCutoff_Upper 9         // 0-5v
#define DEMUX2_filterRes_Upper 10           // 0-5v  
#define DEMUX2_LFO_Rate_Upper 11            // 0-5v
#define DEMUX2_LFO_Wave_Upper 12            // 0-5v 
#define DEMUX2_effectPot1_Upper 13          // 0-3.3v 
#define DEMUX2_effectPot2_Upper 14          // 0-3.3v
#define DEMUX2_effectPot3_Upper 15          // 0-3.3v

//DeMux 3 Connections C
#define DEMUX3_noiseLevel_Lower 0           // 0-2v
#define DEMUX3_osc1SawLevel_Lower 1         // 0-2v
#define DEMUX3_osc1PulseLevel_Lower 2       // 0-2v
#define DEMUX3_osc1SubLevel_Lower 3         // 0-2v
#define DEMUX3_osc1PM_DCO1_level_Lower 4    // 0-2v
#define DEMUX3_osc1PM_Env_level_Lower 5     // 0-2v
#define DEMUX3_osc2SawLevel_Lower 6         // 0-2v
#define DEMUX3_osc2PulseLevel_Lower 7       // 0-2v
#define DEMUX3_osc2TriLevel_Lower 8         // 0-2v
#define DEMUX3_volumeControl_Lower 9        // 0-2v
#define DEMUX3_effectMix_Lower 10           // 0-2v
#define DEMUX3_FM_LFO_Depth_Lower 11        // 0-2v
#define DEMUX3_TM_LFO_Depth_Lower 12        // 0-2v
#define DEMUX3_AM_LFO_Depth_Lower 13        // 0-2v
#define DEMUX3_spare 14                     // 0-2v
#define DEMUX3_PW_LFO_Rate_Lower 15         // 0-5v

//DeMux 4 Connections D
#define DEMUX4_filterAttack_Lower 0         // 0-5v
#define DEMUX4_filterDecay_Lower 1          // 0-5v
#define DEMUX4_filterSustain_Lower 2        // 0-5v
#define DEMUX4_filterRelease_Lower 3        // 0-5v
#define DEMUX4_ampAttack_Lower 4            // 0-5v 
#define DEMUX4_ampDecay_Lower 5             // 0-5v
#define DEMUX4_amp_Sustain_Lower 6          // 0-5v
#define DEMUX4_ampRelease_Lower 7           // 0-5v
#define DEMUX4_egDepth_Lower 8              // 0-5v
#define DEMUX4_filterCutoff_Lower 9         // 0-5v
#define DEMUX4_filterRes_Lower 10           // 0-5v 
#define DEMUX4_LFO_Rate_Lower 11            // 0-5v
#define DEMUX4_LFO_Wave_Lower 12            // 0-5v
#define DEMUX4_effectPot1_Upper 13          // 0-3.3v 
#define DEMUX4_effectPot2_Lower 14          // 0-3.3v 
#define DEMUX4_effectPot3_Lower 15          // 0-3.3v 

// 74HC165 Switches

#define POLY1_SW 0
#define POLY2_SW 1
#define UNISON_SW 2
#define MONO_SW 3
#define LOWER_SW 4
#define UPPER_SW 5
#define CHORD_HOLD_SW 6
#define KEYBOARD_SW 7

#define GLIDE_SW 8
#define PRIORITY_SW 9
#define DCO1_OCT_SW 10
#define DCO2_OCT_SW 11
#define SPARE_SYNC_SW 12
#define FILTER_TYPE_SW 13
#define FILTER_POLE_SW 14
#define EG_INVERT_SW 15

#define FILTER_ENV_VELOCITY_SW 16
#define FILTER_ENV_LIN_LOG_SW 17
#define FILTER_ENV_LOOP_SW 18
#define AMP_ENV_VELOCITY_SW 19
#define AMP_ENV_LIN_LOG_SW 20
#define AMP_ENV_LOOP_SW  21
#define LFO_WAVEFORM_SW 22
#define SYNC_SW 23

#define EFFECT_NUMBER_SW 24
#define PM_DCO1_DEST_SW 25
#define PM_FILT_ENV_DEST_SW 26
#define AMP_GATED_SW 27
#define EFFECT_BANK_SW 28
#define LFO_ALT_SW 29
#define LFO_MULTI_MONO_SW 30
#define LFO_MULT_SW 31

// New 595 outputs X8

#define SYNC_UPPER 0
#define SPARE1 1
#define SPARE2 2
#define SPARE3 3
#define FILTER_EG_INV_UPPER 4
#define FILTER_VELOCITY_UPPER 5
#define AMP_VELOCITY_UPPER 6
#define LFO_ALT_UPPER 7

#define POLYMOD_DEST_DCO1_UPPER 8 
#define POLYMOD_DEST_FILTER_UPPER 9
#define EFFECT_BANK_1_UPPER 10
#define EFFECT_BANK_2_UPPER 11
#define EFFECT_BANK_3_UPPER 12
#define SPARE13 13
#define FILTER_LIN_LOG_UPPER 14
#define AMP_LIN_LOG_UPPER 15

#define EFFECT_O_UPPER 16
#define EFFECT_1_UPPER 17
#define EFFECT_2_UPPER 18
#define EFFECT_INTERNAL_UPPER 19
#define FILTER_POLE_UPPER 20
#define FILTERA_UPPER 21
#define FILTERB_UPPER 22
#define FILTERC_UPPER 23

#define SYNC_LOWER 24
#define SPARE25 25
#define SPARE26 26
#define SPARE27 27
#define FILTER_EG_INV_LOWER 28
#define FILTER_VELOCITY_LOWER 29
#define AMP_VELOCITY_LOWER 30
#define LFO_ALT_LOWER 31

#define POLYMOD_DEST_DCO1_LOWER 32 
#define POLYMOD_DEST_FILTER_LOWER 33
#define EFFECT_BANK_1_LOWER 34
#define EFFECT_BANK_2_LOWER 35
#define EFFECT_BANK_3_LOWER 36
#define SPARE37 37
#define FILTER_LIN_LOG_LOWER 38
#define AMP_LIN_LOG_LOWER 39

#define EFFECT_0_LOWER 40
#define EFFECT_1_LOWER 41
#define EFFECT_2_LOWER 42
#define EFFECT_INTERNAL_LOWER 43
#define FILTER_POLE_LOWER 44
#define FILTERA_LOWER 45
#define FILTERB_LOWER 48
#define FILTERC_LOWER 47

#define FILTER_MODE_BIT0_UPPER 48
#define FILTER_MODE_BIT1_UPPER 49
#define FILTER_MODE_BIT0_LOWER 50
#define FILTER_MODE_BIT1_LOWER 51
#define AMP_MODE_BIT0_UPPER 52
#define AMP_MODE_BIT1_UPPER 53
#define AMP_MODE_BIT0_LOWER 54
#define AMP_MODE_BIT1_LOWER 55

#define UPPER_RELAY_1 56
#define UPPER_RELAY_2 57
#define LFO_MULTI_BIT0_UPPER 58
#define LFO_MULTI_BIT1_UPPER 59
#define LFO_MULTI_BIT2_UPPER 60
#define LFO_MULTI_BIT0_LOWER 61
#define LFO_MULTI_BIT1_LOWER 62
#define LFO_MULTI_BIT2_LOWER 63

// System Switches etc

#define RECALL_SW 20
#define SAVE_SW 23
#define SETTINGS_SW 22
#define BACK_SW 21

#define ENCODER_PINA 5
#define ENCODER_PINB 4

#define MUXCHANNELS 16
#define DEMUXCHANNELS 16
#define QUANTISE_FACTOR 12

#define DEBOUNCE 30

static byte muxInput = 0;
static byte muxOutput = 0;

static int mux1ValuesPrev[MUXCHANNELS] = {};
static int mux2ValuesPrev[MUXCHANNELS] = {};
static int mux3ValuesPrev[MUXCHANNELS] = {};

static int mux1Read = 0;
static int mux2Read = 0;
static int mux3Read = 0;


static long encPrevious = 0;

TButton saveButton{ SAVE_SW, LOW, HOLD_DURATION, DEBOUNCE, CLICK_DURATION };
TButton settingsButton{ SETTINGS_SW, LOW, HOLD_DURATION, DEBOUNCE, CLICK_DURATION };
TButton backButton{ BACK_SW, LOW, HOLD_DURATION, DEBOUNCE, CLICK_DURATION };
TButton recallButton{ RECALL_SW, LOW, HOLD_DURATION, DEBOUNCE, CLICK_DURATION };  //On encoder

Encoder encoder(ENCODER_PINB, ENCODER_PINA);//This often needs the pins swapping depending on the encoder

void setupHardware()
{
     //Volume Pot is on ADC0
  //Volume Pot is on ADC0
  adc->adc0->setAveraging(32); // set number of averages 0, 4, 8, 16 or 32.
  adc->adc0->setResolution(10); // set bits of resolution  8, 10, 12 or 16 bits.
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED); // change the conversion speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED); // change the sampling speed

  //MUXs on ADC1
  adc->adc1->setAveraging(32); // set number of averages 0, 4, 8, 16 or 32.
  adc->adc1->setResolution(10); // set bits of resolution  8, 10, 12 or 16 bits.
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED); // change the conversion speed
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED); // change the sampling speed

  analogReadResolution(10);


  //Mux address pins

  pinMode(DAC_CS1, OUTPUT);
  digitalWrite(DAC_CS1, HIGH);

  pinMode(MUX_0, OUTPUT);
  pinMode(MUX_1, OUTPUT);
  pinMode(MUX_2, OUTPUT);
  pinMode(MUX_3, OUTPUT);

  pinMode(DEMUX_0, OUTPUT);
  pinMode(DEMUX_1, OUTPUT);
  pinMode(DEMUX_2, OUTPUT);
  pinMode(DEMUX_3, OUTPUT);

  digitalWrite(MUX_0, LOW);
  digitalWrite(MUX_1, LOW);
  digitalWrite(MUX_2, LOW);
  digitalWrite(MUX_3, LOW);

  digitalWrite(DEMUX_0, LOW);
  digitalWrite(DEMUX_1, LOW);
  digitalWrite(DEMUX_2, LOW);
  digitalWrite(DEMUX_3, LOW);

  pinMode(DEMUX_EN_1, OUTPUT);

  digitalWrite(DEMUX_EN_1, HIGH);


  //Switches

  pinMode(RECALL_SW, INPUT_PULLUP); //On encoder
  pinMode(SAVE_SW, INPUT_PULLUP);
  pinMode(SETTINGS_SW, INPUT_PULLUP);
  pinMode(BACK_SW, INPUT_PULLUP);

  pinMode(MUX1_S, INPUT_DISABLE);
  pinMode(MUX2_S, INPUT_DISABLE);
  pinMode(MUX3_S, INPUT_DISABLE);
  
}
