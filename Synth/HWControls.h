// This optional setting causes Encoder to use more optimized code,
// It must be defined before Encoder.h is included.
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <Bounce.h>
#include "TButton.h"

#include "Rotary.h"
#include "RotaryEncOverMCP.h"

#define OSC1_PW_BUTTON 0
#define OSC2_PW_BUTTON 1
#define FM_DEPTH_BUTTON 2
#define OSC2_DETUNE_BUTTON 3
#define OSC1_SAW_BUTTON 4
#define OSC1_PULSE_BUTTON 5
#define OSC1_SUB_BUTTON 6
#define OSC2_SAW_BUTTON 7
#define OSC2_PULSE_BUTTON 8
#define OSC2_TRI_BUTTON 9
#define EFFECTS_MIX_BUTTON 10
#define NOISE_BUTTON 11


// Pins for MCP23017
#define GPA0 0
#define GPA1 1
#define GPA2 2
#define GPA3 3
#define GPA4 4
#define GPA5 5
#define GPA6 6
#define GPA7 7
#define GPB0 8
#define GPB1 9
#define GPB2 10
#define GPB3 11
#define GPB4 12
#define GPB5 13
#define GPB6 14
#define GPB7 15

void RotaryEncoderChanged(bool clockwise, int id);

void mainButtonChanged(Button *btn, bool released);

Adafruit_MCP23017 mcp1;
Adafruit_MCP23017 mcp2;
Adafruit_MCP23017 mcp3;
Adafruit_MCP23017 mcp4;
Adafruit_MCP23017 mcp5;
Adafruit_MCP23017 mcp6;
Adafruit_MCP23017 mcp7;

//Array of pointers of all MCPs
Adafruit_MCP23017 *allMCPs[] = { &mcp1, &mcp2, &mcp3, &mcp4, &mcp5, &mcp6, &mcp7 };

/* Array of all rotary encoders and their pins */
RotaryEncOverMCP rotaryEncoders[] = {
  RotaryEncOverMCP(&mcp1, 0, 1, &RotaryEncoderChanged, 1),
  RotaryEncOverMCP(&mcp1, 2, 3, &RotaryEncoderChanged, 2),
  RotaryEncOverMCP(&mcp1, 4, 5, &RotaryEncoderChanged, 3),
  RotaryEncOverMCP(&mcp1, 8, 9, &RotaryEncoderChanged, 4),
  RotaryEncOverMCP(&mcp1, 10, 11, &RotaryEncoderChanged, 5),
  RotaryEncOverMCP(&mcp1, 12, 13, &RotaryEncoderChanged, 6),
  RotaryEncOverMCP(&mcp2, 0, 1, &RotaryEncoderChanged, 7),
  RotaryEncOverMCP(&mcp2, 2, 3, &RotaryEncoderChanged, 8),
  RotaryEncOverMCP(&mcp2, 4, 5, &RotaryEncoderChanged, 9),
  RotaryEncOverMCP(&mcp2, 8, 9, &RotaryEncoderChanged, 10),
  RotaryEncOverMCP(&mcp2, 10, 11, &RotaryEncoderChanged, 11),
  RotaryEncOverMCP(&mcp2, 12, 13, &RotaryEncoderChanged, 12),
  RotaryEncOverMCP(&mcp3, 0, 1, &RotaryEncoderChanged, 13),
  RotaryEncOverMCP(&mcp3, 2, 3, &RotaryEncoderChanged, 14),
  RotaryEncOverMCP(&mcp3, 4, 5, &RotaryEncoderChanged, 15),
  RotaryEncOverMCP(&mcp3, 8, 9, &RotaryEncoderChanged, 16),
  RotaryEncOverMCP(&mcp3, 10, 11, &RotaryEncoderChanged, 17),
  RotaryEncOverMCP(&mcp3, 12, 13, &RotaryEncoderChanged, 18),
  RotaryEncOverMCP(&mcp4, 0, 1, &RotaryEncoderChanged, 19),
  RotaryEncOverMCP(&mcp4, 2, 3, &RotaryEncoderChanged, 20),
  RotaryEncOverMCP(&mcp4, 4, 5, &RotaryEncoderChanged, 21),
  RotaryEncOverMCP(&mcp4, 8, 9, &RotaryEncoderChanged, 22),
  RotaryEncOverMCP(&mcp4, 10, 11, &RotaryEncoderChanged, 23),
  RotaryEncOverMCP(&mcp4, 12, 13, &RotaryEncoderChanged, 24),
  RotaryEncOverMCP(&mcp5, 0, 1, &RotaryEncoderChanged, 25),
  RotaryEncOverMCP(&mcp5, 2, 3, &RotaryEncoderChanged, 26),
  RotaryEncOverMCP(&mcp5, 4, 5, &RotaryEncoderChanged, 27),
  RotaryEncOverMCP(&mcp5, 8, 9, &RotaryEncoderChanged, 28),
  RotaryEncOverMCP(&mcp5, 10, 11, &RotaryEncoderChanged, 29),
  RotaryEncOverMCP(&mcp5, 12, 13, &RotaryEncoderChanged, 30),
  RotaryEncOverMCP(&mcp6, 0, 1, &RotaryEncoderChanged, 31),
  RotaryEncOverMCP(&mcp6, 2, 3, &RotaryEncoderChanged, 32),
  RotaryEncOverMCP(&mcp6, 4, 5, &RotaryEncoderChanged, 33),
  RotaryEncOverMCP(&mcp6, 8, 9, &RotaryEncoderChanged, 34),
  RotaryEncOverMCP(&mcp6, 10, 11, &RotaryEncoderChanged, 35),
  RotaryEncOverMCP(&mcp6, 12, 13, &RotaryEncoderChanged, 36),
  RotaryEncOverMCP(&mcp7, 0, 1, &RotaryEncoderChanged, 37),
  RotaryEncOverMCP(&mcp7, 2, 3, &RotaryEncoderChanged, 39),
  RotaryEncOverMCP(&mcp7, 4, 5, &RotaryEncoderChanged, 39),
  RotaryEncOverMCP(&mcp7, 8, 9, &RotaryEncoderChanged, 40),
  RotaryEncOverMCP(&mcp7, 10, 11, &RotaryEncoderChanged, 41),
  RotaryEncOverMCP(&mcp7, 12, 13, &RotaryEncoderChanged, 42),
};

// after your rotaryEncoders[] definition
constexpr size_t NUM_MCP = sizeof(allMCPs) / sizeof(allMCPs[0]);
constexpr int numMCPs = (int)(sizeof(allMCPs) / sizeof(*allMCPs));
constexpr int numEncoders = (int)(sizeof(rotaryEncoders) / sizeof(*rotaryEncoders));

// an array of vectors to hold pointers to the encoders on each MCP
std::vector<RotaryEncOverMCP *> encByMCP[NUM_MCP];

Button osc1_PW_Button = Button(&mcp1, 6, OSC1_PW_BUTTON, &mainButtonChanged);
Button osc2_PW_Button = Button(&mcp1, 14, OSC2_PW_BUTTON, &mainButtonChanged);
Button fm_depth_Button = Button(&mcp2, 6, FM_DEPTH_BUTTON, &mainButtonChanged);
Button osc2_detune_Button = Button(&mcp2, 14, OSC2_DETUNE_BUTTON, &mainButtonChanged);
Button osc1_saw_Button = Button(&mcp3, 6, OSC1_SAW_BUTTON, &mainButtonChanged);
Button osc1_pulse_Button = Button(&mcp3, 14, OSC1_PULSE_BUTTON, &mainButtonChanged);
Button osc1_sub_Button = Button(&mcp4, 6, OSC1_SUB_BUTTON, &mainButtonChanged);
Button osc2_saw_Button = Button(&mcp4, 14, OSC2_SAW_BUTTON, &mainButtonChanged);
Button osc2_pulse_Button = Button(&mcp5, 6, OSC2_PULSE_BUTTON, &mainButtonChanged);
Button osc2_tri_Button = Button(&mcp5, 14, OSC2_TRI_BUTTON, &mainButtonChanged);
Button effects_mix_Button = Button(&mcp7, 6, EFFECTS_MIX_BUTTON, &mainButtonChanged);
Button noise_Button = Button(&mcp7, 14, NOISE_BUTTON, &mainButtonChanged);

Button *mainButtons[] = {
  &osc1_PW_Button,
  &osc2_PW_Button,
  &fm_depth_Button,
  &osc2_detune_Button,
  &osc1_saw_Button,
  &osc1_pulse_Button,
  &osc1_sub_Button,
  &osc2_saw_Button,
  &osc2_pulse_Button,
  &osc2_tri_Button,
  &effects_mix_Button,
  &noise_Button,
};

Button *allButtons[] = {
  &osc1_PW_Button,
  &osc2_PW_Button,
  &fm_depth_Button,
  &osc2_detune_Button,
  &osc1_saw_Button,
  &osc1_pulse_Button,
  &osc1_sub_Button,
  &osc2_saw_Button,
  &osc2_pulse_Button,
  &osc2_tri_Button,
  &effects_mix_Button,
  &noise_Button,
};

// GP1
#define glide_A 0
#define glide_B 1
#define osc1_PW_A 2
#define osc1_PW_B 3
#define osc1_PWM_A 4
#define osc1_PWM_B 5
#define OSC1_PW_SW 6
// #define unused 7
#define osc1_saw_A 8
#define osc1_saw_B 9
#define osc1_pulse_A 10
#define osc1_pulse_B 11
#define osc1_sub_A 12
#define osc1_sub_B 13
#define OSC2_PW_SW 14
// #define unused 15

// GP2
#define fm_depth_A 0
#define fm_depth_B 1
#define osc2_PW_A 2
#define osc2_PW_B 3
#define osc2_PWM_A 4
#define osc2_PWM_B 5
#define FM_DEPTH_SW 6
//#define unsused 7
#define osc2_saw_A 8
#define osc2_saw_B 9
#define osc2_pulse_A 10
#define osc2_pulse_B 11
#define osc2_tri_A 12
#define osc2_tri_B 13
#define DETUNE_SW 14
//#define unused 15

// GP3
#define vcf_cut_A 0
#define vcf_cut_B 1
#define vcf_res_A 2
#define vcf_res_B 3
#define vcf_eg_A 4
#define vcf_eg_B 5
#define OSC1_SAW_SW 6
//#define unsused 7
#define vcf_key_A 8
#define vcf_key_B 9
#define vcf_lfo_A 10
#define vcf_lfo_B 11
#define filter_attack_A 12
#define filter_attack_B 13
#define OSC1_PULSE_SW 14
//#define unused 15

// GP4
#define filter_decay_A 0
#define filter_decay_B 1
#define filter_sustain_A 2
#define filter_sustain_B 3
#define filter_release_A 4
#define filter_release_B 5
#define OSC1_SUB_SW 6
//#define unsused 7
#define osc2_detune_A 8
#define osc2_detune_B 9
#define osc2_interval_A 10
#define osc2_interval_B 11
#define amp_attack_A 12
#define amp_attack_B 13
#define LFO_SELECT_LED_RED 14
//#define unused 15

// GP5
#define amp_decay_A 0
#define amp_decay_B 1
#define amp_sustain_A 2
#define amp_sustain_B 3
#define amp_release_A 2
#define amp_release_B 3
#define OSC2_PULSE_SW 6
//#define unsused 7
#define lfo_rate_A 8
#define lfo_rate_B 9
#define lfo_delay_A 10
#define lfo_delay_B 11
#define mw_depth_A 12
#define mw_depath_B 13
#define OSC2_TRI_SW 14
//#define unused 15

// GP6
#define pw_lfo_A 0
#define pw_lfo_B 1
#define pb_depth_A 2
#define pb_depth_B 3
#define noise_A 4
#define noise_B 5
//#define unsused 6
//#define unsused 7
#define at_depth_A 8
#define at_depth_B 9
#define effect_mix_A 10
#define effect_mix_B 11
#define volume_A 12
#define volume_B 13
//#define unused 14
//#define unused 15

// GP7
#define effect1_A 0
#define effect_1_B 1
#define effect_2_A 2
#define effect_2_B 3
#define effect_3_A 4
#define effect_3_B 5
#define EFFECT_MIX_SW 6
//#define unused 7
#define pm_dco_A 8
#define pm_dco_B 9
#define pm_env_A 10
#define pm_env_B 11
#define am_dpeth_A 12
#define am_depth_B 13
#define NOISE_SW 14
//#define unused 15

#define DEMUX_0 36
#define DEMUX_1 35
#define DEMUX_2 34
#define DEMUX_3 33

#define DEMUX_EN_1 2

#define AUTOTUNE_INPUT 54


//Note DAC
#define MULT1V 106.4
#define MULT1_2V 122.5
#define MULT2V 217.4
#define MULT5V 256
#define MULT33V 170.85
//#define DACMULT 6.5
//#define MIDICCTOPOT 8.62
#define MULT3V 344
#define CLAMP2V 26500  // DAC value that corresponds to 2V

#define DAC_CS1 10

// New DeMux 1 Connections A

#define DEMUX1_noiseLevel_Upper 0         // 0-2v
#define DEMUX1_osc1SawLevel_Upper 1       // 0-2v
#define DEMUX1_osc1PulseLevel_Upper 2     // 0-2v
#define DEMUX1_osc1SubLevel_Upper 3       // 0-2v
#define DEMUX1_osc1PM_DCO1_level_Upper 4  // 0-2v
#define DEMUX1_osc1PM_Env_level_Upper 5   // 0-2v
#define DEMUX1_osc2SawLevel_Upper 6       // 0-2v
#define DEMUX1_osc2PulseLevel_Upper 7     // 0-2v
#define DEMUX1_osc2TriLevel_Upper 8       // 0-2v
#define DEMUX1_volumeControl_Upper 9      // 0-2v
#define DEMUX1_effectMix_Upper 10         // 0-2v
#define DEMUX1_FM_LFO_Depth_Upper 11      // 0-2v
#define DEMUX1_TM_LFO_Depth_Upper 12      // 0-2v
#define DEMUX1_AM_LFO_Depth_Upper 13      // 0-2v
#define DEMUX1_spare 14                   // 0-2v
#define DEMUX1_PW_LFO_Rate_Upper 15       // 0-5v

//DeMux 2 Connections B
#define DEMUX2_filterAttack_Upper 0   // 0-5v
#define DEMUX2_filterDecay_Upper 1    // 0-5v
#define DEMUX2_filterSustain_Upper 2  // 0-5v
#define DEMUX2_filterRelease_Upper 3  // 0-5v
#define DEMUX2_ampAttack_Upper 4      // 0-5v
#define DEMUX2_ampDecay_Upper 5       // 0-5v
#define DEMUX2_amp_Sustain_Upper 6    // 0-5v
#define DEMUX2_ampRelease_Upper 7     // 0-5v
#define DEMUX2_egDepth_upper 8        // 0-5v
#define DEMUX2_filterCutoff_Upper 9   // 0-5v
#define DEMUX2_filterRes_Upper 10     // 0-5v
#define DEMUX2_LFO_Rate_Upper 11      // 0-5v
#define DEMUX2_LFO_Wave_Upper 12      // 0-5v
#define DEMUX2_effectPot1_Upper 13    // 0-3.3v
#define DEMUX2_effectPot2_Upper 14    // 0-3.3v
#define DEMUX2_effectPot3_Upper 15    // 0-3.3v

//DeMux 3 Connections C
#define DEMUX3_noiseLevel_Lower 0         // 0-2v
#define DEMUX3_osc1SawLevel_Lower 1       // 0-2v
#define DEMUX3_osc1PulseLevel_Lower 2     // 0-2v
#define DEMUX3_osc1SubLevel_Lower 3       // 0-2v
#define DEMUX3_osc1PM_DCO1_level_Lower 4  // 0-2v
#define DEMUX3_osc1PM_Env_level_Lower 5   // 0-2v
#define DEMUX3_osc2SawLevel_Lower 6       // 0-2v
#define DEMUX3_osc2PulseLevel_Lower 7     // 0-2v
#define DEMUX3_osc2TriLevel_Lower 8       // 0-2v
#define DEMUX3_volumeControl_Lower 9      // 0-2v
#define DEMUX3_effectMix_Lower 10         // 0-2v
#define DEMUX3_FM_LFO_Depth_Lower 11      // 0-2v
#define DEMUX3_TM_LFO_Depth_Lower 12      // 0-2v
#define DEMUX3_AM_LFO_Depth_Lower 13      // 0-2v
#define DEMUX3_spare 14                   // 0-2v
#define DEMUX3_PW_LFO_Rate_Lower 15       // 0-5v

//DeMux 4 Connections D
#define DEMUX4_filterAttack_Lower 0   // 0-5v
#define DEMUX4_filterDecay_Lower 1    // 0-5v
#define DEMUX4_filterSustain_Lower 2  // 0-5v
#define DEMUX4_filterRelease_Lower 3  // 0-5v
#define DEMUX4_ampAttack_Lower 4      // 0-5v
#define DEMUX4_ampDecay_Lower 5       // 0-5v
#define DEMUX4_amp_Sustain_Lower 6    // 0-5v
#define DEMUX4_ampRelease_Lower 7     // 0-5v
#define DEMUX4_egDepth_Lower 8        // 0-5v
#define DEMUX4_filterCutoff_Lower 9   // 0-5v
#define DEMUX4_filterRes_Lower 10     // 0-5v
#define DEMUX4_LFO_Rate_Lower 11      // 0-5v
#define DEMUX4_LFO_Wave_Lower 12      // 0-5v
#define DEMUX4_effectPot1_Upper 13    // 0-3.3v
#define DEMUX4_effectPot2_Lower 14    // 0-3.3v
#define DEMUX4_effectPot3_Lower 15    // 0-3.3v

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
#define KEYTRACK_SW 12
#define FILTER_TYPE_SW 13
#define FILTER_POLE_SW 14
#define EG_INVERT_SW 15

#define FILTER_ENV_VELOCITY_SW 16
#define FILTER_ENV_LIN_LOG_SW 17
#define FILTER_ENV_LOOP_SW 18
#define AMP_ENV_VELOCITY_SW 19
#define AMP_ENV_LIN_LOG_SW 20
#define AMP_ENV_LOOP_SW 21
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

#define EFFECT_2_UPPER 16
#define EFFECT_1_UPPER 17
#define EFFECT_0_UPPER 18
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
#define UPPER_RELAY_3 37
#define FILTER_LIN_LOG_LOWER 38
#define AMP_LIN_LOG_LOWER 39

#define EFFECT_2_LOWER 40
#define EFFECT_1_LOWER 41
#define EFFECT_0_LOWER 42
#define EFFECT_INTERNAL_LOWER 43
#define FILTER_POLE_LOWER 44
#define FILTERA_LOWER 45
#define FILTERB_LOWER 46
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
#define UPPER_RELAY_2 57  // LEDs for LFO (lower Default)
#define LFO_MULTI_BIT0_UPPER 58
#define LFO_MULTI_BIT1_UPPER 59
#define LFO_MULTI_BIT2_UPPER 60
#define LFO_MULTI_BIT0_LOWER 61
#define LFO_MULTI_BIT1_LOWER 62
#define LFO_MULTI_BIT2_LOWER 63

// System Switches etc

#define TUNE_BUTTON 16
#define TUNE_LED 17

#define RECALL_SW 20
#define SAVE_SW 23
#define SETTINGS_SW 22
#define BACK_SW 21

#define ENCODER_PINA 4
#define ENCODER_PINB 5

#define DEBOUNCE 30
#define DEMUXCHANNELS 16

static byte muxOutput = 0;

static long encPrevious = 0;

TButton saveButton{ SAVE_SW, LOW, HOLD_DURATION, DEBOUNCE, CLICK_DURATION };
TButton settingsButton{ SETTINGS_SW, LOW, HOLD_DURATION, DEBOUNCE, CLICK_DURATION };
TButton backButton{ BACK_SW, LOW, HOLD_DURATION, DEBOUNCE, CLICK_DURATION };
TButton recallButton{ RECALL_SW, LOW, HOLD_DURATION, DEBOUNCE, CLICK_DURATION };  //On encoder

Encoder encoder(ENCODER_PINB, ENCODER_PINA);  //This often needs the pins swapping depending on the encoder

void setupHardware() {

  pinMode(DAC_CS1, OUTPUT);
  digitalWrite(DAC_CS1, HIGH);

  pinMode(DEMUX_0, OUTPUT);
  pinMode(DEMUX_1, OUTPUT);
  pinMode(DEMUX_2, OUTPUT);
  pinMode(DEMUX_3, OUTPUT);

  digitalWrite(DEMUX_0, LOW);
  digitalWrite(DEMUX_1, LOW);
  digitalWrite(DEMUX_2, LOW);
  digitalWrite(DEMUX_3, LOW);

  pinMode(DEMUX_EN_1, OUTPUT);

  digitalWrite(DEMUX_EN_1, HIGH);

  pinMode(TUNE_LED, OUTPUT);
  digitalWrite(TUNE_LED, LOW);

  pinMode(TUNE_BUTTON, INPUT_PULLUP);

  pinMode(AUTOTUNE_INPUT, INPUT);

  //Switches

  pinMode(RECALL_SW, INPUT_PULLUP);  //On encoder
  pinMode(SAVE_SW, INPUT_PULLUP);
  pinMode(SETTINGS_SW, INPUT_PULLUP);
  pinMode(BACK_SW, INPUT_PULLUP);
}
