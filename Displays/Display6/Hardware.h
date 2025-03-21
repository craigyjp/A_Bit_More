#define LFO_ALT_LED 6
#define LFO_MULTI_MONO_LED 7

void SetupHardware() {

pinMode(LFO_ALT_LED, OUTPUT);
pinMode(LFO_MULTI_MONO_LED, OUTPUT);

digitalWrite(LFO_ALT_LED, LOW);
digitalWrite(LFO_MULTI_MONO_LED, LOW);

}