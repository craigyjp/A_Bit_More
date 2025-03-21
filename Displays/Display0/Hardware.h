#define GLIDE_LED 6
#define LOWER_LED 7
#define UPPER_LED 8
#define CHORD_HOLD_LED 9
#define KEY_MODE_RED_LED 10
#define KEY_MODE_GREEN_LED 11
#define POLY1_LED 14
#define POLY2_LED 26
#define MONO_LED 28
#define UNISON_LED 29

void SetupHardware() {

    pinMode(GLIDE_LED, OUTPUT);
    pinMode(LOWER_LED, OUTPUT);
    pinMode(UPPER_LED, OUTPUT);
    pinMode(CHORD_HOLD_LED, OUTPUT);
    pinMode(KEY_MODE_RED_LED, OUTPUT);
    pinMode(KEY_MODE_GREEN_LED, OUTPUT);
    pinMode(POLY1_LED, OUTPUT);
    pinMode(POLY2_LED, OUTPUT);
    pinMode(MONO_LED, OUTPUT);
    pinMode(UNISON_LED, OUTPUT);

    digitalWrite(GLIDE_LED, LOW);
    digitalWrite(LOWER_LED, LOW);
    digitalWrite(UPPER_LED, LOW);
    digitalWrite(CHORD_HOLD_LED, LOW);
    digitalWrite(KEY_MODE_RED_LED, LOW);
    digitalWrite(KEY_MODE_GREEN_LED, LOW);
    digitalWrite(POLY1_LED, LOW);
    digitalWrite(POLY2_LED, LOW);
    digitalWrite(MONO_LED, LOW);
    digitalWrite(UNISON_LED, LOW);

}
