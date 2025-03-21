#define FILTER_POLE_LED 6
#define EG_INVERT_LED 7

void SetupHardware() {

  pinMode(FILTER_POLE_LED, OUTPUT);
  pinMode(EG_INVERT_LED, OUTPUT);

  digitalWrite(FILTER_POLE_LED, LOW);
  digitalWrite(EG_INVERT_LED, LOW);

}