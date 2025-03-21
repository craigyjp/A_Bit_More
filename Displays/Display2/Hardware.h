#define SYNC_LED 6

void SetupHardware() {

  pinMode(SYNC_LED, OUTPUT);

  digitalWrite(SYNC_LED, LOW);

}