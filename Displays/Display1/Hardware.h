#define PRIORITY_RED_LED 6
#define PRIORITY_GREEN_LED 7
#define KEYTRACK_LED 8

void SetupHardware() {

  pinMode(PRIORITY_RED_LED, OUTPUT);
  pinMode(PRIORITY_GREEN_LED, OUTPUT);
  pinMode(KEYTRACK_LED, OUTPUT);

  digitalWrite(PRIORITY_RED_LED, LOW);
  digitalWrite(PRIORITY_GREEN_LED, LOW);
  digitalWrite(KEYTRACK_LED, LOW);
  
}