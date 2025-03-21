#define PM_DCO1_DEST_LED 6
#define PM_FILT_ENV_DEST_LED 7
#define AMP_GATED_LED 8

void SetupHardware() {

pinMode(PM_DCO1_DEST_LED, OUTPUT);
pinMode(PM_FILT_ENV_DEST_LED, OUTPUT);
pinMode(AMP_GATED_LED, OUTPUT);

digitalWrite(PM_DCO1_DEST_LED, LOW);
digitalWrite(PM_FILT_ENV_DEST_LED, LOW);
digitalWrite(AMP_GATED_LED, LOW);

}