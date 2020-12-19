#define RX_PIN 7  // input pin for wire from receiver

void setup() {
  // put your setup code here, to run once:
  pinMode(RX_PIN, INPUT);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(pulseIn(RX_PIN, HIGH));
}
