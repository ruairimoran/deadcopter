#define RX_PIN 7  // input pin for wire from receiver

void setup() {
  pinMode(RX_PIN, INPUT);
  Serial.begin(115200);
}

void loop() {
  Serial.println(pulseIn(RX_PIN, HIGH));
}
