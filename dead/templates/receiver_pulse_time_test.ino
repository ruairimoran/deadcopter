// {{timestamp}}

#include <Arduino.h>

#define RX_PIN {{receiver_pin}}  // input pin for wire from receiver

void setup() {
  pinMode(RX_PIN, INPUT);
  Serial.begin(115200);
}

void loop() {
  Serial.println(pulseIn(RX_PIN, HIGH));
}