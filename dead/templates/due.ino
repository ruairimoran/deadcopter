//  Deadcopter is learning. Stay tuned.
// {{timestamp}}

#include <Arduino.h>
#include "receiver.h"
#include "imu.h"
#include "fly.h"
#include "actuators.h"

Receiver _receiver;
Imu _imu;
Fly _controller;
Esc _motors;

void get_read_ppm(void) {
    _receiver.read_ppm();
}

void setup() {
    attachInterrupt(digitalPinToInterrupt(RX_PIN), get_read_ppm, RISING);  // enabling interrupt on pin RX_PIN
    Serial.begin(115200);
    while(!Serial) {
      ; // wait for serial port to connect
    }
}

void loop() {
    Serial.println("Hello");
    delay(100);
}