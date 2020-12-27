//  Deadcopter is learning. Stay tuned.
// 2020-12-27 23:34:16.352123

#include <Arduino.h>
#include "receiver.h"
#include "imu.h"
#include "fly.h"
#include "actuators.h"

Receiver due_receiver;
Imu due_imu;
Fly due_controller;
Esc due_motors;

float u1, u2, u3;

void get_read_ppm(void) {
    due_receiver.read_ppm();
}

void setup() {
    attachInterrupt(digitalPinToInterrupt(RX_PIN), get_read_ppm, RISING);  // enabling interrupt on pin RX_PIN
    due_imu.configure_imu_and_madgwick();
    due_motors.attach_esc_to_pwm_pin();
    due_motors.disarm();
    Serial.begin(115200);
}

void loop() {
    u1, u2, u3 = due_controller.observe_and_control(due_imu, due_receiver);
    Serial.print(u1, 7);Serial.print("\t");
    Serial.print(u2, 7);Serial.print("\t");
    Serial.print(u3, 7);Serial.print("\n");

    due_receiver.decode_ppm();
    Serial.print(due_receiver.rx_throttle);Serial.print("\t");
    Serial.print(due_receiver.rx_rudder);Serial.print("\t");
    Serial.print(due_receiver.rx_pitch);Serial.print("\t");
    Serial.print(due_receiver.rx_roll);Serial.print("\n");
    delay(10);
}