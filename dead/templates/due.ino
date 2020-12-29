//  Deadcopter is learning. Stay tuned.
// {{timestamp}}

#include <Arduino.h>
#include "receiver.h"
#include "imu.h"
#include "fly.h"
#include "actuators.h"

Receiver due_receiver;
Imu due_imu;
Fly due_controller;
Esc due_motors;

int due_throttle, due_roll, due_pitch, due_yaw;  // for receiver control inputs
float due_y_negative1, due_y_0, due_y_1, due_y_2, due_y_3, due_y_4, due_y_5;  // for filtered imu readings
int due_front_left, due_front_right, due_back_left, due_back_right;  // for motor speed pwm in milliseconds

float _u0, _u1, _u2;
float q0y, _y0, _y1, _y2, _y3, _y4, _y5;

unsigned long millisPerSerialOutput, millisPerReading, millisNow0, millisPrevious0, millisNow1, millisPrevious1;

void get_read_ppm(void) {
    due_receiver.read_ppm();
}

void setup() {
    attachInterrupt(digitalPinToInterrupt(RX_PIN), get_read_ppm, RISING);  // enabling interrupt on pin RX_PIN
    due_imu.configure_imu_and_madgwick();
    due_motors.attach_esc_to_pwm_pin();
    due_motors.disarm();
    Serial.begin(115200);
    millisPerSerialOutput = 500;  // 0.5s refresh rate
    millisPerReading = 10;  // 10 ms refresh rate
    millisPrevious0 = millis();
    millisPrevious1 = millis();
}

void loop() {
    millisNow0 = millis();
    millisNow1 = millis();

    if(millisNow0 - millisPrevious0 >= millisPerReading) {
        due_receiver.decode_ppm(due_throttle, due_roll, due_pitch, due_yaw);
        due_imu.update_imu_data(due_y_negative1, due_y_0, due_y_1, due_y_2, due_y_3, due_y_4, due_y_5);
        due_controller.set_matrix_r_and_y(due_roll, due_pitch, due_yaw, due_y_negative1, due_y_0, due_y_1, due_y_2, due_y_3, due_y_4, due_y_5);
        due_controller.observe_and_control(due_front_left, due_front_right, due_back_left, due_back_right, _u0, _u1, _u2, q0y, _y0, _y1, _y2, _y3, _y4, _y5);

        millisPrevious0 = millisPrevious0 + millisPerReading;
    }

    if(millisNow1 - millisPrevious1 >= millisPerSerialOutput) {
        Serial.print(_u0, 7);Serial.print("\t");
        Serial.print(_u1, 7);Serial.print("\t");
        Serial.print(_u2, 7);Serial.print("\n");

        Serial.print(q0y, 7);Serial.print("\t");
        Serial.print(_y0, 7);Serial.print("\t");
        Serial.print(_y1, 7);Serial.print("\t");
        Serial.print(_y2, 7);Serial.print("\t");
        Serial.print(_y3, 7);Serial.print("\t");
        Serial.print(_y4, 7);Serial.print("\t");
        Serial.print(_y5, 7);Serial.print("\n");

        Serial.print(due_throttle);Serial.print("\t");
        Serial.print(due_yaw);Serial.print("\t");
        Serial.print(due_pitch);Serial.print("\t");
        Serial.print(due_roll);Serial.print("\n");

        Serial.print(due_front_left);Serial.print("\t");
        Serial.print(due_front_right);Serial.print("\t");
        Serial.print(due_back_left);Serial.print("\t");
        Serial.print(due_back_right);Serial.print("\n");

        millisPrevious1 = millisPrevious1 + millisPerSerialOutput;
    }
}
