//  Deadcopter is learning. Stay tuned.
// {{timestamp}}

// DueTimer Timers 0,2,3,4,5 unavailable due to use of Servo library
#include <Arduino.h>
#include <DueTimer.h>
#include "receiver.h"
#include "imu.h"
#include "fly.h"
#include "actuators.h"

Receiver due_receiver;  // create receiver object from receiver.h
Imu due_imu;  // create imu object from imu.h
Fly due_controller;  // create controller/observer object from fly.h
Esc due_motors;  // create receiver object from receiver.h

bool flag_refresh_receiver = false;  // for interrupt to tell loop to update receiver values
bool flag_flight_control = false;  // for interrupt to tell loop to run flight control
bool due_arm_status = false;  // motors armed status
bool due_arm_switch_status = false;  // Tx arm switch status
int due_throttle = 0;  // receiver throttle value
int due_roll = 0;  // receiver roll value
int due_pitch = 0;  // receiver pitch value
int due_yaw = 0;  // receiver yaw value
float due_y_negative1 = 0;  // imu q0
float due_y_0 = 0;  // imu q1
float due_y_1 = 0;  // imu q2
float due_y_2 = 0;  // imu q3
float due_y_3 = 0;  // imu angular acceleration x
float due_y_4 = 0;  // imu angular acceleration y
float due_y_5 = 0;  // imu angular acceleration z
int due_front_left = 0;  // front left motor pwm
int due_front_right = 0;  // front right motor pwm
int due_back_left = 0;  // back left motor pwm
int due_back_right = 0;  // back right motor pwm

// for readings and timings // to be deleted
float _u0, _u1, _u2;
float q0y, _y0, _y1, _y2, _y3, _y4, _y5;
unsigned long millisPerSerialOutput, millisNow1, millisPrevious1;

void get_read_ppm() {
    due_receiver.read_ppm();  // cannot call function directly in interrupt
}

void flight_control() {
    // get receiver control
    due_receiver.read_channels(due_throttle, due_roll, due_pitch, due_yaw);
    // get imu values
    due_imu.update_imu_data(due_y_negative1, due_y_0, due_y_1, due_y_2, due_y_3, due_y_4, due_y_5);
    // setup y and r matrices for calculations in observe_and_control method
    due_controller.set_matrix_r_and_y(due_roll, due_pitch, due_yaw, due_y_negative1, due_y_0, due_y_1, due_y_2, due_y_3, due_y_4, due_y_5);
    // compute control actions for each motor
    due_controller.observe_and_control(due_throttle, due_front_left, due_front_right, due_back_left, due_back_right, _u0, _u1, _u2, q0y, _y0, _y1, _y2, _y3, _y4, _y5);
    // check if copter is armed
    due_motors.get_arm_status(due_arm_status);
    // if copter is armed, send control action to motor ESCs
    if(due_arm_status == true) {
        due_motors.write_speed_to_esc(due_front_left, due_front_right, due_back_left, due_back_right);
    }
}

void setup() {
    attachInterrupt(digitalPinToInterrupt(RX_PIN), get_read_ppm, RISING);  // enable receiver ppm interrupt
    attachInterrupt(digitalPinToInterrupt(IMU_INTERRUPT_PIN), flight_control, RISING);  // enable data ready interrupt from imu
    due_imu.configure_imu_and_madgwick();  // start communication with imu with madgwick filter
    due_motors.attach_esc_to_pwm_pin();
    due_motors.disarm();
    due_motors.get_arm_status(due_arm_status);

//     // for serial output // to be deleted
//     Serial.begin(115200);
//     millisPerSerialOutput = 500;  // 0.5s refresh rate
//     millisPrevious1 = millis();
}

void loop() {
//----------------------------------------------------------------------------------------------------------------------
    // arm/disarm motors with switch on aux_channel_1

    due_motors.get_arm_status(due_arm_status);  // check if motors are armed
    if(due_receiver.aux_channel_1 < 1500 && due_throttle < 1160) {
        due_arm_switch_status = true;  // allow motors to arm
    }
    if(due_receiver.aux_channel_1 > 1500 && due_throttle > 1160) {
        due_arm_switch_status = false;  // stop motors from arming until throttle low and Tx arm switch has been toggled off first
    }
    if(due_receiver.aux_channel_1 > 1500 && due_arm_switch_status == true && due_throttle < 1160 && due_arm_status == false) {
        due_motors.arm();  // if Tx arm switch is High and went high while throttle was low, throttle is low and motors are disarmed
    }                      // then arm motors
    if (due_receiver.aux_channel_1 < 1500) {
        due_motors.disarm();  // if Tx arm switch Low, disarm motors indefinitely
    }

//----------------------------------------------------------------------------------------------------------------------

//     // to be deleted
//     millisNow1 = millis();
//     if(millisNow1 - millisPrevious1 >= millisPerSerialOutput) {
//          Serial.print(_u0, 7);Serial.print("\t");
//          Serial.print(_u1, 7);Serial.print("\t");
//          Serial.print(_u2, 7);Serial.print("\n");
//
//          Serial.print(q0y, 7);Serial.print("\t");
//          Serial.print(_y0, 7);Serial.print("\t");
//          Serial.print(_y1, 7);Serial.print("\t");
//          Serial.print(_y2, 7);Serial.print("\t");
//          Serial.print(_y3, 7);Serial.print("\t");
//          Serial.print(_y4, 7);Serial.print("\t");
//          Serial.print(_y5, 7);Serial.print("\n");
//
//          Serial.print(due_arm_status);Serial.print("\t");
//          Serial.print(due_throttle);Serial.print("\t");
//          Serial.print(due_yaw);Serial.print("\t");
//          Serial.print(due_pitch);Serial.print("\t");
//          Serial.print(due_roll);Serial.print("\n");
//
//          Serial.print(due_front_left);Serial.print("\t");
//          Serial.print(due_front_right);Serial.print("\t");
//          Serial.print(due_back_left);Serial.print("\t");
//          Serial.print(due_back_right);Serial.print("\n");
//
//          millisPrevious1 = millisPrevious1 + millisPerSerialOutput;
//     }
}

