//  Deadcopter is learning. Stay tuned.
// {{timestamp}}

// DueTimer Timers 0,2,3,4,5 unavailable due to use of Servo library
#include <Arduino.h>
#include <DueTimer.h>
#include "receiver.h"
#include "imu.h"
#include "fly.h"
#include "actuators.h"

Receiver due_receiver;
Imu due_imu;
Fly due_controller;
Esc due_motors;

bool flag_refresh_receiver, flag_flight_control;  // flags for interrupt service routines
bool due_arm_status, due_arm_switch_status;  // for motors armed status and Tx switch status
int due_throttle, due_roll, due_pitch, due_yaw;  // for receiver control inputs
float due_y_negative1, due_y_0, due_y_1, due_y_2, due_y_3, due_y_4, due_y_5;  // for filtered imu readings
int due_front_left, due_front_right, due_back_left, due_back_right;  // for motor speed pwm in milliseconds

// for readings and timings // to be deleted
float _u0, _u1, _u2;
float q0y, _y0, _y1, _y2, _y3, _y4, _y5;
unsigned long millisPerSerialOutput, millisNow1, millisPrevious1;

void get_read_ppm() {
    due_receiver.read_ppm();  // cannot call function directly in interrupt
}

void refresh_receiver() {
    flag_refresh_receiver = true;
}

void flight_control() {
    flag_flight_control = true;
}

void setup() {
    due_arm_switch_status = false;  // motors cannot arm
    flag_refresh_receiver = false;  // for timed update of stored receiver values
    flag_flight_control = false;  // for timed flight control updating
    attachInterrupt(digitalPinToInterrupt(RX_PIN), get_read_ppm, RISING);  // enable receiver ppm interrupt
    Timer1.attachInterrupt(refresh_receiver).setPeriod(40000).start();  // refresh receiver values every 40ms - each ppm frame is 20ms
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
    // update receiver values
    if(flag_refresh_receiver == true) {
        // get receiver control
        due_receiver.decode_ppm(due_throttle, due_roll, due_pitch, due_yaw);
        flag_refresh_receiver = false;
    }

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
    // flight control

    if(flag_flight_control == true) {
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
        flag_flight_control = false;
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

