//  Deadcopter is learning. Stay tuned.
// {{timestamp}}

// DueTimer Timers 0,2,3,4,5 unavailable due to use of Servo library
#include <Arduino.h>
#include <DueTimer.h>
#include "receiver.h"
#include "imu.h"
#include "fly.h"
#include "actuators.h"

#define BUZZ_PIN {{buzzer_pin}}

Receiver due_receiver;  // create receiver object from receiver.h
Imu due_imu;  // create imu object from imu.h
Fly due_controller;  // create controller/observer object from fly.h
Esc due_motors;  // create receiver object from receiver.h

bool flag_flight_control = false;  // for interrupt to tell loop to run flight control
bool flag_channels_updated = false;  // only allow channels to be read once each rx timeframe
int fc_count = 0;  // longer timer for "clean up" code
int due_throttle = 0;  // receiver throttle value
int due_roll = 0;  // receiver roll value
int due_pitch = 0;  // receiver pitch value
int due_yaw = 0;  // receiver yaw value
int due_aux_channel_1 = 0;  // receiver aux 1 value
int due_aux_channel_2 = 0;  // receiver aux 2 value
int due_aux_channel_3 = 0;  // receiver aux 3 value
int due_aux_channel_4 = 0;  // receiver aux 4 value
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
float _u0 = 0;
float _u1 = 0;
float _u2 = 0;
float q0y = 0;
float _y0 = 0;
float _y1 = 0;
float _y2 = 0;
float _y3 = 0;
float _y4 = 0;
float _y5 = 0;
unsigned long millisPerSerialOutput = 500;  // 500ms refresh rate
unsigned long millisNow = 0;
unsigned long millisPrevious = 0;
unsigned long imu_read_time_new = 0;
unsigned long imu_read_time_old = 0;
unsigned long imu_read_time_difference = 0;

void get_ISR_read_ppm() {
    due_receiver.ISR_read_ppm();  // cannot call function directly from interrupt setup
}

void ISR_flight_control() {
    // set interrupt flag
    flag_flight_control = true;
}

void setup() {
    pinMode(BUZZ_PIN, OUTPUT);
    due_imu.configure_imu_and_madgwick();  // start communication with imu with madgwick filter
    due_motors.attach_esc_to_pwm_pin();
    due_motors.disarm();
    attachInterrupt(digitalPinToInterrupt(RX_PIN), get_ISR_read_ppm, RISING);  // enable receiver ppm interrupt
    Timer6.attachInterrupt(ISR_flight_control).setFrequency(SAMPLING_FREQUENCY).start();  // read imu data at SAMPLING_FREQUENCY

    // for serial output // to be deleted
    Serial.begin(115200);
}

void loop() {
//----------------------------------------------------------------------------------------------------------------------
    // poll imu interrupt pin for data ready

    if(flag_flight_control == true) {
        // count 5 runs of function - makes 40ms timer
        fc_count += 1;
        if(fc_count > 5) {
            flag_channels_updated = false;
            fc_count = 0;
        }

        // make sure imu data is being read at correct sampling time
        imu_read_time_new = micros();
        imu_read_time_difference = imu_read_time_new - imu_read_time_old;
        imu_read_time_old = imu_read_time_new;

        // get imu values
        due_imu.update_imu_data(due_y_negative1, due_y_0, due_y_1, due_y_2, due_y_3, due_y_4, due_y_5);
        // setup y and r matrices for calculations in observe_and_control method
        due_controller.set_matrix_r_and_y(due_roll, due_pitch, due_yaw, due_y_negative1, due_y_0, due_y_1, due_y_2, due_y_3, due_y_4, due_y_5);
        // compute control actions for each motor
        due_controller.observe_and_control(due_throttle, due_front_left, due_front_right, due_back_left, due_back_right, _u0, _u1, _u2, q0y, _y0, _y1, _y2, _y3, _y4, _y5);
        // if copter is armed
        if(due_motors.get_arm_status()) {
            // send control action to motor ESCs
            due_motors.write_speed_to_esc(due_front_left, due_front_right, due_back_left, due_back_right);
        }

        // reset interrupt flag
        flag_flight_control = false;
    }

//----------------------------------------------------------------------------------------------------------------------
    // "clean up" code - wait for break in PPM and only allowed to run once every 40ms

    if((flag_channels_updated == false) && (due_receiver.get_channel() > NO_OF_CHANNELS)) {
    //------------------------------------------------------------------------------------------------------------------

        flag_channels_updated = true;  // set channels updated flag

    //------------------------------------------------------------------------------------------------------------------
        // get receiver control

        due_receiver.read_channels(due_throttle, due_roll, due_pitch, due_yaw, due_aux_channel_1, due_aux_channel_2, due_aux_channel_3, due_aux_channel_4);

    //------------------------------------------------------------------------------------------------------------------
        // disarm motors when left stick held in bottom left

        if((due_throttle < 1080) && (due_yaw < 1080) && (due_motors.get_arm_status() == true)) {
            due_motors.disarm();  // disarm motors
        }

    //------------------------------------------------------------------------------------------------------------------
        // arm motors with left stick held in bottom right

        if((due_throttle < 1080) && (due_yaw > 1910) && (due_motors.get_arm_status() == false)) {
            due_motors.arm();  // arm motors
        }

    //------------------------------------------------------------------------------------------------------------------
        // sound buzzer if imu reading slows down
        if(imu_read_time_difference > (1000 + 1.0e+6/(float) SAMPLING_FREQUENCY)) {
            digitalWrite(BUZZ_PIN, HIGH);
        }
        else {
            digitalWrite(BUZZ_PIN, LOW);
        }

    //------------------------------------------------------------------------------------------------------------------
        // to be deleted

        millisNow = millis();
        if(millisNow - millisPrevious >= millisPerSerialOutput) {
            Serial.print(imu_read_time_difference);Serial.print("\t");

            Serial.print(_u0, 7);Serial.print("\t");
            Serial.print(_u1, 7);Serial.print("\t");
            Serial.print(_u2, 7);Serial.print("\t");

            Serial.print(q0y, 7);Serial.print("\t");
            Serial.print(_y0, 7);Serial.print("\t");
            Serial.print(_y1, 7);Serial.print("\t");
            Serial.print(_y2, 7);Serial.print("\t");

            Serial.print(due_motors.get_arm_status());Serial.print("\t");
            Serial.print(due_throttle);Serial.print("\t");
            Serial.print(due_yaw);Serial.print("\t");
            Serial.print(due_pitch);Serial.print("\t");
            Serial.print(due_roll);Serial.print("\t");

            Serial.print(due_front_left);Serial.print("\t");
            Serial.print(due_front_right);Serial.print("\t");
            Serial.print(due_back_left);Serial.print("\t");
            Serial.print(due_back_right);Serial.print("\n");

            millisPrevious = millisPrevious + millisPerSerialOutput;
        }
    }
}
