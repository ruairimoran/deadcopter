#include "Servo.h"
#include "Adafruit_Sensor.h"
#include "config.h"
#pragma once
//#define DEG_TO_RAD PI / 180.0 // already defined in Arduino.h
//#define RAD_TO_DEG 180.0 / PI // already defined in Arduino.h

// general vars
#ifdef RADIANS
#define ANGLE_CONVERTION_FACTOR 1
#elif defined(DEGREES)
#define ANGLE_CONVERTION_FACTOR RAD_TO_DEG
#endif
unsigned long dt = 0;


//IMU vars :: states => [quaternions, gyro_event, **motors_rps**]
/* delta-acceleration for calibration (m/s^2) */
double a_calib[3];

/* delta-w for calibration (rad/s) */
double w_calib[3];

#define GRAVITATIONAL_ACCELERATION 9.81
int imu_status;
sensors_event_t gyro_event, acc_event, magneto, temperature;
sensors_vec_t orientation;
typedef struct {
  float q0, q1, q2, q3;
} Quaternions;
//typedef struct Quaternions Quaternions;
Quaternions quaternions;

//radio vars
typedef struct Radio_data {
  sensors_vec_t orientation;
  float throttle;
  float switches[12];
};
Radio_data radio_data;
Quaternions radio_quaternions;


//motor vars
Servo motors[4];
typedef struct {
  int motor0, motor1, motor2, motor3;
} Motor_speeds;
Motor_speeds motor_speeds;

//states, inputs and outputs of ss vars
typedef struct {
  float nx, ny, nz;
} Rps;
Rps rps;

typedef struct {
  float u1, u2, u3;
} Inputs;
Inputs inputs, eq_inputs;

typedef struct {
  Quaternions quaternions;
  sensors_event_t omega;
} Outputs;
Outputs outputs;

typedef struct {
  Quaternions quaternions;
  sensors_event_t omega;
  Rps rps;
} States;
States eq_states, x_minus_xeq;
