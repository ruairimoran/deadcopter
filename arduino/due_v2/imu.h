#include "Arduino.h"
#include <math.h>

#include "config.h"
#include "Adafruit_10DOF_mod.h"
#include "MadgwickAHRS.h"
#include "mpu9250.h"
#include "debug.h"
#include "vars.h"
#include "gen_functions.h"


bfs::Mpu9250 imu_lib(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);  // for using MPU library
//bfs::Mpu9250 imu;

int _configure_imu(void) {
  // start communication with imu
  imu_status = imu_lib.Begin();
  imu_lib.ConfigAccelRange(bfs::Mpu9250::ACCEL_RANGE_8G);     // GOTO imu_lib readme for possible ranges
  imu_lib.ConfigGyroRange(bfs::Mpu9250::GYRO_RANGE_1000DPS);  // GOTO imu_lib readme for possible ranges
  imu_lib.ConfigDlpfBandwidth(bfs::Mpu9250::DLPF_BANDWIDTH_20HZ);
  // Data Output Rate = 1000 / (1 + SRD)
  // output rate should be > double Dlpf
  // setting SRD to 7 for a 125Hz output rate
  // mag fixed at 100Hz for SRD<=9, 8Hz for SRD>9
  imu_lib.ConfigSrd(7);  // gyro/accel/temp = 125Hz, mag = 100Hz
  //    imu_lib.setAccelCalX(bias, scaleFactor);  // set sensor calibrations from "imu_calibration.ino" sketch
  //    imu_lib.setAccelCalY(bias, scaleFactor);
  //    imu_lib.setAccelCalZ(bias, scaleFactor);
  //    imu_lib.setMagCalX(bias, scaleFactor);
  //    imu_lib.setMagCalY(bias, scaleFactor);
  //    imu_lib.setMagCalZ(bias, scaleFactor);
  return imu_status;
}

bool _calibrate_imu(void) {
  //    imu_lib.calibrateMag();
  // imu_lib.calibrateAccel();
  // imu_lib.calibrateGyro();
  int num_samples_calib = 80;
  int i;
  for (i = 0; i < num_samples_calib; i++) {
    if (imu_lib.Read()) {
      acc_event.acceleration.x = imu_lib.accel_x_mps2();
      acc_event.acceleration.y = imu_lib.accel_y_mps2();
      acc_event.acceleration.z = imu_lib.accel_z_mps2();
      gyro_event.gyro.x = imu_lib.gyro_x_radps();
      gyro_event.gyro.y = imu_lib.gyro_y_radps();
      gyro_event.gyro.z = imu_lib.gyro_z_radps();
    }
    a_calib[0] += acc_event.acceleration.x;
    a_calib[1] += acc_event.acceleration.y;
    a_calib[2] += acc_event.acceleration.z;
    w_calib[0] += gyro_event.gyro.x;
    w_calib[1] += gyro_event.gyro.y;
    w_calib[2] += gyro_event.gyro.z;
  }
  a_calib[0] /= double(num_samples_calib);
  a_calib[1] /= double(num_samples_calib);
  a_calib[2] /= double(num_samples_calib);
  w_calib[0] /= double(num_samples_calib);
  w_calib[1] /= double(num_samples_calib);
  w_calib[2] /= double(num_samples_calib);
  return true;
}


void correct_a() {
  acc_event.acceleration.x -= a_calib[0];
  acc_event.acceleration.y -= a_calib[1];
  acc_event.acceleration.z = acc_event.acceleration.z - a_calib[2] - GRAVITATIONAL_ACCELERATION;
  gyro_event.gyro.x -= w_calib[0];
  gyro_event.gyro.y -= w_calib[1];
  gyro_event.gyro.z -= w_calib[2];
}

bool _begin_imu() {
  if (!(_configure_imu() == 1 && _calibrate_imu())) return false;
  return true;
}

bool _update_imu_data() {

  // read imu_lib and store in buffer
  if (!imu_lib.Read()) return false;
  // get data from buffer in m.s^-2 and rads.s^-1 and micro Tesla
  acc_event.acceleration.x = imu_lib.accel_x_mps2();
  acc_event.acceleration.y = imu_lib.accel_y_mps2();
  acc_event.acceleration.z = imu_lib.accel_z_mps2();
  gyro_event.gyro.x = imu_lib.gyro_x_radps();
  gyro_event.gyro.y = imu_lib.gyro_y_radps();
  gyro_event.gyro.z = imu_lib.gyro_z_radps();
  // magneto.magnetic.x = imu_lib.getMagX_uT();
  // magneto.magnetic.y = imu_lib.getMagY_uT();
  // magneto.magnetic.z = imu_lib.getMagZ_uT();
  // temperature.temperature = imu_lib.getTemperature_C();
  return true;
}


#ifdef USE_KALMAN_IMU
Adafruit_10DOF_Kalman_Filter kalman = Adafruit_10DOF_Kalman_Filter(SAMPLING_FREQUENCY);
float offset_gyro[3] = { 0, 0, 0 };
int i;

bool calibrate_imu() {
  debug_print2("imu.h::kalman::calibrate_imu()\n", "->Starting the calibration. Don't move the sensor during the calibration.\n");
  for (i = 0; i < 20; i++) {
    if (_update_imu_data() != 1) return false;
    offset_gyro[0] += gyro_event.gyro.x;
    offset_gyro[1] += gyro_event.gyro.y;
    offset_gyro[2] += gyro_event.gyro.z;
    delay(50);
  }
  for (i = 0; i < 3; i++) {
    offset_gyro[i] /= 20;
  }
  /* Get a new accel event */
  if (_update_imu_data() != 1) return false;
  debug_print2("imu.h::kalman::calibrate_imu()->Calibration done.", "\n");

  delay(500);
  return true;
}

bool begin_imu() {
  orientation.heading = 0;
  return _begin_imu() ? (calibrate_imu() ? (!(kalman.begin(&acc_event, offset_gyro) < 0) ? true : debug_print(F("imu.h::kalman::kalman.begin()->accel_event is NULL... Check your accel_event structure!")) && false) : debug_print2("imu.h::kalman::calibrate_imu() failed", "\n") && false) : debug_print4("imu.h::kalman::begin_imu()::_begin_imu() failed", " ->code: ", imu_status, "\n") && false;
}

bool get_orientation() {
  if (_update_imu_data() != 1) return false;
  orientation.heading += (gyro_event.gyro.z - offset_gyro[2]) * dt * RAD_TO_DEG * 0.001;
  //debug_print8("gyro z: ", gyro_event.gyro.z, " off z: ", offset_gyro[2], " dt: ", dt, "\n", "\n");
  kalman.update(&acc_event, &gyro_event, &orientation);
  //quaternions = _to_quaternions(orientation);
  _to_quaternions(orientation, DEG_TO_RAD, &quaternions);
  return true;
}
#endif

#ifdef USE_MADGWICK_IMU
Madgwick madgwick_lib;  // for using Madgwick library


void configure_madgwick_lib(void) {
  madgwick_lib.begin(SAMPLING_FREQUENCY);
  // edited MadgwickAHRS.cpp to allow gain (beta) to be set from sketch
  madgwick_lib.set_beta(0.1f);  // set filter gain
}

bool begin_imu() {
  configure_madgwick_lib();
  return _begin_imu() ? true : debug_print4("imu.h::madgwick::begin_imu()::_begin_imu() failed", " ->code: ", imu_status, "\n") && false;
}

bool get_orientation() {
  if (!_update_imu_data()) return false;
  // edited MadgwickAHRS.cpp to stop it converting from deg to rad
  correct_a();
  madgwick_lib.updateIMU(-gyro_event.gyro.x, -gyro_event.gyro.y, -gyro_event.gyro.z,
                         -acc_event.acceleration.x, -acc_event.acceleration.y, -acc_event.acceleration.z,
                         quaternions.q0, quaternions.q1, quaternions.q2, quaternions.q3);
  quaternions.q0 = -quaternions.q0;
  _to_euler(quaternions, RAD_TO_DEG, &orientation);
  return true;
}
#endif
