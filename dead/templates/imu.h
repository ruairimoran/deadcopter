// {{timestamp}}

#ifndef imu.h
#define imu.h

#include <Arduino.h>
#include <MPU9250.h>
#include <MadgwickAHRS.h>
#include <math.h>

class Imu {
    private:
    static int status;
    static unsigned long microsPerReading, microsNow, microsPrevious;
    static float ax, ay, az, gx, gy, gz, mx, my, mz;
    static float temp;
    static float q0, q1, q2, q3;
    configure_imu();

    public:
    Imu();
    calibrate_imu();
}

/*--------------------------------------------------------------------------------------------------------------------*/

Imu::Imu() {
    MPU9250 IMU(Wire,0x68);  // create class from libraries
    configure_imu();
    // calibrate_imu();
}

#endif

void configure_imu(void) {
    // start communication with imu
    status = IMU.begin();
    // setting the accelerometer full scale range to +/-8G
    IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G); // GOTO IMU readme for possible ranges
    // setting the gyroscope full scale range to +/-500 deg/s
    IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS); // GOTO IMU readme for possible ranges
    // setting DLPF bandwidth to 184 Hz
    IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_184HZ);
    // Data Output Rate = 1000 / (1 + SRD)
    // setting SRD to 0 for a 1000Hz update rate
    // mag fixed at 100Hz for SRD<=9, 8Hz for SRD>9
    IMU.setSrd(0); // gyro/accel/temp = 1000Hz, mag = 100Hz
}

void calibrate_imu(void) {
  IMU.calibrateMag();
  IMU.calibrateAccel();
  IMU.calibrateGyro();
}

void get_raw_imu_data(void) {

  // check if it's time to read data and update the filter
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {

    // read IMU and store in buffer
    IMU.readSensor();

    // get data from buffer in m.s^-2 and degs.s^-1 and micro T
    ax = IMU.getAccelX_mss();
    ay = IMU.getAccelY_mss();
    az = IMU.getAccelZ_mss();
    gx = IMU.getGyroX_rads();
    gy = IMU.getGyroY_rads();
    gz = IMU.getGyroZ_rads();
    mx = IMU.getMagX_uT();
    my = IMU.getMagY_uT();
    mz = IMU.getMagZ_uT();
    temp = IMU.getTemperature_C();

    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }
}