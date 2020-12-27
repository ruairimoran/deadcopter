// {{timestamp}}

#ifndef imu.h
#define imu.h

#include <Arduino.h>
#include <MPU9250.h>
#include <MadgwickAHRS.h>
#include <math.h>

class Imu {
    private:
    MPU9250 imu_lib{Wire, 0x68};  // for using MPU library
    Madgwick madgwick_lib;  // for using Madgwick library
    int imu_status;  // imu status
    float ax, ay, az, gx, gy, gz, mx, my, mz, temp;  // imu raw data variables
    float q0, q1, q2, q3, omega_x, omega_y, omega_z;  // madgwick filtered variables
    void configure_imu(void);
    void configure_madgwick_lib(void);

    public:
    Imu();
    void calibrate_imu(void);
    float update_imu_data(void);
};

/*--------------------------------------------------------------------------------------------------------------------*/

Imu::Imu() {
    configure_imu();
    configure_madgwick_lib();
    // calibrate_imu();
    update_imu_data();
}

void Imu::configure_madgwick_lib(void) {
    madgwick_lib.begin(5000.0f);  // 5x imu gyro/accel/temp rate
    madgwick_lib.set_beta(0.1f);  // set madgwick filter gain
}

void Imu::configure_imu(void) {
    // start communication with imu
    imu_status = imu_lib.begin();
    // setting the accelerometer full scale range to +/-8G
    imu_lib.setAccelRange(MPU9250::ACCEL_RANGE_8G); // GOTO imu_lib readme for possible ranges
    // setting the gyroscope full scale range to +/-500 deg/s
    imu_lib.setGyroRange(MPU9250::GYRO_RANGE_500DPS); // GOTO imu_lib readme for possible ranges
    // setting DLPF bandwidth to 184 Hz
    imu_lib.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_184HZ);
    // Data Output Rate = 1000 / (1 + SRD)
    // setting SRD to 0 for a 1000Hz update rate
    // mag fixed at 100Hz for SRD<=9, 8Hz for SRD>9
    imu_lib.setSrd(0); // gyro/accel/temp = 1000Hz, mag = 100Hz
}

void Imu::calibrate_imu(void) {
    imu_lib.calibrateMag();
    imu_lib.calibrateAccel();
    imu_lib.calibrateGyro();
}

float Imu::update_imu_data(void) {
    // read imu_lib and store in buffer
    imu_lib.readSensor();

    // get data from buffer in m.s^-2 and degs.s^-1 and micro Tesla
    ax = imu_lib.getAccelX_mss();
    ay = imu_lib.getAccelY_mss();
    az = imu_lib.getAccelZ_mss();
    gx = imu_lib.getGyroX_rads();
    gy = imu_lib.getGyroY_rads();
    gz = imu_lib.getGyroZ_rads();
    mx = imu_lib.getMagX_uT();
    my = imu_lib.getMagY_uT();
    mz = imu_lib.getMagZ_uT();
    temp = imu_lib.getTemperature_C();

    madgwick_lib.update(gx, gy, gz, ax, ay, az, mx, my, mz);
    q0, q1, q2, q3 = madgwick_lib.get_quaternion();
    omega_x = ax;
    omega_y = ay;
    omega_z = az;

    return q1, q2, q3, omega_x, omega_y, omega_z;
}

#endif
