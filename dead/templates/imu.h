// {{timestamp}}

#ifndef imu.h
#define imu.h

#include <Arduino.h>
#include <MPU9250.h>
#include <math.h>

#define deltat 0.001  // for integration

class Imu {
    private:
    // Data storage variables
    static int status;  // imu status
    static int pin13_indicator;  // for indicating IMU status
    static float q0, q1, q2, q3, ax, ay, az, gx, gy, gz, mx, my, mz, temp;  // imu raw data variables

    // Arithmetic variables
    float norm;
    float hx, hy, _2bx, _2bz;
    float s0, s1, s2, s3;
    float qDot0, qDot1, qDot2, qDot3;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q0mx;
    float _2q0my;
    float _2q0mz;
    float _2q1mx;
    float _4bx;
    float _4bz;
    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q0q2 = 2.0f * q0 * q2;
    float _2q2q3 = 2.0f * q2 * q3;
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
    configure_imu();

    public:
    float op_q0, op_q1, op_q2, op_q3, omega_x, omega_y, omega_z;  // output variables
    Imu();
    calibrate_imu();
    update_imu_data();
}

/*--------------------------------------------------------------------------------------------------------------------*/

Imu::Imu() {
    pin13_indicator = PinMode(13, OUTPUT);
    MPU9250 IMU(Wire,0x68);  // create class from libraries
    configure_imu();
    // calibrate_imu();
    update_imu_data();
}

void Imu::configure_imu(void) {
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

void Imu::calibrate_imu(void) {
    IMU.calibrateMag();
    IMU.calibrateAccel();
    IMU.calibrateGyro();
}

void Imu::update_imu_data(void) {
    pin13_indicator = status;

    // read IMU and store in buffer
    IMU.readSensor();

    // get data from buffer in m.s^-2 and degs.s^-1 and micro Tesla
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

    // Normalise accelerometer measurement
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    norm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);    // normalise step magnitude
    norm = 1.0f/norm;
    s0 *= norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;

    // Compute rate of change of quaternion
    qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0;
    qDot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - beta * s1;
    qDot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - beta * s2;
    qDot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - beta * s3;

    // Integrate to yield quaternion
    q0 += qDot0 * deltat;
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);    // normalise quaternion
    norm = 1.0f/norm;
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;

    // Quaternion
    op_q0 = q0;
    op_q1 = q1;
    op_q2 = q2;
    op_q3 = q3;

    // Accel values
    omega_x = ax;
    omega_y = ay;
    omega_z = az;

    pin13_indicator = LOW;
}

#endif
