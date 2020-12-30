// 2020-12-30 00:07:59.103692

#ifndef imu.h
#define imu.h

#include <Arduino.h>
#include <MPU9250.h>
#include <MadgwickAHRS.h>
#include <math.h>

class Imu {
    private:
    MPU9250 imu_lib;  // for using MPU library
    Madgwick madgwick_lib;  // for using Madgwick library
    int imu_status;  // imu status, starts with communication as False
    int led_flash;  // LED indicating imu communication
    float ax, ay, az, gx, gy, gz, mx, my, mz, temp;  // imu raw data variables
    void configure_imu(void);
    void configure_madgwick_lib(void);

    public:
    Imu();
    void configure_imu_and_madgwick(void);
    void calibrate_imu(void);
    float update_imu_data(float &imu_y_negative1, float &imu_y_0, float &imu_y_1, float &imu_y_2, float &imu_y_3, float &imu_y_4, float &imu_y_5);
};

/*--------------------------------------------------------------------------------------------------------------------*/

Imu::Imu() : imu_lib{Wire,0x68}, imu_status{-1}, led_flash{0} {
    pinMode(LED_BUILTIN, OUTPUT);  // setup LED output on pin 13 (default LED pin)
    digitalWrite(LED_BUILTIN, LOW);
}

void Imu::configure_imu_and_madgwick(void) {
    while(imu_status < 0) {
        digitalWrite(LED_BUILTIN, LOW);
        configure_madgwick_lib();
        configure_imu();
    }
}

void Imu::configure_madgwick_lib(void) {
    float freq = 100.0f;  // for 10ms rate
    madgwick_lib.begin(freq);
    madgwick_lib.set_beta(1.0f);  // set madgwick filter gain
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

float Imu::update_imu_data(float &imu_y_negative1, float &imu_y_0, float &imu_y_1, float &imu_y_2, float &imu_y_3, float &imu_y_4, float &imu_y_5) {
    // led flashing to show update is being run
    // high speed flashing means imu not being read
    // normal speed flashing means imu is being read
    led_flash += 1;
    if(led_flash == 50) {
        digitalWrite(LED_BUILTIN, HIGH);
    }
    if(led_flash == 100) {
        digitalWrite(LED_BUILTIN, LOW);
        led_flash = 0;
    }
    if(led_flash > 100 || led_flash < 0) {
        led_flash = 0;
    }

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

    madgwick_lib.update(gx, gy, gz, ax, ay, az, mx, my, mz, imu_y_negative1, imu_y_0, imu_y_1, imu_y_2);
    imu_y_3 = ax;
    imu_y_4 = ay;
    imu_y_5 = az;
}

#endif