// 2021-03-27 14:39:23.118218

#ifndef imu.h
#define imu.h

#include <Arduino.h>
#include <MPU9250.h>
#include <MadgwickAHRS.h>
#include <math.h>

#define SAMPLING_FREQUENCY 125  // rate for reading imu

class Imu {
    private:
    MPU9250 imu_lib;  // for using MPU library
    Madgwick madgwick_lib;  // for using Madgwick library
    int imu_status = -1;  // imu status, starts with communication error
    int led_flash = 0;  // LED indicating imu communication
    float ax = 0;  // accelerometer x
    float ay = 0;  // accelerometer y
    float az = 0;  // accelerometer z
    float gx = 0;  // gyro x
    float gy = 0;  // gyro y
    float gz = 0;  // gyro z
    // float mx = 0;  // magnetometer x
    // float my = 0;  // magnetometer y
    // float mz = 0;  // magnetometer z
    // float temp = 0;  // imu temperature
    void configure_imu(void);
    void configure_madgwick_lib(void);

    public:
    Imu();
    void configure_imu_and_madgwick(void);
    void calibrate_imu(void);
    void update_imu_data(float &imu_y_negative1, float &imu_y_0, float &imu_y_1, float &imu_y_2,
                          float &imu_y_3, float &imu_y_4, float &imu_y_5);
};

/*--------------------------------------------------------------------------------------------------------------------*/

Imu::Imu() : imu_lib{Wire,0x68} {
    pinMode(LED_BUILTIN, OUTPUT);  // setup LED output on pin 13 (default LED pin)
    digitalWrite(LED_BUILTIN, LOW);
}

void Imu::configure_imu_and_madgwick(void) {
    while(imu_status < 0) {  // until imu starts communicating
        configure_madgwick_lib();
        configure_imu();
        calibrate_imu();
        digitalWrite(LED_BUILTIN, HIGH);  // LED will remain solid on if imu not communicating
    }
}

void Imu::configure_madgwick_lib(void) {
    madgwick_lib.begin(SAMPLING_FREQUENCY);
    // edited MadgwickAHRS.cpp to allow gain (beta) to be set from sketch
    madgwick_lib.set_beta(5.0f);  // set filter gain
}

void Imu::configure_imu(void) {
    // start communication with imu
    imu_status = imu_lib.begin();
    // setting the accelerometer full scale range to +/-8G
    imu_lib.setAccelRange(MPU9250::ACCEL_RANGE_8G); // GOTO imu_lib readme for possible ranges
    // setting the gyroscope full scale range to +/-1000 deg/s
    imu_lib.setGyroRange(MPU9250::GYRO_RANGE_1000DPS); // GOTO imu_lib readme for possible ranges
    // setting DLPF bandwidth to 92 Hz
    imu_lib.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
    // Data Output Rate = 1000 / (1 + SRD)
    // output rate should be > double Dlpf
    // setting SRD to 7 for a 125Hz output rate
    // mag fixed at 100Hz for SRD<=9, 8Hz for SRD>9
    imu_lib.setSrd(7); // gyro/accel/temp = 125Hz, mag = 100Hz
//    imu_lib.setAccelCalX(bias, scaleFactor);  // set sensor calibrations from "imu_calibration.ino" sketch
//    imu_lib.setAccelCalY(bias, scaleFactor);
//    imu_lib.setAccelCalZ(bias, scaleFactor);
//    imu_lib.setMagCalX(bias, scaleFactor);
//    imu_lib.setMagCalY(bias, scaleFactor);
//    imu_lib.setMagCalZ(bias, scaleFactor);
}

void Imu::calibrate_imu(void) {
//    imu_lib.calibrateMag();
    imu_lib.calibrateAccel();
    imu_lib.calibrateGyro();
}

void Imu::update_imu_data(float &imu_y_negative1, float &imu_y_0, float &imu_y_1, float &imu_y_2,
                           float &imu_y_3, float &imu_y_4, float &imu_y_5) {
    // led flashing to show update is being run
    // should toggle once per second
    led_flash += 1;
    if(led_flash == 125) {
        digitalWrite(LED_BUILTIN, HIGH);
    }
    if(led_flash == 250) {
        digitalWrite(LED_BUILTIN, LOW);
        led_flash = 0;
    }
    if(led_flash > 250 || led_flash < 0) {
        led_flash = 0;
    }

    // read imu_lib and store in buffer
    imu_lib.readSensor();

    // get data from buffer in m.s^-2 and rads.s^-1 and micro Tesla
    ax = imu_lib.getAccelX_mss();
    ay = imu_lib.getAccelY_mss();
    az = imu_lib.getAccelZ_mss();
    gx = imu_lib.getGyroX_rads();
    gy = imu_lib.getGyroY_rads();
    gz = imu_lib.getGyroZ_rads();
    // mx = imu_lib.getMagX_uT();
    // my = imu_lib.getMagY_uT();
    // mz = imu_lib.getMagZ_uT();
    // temp = imu_lib.getTemperature_C();

    // edited MadgwickAHRS.cpp to stop it converting g from deg to rad
    madgwick_lib.updateIMU(-gx, gy, gz, -ax, ay, az, imu_y_negative1, imu_y_0, imu_y_1, imu_y_2);
    imu_y_3 = -gx;
    imu_y_4 = gy;
    imu_y_5 = gz;
}

#endif
