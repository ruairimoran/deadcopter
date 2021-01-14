#include "MPU9250.h"

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;

void setup() {
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_1000DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(7);

  Serial.println("Calibrating gyro...");
  IMU.calibrateGyro();
  Serial.println("Calibrating accelerometers...");
  IMU.calibrateAccel();
  Serial.println("Calibrating magnetometer...");
  IMU.calibrateMag();
  Serial.println("Done!");

  Serial.print("Acc Bias X = ");Serial.println(IMU.getAccelBiasX_mss(), 10);
  Serial.print("Acc Bias Y = ");Serial.println(IMU.getAccelBiasY_mss(), 10);
  Serial.print("Acc Bias Z = ");Serial.println(IMU.getAccelBiasZ_mss(), 10);
  Serial.print("Acc SF X = ");Serial.println(IMU.getAccelScaleFactorX(), 10);
  Serial.print("Acc SF Y = ");Serial.println(IMU.getAccelScaleFactorY(), 10);
  Serial.print("Acc SF Z = ");Serial.println(IMU.getAccelScaleFactorZ(), 10);

  Serial.print("Mag Bias X = ");Serial.println(IMU.getMagBiasX_uT(), 10);
  Serial.print("Mag Bias Y = ");Serial.println(IMU.getMagBiasY_uT(), 10);
  Serial.print("Mag Bias Z = ");Serial.println(IMU.getMagBiasZ_uT(), 10);
  Serial.print("Mag SF X = ");Serial.println(IMU.getMagScaleFactorX(), 10);
  Serial.print("Mag SF Y = ");Serial.println(IMU.getMagScaleFactorY(), 10);
  Serial.print("Mag SF Z = ");Serial.println(IMU.getMagScaleFactorZ(), 10);
}

void loop() {
  while(1) {
    // nahin
  }
}
