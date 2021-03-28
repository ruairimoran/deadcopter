#include <MPU9250.h>
#include <MadgwickAHRS.h>
#include <math.h>

MPU9250 IMU(Wire,0x68);
Madgwick filter;

int status;
unsigned long microsPerReading, microsPerDisplay, microsPreviousRead, microsPreviousDisp;
float accelScale, gyroScale;
float q0, q1, q2, q3;

void start_serial() {
  Serial.begin(115200);
  while(!Serial) {}
}

void start_imu_communication() {
  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
}

void configure_imu() {
  /// setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G); // GOTO IMU readme for possible ranges
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS); // GOTO IMU readme for possible ranges
  // setting DLPF bandwidth to 184 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
  // Data Output Rate = 1000 / (1 + SRD)
  // setting SRD to 0 for a 1000Hz update rate
  // mag fixed at 100Hz for SRD<=9, 8Hz for SRD>9
  IMU.setSrd(7); // gyro/accel/temp = 1000Hz, mag = 100Hz
}

void configure_filter() {
  float frequency = 125;
  filter.begin(frequency);
  filter.set_beta(10.0f);
  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / frequency;
  microsPerDisplay = 1000000 / 100;
  microsPreviousRead = micros();
  microsPreviousDisp = micros();
}

void calibrate_imu() {
  Serial.println("Calibrating accelerometers...");
  IMU.calibrateAccel();
  Serial.println("Calibrating gyroscope...");
  IMU.calibrateGyro();
//  Serial.println("Calibrating magnetometer...");
//  IMU.calibrateMag();
  Serial.println("Done!");
}

void setup() {
  start_serial();
  start_imu_communication();
  configure_imu();
  configure_filter();
  calibrate_imu();
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float temp;
  float roll, pitch, heading;
  unsigned long microsNow;

  // check if it's time to read data and update the filter
  microsNow = micros();
  if (microsNow - microsPreviousRead >= microsPerReading) {

    // read IMU and store in buffer
    IMU.readSensor();

    // get data from buffer in m.s^-2 and degs.s^-1 and micro T
    ax = IMU.getAccelX_mss();
    ay = IMU.getAccelY_mss();
    az = IMU.getAccelZ_mss();
    gx = IMU.getGyroX_rads();
    gy = IMU.getGyroY_rads();
    gz = IMU.getGyroZ_rads();
//    mx = IMU.getMagX_uT();
//    my = IMU.getMagY_uT();
//    mz = IMU.getMagZ_uT();
//    temp = IMU.getTemperature_C();
    
    // update the filter, which computes orientation
    filter.updateIMU(-gx, gy, gz, -ax, ay, az, q0, q1, q2, q3);

    microsPreviousRead = microsPreviousRead + microsPerReading;
  }

  if (microsNow - microsPreviousDisp >= microsPerDisplay) {
    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
//    heading = filter.getYaw();
    Serial.print("Orientation: ");
    Serial.print(roll);
    Serial.print(" ");
    Serial.println(pitch);

    // increment previous time, so we keep proper pace
    microsPreviousDisp = microsPreviousDisp + microsPerDisplay;
  }
}
