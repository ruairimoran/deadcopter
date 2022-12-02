#include <mpu9250.h>

//#include "MPU9250.h"
#include "MadgwickAHRS.h"
#include <math.h>

MPU9250 IMU(Wire,0x68);
Madgwick MadgwickAHRS;

int status;
static unsigned long microsPerReading, microsPrevious;
static float accelScale, gyroScale;
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
  float frequency = 400;
  //MadgwickAHRS.SetBeta(0.15f);
  //MadgwickAHRS.SetFrequency(frequency);  
  microsPerReading = 1000000 / (int) frequency;
  microsPrevious = micros();
}

void calibrate_imu() {
  //Serial.println("Calibrating magnetometer...");
  //IMU.calibrateMag();
  //Serial.println("Calibrating accelerometers...");
  //IMU.calibrateAccel();
  //Serial.println("Calibrating gyroscope...");
  //IMU.calibrateGyro();
  Serial.println("Done!");
}

void setup() {  
  start_serial();
  start_imu_communication();
  configure_imu();
  calibrate_imu();
}

int sample_id = 0;
int do_init = 1;
float v0 = 0.f, v1 = 0.f, v2 = 0.f, v3 = 0.f;
float qr0 = 0.f, qr1 = 0.f, qr2 = 0.f, qr3 = 0.f;
#define N_DISCARD_Q 800
#define N_INIT_Q 2000
// res = a (*) b
void quat_multiply(float a0, float a1, float a2, float a3,
                   float b0, float b1, float b2, float b3,
                   float *res0, float *res1, float *res2, float *res3) 
{
  *res0 = a0 * b0 - a1 * b1 - a2 * b2 - a3 * b3;
  *res1 = a1 * b0 + a0 * b1 - a3 * b2 + a2 * b3;
  *res2 = a2 * b0 + a3 * b1 + a0 * b2 - a1 * b3;
  *res3 = a3 * b0 - a2 * b1 + a1 * b2 + a0 * b3;
}
void quat_normalise(float *a0, float *a1, float *a2, float *a3)
{
  float norm_a = sq(*a0) + sq(*a1) + sq(*a2) + sq(*a3);
  norm_a = sqrt(norm_a);
  *a0 /= norm_a;
  *a1 /= norm_a;
  *a2 /= norm_a;
  *a3 /= norm_a;
}

void loop() {
  unsigned long microsNow; 
  float array MadgwickAHRSupdate[9]; 
  
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {    
    IMU.readSensor();
    MadgwickAHRSupdate = (IMU.getGyroX_rads(), IMU.getGyroY_rads(), IMU.getGyroZ_rads(),
                       IMU.getAccelX_mss(), IMU.getAccelY_mss() , IMU.getAccelZ_mss(),
                       IMU.getMagX_uT(), IMU.getMagY_uT(), IMU.getMagZ_uT());        
    sample_id ++;
    microsPrevious = microsPrevious + microsPerReading;
  }
  GetQuaternion(&q0, &q1, &q2, &q3);
  if (do_init == 1 && sample_id < N_DISCARD_Q) {
    // discard first N_DISCARD_Q samples
  } else if (do_init == 1 
             && sample_id >= N_DISCARD_Q 
             && sample_id < N_DISCARD_Q + N_INIT_Q) {
    v0 += q0; v1 += q1; v2 += q2; v3 += q3;
  } else if (sample_id == N_DISCARD_Q + N_INIT_Q) {
    do_init = 0;
    sample_id = 0;
    v0 /= N_INIT_Q;
    v1 /= N_INIT_Q;
    v2 /= N_INIT_Q;
    v3 /= N_INIT_Q;
    quat_normalise(&v0, &v1, &v2, &v3);
    Serial.print("q_init = ");
    Serial.print(v0, 5);
    Serial.print(",  ");
    Serial.print(v1, 5);
    Serial.print(",  ");
    Serial.print(v2, 5);
    Serial.print(",  ");
    Serial.println(v3, 5);
  }
  
  
  float w0, w1, w2, w3;
  quat_multiply(
    v0, -v1, -v2, -v3,   
    q0, q1, q2, q3,    
    &w0, &w1, &w2, &w3);
  quat_normalise(&w0, &w1, &w2, &w3);
  float a = 2. * (w1*w2  + w0*w3);
  float b = sq(w0) + sq(w1) - sq(w2) - sq(w3);
  float c = -2. * (w1*w3 - w0*w2);
  float d = 2. * (w2*w3  + w0*w1);
  float e = sq(w0) - sq(w1) - sq(w2) + sq(w3);
  
  double r1 = atan2(a, b) * 57296 / 1000;
  double r2 = asin(c) * 57296 / 1000;
  double r3 = atan2(d, e) * 57296 / 1000;

//  double pitch_deg = -asin(2.0f*(w1*w3 - w0*w2)) * 57296 / 1000;
//  double yaw_deg = atan2(2.0f * (w1 * w2 + w0 * w3), 
//                         w0 * w0 + w1 * w1 - w2 * w2 - w3 * w3) * 57296 / 1000;
//  double roll_deg = atan2(2.0f * (w0 * w1 + w2 * w3), 
//                          w0 * w0 - w1 * w1 - w2 * w2 + w3 * w3) * 57296 / 1000;
  char line[256];
  if (sample_id == 20 && do_init == 0) {  
//      Serial.print(r1, 2);
//      Serial.print(",  ");
//      Serial.print(r2, 2);
//      Serial.print(",  ");
//      Serial.print(r3, 2);
//      Serial.print(",  ");
      String mx_str = String(IMU.getMagX_uT());
      String my_str = String(IMU.getMagY_uT());
      String mz_str = String(IMU.getMagZ_uT());
      sprintf(line, "%s, %s, %s\n", 
              mx_str.c_str(),
              my_str.c_str(),
              mz_str.c_str());
      Serial.print(line);
      
      
      sample_id = 0;
  }
  
}
