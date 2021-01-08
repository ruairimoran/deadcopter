// 2021-01-07 15:49:37.113125

// q means quaternion(q0, q1, q2, q3)

#include <Arduino.h>
#include <DueTimer.h>
#include <MPU9250.h>
#include <MadgwickAHRS.h>
#include <math.h>

//----------------------------------------------------------------------------------------------------------------------

// Receiver (rx)
    #define RX_PIN 7  // input pin for wire from receiver
    #define NO_OF_CHANNELS 8 // number of receiver channels
    #define RECEIVER_MIN 1070  // minimum pwm input from receiver channel
    #define RECEIVER_MAX 1930  // maximum pwm input from receiver channel
    #define THROTTLE_MIN 1150  // minimum throttle input
    #define THROTTLE_MAX 1850  // maximum throttle input
    #define ABSOLUTE_MAX_COPTER_ANGLE 30  // maximum angle the copter should rotate on any axis
    
    unsigned long int current_time = 0;  // for calculating pulse separation time
    unsigned long int previous_time = 0;
    unsigned long int time_difference = 0;
    int channel = 0;
    int output_rx[NO_OF_CHANNELS+1] = {RECEIVER_MIN};  // store receiver channel values starting from channel 1 (not using 0)
    int throttle = 0;  // receiver throttle value
    int roll = 0;  // receiver roll value
    int pitch = 0;  // receiver pitch value
    int yaw = 0;  // receiver yaw value
    int aux_channel_1 = 0;
    int aux_channel_2 = 0;
    int aux_channel_3 = 0;
    int aux_channel_4 = 0;

    void ISR_read_ppm(void) {  // takes ~5us to complete
        current_time = micros();  // store current time in microseconds
        time_difference = current_time - previous_time;
        previous_time = current_time;  // update previous_time for next interrupt call
        if(time_difference>5000) {  // if time difference between pulses is >5000us, this indicates the start of a PPM frame (which are 20ms)
            channel = 1;  // therefore the next pulse time read will be channel 1
        } 
        else {
            if((900<time_difference) && (time_difference<2100) && (channel<NO_OF_CHANNELS+1)) {  // PPM signals only valid between 900 and 2100us
                output_rx[channel] = time_difference;  // Set channel value
            }
            channel += 1;  // Next interrupt will calculate next channel
        }
    }

    void read_channels(int &rx_throttle, int &rx_roll, int &rx_pitch, int &rx_yaw) {
        // reformat receiver values
        rx_throttle = map(output_rx[3], RECEIVER_MIN, RECEIVER_MAX, THROTTLE_MIN, THROTTLE_MAX);
        rx_roll = map(output_rx[1], RECEIVER_MIN, RECEIVER_MAX, -ABSOLUTE_MAX_COPTER_ANGLE, ABSOLUTE_MAX_COPTER_ANGLE);
        rx_pitch = map(output_rx[2], RECEIVER_MIN, RECEIVER_MAX, -ABSOLUTE_MAX_COPTER_ANGLE, ABSOLUTE_MAX_COPTER_ANGLE);
        rx_yaw = map(output_rx[4], RECEIVER_MIN, RECEIVER_MAX, -ABSOLUTE_MAX_COPTER_ANGLE, ABSOLUTE_MAX_COPTER_ANGLE);
        aux_channel_1 = output_rx[5];
        aux_channel_2 = output_rx[6];
        aux_channel_3 = output_rx[7];
        aux_channel_4 = output_rx[8];
    }

//----------------------------------------------------------------------------------------------------------------------

// Inertial measurement unit (imu)
    #define SAMPLING_FREQUENCY 125  // rate for reading imu
    #define IMU_INTERRUPT_PIN 6  // interrupt pin to signal when imu data ready

    int imu_status = -1;  // imu status, starts with communication error
    int led_flash = 0;  // LED indicating imu communication
    float ax = 0;  // accelerometer x
    float ay = 0;  // accelerometer y
    float az = 0;  // accelerometer z
    float gx = 0;  // gyro x
    float gy = 0;  // gyro y
    float gz = 0;  // gyro z
    float mx = 0;  // magnetometer x
    float my = 0;  // magnetometer y
    float mz = 0;  // magnetometer z
    float y_negative1 = 0;  // imu q0 (q = quaternion)
    float y_0 = 0;  // imu q1
    float y_1 = 0;  // imu q2
    float y_2 = 0;  // imu q3
    float y_3 = 0;  // imu angular acceleration x
    float y_4 = 0;  // imu angular acceleration y
    float y_5 = 0;  // imu angular acceleration z
    
    MPU9250 imu_lib(Wire,0x68);  // for using MPU library
    Madgwick madgwick_lib;  // for using Madgwick library

    void configure_imu(void) {
        while(imu_status <= 0) {  // until imu begins communicating
            // start communication with imu
            imu_status = imu_lib.begin();
            digitalWrite(LED_BUILTIN, HIGH);
        }
        // setting DLPF bandwidth to 41 Hz
        imu_lib.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
        // Data Output Rate = 1000 / (1 + SRD)
        // output rate should be > double Dlpf
        // setting SRD to 7 for a 125Hz output rate
        // mag fixed at 100Hz for SRD<=9, 8Hz for SRD>9
        imu_lib.setSrd(7); // gyro/accel/temp = 125Hz, mag = 100Hz
    }

    void configure_madgwick(void) {
        madgwick_lib.begin(SAMPLING_FREQUENCY);  // start filtering
        // edited MadgwickAHRS.cpp to allow gain (beta) to be set from sketch
        madgwick_lib.set_beta(1.0f);  // set filter gain
    }

    void update_imu_data(float &imu_y_negative1, float &imu_y_0, float &imu_y_1, float &imu_y_2,
                           float &imu_y_3, float &imu_y_4, float &imu_y_5) {
        // led flashing to show update is being run
        // should toggle once per second (imu read at 125Hz)
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
        mx = imu_lib.getMagX_uT();
        my = imu_lib.getMagY_uT();
        mz = imu_lib.getMagZ_uT();
        // edited MadgwickAHRS.cpp to stop it converting gyro from deg to rad
        madgwick_lib.update(gx, gy, gz, ax, ay, az, mx, my, mz, imu_y_negative1, imu_y_0, imu_y_1, imu_y_2);
        imu_y_3 = gx;
        imu_y_4 = gy;
        imu_y_5 = gz;
    }

//----------------------------------------------------------------------------------------------------------------------

// Flight Control (fly)
    bool flag_flight_control = false;
    
    void ISR_flight_control(void) {
        flag_flight_control = true;
    }

//----------------------------------------------------------------------------------------------------------------------

// for serial timings
    unsigned long fc_time = 0;  // 500ms refresh rate
    unsigned long fc_new = 0;
    unsigned long fc_old = 0;
    unsigned long millisPerSerialOutput = 500;  // 500ms refresh rate
    unsigned long millisNow = 0;
    unsigned long millisPrevious = 0;

//----------------------------------------------------------------------------------------------------------------------

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);  // setup LED output on pin 13 (default LED pin)
    configure_imu();  // start communication with imu
    configure_madgwick();  // start madgwick filter
    attachInterrupt(digitalPinToInterrupt(RX_PIN), ISR_read_ppm, RISING);  // enable receiver ppm interrupt (fires 9 times in 20ms, every 20ms)
    Timer6.attachInterrupt(ISR_flight_control).setFrequency(SAMPLING_FREQUENCY).start();

     // for serial output
     Serial.begin(115200);
}

void loop() {
//----------------------------------------------------------------------------------------------------------------------
    // poll imu interrupt pin for data ready
    if(flag_flight_control == true) {
        fc_new = micros();
        fc_time = fc_new - fc_old;
        fc_old = fc_new;
        // get imu values
        update_imu_data(y_negative1, y_0, y_1, y_2, y_3, y_4, y_5);  // takes ~910us to complete
        // reset interrupt flag
        flag_flight_control = false;
    }
  
//----------------------------------------------------------------------------------------------------------------------
    // wait for break in PPM to run rest of loop (runs every ~20ms)
    if(channel > 8) {
        // detach ppm interrupt
        detachInterrupt(digitalPinToInterrupt(RX_PIN));

        // get receiver values
        read_channels(throttle, roll, pitch, yaw);  // takes ~6us to complete
        
        // serial output
        millisNow = millis();
        if(millisNow - millisPrevious >= millisPerSerialOutput) {
            Serial.print(fc_time);Serial.print("\t");Serial.print("\t");
        
            Serial.print(throttle);Serial.print("\t");
            Serial.print(yaw);Serial.print("\t");
            Serial.print(pitch);Serial.print("\t");
            Serial.print(roll);Serial.print("\n");
        
            millisPrevious += millisPerSerialOutput;
        }

//        // attach ppm interrupt
        attachInterrupt(digitalPinToInterrupt(RX_PIN), ISR_read_ppm, RISING);  // enable receiver ppm interrupt (fires 9 times in 20ms, every 20ms)
    }
}
