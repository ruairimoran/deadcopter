// 2021-01-07 15:49:37.113125

// q means quaternion(q0, q1, q2, q3)
// DueTimer Timers 0,2,3,4,5 unavailable due to use of Servo library
#include <Arduino.h>
#include <DueTimer.h>
#include <MPU9250.h>
#include <MadgwickAHRS.h>
#include <Servo.h>
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

    void read_channels(void) {
        throttle = output_rx[3];
        roll = output_rx[1];
        pitch = output_rx[2];
        yaw = output_rx[4];
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
    void observe_and_control(int throttle, int &front_left, int &front_right, int &back_left, int &back_right) {
        // some pure maths, no libraries used, takes ~600us to complete
        delayMicroseconds(600);
        front_left = throttle;
        front_right = throttle;
        back_left = throttle;
        back_right = throttle;
    }
    
    bool flag_flight_control = false;
    
    void ISR_flight_control(void) {
        flag_flight_control = true;
    }

//----------------------------------------------------------------------------------------------------------------------
// Electronic Speed Controllers to motors (esc)
    #define FRONT_LEFT_ESC_PIN 2
    #define FRONT_RIGHT_ESC_PIN 3
    #define BACK_LEFT_ESC_PIN 4
    #define BACK_RIGHT_ESC_PIN 5
    #define ZERO_ROTOR_SPEED 1000  // 1000
    #define IDLE_ROTOR_SPEED 1150  // 1150
    #define ABSOLUTE_MIN_PWM 1000  // 1000
    #define ABSOLUTE_MAX_PWM 2000  // 2000

    int front_left = 0;  // front left motor pwm
    int front_right = 0;  // front right motor pwm
    int back_left = 0;  // back left motor pwm
    int back_right = 0;  // back right motor pwm

    Servo esc_front_left;  // create servo object to control an ESC
    Servo esc_front_right;
    Servo esc_back_left;
    Servo esc_back_right;

    void attach_esc_to_pwm_pin(void) {
        esc_front_left.attach(FRONT_LEFT_ESC_PIN, ABSOLUTE_MIN_PWM, ABSOLUTE_MAX_PWM);  // (PWN pin, absolute min (1000), absolute max (2000))
        esc_front_right.attach(FRONT_RIGHT_ESC_PIN, ABSOLUTE_MIN_PWM, ABSOLUTE_MAX_PWM);
        esc_back_left.attach(BACK_LEFT_ESC_PIN, ABSOLUTE_MIN_PWM, ABSOLUTE_MAX_PWM);
        esc_back_right.attach(BACK_RIGHT_ESC_PIN, ABSOLUTE_MIN_PWM, ABSOLUTE_MAX_PWM);
    }

    void write_speed_to_esc(int front_left, int front_right, int back_left, int back_right) {
        // send speed to ESCs
        esc_front_left.writeMicroseconds(front_left);
        esc_front_right.writeMicroseconds(front_right);
        esc_back_left.writeMicroseconds(back_left);
        esc_back_right.writeMicroseconds(back_right);
    }

//----------------------------------------------------------------------------------------------------------------------    

// for serial timings
    unsigned long fc_time = 0;  // 500ms refresh rate
    unsigned long fc_new = 0;
    unsigned long fc_old = 0;
//    unsigned long millisPerSerialOutput = 500;  // 500ms refresh rate
//    unsigned long millisNow = 0;
//    unsigned long millisPrevious = 0;

//----------------------------------------------------------------------------------------------------------------------

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);  // setup LED output on pin 13 (default LED pin)
    configure_imu();  // start communication with imu
    configure_madgwick();  // start madgwick filter
    attach_esc_to_pwm_pin();  // attach motor esc pins
    attachInterrupt(digitalPinToInterrupt(RX_PIN), ISR_read_ppm, RISING);  // enable receiver ppm interrupt (fires 9 times in 20ms, every 20ms)
    Timer6.attachInterrupt(ISR_flight_control).setFrequency(SAMPLING_FREQUENCY).start();

//     // for serial output
//     Serial.begin(115200);
}

//----------------------------------------------------------------------------------------------------------------------

void loop() {
    // poll imu interrupt pin for data ready
    if(flag_flight_control == true) {
        fc_new = micros();
        fc_time = fc_new - fc_old;
        fc_old = fc_new;
        // get imu values
        update_imu_data(y_negative1, y_0, y_1, y_2, y_3, y_4, y_5);  // takes ~910us to complete
        // compute control actions for each motor
        observe_and_control(throttle, front_left, front_right, back_left, back_right); // _u0, _u1, _u2, q0y, _y0, _y1, _y2, _y3, _y4, _y5);
        // send control action to motor ESCs
        write_speed_to_esc(front_left, front_right, back_left, back_right);
        // reset interrupt flag
        flag_flight_control = false;
    }
  
    //------------------------------------------------------------------------------------------------------------------
    // wait for long (>4ms) break in PPM to run rest of loop (runs every ~20ms)
    if(channel > NO_OF_CHANNELS) {
        // update receiver values
        read_channels();  // takes ~6us to complete
        
//        // serial output
//        millisNow = millis();
//        if(millisNow - millisPrevious >= millisPerSerialOutput) {
//            Serial.print(fc_time);Serial.print("\t");Serial.print("\t");
//        
//            Serial.print(throttle);Serial.print("\t");
//            Serial.print(yaw);Serial.print("\t");
//            Serial.print(pitch);Serial.print("\t");
//            Serial.print(roll);Serial.print("\t");
//            Serial.print(aux_channel_1);Serial.print("\t");
//            Serial.print(aux_channel_2);Serial.print("\t");
//            Serial.print(aux_channel_3);Serial.print("\t");
//            Serial.print(aux_channel_4);Serial.print("\t");Serial.print("\t");
//
//            Serial.print(front_left);Serial.print("\t");
//            Serial.print(front_right);Serial.print("\t");
//            Serial.print(back_left);Serial.print("\t");
//            Serial.print(back_right);Serial.print("\n");
//        
//            millisPrevious += millisPerSerialOutput;
//        }
    }
}
