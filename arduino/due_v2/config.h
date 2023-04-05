

/*****************************************************************************************
**************************************GENERAL CONFIG**************************************
*****************************************************************************************/
#define DEBUG_MODE
#define SERIAL_OBJECT Serial
#define RADIO_SERIAL Serial
#define NO_MOTOR_RPS_IN_SS
#define BUZZ_PIN 32
#define HALT while (true)

/*****************************************************************************************
****************************************IMU CONFIG****************************************
*****************************************************************************************/
#define IMU_ADDRESS 0x68
#define SAMPLING_FREQUENCY 125  // rate for reading imu
#define SAMPLING_TIME_MS 1000.0/SAMPLING_FREQUENCY

#define DEGREES  // get results in degrees
//#define RADIANS
#define USE_MADGWICK_IMU  //to use Madgwick filter to directly obtain quaternions from IMU
//#define USE_KALMAN_IMU  //to use kalman to obtain Pitch and Roll angles from IMU


/*****************************************************************************************
***************************************RADIO CONFIG***************************************
*****************************************************************************************/
// define this to off-load all necessary calculations to raspberry-pi
// this enables raspi to send set-point yaw,pitch,roll,thrust,switches data directly to arduino
#define OFFLOAD_CALC_TO_RPI  // currently no code depends on this macro, defined for later code extension

#define RECEIVER_YAW_MIN 300.0
#define RECEIVER_YAW_MAX 1700.0
#define RECEIVER_PITCH_MIN 300.0
#define RECEIVER_PITCH_MAX 1700.0
#define RECEIVER_ROLL_MIN 300.0
#define RECEIVER_ROLL_MAX 1700.0
#define RECEIVER_THROTTLE_MIN 300.0
#define RECEIVER_THROTTLE_MAX 1700.0

#define COPTER_YAW_LIMIT 180
#define COPTER_PITCH_LIMIT 10
#define COPTER_ROLL_LIMIT 10
#define COPTER_THROTTLE_MAX 700.0

#define RECEIVER_MIN 300f                  // minimum pwm input from receiver channel
#define RECEIVER_MAX 1700f                 // maximum pwm input from receiver channel
#define THROTTLE_MAX 1700f                 // (max 1850) max throttle to allow stability control at full throttle
#define MAX_COPTER_ANGLE 10f * DEG_TO_RAD  // maximum angle the quadcopter can tilt from upright

#define REF_DRONE_ANG_VEL_X 0
#define REF_DRONE_ANG_VEL_Y 0
#define REF_DRONE_ANG_VEL_Z 0




/*****************************************************************************************
***************************************MOTORS CONFIG**************************************
*****************************************************************************************/
#define FRONT_LEFT_ESC_PIN 2
#define FRONT_RIGHT_ESC_PIN 3
#define BACK_LEFT_ESC_PIN 4
#define BACK_RIGHT_ESC_PIN 5

#define ARM_ROTOR_SPEED 900
#define ZERO_ROTOR_SPEED 1000  // 1000
#define IDLE_ROTOR_SPEED 1130  // 1130
#define ABSOLUTE_MIN_PWM 800   // 800
#define ABSOLUTE_MAX_PWM 2000  // 2000
#define U_TO_PWM 10
