// {{timestamp}}

#ifndef actuators.h
#define actuators.h

#include <Arduino.h>
#include <Servo.h>  // must map range onto 0 - 180 degrees

#define FRONT_LEFT_ESC_PIN {{front_left_pin}}
#define FRONT_RIGHT_ESC_PIN {{front_right_pin}}
#define BACK_LEFT_ESC_PIN {{back_left_pin}}
#define BACK_RIGHT_ESC_PIN {{back_right_pin}}

#define ZERO_ROTOR_SPEED {{zero_thrust_pwm}}  // 1000
#define IDLE_ROTOR_SPEED {{idle_thrust_pwm}}  // 1150
#define ABSOLUTE_MIN_PWM {{absolute_min_pwm_value}}  // 1000
#define ABSOLUTE_MAX_PWM {{absolute_max_pwm_value}}  // 2000
//  need to include in receiver.h instead  //  #define ESC_MIN_PWM {{esc_min_pwm_value}}  // 1150 so props idle
                                           // #define ESC_MAX_PWM {{esc_max_pwm_value}}  // 1850 so full throttle still allows room for LQR control
#define SERVO_LIBRARY_MIN_ANGLE {{servo_range_min}}  // 0 degrees
#define SERVO_LIBRARY_MAX_ANGLE {{servo_range_max}}  // 180 degrees

class Esc {
    private:
    Servo esc_front_left;  // create servo object to control an ESC
    Servo esc_front_right;
    Servo esc_back_left;
    Servo esc_back_right;
    bool arm;

    public:
    Esc();
    int rotor_speed_front_left;
    int rotor_speed_front_right;
    int rotor_speed_back_left;
    int rotor_speed_back_right;
    void attach_esc_to_pwm_pin(void);
    void write_speed_to_esc(rotor_speed_front_left, rotor_speed_front_right, rotor_speed_back_left, rotor_speed_back_right);
    void disarm(void);
    void arm(void);
}

/*--------------------------------------------------------------------------------------------------------------------*/

Esc::Esc(){
    Esc.attach_esc_to_pwm_pin();
    Esc.disarm();
}

void Esc::attach_esc_to_pwm_pin(void){
    esc_front_left.attach(FRONT_LEFT_ESC_PIN, ABSOLUTE_MIN_PWM, ABSOLUTE_MAX_PWM);  // (PWN pin, absolute min (1000), absolute max (2000))
    esc_front_right.attach(FRONT_RIGHT_ESC_PIN, ABSOLUTE_MIN_PWM, ABSOLUTE_MAX_PWM);
    esc_back_left.attach(BACK_LEFT_ESC_PIN, ABSOLUTE_MIN_PWM, ABSOLUTE_MAX_PWM);
    esc_back_right.attach(BACK_RIGHT_ESC_PIN, ABSOLUTE_MIN_PWM, ABSOLUTE_MAX_PWM);
}

void Esc::write_speed_to_esc(rotor_speed_front_left, rotor_speed_front_right, rotor_speed_back_left, rotor_speed_back_right){
    // send speed to ESCs
    esc_front_left.writeMicroseconds(rotor_speed_front_left);
    esc_front_right.writeMicroseconds(rotor_speed_front_right);
    esc_back_left.writeMicroseconds(rotor_speed_back_left);
    esc_back_right.writeMicroseconds(rotor_speed_back_right);
}

void Esc::disarm(void){
    arm = 0;
    rotor_speed_front_left = ZERO_ROTOR_SPEED;
    rotor_speed_front_right = ZERO_ROTOR_SPEED;
    rotor_speed_back_left = ZERO_ROTOR_SPEED;
    rotor_speed_back_right = ZERO_ROTOR_SPEED;
    Esc.write_speed_to_esc(rotor_speed_front_left, rotor_speed_front_right, rotor_speed_back_left, rotor_speed_back_right);
}

void Esc::arm(void){
    arm = 1;
    rotor_speed_front_left = IDLE_ROTOR_SPEED;
    rotor_speed_front_right = IDLE_ROTOR_SPEED;
    rotor_speed_back_left = IDLE_ROTOR_SPEED;
    rotor_speed_back_right = IDLE_ROTOR_SPEED;
    Esc.write_speed_to_esc(rotor_speed_front_left, rotor_speed_front_right, rotor_speed_back_left, rotor_speed_back_right);
}

#endif