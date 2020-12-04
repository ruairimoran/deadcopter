// 2020-12-04 17:32:34.814543

#ifndef actuators.h
#define actuators.h

#include <Arduino.h>
#include <Servo.h>  // must map range onto 0 - 180 degrees

#define FRONT_LEFT_ESC_PIN 2
#define FRONT_RIGHT_ESC_PIN 3
#define BACK_LEFT_ESC_PIN 4
#define BACK_RIGHT_ESC_PIN 5

#define ABSOLUTE_MIN_PWM 1000  // 1000
#define ABSOLUTE_MAX_PWM 2000  // 2000
#define ESC_MIN_PWM 1150  // 1150 so props idle
#define ESC_MAX_PWM 1850  // 1850 so full throttle still allows room for LQR control
#define SERVO_LIBRARY_MIN_ANGLE 0  // 0 degrees
#define SERVO_LIBRARY_MAX_ANGLE 180  // 180 degrees

class Esc {
    private:
    Servo esc_front_left;  // create servo object to control an ESC
    Servo esc_front_right;
    Servo esc_back_left;
    Servo esc_back_right;
    double mapped_rotor_speed_front_left;
    double mapped_rotor_speed_front_right;
    double mapped_rotor_speed_back_left;
    double mapped_rotor_speed_back_right;

    public:
    Esc();
    int rotor_speed_front_left;
    int rotor_speed_front_right;
    int rotor_speed_back_left;
    int rotor_speed_back_right;
    void attach_esc_to_pwm_pin(void)
    void map_to_servo_range(rotor_speed_front_left, rotor_speed_front_right, rotor_speed_back_left, rotor_speed_back_right)
    void write_speed_to_esc(mapped_rotor_speed_front_left, mapped_rotor_speed_front_right, mapped_rotor_speed_back_left, mapped_rotor_speed_back_right)
}

/*--------------------------------------------------------------------------------------------------------------------*/

Esc::Esc(){

}

void Esc::attach_esc_to_pwm_pin(void){
    esc_front_left.attach(FRONT_LEFT_ESC_PIN, ESC_MIN_PWM, ESC_MAX_PWM);  // (PWN pin, absolute min (1000), absolute max (2000))
    esc_front_right.attach(FRONT_RIGHT_ESC_PIN, ESC_MIN_PWM, ESC_MAX_PWM);
    esc_back_left.attach(BACK_LEFT_ESC_PIN, ESC_MIN_PWM, ESC_MAX_PWM);
    esc_back_right.attach(BACK_RIGHT_ESC_PIN, ESC_MIN_PWM, ESC_MAX_PWM);
}

void Esc::map_to_servo_range(rotor_speed_front_left, rotor_speed_front_right, rotor_speed_back_left, rotor_speed_back_right){
    // scale to use with the servo library (value between 0 and 180)
    mapped_rotor_speed_front_left = map(rotor_speed_front_left, ABSOLUTE_MIN_PWM, ABSOLUTE_MAX_PWM, SERVO_LIBRARY_MIN_ANGLE, SERVO_LIBRARY_MAX_ANGLE);
    mapped_rotor_speed_front_right = map(rotor_speed_front_right, ABSOLUTE_MIN_PWM, ABSOLUTE_MAX_PWM, SERVO_LIBRARY_MIN_ANGLE, SERVO_LIBRARY_MAX_ANGLE);
    mapped_rotor_speed_back_left = map(rotor_speed_back_left, ABSOLUTE_MIN_PWM, ABSOLUTE_MAX_PWM, SERVO_LIBRARY_MIN_ANGLE, SERVO_LIBRARY_MAX_ANGLE);
    mapped_rotor_speed_back_right = map(rotor_speed_back_right, ABSOLUTE_MIN_PWM, ABSOLUTE_MAX_PWM, SERVO_LIBRARY_MIN_ANGLE, SERVO_LIBRARY_MAX_ANGLE);
}

void Esc::write_speed_to_esc(mapped_rotor_speed_front_left, mapped_rotor_speed_front_right, mapped_rotor_speed_back_left, mapped_rotor_speed_back_right){
    // send the signal to the ESC
    esc_front_left.write(mapped_rotor_speed_front_left);
    esc_front_right.write(mapped_rotor_speed_front_right);
    esc_back_left.write(mapped_rotor_speed_back_left);
    esc_back_right.write(mapped_rotor_speed_back_right);
}