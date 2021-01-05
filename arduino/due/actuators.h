// 2021-01-05 21:43:24.493286

#ifndef actuators.h
#define actuators.h

#include <Arduino.h>
#include <Servo.h>  // must map range onto 0 - 180 degrees

#define FRONT_LEFT_ESC_PIN 2
#define FRONT_RIGHT_ESC_PIN 3
#define BACK_LEFT_ESC_PIN 4
#define BACK_RIGHT_ESC_PIN 5

#define ZERO_ROTOR_SPEED 1000  // 1000
#define IDLE_ROTOR_SPEED 1150  // 1150
#define ABSOLUTE_MIN_PWM 1000  // 1000
#define ABSOLUTE_MAX_PWM 2000  // 2000
#define SERVO_LIBRARY_MIN_ANGLE 0  // 0 degrees
#define SERVO_LIBRARY_MAX_ANGLE 180  // 180 degrees

class Esc {
    private:
    Servo esc_front_left;  // create servo object to control an ESC
    Servo esc_front_right;
    Servo esc_back_left;
    Servo esc_back_right;
    bool arm_status;

    public:
    Esc();
    void attach_esc_to_pwm_pin(void);
    void write_speed_to_esc(int rotor_speed_front_left, int rotor_speed_front_right, int rotor_speed_back_left, int rotor_speed_back_right);
    void disarm(void);
    void arm(void);
    void get_arm_status(bool &esc_arm_status);
};

/*--------------------------------------------------------------------------------------------------------------------*/

Esc::Esc() {

}

void Esc::attach_esc_to_pwm_pin(void) {
    esc_front_left.attach(FRONT_LEFT_ESC_PIN, ABSOLUTE_MIN_PWM, ABSOLUTE_MAX_PWM);  // (PWN pin, absolute min (1000), absolute max (2000))
    esc_front_right.attach(FRONT_RIGHT_ESC_PIN, ABSOLUTE_MIN_PWM, ABSOLUTE_MAX_PWM);
    esc_back_left.attach(BACK_LEFT_ESC_PIN, ABSOLUTE_MIN_PWM, ABSOLUTE_MAX_PWM);
    esc_back_right.attach(BACK_RIGHT_ESC_PIN, ABSOLUTE_MIN_PWM, ABSOLUTE_MAX_PWM);
}

void Esc::write_speed_to_esc(int rotor_speed_front_left, int rotor_speed_front_right, int rotor_speed_back_left, int rotor_speed_back_right) {
    // send speed to ESCs
    esc_front_left.writeMicroseconds(rotor_speed_front_left);
    esc_front_right.writeMicroseconds(rotor_speed_front_right);
    esc_back_left.writeMicroseconds(rotor_speed_back_left);
    esc_back_right.writeMicroseconds(rotor_speed_back_right);
}

void Esc::disarm(void) {
    arm_status = 0;
    write_speed_to_esc(ZERO_ROTOR_SPEED, ZERO_ROTOR_SPEED, ZERO_ROTOR_SPEED, ZERO_ROTOR_SPEED);
}

void Esc::arm(void) {
    arm_status = 1;
    write_speed_to_esc(IDLE_ROTOR_SPEED, IDLE_ROTOR_SPEED, IDLE_ROTOR_SPEED, IDLE_ROTOR_SPEED);
}

void Esc::get_arm_status(bool &esc_arm_status) {
    esc_arm_status = arm_status;
}

#endif