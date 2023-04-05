#include "vars.h"
#include <Servo.h>
#include "config.h"


bool arm_status = false;


void attach_motors(void) {
  motors[0].attach(FRONT_LEFT_ESC_PIN, ABSOLUTE_MIN_PWM, ABSOLUTE_MAX_PWM);  // (PWN pin, absolute min (1000), absolute max (2000))
  motors[1].attach(FRONT_RIGHT_ESC_PIN, ABSOLUTE_MIN_PWM, ABSOLUTE_MAX_PWM);
  motors[2].attach(BACK_LEFT_ESC_PIN, ABSOLUTE_MIN_PWM, ABSOLUTE_MAX_PWM);
  motors[3].attach(BACK_RIGHT_ESC_PIN, ABSOLUTE_MIN_PWM, ABSOLUTE_MAX_PWM);
}

void write_speed_to_esc() {
  // send speed to ESCs
  motors[0].writeMicroseconds(motor_speeds.motor0);
  motors[1].writeMicroseconds(motor_speeds.motor1);
  motors[2].writeMicroseconds(motor_speeds.motor2);
  motors[3].writeMicroseconds(motor_speeds.motor3);
}

void disarm(void) {
  arm_status = 0;
  motor_speeds = (Motor_speeds){ .motor0 = ZERO_ROTOR_SPEED, .motor1 = ZERO_ROTOR_SPEED, .motor2 = ZERO_ROTOR_SPEED, .motor3 = ZERO_ROTOR_SPEED };
  write_speed_to_esc();
}

void arm(void) {
  arm_status = 1;
  motor_speeds = (Motor_speeds){ .motor0 = ARM_ROTOR_SPEED, .motor1 = ARM_ROTOR_SPEED, .motor2 = ARM_ROTOR_SPEED, .motor3 = ARM_ROTOR_SPEED };
  write_speed_to_esc();
}

bool get_arm_status(void) {
  return arm_status;
}
