#include "vars.h"
#include "Adafruit_Sensor.h"
#pragma once


float __cy,
  __sy, __cp, __sp, __cr, __sr, __q0, __q1, __q2, __q3;

float _invSqrt(float input) {
  // Fast inverse square-root
  // See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
  float half_input = 0.5f * input;
  float output = input;
  long i = *(long *)&output;
  i = 0x5f3759df - (i >> 1);
  output = *(float *)&i;
  output = output * (1.5f - (half_input * output * output));  // first Newton Raphson iteration
  output = output * (1.5f - (half_input * output * output));  // second Newton Raphson iteration
  return output;
}

float _solve_q0(float q1, float q2, float q3) {
  // find q0 for unit quaternion
  return sqrt(1.0f - pow(q1, 2.0f) - pow(q2, 2.0f) - pow(q3, 2.0f));
}


void _to_quaternions(sensors_vec_t orientation, float conversion_factor, Quaternions *quaternions) {
  __cy = cos(orientation.heading * conversion_factor * 0.5f);
  __sy = sin(orientation.heading * conversion_factor * 0.5f);
  __cp = cos(orientation.pitch * conversion_factor);
  __sp = sin(orientation.pitch * conversion_factor);
  __cr = cos(orientation.roll * conversion_factor);
  __sr = sin(orientation.roll * conversion_factor);
  quaternions->q0 = __cr * __cp * __cy + __sr * __sp * __sy;
  quaternions->q1 = __sr * __cp * __cy - __cr * __sp * __sy;
  quaternions->q2 = __cr * __sp * __cy + __sr * __cp * __sy;
  quaternions->q3 = __cr * __cp * __sy - __sr * __sp * __cy;
}

void _to_euler(Quaternions quaternions, float conversion_factor, sensors_vec_t *orientation) {
  orientation->roll = atan2(2 * (quaternions.q0 * quaternions.q1 + quaternions.q2 * quaternions.q3), 1 - 2 * (quaternions.q1 * quaternions.q1 + quaternions.q2 * quaternions.q2)) * conversion_factor;
  orientation->pitch = asin(2 * (quaternions.q0 * quaternions.q2 - quaternions.q1 * quaternions.q3)) * conversion_factor;
  orientation->heading = atan2(2 * (quaternions.q0 * quaternions.q3 + quaternions.q2 * quaternions.q1), 1 - 2 * (quaternions.q3 * quaternions.q3 + quaternions.q2 * quaternions.q2)) * conversion_factor;
}

void _quaternion_difference(Quaternions quat1,
                            Quaternions quat2,
                            Quaternions *quat3) {
  // form conjugate of second quaternion
  quat2.q1 = -quat2.q1;
  quat2.q2 = -quat2.q2;
  quat2.q3 = -quat2.q3;
  // multiply first quaternion by conjugate of second quaternion
  quat3->q0 = quat1.q0 * quat2.q0 - quat1.q1 * quat2.q1 - quat1.q2 * quat2.q2 - quat1.q3 * quat2.q3;
  quat3->q1 = quat1.q0 * quat2.q1 + quat1.q1 * quat2.q0 + quat1.q2 * quat2.q3 - quat1.q3 * quat2.q2;
  quat3->q2 = quat1.q0 * quat2.q2 - quat1.q1 * quat2.q3 + quat1.q2 * quat2.q0 + quat1.q3 * quat2.q1;
  quat3->q3 = quat1.q0 * quat2.q3 + quat1.q1 * quat2.q2 - quat1.q2 * quat2.q1 + quat1.q3 * quat2.q0;
}
