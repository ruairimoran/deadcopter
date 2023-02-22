#include "debug.h"
#include "config.h"
#include "vars.h"
#include "ss_and_mats.h"


#ifndef NO_MOTOR_RPS_IN_SS
void generate_inputs(int fly_throttle, int &fly_front_left, int &fly_front_right, int &fly_back_left, int &fly_back_right,
                     float &f_u0, float &f_u1, float &f_u2, float &fq0y, float &f_y0,
                     float &f_y1, float &f_y2, float &f_y3, float &f_y4, float &f_y5) {

  // find observed y_hat
  y_hat[0] = +Cd[0][0] * x_hat[0] + Cd[0][1] * x_hat[1] + Cd[0][2] * x_hat[2] + Cd[0][3] * x_hat[3] + Cd[0][4] * x_hat[4] + Cd[0][5] * x_hat[5] + Cd[0][6] * x_hat[6] + Cd[0][7] * x_hat[7] + Cd[0][8] * x_hat[8];
  y_hat[1] = +Cd[1][0] * x_hat[0] + Cd[1][1] * x_hat[1] + Cd[1][2] * x_hat[2] + Cd[1][3] * x_hat[3] + Cd[1][4] * x_hat[4] + Cd[1][5] * x_hat[5] + Cd[1][6] * x_hat[6] + Cd[1][7] * x_hat[7] + Cd[1][8] * x_hat[8];
  y_hat[2] = +Cd[2][0] * x_hat[0] + Cd[2][1] * x_hat[1] + Cd[2][2] * x_hat[2] + Cd[2][3] * x_hat[3] + Cd[2][4] * x_hat[4] + Cd[2][5] * x_hat[5] + Cd[2][6] * x_hat[6] + Cd[2][7] * x_hat[7] + Cd[2][8] * x_hat[8];
  y_hat[3] = +Cd[3][0] * x_hat[0] + Cd[3][1] * x_hat[1] + Cd[3][2] * x_hat[2] + Cd[3][3] * x_hat[3] + Cd[3][4] * x_hat[4] + Cd[3][5] * x_hat[5] + Cd[3][6] * x_hat[6] + Cd[3][7] * x_hat[7] + Cd[3][8] * x_hat[8];
  y_hat[4] = +Cd[4][0] * x_hat[0] + Cd[4][1] * x_hat[1] + Cd[4][2] * x_hat[2] + Cd[4][3] * x_hat[3] + Cd[4][4] * x_hat[4] + Cd[4][5] * x_hat[5] + Cd[4][6] * x_hat[6] + Cd[4][7] * x_hat[7] + Cd[4][8] * x_hat[8];
  y_hat[5] = +Cd[5][0] * x_hat[0] + Cd[5][1] * x_hat[1] + Cd[5][2] * x_hat[2] + Cd[5][3] * x_hat[3] + Cd[5][4] * x_hat[4] + Cd[5][5] * x_hat[5] + Cd[5][6] * x_hat[6] + Cd[5][7] * x_hat[7] + Cd[5][8] * x_hat[8];


  // find y difference (y_hat - y)
  q0_y_hat = solve_q0(y_hat[0], y_hat[1], y_hat[2]);
  quaternion_difference(q0_y_hat, y_hat[0], y_hat[1], y_hat[2],
                        q0_y, y[0], y[1], y[2],
                        q0_y_diff, y_diff[0], y_diff[1], y_diff[2]);
  y_diff[3] = y_hat[3] - y[3];
  y_diff[4] = y_hat[4] - y[4];
  y_diff[5] = y_hat[5] - y[5];


  // find observed x_hat
  x_hat_buffer[0] = x_hat[0];
  x_hat_buffer[1] = x_hat[1];
  x_hat_buffer[2] = x_hat[2];
  x_hat_buffer[3] = x_hat[3];
  x_hat_buffer[4] = x_hat[4];
  x_hat_buffer[5] = x_hat[5];
  x_hat_buffer[6] = x_hat[6];
  x_hat_buffer[7] = x_hat[7];
  x_hat_buffer[8] = x_hat[8];
  x_hat[0] = +Ad[0][0] * x_hat_buffer[0] + Ad[0][1] * x_hat_buffer[1] + Ad[0][2] * x_hat_buffer[2] + Ad[0][3] * x_hat_buffer[3] + Ad[0][4] * x_hat_buffer[4] + Ad[0][5] * x_hat_buffer[5] + Ad[0][6] * x_hat_buffer[6] + Ad[0][7] * x_hat_buffer[7] + Ad[0][8] * x_hat_buffer[8] + Bd[0][0] * throttle_and_u[1] + Bd[0][1] * throttle_and_u[2] + Bd[0][2] * throttle_and_u[3] + L[0][0] * y_diff[0] + L[0][1] * y_diff[1] + L[0][2] * y_diff[2] + L[0][3] * y_diff[3] + L[0][4] * y_diff[4] + L[0][5] * y_diff[5];
  x_hat[1] = +Ad[1][0] * x_hat_buffer[0] + Ad[1][1] * x_hat_buffer[1] + Ad[1][2] * x_hat_buffer[2] + Ad[1][3] * x_hat_buffer[3] + Ad[1][4] * x_hat_buffer[4] + Ad[1][5] * x_hat_buffer[5] + Ad[1][6] * x_hat_buffer[6] + Ad[1][7] * x_hat_buffer[7] + Ad[1][8] * x_hat_buffer[8] + Bd[1][0] * throttle_and_u[1] + Bd[1][1] * throttle_and_u[2] + Bd[1][2] * throttle_and_u[3] + L[1][0] * y_diff[0] + L[1][1] * y_diff[1] + L[1][2] * y_diff[2] + L[1][3] * y_diff[3] + L[1][4] * y_diff[4] + L[1][5] * y_diff[5];
  x_hat[2] = +Ad[2][0] * x_hat_buffer[0] + Ad[2][1] * x_hat_buffer[1] + Ad[2][2] * x_hat_buffer[2] + Ad[2][3] * x_hat_buffer[3] + Ad[2][4] * x_hat_buffer[4] + Ad[2][5] * x_hat_buffer[5] + Ad[2][6] * x_hat_buffer[6] + Ad[2][7] * x_hat_buffer[7] + Ad[2][8] * x_hat_buffer[8] + Bd[2][0] * throttle_and_u[1] + Bd[2][1] * throttle_and_u[2] + Bd[2][2] * throttle_and_u[3] + L[2][0] * y_diff[0] + L[2][1] * y_diff[1] + L[2][2] * y_diff[2] + L[2][3] * y_diff[3] + L[2][4] * y_diff[4] + L[2][5] * y_diff[5];
  x_hat[3] = +Ad[3][0] * x_hat_buffer[0] + Ad[3][1] * x_hat_buffer[1] + Ad[3][2] * x_hat_buffer[2] + Ad[3][3] * x_hat_buffer[3] + Ad[3][4] * x_hat_buffer[4] + Ad[3][5] * x_hat_buffer[5] + Ad[3][6] * x_hat_buffer[6] + Ad[3][7] * x_hat_buffer[7] + Ad[3][8] * x_hat_buffer[8] + Bd[3][0] * throttle_and_u[1] + Bd[3][1] * throttle_and_u[2] + Bd[3][2] * throttle_and_u[3] + L[3][0] * y_diff[0] + L[3][1] * y_diff[1] + L[3][2] * y_diff[2] + L[3][3] * y_diff[3] + L[3][4] * y_diff[4] + L[3][5] * y_diff[5];
  x_hat[4] = +Ad[4][0] * x_hat_buffer[0] + Ad[4][1] * x_hat_buffer[1] + Ad[4][2] * x_hat_buffer[2] + Ad[4][3] * x_hat_buffer[3] + Ad[4][4] * x_hat_buffer[4] + Ad[4][5] * x_hat_buffer[5] + Ad[4][6] * x_hat_buffer[6] + Ad[4][7] * x_hat_buffer[7] + Ad[4][8] * x_hat_buffer[8] + Bd[4][0] * throttle_and_u[1] + Bd[4][1] * throttle_and_u[2] + Bd[4][2] * throttle_and_u[3] + L[4][0] * y_diff[0] + L[4][1] * y_diff[1] + L[4][2] * y_diff[2] + L[4][3] * y_diff[3] + L[4][4] * y_diff[4] + L[4][5] * y_diff[5];
  x_hat[5] = +Ad[5][0] * x_hat_buffer[0] + Ad[5][1] * x_hat_buffer[1] + Ad[5][2] * x_hat_buffer[2] + Ad[5][3] * x_hat_buffer[3] + Ad[5][4] * x_hat_buffer[4] + Ad[5][5] * x_hat_buffer[5] + Ad[5][6] * x_hat_buffer[6] + Ad[5][7] * x_hat_buffer[7] + Ad[5][8] * x_hat_buffer[8] + Bd[5][0] * throttle_and_u[1] + Bd[5][1] * throttle_and_u[2] + Bd[5][2] * throttle_and_u[3] + L[5][0] * y_diff[0] + L[5][1] * y_diff[1] + L[5][2] * y_diff[2] + L[5][3] * y_diff[3] + L[5][4] * y_diff[4] + L[5][5] * y_diff[5];
  x_hat[6] = +Ad[6][0] * x_hat_buffer[0] + Ad[6][1] * x_hat_buffer[1] + Ad[6][2] * x_hat_buffer[2] + Ad[6][3] * x_hat_buffer[3] + Ad[6][4] * x_hat_buffer[4] + Ad[6][5] * x_hat_buffer[5] + Ad[6][6] * x_hat_buffer[6] + Ad[6][7] * x_hat_buffer[7] + Ad[6][8] * x_hat_buffer[8] + Bd[6][0] * throttle_and_u[1] + Bd[6][1] * throttle_and_u[2] + Bd[6][2] * throttle_and_u[3] + L[6][0] * y_diff[0] + L[6][1] * y_diff[1] + L[6][2] * y_diff[2] + L[6][3] * y_diff[3] + L[6][4] * y_diff[4] + L[6][5] * y_diff[5];
  x_hat[7] = +Ad[7][0] * x_hat_buffer[0] + Ad[7][1] * x_hat_buffer[1] + Ad[7][2] * x_hat_buffer[2] + Ad[7][3] * x_hat_buffer[3] + Ad[7][4] * x_hat_buffer[4] + Ad[7][5] * x_hat_buffer[5] + Ad[7][6] * x_hat_buffer[6] + Ad[7][7] * x_hat_buffer[7] + Ad[7][8] * x_hat_buffer[8] + Bd[7][0] * throttle_and_u[1] + Bd[7][1] * throttle_and_u[2] + Bd[7][2] * throttle_and_u[3] + L[7][0] * y_diff[0] + L[7][1] * y_diff[1] + L[7][2] * y_diff[2] + L[7][3] * y_diff[3] + L[7][4] * y_diff[4] + L[7][5] * y_diff[5];
  x_hat[8] = +Ad[8][0] * x_hat_buffer[0] + Ad[8][1] * x_hat_buffer[1] + Ad[8][2] * x_hat_buffer[2] + Ad[8][3] * x_hat_buffer[3] + Ad[8][4] * x_hat_buffer[4] + Ad[8][5] * x_hat_buffer[5] + Ad[8][6] * x_hat_buffer[6] + Ad[8][7] * x_hat_buffer[7] + Ad[8][8] * x_hat_buffer[8] + Bd[8][0] * throttle_and_u[1] + Bd[8][1] * throttle_and_u[2] + Bd[8][2] * throttle_and_u[3] + L[8][0] * y_diff[0] + L[8][1] * y_diff[1] + L[8][2] * y_diff[2] + L[8][3] * y_diff[3] + L[8][4] * y_diff[4] + L[8][5] * y_diff[5];


  // find x and u equilibrium values
  r_short[0] = r[0];
  r_short[1] = r[1];
  r_short[2] = r[2];
  xu_e[0] = +G[0][0] * r_short[0] + G[0][1] * r_short[1] + G[0][2] * r_short[2];
  xu_e[1] = +G[1][0] * r_short[0] + G[1][1] * r_short[1] + G[1][2] * r_short[2];
  xu_e[2] = +G[2][0] * r_short[0] + G[2][1] * r_short[1] + G[2][2] * r_short[2];
  xu_e[3] = +G[3][0] * r_short[0] + G[3][1] * r_short[1] + G[3][2] * r_short[2];
  xu_e[4] = +G[4][0] * r_short[0] + G[4][1] * r_short[1] + G[4][2] * r_short[2];
  xu_e[5] = +G[5][0] * r_short[0] + G[5][1] * r_short[1] + G[5][2] * r_short[2];
  xu_e[6] = +G[6][0] * r_short[0] + G[6][1] * r_short[1] + G[6][2] * r_short[2];
  xu_e[7] = +G[7][0] * r_short[0] + G[7][1] * r_short[1] + G[7][2] * r_short[2];
  xu_e[8] = +G[8][0] * r_short[0] + G[8][1] * r_short[1] + G[8][2] * r_short[2];
  xu_e[9] = +G[9][0] * r_short[0] + G[9][1] * r_short[1] + G[9][2] * r_short[2];
  xu_e[10] = +G[10][0] * r_short[0] + G[10][1] * r_short[1] + G[10][2] * r_short[2];
  xu_e[11] = +G[11][0] * r_short[0] + G[11][1] * r_short[1] + G[11][2] * r_short[2];


  // find x difference (x_hat - x_e)
  q0_x_hat = solve_q0(x_hat[0], x_hat[1], x_hat[2]);
  q0_x_e = solve_q0(xu_e[0], xu_e[1], xu_e[2]);
  quaternion_difference(q0_x_hat, x_hat[0], x_hat[1], x_hat[2],
                        q0_x_e, xu_e[0], xu_e[1], xu_e[2],
                        q0_x_diff, x_diff[0], x_diff[1], x_diff[2]);
  x_diff[3] = x_hat[3] - xu_e[3];
  x_diff[4] = x_hat[4] - xu_e[4];
  x_diff[5] = x_hat[5] - xu_e[5];
  x_diff[6] = x_hat[6] - xu_e[6];
  x_diff[7] = x_hat[7] - xu_e[7];
  x_diff[8] = x_hat[8] - xu_e[8];


  // find control action matrix, u
  throttle_and_u[1] = xu_e[9] + K[0][0] * x_diff[0] + K[0][1] * x_diff[1] + K[0][2] * x_diff[2] + K[0][3] * x_diff[3] + K[0][4] * x_diff[4] + K[0][5] * x_diff[5] + K[0][6] * x_diff[6] + K[0][7] * x_diff[7] + K[0][8] * x_diff[8];
  throttle_and_u[2] = xu_e[10] + K[1][0] * x_diff[0] + K[1][1] * x_diff[1] + K[1][2] * x_diff[2] + K[1][3] * x_diff[3] + K[1][4] * x_diff[4] + K[1][5] * x_diff[5] + K[1][6] * x_diff[6] + K[1][7] * x_diff[7] + K[1][8] * x_diff[8];
  throttle_and_u[3] = xu_e[11] + K[2][0] * x_diff[0] + K[2][1] * x_diff[1] + K[2][2] * x_diff[2] + K[2][3] * x_diff[3] + K[2][4] * x_diff[4] + K[2][5] * x_diff[5] + K[2][6] * x_diff[6] + K[2][7] * x_diff[7] + K[2][8] * x_diff[8];


  // format into control output for motors
  throttle_and_u[0] = (fly_throttle - (float)RECEIVER_MIN) * ((float)THROTTLE_MAX) / ((float)RECEIVER_MAX - (float)RECEIVER_MIN);  // form matrix of throttle on top of u
  output_to_motor[0] = +motor_proportions[0][0] * throttle_and_u[0] + motor_proportions[0][1] * throttle_and_u[1] + motor_proportions[0][2] * throttle_and_u[2] + motor_proportions[0][3] * throttle_and_u[3];
  output_to_motor[1] = +motor_proportions[1][0] * throttle_and_u[0] + motor_proportions[1][1] * throttle_and_u[1] + motor_proportions[1][2] * throttle_and_u[2] + motor_proportions[1][3] * throttle_and_u[3];
  output_to_motor[2] = +motor_proportions[2][0] * throttle_and_u[0] + motor_proportions[2][1] * throttle_and_u[1] + motor_proportions[2][2] * throttle_and_u[2] + motor_proportions[2][3] * throttle_and_u[3];
  output_to_motor[3] = +motor_proportions[3][0] * throttle_and_u[0] + motor_proportions[3][1] * throttle_and_u[1] + motor_proportions[3][2] * throttle_and_u[2] + motor_proportions[3][3] * throttle_and_u[3];
  fly_front_left = ceil(output_to_motor[0] + 1000.0f);
  fly_front_right = ceil(output_to_motor[1] + 1000.0f);
  fly_back_left = ceil(output_to_motor[2] + 1000.0f);
  fly_back_right = ceil(output_to_motor[3] + 1000.0f);

  // if control action result is nan, reset x_hat and y_hat
  if (isnan(throttle_and_u[1]) || isnan(throttle_and_u[2]) || isnan(throttle_and_u[3])) {
    q0_x_hat = 1.0f;
    x_hat[0] = 0.0f;
    x_hat[1] = 0.0f;
    x_hat[2] = 0.0f;
    x_hat[3] = 0.0f;
    x_hat[4] = 0.0f;
    x_hat[5] = 0.0f;
    x_hat[6] = 0.0f;
    x_hat[7] = 0.0f;
    x_hat[8] = 0.0f;
    q0_y_hat = 1.0f;
    y_hat[0] = 0.0f;
    y_hat[1] = 0.0f;
    y_hat[2] = 0.0f;
    y_hat[3] = 0.0f;
    y_hat[4] = 0.0f;
    y_hat[5] = 0.0f;
  }


  // testing outputs for serial print // to be deleted
  f_u0 = throttle_and_u[1];
  f_u1 = throttle_and_u[2];
  f_u2 = throttle_and_u[3];

  fq0y = q0_y;
  f_y0 = y[0];
  f_y1 = y[1];
  f_y2 = y[2];
  f_y3 = y[3];
  f_y4 = y[4];
  f_y5 = y[5];
}
#elif defined(NO_MOTOR_RPS_IN_SS)
void generate_inputs() {
  /*****************************************************************
    Write code to generate the ====> eq_inputs and eq_states
  ******************************************************************/
  eq_states.quaternions.q0 = radio_quaternions.q0;
  eq_states.quaternions.q1 = radio_quaternions.q1;
  eq_states.quaternions.q2 = radio_quaternions.q2;
  eq_states.quaternions.q3 = radio_quaternions.q3;
  eq_states.omega.gyro.x = 0;
  eq_states.omega.gyro.y = 0;
  eq_states.omega.gyro.z = 0;

  // find control action matrix, u
  _quaternion_difference(quaternions, eq_states.quaternions, &x_minus_xeq.quaternions);
  x_minus_xeq.omega.gyro.x = gyro_event.gyro.x - eq_states.omega.gyro.x;
  x_minus_xeq.omega.gyro.y = gyro_event.gyro.y - eq_states.omega.gyro.y;
  x_minus_xeq.omega.gyro.z = gyro_event.gyro.z - eq_states.omega.gyro.z;

  inputs.u1 = eq_inputs.u1 + K[0][0] * x_minus_xeq.quaternions.q1 + K[0][1] * x_minus_xeq.quaternions.q2 + K[0][2] * x_minus_xeq.quaternions.q3 + K[0][3] * x_minus_xeq.omega.gyro.x + K[0][4] * x_minus_xeq.omega.gyro.y + K[0][5] * x_minus_xeq.omega.gyro.z;
  inputs.u2 = eq_inputs.u2 + K[1][0] * x_minus_xeq.quaternions.q1 + K[1][1] * x_minus_xeq.quaternions.q2 + K[1][2] * x_minus_xeq.quaternions.q3 + K[1][3] * x_minus_xeq.omega.gyro.x + K[1][4] * x_minus_xeq.omega.gyro.y + K[1][5] * x_minus_xeq.omega.gyro.z;
  inputs.u3 = eq_inputs.u3 + K[2][0] * x_minus_xeq.quaternions.q1 + K[2][1] * x_minus_xeq.quaternions.q2 + K[2][2] * x_minus_xeq.quaternions.q3 + K[2][3] * x_minus_xeq.omega.gyro.x + K[2][4] * x_minus_xeq.omega.gyro.y + K[2][5] * x_minus_xeq.omega.gyro.z;


  // format into control output for motors
  motor_speeds.motor0 = ceil(ip_to_mot[0][0] * radio_data.throttle + ip_to_mot[0][1] * inputs.u1 + ip_to_mot[0][2] * inputs.u2 + ip_to_mot[0][3] * inputs.u3 + 1000.0);
  motor_speeds.motor1 = ceil(ip_to_mot[1][0] * radio_data.throttle + ip_to_mot[1][1] * inputs.u1 + ip_to_mot[1][2] * inputs.u2 + ip_to_mot[1][3] * inputs.u3 + 1000.0);
  motor_speeds.motor2 = ceil(ip_to_mot[2][0] * radio_data.throttle + ip_to_mot[2][1] * inputs.u1 + ip_to_mot[2][2] * inputs.u2 + ip_to_mot[2][3] * inputs.u3 + 1000.0);
  motor_speeds.motor3 = ceil(ip_to_mot[3][0] * radio_data.throttle + ip_to_mot[3][1] * inputs.u1 + ip_to_mot[3][2] * inputs.u2 + ip_to_mot[3][3] * inputs.u3 + 1000.0);
}
#endif
