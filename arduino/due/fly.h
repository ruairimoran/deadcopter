// 2021-01-05 21:43:24.493286

#ifndef fly.h
#define fly.h

#include <Arduino.h>
#include <math.h>
#include <MadgwickAHRS.h>
#include "imu.h"
#include "receiver.h"

class Fly {
    private:
    float Ad[9][9] = {{ 1.00000000e+00,0.00000000e+00,0.00000000e+00,4.00000000e-03
, 0.00000000e+00,0.00000000e+00,2.94416480e-05,0.00000000e+00
, 0.00000000e+00},
 { 0.00000000e+00,1.00000000e+00,0.00000000e+00,0.00000000e+00
, 4.00000000e-03,0.00000000e+00,0.00000000e+00,2.77963794e-05
, 0.00000000e+00},
 { 0.00000000e+00,0.00000000e+00,1.00000000e+00,0.00000000e+00
, 0.00000000e+00,4.00000000e-03,0.00000000e+00,0.00000000e+00
,-2.10785433e-05},
 { 0.00000000e+00,0.00000000e+00,0.00000000e+00,1.00000000e+00
, 0.00000000e+00,0.00000000e+00,1.41810629e-02,0.00000000e+00
, 0.00000000e+00},
 { 0.00000000e+00,0.00000000e+00,0.00000000e+00,0.00000000e+00
, 1.00000000e+00,0.00000000e+00,0.00000000e+00,1.33885917e-02
, 0.00000000e+00},
 { 0.00000000e+00,0.00000000e+00,0.00000000e+00,0.00000000e+00
, 0.00000000e+00,1.00000000e+00,0.00000000e+00,0.00000000e+00
,-1.01528334e-02},
 { 0.00000000e+00,0.00000000e+00,0.00000000e+00,0.00000000e+00
, 0.00000000e+00,0.00000000e+00,7.95669462e-01,0.00000000e+00
, 0.00000000e+00},
 { 0.00000000e+00,0.00000000e+00,0.00000000e+00,0.00000000e+00
, 0.00000000e+00,0.00000000e+00,0.00000000e+00,7.95669462e-01
, 0.00000000e+00},
 { 0.00000000e+00,0.00000000e+00,0.00000000e+00,0.00000000e+00
, 0.00000000e+00,0.00000000e+00,0.00000000e+00,0.00000000e+00
, 7.95669462e-01}};  // discrete A array
    float Bd[9][3] = {{1.14261874e-04,0.00000000e+00,0.00000000e+00},
{0.00000000e+00,1.07876652e-04,0.00000000e+00},
{0.00000000e+00,0.00000000e+00,1.14393513e-03},
{8.41189943e-02,0.00000000e+00,0.00000000e+00},
{0.00000000e+00,7.94182270e-02,0.00000000e+00},
{0.00000000e+00,0.00000000e+00,5.52645655e-01},
{1.02165269e+01,0.00000000e+00,0.00000000e+00},
{0.00000000e+00,1.02165269e+01,0.00000000e+00},
{0.00000000e+00,0.00000000e+00,1.02165269e+01}};  // discrete B array
    float Cd[6][9] = {{1.,0.,0.,0.,0.,0.,0.,0.,0.},
{0.,1.,0.,0.,0.,0.,0.,0.,0.},
{0.,0.,1.,0.,0.,0.,0.,0.,0.},
{0.,0.,0.,1.,0.,0.,0.,0.,0.},
{0.,0.,0.,0.,1.,0.,0.,0.,0.},
{0.,0.,0.,0.,0.,1.,0.,0.,0.}};  // discrete C array
    float K_x[3][9] = {{-3.23772498e+01, -0.00000000e+00, -0.00000000e+00, -1.34904149e+00
,-0.00000000e+00, -0.00000000e+00, -8.73212276e-02, -0.00000000e+00
,-0.00000000e+00},
 {-0.00000000e+00, -3.23429357e+01, -0.00000000e+00, -0.00000000e+00
,-1.38136873e+00, -0.00000000e+00, -0.00000000e+00, -8.65884897e-02
,-0.00000000e+00},
 {-0.00000000e+00, -0.00000000e+00, -3.41565261e+01, -0.00000000e+00
,-0.00000000e+00, -1.73454726e+00, -0.00000000e+00, -0.00000000e+00
, 1.19250669e-02}};  // LQR gain for state control
    float K_z[3][6] = {{ 1.23039786e-01, -0.00000000e+00, -0.00000000e+00,2.07989862e-07
,-0.00000000e+00, -0.00000000e+00},
 {-0.00000000e+00,1.22782364e-01, -0.00000000e+00, -0.00000000e+00
, 2.67121096e-07, -0.00000000e+00},
 {-0.00000000e+00, -0.00000000e+00,1.28633938e-01, -0.00000000e+00
,-0.00000000e+00,6.20727128e-08}};  // LQR gain for integral action
    float L[9][6] = {{-6.18061118e-01, -0.00000000e+00, -0.00000000e+00, -3.02208972e-03
,-0.00000000e+00, -0.00000000e+00},
 {-0.00000000e+00, -6.18061114e-01, -0.00000000e+00, -0.00000000e+00
,-3.02191474e-03, -0.00000000e+00},
 {-0.00000000e+00, -0.00000000e+00, -6.18061101e-01, -0.00000000e+00
,-0.00000000e+00, -3.02130418e-03},
 {-3.18473493e-03, -0.00000000e+00, -0.00000000e+00, -6.56284238e-01
,-0.00000000e+00, -0.00000000e+00},
 {-0.00000000e+00, -3.18380172e-03, -0.00000000e+00, -0.00000000e+00
,-6.56238762e-01, -0.00000000e+00},
 {-0.00000000e+00, -0.00000000e+00, -3.18054512e-03, -0.00000000e+00
,-0.00000000e+00, -6.56080094e-01},
 {-2.88533838e-04, -0.00000000e+00, -0.00000000e+00, -1.44735424e-02
,-0.00000000e+00, -0.00000000e+00},
 {-0.00000000e+00, -2.72463954e-04, -0.00000000e+00, -0.00000000e+00
,-1.36668244e-02, -0.00000000e+00},
 {-0.00000000e+00, -0.00000000e+00,2.06757919e-04, -0.00000000e+00
,-0.00000000e+00,1.03693753e-02}};  // Kalman filter gain for minimising observation error
    float G[12][3] = {{1.,0.,0.},
{0.,1.,0.},
{0.,0.,1.},
{0.,0.,0.},
{0.,0.,0.},
{0.,0.,0.},
{0.,0.,0.},
{0.,0.,0.},
{0.,0.,0.},
{0.,0.,0.},
{0.,0.,0.},
{0.,0.,0.}};  // for calculating new equilibrium state and control action

    
    float throttle_and_u[4] = {{0}, {0}, {0}, {0}};  // control action on quadcopter (throttle, u_x, u_y, u_z)
    float r[6];  // control input from receiver (q1, q2, q3, 0, 0, 0)
    float r_short[3];  // for solving equilibrium faster
    float z[6] = {{0}, {0}, {0}, {0}, {0}, {0}};  // storing integral values
    float y[6];  // IMU values
    float y_hat[6];  // observed IMU values
    float x_hat[9] = {{0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}};  // observed state
    float x_hat_buffer[9];  // for using x_hat when calculating x_hat
    float xu_e[12];  // for equilibrium state
    float y_diff[6];  // for full y_hat - y
    float x_diff[9];  // for full x_hat - x_e
    

    // for formatting into output to motor
    float output_to_motor[4];
    float motor_proportions[4][4] = {{1.0, 1.0, -1.0, -1.0},
                                     {1.0, -1.0, -1.0, 1.0},
                                     {1.0, 1.0, 1.0, 1.0},
                                     {1.0, -1.0, 1.0, -1.0}};  // motor_speeds = motor_proportions * throttle_and_control

    // for solving quaternion differences
    float invSqrt(float input);
    float solve_q0(float q1, float q2, float q3);
    float quaternion_difference(float w1, float x1, float y1, float z1,
                                float w2, float x2, float y2, float z2,
                                float &w3, float &x3, float &y3, float &z3);
    float q0_y_hat;
    float q0_y;
    float q0_y_diff;
    float q0_x_hat;
    float q0_x_e;  // x_equilibrium
    float q0_x_diff;

    public:
    Fly();
    void set_matrix_r_and_y(float fly_roll, float fly_pitch, float fly_yaw,
                            float y_negative1, float y_0, float y_1, float y_2,
                            float y_3, float y_4, float y_5);
    void observe_and_control(int fly_throttle, int &fly_front_left, int &fly_front_right, int &fly_back_left, int &fly_back_right,
                             float &f_u0, float &f_u1, float &f_u2, float &fq0y, float &f_y0,
                             float &f_y1, float &f_y2, float &f_y3, float &f_y4, float &f_y5);

};

/*--------------------------------------------------------------------------------------------------------------------*/

Fly::Fly() {

}

float Fly::invSqrt(float input) {
    // Fast inverse square-root
    // See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
//	float half_input = 0.5f * input;
//	float output = input;
//	long i = *(long*)&output;
//	i = 0x5f3759df - (i>>1);
//	output = *(float*)&i;
//	output = output * (1.5f - (half_input * output * output));
//	output = output * (1.5f - (half_input * output * output));
	return input;
}

float Fly::solve_q0(float q1, float q2, float q3) {
    // find q0 for unit quaternion
    return sqrt(1 - pow(q1,2) - pow(q2,2) - pow(q3,2));
}

float Fly::quaternion_difference(float w1, float x1, float y1, float z1,
                                 float w2, float x2, float y2, float z2,
                                 float &w3, float &x3, float &y3, float &z3) {
    // form conjugate of second quaternion
    x2 = -x2;
    y2 = -y2;
    z2 = -z2;
    // multiply first quaternion by conjugate of second quaternion
    w3 = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    x3 = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    y3 = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    z3 = w1*z2 + x1*y2 - y1*x2 + z1*w2;
}

void Fly::set_matrix_r_and_y(float fly_roll, float fly_pitch, float fly_yaw,
                             float y_negative1, float y_0, float y_1, float y_2,
                             float y_3, float y_4, float y_5) {
    // get input quaternion from receiver euler angles
    float cy = cos(fly_yaw * 0.5f);
    float sy = sin(fly_yaw * 0.5f);
    float cp = cos(fly_pitch * 0.5f);
    float sp = sin(fly_pitch * 0.5f);
    float cr = cos(fly_roll * 0.5f);
    float sr = sin(fly_roll * 0.5f);
    float rx_q0 = cr * cp * cy + sr * sp * sy;
    float rx_q1 = sr * cp * cy - cr * sp * sy;
    float rx_q2 = cr * sp * cy + sr * cp * sy;
    float rx_q3 = cr * cp * sy - sr * sp * cy;

    // normalise receiver quaternion and set r matrix
    float fly_reciprocalNorm = invSqrt(rx_q0*rx_q0 + rx_q1*rx_q1 + rx_q2*rx_q2 + rx_q3*rx_q3);
	r[0] = rx_q1 * fly_reciprocalNorm;  // q1
	r[1] = rx_q2 * fly_reciprocalNorm;  // q2
	r[2] = rx_q3 * fly_reciprocalNorm;  // q3
    r[3] = 0.0f;
    r[4] = 0.0f;
    r[5] = 0.0f;

    // put imu values in y matrix
    q0_y = y_negative1;
    y[0] = y_0;
    y[1] = y_1;
    y[2] = y_2;
    y[3] = y_3;
    y[4] = y_4;
    y[5] = y_5;
}

void Fly::observe_and_control(int fly_throttle, int &fly_front_left, int &fly_front_right, int &fly_back_left, int &fly_back_right,
                              float &f_u0, float &f_u1, float &f_u2, float &fq0y, float &f_y0,
                              float &f_y1, float &f_y2, float &f_y3, float &f_y4, float &f_y5) {
    // integral action
    z[0] += r[0] - y[0];
    z[1] += r[1] - y[1];
    z[2] += r[2] - y[2];
    z[3] += r[3] - y[3];
    z[4] += r[4] - y[4];
    z[5] += r[5] - y[5];
    

    // find x and u equilibrium values
    r_short[0] = r[0];
    r_short[1] = r[1];
    r_short[2] = r[2];
    xu_e[0] = + G[0][0]*r_short[0] + G[0][1]*r_short[1] + G[0][2]*r_short[2];
    xu_e[1] = + G[1][0]*r_short[0] + G[1][1]*r_short[1] + G[1][2]*r_short[2];
    xu_e[2] = + G[2][0]*r_short[0] + G[2][1]*r_short[1] + G[2][2]*r_short[2];
    xu_e[3] = + G[3][0]*r_short[0] + G[3][1]*r_short[1] + G[3][2]*r_short[2];
    xu_e[4] = + G[4][0]*r_short[0] + G[4][1]*r_short[1] + G[4][2]*r_short[2];
    xu_e[5] = + G[5][0]*r_short[0] + G[5][1]*r_short[1] + G[5][2]*r_short[2];
    xu_e[6] = + G[6][0]*r_short[0] + G[6][1]*r_short[1] + G[6][2]*r_short[2];
    xu_e[7] = + G[7][0]*r_short[0] + G[7][1]*r_short[1] + G[7][2]*r_short[2];
    xu_e[8] = + G[8][0]*r_short[0] + G[8][1]*r_short[1] + G[8][2]*r_short[2];
    xu_e[9] = + G[9][0]*r_short[0] + G[9][1]*r_short[1] + G[9][2]*r_short[2];
    xu_e[10] = + G[10][0]*r_short[0] + G[10][1]*r_short[1] + G[10][2]*r_short[2];
    xu_e[11] = + G[11][0]*r_short[0] + G[11][1]*r_short[1] + G[11][2]*r_short[2];
    

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
    throttle_and_u[1] = xu_e[9] + K_x[0][0]*x_diff[0] + K_x[0][1]*x_diff[1] + K_x[0][2]*x_diff[2] + K_x[0][3]*x_diff[3] + K_x[0][4]*x_diff[4] + K_x[0][5]*x_diff[5] + K_x[0][6]*x_diff[6] + K_x[0][7]*x_diff[7] + K_x[0][8]*x_diff[8] + K_z[0][0]*z[0] + K_z[0][1]*z[1] + K_z[0][2]*z[2] + K_z[0][3]*z[3] + K_z[0][4]*z[4] + K_z[0][5]*z[5];
    throttle_and_u[2] = xu_e[10] + K_x[1][0]*x_diff[0] + K_x[1][1]*x_diff[1] + K_x[1][2]*x_diff[2] + K_x[1][3]*x_diff[3] + K_x[1][4]*x_diff[4] + K_x[1][5]*x_diff[5] + K_x[1][6]*x_diff[6] + K_x[1][7]*x_diff[7] + K_x[1][8]*x_diff[8] + K_z[1][0]*z[0] + K_z[1][1]*z[1] + K_z[1][2]*z[2] + K_z[1][3]*z[3] + K_z[1][4]*z[4] + K_z[1][5]*z[5];
    throttle_and_u[3] = xu_e[11] + K_x[2][0]*x_diff[0] + K_x[2][1]*x_diff[1] + K_x[2][2]*x_diff[2] + K_x[2][3]*x_diff[3] + K_x[2][4]*x_diff[4] + K_x[2][5]*x_diff[5] + K_x[2][6]*x_diff[6] + K_x[2][7]*x_diff[7] + K_x[2][8]*x_diff[8] + K_z[2][0]*z[0] + K_z[2][1]*z[1] + K_z[2][2]*z[2] + K_z[2][3]*z[3] + K_z[2][4]*z[4] + K_z[2][5]*z[5];
    

    // format into control output for motors
    throttle_and_u[0] = fly_throttle;  // form matrix of throttle on top of u
    output_to_motor[0] = + motor_proportions[0][0]*throttle_and_u[0] + motor_proportions[0][1]*throttle_and_u[1] + motor_proportions[0][2]*throttle_and_u[2] + motor_proportions[0][3]*throttle_and_u[3];
    output_to_motor[1] = + motor_proportions[1][0]*throttle_and_u[0] + motor_proportions[1][1]*throttle_and_u[1] + motor_proportions[1][2]*throttle_and_u[2] + motor_proportions[1][3]*throttle_and_u[3];
    output_to_motor[2] = + motor_proportions[2][0]*throttle_and_u[0] + motor_proportions[2][1]*throttle_and_u[1] + motor_proportions[2][2]*throttle_and_u[2] + motor_proportions[2][3]*throttle_and_u[3];
    output_to_motor[3] = + motor_proportions[3][0]*throttle_and_u[0] + motor_proportions[3][1]*throttle_and_u[1] + motor_proportions[3][2]*throttle_and_u[2] + motor_proportions[3][3]*throttle_and_u[3];
    fly_front_left = ceil(output_to_motor[0]);
    fly_front_right = ceil(output_to_motor[1]);
    fly_back_left = ceil(output_to_motor[2]);
    fly_back_right = ceil(output_to_motor[3]);

    // find observed y_hat
    y_hat[0] = + Cd[0][0]*x_hat[0] + Cd[0][1]*x_hat[1] + Cd[0][2]*x_hat[2] + Cd[0][3]*x_hat[3] + Cd[0][4]*x_hat[4] + Cd[0][5]*x_hat[5] + Cd[0][6]*x_hat[6] + Cd[0][7]*x_hat[7] + Cd[0][8]*x_hat[8];
    y_hat[1] = + Cd[1][0]*x_hat[0] + Cd[1][1]*x_hat[1] + Cd[1][2]*x_hat[2] + Cd[1][3]*x_hat[3] + Cd[1][4]*x_hat[4] + Cd[1][5]*x_hat[5] + Cd[1][6]*x_hat[6] + Cd[1][7]*x_hat[7] + Cd[1][8]*x_hat[8];
    y_hat[2] = + Cd[2][0]*x_hat[0] + Cd[2][1]*x_hat[1] + Cd[2][2]*x_hat[2] + Cd[2][3]*x_hat[3] + Cd[2][4]*x_hat[4] + Cd[2][5]*x_hat[5] + Cd[2][6]*x_hat[6] + Cd[2][7]*x_hat[7] + Cd[2][8]*x_hat[8];
    y_hat[3] = + Cd[3][0]*x_hat[0] + Cd[3][1]*x_hat[1] + Cd[3][2]*x_hat[2] + Cd[3][3]*x_hat[3] + Cd[3][4]*x_hat[4] + Cd[3][5]*x_hat[5] + Cd[3][6]*x_hat[6] + Cd[3][7]*x_hat[7] + Cd[3][8]*x_hat[8];
    y_hat[4] = + Cd[4][0]*x_hat[0] + Cd[4][1]*x_hat[1] + Cd[4][2]*x_hat[2] + Cd[4][3]*x_hat[3] + Cd[4][4]*x_hat[4] + Cd[4][5]*x_hat[5] + Cd[4][6]*x_hat[6] + Cd[4][7]*x_hat[7] + Cd[4][8]*x_hat[8];
    y_hat[5] = + Cd[5][0]*x_hat[0] + Cd[5][1]*x_hat[1] + Cd[5][2]*x_hat[2] + Cd[5][3]*x_hat[3] + Cd[5][4]*x_hat[4] + Cd[5][5]*x_hat[5] + Cd[5][6]*x_hat[6] + Cd[5][7]*x_hat[7] + Cd[5][8]*x_hat[8];
    

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
    x_hat[0] = + Ad[0][0]*x_hat_buffer[0] + Ad[0][1]*x_hat_buffer[1] + Ad[0][2]*x_hat_buffer[2] + Ad[0][3]*x_hat_buffer[3] + Ad[0][4]*x_hat_buffer[4] + Ad[0][5]*x_hat_buffer[5] + Ad[0][6]*x_hat_buffer[6] + Ad[0][7]*x_hat_buffer[7] + Ad[0][8]*x_hat_buffer[8] + Bd[0][0]*throttle_and_u[1] + Bd[0][1]*throttle_and_u[2] + Bd[0][2]*throttle_and_u[3] + L[0][0]*y_diff[0] + L[0][1]*y_diff[1] + L[0][2]*y_diff[2] + L[0][3]*y_diff[3] + L[0][4]*y_diff[4] + L[0][5]*y_diff[5];
    x_hat[1] = + Ad[1][0]*x_hat_buffer[0] + Ad[1][1]*x_hat_buffer[1] + Ad[1][2]*x_hat_buffer[2] + Ad[1][3]*x_hat_buffer[3] + Ad[1][4]*x_hat_buffer[4] + Ad[1][5]*x_hat_buffer[5] + Ad[1][6]*x_hat_buffer[6] + Ad[1][7]*x_hat_buffer[7] + Ad[1][8]*x_hat_buffer[8] + Bd[1][0]*throttle_and_u[1] + Bd[1][1]*throttle_and_u[2] + Bd[1][2]*throttle_and_u[3] + L[1][0]*y_diff[0] + L[1][1]*y_diff[1] + L[1][2]*y_diff[2] + L[1][3]*y_diff[3] + L[1][4]*y_diff[4] + L[1][5]*y_diff[5];
    x_hat[2] = + Ad[2][0]*x_hat_buffer[0] + Ad[2][1]*x_hat_buffer[1] + Ad[2][2]*x_hat_buffer[2] + Ad[2][3]*x_hat_buffer[3] + Ad[2][4]*x_hat_buffer[4] + Ad[2][5]*x_hat_buffer[5] + Ad[2][6]*x_hat_buffer[6] + Ad[2][7]*x_hat_buffer[7] + Ad[2][8]*x_hat_buffer[8] + Bd[2][0]*throttle_and_u[1] + Bd[2][1]*throttle_and_u[2] + Bd[2][2]*throttle_and_u[3] + L[2][0]*y_diff[0] + L[2][1]*y_diff[1] + L[2][2]*y_diff[2] + L[2][3]*y_diff[3] + L[2][4]*y_diff[4] + L[2][5]*y_diff[5];
    x_hat[3] = + Ad[3][0]*x_hat_buffer[0] + Ad[3][1]*x_hat_buffer[1] + Ad[3][2]*x_hat_buffer[2] + Ad[3][3]*x_hat_buffer[3] + Ad[3][4]*x_hat_buffer[4] + Ad[3][5]*x_hat_buffer[5] + Ad[3][6]*x_hat_buffer[6] + Ad[3][7]*x_hat_buffer[7] + Ad[3][8]*x_hat_buffer[8] + Bd[3][0]*throttle_and_u[1] + Bd[3][1]*throttle_and_u[2] + Bd[3][2]*throttle_and_u[3] + L[3][0]*y_diff[0] + L[3][1]*y_diff[1] + L[3][2]*y_diff[2] + L[3][3]*y_diff[3] + L[3][4]*y_diff[4] + L[3][5]*y_diff[5];
    x_hat[4] = + Ad[4][0]*x_hat_buffer[0] + Ad[4][1]*x_hat_buffer[1] + Ad[4][2]*x_hat_buffer[2] + Ad[4][3]*x_hat_buffer[3] + Ad[4][4]*x_hat_buffer[4] + Ad[4][5]*x_hat_buffer[5] + Ad[4][6]*x_hat_buffer[6] + Ad[4][7]*x_hat_buffer[7] + Ad[4][8]*x_hat_buffer[8] + Bd[4][0]*throttle_and_u[1] + Bd[4][1]*throttle_and_u[2] + Bd[4][2]*throttle_and_u[3] + L[4][0]*y_diff[0] + L[4][1]*y_diff[1] + L[4][2]*y_diff[2] + L[4][3]*y_diff[3] + L[4][4]*y_diff[4] + L[4][5]*y_diff[5];
    x_hat[5] = + Ad[5][0]*x_hat_buffer[0] + Ad[5][1]*x_hat_buffer[1] + Ad[5][2]*x_hat_buffer[2] + Ad[5][3]*x_hat_buffer[3] + Ad[5][4]*x_hat_buffer[4] + Ad[5][5]*x_hat_buffer[5] + Ad[5][6]*x_hat_buffer[6] + Ad[5][7]*x_hat_buffer[7] + Ad[5][8]*x_hat_buffer[8] + Bd[5][0]*throttle_and_u[1] + Bd[5][1]*throttle_and_u[2] + Bd[5][2]*throttle_and_u[3] + L[5][0]*y_diff[0] + L[5][1]*y_diff[1] + L[5][2]*y_diff[2] + L[5][3]*y_diff[3] + L[5][4]*y_diff[4] + L[5][5]*y_diff[5];
    x_hat[6] = + Ad[6][0]*x_hat_buffer[0] + Ad[6][1]*x_hat_buffer[1] + Ad[6][2]*x_hat_buffer[2] + Ad[6][3]*x_hat_buffer[3] + Ad[6][4]*x_hat_buffer[4] + Ad[6][5]*x_hat_buffer[5] + Ad[6][6]*x_hat_buffer[6] + Ad[6][7]*x_hat_buffer[7] + Ad[6][8]*x_hat_buffer[8] + Bd[6][0]*throttle_and_u[1] + Bd[6][1]*throttle_and_u[2] + Bd[6][2]*throttle_and_u[3] + L[6][0]*y_diff[0] + L[6][1]*y_diff[1] + L[6][2]*y_diff[2] + L[6][3]*y_diff[3] + L[6][4]*y_diff[4] + L[6][5]*y_diff[5];
    x_hat[7] = + Ad[7][0]*x_hat_buffer[0] + Ad[7][1]*x_hat_buffer[1] + Ad[7][2]*x_hat_buffer[2] + Ad[7][3]*x_hat_buffer[3] + Ad[7][4]*x_hat_buffer[4] + Ad[7][5]*x_hat_buffer[5] + Ad[7][6]*x_hat_buffer[6] + Ad[7][7]*x_hat_buffer[7] + Ad[7][8]*x_hat_buffer[8] + Bd[7][0]*throttle_and_u[1] + Bd[7][1]*throttle_and_u[2] + Bd[7][2]*throttle_and_u[3] + L[7][0]*y_diff[0] + L[7][1]*y_diff[1] + L[7][2]*y_diff[2] + L[7][3]*y_diff[3] + L[7][4]*y_diff[4] + L[7][5]*y_diff[5];
    x_hat[8] = + Ad[8][0]*x_hat_buffer[0] + Ad[8][1]*x_hat_buffer[1] + Ad[8][2]*x_hat_buffer[2] + Ad[8][3]*x_hat_buffer[3] + Ad[8][4]*x_hat_buffer[4] + Ad[8][5]*x_hat_buffer[5] + Ad[8][6]*x_hat_buffer[6] + Ad[8][7]*x_hat_buffer[7] + Ad[8][8]*x_hat_buffer[8] + Bd[8][0]*throttle_and_u[1] + Bd[8][1]*throttle_and_u[2] + Bd[8][2]*throttle_and_u[3] + L[8][0]*y_diff[0] + L[8][1]*y_diff[1] + L[8][2]*y_diff[2] + L[8][3]*y_diff[3] + L[8][4]*y_diff[4] + L[8][5]*y_diff[5];
    


//    // testing outputs for serial print // to be deleted
//    f_u0 = throttle_and_u[0];
//    f_u1 = throttle_and_u[1];
//    f_u2 = throttle_and_u[2];
//
//    fq0y = q0_y;
//    f_y0 = y[0];
//    f_y1 = y[0];
//    f_y2 = y[0];
//    f_y3 = y[0];
//    f_y4 = y[0];
//    f_y5 = y[0];
}

#endif
