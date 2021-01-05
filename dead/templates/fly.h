// {{timestamp}}

#ifndef fly.h
#define fly.h

#include <Arduino.h>
#include <math.h>
#include <BasicLinearAlgebra.h>
#include <MadgwickAHRS.h>
#include "imu.h"
#include "receiver.h"

class Fly {
    private:
    float Ad[9][9] = {{discrete_A}};  // discrete A array
    BLA::Matrix<9,9> Mat_Ad = Ad;  // discrete A matrix
    float Bd[9][3] = {{discrete_B}};  // discrete B array
    BLA::Matrix<9,3> Mat_Bd = Bd;  // discrete B matrix
    float Cd[6][9] = {{discrete_C}};  // discrete C array
    BLA::Matrix<6,9> Mat_Cd = Cd;  // discrete C matrix
    float K_x[3][9] = {{lqr_K_x_gain}};  // LQR gain for state control
    BLA::Matrix<3,9> Mat_K_x = K_x;
    float K_z[3][6] = {{lqr_K_z_gain}};  // LQR gain for integral action
    BLA::Matrix<3,6> Mat_K_z = K_z;
    float L[9][6] = {{kf_gain}};  // Kalman filter gain for minimising observation error
    BLA::Matrix<9,6> Mat_L = L;
    float G[12][3] = {{equilibrium_G}};  // for calculating new equilibrium state and control action
    BLA::Matrix<12,3> Mat_G = G;

    {% raw %}

    float u[3][1] = {{0}, {0}, {0}};  // control action on quadcopter (u_x, u_y, u_z)
    BLA::Matrix<3,1> Mat_u = u;
    BLA::Matrix<6,1> Mat_r;  // control input from receiver (q1, q2, q3, 0, 0, 0)
    BLA::Matrix<3,1> Mat_r_short;  // for solving equilibrium faster
    float z[6][1] = {{0}, {0}, {0}, {0}, {0}, {0}};  // storing integral values
    BLA::Matrix<6,1> Mat_z = z;
    BLA::Matrix<6,1> Mat_y;  // IMU values
    BLA::Matrix<6,1> Mat_y_hat;  // observed IMU values
    float x_hat[9][1] = {{0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}};  // observed state
    BLA::Matrix<9,1> Mat_x_hat = x_hat;
    BLA::Matrix<12,1> Mat_xu_e;  // for equilibrium state
    BLA::Matrix<6,1> Mat_y_diff;  // for full y_hat - y
    BLA::Matrix<9,1> Mat_x_diff;  // for full x_hat - x_e
    BLA::Matrix<3,1> Mat_u_e;  // for singling out u_e from xu_e

    {% endraw %}

    // for formatting into output to motor
    BLA::Matrix<1,1> Mat_throttle;
    BLA::Matrix<4,1> Mat_throttle_and_u;
    BLA::Matrix<4,1> Mat_output_to_motor;
    float motor_proportions[4][4] = {{ '{{' }}1.0, 1.0, {{ '-' }}1.0, {{ '-' }}1.0},
                                     {1.0, {{ '-' }}1.0, {{ '-' }}1.0, 1.0},
                                     {1.0, 1.0, 1.0, 1.0},
                                     {1.0, {{ '-' }}1.0, 1.0, {{ '-' }}1.0{{ '}}' }};  // motor_speeds = motor_proportions * throttle_and_control
    BLA::Matrix<4,4> Mat_motor_proportions = motor_proportions;

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
                            float Mat_y_negative1, float Mat_y_0, float Mat_y_1, float Mat_y_2,
                            float Mat_y_3, float Mat_y_4, float Mat_y_5);
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
	float half_input = 0.5f * input;
	float output = input;
	long i = *(long*)&output;
	i = 0x5f3759df - (i>>1);
	output = *(float*)&i;
	output = output * (1.5f - (half_input * output * output));
	output = output * (1.5f - (half_input * output * output));
	return output;
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
                             float Mat_y_negative1, float Mat_y_0, float Mat_y_1, float Mat_y_2,
                             float Mat_y_3, float Mat_y_4, float Mat_y_5) {
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
	Mat_r(0,0) = rx_q1 * fly_reciprocalNorm;  // q1
	Mat_r(1,0) = rx_q2 * fly_reciprocalNorm;  // q2
	Mat_r(2,0) = rx_q3 * fly_reciprocalNorm;  // q3
    Mat_r(3,0) = 0;
    Mat_r(4,0) = 0;
    Mat_r(5,0) = 0;

    // put imu values in y matrix
    q0_y = Mat_y_negative1;
    Mat_y(0, 0) = Mat_y_0;
    Mat_y(1, 0) = Mat_y_1;
    Mat_y(2, 0) = Mat_y_2;
    Mat_y(3, 0) = Mat_y_3;
    Mat_y(4, 0) = Mat_y_4;
    Mat_y(5, 0) = Mat_y_5;
}

void Fly::observe_and_control(int fly_throttle, int &fly_front_left, int &fly_front_right, int &fly_back_left, int &fly_back_right,
                              float &f_u0, float &f_u1, float &f_u2, float &fq0y, float &f_y0,
                              float &f_y1, float &f_y2, float &f_y3, float &f_y4, float &f_y5) {
    // integral action
    Mat_z += Mat_r - Mat_y;

    // find x and u equilibrium values
    Mat_r_short(0,0) = Mat_r(0,0);
    Mat_r_short(1,0) = Mat_r(1,0);
    Mat_r_short(2,0) = Mat_r(2,0);
    Multiply(Mat_G, Mat_r_short, Mat_xu_e);

    // find x difference (x_hat - x_e)
    q0_x_hat = solve_q0(Mat_x_hat(0,0), Mat_x_hat(1,0), Mat_x_hat(2,0));
    q0_x_e = solve_q0(Mat_xu_e(0,0), Mat_xu_e(1,0), Mat_xu_e(2,0));
    quaternion_difference(q0_x_hat, Mat_x_hat(0,0), Mat_x_hat(1,0), Mat_x_hat(2,0),
                          q0_x_e, Mat_xu_e(0,0), Mat_xu_e(1,0), Mat_xu_e(2,0),
                          q0_x_diff, Mat_x_diff(0,0), Mat_x_diff(1,0), Mat_x_diff(2,0));
    for(int i=3; i<9; i++) {
        Mat_x_diff(i,0) = Mat_x_hat(i,0) - Mat_xu_e(i,0);
    }

    // single out u_e
    Mat_u_e(0,0) = Mat_xu_e(9,0);
    Mat_u_e(1,0) = Mat_xu_e(10,0);
    Mat_u_e(2,0) = Mat_xu_e(11,0);

    // find control action matrix, u
    Mat_u = Mat_u_e + Mat_K_x*Mat_x_diff + Mat_K_z*Mat_z;

    // format into control output for motors
    Mat_throttle(0,0) = fly_throttle;
    Mat_throttle_and_u = Mat_throttle && Mat_u;  // form matrix of throttle on top of u
    Multiply(Mat_motor_proportions, Mat_throttle_and_u, Mat_output_to_motor);
    fly_front_left = ceil(Mat_output_to_motor(0,0));
    fly_front_right = ceil(Mat_output_to_motor(1,0));
    fly_back_left = ceil(Mat_output_to_motor(2,0));
    fly_back_right = ceil(Mat_output_to_motor(3,0));

    // find observed y_hat
    Multiply(Mat_Cd, Mat_x_hat, Mat_y_hat);

    // find y difference (y_hat - y)
    q0_y_hat = solve_q0(Mat_y_hat(0,0), Mat_y_hat(1,0), Mat_y_hat(2,0));
    quaternion_difference(q0_y_hat, Mat_y_hat(0,0), Mat_y_hat(1,0), Mat_y_hat(2,0),
                          q0_y, Mat_y(0,0), Mat_y(1,0), Mat_y(2,0),
                          q0_y_diff, Mat_y_diff(0,0), Mat_y_diff(1,0), Mat_y_diff(2,0));
    for(int i=3; i<6; i++) {
        Mat_y_diff(i,0) = Mat_y_hat(i,0) - Mat_y(i,0);
    }

    // find observed x_hat
    Mat_x_hat = Mat_Ad*Mat_x_hat + Mat_Bd*Mat_u + Mat_L*Mat_y_diff;



    // testing outputs for serial print // to be deleted
    f_u0 = Mat_u(0,0);
    f_u1 = Mat_u(1,0);
    f_u2 = Mat_u(2,0);

    fq0y = q0_y;
    f_y0 = Mat_y(0,0);
    f_y1 = Mat_y(1,0);
    f_y2 = Mat_y(2,0);
    f_y3 = Mat_y(3,0);
    f_y4 = Mat_y(4,0);
    f_y5 = Mat_y(5,0);
}

#endif
