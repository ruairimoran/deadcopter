// {{timestamp}}

#ifndef fly.h
#define fly.h

#include <Arduino.h>
#include <math.h>
#include <MadgwickAHRS.h>
#include "imu.h"
#include "receiver.h"

class Fly {
    private:
    float Ad[9][9] = {{discrete_A}};  // discrete A array
    float Bd[9][3] = {{discrete_B}};  // discrete B array
    float Cd[6][9] = {{discrete_C}};  // discrete C array
    float K_x[3][9] = {{lqr_K_x_gain}};  // LQR gain for state control
    float K_z[3][6] = {{lqr_K_z_gain}};  // LQR gain for integral action
    float L[9][6] = {{kf_gain}};  // Kalman filter gain for minimising observation error
    float G[12][3] = {{equilibrium_G}};  // for calculating new equilibrium state and control action

    float throttle_and_u[4] = {0};  // control action on quadcopter (throttle, u_x, u_y, u_z)
    float r[6] = {0};  // control input from receiver (q1, q2, q3, 0, 0, 0)
    float r_short[3] = {0};  // for solving equilibrium faster
    float z[6] = {0};  // storing integral values
    float y[6] = {0};  // IMU values
    float y_hat[6] = {0};  // observed IMU values
    float x_hat[9] = {0};  // observed state
    float x_hat_buffer[9] = {0};  // for using x_hat when calculating x_hat
    float xu_e[12] = {0};  // for equilibrium state
    float y_diff[6] = {0};  // for full y_hat - y
    float x_diff[9] = {0};  // for full x_hat - x_e

    // for formatting into output to motor
    float output_to_motor[4] = {0};  // motor pwm from calculations
    float motor_proportions[4][4] = {{ '{{' }}1.0, 1.0, {{ '-' }}1.0, {{ '-' }}1.0},
                                     {1.0, {{ '-' }}1.0, {{ '-' }}1.0, 1.0},
                                     {1.0, 1.0, 1.0, 1.0},
                                     {1.0, {{ '-' }}1.0, 1.0, {{ '-' }}1.0{{ '}}' }};  // motor_speeds = motor_proportions * throttle_and_control

    // for solving quaternion differences
    float solve_q0(float q1, float q2, float q3);
    float quaternion_difference(float w1, float x1, float y1, float z1,
                                float w2, float x2, float y2, float z2,
                                float &w3, float &x3, float &y3, float &z3);
    float q0_y_hat = 0;  // observed y q0
    float q0_y = 0;  // measured q0
    float q0_y_diff = 0;  // q0 of y_hat - y
    float q0_x_hat = 0;  // observed x q0
    float q0_x_e = 0;  // x_equilibrium
    float q0_x_diff = 0;  // q0 of x_hat - x_e

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

float invSqrt(float input) {
    // Fast inverse square-root
    // See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
	float half_input = 0.5f * input;
	float output = input;
	long i = *(long*)&output;
	i = 0x5f3759df - (i>>1);
	output = *(float*)&i;
	output = output * (1.5f - (half_input * output * output));  // first Newton Raphson iteration
	output = output * (1.5f - (half_input * output * output));  // second Newton Raphson iteration
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
    {% for count in range(6) -%}
    z[{{count}}] += r[{{count}}] - y[{{count}}];
    {% endfor %}

    // find x and u equilibrium values
    {% for count in range(3) -%}
    r_short[{{count}}] = r[{{count}}];
    {% endfor -%}
    {% for row in range(12) -%}
    xu_e[{{row}}] =
        {%- for col in range(3) -%}
        {{' '}}+ G[{{row}}][{{col}}]*r_short[{{col}}]
        {%- endfor -%}
    ;
    {% endfor %}

    // find x difference (x_hat - x_e)
    q0_x_hat = solve_q0(x_hat[0], x_hat[1], x_hat[2]);
    q0_x_e = solve_q0(xu_e[0], xu_e[1], xu_e[2]);
    quaternion_difference(q0_x_hat, x_hat[0], x_hat[1], x_hat[2],
                          q0_x_e, xu_e[0], xu_e[1], xu_e[2],
                          q0_x_diff, x_diff[0], x_diff[1], x_diff[2]);
    {% for count in range(3,9) -%}
    x_diff[{{count}}] = x_hat[{{count}}] - xu_e[{{count}}];
    {% endfor %}

    // find control action matrix, u
    {% for row in range(3) -%}
    throttle_and_u[{{row+1}}] = xu_e[{{row+9}}]
        {%- for col in range(9) -%}
        {{' '}}+ K_x[{{row}}][{{col}}]*x_diff[{{col}}]
        {%- endfor -%}
        {%- for col in range(6) -%}
        {{' '}}+ K_z[{{row}}][{{col}}]*z[{{col}}]
        {%- endfor -%}
    ;
    {% endfor %}

    // format into control output for motors
    throttle_and_u[0] = fly_throttle;  // form matrix of throttle on top of u
    {% for row in range(4) -%}
    output_to_motor[{{row}}] =
        {%- for col in range(4) -%}
        {{' '}}+ motor_proportions[{{row}}][{{col}}]*throttle_and_u[{{col}}]
        {%- endfor -%}
    ;
    {% endfor -%}
    fly_front_left = ceil(output_to_motor[0]);
    fly_front_right = ceil(output_to_motor[1]);
    fly_back_left = ceil(output_to_motor[2]);
    fly_back_right = ceil(output_to_motor[3]);

    // find observed y_hat
    {% for row in range(6) -%}
    y_hat[{{row}}] =
        {%- for col in range(9) -%}
        {{' '}}+ Cd[{{row}}][{{col}}]*x_hat[{{col}}]
        {%- endfor -%}
    ;
    {% endfor %}

    // find y difference (y_hat - y)
    q0_y_hat = solve_q0(y_hat[0], y_hat[1], y_hat[2]);
    quaternion_difference(q0_y_hat, y_hat[0], y_hat[1], y_hat[2],
                          q0_y, y[0], y[1], y[2],
                          q0_y_diff, y_diff[0], y_diff[1], y_diff[2]);
    {% for count in range(3,6) -%}
    y_diff[{{count}}] = y_hat[{{count}}] - y[{{count}}];
    {% endfor %}

    // find observed x_hat
    {% for count in range(9) -%}
    x_hat_buffer[{{count}}] = x_hat[{{count}}];
    {% endfor -%}
    {% for row in range(9) -%}
    x_hat[{{row}}] =
        {%- for col in range(9) -%}
        {{' '}}+ Ad[{{row}}][{{col}}]*x_hat_buffer[{{col}}]
        {%- endfor -%}
        {%- for col in range(3) -%}
        {{' '}}+ Bd[{{row}}][{{col}}]*throttle_and_u[{{col+1}}]
        {%- endfor -%}
        {%- for col in range(6) -%}
        {{' '}}+ L[{{row}}][{{col}}]*y_diff[{{col}}]
        {%- endfor -%}
    ;
    {% endfor %}


    // testing outputs for serial print // to be deleted
    f_u0 = throttle_and_u[1];
    f_u1 = throttle_and_u[2];
    f_u2 = throttle_and_u[3];

    fq0y = q0_y;
    f_y0 = y[0];
    f_y1 = y[0];
    f_y2 = y[0];
    f_y3 = y[0];
    f_y4 = y[0];
    f_y5 = y[0];
}

#endif
