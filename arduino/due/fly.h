// 2020-12-27 23:34:16.352123

#ifndef fly.h
#define fly.h

#include <Arduino.h>
#include <math.h>
#include <BasicLinearAlgebra.h>
#include "imu.h"
#include "receiver.h"

class Fly {
    private:
    float Ad[9][9] = {{ 1.00000000e+00,0.00000000e+00,0.00000000e+00,2.10084034e-03
, 0.00000000e+00,0.00000000e+00,1.17469529e-05,0.00000000e+00
, 0.00000000e+00},
 { 0.00000000e+00,1.00000000e+00,0.00000000e+00,0.00000000e+00
, 2.10084034e-03,0.00000000e+00,0.00000000e+00,1.10905055e-05
, 0.00000000e+00},
 { 0.00000000e+00,0.00000000e+00,1.00000000e+00,0.00000000e+00
, 0.00000000e+00,2.10084034e-03,0.00000000e+00,0.00000000e+00
,-7.08490452e-06},
 { 0.00000000e+00,0.00000000e+00,0.00000000e+00,1.00000000e+00
, 0.00000000e+00,0.00000000e+00,1.09637883e-02,0.00000000e+00
, 0.00000000e+00},
 { 0.00000000e+00,0.00000000e+00,0.00000000e+00,0.00000000e+00
, 1.00000000e+00,0.00000000e+00,0.00000000e+00,1.03511060e-02
, 0.00000000e+00},
 { 0.00000000e+00,0.00000000e+00,0.00000000e+00,0.00000000e+00
, 0.00000000e+00,1.00000000e+00,0.00000000e+00,0.00000000e+00
,-6.61255682e-03},
 { 0.00000000e+00,0.00000000e+00,0.00000000e+00,0.00000000e+00
, 0.00000000e+00,0.00000000e+00,8.86877849e-01,0.00000000e+00
, 0.00000000e+00},
 { 0.00000000e+00,0.00000000e+00,0.00000000e+00,0.00000000e+00
, 0.00000000e+00,0.00000000e+00,0.00000000e+00,8.86877849e-01
, 0.00000000e+00},
 { 0.00000000e+00,0.00000000e+00,0.00000000e+00,0.00000000e+00
, 0.00000000e+00,0.00000000e+00,0.00000000e+00,0.00000000e+00
, 8.86877849e-01}};  // discrete A array
    BLA::Matrix<9,9> Mat_Ad = Ad;  // discrete A matrix
    float Bd[9][3] = {{5.53298612e-05,0.00000000e+00,0.00000000e+00},
{0.00000000e+00,5.22378983e-05,0.00000000e+00},
{0.00000000e+00,0.00000000e+00,8.99831983e-04},
{7.82347063e-02,0.00000000e+00,0.00000000e+00},
{0.00000000e+00,7.38627668e-02,0.00000000e+00},
{0.00000000e+00,0.00000000e+00,8.41223712e-01},
{1.31843868e+01,0.00000000e+00,0.00000000e+00},
{0.00000000e+00,1.31843868e+01,0.00000000e+00},
{0.00000000e+00,0.00000000e+00,1.31843868e+01}};  // discrete B array
    BLA::Matrix<9,3> Mat_Bd = Bd;  // discrete B matrix
    float Cd[6][9] = {{1.,0.,0.,0.,0.,0.,0.,0.,0.},
{0.,1.,0.,0.,0.,0.,0.,0.,0.},
{0.,0.,1.,0.,0.,0.,0.,0.,0.},
{0.,0.,0.,1.,0.,0.,0.,0.,0.},
{0.,0.,0.,0.,1.,0.,0.,0.,0.},
{0.,0.,0.,0.,0.,1.,0.,0.,0.}};  // discrete C array
    BLA::Matrix<6,9> Mat_Cd = Cd;  // discrete C matrix
    float K_x[3][9] = {{-1.01217438e+02, -0.00000000e+00, -0.00000000e+00, -1.78119824e+00
,-0.00000000e+00, -0.00000000e+00, -7.69350998e-02, -0.00000000e+00
,-0.00000000e+00},
 {-0.00000000e+00, -1.01661441e+02, -0.00000000e+00, -0.00000000e+00
,-1.83350638e+00, -0.00000000e+00, -0.00000000e+00, -7.66691353e-02
,-0.00000000e+00},
 {-0.00000000e+00, -0.00000000e+00, -1.01777465e+02, -0.00000000e+00
,-0.00000000e+00, -1.27427509e+00, -0.00000000e+00, -0.00000000e+00
, 6.73803082e-03}};  // LQR gain for state control
    BLA::Matrix<3,9> Mat_K_x = K_x;
    float K_z[3][6] = {{ 2.08633533e-01, -0.00000000e+00, -0.00000000e+00,3.83308332e-06
,-0.00000000e+00, -0.00000000e+00},
 {-0.00000000e+00,2.09456828e-01, -0.00000000e+00, -0.00000000e+00
, 3.58623084e-06, -0.00000000e+00},
 {-0.00000000e+00, -0.00000000e+00,2.10885753e-01, -0.00000000e+00
,-0.00000000e+00,3.60794168e-12}};  // LQR gain for integral action
    BLA::Matrix<3,6> Mat_K_z = K_z;
    float L[9][6] = {{-6.18041474e-01, -0.00000000e+00, -0.00000000e+00, -1.58736062e-03
,-0.00000000e+00, -0.00000000e+00},
 {-0.00000000e+00, -6.18041473e-01, -0.00000000e+00, -0.00000000e+00
,-1.58725628e-03, -0.00000000e+00},
 {-0.00000000e+00, -0.00000000e+00, -6.18041467e-01, -0.00000000e+00
,-0.00000000e+00, -1.58674863e-03},
 {-1.67375157e-03, -0.00000000e+00, -0.00000000e+00, -6.56353382e-01
,-0.00000000e+00, -0.00000000e+00},
 {-0.00000000e+00, -1.67314940e-03, -0.00000000e+00, -0.00000000e+00
,-6.56300753e-01, -0.00000000e+00},
 {-0.00000000e+00, -0.00000000e+00, -1.67021931e-03, -0.00000000e+00
,-0.00000000e+00, -6.56044742e-01},
 {-2.81172513e-04, -0.00000000e+00, -0.00000000e+00, -2.49486407e-02
,-0.00000000e+00, -0.00000000e+00},
 {-0.00000000e+00, -2.65548314e-04, -0.00000000e+00, -0.00000000e+00
,-2.35610436e-02, -0.00000000e+00},
 {-0.00000000e+00, -0.00000000e+00,1.69914230e-04, -0.00000000e+00
,-0.00000000e+00,1.50719204e-02}};  // Kalman filter gain for minimising observation error
    BLA::Matrix<9,6> Mat_L = L;
    float G[12][6] = {{ 1.00000000e+00,0.00000000e+00,0.00000000e+00,0.00000000e+00
, 0.00000000e+00,0.00000000e+00},
 { 0.00000000e+00,1.00000000e+00,0.00000000e+00,0.00000000e+00
, 0.00000000e+00,0.00000000e+00},
 { 0.00000000e+00,0.00000000e+00,1.00000000e+00,0.00000000e+00
, 0.00000000e+00,0.00000000e+00},
 { 0.00000000e+00,0.00000000e+00,0.00000000e+00,9.99995586e-01
,-3.10027686e-23,1.97284625e-22},
 { 0.00000000e+00,0.00000000e+00,0.00000000e+00, -1.28749008e-19
, 9.99995586e-01,1.88716618e-16},
 { 0.00000000e+00,0.00000000e+00,0.00000000e+00,0.00000000e+00
,-8.68919801e-18,9.99995586e-01},
 { 0.00000000e+00,0.00000000e+00,0.00000000e+00, -1.89666537e-04
, 1.19348975e-15, -7.71154780e-15},
 { 0.00000000e+00,0.00000000e+00,0.00000000e+00,1.71446727e-17
,-2.00892654e-04, -1.26010313e-14},
 { 0.00000000e+00,0.00000000e+00,0.00000000e+00, -5.87669361e-16
,-1.78079051e-13, -3.64635015e-03},
 { 0.00000000e+00,0.00000000e+00,0.00000000e+00, -1.62701602e-06
,-1.11022302e-16, -6.04519300e-17},
 { 0.00000000e+00,0.00000000e+00,0.00000000e+00,1.48921997e-19
,-1.72335434e-06, -1.11022302e-16},
 { 0.00000000e+00,0.00000000e+00,0.00000000e+00, -5.04204124e-18
,-1.49010076e-15, -3.12859113e-05}};  // for calculating new equilibrium state and control action
    BLA::Matrix<12,6> Mat_G = G;
    BLA::Matrix<3,1> Mat_u;  // control action on quadcopter (u_x, u_y, u_z)
    BLA::Matrix<6,1> Mat_r;  // control input from receiver (q1, q2, q3, 0, 0, 0)
    BLA::Matrix<6,1> Mat_z;  // storing integral values
    BLA::Matrix<6,1> Mat_y;  // IMU values
    BLA::Matrix<6,1> Mat_y_hat;  // observed IMU values
    BLA::Matrix<9,1> Mat_x_hat;  // observed state
    BLA::Matrix<12,1> Mat_xu_e;  // for equilibrium state
    BLA::Matrix<6,1> Mat_y_diff;  // for full y_hat - y
    BLA::Matrix<9,1> Mat_x_diff;  // for full x_hat - x_e
    BLA::Matrix<3,1> Mat_u_e;  // for singling out u_e from xu_e

    // for solving matrix equations
    BLA::Matrix<9,1> Ad__x_hat;
    BLA::Matrix<9,1> Bd__u;
    BLA::Matrix<9,1> L__y_diff;
    BLA::Matrix<3,1> K_x__x_diff;
    BLA::Matrix<3,1> K_z__z;

    // for solving quaternion differences
    float solve_q0(float q1, float q2, float q3);
    float quaternion_difference(float w1, float x1, float y1, float z1, float w2, float x2, float y2, float z2);
    float q0_y_hat;
    float q0_y;
    float q0_y_diff;
    float q0_x_hat;
    float q0_x_e;  // x_equilibrium
    float q0_x_diff;

    public:
    Fly();
    float observe_and_control(class Imu fly_imu, class Receiver fly_receiver);

};

/*--------------------------------------------------------------------------------------------------------------------*/

Fly::Fly() : Mat_u{0, 0, 0}, Mat_x_hat{0, 0, 0, 0, 0, 0, 0, 0, 0}, Mat_z{0, 0, 0, 0, 0, 0} {

}

float Fly::solve_q0(float q1, float q2, float q3) {
    // find q0 for unit quaternion
    return sqrtf(1 - powf(q1,2) - powf(q2,2) - powf(q3,2));
}

float Fly::quaternion_difference(float w1, float x1, float y1, float z1, float w2, float x2, float y2, float z2) {
    // form conjugate of second quaternion
    x2 = -x2;
    y2 = -y2;
    z2 = -z2;
    // multiply first quaternion by conjugate of second quaternion
    float w3 = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    float x3 = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    float y3 = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    float z3 = w1*z2 + x1*y2 - y1*x2 + z1*w2;
    return w3, x3, y3, z3;
}

float Fly::observe_and_control(class Imu fly_imu, class Receiver fly_receiver) {
    // get control values from receiver
    float phi = fly_receiver.rx_roll / 2.0f;
    float theta = fly_receiver.rx_pitch / 2.0f;
    float psi = fly_receiver.rx_rudder / 2.0f;
    Mat_r(0,0) = sin(phi)*cos(theta)*cos(psi) - cos(phi)*sin(theta)*sin(psi);  // q1
    Mat_r(1,0) = cos(phi)*sin(theta)*cos(psi) + sin(phi)*cos(theta)*sin(psi);  // q2
    Mat_r(2,0) = cos(phi)*cos(theta)*sin(psi) - sin(phi)*sin(theta)*cos(psi);  // q3
    Mat_r(3,0) = 0;
    Mat_r(4,0) = 0;
    Mat_r(5,0) = 0;

    // get angle measurement values from imu
    q0_y, Mat_y(0,0), Mat_y(1,0), Mat_y(2,0), Mat_y(3,0), Mat_y(4,0), Mat_y(5,0) = fly_imu.update_imu_data();

    // find observed y_hat
    Multiply(Mat_Cd, Mat_x_hat, Mat_y_hat);

    // find y difference (y_hat - y)
    q0_y_hat = solve_q0(Mat_y_hat(0,0), Mat_y_hat(1,0), Mat_y_hat(2,0));
    q0_y_diff, Mat_y_diff(0,0), Mat_y_diff(1,0), Mat_y_diff(2,0) = quaternion_difference(q0_y_hat, Mat_y_hat(0,0), Mat_y_hat(1,0), Mat_y_hat(2,0),
                                                                                         q0_y, Mat_y(0,0), Mat_y(1,0), Mat_y(2,0));
    for(int i=3; i<6; i++) {
        Mat_y_diff(i,0) = Mat_y_hat(i,0) - Mat_y(i,0);
    }

    // find observed x_hat
    Multiply(Mat_Ad, Mat_x_hat, Ad__x_hat);
    Multiply(Mat_Bd, Mat_u, Bd__u);
    Multiply(Mat_L, Mat_y_diff, L__y_diff);
    Mat_x_hat = Ad__x_hat + Bd__u + L__y_diff;

    // integral action
    for(int i=0; i<6; i++) {
        Mat_z(i,0) = Mat_z(i,0) + Mat_r(i,0) - Mat_y(i,0);
    }

    // find x and u equilibrium values
    Multiply(Mat_G, Mat_r, Mat_xu_e);

    // find x difference (x_hat - x_e)
    q0_x_hat = solve_q0(Mat_x_hat(0,0), Mat_x_hat(1,0), Mat_x_hat(2,0));
    q0_x_e = solve_q0(Mat_xu_e(0,0), Mat_xu_e(1,0), Mat_xu_e(2,0));
    q0_x_diff, Mat_x_diff(0,0), Mat_x_diff(1,0), Mat_x_diff(2,0) = quaternion_difference(q0_x_hat, Mat_x_hat(0,0), Mat_x_hat(1,0), Mat_x_hat(2,0),
                                                                                         q0_x_e, Mat_xu_e(0,0), Mat_xu_e(1,0), Mat_xu_e(2,0));
    for(int i=3; i<9; i++) {
        Mat_x_diff(i,0) = Mat_x_hat(i,0) - Mat_xu_e(i,0);
    }

    // single out u_e
    Mat_u_e(0,0) = Mat_xu_e(9,0);
    Mat_u_e(1,0) = Mat_xu_e(10,0);
    Mat_u_e(2,0) = Mat_xu_e(11,0);

    // find control action matrix, u
    Multiply(Mat_K_x, Mat_x_diff, K_x__x_diff);
    Multiply(Mat_K_z, Mat_z, K_z__z);
    Mat_u = Mat_u_e + K_x__x_diff + K_z__z;

    return Mat_u(0,0), Mat_u(1,0), Mat_u(2,0);

}

#endif