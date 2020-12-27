// {{timestamp}}

#ifndef fly.h
#define fly.h

#include <Arduino.h>
#include <math.h>
#include "imu.h"
#include "receiver.h"

class Fly {
    private:
    float Ad[9][9] = {{discrete_A}};  // discrete A matrix
    float Bd[9][3] = {{discrete_B}};  // discrete B matrix
    float Cd[6][9] = {{discrete_C}};  // discrete C matrix
    float K_x[3][9] = {{lqr_K_x_gain}};  // LQR gain for state control
    float K_z[3][6] = {{lqr_K_z_gain}};  // LQR gain for integral action
    float L[9][6] = {{kf_gain}};  // Kalman filter gain for minimising observation error
    float G[12][6] = {{equilibrium_G}};  // for calculating new equilibrium state and control action
    int u[3];  // control action on quadcopter (u_x, u_y, u_z)
    float r[6];  // control input from receiver (q1, q2, q3, 0, 0, 0)
    float z[6];  // storing integral values
    float y[6];  // IMU values
    float y_hat[6];  // observed IMU values
    float x_hat[9];  // observed state

    public:
    Fly();
    float observe_and_control(class Imu fly_imu, class Receiver fly_receiver);

};

/*--------------------------------------------------------------------------------------------------------------------*/

Fly::Fly() {

}

float Fly::observe_and_control(class Imu fly_imu, class Receiver fly_receiver) {
    // get imu values
    y[0], y[1], y[2], y[3], y[4], y[5] = fly_imu.update_imu_data();

    // get control input from receiver
    float phi = fly_receiver.rx_roll / 2.0f;
    float theta = fly_receiver.rx_pitch / 2.0f;
    float psi = fly_receiver.rx_rudder / 2.0f;
    r[0] = sin(phi)*cos(theta)*cos(psi) - cos(phi)*sin(theta)*sin(psi);
    r[1] = cos(phi)*sin(theta)*cos(psi) + sin(phi)*cos(theta)*sin(psi);
    r[2] = cos(phi)*cos(theta)*sin(psi) - sin(phi)*sin(theta)*cos(psi);

    // observe
    y_hat[0] = Cd[0][0] * x_hat[0];
    y_hat[1] = Cd[1][1] * x_hat[1];
    y_hat[2] = Cd[2][2] * x_hat[2];
    y_hat[3] = Cd[3][3] * x_hat[3];
    y_hat[4] = Cd[4][4] * x_hat[4];
    y_hat[5] = Cd[5][5] * x_hat[5];
    x_hat[0] = 0;

    // control

}

#endif
