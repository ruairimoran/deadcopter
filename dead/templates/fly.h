// {{timestamp}}

#ifndef fly.h
#define fly.h

#include <Arduino.h>
#include <math.h>
#include <MatrixMath.h>
#include "imu.h"
#include "receiver.h"

#define Ad {{discrete_A}}  // discrete A matrix
#define Bd {{discrete_B}}  // discrete B matrix
#define Cd {{discrete_C}}  // discrete C matrix
#define K_x {{lqr_K_x_gain}}  // LQR gain for state control
#define K_z {{lqr_K_z_gain}}  // LQR gain for integral action
#define L {{kf_gain}}  // Kalman filter gain for minimising observation error
#define G {{equilibrium_G}}  // for calculating new equilibrium state and control action

class Fly {
    private:
    int u[3];  // control action on quadcopter (u_x, u_y, u_z)
    float r[6];  // control input from receiver (q1, q2, q3, 0, 0, 0)
    float z[6];  // storing integral values
    float y[6];  // IMU values
    float y_hat[6];  // observed IMU values
    float x_hat[9];  // observed state

    public:
    Fly();

}

/*--------------------------------------------------------------------------------------------------------------------*/

Fly::Fly() {
    u = (0, 0, 0);
    x_hat = (0, 0, 0, 0, 0, 0, 0, 0, 0);
    z = (0, 0, 0, 0, 0, 0);
}

void observe_and_control(void) {
    // get imu values
    y = Imu.update_imu_data();

    // get control input from receiver
    phi = Receiver.rx_roll / 2.0f;
    theta = Receiver.rx_pitch / 2.0f;
    psi = Receiver.rx_rudder / 2.0f;
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
    x_hat[0] =

    // control

}

#endif
