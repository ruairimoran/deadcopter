from pyquaternion import Quaternion
import numpy as np


class DeadCopter:

    def __init__(self, **kwargs):
        # SYSTEM PARAMETERS
        # general
        self.m = 1.5  # kg                                      # total mass of aircraft
        self.L = 0.225  # m                                     # quad arm length
        self.Nmotor = 4  # motors                               # no. of motors
        self.Ixx = 0.032  # kg.m^2                              # moment of inertia xx
        self.Iyy = 0.032  # kg.m^2                              # moment of inertia yy
        self.Ixx = 0.058  # kg.m^2                              # moment of inertia zz
        self.g = 9.81  # m/s^2                                  # acceleration of gravity
        self.A_rho = 1.225  # kg/m^3                            # air density at sea level, 15 deg C

        # motors
        self.Kv = 1000  # rpm/V                                 # motor speed constant
        self.tau_m = 35  # ms                                   # motor time constant
        self.m_r = 10  # g                                      # rotor mass
        self.R_r = 15  # mm                                     # rotor radius
        self.m_m = 50  # g                                      # total mass of motor

        # props
        self.C_T = 0.1                                          # thrust coefficient
        self.C_P = 0.04                                         # power coefficient
        self.m_p = 10  # g                                      # prop mass
        self.D_p = 9  # in                                      # prop diameter

        self.Icm = np.diagflat([1, 1, 1])
        self.Icm_inv = np.linalg.inv(self.Icm)
        self.k1 = 1
        self.k2 = 1
        self.tau = 1
        # include the system state

    def dynamics(self, state, u):
        # define
        q = Quaternion(state[0:4])                              # q = quaternion
        w = state[4:7]                                          # w = omega
        n = state[7:10]                                         # n = ?
        c = np.array(u)                                         # c = control input

        w_quat = Quaternion([0] + w)
        q_dot = 0.5 * q * w_quat

        w_dot = self.Icm_inv * (self.tau - np.cross(w, self.Icm*w))

        n_dot = self.k2 * (self.k1 * c - n)

        return {'q_': q_dot, 'w_': w_dot, 'n_': n_dot}