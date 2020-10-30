from pyquaternion import Quaternion
import numpy as np


class DeadCopter:

    def __init__(self, **kwargs):
        # SYSTEM PARAMETERS
        # general
        self.m = 1.5  # kg                                      # total mass of aircraft
        self.L = 0.225  # m                                     # quad arm length
        self.Nmotor = 4  # motors                               # no. of motors
        self.I_xx = 0.032  # kg.m^2                             # moment of inertia xx
        self.I_yy = 0.032  # kg.m^2                             # moment of inertia yy
        self.I_zz = 0.058  # kg.m^2                             # moment of inertia zz
        self.I = np.diagflat([self.I_xx, self.I_yy, self.I_zz]) # inertia matrix
        self.g = 9.81  # m/s^2                                  # acceleration of gravity
        self.A_rho = 1.225  # kg/m^3                            # air density at sea level, 15 deg C

        # motors
        self.K_v = 1000  # rpm/V                                # motor speed constant
        self.tau_m = 35  # ms                                   # motor time constant
        self.m_r = 10  # g                                      # rotor mass
        self.R_r = 15  # mm                                     # rotor radius
        self.m_m = 50  # g                                      # total mass of motor
        self.V_max = 14  # V                                    # max voltage to motor
        self.V_min = 10  # V                                    # min voltage to motor
        self.I_m = 1  # kg.m^2                                  # motor moment of inertia

        # props
        self.C_T = 0.1                                          # thrust coefficient
        self.C_P = 0.04                                         # power coefficient
        self.m_p = 10  # g                                      # prop mass
        self.D_p = 9  # in                                      # prop diameter
        self.I_p = 1  # kg.m^2                                  # prop moment of inertia

        # modelling
        self.n_h = ((self.m*self.g)/(self.Nmotor*self.C_T*self.A_rho*(self.D_p**4)))**0.5
        self.k1 = (self.K_v * (self.V_max - self.V_min))/60     # /60 for rps
        self.k2 = 1 / self.tau_m
        self.k3_x = (2*self.n_h) * (self.C_T*self.A_rho*(self.D_p**4)) * (self.Nmotor*self.L) / ((2**0.5)*self.I_xx)
        self.k3_y = (2*self.n_h) * (self.C_T*self.A_rho*(self.D_p**4)) * (self.Nmotor*self.L) / ((2**0.5)*self.I_yy)
        self.k3_z = (2*self.n_h) * (self.C_P*self.A_rho*(self.D_p**5)) * self.Nmotor / (2*np.pi*self.I_zz)
        self.k4_xy = 0
        self.k4_z = 2*np.pi*self.Nmotor*(self.I_p + self.I_m) / self.I_zz
        self.gamma_n = np.diagflat([self.k3_x, self.k3_y, self.k3_z-(self.k4_z*self.k2)])
        self.gamma_u = np.diagflat([0, 0, self.k4_z*self.k2*self.k1])
        # include the system state ??

    def dynamics(self, state, u):
        # define
        q = Quaternion(state[0:4])                              # q = quaternion / attitude
        w = state[4:7]                                          # w = omega / angular frequencies
        n = state[7:10]                                         # n = rotor frequencies
        c = np.array(u)                                         # c = control input

        w_quat = Quaternion([0] + w)
        q_dot = list(0.5 * q * w_quat)

        w1 = np.diag(self.I*w)                                  # flatten 3x3 array to 1x3
        w_cross = np.cross(w, w1)
        w_dot = list(np.diag(self.gamma_n*n + self.gamma_u*u - np.linalg.inv(self.I)*w_cross))

        n_dot = list(self.k2*(self.k1*c - n))

        dynamics = list(q_dot + w_dot + n_dot)

        return [dynamics]