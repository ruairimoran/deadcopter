from pyquaternion import *
import numpy as np
from scipy.integrate import solve_ivp
import control as C


class DeadCopter:

    def __init__(self, **kwargs):
        # system state [q0, q1, q2, q3, wx, wy, wz, nx, ny, nz]
        self.__state = np.array([1] + [0] * 9)

        # general
        self.__mass = 1 # kg                           # total mass of aircraft
        self.__arm_length = 0.225  # m                    # quad arm length
        self.__num_motors = 4  # motors                   # no. of motors
        self.__moi_xx = 0.01788  # kg.m^2                   # moment of inertia xx
        self.__moi_yy = 0.03014  # kg.m^2                   # moment of inertia yy
        self.__moi_zz = 0.04614  # kg.m^2                   # moment of inertia zz
        self.__moi = np.diagflat(
            [self.__moi_xx, self.__moi_yy, self.__moi_zz])    # inertia matrix
        self.__gravity_acc = 9.81  # m/s^2                # acceleration of gravity
        self.__air_density = 1.225  # kg/m^3              # air density at sea level, 15 deg C

        # motors
        self.__K_v = 1000  # rpm/V                        # motor speed constant
        self.__motor_time_constant = 50 / 1000  # s       # motor time constant
        self.__rotor_mass = 40 / 1000  # kg               # rotor mass
        self.__rotor_radius = 19 / 1000  # m              # rotor radius
        self.__motor_mass = 112 / 1000  # kg              # total mass of motor
        self.__voltage_max = 16.8  # V                    # max voltage to motor
        self.__voltage_min = 15  # V                      # min voltage to motor

        # props
        self.__thrust_coeff = 0.112                        # thrust coefficient
        self.__power_coeff = 0.044                         # power coefficient
        self.__prop_mass = 9 / 1000  # kg                 # prop mass
        self.__prop_diameter_in = 10                      # prop diameter in inches

        # noise
        self.__disturbance_level = 1e-3

        # parse `kwargs` and set as attribute, provided the keyword corresponds
        # to one of the variables defined above
        for key in kwargs:
            masqueraded_attribute = f"_{self.__class__.__name__}__{key}"
            if masqueraded_attribute in self.__dict__.keys():
                setattr(self, masqueraded_attribute, kwargs[key])
            else:
                raise Exception(f"unknown argument: {key}")

        self.__compute_parameters()

    def __compute_parameters(self):
        # modelling
        self.__disturbance_covariance = np.diagflat([self.__disturbance_level] * 3)
        self.__prop_diameter_m = self.__prop_diameter_in * 0.0254                                   # prop diameter in meters
        self.__motor_moi = self.__rotor_mass * (self.__rotor_radius**2)
        self.__prop_moi = (self.__prop_mass * self.__prop_diameter_m ** 2) / 12  # kg.m^2           # prop moment of inertia
        self.__n_h = np.sqrt((self.__mass * self.__gravity_acc) /
                             (self.__num_motors * self.__thrust_coeff
                              * self.__air_density * (self.__prop_diameter_m ** 4)))
        self.__k1 = (self.__K_v * (self.__voltage_max - self.__voltage_min)) / 60  # /60 for rps
        self.__k2 = 1 / self.__motor_time_constant
        self.__k3_x = (2 * self.__n_h * self.__thrust_coeff * self.__air_density * (self.__prop_diameter_m ** 4) \
            * self.__num_motors * self.__arm_length) / ((2**0.5) * self.__moi_xx)
        self.__k3_y = (2 * self.__n_h * self.__thrust_coeff * self.__air_density * (self.__prop_diameter_m ** 4) \
            * self.__num_motors * self.__arm_length) / ((2**0.5) * self.__moi_yy)
        self.__k3_z = (2 * self.__n_h * self.__power_coeff * self.__air_density * (self.__prop_diameter_m ** 5) \
            * self.__num_motors) / (2 * np.pi * self.__moi_zz)
        self.__k4_xy = 0
        self.__k4_z = (2 * np.pi * self.__num_motors * (self.__prop_moi + self.__motor_moi)) / self.__moi_zz
        self.__gamma_n = np.diagflat([self.__k3_x, self.__k3_y, self.__k3_z - (self.__k4_z * self.__k2)])
        self.__gamma_u = np.diagflat([0, 0, self.__k4_z * self.__k2 * self.__k1])

    @property
    def mass(self):
        return self.__mass

    @property
    def state(self):
        return self.__state

    @state.setter  # for reset
    def state(self, vector):
        self.__state = vector

    @property
    def quaternion(self):
        return self.__state[0:4]

    @property
    def hover_rpm(self):
        return self.__n_h

    def controllability(self, a, b, n):
        ctrb_rank = np.linalg.matrix_rank(C.ctrb(a, b))
        if ctrb_rank < n:
            raise Exception(f"System not controllable. Ctrb Matrix Rank ({ctrb_rank}) < States ({n})")

    def observability(self, a, c, n):
        obsv_rank = np.linalg.matrix_rank(C.obsv(a, c))
        if obsv_rank < n:
            raise Exception(f"System not observable. Obsv Matrix Rank ({obsv_rank}) < Measured States ({n})")

    def LQR(self, a, b):  # 125 Hz
        Q_lqr = np.diagflat([4000, 4000, 1000, 1, 1, 1, 1, 1, 1])  # a.shape[0]
        R_lqr = np.diagflat([1, 1, 1])  # b.shape[1]
        solution_P_lqr, eigenvalues_cl_lqr, negative_gain_K_lqr = C.dare(a, b, Q_lqr, R_lqr)
        return -negative_gain_K_lqr

    def Kf(self, a, c):
        Q_Kf = np.diagflat([1, 1, 1, 1, 1, 1, 500, 500, 500])  # a.shape[0]
        R_Kf = np.diagflat([1, 1, 1, 1, 1, 1])  # c.shape[0]
        solution_P_Kf, eigenvalues_cl_Kf, negative_gain_L_Kf = C.dare(a.T, c.T, Q_Kf, R_Kf)
        return -negative_gain_L_Kf.T

    def normalise_quaternion(self, non_norm_quaternion):
        norm = Quaternion(non_norm_quaternion).norm
        return non_norm_quaternion / norm

    def solve_q0(self, vector):
        q0 = np.sqrt(1 - vector[0]**2 - vector[1]**2 - vector[2]**2)
        return q0

    def euler_angles(self, measured_quaternion):
        """
        Documentation
        :return:
        """
        if measured_quaternion != 0:
            if np.shape(measured_quaternion) != (4,):
                raise Exception(f"Error with quaternion input: {measured_quaternion}")

        q = measured_quaternion
        if measured_quaternion == 0:
            q = self.quaternion

        phi = np.arctan2(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]**2 + q[2]**2))
        theta = np.arcsin(2*(q[0]*q[2] - q[1]*q[3]))
        psi = np.arctan2(2*(q[0]*q[3] + q[2]*q[3]), 1 - 2*(q[2]**2 + q[3]**2))
        return np.array([phi, theta, psi])

    def linearisation(self):
        a = np.zeros(shape=(9, 9))
        b = np.zeros(shape=(9, 3))
        c = np.zeros(shape=(6, 9))
        for i in range(3):
            a[i, 3+i] = 0.5
            a[3+i, 6+i] = self.__gamma_n[i, i]
            a[6+i, 6+i] = -self.__k2
            b[3+i, i] = self.__gamma_u[i, i]
            b[6+i, i] = self.__k2 * self.__k1

        for i in range(6):
            c[i, i] = 1

        return a, b, c

    def discretisation(self, dt):
        a, b, c = self.linearisation()  # return linearised dynamics matrices
        continuous_system = C.ss(a, b, c, 0)
        discrete_system = C.c2d(continuous_system, dt)
        Ad = discrete_system.A
        Bd = discrete_system.B
        Cd = discrete_system.C
        return Ad, Bd, Cd


    def linear_fly_simulate(self, u, dt):
        def linear_dynamics(_t, reduced_state):
            control_action = np.asarray(u).reshape(3,)
            x = reduced_state[0:3]
            w = reduced_state[3:6]
            n = reduced_state[6:9]
            x_dot = 0.5*w
            w_dot = self.__gamma_n@n + self.__gamma_u@control_action
            n_dot = -self.__k2*n + self.__k1*self.__k2*control_action
            return x_dot.tolist() + w_dot.tolist() + n_dot.tolist()

        initial_state = self.__state
        solution = solve_ivp(linear_dynamics,
                             [0, dt],
                             initial_state[1:10])
        updated_reduced_state = solution.y[:, -1]
        q0 = self.solve_q0(updated_reduced_state[0:3])
        self.__state = [q0] + updated_reduced_state.tolist()

    def fly_simulate(self, u, dt):
        def dynamics(_t, state):
            control_action = np.asarray(u).reshape(3,)  # control input
            attitude_quat = Quaternion(state[0:4])  # attitude as a quaternion
            angular_freq = state[4:7]  # angular frequencies
            rotor_freq = state[7:10]  # rotor frequencies
            angular_freq_quat = Quaternion(np.append(0, angular_freq))
            attitude_quat_dot = list(0.5 * attitude_quat * angular_freq_quat)
            af1 = np.diag(self.__moi * angular_freq)  # af = angular_freq  # flatten 3x3 array to 1x3
            angular_freq_cross = np.cross(angular_freq, af1)
            angular_freq_dot = list(np.diag(self.__gamma_n * rotor_freq + self.__gamma_u * control_action
                                            - np.diagflat(1 / np.diag(self.__moi)) * angular_freq_cross))
            rotor_freq_dot = list(self.__k2 * (self.__k1 * control_action - rotor_freq))
            return attitude_quat_dot + angular_freq_dot + rotor_freq_dot

        solution = solve_ivp(dynamics,
                             [0, dt],
                             self.__state)
        self.__state = solution.y[:, -1] + [0.0001]  # add constant disturbance
        state_noise = np.random.multivariate_normal(np.zeros(3,), self.__disturbance_covariance)
        self.__state[7:10] += state_noise
        self.__state[0:4] = self.normalise_quaternion(self.__state[0:4])
