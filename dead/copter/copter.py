from pyquaternion import *
import numpy as np
from scipy.integrate import solve_ivp


class DeadCopter:

    def __init__(self, **kwargs):
        # system state [q0, q1, q2, q3, wx, wy, wz, nx, ny, nz]
        self.__state = np.array([1] + [0] * 9)

        # general
        self.__mass = 1.5  # kg                           # total mass of aircraft
        self.__arm_length = 0.225  # m                    # quad arm length
        self.__num_motors = 4  # motors                   # no. of motors
        self.__moi_xx = 0.032  # kg.m^2                   # moment of inertia xx
        self.__moi_yy = 0.032  # kg.m^2                   # moment of inertia yy
        self.__moi_zz = 0.058  # kg.m^2                   # moment of inertia zz
        self.__moi = np.diagflat(
            [self.__moi_xx, self.__moi_yy, self.__moi_zz])    # inertia matrix
        self.__gravity_acc = 9.81  # m/s^2                # acceleration of gravity
        self.__air_density = 1.225  # kg/m^3              # air density at sea level, 15 deg C

        # motors
        self.__K_v = 1000  # rpm/V                        # motor speed constant
        self.__motor_time_constant = 35 / 1000  # s       # motor time constant
        self.__rotor_mass = 10 / 1000  # kg               # rotor mass
        self.__rotor_radius = 15 / 1000  # m              # rotor radius
        self.__motor_mass = 50 / 1000  # kg               # total mass of motor
        self.__voltage_max = 14  # V                      # max voltage to motor
        self.__voltage_min = 10  # V                      # min voltage to motor
        self.__motor_moi = 1  # kg.m^2                    # motor moment of inertia

        # props
        self.__thrust_coeff = 0.1                         # thrust coefficient
        self.__power_coeff = 0.04                         # power coefficient
        self.__prop_mass = 10 / 1000  # kg                # prop mass
        self.__prop_diameter = 9 * 0.0254  # m            # prop diameter
        self.__prop_moi = 1  # kg.m^2                     # prop moment of inertia

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
        self.__n_h = ((self.__mass * self.__gravity_acc) /
                      np.sqrt(self.__num_motors * self.__thrust_coeff
                              * self.__air_density * (self.__prop_diameter ** 4)))
        self.__k1 = (self.__K_v * (self.__voltage_max - self.__voltage_min)) / 60  # /60 for rps
        self.__k2 = 1 / self.__motor_time_constant
        self.__k3_x = (2 * self.__n_h) * (self.__thrust_coeff * self.__air_density * (self.__prop_diameter ** 4)) \
            * (self.__num_motors * self.__arm_length) / ((2 ** 0.5) * self.__moi_xx)
        self.__k3_y = (2 * self.__n_h) * (self.__thrust_coeff * self.__air_density * (self.__prop_diameter ** 4)) \
            * (self.__num_motors * self.__arm_length) / ((2 ** 0.5) * self.__moi_yy)
        self.__k3_z = (2 * self.__n_h) * (self.__power_coeff * self.__air_density * (self.__prop_diameter ** 5)) \
            * self.__num_motors / (2 * np.pi * self.__moi_zz)
        self.__k4_xy = 0
        self.__k4_z = 2 * np.pi * self.__num_motors * (self.__prop_moi + self.__motor_moi) / self.__moi_zz
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

    def euler_angles(self):
        """
        Documentation
        :return:
        """
        q = self.quaternion
        phi = np.arctan2(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]**2 + q[2]**2))
        theta = np.arcsin(2*(q[0]*q[2] - q[1]*q[3]))
        psi = np.arctan2(2*(q[0]*q[3] + q[2]*q[3]), 1 - 2*(q[2]**2 + q[3]**2))
        return np.array([phi, theta, psi])

    def linearisation(self):
        a = np.zeros(shape=(9, 9))
        b = np.zeros(shape=(9, 1))

        for i in range(3):
            a[i, 3+i] = 0.5
            a[3+i, 6+i] = self.__gamma_n[i, i]
            a[6+i, 6+i] = -self.__k2

        return a, b

    def linear_fly_simulate(self, u, dt):
        def linearised(_t, state):
            quaternion_vector = np.array([[state[0]],
                                          [state[1]],
                                          [state[2]]])
            angular_frequencies = np.array([[state[3]],
                                            [state[4]],
                                            [state[5]]])
            rotor_frequencies = np.array([[state[6]],
                                          [state[7]],
                                          [state[8]]])
            zeros = np.zeros([3, 3])
            identity_3 = np.identity(3)
            a = np.array([[zeros, 0.5*identity_3, zeros],
                          [zeros, zeros, self.__gamma_n],
                          [zeros, zeros, -self.__k2*identity_3]])
            b = np.array([[zeros],
                          [self.__gamma_u],
                          [self.__k2*self.__k1*identity_3]])
            a_quaternion_vector = a[0][0] @ quaternion_vector \
                                + a[0][1] @ angular_frequencies \
                                + a[0][2] @ rotor_frequencies
            a_angular_frequencies = a[1][0] @ quaternion_vector \
                                  + a[1][1] @ angular_frequencies \
                                  + a[1][2] @ rotor_frequencies
            a_rotor_frequencies = a[2][0] @ quaternion_vector \
                                + a[2][1] @ angular_frequencies \
                                + a[2][2] @ rotor_frequencies
            a_state = np.array([a_quaternion_vector[0][0], a_quaternion_vector[1][0], a_quaternion_vector[2][0],
                                a_angular_frequencies[0][0], a_angular_frequencies[1][0], a_angular_frequencies[2][0],
                                a_rotor_frequencies[0][0], a_rotor_frequencies[1][0], a_rotor_frequencies[2][0]])
            ux = np.array([b[0] @ u])
            uy = np.array([b[1] @ u])
            uz = np.array([b[2] @ u])
            b_u = np.array([ux[0][0][0], ux[0][0][1], ux[0][0][2],
                            uy[0][0][0], uy[0][0][1], uy[0][0][2],
                            uz[0][0][0], uz[0][0][1], uz[0][0][2]])
            state_dot = a_state + b_u
            return state_dot

        augmented_state = self.__state[1:10]
        solution = solve_ivp(linearised,
                             [0, dt],
                             augmented_state)

        self.__state[1:10] = solution.y[:, -1]
        not_norm_quaternion = Quaternion(self.__state[0:4]).norm
        self.__state[0:4] = self.__state[0:4] / not_norm_quaternion

    def fly_simulate(self, u, dt):
        def dynamics(_t, state):
            attitude_quat = Quaternion(state[0:4])  # attitude as a quaternion
            angular_freq = state[4:7]  # angular frequencies
            rotor_freq = state[7:10]  # rotor frequencies
            control_action = np.array(u)  # control input
            angular_freq_quat = Quaternion(np.append(0, angular_freq))
            attitude_quat_dot = list(0.5 * attitude_quat * angular_freq_quat)
            af1 = np.diag(self.__moi * angular_freq)  # af = angular_freq  # flatten 3x3 array to 1x3
            angular_freq_cross = np.cross(angular_freq, af1)
            angular_freq_dot = list(np.diag(self.__gamma_n * rotor_freq + self.__gamma_u * u
                                            - np.diagflat(1 / np.diag(self.__moi)) * angular_freq_cross))
            rotor_freq_dot = list(self.__k2 * (self.__k1 * control_action - rotor_freq))
            return attitude_quat_dot + angular_freq_dot + rotor_freq_dot

        solution = solve_ivp(dynamics,
                             [0, dt],
                             self.__state)

        self.__state = solution.y[:, -1]
        not_norm_quaternion = Quaternion(self.__state[0:4]).norm
        self.__state[0:4] = self.__state[0:4] / not_norm_quaternion
