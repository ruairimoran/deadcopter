from pyquaternion import Quaternion
import numpy as np


class DeadCopter:

    def __init__(self, **kwargs):
        # SYSTEM PARAMETERS
        # general
        self.mass = 1.5  # kg                           # total mass of aircraft
        self.arm_length = 0.225  # m                    # quad arm length
        self.num_motors = 4  # motors                   # no. of motors
        self.moi_xx = 0.032  # kg.m^2                   # moment of inertia xx
        self.moi_yy = 0.032  # kg.m^2                   # moment of inertia yy
        self.moi_zz = 0.058  # kg.m^2                   # moment of inertia zz
        self.moi = np.diagflat(
            [self.moi_xx, self.moi_yy, self.moi_zz])    # inertia matrix
        self.gravity_acc = 9.81  # m/s^2                # acceleration of gravity
        self.air_density = 1.225  # kg/m^3              # air density at sea level, 15 deg C

        # motors
        self.K_v = 1000  # rpm/V                        # motor speed constant
        self.motor_time_constant = 35  # ms             # motor time constant
        self.rotor_mass = 10  # g                       # rotor mass
        self.rotor_radius = 15  # mm                    # rotor radius
        self.motor_mass = 50  # g                       # total mass of motor
        self.voltage_max = 14  # V                      # max voltage to motor
        self.voltage_min = 10  # V                      # min voltage to motor
        self.motor_moi = 1  # kg.m^2                    # motor moment of inertia

        # props
        self.thrust_coeff = 0.1                         # thrust coefficient
        self.power_coeff = 0.04                         # power coefficient
        self.prop_mass = 10  # g                        # prop mass
        self.prop_diameter = 9  # in                    # prop diameter
        self.prop_moi = 1  # kg.m^2                     # prop moment of inertia

        # parse `kwargs` and set as attribute, provided the keyword corresponds
        # to one of the variables defined above
        for key in self.__dict__.keys():
            if key in kwargs:
                setattr(self, key, kwargs[key])

        # modelling --- the following variables are hidden
        self.__n_h = ((self.mass * self.gravity_acc) /
                      np.sqrt(self.num_motors * self.thrust_coeff
                              * self.air_density * (self.prop_diameter ** 4)))
        self.__k1 = (self.K_v * (self.voltage_max - self.voltage_min)) / 60     # /60 for rps
        self.__k2 = 1 / self.motor_time_constant
        self.__k3_x = (2 * self.__n_h) * (self.thrust_coeff * self.air_density * (self.prop_diameter ** 4)) \
            * (self.num_motors * self.arm_length) / ((2 ** 0.5) * self.moi_xx)
        self.__k3_y = (2 * self.__n_h) * (self.thrust_coeff * self.air_density * (self.prop_diameter ** 4)) \
            * (self.num_motors * self.arm_length) / ((2 ** 0.5) * self.moi_yy)
        self.__k3_z = (2 * self.__n_h) * (self.power_coeff * self.air_density * (self.prop_diameter ** 5)) \
            * self.num_motors / (2 * np.pi * self.moi_zz)
        self.__k4_xy = 0
        self.__k4_z = 2 * np.pi * self.num_motors * (self.prop_moi + self.motor_moi) / self.moi_zz
        self.__gamma_n = np.diagflat([self.__k3_x, self.__k3_y, self.__k3_z - (self.__k4_z * self.__k2)])
        self.__gamma_u = np.diagflat([0, 0, self.__k4_z * self.__k2 * self.__k1])
        # include the system state ??

    def linearisation(self):
        pass

    def dynamics(self, state, u):
        # define
        q = Quaternion(state[0:4])                              # q = quaternion / attitude
        w = state[4:7]                                          # w = omega / angular frequencies
        n = state[7:10]                                         # n = rotor frequencies
        control_action = np.array(u)                            # c = control input

        w_quat = Quaternion([0] + w)
        q_dot = list(0.5 * q * w_quat)

        w1 = np.diag(self.moi * w)                                  # flatten 3x3 array to 1x3
        w_cross = np.cross(w, w1)
        w_dot = list(np.diag(self.__gamma_n * n + self.__gamma_u * u - np.linalg.inv(self.moi) * w_cross))

        n_dot = list(self.__k2 * (self.__k1 * control_action - n))

        dynamics = list(q_dot + w_dot + n_dot)

        return dynamics
