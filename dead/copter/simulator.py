import matplotlib.pyplot as plt
import numpy as np


class Simulator:

    def __init__(self, **kwargs):
        self.__t_sampling = 0.01             # 10 ms
        self.__t_simulation = 2              # simulation time (in s)
        self.__omega_noise_multiplier = 0    # angular freq noise

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
        self.__num_simulation_points = np.int(np.ceil(self.__t_simulation / self.__t_sampling))  # for simulation loop
        self.t_span = np.arange(self.__num_simulation_points + 1) * self.__t_sampling  # for plot

    def system_design(self, copter):
        self.__Ad, self.__Bd, self.__Cd = copter.discretisation(self.__t_sampling)  # return discrete matrices, arg=dt
        n_control = self.__Ad.shape[0]
        n_observe = np.linalg.matrix_rank(self.__Cd)
        #nA = n_control
        #nB = Bd.shape[1]
        #nC = Cd.shape[1]

        # Controllability check
        copter.controllability(self.__Ad, self.__Bd, n_control)  # number of states

        # Observability check
        copter.observability(self.__Ad, self.__Cd, n_observe)  # n = number of observed states

        # LQR design
        self.__gain_K_lqr = copter.LQR(self.__Ad, self.__Bd)

        # Kf design
        self.__gain_L_Kf = copter.Kf(self.__Ad, self.__Cd)

        return self.__gain_K_lqr, self.__gain_L_Kf

    def simulate(self, copter):
        copter.state = [0.9994, 0.0044, 0.0251, 0.0249, 0.1, 0.1, 0.1, 0, 0, 0]  # initial state offset from equilibrium
        state_cache = copter.state
        euler_state_cache = copter.euler_angles(0)  # 0 for use of state

        state_hat = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        state_hat_cache = state_hat
        euler_state_hat_cache = copter.euler_angles(state_hat[0:4])

        for k in range(self.__num_simulation_points):
            control_action = (self.__gain_K_lqr @ copter.state[1:10])
            v_omega = np.random.multivariate_normal(np.zeros((3,)), np.identity(3))
            y = np.asarray(self.__Cd @ copter.state[1:10]).reshape(6, )
            y[3:6] += self.__omega_noise_multiplier * v_omega
            copter.fly_simulate(control_action, self.__t_sampling)
            euler_state_cache = np.vstack((euler_state_cache, copter.euler_angles(0)))  # 0 for use of state
            y_hat = np.asarray(self.__Cd @ state_hat[1:10]).reshape(6, )
            state_hat = np.asarray(self.__Ad @ state_hat[1:10] + (self.__Bd @ control_action.T).T
                                   + self.__gain_L_Kf @ (y_hat - y)).reshape(9, )
            state_hat_q0 = copter.solve_q0(state_hat[0:3])
            state_hat = [state_hat_q0] + state_hat.tolist()
            euler_state_hat_cache = np.vstack((euler_state_hat_cache, copter.euler_angles(state_hat[0:4])))
            state_cache = np.vstack((state_cache, copter.state))
            state_hat_cache = np.vstack((state_hat_cache, state_hat))

        return euler_state_cache, euler_state_hat_cache, state_cache, state_hat_cache

    def plot_all(self,
                 euler_angle_cache, euler_state_hat_cache,
                 state_cache, state_hat_cache,
                 ):
        # plot euler angles cache
        plt.subplot(2, 2, 1)
        plt.plot(self.t_span, np.rad2deg(euler_angle_cache))
        plt.plot(self.t_span, np.rad2deg(euler_state_hat_cache), '--')
        plt.title("euler angles  (degrees)")
        plt.legend(["roll", "pitch", "yaw", "roll_estimate", "pitch_estimate", "yaw_estimate"], loc="upper right")

        # plot quaternion cache
        plt.subplot(2, 2, 2)
        plt.plot(self.t_span, state_cache[:, 0:4])
        plt.plot(self.t_span, state_hat_cache[:, 0:4], '--')
        plt.title("quaternion")
        plt.legend(["q0", "q1", "q2", "q3", "q0_hat", "q1_hat", "q2_hat", "q3_hat"], loc="upper right")
        plt.ylim([-0.03, 0.03])

        # plot angular frequency cache
        plt.subplot(2, 2, 3)
        plt.plot(self.t_span, state_cache[:, 4:7])
        plt.plot(self.t_span, state_hat_cache[:, 4:7], '--')
        plt.title("angular frequency")
        plt.legend(["w0", "w1", "w2", "w0_hat", "w1_hat", "w2_hat"], loc="upper right")

        # plot rotation frequency cache
        plt.subplot(2, 2, 4)
        plt.plot(self.t_span, state_cache[:, 7:10])
        plt.plot(self.t_span, state_hat_cache[:, 7:10], '--')
        plt.title("rotation frequency")
        plt.legend(["n0", "n1", "n2", "n0_hat", "n1_hat", "n2_hat"], loc="upper right")

        # show sub plots
        plt.get_current_fig_manager().full_screen_toggle()  # toggle full-screen mode, exit = Ctrl+F
        plt.show()
