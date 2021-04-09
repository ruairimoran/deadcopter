import matplotlib.pyplot as plt
import numpy as np
from pyquaternion import *


class Simulator:

    def __init__(self, **kwargs):
        self.__t_sampling = 0.01             # 10 ms
        self.__t_simulation = 2              # simulation time (in s)
        self.__measurement_noise_multiplier = 1e-9    # imu noise

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
        self.__n_control = self.__Ad.shape[0]
        self.__n_observe = self.__Cd.shape[0]

        # Controllability check
        copter.controllability(self.__Ad, self.__Bd, self.__n_control)  # number of states

        # Observability check
        copter.observability(self.__Ad, self.__Cd, self.__n_observe)  # n = number of observed states

        # LQR design
        self.__gain_K_lqr = copter.LQR(self.__Ad, self.__Bd)

        # Kf design
        self.__gain_L_Kf = copter.Kf(self.__Ad, self.__Cd)

        # v2 reference tracking
        self.__W = np.vstack((np.hstack((self.__Ad-np.eye(9), self.__Bd)),
                              np.hstack((self.__Cd, np.zeros(shape=(6, 3))))))
        self.__G, residuals, rank, singulars = np.linalg.lstsq(self.__W,
                                                               np.vstack((np.zeros(shape=(self.__n_control, self.__n_observe)),
                                                                          np.eye(self.__n_observe))), rcond=0)

        return self.__Ad, self.__Bd, self.__Cd, self.__gain_K_lqr, self.__gain_L_Kf, self.__G

    def simulate(self, copter):
        copter.state = [0.9994, 0.0044, 0.0251, 0.0249, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0]  # initial state offset from equilibrium
        state_cache = copter.state
        euler_state_cache = copter.euler_angles(0)  # 0 for use of state

        state_hat = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        state_hat_cache = state_hat
        euler_state_hat_cache = copter.euler_angles(state_hat[0:4])

        control_action = np.zeros((3,))
        control_action_cache = control_action
        r = np.zeros(6,)

        for k in range(self.__num_simulation_points):
            if k > self.__num_simulation_points/2:
                r = [0.0044, 0.0251, 0.0249, 0, 0, 0]

            v_omega = np.random.multivariate_normal(np.zeros((6,)), np.identity(6))
            y = np.asarray(self.__Cd @ copter.state[1:10]).reshape(6, )
            y += (self.__measurement_noise_multiplier * v_omega)

            xu_equilibriums = self.__G @ r
            state_quaternion = Quaternion(copter.quaternion)
            xu_equilibriums_q0 = copter.solve_q0(xu_equilibriums[0:3])
            xu_equilibriums_quaternion = Quaternion(xu_equilibriums_q0, xu_equilibriums[0], xu_equilibriums[1], xu_equilibriums[2])
            difference_quaternion = state_quaternion * xu_equilibriums_quaternion.inverse
            difference_state = np.array(copter.state[4:10]) - np.array(xu_equilibriums[3:9])
            full_difference = difference_quaternion.elements[1:4].tolist() + difference_state.tolist()

            control_action = xu_equilibriums[self.__n_control:self.__n_control + self.__n_observe] + self.__gain_K_lqr @ full_difference

            copter.fly_simulate(control_action, self.__t_sampling)

            y_hat = np.asarray(self.__Cd @ state_hat[1:10]).reshape(6,)

            state_hat = np.asarray(self.__Ad @ state_hat[1:10] + (self.__Bd @ control_action.T).T + self.__gain_L_Kf @ (y_hat - y)).reshape(9, )
            state_hat_q0 = copter.solve_q0(state_hat[0:3])
            state_hat = [state_hat_q0] + state_hat.tolist()

            # cache
            euler_state_cache = np.vstack((euler_state_cache, copter.euler_angles(0)))  # 0 state
            euler_state_hat_cache = np.vstack((euler_state_hat_cache, copter.euler_angles(state_hat[0:4])))
            control_action_cache = np.vstack((control_action_cache, np.asarray(control_action)))
            state_cache = np.vstack((state_cache, copter.state))
            state_hat_cache = np.vstack((state_hat_cache, state_hat))

        return euler_state_cache, euler_state_hat_cache, control_action_cache, state_cache, state_hat_cache

    def plot_all(self,
                 plot_euler_angle_cache, plot_euler_state_hat_cache,
                 plot_control_action_cache,
                 plot_state_cache, plot_state_hat_cache):
        plt.rcParams.update({'font.size': 18})
        plt.subplots_adjust(left=0.08, bottom=0.08, right=0.98, top=0.94, wspace=0.27, hspace=0.43)

        # plot euler angles cache
        plt.subplot(2, 2, 1)
        plt.plot(self.t_span, np.rad2deg(plot_euler_angle_cache))
        plt.plot(self.t_span, np.rad2deg(plot_euler_state_hat_cache), '--')
        plt.title("Euler angles")
        plt.legend(["roll(x)", "pitch(y)", "yaw(z)", "roll_est.", "pitch_est.", "yaw_est."], loc="upper right")
        plt.ylabel("degrees")
        plt.xlabel("time /s")
        plt.grid()

        # # plot quaternion cache
        # plt.subplot(2, 2, 2)
        # plt.plot(self.t_span, plot_state_cache[:, 0:4])
        # plt.plot(self.t_span, plot_state_hat_cache[:, 0:4], '--')
        # plt.title("quaternion")
        # plt.legend(["q0", "q1", "q2", "q3", "q0_est.", "q1_est.", "q2_est.", "q3_est."], loc="upper right")
        # plt.ylim([-0.03, 0.03])
        # plt.ylabel("arbitrary")
        # plt.xlabel("time /s")
        # plt.grid()

        # plot control action cache
        plt.subplot(2, 2, 2)
        plt.plot(self.t_span, plot_control_action_cache)
        plt.title("control action")
        plt.legend(["ux", "uy", "uz"], loc="upper right")
        plt.ylabel("arbitrary")
        plt.xlabel("time /s")
        plt.grid()

        # plot angular frequency cache
        plt.subplot(2, 2, 3)
        plt.plot(self.t_span, plot_state_cache[:, 4:7])
        plt.plot(self.t_span, plot_state_hat_cache[:, 4:7], '--')
        plt.title("angular rate")
        plt.legend(["wx", "wy", "wz", "wx_est.", "wy_est.", "wz_est."], loc="upper right")
        plt.ylabel("rad/s")
        plt.xlabel("time /s")
        plt.grid()

        # plot rotation frequency cache
        plt.subplot(2, 2, 4)
        plt.plot(self.t_span, plot_state_cache[:, 7:10])
        plt.plot(self.t_span, plot_state_hat_cache[:, 7:10], '--')
        plt.title("deviation from hovering spin")
        plt.legend(["nx", "ny", "nz", "nx_est.", "ny_est.", "nz_est."], loc="upper right")
        plt.ylabel("revolutions/s")
        plt.xlabel("time /s")
        plt.grid()

        # show sub plots
        plt.get_current_fig_manager().full_screen_toggle()  # toggle full-screen mode, exit = Ctrl+F
        plt.show()
