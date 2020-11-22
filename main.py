import dead
import numpy as np
import matplotlib.pyplot as plt
import control as C

copter = dead.copter.DeadCopter(mass=1.5, arm_length=0.225, K_v=1000)

# Simulation parameters
t_sampling = 0.01  # 10 ms
t_simulation = 5   # simulation time (in s)
num_simulation_points = np.int(np.ceil(t_simulation / t_sampling))  # for simulation loop
t_span = np.arange(num_simulation_points+1) * t_sampling  # for plot

# System design
Ad, Bd, Cd = copter.discretisation(t_sampling)  # return discrete matrices, arg=dt
n_control = Ad.shape[0]
n_observe = np.linalg.matrix_rank(Cd)
nA = n_control
nB = Bd.shape[1]
nC = Cd.shape[1]

# Controllability check
copter.controllability(Ad, Bd, n_control)  # number of states

# Observability check
copter.observability(Ad, Cd, n_observe)  # n = number of observed states

# LQR design
Q_lqr = np.diagflat([1850, 1850, 1100, 120, 12, 12, 1, 1, 1])  # 1 * np.eye(nA)
R_lqr = np.diagflat([0.3, 0.3, 0.3])  # 1 * np.eye(nB)
solution_P_lqr, eigenvalues_cl_lqr, negative_gain_K_lqr = C.dare(Ad, Bd, Q_lqr, R_lqr)
gain_K_lqr = -negative_gain_K_lqr
# print(eigenvalues_cl_lqr)

# Kalman filter design
Q_Kf = 1000 * np.eye(nA)
R_Kf = 1 * np.eye(nC)
solution_P_Kf, eigenvalues_cl_Kf, negative_gain_L_Kf = C.dare(Ad.T, Cd.T, Q_Kf, R_Kf)
gain_L_Kf = -negative_gain_L_Kf
# print(eigenvalues_cl_Kf)

# Simulation
copter.state = [0.9994, 0.0044, 0.0251, 0.0249, 0.1, 0.1, 0.1, 0, 0, 0]  # initial state offset from equilibrium
state_cache = copter.state
euler_angle_cache = copter.euler_angles(0)  # 0 for use of state

state_hat = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
state_hat_cache = state_hat
euler_state_hat_cache = copter.euler_angles(state_hat[0:4])

for k in range(num_simulation_points):
    kick = (k == num_simulation_points/2) * np.array([1, 1, 1e-4])  # yaw extremely susceptible to change
    control_action = (gain_K_lqr @ copter.state[1:10]) + kick

    y = np.asarray(Cd @ copter.state[1:10]).reshape(9,)

    copter.fly_simulate(control_action, t_sampling)
    euler_angle_cache = np.vstack((euler_angle_cache, copter.euler_angles(0)))  # 0 for use of state

    y_hat = np.asarray(Cd @ state_hat[1:10]).reshape(9,)

    state_hat = np.asarray(Ad @ state_hat[1:10] + (Bd @ control_action.T).T + gain_L_Kf @ (y_hat - y)).reshape(9,)
    state_hat_q0 = copter.solve_q0(state_hat[0:3])
    state_hat = [state_hat_q0] + state_hat.tolist()
    euler_state_hat_cache = np.vstack((euler_state_hat_cache, copter.euler_angles(state_hat[0:4])))

    state_cache = np.vstack((state_cache, copter.state))
    state_hat_cache = np.vstack((state_hat_cache, state_hat))

# plot euler angles cache
# plt.subplot(2, 2, 1)
plt.plot(t_span, np.rad2deg(euler_angle_cache))
plt.plot(t_span, np.rad2deg(euler_state_hat_cache), '--')
plt.title("euler angles  (degrees)")
plt.legend(["roll", "pitch", "yaw", "roll_estimate", "pitch_estimate", "yaw_estimate"], loc="upper right")

# # plot quaternion cache
# plt.subplot(2, 2, 2)
# plt.plot(t_span, state_cache[:, 0:4])
# plt.plot(t_span, state_hat_cache[:, 0:4], '--')
# plt.title("quaternion")
# plt.legend(["q0", "q1", "q2", "q3", "q0_hat", "q1_hat", "q2_hat", "q3_hat"], loc="upper right")
# plt.ylim([-0.03, 0.03])
#
# # plot angular frequency cache
# plt.subplot(2, 2, 3)
# plt.plot(t_span, state_cache[:, 4:7])
# plt.plot(t_span, state_hat_cache[:, 4:7], '--')
# plt.title("angular frequency")
# plt.legend(["w0", "w1", "w2", "w0_hat", "w1_hat", "w2_hat"], loc="upper right")
#
# # plot rotation frequency cache
# plt.subplot(2, 2, 4)
# plt.plot(t_span, state_cache[:, 7:10])
# plt.plot(t_span, state_hat_cache[:, 7:10], '--')
# plt.title("rotation frequency")
# plt.legend(["n0", "n1", "n2", "n0_hat", "n1_hat", "n2_hat"], loc="upper right")

# show sub plots
plt.get_current_fig_manager().full_screen_toggle()  # toggle fullscreen mode, exit = Ctrl+F
plt.show()
