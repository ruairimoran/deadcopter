import dead
import numpy as np
import matplotlib.pyplot as plt
import control as C

copter = dead.copter.DeadCopter(mass=1.5, arm_length=0.225, K_v=1000)

# Simulation parameters
t_sampling = 0.01  # 10 ms
t_simulation = 2   # simulation time (in s)
num_simulation_points = np.int(np.ceil(t_simulation / t_sampling))  # for simulation loop
t_span = np.arange(num_simulation_points+1) * t_sampling  # for plot

# System design
Ad, Bd, Cd = copter.discretisation(t_sampling)  # return discrete matrices, arg=dt

# Controllability check
controllability_matrix = C.ctrb(Ad, Bd)
ctrb_rank = np.linalg.matrix_rank(controllability_matrix)
# print(ctrb_rank)

# Observability check
observability_matrix = C.obsv(Ad, Cd)
obsv_rank = np.linalg.matrix_rank(observability_matrix)
# print(obsv_rank)

# LQR design
nA = Ad.shape[0]
nB = Bd.shape[1]
Q_lqr = np.diagflat([1850, 1850, 1100, 12, 12, 12, 1, 1, 1])  # 1 * np.eye(nA)
R_lqr = np.diagflat([0.3, 0.3, 0.3])  # 1 * np.eye(nB)
solution_P_lqr, eigenvalues_cl_lqr, negative_gain_K_lqr = C.dare(Ad, Bd, Q_lqr, R_lqr)
gain_K_lqr = -negative_gain_K_lqr
# print(eigenvalues_cl_lqr)

# Kalman filter design
nC = Cd.shape[1]
Q_Kf = 1 * np.eye(nA)
R_Kf = 1 * np.eye(nC)
solution_P_Kf, eigenvalues_cl_Kf, negative_gain_L_Kf = C.dare(Ad, Cd, Q_Kf, R_Kf)
gain_L_Kf = -negative_gain_L_Kf
# print(eigenvalues_cl_Kf)

# Simulation
copter.state = [0.9994, 0.0044, 0.0251, 0.0249, 0, 0, 0, 0, 0, 0]  # initial state offset from equilibrium
state_cache = copter.state
euler_angle_cache = copter.euler_angles(0)  # 0 for use of state

state_hat = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
state_hat_cache = state_hat
euler_state_hat_cache = copter.euler_angles(state_hat[0:4])

for k in range(num_simulation_points):
    control_action = gain_K_lqr @ copter.state[1:10]

    y = np.asarray(Cd @ copter.state[1:10]).reshape(9,)
    #y_q0 = copter.solve_q0(y[0:3])
    #y = [y_q0] + y.tolist()

    copter.fly_simulate(control_action, t_sampling)
    euler_angle_cache = np.vstack((euler_angle_cache, copter.euler_angles(0)))  # 0 for use of state

    y_hat = np.asarray(Cd @ state_hat[1:10]).reshape(9,)
    #y_hat_q0 = copter.solve_q0(y_hat[0:3])
    #y_hat = [y_hat_q0] + y_hat.tolist()

    state_hat = np.asarray(Ad @ state_hat[1:10] + (Bd @ control_action.T).T + gain_L_Kf @ (y_hat - y)).reshape(9,)
    state_hat_q0 = copter.solve_q0(state_hat[0:3])
    state_hat = [state_hat_q0] + state_hat.tolist()
    euler_state_hat_cache = np.vstack((euler_state_hat_cache, copter.euler_angles(state_hat[0:4])))

    state_cache = np.vstack((state_cache, copter.state))
    state_hat_cache = np.vstack((state_hat_cache, state_hat))

# plot euler angles cache
e_plot = plt.plot(t_span, np.hstack((np.rad2deg(euler_angle_cache), np.rad2deg(euler_state_hat_cache))))
plt.title("euler angles")
plt.legend(["roll", "pitch", "yaw", "roll_estimate", "pitch_estimate", "yaw_estimate"])
plt.show()

# plot quaternion cache
q_plot = plt.plot(t_span, np.hstack((state_cache[:, 0:4], state_hat_cache[:, 0:4])))
plt.title("quaternion")
plt.legend(["q0", "q1", "q2", "q3", "q0_hat", "q1_hat", "q2_hat", "q3_hat"])
plt.ylim([-0.03, 0.03])
plt.show()

# plot angular frequency cache
w_plot = plt.plot(t_span, np.hstack((state_cache[:, 4:7], state_hat_cache[:, 4:7])))
plt.title("angular frequency")
plt.legend(["w0", "w1", "w2", "w0_hat", "w1_hat", "w2_hat"])
plt.show()

# plot rotation frequency cache
n_plot = plt.plot(t_span, np.hstack((state_cache[:, 7:10], state_hat_cache[:, 7:10])))
plt.title("rotation frequency")
plt.legend(["n0", "n1", "n2", "n0_hat", "n1_hat", "n2_hat"])
plt.show()
