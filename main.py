import dead
import numpy as np
import matplotlib.pyplot as plt
import control as C

copter = dead.copter.DeadCopter(mass=1.3, arm_length=0.8, K_v=100)

# Simulation parameters
t_sampling = 0.01  # 10 ms
t_simulation = 6   # simulation time (in s)
num_simulation_points = np.int(np.ceil(t_simulation / t_sampling))
t_span = np.arange(num_simulation_points+1) * t_sampling  # for plotting time

# System design
a, b, c = copter.linearisation()
Ad, Bd, Cd = copter.discretisation(a, b, c, 0, 0.1)

# Controllability check
controllability_matrix = C.ctrb(Ad, Bd)
ctrb_rank = np.linalg.matrix_rank(controllability_matrix)  # = 4 so not controllable ?
# print(ctrb_rank)

# LQR design
Q_lqr = 1 * np.eye(9)
R_lqr = 1 * np.eye(3)
solution_P, eigenvalues_cl, negative_gain_K = C.dare(Ad, Bd, Q_lqr, R_lqr)
gain_K = -negative_gain_K
# print(eigenvalues_cl)

# Simulation
copter.state = [0, 1, 0, 0, 0, 0, 0, 0, 0, 0]  # initial state offset from equilibrium
euler_angle_cache = copter.euler_angles()
for k in range(50):
    control_action = gain_K@copter.state[1:10]
    copter.linear_fly_simulate(control_action, t_sampling)
    euler_angle_cache = np.vstack((euler_angle_cache, copter.euler_angles()))
plt.plot(euler_angle_cache)
plt.legend(["roll", "pitch", "yaw"])
plt.show()
