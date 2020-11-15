import dead
import numpy as np
import matplotlib.pyplot as plt
import control as C

copter = dead.copter.DeadCopter(mass=1.3, arm_length=0.8, K_v=100)

# Simulation parameters
t_sampling = 0.01  # 10 ms
t_simulation = 5   # simulation time (in s)
num_simulation_points = np.int(np.ceil(t_simulation / t_sampling))  # for simulation loop
t_span = np.arange(num_simulation_points+1) * t_sampling  # for plot

# System design
Ad, Bd, Cd = copter.discretisation(0.1)  # return discrete matrices, dt=0.1
print()

# Controllability check
controllability_matrix = C.ctrb(Ad, Bd)
ctrb_rank = np.linalg.matrix_rank(controllability_matrix)
# print(ctrb_rank)

# LQR design
nx = Ad.shape[0]
nu = Bd.shape[1]
Q_lqr = np.diagflat([1850, 1850, 1100, 12, 12, 12, 1, 1, 1])  # 1 * np.eye(nx)
R_lqr = np.diagflat([0.3, 0.3, 0.3])  # 1 * np.eye(nu)
solution_P, eigenvalues_cl, negative_gain_K = C.dare(Ad, Bd, Q_lqr, R_lqr)
gain_K = -negative_gain_K
# print(eigenvalues_cl)

# Simulation controller
copter.state = [0.9994, 0.0044, 0.0251, 0.0249, 0, 0, 0, 0, 0, 0]  # initial state offset from equilibrium
euler_angle_cache = copter.euler_angles()
for k in range(num_simulation_points):
    control_action = gain_K@copter.state[1:10]
    copter.fly_simulate(control_action, t_sampling)
    euler_angle_cache = np.vstack((euler_angle_cache, copter.euler_angles()))
plt.plot(t_span, np.rad2deg(euler_angle_cache))
plt.legend(["roll", "pitch", "yaw"])
plt.show()
