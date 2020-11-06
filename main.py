import dead
import numpy as np
import matplotlib.pyplot as plt


copter = dead.copter.DeadCopter(mass=1.3, arm_length=0.8, K_v=100)

t_sampling = 0.01  # 10ms
t_simulation = 10   # simulation time (in s)
num_simulation_points = np.int(np.ceil(t_simulation / t_sampling))

black_box_euler = copter.euler_angles()
linear_black_box_euler = copter.euler_angles()

for k in range(num_simulation_points):
    black_box_euler = np.vstack((black_box_euler, copter.euler_angles()))
    copter.fly_simulate([0.001, 0, 0], 0.01)  # np.sin(k/50)/100

copter.state = np.array([1] + [0] * 9)  # reset state

for k in range(num_simulation_points):
    linear_black_box_euler = np.vstack((linear_black_box_euler, copter.euler_angles()))
    copter.linear_fly_simulate([0.001, 0, 0], 0.01)  # np.sin(k/50)/100

t_span = np.arange(num_simulation_points+1) * t_sampling
plt.plot(t_span, black_box_euler)
plt.plot(t_span, linear_black_box_euler)
plt.legend(["roll", "pitch", "yaw", "Lroll", "Lpitch", "Lyaw"])
plt.show()
