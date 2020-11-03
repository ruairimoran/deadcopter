import dead
import numpy as np
import matplotlib.pyplot as plt


copter = dead.copter.DeadCopter(mass=1.3, arm_length=0.8, K_v=100)
copter.mass = 2

t_sampling = 0.01  # 10ms
t_simulation = 2   # simulation time (in s)
num_simulation_points = np.int(np.ceil(t_simulation / t_sampling))

black_box_euler = copter.euler_angles()

for k in range(num_simulation_points):
    black_box_euler = np.vstack((black_box_euler, copter.euler_angles()))
    copter.fly_simulate([0, np.sin(k/50)/100, 0], 0.01)


t_span = np.arange(num_simulation_points+1) * t_sampling
plt.plot(t_span, black_box_euler)
plt.show()
