import dead
import numpy as np
import matplotlib.pyplot as plt

copter = dead.copter.DeadCopter()

t_sampling = 0.01  # 10ms
t_simulation = 6   # simulation time (in s)
num_simulation_points = np.int(np.ceil(t_simulation / t_sampling))

black_box_euler = copter.euler_angles()
linear_black_box_euler = copter.euler_angles()

for k in range(num_simulation_points):
    black_box_euler = np.vstack((black_box_euler, copter.euler_angles()))
    copter.fly_simulate([0.001, 0.001, 0], 0.01)

copter.state = np.array([1] + [0] * 9)  # reset state

for k in range(num_simulation_points):
    linear_black_box_euler = np.vstack((linear_black_box_euler, copter.euler_angles()))
    copter.linear_fly_simulate([0.001, 0.001, 0], 0.01)

t_span = np.arange(num_simulation_points+1) * t_sampling

fig, (ax0_roll, ax1_pitch, ax2_yaw) = plt.subplots(3)
fig.suptitle('euler angles')
ax0_roll.plot(t_span, black_box_euler[:, 0], label='roll')
ax0_roll.plot(t_span, linear_black_box_euler[:, 0], label='linearised roll')
ax0_roll.legend(loc="upper left")
ax1_pitch.plot(t_span, black_box_euler[:, 1], label='pitch')
ax1_pitch.plot(t_span, linear_black_box_euler[:, 1], label='linearised pitch')
ax1_pitch.legend(loc="upper left")
ax2_yaw.plot(t_span, black_box_euler[:, 2], label='yaw')
ax2_yaw.plot(t_span, linear_black_box_euler[:, 2], label='linearised yaw')
ax2_yaw.legend(loc="upper left")

plt.show()