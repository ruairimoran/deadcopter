import dead
import numpy as np
import matplotlib.pyplot as plt


copter = dead.copter.DeadCopter(mass=1.3, arm_length=0.8, K_v=100)
copter.mass = 2

x_cache = []
for k in range(2000):
    x = copter.quaternion
    x_norm = np.linalg.norm(x)
    x_cache += [copter.euler_angles()]
    copter.fly_simulate([0, -np.sin(k/50)/100, 0], 0.01)

plt.plot(x_cache)
plt.show()
