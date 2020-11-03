import dead
import numpy as np


copter = dead.copter.DeadCopter(mass=1.3, arm_length=0.8)
#print(f"copter mass: {copter.__mass}")

# dynamics =
# input: [q0, q1, q2, q3, wx, wy, wz, nx, ny, nz], [ux, uy, uz]
# output: [q_dot[0:4], w_dot[4:7], n_dot[7:10]]
#dynam = copter.dynamics([1, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0])
#print(dynam)

for k in range(50):
    x = np.array(copter.state)
    x_norm = np.linalg.norm(x[0:4])
    print(x_norm)
    copter.fly_simulate([k/100, 0, 0], 0.01)


