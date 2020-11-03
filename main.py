import dead
import numpy as np


copter = dead.copter.DeadCopter(mass=1.2, arm_length=0.8)
print(f"copter mass: {copter.mass}")

# dynamics =
# input: [q0, q1, q2, q3, wx, wy, wz, nx, ny, nz], [ux, uy, uz]
# output: [q_dot[0:4], w_dot[4:7], n_dot[7:10]]
dynam = copter.dynamics([5, 6, 7, 8, 9, 10, 11, 12, 13, 14], [1, 1, 1])
print(dynam)
