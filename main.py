import dead
import numpy as np


copter = dead.copter.DeadCopter()
print(f"copter mass: {copter.mass}")

result = copter.dynamics([5, 6, 7, 8, 9, 10, 11, 12, 13, 14], [1, 1, 1])
attitude = result['q_'].elements

print(attitude)
print(result['q_'])
print(result['w_'])
print(result['n_'])