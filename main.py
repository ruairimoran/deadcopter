import dead

copter = dead.copter.DeadCopter(asdf=1)
print(f"copter mass: {copter.mass}")
result = copter.dynamics([1, 2, 3], [1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
print(result)