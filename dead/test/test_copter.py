import unittest
import dead
import numpy as np


class CopterTestCase(unittest.TestCase):

    def test_attribute(self):
        copter = dead.copter.DeadCopter(mass=1.1)
        self.assertAlmostEqual(copter.mass, 1.1, 12, "fail1")

    def test_single_axis_control(self):
        copter = dead.copter.DeadCopter()
        t_sampling = 0.01  # 10ms
        t_simulation = 0.05  # simulation time (in s)
        num_simulation_points = np.int(np.ceil(t_simulation / t_sampling))

        for k in range(num_simulation_points):
            copter.fly_simulate([0.001, 0, 0], 0.01)
            euler_angles = copter.euler_angles()
            self.assertAlmostEqual(euler_angles[1], 0, delta=1e-12)
            self.assertAlmostEqual(euler_angles[2], 0, delta=1e-12)

        copter.state = np.array([1] + [0] * 9)  # reset state

        for k in range(num_simulation_points):
            copter.fly_simulate([0, 0.001, 0], 0.01)
            euler_angles = copter.euler_angles()
            self.assertAlmostEqual(euler_angles[0], 0, delta=1e-12)
            self.assertAlmostEqual(euler_angles[2], 0, delta=1e-12)

        copter.state = np.array([1] + [0] * 9)  # reset state

        for k in range(num_simulation_points):
            copter.fly_simulate([0, 0, 0.001], 0.01)
            euler_angles = copter.euler_angles()
            self.assertAlmostEqual(euler_angles[0], 0, delta=1e-12)
            self.assertAlmostEqual(euler_angles[1], 0, delta=1e-12)

        copter.state = np.array([1] + [0] * 9)  # reset state



if __name__ == '__main__':
    unittest.main()
