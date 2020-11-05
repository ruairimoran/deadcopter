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
        t_simulation = 5  # simulation time (in s)
        num_simulation_points = np.int(np.ceil(t_simulation / t_sampling))

        black_box_euler_test_roll = copter.euler_angles()
        black_box_euler_test_pitch = copter.euler_angles()
        black_box_euler_test_yaw = copter.euler_angles()

        for k in range(num_simulation_points):
            black_box_euler_test_roll = np.vstack((black_box_euler_test_roll, copter.euler_angles()))
            copter.fly_simulate([0.001, 0, 0], 0.01)
        self.assertEqual(black_box_euler_test_roll[:, 1:3].any(), 0, "fail2_roll")

        copter.state = np.array([1] + [0] * 9)  # reset state

        for k in range(num_simulation_points):
            black_box_euler_test_pitch = np.vstack((black_box_euler_test_pitch, copter.euler_angles()))
            copter.fly_simulate([0, 0.001, 0], 0.01)
        rollyaw_possible_values = np.array([0, 3.14159265])
        self.assertAlmostEqual(black_box_euler_test_pitch[:, 0:3:2].all(), rollyaw_possible_values.any(), 4, "fail2_pitch")

        copter.state = np.array([1] + [0] * 9)  # reset state

        for k in range(num_simulation_points):
            black_box_euler_test_yaw = np.vstack((black_box_euler_test_yaw, copter.euler_angles()))
            copter.fly_simulate([0, 0, 0.001], 0.01)
        self.assertEqual(black_box_euler_test_yaw[:, 0:2].any(), 0, "fail2_yaw")

        copter.state = np.array([1] + [0] * 9)  # reset state

    # compare with linearised system state

if __name__ == '__main__':
    unittest.main()
