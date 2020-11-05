import unittest
import dead
import numpy as np


class CopterTestCase(unittest.TestCase):

    def test_attribute(self):
        copter = dead.copter.DeadCopter(mass=1.1)
        self.assertAlmostEqual(copter.mass, 1.1, 12, "fail1")

    # def test_single_axis_control(self):
    #     copter = dead.copter.DeadCopter()
    #     t_sampling = 0.01  # 10ms
    #     t_simulation = 10  # simulation time (in s)
    #     num_simulation_points = np.int(np.ceil(t_simulation / t_sampling))
    #     black_box_euler_test_roll = copter.euler_angles()
    #     black_box_euler_test_pitch = copter.euler_angles()
    #     black_box_euler_test_yaw = copter.euler_angles()
    #     for k in range(num_simulation_points):
    #         black_box_euler_test_roll = np.vstack((black_box_euler_test_roll, copter.euler_angles()))
    #         copter.fly_simulate([0.001, 0, 0], 0.01)
    #     self.assertEqual(black_box_euler_test_roll[:, 2:4].any(), 0, "fail2_roll")
    #     copter.state[0:10] = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # reset state
    #     for k in range(num_simulation_points):
    #         black_box_euler_test_pitch = np.vstack((black_box_euler_test_pitch, copter.euler_angles()))
    #         copter.fly_simulate([0, 0.001, 0], 0.01)
    #     self.assertEqual(black_box_euler_test_pitch[:, 1].any(), 0, "fail2_pitch")
    #     #self.assertAlmostEqual(black_box_euler_test_pitch[:, 3].any(), (0. or 3.14 or -3.14), 2, "fail2_pitch")

    # compare with linearised system state

if __name__ == '__main__':
    unittest.main()
