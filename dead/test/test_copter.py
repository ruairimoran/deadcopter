import unittest
import dead


class CopterTestCase(unittest.TestCase):

    def test_attribute(self):
        copter = dead.copter.DeadCopter(mass=1.1)
        self.assertAlmostEqual(copter.mass, 1.1, 12, "fail")

    # test single axis rotations

    # compare with linearised system state

if __name__ == '__main__':
    unittest.main()
