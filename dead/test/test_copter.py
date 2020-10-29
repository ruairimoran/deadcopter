import unittest
import dead


class CopterTestCase(unittest.TestCase):

    def test_copter(self):
        copter = dead.copter.DeadCopter()
        self.assertTrue(copter.mass > 0)
        self.fail("failed!")


if __name__ == '__main__':
    unittest.main()
