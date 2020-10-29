from pyquaternion import Quaternion


class DeadCopter:

    def __init__(self, **kwargs):
        # system parameters
        self.mass = 1.5
        print(kwargs)
        # include the system state

    def dynamics(self, control_actions, state):
        omega = state[4:7]
        q = Quaternion(state[0:4])
        w_quat = Quaternion([0] + omega)
        return [0.5*q*w_quat, ]
