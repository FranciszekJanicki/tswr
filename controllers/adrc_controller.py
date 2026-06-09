import numpy as np
from .adrc_joint_controller import ADRCJointController
from .controller import Controller
from models.manipulator_model import ManiuplatorModel


class ADRController(Controller):
    def __init__(self, Tp, params):
        self.model = ManiuplatorModel(Tp)
        self.joint_controllers = []
        for param in params:
            self.joint_controllers.append(ADRCJointController(*param, Tp))

    def calculate_control(self, x, q_d, q_d_dot, q_d_ddot):
        # M(q) * q_ddot + C(q, q_dot) * q_dot + G(q) = tau
        # q_ddot = M_inv(q) * tau - M_inv(q) * (C(q, q_dot) * q_dot + G(q))
        # q_ddot = f + b * u
        # b = M_inv(q)
        M = self.model.M(x)
        invM = np.linalg.inv(M)
        self.joint_controllers[0].set_b(invM[0, 0])
        self.joint_controllers[1].set_b(invM[1, 1])
        u = []
        for i, controller in enumerate(self.joint_controllers):
            u.append(controller.calculate_control(
                [x[i], x[i+2]], q_d[i], q_d_dot[i], q_d_ddot[i]))
        return np.array(u)
