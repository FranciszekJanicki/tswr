import numpy as np
from models.manipulator_model import ManiuplatorModel
from .controller import Controller


class FeedbackLinearizationController(Controller):
    def __init__(self, Tp):
        self.model = ManiuplatorModel(Tp)

    def calculate_control(self, x, q_r, q_r_dot, q_r_ddot):
        """
        Please implement the feedback linearization using self.model (which you have to implement also),
        robot state x and desired control v.
        """
        # Bez feedbacka
        # M_mat = self.model.M(x)
        # C_mat = self.model.C(x)
        # q1, q2, q1_dot, q2_dot = x

        # q_dot = np.array([[q1_dot], [q2_dot]])
        # q_r_ddot = q_r_ddot.reshape(2, 1)
        # tau = M_mat @ q_r_ddot + C_mat @ q_dot

        Kp = np.array([[100.0, 0.0],
                       [0.0, 100.0]])
        Kd = np.array([[20.0, 0.0],
                       [0.0, 20.0]])
        M_mat = self.model.M(x)
        C_mat = self.model.C(x)
        q1, q2, q1_dot, q2_dot = x

        q_dot = np.array([[q1_dot], [q2_dot]])
        q_r_ddot = q_r_ddot.reshape(2, 1)
        q_r_dot = q_r_dot.reshape(2, 1)
        q = np.array([[q1], [q2]])
        q_r = q_r.reshape(2, 1)
        v = q_r_ddot + Kd @ (q_r_dot - q_dot) + Kp @ (q_r - q)

        tau = M_mat @ v + C_mat @ q_dot

        return np.squeeze(tau)
