import numpy as np

from observers.eso import ESO
from .adrc_joint_controller import ADRCJointController
from .controller import Controller
from models.manipulator_model import ManiuplatorModel


class ADRFLController(Controller):
    def __init__(self, Tp, q0, Kp, Kd, p):
        self.model = ManiuplatorModel(Tp)
        self.Kp = Kp
        self.Kd = Kd
        p1, p2 = p[0], p[1]
        self.L = np.array([
            [3*p1, 0],
            [0, 3*p2],
            [3*p1**2, 0],
            [0, 3*p2**2],
            [p1**3, 0],
            [0, p2**3]
        ])

        W = np.array([
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
        ])

        A = np.array([
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ])
        B = np.zeros((6, 2))
        self.previous_u = np.zeros(2)

        self.eso = ESO(A, B, W, self.L, q0, Tp)
        self.update_params(q0[:2], q0[2:])

    def update_params(self, q, q_dot):
        x = np.concatenate([q, q_dot])
        self.M = self.model.M(x)
        self.C = self.model.C(x)
        M_inv = np.linalg.inv(self.M)

        new_B = np.zeros((6, 2))
        new_B[2:4, :] = M_inv
        self.eso.set_B(new_B)

        new_A = np.zeros((6, 6))
        new_A[0, 2] = 1.0
        new_A[1, 3] = 1.0
        new_A[2, 4] = 1.0
        new_A[3, 5] = 1.0
        new_A[2:4, 2:4] = -M_inv @ self.C
        self.eso.set_A(new_A)

    def calculate_control(self, x, q_d, q_d_dot, q_d_ddot):
        # AIM:
        # q_ddot = v

        # q_ddot = M_hat_inv @ u - M_hat_inv @ C_hat @ q_dot_hat + f_hat
        # v = M_hat_inv @ u - M_hat_inv @ C_hat @ q_dot_hat + f_hat
        # M_hat @ v = u - C_hat @ q_dot_hat + M_hat @ f_hat
        # u = M_hat @ v - M_hat @ f_hat + C_hat @ q_dot_hat
        # u = M_hat @ (v - f_hat) + C_hat @ q_dot_hat

        q = x[:2]
        q_dot = x[2:]

        self.update_params(q, q_dot)
        self.eso.update(q, self.previous_u)
        z_hat = self.eso.get_state()
        q_hat = z_hat[0:2]
        q_dot_hat = z_hat[2:4]
        f_hat = z_hat[4:6]

        v = q_d_ddot + self.Kd @ (q_d_dot - q_dot_hat) + \
            self.Kp @ (q_d - q_hat)
        u = self.M @ (v - f_hat) + self.C @ q_dot_hat

        self.previous_u = u

        return u
