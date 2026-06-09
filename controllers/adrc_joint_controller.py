import numpy as np
from observers.eso import ESO
from .controller import Controller


class ADRCJointController(Controller):
    def __init__(self, b, kp, kd, p, q0, Tp):
        self.b = b
        self.kp = kp
        self.kd = kd

        A = np.array([
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0]
        ])
        B = np.array([
            [0.0],
            [self.b],
            [0.0]
        ])
        L = np.array([
            [3.0 * p],
            [3.0 * (p**2)],
            [p**3]
        ])
        W = np.array([[1.0, 0.0, 0.0]])

        self.eso = ESO(A, B, W, L, q0, Tp)
        self.u = 0.0

    def set_b(self, b):
        self.b = b
        new_B = np.array([
            [0.0],
            [self.b],
            [0.0]
        ])
        self.eso.set_B(new_B)

    def calculate_control(self, x, q_d, q_d_dot, q_d_ddot):
        q = np.squeeze(x[0])
        q_d = np.squeeze(q_d)
        q_d_dot = np.squeeze(q_d_dot)
        q_d_ddot = np.squeeze(q_d_ddot)

        u_array = np.array([self.u])
        self.eso.update(q, u_array)
        eso_state = np.squeeze(self.eso.get_state())
        q_hat = eso_state[0]
        q_dot_hat = eso_state[1]
        f_hat = eso_state[2]
        e = q_d - q_hat
        e_dot = q_d_dot - q_dot_hat
        v = q_d_ddot + self.kd * e_dot + self.kp * e
        self.u = (v - f_hat) / self.b
        return float(np.squeeze(self.u))
