import numpy as np
from .controller import Controller


class PDDecentralizedController(Controller):
    def __init__(self, kp, kd):
        self.kp = np.array(kp)
        self.kd = np.array(kd)

    def calculate_control(self, x, q_d, q_d_dot, q_d_ddot):
        q = np.array(x[:2])
        q_dot = np.array(x[2:])

        q_d = np.array(q_d)
        q_d_dot = np.array(q_d_dot)

        e = q_d - q
        e_dot = q_d_dot - q_dot

        u = self.kp * e + self.kd * e_dot

        return u.flatten()
