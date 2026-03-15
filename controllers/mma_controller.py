import numpy as np
from models.manipulator_model import ManiuplatorModel
from .controller import Controller


class MMAController(Controller):
    def __init__(self, Tp):
        # TODO: Fill the list self.models with 3 models of 2DOF manipulators with different m3 and r3
        # I:   m3=0.1,  r3=0.05
        # II:  m3=0.01, r3=0.01
        # III: m3=1.0,  r3=0.3
        self.models = [ManiuplatorModel(Tp, 0), ManiuplatorModel(
            Tp, 1), ManiuplatorModel(Tp, 2)]
        self.i = 0
        self.Tp = Tp
        self.prev_u = np.zeros((2, 1))
        self.prev_q_dot = np.zeros((2, 1))

    def choose_model(self, x):
        q1, q2, q1_dot, q2_dot = x
        q = np.array([[q1], [q2]])
        q_dot = np.array([[q1_dot], [q2_dot]])
        q_ddot = (q_dot - self.prev_q_dot) / self.Tp
        self.prev_q_dot = q_dot.copy()
        lowest_error = float('inf')
        for i in range(len(self.models)):
            model = self.models[i]
            tau_est = model.M(x) @ q_ddot + model.C(x) @ q_dot
            error = np.linalg.norm(self.prev_u - tau_est)
            if error < lowest_error:
                lowest_error = error
                self.i = i
        if i != self.i:
            print(f"BŁĄD MODELU! Przełączam z {self.i} na {i}")

    def calculate_control(self, x, q_r, q_r_dot, q_r_ddot):
        self.choose_model(x)
        q = x[:2]
        q_dot = x[2:]
        v = q_r_ddot  # TODO: add feedback

        Kp = np.diag([100.0, 100.0])
        Kd = np.diag([20.0, 20.0])
        e = q_r - q
        e_dot = q_r_dot - q_dot
        v = q_r_ddot + Kd @ e_dot + Kp @ e

        M = self.models[self.i].M(x)
        C = self.models[self.i].C(x)
        u = M @ v[:, np.newaxis] + C @ q_dot[:, np.newaxis]
        self.prev_u = u
        return np.squeeze(u)
