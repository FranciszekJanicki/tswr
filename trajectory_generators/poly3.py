import numpy as np
from trajectory_generators.trajectory_generator import TrajectoryGenerator


class Poly3(TrajectoryGenerator):
    def __init__(self, start_q, desired_q, T):
        self.T = T
        self.q_0 = start_q
        self.q_k = desired_q
        """
        Please implement the formulas for a_0 till a_3 using self.q_0 and self.q_k
        Assume that the velocities at start and end are zero.
        """
        #  Normalnie tak by trzeba robic:
        # self.a_0 = start_q
        # self.a_1 = 3 * start_q
        # alpha = 3 * self.a_0 - self.a_1 + \
        #     (-6*self.a_0+4*self.a_1)*T + (3*self.a_0-3*self.a_1)*T**2
        # beta = desired_q - self.a_0 - (-3*self.a_0 + self.a_1) * T - (
        #     3 * self.a_0 - 2 * self.a_1) * T**2 - (-self.a_0+self.a_1)*T**3
        # self.a_2 = (3*beta) / T**2 - alpha/T
        # self.a_3 = (-2*beta+3*beta*T+alpha*T+alpha*T**2) / T**3
        #
        # Normalizujac czas do tak ze T_koncowa = 1 mozna ucyc wielomianow BERNSTEINA:
        self.a_0 = start_q
        self.a_1 = 3 * start_q
        self.a_2 = 3 * desired_q
        self.a_3 = desired_q

    def generate(self, t):
        """
        Implement trajectory generator for your manipulator.
        Positional trajectory should be a 3rd degree polynomial going from an initial state q_0 to desired state q_k.
        Remember to derive the first and second derivative of it also.
        Use following formula for the polynomial from the instruction.
        """
        t /= self.T
        q = self.a_3 * t**3 + self.a_2 * t**2 * \
            (1 - t) + self.a_1 * t * (1 - t)**2 + self.a_0 * (1 - t)**3
        q_dot = 3 * self.a_3 * t**2 + \
            self.a_2 * (2 * t - 3 * t**2) + \
            self.a_1 * (1 - 4 * t + 3 * t**2) + \
            self.a_0 * (-3 + 6 * t - 3 * t**2)

        q_ddot = 6 * self.a_3 * t + \
            self.a_2 * (2 - 6 * t) + \
            self.a_1 * (-4 + 6 * t) + \
            self.a_0 * (6 - 6 * t)
        return q, q_dot / self.T, q_ddot / self.T**2
        # return q, q_dot, q_ddot
