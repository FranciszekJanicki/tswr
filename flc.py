import matplotlib.pyplot as plt
import numpy as np

from controllers.dummy_controller import DummyController
from controllers.feedback_linearization_controller import FeedbackLinearizationController
from trajectory_generators.constant_torque import ConstantTorque
from trajectory_generators.sinusonidal import Sinusoidal
from trajectory_generators.poly3 import Poly3
from utils.simulation import simulate

Tp = 0.01
start = 0
end = 3

"""
Switch to FeedbackLinearizationController as soon as you implement it
"""
controller = FeedbackLinearizationController(Tp)
# controller = DummyController(Tp)

"""
Here you have some trajectory generators. You can use them to check your implementations.
At the end implement Point2point trajectory generator to move your manipulator to some desired state.
"""
# traj_gen = ConstantTorque(np.array([0., 1.0])[:, np.newaxis])
traj_gen = Sinusoidal(np.array([0., 1.]), np.array(
    [2., 2.]), np.array([0., 0.]))
# traj_gen = Poly3(np.array([0., 0.]), np.array([np.pi/2, np.pi/2]), end)


Q, Q_d, u, T = simulate("PYBULLET", traj_gen, controller, Tp, end)


"""
You can add here some plots of the state 'Q' (consists of q and q_dot), controls 'u', desired trajectory 'Q_d'
with respect to time 'T' to analyze what is going on in the system
"""
# --- Wykres 1: Pozycja pierwszego przegubu (q1) ---
plt.subplot(221)
plt.plot(T, Q[:, 0], 'r', label='Rzeczywista (q1)')
plt.plot(T, Q_d[:, 0], 'b--', label='Zadana (q1_d)')
plt.title("Przegub 1: Pozycja")
plt.xlabel("Czas [s]")
plt.ylabel("Kąt [rad]")
plt.legend()
plt.grid(True)

# --- Wykres 2: Pozycja drugiego przegubu (q2) ---
plt.subplot(222)
plt.plot(T, Q[:, 1], 'r', label='Rzeczywista (q2)')
plt.plot(T, Q_d[:, 1], 'b--', label='Zadana (q2_d)')
plt.title("Przegub 2: Pozycja")
plt.xlabel("Czas [s]")
plt.ylabel("Kąt [rad]")
plt.legend()
plt.grid(True)

# --- Wykres 3: Sygnały sterujące (Momenty sił) ---
plt.subplot(223)
plt.plot(T, u[:, 0], 'r', label='Silnik 1 (tau1)')
plt.plot(T, u[:, 1], 'b', label='Silnik 2 (tau2)')
plt.title("Sygnały sterujące (Momenty sił)")
plt.xlabel("Czas [s]")
plt.ylabel("Moment obrotowy [Nm]")
plt.legend()
plt.grid(True)

# plt.tight_layout() dba o to, żeby napisy nie nachodziły na siebie
plt.tight_layout()
plt.show()
