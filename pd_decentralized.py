import matplotlib.pyplot as plt
import numpy as np
from numpy import pi
from scipy.integrate import odeint

from controllers.adrc_controller import ADRController
from controllers.pd_controller import PDDecentralizedController
from trajectory_generators.constant_torque import ConstantTorque
from trajectory_generators.sinusonidal import Sinusoidal
from trajectory_generators.poly3 import Poly3
from utils.simulation import simulate


Tp = 0.001
end = 1

# traj_gen = ConstantTorque(np.array([0., 1.0])[:, np.newaxis])
# traj_gen = Sinusoidal(np.array([0., 1.]), np.array(
#     [2., 2.]), np.array([0., 0.]))
traj_gen = Poly3(np.array([0., 0.]), np.array([np.pi/2, np.pi/2]), end)

kp_est_1 = 1000
kp_est_2 = 1000
kd_est_1 = 50
kd_est_2 = 50

kp_pd = [kp_est_1, kp_est_2]
kd_pd = [kd_est_1, kd_est_2]
controller = PDDecentralizedController(kp_pd, kd_pd)

Q, Q_d, u, T = simulate("PYBULLET", traj_gen, controller, Tp, end)

plt.subplot(221)
plt.plot(T, Q[:, 0], 'r')
plt.plot(T, Q_d[:, 0], 'b')
plt.subplot(222)
plt.plot(T, Q[:, 1], 'r')
plt.plot(T, Q_d[:, 1], 'b')
plt.subplot(223)
plt.plot(T, u[:, 0], 'r')
plt.plot(T, u[:, 1], 'b')
plt.show()
