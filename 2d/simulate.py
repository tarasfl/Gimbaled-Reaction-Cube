
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import numpy as np
from dynamics_2d import Dynamics2D

from main import cube

# Initial conditions
x0 = [
    0.0,  # theta
    0.0,  # theta_dot
    0.0,  # gamma
    0.0   # gamma_dot
]

dynamics = Dynamics2D(cube)

t_start = 0
t_end = 10.0

t_eval = np.linspace(t_start, t_end, 10001)

solution = solve_ivp(
    fun=dynamics.derivatives,
    t_span=(t_start, t_end),
    y0=x0,
    method="RK45",
    t_eval=t_eval,
    max_step=0.001,
    rtol=1e-7,
    atol=1e-9
)

theta, theta_dot, gamma, gamma_dot = solution.y

plt.plot(solution.t, np.rad2deg(theta), label="Cube angle")
plt.plot(solution.t, np.rad2deg(gamma), label="Gimbal angle")

plt.xlabel("Time (s)")
plt.ylabel("Angle (degrees)")
plt.legend()
plt.grid()
plt.show()
