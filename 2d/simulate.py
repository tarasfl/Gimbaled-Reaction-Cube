import numpy as np
from visualization.plotter import plot_result
from simulation.simulation_runner import SimulationRunner

from main import cube

# Initial conditions
x0 = [
    0.0,  # theta
    0.0,  # theta_dot
    np.deg2rad(-10.0),  # gamma
    0.0   # gamma_dot
]

runner = SimulationRunner(cube)
result = runner.run(x0, desired_gamma_deg=90.0, t_end=10.0)

plot_result(result)


print(np.rad2deg(result.y[:][0]))
print("Cube total mass: ", cube.total_mass)