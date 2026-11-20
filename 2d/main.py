import numpy as np

from hardware.parameters import make_cube
from simulation.simulation_runner import SimulationRunner

import visualization.plotter as plotter

cube = make_cube()
brushless_motor = cube.brushless_motor
servo_motor = cube.servo_motor
flywheel = cube.flywheel
gimbal = cube.gimbal

simul = SimulationRunner(cube)
x0 = [np.deg2rad(40.6), 0, 0, 0]

if __name__ == "__main__":
    plotter.plot_result(simul.run(x0, 0, 0))