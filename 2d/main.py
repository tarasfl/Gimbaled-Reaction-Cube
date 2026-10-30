from hardware.brushless_motor import BrushlessMotor
from hardware.servo_motor import ServoMotor
from hardware.flywheel import Flywheel
from hardware.gimbal import Gimbal
from hardware.cube import Cube
import numpy as np
from simulation.simulation_runner import SimulationRunner
from visualization.plotter import plot_result
from simulation.evaluate_simulations import evaluate_run, select_best

from itertools import product
from types import SimpleNamespace

brushless_motor = BrushlessMotor(nominal_rpm = 10000, nominal_torque = 0.118, rotor_inertia = 8.75*10**(-6), mass=0.071, radius=21*10**-3, length=0.05)
servo_motor = ServoMotor(mass=0.190, radius=0.053/2, length=0.04, rotor_inertia=97*1e-7, nominal_torque=1.6, max_rpm=370)
flywheel = Flywheel()
gimbal = Gimbal(mass=0.2, outter_radii=0.07, inner_radii=0.065)


cube = Cube(size=0.15, cage_mass=1, flywheel=flywheel, brushless_motor=brushless_motor, servo_motor=servo_motor, gimbal=gimbal)



# Initial conditions
x0 = [
    0.0,  # theta
    0.0,  # theta_dot
    0.0,  # gamma
    0.0   # gamma_dot
]


def sweep(cube, rpms, initial_degs, target_degs, t_end=10.0):
    successful = []
    for target_rpm in rpms:
        print("-------------------------------")
        print(f"Running simulations for target_rpm={target_rpm}")
        for initial_deg, target_deg in product(initial_degs, target_degs):
            x0 = [0.0, 0.0, np.deg2rad(initial_deg), 0.0]

            print(f"Running simulation for initial_gamma_deg={initial_deg}, target_gamma_deg={target_deg}")

            result = SimulationRunner(cube).run(
                x0,
                desired_gamma_deg=target_deg,
                t_end=t_end,
                target_rpm=target_rpm,
            )

            metrics = evaluate_run(result)
            if metrics is None:
                continue

            successful.append(SimpleNamespace(
                initial_gamma_deg=initial_deg,
                target_gamma_deg=target_deg,
                flywheel_rpm=float(result.rpm[-1]),
                **vars(metrics),
                result=result,  # Keep the trajectory for plotting.
            ))
    return successful



def main():
    successful = sweep(
        cube,
        rpms=range(1000, 10001, 1000),
        initial_degs=[-30.0, -20.0, -10.0, 0.0, 30.0],
        target_degs=[30.0, 60.0, 75.0, 90.0],
    )

    if not successful:
        print("No successful runs found.")
        return

    best = select_best(successful)
    print({k: v for k, v in vars(best).items() if k != "result"})
    plot_result(best.result)


if __name__ == "__main__":
    main()
    