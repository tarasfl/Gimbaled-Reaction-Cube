from hardware.brushless_motor import BrushlessMotor
from hardware.servo_motor import ServoMotor
from hardware.flywheel import Flywheel
from hardware.gimbal import Gimbal
from hardware.cube import Cube
import numpy as np
from simulation.simulation_runner import SimulationRunner
from visualization.plotter import plot_result
from simulation.evaluate_simulations import evaluate_run

from itertools import product

brushless_motor = BrushlessMotor(nominal_rpm = 6290, nominal_torque = 0.0598, rotor_inertia = 8.75*10**(-6), mass=0.112, radius=21*10**-3, length=0.05)
servo_motor = ServoMotor(mass=0.112, radius=33*10**-3, length=0.134, rotor_inertia=8.75*10**-6, nominal_torque=0.198, max_rpm=1600)
flywheel = Flywheel()
gimbal = Gimbal(mass=0.2, outter_radii=0.07, inner_radii=0.065)


cube = Cube(size=0.15, flywheel=flywheel, brushless_motor=brushless_motor, servo_motor=servo_motor, gimbal=gimbal)



# Initial conditions
x0 = [
    0.0,  # theta
    0.0,  # theta_dot
    np.deg2rad(-10.0),  # gamma
    0.0   # gamma_dot
]



successful = []

for initial_deg, target_deg in product(
    [-30.0, -20.0, -10.0],
    [60.0, 75.0, 90.0],
):
    x0 = [0.0, 0.0, np.deg2rad(initial_deg), 0.0]

    result = SimulationRunner(cube).run(
        x0,
        desired_gamma_deg=target_deg,
        flywheel_rpm=5000.0,
        t_end=10.0,
    )

    metrics = evaluate_run(result)
    if metrics is None:
        continue

    successful.append({
        "initial_gamma_deg": initial_deg,
        "target_gamma_deg": target_deg,
        "flywheel_rpm": 5000.0,
        **vars(metrics),
        "result": result,  # Keep the trajectory for plotting.
    })

if not successful:
    print("No successful runs found.")
else:
    fastest_time = min(r["flip_time_s"] for r in successful)

    candidates = [
        r for r in successful
        if r["flip_time_s"] <= 1.05 * fastest_time
    ]

    best = min(
        candidates,
        key=lambda r: (
            r["gimbal_travel_deg"],
            r["final_angular_velocity_deg_s"],
        ),
    )

    for name, value in best.items():
        if name != "result":
            print(f"{name}: {value:.4f}")

    # Optional:
    plot_result(best["result"])