from functools import partial

import numpy as np
from itertools import product

from hardware.servo_motor import ServoMotor
from optimisation.optimisation import optimise, select_best_weighted

from visualization.csv_io import save_csv


def compose_candidates(power_limit_w, rpm_values, torque_values):
    candidates = [
    {
        "nominal_torque": float(torque),
        "max_rpm": float(rpm),
    }
    for torque, rpm in product(torque_values, rpm_values)
    if torque * rpm * 2 * np.pi / 60 <= power_limit_w
]
    return candidates


def make_servo(nominal_torque, max_rpm, reference, power_limit_w):
    values = [nominal_torque, max_rpm, power_limit_w]

    if not np.isfinite(values).all() or min(values) <= 0:
        raise ValueError("Torque, RPM and power limit must be positive and finite.")

    rated_power = nominal_torque * max_rpm * 2 * np.pi / 60

    if rated_power > power_limit_w:
        raise ValueError("Candidate exceeds the power limit.")

    return ServoMotor(
        mass=reference.mass,
        radius=reference.radius,
        length=reference.length,
        rotor_inertia=reference.rotor_inertia,
        nominal_torque=nominal_torque,
        max_rpm=max_rpm,
    )

def optimise_servo(cube, flywheel_rpm, initial_degs=[-10, 0], target_degs=[60, 90], power_limit_w=50.0, steps=3, duration=15.0):
    candidates = compose_candidates(power_limit_w, np.geomspace(100, 500, 7), np.geomspace(0.1, 10, 7))

    weights = {"torque": 1.0, "max_rpm": 0.75, "flip_time_s": 0.75}
    successful_runs = optimise(
        candidates=candidates,
        base_cube=cube,
        component_attribute="servo_motor",
        constructor=partial(
            make_servo,
            reference=cube.servo_motor,
            power_limit_w=power_limit_w,
            ),
        weights=weights,
        rpms=[flywheel_rpm],
        initial_degs=initial_degs,
        target_degs=target_degs,
        refinement_steps=steps,
        duration=duration,
    )
    save_csv(successful_runs, specs="SERVO_FULL_RANGE_TEST")

    return select_best_weighted(successful_runs, weights)