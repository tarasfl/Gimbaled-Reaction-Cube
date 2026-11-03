from itertools import product

from types import SimpleNamespace

import numpy as np

from hardware.flywheel import Flywheel
from hardware.cube import Cube
from simulation.evaluate_simulations import sweep
from visualization.csv_io import save_csv
from visualization.plotter import plot_result

from optimisation.optimisation import optimise, select_best_weighted

ro_values = [0.060, 0.063]  # metres
ri_values = [0.03, 0.035, 0.045]
thickness_values = [0.008, 0.010, 0.014]

i_d = [-10, 0.0]
t_d = [ 60.0 ,90.0]

def compose_candidates(outer_radius_values, inner_radius_values, thickness_values):
    initial_candidates = [
        {
            "outer_radii": ro,
            "inner_radii": ri,
            "thickness": h,
        }
        for ro, ri, h in product(
            outer_radius_values, inner_radius_values, thickness_values
        )
        if ri < ro
    ]

    return initial_candidates

def save_optimisation_csv(results, cube, specs="flywheel_optimisation"):
    cube_metadata = SimpleNamespace(
        cube_size=cube.size,
        cube_cage_mass=cube.cage_mass,
        cube_brushless_motor_mass=cube.brushless_motor.mass,
        cube_brushless_motor_nominal_rpm=cube.brushless_motor.nominal_rpm,
        cube_brushless_motor_nominal_torque=cube.brushless_motor.nominal_torque,
        cube_servo_motor_mass=cube.servo_motor.mass,
        cube_servo_motor_torque=cube.servo_motor.torque,
        cube_servo_motor_max_rpm=cube.servo_motor.max_rpm,
        cube_gimbal_mass=cube.gimbal.mass,
        cube_gimbal_outer_radii=cube.gimbal.outer_radii,
        cube_gimbal_inner_radii=cube.gimbal.inner_radii,
    )
    return save_csv(results, specs=specs, metadata=cube_metadata)


def optimise_flywheel(cube, weights=None, steps=3, initial_degs=i_d, target_degs=t_d, outer_radius_values=ro_values, inner_radius_values=ri_values, thickness_values=thickness_values, csv_specs="flywheel_optimisation"):
    if weights is None:
        weights = {
            "component_mass": 1.0,
            "flip_time_s": 1.0,
            "target_rpm": 1.0,
        }

    successful_runs = optimise(
        candidates=compose_candidates(outer_radius_values, inner_radius_values, thickness_values),
        base_cube=cube,
        component_attribute="flywheel",
        constructor=Flywheel,
        weights=weights,
        rpms=range(1000, 5001, 1000),
        initial_degs=initial_degs,
        target_degs=target_degs,
        refinement_steps=steps,
    )
    save_optimisation_csv(successful_runs, cube, specs=csv_specs)
    best = select_best_weighted(successful_runs, weights)
    return best

# Small secondary weights break ties between runs with equal primary metric.
def optimise_flywheel_fastest(cube):
    return optimise_flywheel(
        cube,
        weights={
            "component_mass": 1e-6,
            "flip_time_s": 1.0,
            "target_rpm": 0.0,
        },
        csv_specs="flywheel_fastest",
    )


def optimise_flywheel_lightest(cube):
    return optimise_flywheel(
        cube,
        weights={
            "component_mass": 1.0,
            "flip_time_s": 1e-6,
            "target_rpm": 0.0,
        },
        csv_specs="flywheel_lightest",
    )
