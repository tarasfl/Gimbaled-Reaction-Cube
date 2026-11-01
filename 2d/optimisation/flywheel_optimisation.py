from itertools import product

from types import SimpleNamespace

import numpy as np

from hardware.flywheel import Flywheel
from hardware.cube import Cube
from simulation.evaluate_simulations import sweep
from visualization.csv_io import save_csv
from visualization.plotter import plot_result

ro_values = [0.050, 0.055, 0.060, 0.063]  # metres
ri_values = [0.025, 0.035, 0.045]
thickness_values = [0.006, 0.010, 0.014]

i_d = [-10, 0.0]
t_d = [ 90.0]

def compose_candidates(outer_radius_values, inner_radius_values, thickness_values):
    initial_candidates = [
        SimpleNamespace(outer_radius=ro, inner_radius=ri, thickness=h)
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


def optimise_flywheel(cube, weights=(1.0, 1.0, 1.0), steps=3, initial_degs=i_d, target_degs=t_d, outer_radius_values=ro_values, inner_radius_values=ri_values, thickness_values=thickness_values, csv_specs="flywheel_optimisation"):
    candidates = compose_candidates(outer_radius_values, inner_radius_values, thickness_values)
    sweep_results = []
    for _ in range(steps):
        print("_____________________STEP{}______________________".format(_ + 1))
        for i in candidates:
            print("Evaluating candidate:", i)
            flywheel = Flywheel(outer_radii=i.outer_radius, inner_radii=i.inner_radius, thickness=i.thickness)
            cube = Cube(size=cube.size, cage_mass=cube.cage_mass, flywheel=flywheel, brushless_motor=cube.brushless_motor, servo_motor=cube.servo_motor, gimbal=cube.gimbal)
            
            runs = sweep(cube, rpms=range(1000, 5001, 1000), initial_degs=initial_degs, target_degs=target_degs)

            if not runs:
                print("No successful runs found for candidate:", i)
            
            for run in runs:
                run.cube_total_mass = float(cube.total_mass)
                run.flywheel_mass = float(flywheel.mass)
                run.outer_radius = i.outer_radius
                run.inner_radius = i.inner_radius
                run.thickness = i.thickness
            
            sweep_results.extend(runs)
        best = select_best_weighted(sweep_results, weights=weights)
        outer_radius_values = refined_values(outer_radius_values, best.outer_radius)
        inner_radius_values = refined_values(inner_radius_values, best.inner_radius)
        thickness_values = refined_values(thickness_values, best.thickness)
        candidates = compose_candidates(
            outer_radius_values,
            inner_radius_values,
            thickness_values
        )

    save_optimisation_csv(sweep_results, cube, specs=csv_specs)
   
    summary = SimpleNamespace(**vars(best))
    del summary.result
    print(summary)
    plot_result(best.result)
    return best


def select_best_weighted(runs, weights=(1.0, 1.0, 1.0)):
    """Return the run with the lowest weighted, min-max normalised score.

    Minimises flywheel mass, flip time and target RPM; weights follow that order.
    """
    if not runs:
        raise ValueError("No runs to select from.")

    values = np.column_stack((
        [run.flywheel_mass for run in runs],
        [run.flip_time_s for run in runs],
        [run.target_rpm for run in runs],
    )).astype(float)

    minimums = values.min(axis=0)
    ranges = np.ptp(values, axis=0)

    # A characteristic identical across all candidates contributes zero.
    normalized = np.divide(
        values - minimums,
        ranges,
        out=np.zeros_like(values),
        where=ranges != 0,
    )

    weights = np.asarray(weights, dtype=float)
    scores = normalized @ weights / weights.sum()

    return runs[int(np.argsort(scores)[0])]


def refined_values(values, best_value):
    # Assumes values are sorted and contain no duplicates.
    index = values.index(best_value)

    lower = values[max(0, index - 1)]
    upper = values[min(len(values) - 1, index + 1)]

    return sorted({round((lower + best_value) / 2, 9), best_value, round((best_value + upper) / 2, 9),})


# Small secondary weights break ties between runs with equal primary metric.
def optimise_flywheel_fastest(cube):
    return optimise_flywheel(
        cube,
        weights=(1e-6, 1.0, 0.0),
        csv_specs="flywheel_fastest",
    )


def optimise_flywheel_lightest(cube):
    return optimise_flywheel(
        cube,
        weights=(1.0, 1e-6, 0.0),
        csv_specs="flywheel_lightest",
    )


def optimise_flywheel_weighted(cube, weights=(1.0, 1.0, 1.0)):
    return optimise_flywheel(
        cube,
        weights=weights,
        csv_specs="flywheel_weighted",
    )
