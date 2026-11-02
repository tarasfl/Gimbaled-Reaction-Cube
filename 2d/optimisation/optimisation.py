from types import SimpleNamespace

from hardware.flywheel import Flywheel
from hardware.cube import Cube
from simulation.evaluate_simulations import sweep
from visualization.plotter import plot_result
from optimisation.flywheel_optimisation import (
    ro_values,
    ri_values,
    thickness_values,
    i_d,
    t_d,
    compose_candidates,
    save_optimisation_csv,
    select_best_weighted,
    refined_values,
)


def optimise(cube, weights=(1.0, 1.0, 1.0), steps=3, initial_degs=i_d, target_degs=t_d, args=(ro_values, ri_values, thickness_values), csv_specs="flywheel_optimisation"):
    # args: [outer_radius_values, inner_radius_values, thickness_values]
    args = [list(values) for values in args]
    # Every value tried so far per parameter; refinement picks neighbours from these.
    tried = [set(values) for values in args]
    evaluated = set()
    candidates = compose_candidates(*args)
    sweep_results = []
    for step in range(steps):
        print("_____________________STEP{}______________________".format(step + 1))
        for i in candidates:
            key = (i.outer_radius, i.inner_radius, i.thickness)
            if key in evaluated:
                continue
            evaluated.add(key)
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
        # The best run decides where the next step zooms in.
        best_values = (best.outer_radius, best.inner_radius, best.thickness)
        args = [refined_values(sorted(t), b) for t, b in zip(tried, best_values)]
        for t, values in zip(tried, args):
            t.update(values)
        candidates = compose_candidates(*args)

    save_optimisation_csv(sweep_results, cube, specs=csv_specs)

    summary = SimpleNamespace(**vars(best))
    del summary.result
    print(summary)
    plot_result(best.result)
    return best
