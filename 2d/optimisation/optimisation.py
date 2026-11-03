from copy import deepcopy
from itertools import product
from math import isfinite
from numbers import Real
from hardware.cube import Cube

from simulation.evaluate_simulations import sweep


def optimise(candidates, base_cube: Cube, component_attribute, constructor, weights, rpms, initial_degs, target_degs, refinement_steps=3, duration=10.0,):
    """Evaluate candidates, refine around the best run, and return that run

    candidates - list of dictionaries, each representing a candidate configuration of the component. for example, [{'iner_r': value1, 'outter_r': value2}, ...]
    base_cube - an instance of the Cube class representing the base configuration of the cube.
    component_attribute is a string naming the attribute on the cube that should be replaced by the newly constructed component. 
    constructor - a callable that constructs the component from a candidate dictionary.
    weights - a dictionary mapping run metric attribute names to non-negative weights.
    rpms - a sequence of RPM values to sweep over.
    initial_degs - a sequence of initial degrees to sweep over.
    target_degs - a sequence of target degrees to sweep over.
    refinement_steps - the maximum number of refinement rounds after the initial candidate batch.
    duration - the duration of each simulation run.
    """

    candidates = [dict(candidate) for candidate in candidates]

    rpms = tuple(rpms)
    initial_degs = tuple(initial_degs)
    target_degs = tuple(target_degs)

    _validate_candidate_keys(candidates)
    evaluated_candidates = []
    successful_runs = []

    for step in range(refinement_steps + 1):
        pending_candidates = []
        for candidate in candidates:
            if (candidate not in evaluated_candidates and candidate not in pending_candidates):
                pending_candidates.append(candidate)
        if not pending_candidates:
            break

        print(f"_____________________STEP{step + 1}______________________")
        for candidate in pending_candidates:
            evaluated_candidates.append(dict(candidate))
            print("Evaluating candidate:", candidate)

            try:
                component = constructor(**candidate)
            except ValueError as error:
                print(f"Rejected invalid candidate {candidate}: {error}")
                continue
            cube = deepcopy(base_cube)
            setattr(cube, component_attribute, component)
            cube.update_total_mass()

            runs = sweep(
                cube,
                rpms=rpms,
                initial_degs=initial_degs,
                target_degs=target_degs,
                t_end=duration,
            )

            if not runs:
                print("No successful runs found for candidate:", candidate)
                continue

            for run in runs:
                run.candidate = dict(candidate)
                run.component_mass = float(component.mass)
                run.cube_total_mass = float(cube.total_mass)
                for metric in weights:
                    if not hasattr(run, metric):
                        setattr(run, metric, getattr(component, metric))
            successful_runs.extend(runs)

        if not successful_runs:
            raise ValueError("No successful simulation runs were produced.")

        if step == refinement_steps:
            break

        best = select_best_weighted(successful_runs, weights)
        candidates = refine_candidates(evaluated_candidates, best.candidate)
        if not any(candidate not in evaluated_candidates for candidate in candidates):
            break

    return successful_runs


def select_best_weighted(runs, weights):
    """Select the run with the lowest weighted min-max-normalised score."""

    metrics = list(weights)
    # metric_weights - each metrics numeric weight weight, in the same order as metrics
    metric_weights = []
    for metric, weight in weights.items():
        numeric_weight = float(weight)
        metric_weights.append(numeric_weight)

    weight_total = sum(metric_weights)
    # metric_values - list of list, one list per each run containing val;ues in metrics order 
    metric_values = []
    # iterate over each run to collect metric values
    for run in runs:
        run_values = []
        for metric in metrics:
            value = float(getattr(run, metric))
            run_values.append(value)
        metric_values.append(run_values)

    # minima - minimum values for each metric across all runs
    minima = [min(values[index] for values in metric_values) for index in range(len(metrics))]
    # ranges - max - min for each metric, used to normalise values
    ranges = [max(values[index] for values in metric_values) - minima[index] for index in range(len(metrics))]
    # scores - list of weighted normalised scores for each run
    scores = []
        
    for values in metric_values:
        score = sum(weight * (0.0 if metric_range == 0 else (value - minimum) / metric_range)
                for value, minimum, metric_range, weight in zip(values, minima, ranges, metric_weights)) / weight_total
        scores.append(score)

    return runs[scores.index(min(scores))]


def refine_candidates(tested_candidates, best_candidate):
    # Generate a Cartesian refinement around the winning candidate 
    if not tested_candidates:
        return []
    _validate_candidate_keys(tested_candidates)

    numeric_parameters = _infer_numeric_parameters(tested_candidates)
    parameter_values = [
        refined_values(
            sorted({candidate[key] for candidate in tested_candidates}),
            best_candidate[key],
        )
        for key in numeric_parameters
    ]

    refined = []
    for values in product(*parameter_values):
        candidate = dict(best_candidate)
        candidate.update(zip(numeric_parameters, values))
        if candidate not in refined:
            refined.append(candidate)
    return refined


def refined_values(values, best_value):
    # Return the winner and midpoints to its nearest tested neighbours
    lower = max((value for value in values if value < best_value), default=best_value)
    upper = min((value for value in values if value > best_value), default=best_value)
    return sorted({
        (lower + best_value) / 2, best_value, (best_value + upper) / 2,
    })


def _infer_numeric_parameters(candidates):
    keys = candidates[0]
    return [
        key
        for key in keys
        if all(
            isinstance(candidate[key], Real) and not isinstance(candidate[key], bool)
            for candidate in candidates
        )
        and len({candidate[key] for candidate in candidates}) > 1
    ]


def _validate_candidate_keys(candidates):
    expected_keys = set(candidates[0])
    if any(set(candidate) != expected_keys for candidate in candidates):
        raise ValueError("All candidate dictionaries must have the same keys.")
