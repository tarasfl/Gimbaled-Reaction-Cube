import numpy as np
from types import SimpleNamespace
from itertools import product
from simulation.simulation_runner import SimulationRunner


def evaluate_run(result):
    t = result.t
    theta, theta_dot, gamma, gamma_dot = result.y
    start = result.controller.control_start

    # Reject runs stopped by the existing overspeed event.
    if result.stopped_by == 0:
        return None

    # Small numerical tolerance around the 90-degree target.
    hits = np.flatnonzero(
        (t >= start) & (theta >= np.pi / 2 - 1e-8)
    )
    if hits.size == 0:
        return None

    flip_index = int(hits[0])
    if flip_index == 0 or t[flip_index] <= start:
        return None

    # Include the exact control-start time by interpolation.
    times = np.concatenate(([start], t[(t > start) & (t <= t[flip_index])]))
    angles = np.interp(times, t, gamma)
    rates = np.interp(times, t, gamma_dot)
    torque = np.interp(times, t, result.gimbal_torque)

    positive_power = np.maximum(torque * rates, 0.0)


    return SimpleNamespace(
        flip_time_s=float(t[flip_index] - start),
        gimbal_travel_deg=float(
            np.rad2deg(np.sum(np.abs(np.diff(angles))))
        ),
        gimbal_angle_at_flip_deg=float(
            np.rad2deg(gamma[flip_index])
        ),
        positive_work_j=float(
            np.trapezoid(positive_power, times)
        ),
        peak_torque_nm=float(np.max(np.abs(torque))),
        peak_gimbal_speed_deg_s=float(
            np.rad2deg(np.max(np.abs(rates)))
        ),
        final_angular_velocity_deg_s=float(
            np.rad2deg(abs(theta_dot[flip_index - 1]))
        ),
        landing_sample_gap_s=float(
            t[flip_index] - t[flip_index - 1]
        )
    )


def select_best(successful, key=None):
    if key is None:
        key = lambda run: run.flip_time_s
    return min(successful, key=key)


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
                target_rpm=target_rpm,
                flywheel_rpm=float(result.rpm[-1]),
                **vars(metrics),
                result=result,  # Keep the trajectory for plotting.
            ))
    return successful
