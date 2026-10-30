import numpy as np
from types import SimpleNamespace


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


def select_best(successful, tolerance=1.05):
    fastest_time = min(r.flip_time_s for r in successful)
    candidates = [
        r for r in successful
        if r.flip_time_s <= tolerance * fastest_time
    ]
    return min(
        candidates,
        key=lambda r: (
            r.gimbal_travel_deg,
            r.final_angular_velocity_deg_s,
        ),
    )