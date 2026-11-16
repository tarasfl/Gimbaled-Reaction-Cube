"""Save scene-simulation summaries in the repository's optimization CSV format."""
import sys
from types import SimpleNamespace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from visualization.csv_io import save_csv
from visualization.plotter import plot_result


def plot_scene_result(result, cube, show=True):
    """Adapt the Drake trajectory to the project's standard simulation plot."""
    plot_data = SimpleNamespace(
        t=result["times"],
        y=np.vstack([
            result["cube_angles"],
            result["cube_rates"],
            np.asarray(result["gimbal_angles"]) - np.pi / 2,
            result["gimbal_rates"],
        ]),
        rpm=np.asarray(result["rates"]) * 60 / (2 * np.pi),
        gimbal_torque=result["torques"],
        controller=SimpleNamespace(max_torque=cube.servo_motor.torque, cube=cube),
    )
    return plot_result(plot_data, cube=cube, show=show)


def save_scene_result(result, cube, target_rpm, initial_gamma_deg, target_gamma_deg):
    times = np.asarray(result["times"], dtype=float)
    cube_angles = np.asarray(result["cube_angles"], dtype=float)
    cube_rates = np.asarray(result["cube_rates"], dtype=float)
    gimbal_angles = np.asarray(result["gimbal_angles"], dtype=float) - np.pi / 2
    gimbal_rates = np.asarray(result["gimbal_rates"], dtype=float)
    torques = np.asarray(result["torques"], dtype=float)

    sample_count = len(times)
    if sample_count < 2 or any(
        len(values) != sample_count
        for values in (cube_angles, cube_rates, gimbal_angles, gimbal_rates, torques)
    ):
        raise ValueError("Scene telemetry arrays must have matching lengths and at least two samples.")
    if not all(np.isfinite(values).all() for values in (
        times, cube_angles, cube_rates, gimbal_angles, gimbal_rates, torques,
    )) or np.any(np.diff(times) <= 0):
        raise ValueError("Scene telemetry must be finite and time-ordered.")

    flip_indices = np.flatnonzero(cube_angles >= np.pi / 2 - 1e-8)
    flipped = flip_indices.size > 0
    end_index = int(flip_indices[0]) if flipped else sample_count - 1
    times_to_end = times[:end_index + 1]
    gimbal_angles_to_end = gimbal_angles[:end_index + 1]
    gimbal_rates_to_end = gimbal_rates[:end_index + 1]
    torques_to_end = torques[:end_index + 1]

    positive_work = float(np.trapezoid(
        np.maximum(torques_to_end * gimbal_rates_to_end, 0.0),
        times_to_end,
    ))
    metrics = SimpleNamespace(
        initial_gamma_deg=float(initial_gamma_deg),
        target_gamma_deg=float(target_gamma_deg),
        target_rpm=float(target_rpm),
        flywheel_rpm=float(result["rates"][-1] * 60 / (2 * np.pi)),
        flip_time_s=float(times[end_index]) if flipped else np.nan,
        gimbal_travel_deg=float(np.rad2deg(np.sum(np.abs(np.diff(gimbal_angles_to_end))))),
        gimbal_angle_at_flip_deg=float(np.rad2deg(gimbal_angles[end_index])) if flipped else np.nan,
        positive_work_j=positive_work,
        peak_torque_nm=float(np.max(np.abs(torques_to_end))),
        peak_gimbal_speed_deg_s=float(np.rad2deg(np.max(np.abs(gimbal_rates_to_end)))),
        final_angular_velocity_deg_s=(
            float(np.rad2deg(abs(cube_rates[end_index]))) if flipped else np.nan
        ),
        candidate={
            "outer_radii": cube.flywheel.outer_radii,
            "inner_radii": cube.flywheel.inner_radii,
            "thickness": cube.flywheel.thickness,
        },
        component_mass=float(cube.flywheel.mass),
        cube_total_mass=float(cube.total_mass),
    )
    metadata = SimpleNamespace(
        cube_size=float(cube.size),
        cube_cage_mass=float(cube.cage_mass),
        cube_brushless_motor_mass=float(cube.brushless_motor.mass),
        cube_brushless_motor_nominal_rpm=float(cube.brushless_motor.nominal_rpm),
        cube_brushless_motor_nominal_torque=float(cube.brushless_motor.nominal_torque),
        cube_servo_motor_mass=float(cube.servo_motor.mass),
        cube_servo_motor_torque=float(cube.servo_motor.torque),
        cube_servo_motor_max_rpm=float(cube.servo_motor.max_rpm),
        cube_gimbal_mass=float(cube.gimbal.mass),
        cube_gimbal_outer_radii=float(cube.gimbal.outer_radii),
        cube_gimbal_inner_radii=float(cube.gimbal.inner_radii),
    )
    save_csv([metrics], specs="scene_simulation", metadata=metadata)
