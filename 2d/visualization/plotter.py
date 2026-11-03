import matplotlib.pyplot as plt
import numpy as np

from visualization.csv_io import fastest_row, load_csv
from hardware.flywheel import Flywheel
from simulation.simulation_runner import SimulationRunner


def _finish(ax, xlabel, ylabel):
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.legend()
    ax.grid()


def _characteristics_text(cube):
    # Summarize the cube's hardware so it can be read alongside the plots.
    flywheel, gimbal = cube.flywheel, cube.gimbal
    brushless, servo = cube.brushless_motor, cube.servo_motor

    lines = [
        "Cube characteristics",
        f"Size: {cube.size * 1000:.1f} mm",
        f"Total mass: {cube.total_mass * 1000:.1f} g",
        "",
        "Flywheel",
        f"  Mass: {flywheel.mass * 1000:.1f} g",
        f"  Radii (outer/inner): {flywheel.outer_radii * 1000:.1f} / {flywheel.inner_radii * 1000:.1f} mm",
        f"  Thickness: {flywheel.thickness * 1000:.1f} mm",
        f"  Max RPM: {flywheel.max_rpm:.0f}",
        "",
        "Brushless motor (flywheel drive)",
        f"  Mass: {brushless.mass * 1000:.1f} g",
        f"  Nominal torque: {brushless.nominal_torque:.4f} N·m",
        f"  Nominal RPM: {brushless.nominal_rpm:.0f}",
        "",
        "Servo motor (gimbal drive)",
        f"  Mass: {servo.mass * 1000:.1f} g",
        f"  Torque: {servo.torque:.4f} N·m",
        f"  Max RPM: {servo.max_rpm:.0f}",
        "",
        "Gimbal",
        f"  Mass: {gimbal.mass * 1000:.1f} g",
    ]
    return "\n".join(lines)


def plot_result(result, cube=None, show=True):
    # Plot angles, speeds, gimbal torque and cube characteristics in one window.
    theta, _, gamma, gamma_dot = result.y
    max_torque = result.controller.max_torque
    cube = cube or result.controller.cube

    fig, ((ax_angle, ax_speed), (ax_torque, ax_info)) = plt.subplots(2, 2, figsize=(12, 8))

    ax_angle.plot(result.t, np.rad2deg(theta), label="Cube angle")
    ax_angle.plot(result.t, np.rad2deg(gamma), label="Gimbal angle")
    _finish(ax_angle, "Time (s)", "Angle (degrees)")

    ax_speed.plot(result.t, result.rpm, label="Flywheel RPM")
    ax_speed.plot(result.t, gamma_dot * 60 / (2 * np.pi), label="Gimbal motor RPM")
    _finish(ax_speed, "Time (s)", "Speed (RPM)")

    ax_torque.plot(result.t, result.gimbal_torque, label="Gimbal torque")
    ax_torque.axhline(max_torque, color="r", linestyle="--", label="Torque limit")
    ax_torque.axhline(-max_torque, color="r", linestyle="--")
    _finish(ax_torque, "Time (s)", "Torque (N·m)")

    ax_info.axis("off")
    ax_info.text(
        0.0, 1.0, _characteristics_text(cube),
        transform=ax_info.transAxes,
        va="top", ha="left",
        family="monospace", fontsize=9,
    )

    fig.tight_layout()

    if show:
        plt.show()


def _row_flywheel_mass(row):
    if getattr(row, "flywheel_mass", None) is not None:
        return row.flywheel_mass
    # Older CSVs have no mass column, so derive it from the flywheel geometry.
    return Flywheel(
        outer_radii=row.outer_radius,
        inner_radii=row.inner_radius,
        thickness=row.thickness,
    ).mass


def plot_mass_vs_flip_time_from_csv(path, show=True):
    rows = load_csv(path)
    if not rows:
        print("CSV is empty.")
        return

    masses = np.array([_row_flywheel_mass(r) for r in rows]) * 1000
    flip_times = np.array([r.flip_time_s for r in rows])

    rpms = np.array([r.target_rpm for r in rows])

    unique_rpms = np.unique(rpms)
    n_plots = len(unique_rpms) + 1  # combined plot + one per RPM
    cols = min(n_plots, 3)
    n_rows = -(-n_plots // cols)
    fig, axes = plt.subplots(
        n_rows, cols, figsize=(5 * cols, 4 * n_rows),
        sharex=True, sharey=True, squeeze=False,
    )
    ax_all, rpm_axes = axes.flat[0], axes.flat[1:]

    for rpm in unique_rpms:
        mask = rpms == rpm
        ax_all.scatter(masses[mask], flip_times[mask], s=15, label=f"{rpm:.0f} RPM")
    best = int(np.argmin(flip_times))
    ax_all.scatter(masses[best], flip_times[best], color="r", zorder=3, label="Fastest")
    ax_all.set_title("All RPMs")
    _finish(ax_all, "Flywheel mass (g)", "Flip time (s)")

    for ax, rpm in zip(rpm_axes, unique_rpms):
        mask = rpms == rpm
        ax.scatter(masses[mask], flip_times[mask], s=15, label="Runs")

        best = np.flatnonzero(mask)[int(np.argmin(flip_times[mask]))]
        ax.scatter(masses[best], flip_times[best], color="r", zorder=3, label="Fastest")
        ax.set_title(f"{rpm:.0f} RPM")
        _finish(ax, "Flywheel mass (g)", "Flip time (s)")

    for ax in rpm_axes[len(unique_rpms):]:
        ax.axis("off")

    fig.tight_layout()
    if show:
        plt.show()


def plot_fastest_from_csv(cube, path="sweep_results.csv"):
    rows = load_csv(path)
    if not rows:
        print("CSV is empty.")
        return
    best = fastest_row(rows)
    print(best)

    cube.flywheel = Flywheel(
        outer_radii=best.outer_radius,
        inner_radii=best.inner_radius,
        thickness=best.thickness,
    )
    x0 = [0.0, 0.0, np.deg2rad(best.initial_gamma_deg), 0.0]
    result = SimulationRunner(cube).run(
        x0,
        desired_gamma_deg=best.target_gamma_deg,
        target_rpm=best.flywheel_rpm,
        t_end=10.0,
    )
    plot_result(result)

