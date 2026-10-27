import matplotlib.pyplot as plt
import numpy as np


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
