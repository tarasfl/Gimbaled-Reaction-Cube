import matplotlib.pyplot as plt
import numpy as np


def _finish(xlabel, ylabel):
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.legend()
    plt.grid()


def plot_result(result, show=True):
    # Plot angles, speeds and gimbal torque from a SimulationRunner result
    theta, _, gamma, gamma_dot = result.y
    max_torque = result.controller.max_torque

    plt.figure()
    plt.plot(result.t, np.rad2deg(theta), label="Cube angle")
    plt.plot(result.t, np.rad2deg(gamma), label="Gimbal angle")
    _finish("Time (s)", "Angle (degrees)")

    plt.figure()
    plt.plot(result.t, result.rpm, label="Flywheel RPM")
    plt.plot(result.t, gamma_dot * 60 / (2 * np.pi), label="Gimbal motor RPM")
    _finish("Time (s)", "Speed (RPM)")

    plt.figure()
    plt.plot(result.t, result.gimbal_torque, label="Gimbal torque")
    plt.axhline(max_torque, color="r", linestyle="--", label="Torque limit")
    plt.axhline(-max_torque, color="r", linestyle="--")
    _finish("Time (s)", "Torque (N·m)")

    if show:
        plt.show()
