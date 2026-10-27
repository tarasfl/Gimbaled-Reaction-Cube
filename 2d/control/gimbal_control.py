import numpy as np


class GimbalControl:
    """Saturated PD gimbal-angle controller with model-based coupling compensation."""

    def __init__(self, cube, t_eval, work_rpm, initial_angle, desired_angle_deg, ready_rpm=5000.0):
        self.cube = cube
        ready_indices = np.flatnonzero(work_rpm >= ready_rpm)
        self.control_start = t_eval[ready_indices[0]]

        self.desired_angle = np.deg2rad(desired_angle_deg)
        angle_change = self.desired_angle - initial_angle

        self.speed_limit = cube.servo_motor.max_rpm * 2 * np.pi / 60
        self.max_torque = cube.servo_motor.torque
        B = cube.gimbal.effective_inertia(
            flywheel=cube.flywheel,
            brushless_motor=cube.brushless_motor,
            servo_motor=cube.servo_motor,
        )

        move_distance = max(abs(angle_change), np.deg2rad(1.0))
        self.natural_frequency = 2.5 * min(
            np.sqrt(self.max_torque / (B * move_distance)),
            self.speed_limit / move_distance,
        )
        self.gimbal_inertia = B
        self.step_limit = 0.001

    def command(self, t, x, rpm):
        if t < self.control_start:
            return 0.0

        theta_dot, gamma, gamma_dot = x[1], x[2], x[3]
        angular_momentum = self.cube.flywheel.calculate_angular_momentum(rpm)
        inertia_derivative = self.cube.effective_inertia_derivative(gamma)
        coupling_torque = (
            angular_momentum * theta_dot * np.cos(gamma)
            - 0.5 * inertia_derivative * theta_dot**2
        )
        desired_acceleration = (
            self.natural_frequency**2 * (self.desired_angle - gamma)
            - 2 * self.natural_frequency * gamma_dot
        )
        requested_torque = (
            coupling_torque + self.gimbal_inertia * desired_acceleration
        )
        return float(np.clip(requested_torque, -self.max_torque, self.max_torque))

    def overspeed(self, t, x):
        return self.speed_limit - abs(x[3])

    def flip_limit(self, t, x):
        return np.pi / 2 - abs(x[0])


GimbalControl.overspeed.terminal = True
GimbalControl.overspeed.direction = -1

GimbalControl.flip_limit.terminal = True
GimbalControl.flip_limit.direction = -1
