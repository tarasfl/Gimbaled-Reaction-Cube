import numpy as np


class FlywheelControl:

    def __init__(self, cube, friction_coef:float, target_rpm:float, t_start:float = 0.0):

        motor, wheel = cube.brushless_motor, cube.flywheel


        self.inertia = wheel.inertia + motor.rotor_inertia
        self.t_start = float(t_start)
        self.max_rpm = min(motor.nominal_rpm, wheel.max_rpm)    
        self.target_rpm = target_rpm if target_rpm < self.max_rpm else self.max_rpm

        self.friction_coef = friction_coef

        self.max_torque = motor.nominal_torque


        b = self.friction_coef
        max_omega = self.max_torque / b if b > 0 else np.inf
        target_omega = min(self.target_rpm * 2 * np.pi / 60, 0.99 * max_omega)
        self.target_rpm = target_omega * 60 / (2 * np.pi)

        self.acceleration = self.max_torque / self.inertia # ideal acceleration in rad/s²
        if b == 0:
            self.spin_up_time = target_omega / self.acceleration
        else:
            self.spin_up_time = -self.inertia / b * np.log1p(-b * target_omega / self.max_torque)
        self.ready_time = self.t_start + self.spin_up_time

    def rpm_at(self, t):
        t = np.asarray(t, dtype=float)
        elapsed = np.maximum(t - self.t_start, 0.0)
        b = self.friction_coef
        if b == 0:
            omega = self.acceleration * elapsed
        else:
            omega = self.max_torque / b * (-np.expm1(-b * elapsed / self.inertia))
        rpm = np.minimum(omega * 60 / (2 * np.pi), self.target_rpm)
        return np.where(t >= self.ready_time, self.target_rpm, rpm)
