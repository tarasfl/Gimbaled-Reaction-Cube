from types import SimpleNamespace

import numpy as np
from control.flywheel_control import FlywheelControl

from dynamics.dynamics_2d import Dynamics2D
from control.gimbal_control import GimbalControl
from dynamics.physical_constraints import FloorConstraint


class SimulationRunner:
    def __init__(self, cube):
        self.cube = cube
        self.dynamics = Dynamics2D(cube)

    def run(self, x0, desired_gamma_deg, target_rpm:float,t_end=10.0, t_start=0.0, samples=10001, controller=None, **solver_options):
        """Simulate a flip; `controller` is any object with command(t, state, flywheel_rpm) -> gimbal torque.

        Defaults to GimbalControl. A controller may expose `control_start` (the lock is released then)
        and `step_limit` (max solver step).
        """
        
        solver_options = {"method": "RK45", "rtol": 1e-7, "atol": 1e-9, **solver_options}


        
        flywheel_control = FlywheelControl(self.cube,t_start=t_start, friction_coef=0, target_rpm=target_rpm)
        t_grid = np.linspace(t_start, t_end, samples)

        work_rpm = flywheel_control.rpm_at(t_grid)

        t_ready = flywheel_control.ready_time

        control = controller
        if control is None:
            control = GimbalControl(
                self.cube, t_grid, work_rpm,
                initial_angle=x0[2],
                desired_angle_deg=desired_gamma_deg,
                ready_rpm=5000,
                control_start=t_ready
            )

        control_start = getattr(control, "control_start", t_ready)
        step_limit = getattr(control, "step_limit", 0.005)
        speed_limit = self.cube.servo_motor.max_rpm * 2 * np.pi / 60

        # The gimbal speed limit is physical, so it applies to every controller.
        def overspeed(t, x):
            return speed_limit - abs(x[3])

        overspeed.terminal = True
        overspeed.direction = -1

        rpm_at = flywheel_control.rpm_at

        def rhs(t, x):
            rpm = rpm_at(t)
            dx = self.dynamics.derivatives(
                t, x, rpm, gimbal_torque=control.command(t, x, rpm)
            )
            if t < control_start:
                # Ideal mechanical lock during spin-up.
                dx[2] = 0.0
                dx[3] = 0.0
            return dx

        solution = FloorConstraint(rhs).run(
            (t_ready, t_end),
            x0,
            stop_events=[overspeed],
            max_step=step_limit,
            **solver_options,
        )

        rpm = rpm_at(solution.t)

        torque = np.array([
            control.command(t, solution.y[:, i], rpm[i])
            for i, t in enumerate(solution.t)
        ])
        
        return SimpleNamespace(
            t=solution.t,
            y=solution.y,
            rpm=rpm,
            gimbal_torque=torque,
            stopped_by=solution.stopped_by,
            controller=control,
            landing_time_s=solution.landing_time_s,
            landing_velocity_rad_s=solution.landing_velocity_rad_s,

        )