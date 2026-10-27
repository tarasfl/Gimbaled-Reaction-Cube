from types import SimpleNamespace

import numpy as np

from dynamics.dynamics_2d import Dynamics2D
from control.gimbal_control import GimbalControl
from dynamics.physical_constraints import FloorConstraint


class SimulationRunner:
    def __init__(self, cube):
        self.cube = cube
        self.dynamics = Dynamics2D(cube)

    def run(self, x0, desired_gamma_deg, t_end=10.0, t_start=0.0, samples=10001, spin_up_samples=5000, flywheel_rpm=5000.0, **solver_options,):
        
        solver_options = {"method": "RK45", "rtol": 1e-7, "atol": 1e-9, **solver_options}

        t_grid = np.linspace(t_start, t_end, samples)
        work_rpm = np.concatenate([
            np.linspace(0, flywheel_rpm, spin_up_samples, endpoint=False),
            np.full(samples - spin_up_samples, flywheel_rpm),
        ])

        control = GimbalControl(
            self.cube, t_grid, work_rpm,
            initial_angle=x0[2],
            desired_angle_deg=desired_gamma_deg,
            ready_rpm=flywheel_rpm,
        )

        def rpm_at(t):
            index = (np.asarray(t) - t_start) / (t_end - t_start) * (samples - 1)
            return work_rpm[np.clip(index.astype(int), 0, samples - 1)]

        def rhs(t, x):
            rpm = rpm_at(t)
            dx = self.dynamics.derivatives(
                t, x, rpm, gimbal_torque=control.command(t, x, rpm)
            )
            if t < control.control_start:
                # Ideal mechanical lock during spin-up.
                dx[2] = 0.0
                dx[3] = 0.0
            return dx

        # stop_events order: index 0 -> overspeed, index 1 -> flip_limit.
        FLIP = 1
        solution = FloorConstraint(rhs).run(
            (t_start, t_end),
            x0,
            stop_events=[control.overspeed, control.flip_limit],
            max_step=control.step_limit,
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
            flipped=solution.stopped_by == FLIP,
        )
