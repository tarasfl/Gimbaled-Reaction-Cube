from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np
from scipy.integrate import solve_ivp


@dataclass(frozen=True)
class Wall:
    angle: float
    inward: float  # +1 if the allowed range lies above the wall, -1 if below


class FloorConstraint:
    def __init__(self, rhs, lower=0.0, upper=np.pi/2, lift_threshold=1e-8):
        self.rhs = rhs
        self.walls = (Wall(lower, +1.0), Wall(upper, -1.0))
        self.lift_threshold = lift_threshold

    def _supported_rhs(self, t, x):
        dx = self.rhs(t, x)
        dx[0] = 0.0
        dx[1] = 0.0
        return dx

    def _depth(self, wall, x):
        return wall.inward * (x[0] - wall.angle)

    def _leaves_floor(self, wall, t, x):
        return wall.inward * self.rhs(t, x)[1] > self.lift_threshold

    def _contact_event(self, wall):
        def contact(t, x):
            depth = self._depth(wall, x)
            if depth == 0.0 and wall.inward * x[1] >= 0.0:
                # A state leaving the wall is not a new impact. Avoid an
                # initial zero that solve_ivp can report at the restart time.
                return np.finfo(float).eps
            return depth

        contact.terminal = True
        contact.direction = -1
        return contact

    def _lift_off_event(self, wall):
        def lift_off(t, x):
            return wall.inward * self.rhs(t, x)[1] - self.lift_threshold

        lift_off.terminal = True
        lift_off.direction = 1
        return lift_off

    def _land_on(self, wall, x):
        landed = np.array(x, dtype=float)
        landed[0] = wall.angle
        landed[1] = 0.0
        return landed

    def run(self, t_span, x0, stop_events=(), **solver_options):
        """Integrate and return a result with t, y, and stopped_by.

        stopped_by is the index into stop_events that ended the run, or None.
        """
        t, t_end = t_span
        state = np.array(x0, dtype=float)
        wall = next((w for w in self.walls if self._depth(w, state) <= 0), None)

        times = [np.array([t])]
        states = [state[:, np.newaxis]]
        stopped_by = None

        while t < t_end and stopped_by is None:
            if wall is not None:
                state = self._land_on(wall, state)
                if self._leaves_floor(wall, t, state):
                    wall = None
                    continue
                fun = self._supported_rhs
                own_events = [self._lift_off_event(wall)]
            else:
                fun = self.rhs
                own_events = [self._contact_event(w) for w in self.walls]

            segment = solve_ivp(
                fun,
                (t, t_end),
                state,
                events=[*own_events, *stop_events],
                **solver_options,
            )
            if not segment.success:
                raise RuntimeError(f"Integration failed at t={segment.t[-1]}: {segment.message}")

            fired = next(
                (i for i, te in enumerate(segment.t_events) if te.size), None
            )
            if segment.t[-1] <= t and (fired is None or fired < len(own_events)):
                raise RuntimeError(f"Floor-contact integration made no time progress at t={t}.")

            # The segment's first point replaces the previous last one, so the
            # velocity reset on landing shows up without duplicated times.
            times[-1] = times[-1][:-1]
            states[-1] = states[-1][:, :-1]
            times.append(segment.t)
            states.append(segment.y)

            t, state = segment.t[-1], segment.y[:, -1]
            if fired is None:
                break
            if fired >= len(own_events):
                stopped_by = fired - len(own_events)
            elif wall is None:
                wall = self.walls[fired]
            else:
                wall = None

        return SimpleNamespace(
            t=np.concatenate(times),
            y=np.concatenate(states, axis=1),
            stopped_by=stopped_by,
        )
