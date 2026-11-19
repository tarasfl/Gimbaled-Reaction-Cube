# Code review: state as of commit `10ee271` ("lqr prep")

Scope: all 29 Python files in `2d/` (≈2 500 lines), `requires.txt`, `.gitignore`,
and `static/`. The analytical simulation was smoke-tested with NumPy 2.5, SciPy
and Matplotlib. The baseline cube flips in 0.19 s at 5000 RPM (0°→90° gimbal),
and in 0.34 s at 3000 RPM (−10°→60°). At 1000 RPM it does not flip, and the
evaluator rejects the run as expected. The Drake scene was **not** run (pydrake
was not installed).

## Fixed after this review

- NumPy pinned to `>=2.0` (needed for `np.trapezoid`); unused `sympy` removed
  from `requires.txt`.
- `*.toml` removed from `.gitignore`; `.DS_Store` ignored and untracked.
- Dead code removed: `BrushlessMotor.calculate_rpm`, `GimbalControl.overspeed` /
  `flip_limit`, `telemetry.write_replay` / `serve_replay`,
  `optimisation/optimisation_f2f.py`, and unused imports.
- `plot_fastest_from_csv` and the mass helpers read the current `candidate`
  column as well as the legacy columns, and the cube mass is refreshed.

Items 5–9 below and the remaining code-quality points are still open.
`lqr_balance.py` is kept as the placeholder for the planned balancing controller.

## Overall evaluation

| Area | Rating | Comment |
|---|---|---|
| Physics model | ★★★★☆ | Lagrangian-consistent equations; gravity torque matches −dV/dθ; sensible inertia models. Simplifications are reasonable but undocumented in code. |
| Numerical robustness | ★★★★☆ | Event-based floor contact with explicit no-progress and failure checks; tight tolerances; physical limits treated as terminal events. |
| Architecture | ★★★☆☆ | Clear layering (hardware → dynamics → control → simulation → optimisation → visualisation). Hurt by `sys.path` hacks, no package structure, duplicated scripts and dead code. |
| Optimiser | ★★★☆☆ | Generic and reusable; grid + local refinement is simple and transparent. Exhaustive evaluation is expensive and the min-max score is relative. |
| Testing | ★☆☆☆☆ | No automated tests. |
| Reproducibility | ★★☆☆☆ | Loose dependency pins; results committed without the code version that produced them; a hard-coded CSV row selects the Drake case. |
| Documentation | ★★☆☆☆ (before this PR) | Good inline comments in places; no README until now. |

**Summary:** the engineering core is good. The dynamics, contact handling and run
evaluation are careful and correct as far as I could check, and the results are
physically plausible. The weak points are the project scaffolding: no tests, no
packaging, dead and duplicated code, and a few latent bugs listed below. That is
typical of a research codebase growing fast. A few hours of cleanup would make it
much easier to extend into the 3-D model and the LQR balancing work.

## What's done well

- **Floor contact** (`dynamics/physical_constraints.py`) is the best piece of
  engineering in the repo. It switches between supported and free phases with
  terminal events, handles the zero-at-restart edge case, avoids duplicate
  samples at segment joins, and raises on stalls instead of looping forever.
- **The equations of motion** are derived consistently. The `−A'γ̇θ̇` and
  `+½A'θ̇²` terms are the correct Coriolis pair for a γ-dependent inertia, which
  is often missed.
- **The controller** feeds forward the exact coupling torque and picks its
  bandwidth from actuator torque and speed limits, not from hand-tuned gains.
- **`evaluate_run`** is strict. It rejects runs that touch the speed limit even
  when they land, and it interpolates the exact control-start time.
- **The optimiser** is generic over `component_attribute` and `constructor`, so
  the flywheel and servo stages share one implementation.
- **Drake reuse**: the same `GimbalControl` and `make_cube()` drive both
  simulators, so the two can be cross-checked directly.
- **Telemetry replay** is served from memory, bound to `127.0.0.1`, and
  synchronised to Meshcat's animation clock rather than wall time.

## Bugs and correctness issues

Ordered by impact.

1. **`np.trapezoid` breaks with the declared minimum NumPy.**
   `simulation/evaluate_simulations.py:53` and `drake/scene_results.py:57` call
   `np.trapezoid`, which only exists from NumPy 2.0. `requires.txt` allows
   `numpy>=1.26`, where every successful run would raise `AttributeError`.
   *Fix:* pin `numpy>=2.0` or use `scipy.integrate.trapezoid`.

2. **`.gitignore` ignores `*.toml`.** This silently blocks adding a
   `pyproject.toml` (or `ruff.toml`, etc.) to the repo. *Fix:* remove that line.

3. **`BrushlessMotor.calculate_rpm` has wrong units.**
   `hardware/brushless_motor.py:33` divides torque by inertia, which gives an
   angular *acceleration* (rad/s²), and then labels it RPM. It is unused today,
   but it is a trap. *Fix:* delete it, or compute the speed from a torque-speed
   curve.

4. **`plot_fastest_from_csv` uses a stale schema.** `visualization/plotter.py:150`
   reads `best.outer_radius` etc., which current CSVs no longer have (they store
   a `candidate` dict). It also replaces `cube.flywheel` without calling
   `update_total_mass()`.

5. **Angular momentum ignores the motor rotor.** `Dynamics2D` and `GimbalControl`
   compute `H` from `flywheel.inertia` only, while `FlywheelControl` and
   `Cube.spin_inertia()` include the BLDC rotor. For the baseline the error is
   small (3.1e-6 vs 5.7e-4 kg·m², about 0.5 %), but the code is inconsistent.
   *Fix:* use `cube.spin_inertia()` everywhere.

6. **Drake: possible conflict between the pivot joint and floor contact.** With
   `pivoted=True` the cube is pinned to the world by a revolute joint, *and* its
   collision box (with a 30 mm compliant hydroelastic layer) touches the floor
   half-space along the same edge. Contact and friction forces then act against
   the joint constraint. *Suggestion:* compare the reported normal force and the
   flip time against the analytical model. If they differ, either drop the joint
   (`pivoted=False`) or exclude the collision geometry at the hinge.

7. **Hidden side effects in hot paths.** `Flywheel.calculate_angular_momentum`
   writes `self.work_rpm`, and `Dynamics2D.derivatives` calls
   `cube.update_total_mass()` on every right-hand-side evaluation. Neither
   changes results today, but they cost time inside large sweeps, and they make
   the model unsafe to share between threads or processes.

8. **The flywheel speed is prescribed, not simulated.** In the analytical model
   the wheel speed during the flip comes from `rpm_at(t)` and ignores the
   reaction torque. At 0.2 s flips this is likely fine, but it is a modelling
   assumption that the Drake model does *not* make. Documenting it, or adding ω
   as a fifth state, would make the two models comparable.

9. **Impact speed is not penalised.** The fastest flips land at ~740 °/s
   (baseline, 5000 RPM). `final_angular_velocity_deg_s` is recorded but not in
   any weight set. A real cube would bounce, or tip over onto the next edge.
   Consider adding it to the weights, or adding a braking phase to the controller.

## Code-quality issues

- **Dead code**
  - `control/lqr_balance.py` is empty.
  - `GimbalControl.overspeed` / `flip_limit` and their event attributes are
    unused (the runner defines its own `overspeed`).
  - The `ready_rpm` / `t_eval` / `work_rpm` constructor path in `GimbalControl`
    is bypassed because `control_start` is always passed in.
  - The "mechanical lock" branch in `SimulationRunner.rhs` only runs for a custom
    controller whose `control_start` is later than the flywheel ready time; with
    the default controller integration starts at `control_start`.
  - `telemetry.write_replay` / `serve_replay` reference result keys that no
    code produces.
  - `scene.py` imports `save_scene_result` but never calls it.
  - Unused imports in `main.py` and `flywheel_optimisation.py`.
- **Duplication**
  - `optimisation/optimisation_f2f.py` is a copy of `main.py`; only the
    imports differ.
  - The cube-metadata `SimpleNamespace` is built in both
    `flywheel_optimisation.py` and `scene_results.py`.
  - `Cube.effective_inertia` and `effective_inertia_derivative` each recompute
    `J_s`/`J_t`.
  - The two dashboard plotters share most of their code.
- **Packaging and imports**
  - Scripts only work when run from `2d/`, and the Drake modules use
    `sys.path.insert`. A `pyproject.toml` with `2d` as an installable package
    (for example `reaction_cube`), or at least `python -m` entry points, would
    fix this. Note that `2d` is not a valid Python package name.
- **Mutable state and cache coherence**
  - Components cache `inertia` / `transverse_inertia` in `__init__`, so mutating
    a field afterwards leaves stale values. `scene_case.make_scene_cube` has to
    recompute them by hand.
  - Prefer frozen dataclasses with computed properties, or rebuild components
    instead of mutating them.
- **Mutable default arguments**: `optimise_servo(initial_degs=[-10, 0], ...)`.
- **Naming**
  - `radii` is used for a single radius.
  - `outter_radii`, `servo_statistic_polotter.py`, "draka tests".
  - Mixed `optimisation`/`visualization` spelling.
- **Magic numbers**
  - `ready_rpm=5000`.
  - `SELECTED_ROW = 18786`, `selected_row=110`.
  - The flywheel `max_rpm = 12800` is hard-coded on the class.
  - Gravity `9.81` is inline.
- **Logging**: `print` throughout the sweep (one line per run, tens of thousands
  of lines in a full run). Use `logging` or a progress bar.

## Repository hygiene

- `.DS_Store` files are committed (root and `2d/`). Add them to `.gitignore` and
  remove them from the index.
- `static/` holds ~8 MB and ~26 000 rows of generated CSVs, and nothing records
  which commit or parameters produced each file. Store the git SHA in the CSV
  metadata, and consider Git LFS or a release artefact for large result sets.
- `requires.txt` is a non-standard name (`requirements.txt` is conventional), and
  it has no upper bounds or lock file.
- Some file names contain spaces (`static/full test statistic.png`), which is
  awkward in shells and URLs.

## Performance

A full `main.py` run means ≈30 000 simulations in the first flywheel round alone,
all serial. Easy wins:

1. Run candidates (or sweep items) in parallel with
   `concurrent.futures.ProcessPoolExecutor`. The work is embarrassingly parallel.
   Remove the hidden mutations first (item 7 above).
2. Drop the per-step `update_total_mass()` and cache `A`'s constants (`I_fixed`,
   `J_s`, `J_t`) once per cube.
3. Don't keep every full trajectory (`result=result`) in memory across the whole
   optimisation. Keep only the metrics and re-simulate the winners for plotting.
4. Consider replacing exhaustive grids with a proper optimiser (Bayesian
   optimisation, CMA-ES, or `scipy.optimize.differential_evolution` over the
   continuous geometry), with the angle/RPM sweep as the inner objective.

## Testing: recommended minimum suite

Add `pytest` and a `tests/` folder with:

1. **Unit tests for physics**:
   - flywheel mass and inertia against hand-computed values;
   - `Cube.gravitational_torque == −d(potential_energy)/dθ` (finite difference);
   - torque is zero at `balance_angle()`;
   - `effective_inertia_derivative` matches a finite difference of
     `effective_inertia`.
2. **Energy conservation**: with `τ_servo = 0`, constant `H`, and no contact, the
   total energy `½Aθ̇² + ½Bγ̇² + V(θ)` should stay constant within the solver
   tolerance.
3. **Contact**:
   - a cube released at rest at `θ = 0` stays put;
   - a cube started with positive `θ̇` and no CMG torque returns and lands on the
     lower wall;
   - the landing on `π/2` is reported once.
4. **Regression**: the baseline flip at 5000 RPM, 0°→90° gives
   `flip_time_s ≈ 0.189 s` (lock this with a tolerance).
5. **Optimiser**:
   - `refined_values`, `refine_candidates` and `select_best_weighted` on toy data
     (ties, zero ranges, single run);
   - `pareto_mask` on a known front.
6. **Drake vs analytical** (optional, marked slow): the same case flips within
   some tolerance in both simulators.

## Suggested next steps (priority order)

1. Fix the NumPy pin, the `.gitignore` `*.toml` line, and `.DS_Store`.
2. Add `pyproject.toml`, make `2d/` an importable package, and remove the
   `sys.path` hacks.
3. Add the test suite above, starting with energy conservation and the
   regression flip.
4. Delete dead and duplicate code (`optimisation_f2f.py`, unused controller
   events, `write_replay`, `calculate_rpm`).
5. Make components immutable (dataclasses with properties). Use
   `spin_inertia()` for `H`.
6. Parallelise the sweep. Record the git SHA in the CSV metadata.
7. Add impact speed to the objective. Then implement `lqr_balance.py`, using
   the existing linearisable `Dynamics2D` around `balance_angle()`.
