# Architecture and model reference

This document covers the planar model in `2d/`: the physics it assumes, how one
flip is simulated and scored, how the optimiser searches the design space, and
what each module does.

---

## 1. Physical setup and coordinates

```
        z ▲
          │   ┌───────────┐
          │   │   cube    │      θ: cube roll about the pivot edge (rad)
          │   │  ⊙ gimbal │      γ: gimbal tilt of the flywheel spin axis (rad)
          │   │           │      ω: flywheel spin rate
          └───┴───────────●────▶ x
                       pivot edge (origin)
```

- The cube rotates about a fixed floor edge at the origin. Its centre sits at
  `(-s/2, s/2)` relative to the pivot, where `s` is the cube edge length.
- `θ = 0`: resting on the starting face. `θ = π/2`: landed on the next face.
- The flywheel sits on a gimbal at the cube centre. The servo mass sits at
  `servo_position = (0, s/2)` relative to the centre (on the top face), which
  shifts the centre of mass. The balance angle is therefore about 40.6° rather
  than 45° for the baseline.
- State vector (`dynamics/state.py`): `x = [θ, θ̇, γ, γ̇]`.

## 2. Equations of motion (`dynamics/dynamics_2d.py`)

Inertia terms (`hardware/cube.py`, `hardware/gimbal.py`):

| Symbol | Meaning | Code |
|---|---|---|
| `A(γ)` | cube inertia about the pivot, including the tilted rotor | `Cube.effective_inertia(γ)` |
| `A'(γ)` | `dA/dγ = (J_s − J_t)·sin 2γ` | `Cube.effective_inertia_derivative` |
| `B` | inertia about the gimbal axis (constant) | `Gimbal.effective_inertia(...)` |
| `H` | flywheel angular momentum `I_fw · ω` | `Flywheel.calculate_angular_momentum(rpm)` |
| `τ_g(θ)` | gravity torque about the pivot | `Cube.gravitational_torque(θ)` |

with

```
A(γ) = I_fixed + J_t + (J_s − J_t)·sin²γ
I_fixed = I_cage + m_servo·d_servo² + I_servo + (s²/2)·(m_fw + m_bldc + m_gimbal + m_cage)
J_s, J_t = Σ axial / transverse inertias of {flywheel, gimbal ring, BLDC motor}
```

The equations come from a Lagrangian with kinetic energy `½Aθ̇² + ½Bγ̇²` plus the
gyroscopic coupling `H`:

```
A·θ̈ = τ_g(θ) + H·γ̇·cos γ − A'·γ̇·θ̇
B·γ̈ = τ_servo − H·θ̇·cos γ + ½·A'·θ̇²
```

`H·γ̇·cos γ` is the CMG torque that rolls the cube. `−H·θ̇·cos γ` is the reaction
the servo has to overcome.

Modelling assumptions worth knowing:

- Flywheel speed is **prescribed** (`FlywheelControl.rpm_at`). It is not a state,
  so the spin motor's reaction torque and the wheel's speed change during the
  flip are ignored.
- `H` uses the flywheel inertia only. The BLDC rotor inertia appears in
  `Cube.spin_inertia()` but not in `H`.
- Spin-up assumes constant nominal torque with no back-EMF roll-off. With
  `friction_coef=0` (the value the runner uses) the wheel reaches target speed
  after `t = I·ω / τ_nom`.
- Point-mass and simple-shape inertias: thin-walled cube cage `(5/18)·m·s²`, a
  thin ring for the gimbal, a solid cylinder for the motors.

## 3. Floor contact (`dynamics/physical_constraints.py`)

`FloorConstraint` wraps the right-hand side and treats `θ = 0` and `θ = π/2` as
one-sided walls:

1. **Supported phase**: at a wall, `θ̇` and `θ̈` are forced to 0 while γ keeps
   evolving. A `lift_off` event fires when the free dynamics would accelerate
   the cube away from the wall.
2. **Free phase**: the full dynamics are integrated with `solve_ivp`. A
   `contact` event fires on reaching either wall.
3. On the first contact during free flight, the landing time and velocity are
   recorded and **integration stops**. This is a perfectly inelastic impact with
   no bounce or settling.

Extra `stop_events` (the gimbal over-speed event, for example) can end the run.
`result.stopped_by` reports which one fired.

## 4. Simulating one flip (`simulation/simulation_runner.py`)

```
FlywheelControl ──rpm(t)──┐
                          ▼
x0 ─▶ FloorConstraint(rhs) ─▶ trajectory ─▶ evaluate_run ─▶ metrics | reject reason
              ▲
      GimbalControl.command(t, x, rpm) → τ_servo (saturated)
```

1. `FlywheelControl` computes the analytic spin-up curve and `ready_time`.
2. Integration starts at `ready_time`. The gimbal is treated as locked during
   spin-up, so that phase is skipped rather than integrated.
3. `GimbalControl` is a critically damped PD on γ. Its natural frequency is set
   from servo torque and speed limits. It also uses feed-forward of the coupling
   torque `H·θ̇·cos γ − ½A'θ̇²` and saturates at the servo's rated torque.
4. An over-speed event stops the run when `|γ̇|` reaches the servo's maximum
   speed.
5. Solver: RK45, `rtol=1e-7`, `atol=1e-9`, `max_step=5 ms`.

Any object with `command(t, x, rpm)` can be passed as `controller=`. It can also
expose `control_start` and `step_limit`.

### Run evaluation (`simulation/evaluate_simulations.py`)

`evaluate_run` rejects a run, returning a reason string, when:

| Reason | Condition |
|---|---|
| `stopped_by_overspeed` | servo speed limit reached |
| `no_upper_face_contact` | never landed on the next face after control start |
| `contact_at_speed_limit` | landed, but `|γ̇|` touched the limit along the way |
| `no_hits` / `flip_index_invalid` | numerical edge cases |

A successful run returns `flip_time_s` (measured from control start),
`gimbal_travel_deg`, `gimbal_angle_at_flip_deg`, `positive_work_j` (∫max(τ·γ̇,0)),
`peak_torque_nm`, `peak_gimbal_speed_deg_s`, and
`final_angular_velocity_deg_s` (impact speed).

`sweep(cube, rpms, initial_degs, target_degs)` runs the full Cartesian product
and keeps the successful runs together with their trajectories.

## 5. Optimisation (`optimisation/`)

`optimise()` in `optimisation.py` is generic over a component:

1. Build each candidate with `constructor(**candidate)`. Candidates that raise
   `ValueError` are skipped. The component is placed on a deep copy of the base
   cube, and a full `sweep` is run.
2. Score every successful run with `select_best_weighted`. Each metric is
   min-max normalised across all runs, then a weighted mean is taken, and the
   lowest score wins.
3. **Refine**: for each numeric parameter, take the winner's value and the
   midpoints to its nearest tested neighbours, and build the Cartesian product
   of those. Repeat for up to `refinement_steps` rounds.
4. Return all successful runs. Callers save them to CSV and pick the best.

Stages:

| Stage | File | Candidates | Default weights |
|---|---|---|---|
| Flywheel | `flywheel_optimisation.py` | `(outer_radii, inner_radii, thickness)` grid, `ri < ro` | `component_mass`, `flip_time_s`, `target_rpm` = 1 each |
| Servo | `optimise_servo.py` | 7×7 log grid of torque 0.1–10 N·m × speed 100–500 RPM, filtered by `τ·ω ≤ power_limit_w` | `torque` 1, `max_rpm` 0.75, `flip_time_s` 0.75 |

`main.py` chains them. The best flywheel geometry and RPM are fixed, then the
servo is optimised with that flywheel.

Note: the score is relative. Min-max normalisation means adding or removing
runs can change which run wins.

## 6. CSV output (`visualization/csv_io.py`)

`save_csv(results, specs, metadata)` writes `static/<specs>_<timestamp>.csv`.
Each row contains:

- **metadata** (constant per file): `cube_size`, `cube_cage_mass`, BLDC and servo
  mass, torque and speed, gimbal mass and radii;
- **run inputs**: `initial_gamma_deg`, `target_gamma_deg`, `target_rpm`,
  `flywheel_rpm`;
- **metrics** from `evaluate_run`;
- `candidate` (a Python dict literal; parse it with `ast.literal_eval`),
  `component_mass`, `cube_total_mass`, plus any extra weighted metrics taken from
  the component (`torque`, `max_rpm` for the servo stage).

Older files (`sweep_results_*.csv`, `flywheel_weighted_*`) use an earlier schema
with `outer_radius` / `inner_radius` / `thickness` columns.

## 7. Drake model (`2d/drake/`)

| File | Role |
|---|---|
| `scene_case.py` | Loads row `SELECTED_ROW` from `CASE_CSV` and applies it to a fresh `make_cube()` |
| `scene_system.py` | Builds the `MultibodyPlant`. Bodies: cage, servo mass (welded), gimbal carrier (ring + BLDC), flywheel. Joints: `cube_rotation` (revolute at the pivot edge, or x/z prismatic + revolute when `pivoted=False`), `gimbal_rotation` (z), `flywheel_spin` (y). Hydroelastic floor contact with SAP, 1 ms step. `GimbalController` wraps `GimbalControl` as a `LeafSystem`. |
| `scene.py` | Runs the simulation, samples telemetry every 1 ms, records Meshcat, serves the replay locally |
| `telemetry.py` | Injects a telemetry panel and time slider into Meshcat's static HTML, and serves it from memory on `127.0.0.1` |
| `scene_results.py` | Adapts the Drake trajectory to `plot_result`, and saves a summary row in the optimisation CSV format |

Differences from the analytical model:

- The scene gimbal angle is `γ + 90°` (the flywheel disc starts in the XZ plane).
  Telemetry and plots subtract 90° again.
- The flywheel speed is a free state that coasts (the spin actuator command is
  0), so speed loss during the flip is captured here.
- The servo torque limit comes from the actuator effort limit. The speed limit
  is **monitored, not enforced**.

## 8. Module reference

### `hardware/`
- `Flywheel(outer_radii, inner_radii, thickness, density=2700)`: annulus mass and
  axial/transverse inertia. Validates geometry. `recalculate_thickness(target_inertia)`
  solves for the thickness that gives a target inertia. `max_rpm = 12800`.
- `BrushlessMotor(rotor_inertia, nominal_rpm, nominal_torque, mass, radius, length, offset)`:
  solid-cylinder inertias. `offset` is the distance from the gimbal axis.
- `ServoMotor(mass, radius, length, rotor_inertia, nominal_torque, max_rpm)`: the
  rated torque is stored as `.torque`.
- `Gimbal(mass, outter_radii, inner_radii)`: thin ring.
  `effective_inertia(...)` returns `B`.
- `Cube(size, flywheel, brushless_motor, servo_motor, gimbal, cage_mass)`: mass
  totals, inertia about the pivot, centre of mass, gravity torque, potential
  energy, balance angle. Call `update_total_mass()` after swapping a component.
- `parameters.make_cube()`: the baseline hardware shared by `main.py` and Drake.

### `control/`
- `FlywheelControl(cube, friction_coef, target_rpm, t_start)`: `rpm_at(t)` and
  `ready_time`.
- `GimbalControl(cube, t_eval, work_rpm, initial_angle, desired_angle_deg, ready_rpm, control_start)`:
  `command(t, x, rpm)`.
- `lqr_balance.py`: empty placeholder for the planned balancing controller.

### `visualization/`
- `plotter.plot_result(result)`: angles, speeds, torque, and a hardware summary.
- `plotter.plot_mass_vs_flip_time_from_csv(path)`: scatter plot per RPM.
- `pareto.plot_pareto_from_csv(path)`: mass vs flip-time Pareto front.
- `statistic_plotter.plot_csv_dashboard(path, selected_row)`: multi-panel
  dashboard for the flywheel stage.
- `servo_statistic_polotter.plot_servo_dashboard(path, selected_row)`: dashboard
  for the servo stage.
