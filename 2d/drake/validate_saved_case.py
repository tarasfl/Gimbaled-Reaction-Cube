"""Check equivalent multibody dynamics and independently reproduce the saved flip."""
import numpy as np

from model import build_model, dynamics_terms, held_speed_torque, set_state
from saved_case import load_saved_case
from run import simulate
from dynamics.dynamics_2d import Dynamics2D
from simulation.simulation_runner import SimulationRunner


def main():
    cube, saved = load_saved_case()
    model = build_model(cube=cube, include_auxiliary_inertia=True)
    context = model.plant.CreateDefaultContext()
    # Different angles, rates and torques exercise gravity and gyroscopic terms.
    for theta, gamma, theta_d, gamma_d, torque in [
        (0, -30, 0, 0, .531), (15, 0, 2, 5, .1),
        (45, 60, 3, 10, -.4), (80, 99, 6, .2, .2),
    ]:
        x = np.array([np.deg2rad(theta), theta_d, np.deg2rad(gamma), gamma_d])
        set_state(model, context, x, saved["flywheel_rpm"])
        mass, rhs = dynamics_terms(model.plant, context)
        spin_torque = held_speed_torque(model.plant, context, torque)
        acceleration = np.linalg.solve(mass, rhs + [0, torque, spin_torque])
        reference = Dynamics2D(cube).derivatives(0, x, saved["flywheel_rpm"], torque)
        np.testing.assert_allclose(acceleration[:2], reference[[1, 3]], atol=1e-10, rtol=0)
    # This solver locates its own floor-release and landing events independently.
    original = SimulationRunner(cube).run(
        [0, 0, np.deg2rad(saved["initial_gamma_deg"]), 0],
        saved["target_gamma_deg"], saved["flywheel_rpm"], t_end=4,
        rtol=1e-10, atol=1e-12,
    )
    flip_time = original.landing_time_s - original.controller.control_start
    assert abs(flip_time-saved["flip_time_s"]) < 1e-5
    result = simulate(duration=1, rpm=saved["flywheel_rpm"], cube=cube,
                      include_auxiliary_inertia=True, gimbal_move_time=None,
                      gimbal_initial_deg=saved["initial_gamma_deg"]+90,
                      gimbal_target_deg=saved["target_gamma_deg"]+90)
    assert result["stop_reason"].startswith("face boundary")
    assert abs(result["t"][-1]-flip_time) < .001
    np.testing.assert_allclose(result["state"][:, [0, 3, 1, 4]],
                               result["analytical"], atol=1e-6, rtol=0)
    np.testing.assert_allclose(result["state"][:, 5], saved["flywheel_rpm"]*2*np.pi/60,
                               atol=1e-8, rtol=0)
    assert np.max(np.abs(result["torques"][:, 0])) <= cube.servo_motor.torque + 1e-12
    print(f"Passed: original flip {flip_time:.9f} s; Drake sampled flip {result['t'][-1]:.3f} s.")


if __name__ == "__main__":
    main()
