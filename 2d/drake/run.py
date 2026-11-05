import argparse
from pathlib import Path

import numpy as np
from scipy.integrate import solve_ivp

from model import GIMBAL_OFFSET, analytical_state, build_model, dynamics_terms, held_speed_torque, set_state
from control.gimbal_control import GimbalControl
from dynamics.dynamics_2d import Dynamics2D
from pydrake.geometry import StartMeshcat
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import BasicVector, LeafSystem


class MotorControl(LeafSystem):
    def __init__(self, model, control, spin_mode):
        super().__init__()
        self.plant = model.plant
        self.plant_context = self.plant.CreateDefaultContext()
        self.control = control
        self.spin_mode = spin_mode
        # 1 while the flat cube is supported by the floor, otherwise 0.
        self.DeclareNumericParameter(BasicVector([0.0]))
        self.DeclareVectorInputPort("state", 6)
        self.DeclareVectorOutputPort("torques", 2, self.output)

    def output(self, context, output):
        state = self.get_input_port().Eval(context)
        rpm = state[5] * 60/(2*np.pi)
        gimbal_torque = self.control.command(context.get_time(), analytical_state(state), rpm)
        self.plant.SetPositionsAndVelocities(self.plant_context, state)
        cube_fixed = bool(context.get_numeric_parameter(0).GetAtIndex(0))
        spin_torque = (held_speed_torque(self.plant, self.plant_context, gimbal_torque, cube_fixed)
                       if self.spin_mode == "hold" else 0.0)
        output.SetFromVector([gimbal_torque, spin_torque])


def settle_on_floor(model, context, spin_mode):
    """Sampled, inelastic landing on the original face; no bounce/contact solver."""
    plant = model.plant
    state = plant.GetPositionsAndVelocities(context).copy()
    if spin_mode == "coast":
        # A floor impulse stops theta_d while preserving the unactuated
        # gimbal/spin momenta. A free wheel's speed can jump at impact.
        mass = plant.CalcMassMatrix(context)
        response = np.linalg.solve(mass, [1, 0, 0])
        state[3:] -= response * state[3] / response[0]
    # In hold mode the ideal speed source also supplies the impulse required
    # to keep relative RPM constant (matching the analytical floor reset).
    state[0] = state[3] = 0.0
    plant.SetPositionsAndVelocities(context, state)


def simulate(duration=2.0, rpm=5000.0, spin_mode="hold", meshcat=None,
             max_step=0.001, samples=301):
    model = build_model(meshcat=meshcat)
    cube, plant = model.cube, model.plant
    # Flat cube, disc in XZ: gimbal rotates about +Z; wheel spins about +Y.
    # The existing equations use a gimbal zero 90 degrees from this pose.
    x0 = np.array([0.0, 0.0, -GIMBAL_OFFSET, 0.0])
    control = GimbalControl(cube, np.array([0.0]), np.array([rpm]), x0[2],
                            desired_angle_deg=10-np.rad2deg(GIMBAL_OFFSET),
                            ready_rpm=rpm, control_start=0.0)
    controller = model.builder.AddSystem(MotorControl(model, control, spin_mode))
    model.builder.Connect(plant.get_state_output_port(), controller.get_input_port())
    model.builder.Connect(controller.get_output_port(), plant.get_actuation_input_port())
    diagram = model.builder.Build()
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()
    plant_context = plant.GetMyMutableContextFromRoot(context)
    set_state(model, plant_context, x0, rpm)
    controller_context = controller.GetMyMutableContextFromRoot(context)
    theta_joint = model.joints[0]
    theta_joint.Lock(plant_context)
    controller_context.get_mutable_numeric_parameter(0).SetAtIndex(0, 1.0)
    supported = True
    lift_time = None
    landing_times = []
    support_events = [(0.0, True)]
    integrator = simulator.get_mutable_integrator()
    integrator.set_maximum_step_size(max_step)
    integrator.set_target_accuracy(1e-10)
    simulator.Initialize()
    if model.visualizer is not None:
        model.visualizer.StartRecording()
        diagram.ForcedPublish(context)

    times, states, torques = [], [], []
    stop_reason = "duration"
    # Check floor release at least every 1 ms, independently of run duration.
    samples = max(samples, int(np.ceil(duration / 0.001)) + 1)
    for time in np.linspace(0, duration, samples):
        simulator.AdvanceTo(time)
        state = plant.GetPositionsAndVelocities(plant_context).copy()
        if not supported and state[0] < 0:
            settle_on_floor(model, plant_context, spin_mode)
            theta_joint.Lock(plant_context)
            controller_context.get_mutable_numeric_parameter(0).SetAtIndex(0, 1.0)
            supported = True
            landing_times.append(float(time))
            support_events.append((float(time), True))
            simulator.Initialize()
            state = plant.GetPositionsAndVelocities(plant_context).copy()
        torque = controller.get_output_port().Eval(controller_context).copy()
        if supported:
            # The floor may push up, but cannot pull the cube down. Release
            # its support when the unconstrained dynamics would tip upward.
            mass, rhs = dynamics_terms(plant, plant_context)
            spin = (held_speed_torque(plant, plant_context, torque[0])
                    if spin_mode == "hold" else 0.0)
            free_acceleration = np.linalg.solve(mass, rhs + [0, torque[0], spin])
            if free_acceleration[0] > 1e-8:
                theta_joint.Unlock(plant_context)
                controller_context.get_mutable_numeric_parameter(0).SetAtIndex(0, 0.0)
                supported = False
                support_events.append((float(time), False))
                if lift_time is None:
                    lift_time = float(time)
                # Restart integration at the change of constraint.
                simulator.Initialize()
        times.append(time)
        states.append(state)
        torques.append(controller.get_output_port().Eval(controller_context).copy())
        # Landing on the original face is supported above. The opposite face
        # still ends this fixed-edge demo; changing pivot edges is future work.
        if state[0] >= np.pi/2:
            stop_reason = "face boundary (sampled; no impact response)"
            break
        if abs(state[4]) >= control.speed_limit:
            stop_reason = "gimbal speed limit (sampled)"
            break
    if model.visualizer is not None:
        diagram.ForcedPublish(context)
        model.visualizer.StopRecording()
        recording = model.visualizer.get_mutable_recording()
        recording.set_autoplay(True)
        recording.set_repetitions(1000000)
        model.visualizer.PublishRecording()

    times, states, torques = np.array(times), np.array(states), np.array(torques)
    dynamics = Dynamics2D(cube)

    def rhs(time, state):
        return dynamics.derivatives(time, state, rpm, control.command(time, state, rpm))

    def supported_rhs(time, state):
        derivative = rhs(time, state)
        derivative[:2] = 0.0
        return derivative

    # Compare the same sampled support/landing schedule in both models.
    analytical = np.empty((len(times), 4))
    reference_state = x0.copy()
    for i, (start, is_supported) in enumerate(support_events):
        end = support_events[i+1][0] if i+1 < len(support_events) else times[-1]
        if is_supported:
            reference_state[:2] = 0.0
        mask = (times >= start) & (times <= end)
        if end == start:
            analytical[mask] = reference_state
            continue
        reference = solve_ivp(supported_rhs if is_supported else rhs, (start, end),
                              reference_state, dense_output=True, rtol=1e-10,
                              atol=1e-12, max_step=max_step)
        if not reference.success:
            raise RuntimeError(reference.message)
        analytical[mask] = reference.sol(times[mask]).T
        reference_state = reference.y[:, -1].copy()
    # Plot and export the actual gimbal angle, with XZ as zero, in both traces.
    analytical[:, 2] += GIMBAL_OFFSET
    return dict(t=times, state=states, analytical=analytical, torques=torques,
                stop_reason=stop_reason, lift_time=lift_time, landing_times=landing_times,
                model=model, diagram=diagram)


def save_results(result, output):
    output.mkdir(parents=True, exist_ok=True)
    state = result["state"]
    data = np.column_stack([result["t"], state, result["analytical"], result["torques"]])
    np.savetxt(output/"comparison.csv", data, delimiter=",", comments="", header=(
        "time,theta,gamma,phi,theta_d,gamma_d,phi_d,"
        "analytical_theta,analytical_theta_d,analytical_gamma,analytical_gamma_d,"
        "gimbal_torque,spin_torque"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(10, 6), sharex=True)
    for ax, index, label in zip(axes.flat, range(4), ["theta", "theta_d", "gamma", "gamma_d"]):
        ax.plot(result["t"], state[:, [0, 3, 1, 4][index]], label="Drake")
        ax.plot(result["t"], result["analytical"][:, index], "--", label="Analytical (fixed RPM)")
        ax.set_ylabel(label + (" [rad]" if index % 2 == 0 else " [rad/s]"))
        ax.set_xlabel("Time [s]")
        ax.grid(alpha=0.3)
    axes[0, 0].legend()
    fig.tight_layout()
    fig.savefig(output/"comparison.png", dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--rpm", type=float, default=5000)
    parser.add_argument("--spin-mode", choices=["hold", "coast"], default="hold")
    parser.add_argument("--no-meshcat", action="store_true")
    parser.add_argument("--output", type=Path, default=Path(__file__).parent/"output")
    args = parser.parse_args()
    if not np.isfinite([args.duration, args.rpm]).all() or args.duration <= 0:
        parser.error("duration must be positive; duration and RPM must be finite")
    meshcat = None if args.no_meshcat else StartMeshcat()
    result = simulate(args.duration, args.rpm, args.spin_mode, meshcat)
    save_results(result, args.output)
    error = np.max(np.abs(result["state"][:, [0, 3, 1, 4]]-result["analytical"]), axis=0)
    print("Max absolute difference [theta, theta_d, gamma, gamma_d]:", error)
    print("Stop:", result["stop_reason"])
    print("Floor release [s, sampled]:", result["lift_time"])
    print("Floor landings [s, sampled]:", result["landing_times"])
    print("Peak ideal spin torque [N m]:", np.max(np.abs(result["torques"][:, 1])))
    print("Output:", args.output.resolve())
    if args.spin_mode == "coast":
        print("Coasting conserves spin momentum; differences from fixed-RPM equations are expected.")
    if meshcat is not None:
        (args.output/"meshcat.html").write_text(meshcat.StaticHtml())
        print("Meshcat:", meshcat.web_url(), "(or reopen saved meshcat.html)")
        try:
            input("Press Enter to close the live Meshcat server. ")
        except EOFError:
            pass


if __name__ == "__main__":
    main()
