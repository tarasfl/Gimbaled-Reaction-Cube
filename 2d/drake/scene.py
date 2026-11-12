"""Control the gimbal on a planar cube with floor contact and view an in-memory replay."""
from pathlib import Path
import sys

import numpy as np
from pydrake.all import Simulator, StartMeshcat

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scene_case import load_scene_case, make_scene_cube
from scene_system import build_scene_system
from telemetry import scene_replay_html, serve_scene_replay


def simulate_scene(
    meshcat, cube, duration, rpm, initial_gamma_deg, target_gamma_deg,
    gimbal_rate_deg_s, pivoted=True,
):
    diagram, plant, controller = build_scene_system(
        meshcat, cube, rpm, initial_gamma_deg, target_gamma_deg, pivoted=pivoted,
    )
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()
    plant_context = plant.GetMyMutableContextFromRoot(context)
    if not pivoted:
        plant.GetJointByName("cube_x").set_translation(plant_context, -cube.size / 2)
        plant.GetJointByName("cube_z").set_translation(plant_context, cube.size / 2)
    cube_joint = plant.GetJointByName("cube_rotation")
    cube_joint.set_angle(plant_context, 0.0)
    spin_joint = plant.GetJointByName("flywheel_spin")
    gimbal_joint = plant.GetJointByName("gimbal_rotation")
    spin_joint.set_angular_rate(plant_context, rpm * 2 * np.pi / 60)
    gimbal_joint.set_angle(plant_context, np.deg2rad(initial_gamma_deg + 90))
    gimbal_joint.set_angular_rate(plant_context, np.deg2rad(gimbal_rate_deg_s))
    simulator.Initialize()

    meshcat.StartRecording(frames_per_second=60)
    times = np.linspace(0, duration, int(np.ceil(duration * 1000)) + 1)
    angles, rates, gimbal_angles, gimbal_rates, torques = [], [], [], [], []
    cube_angles, contact_forces = [], []
    for time in times:
        simulator.AdvanceTo(float(time))
        diagram.ForcedPublish(context)
        angles.append(spin_joint.get_angle(plant_context))
        rates.append(spin_joint.get_angular_rate(plant_context))
        gimbal_angles.append(gimbal_joint.get_angle(plant_context))
        gimbal_rates.append(gimbal_joint.get_angular_rate(plant_context))
        torques.append(plant.get_net_actuation_output_port().Eval(plant_context)[0])
        cube_angles.append(cube_joint.get_angle(plant_context))
        contacts = plant.get_contact_results_output_port().Eval(plant_context)
        contact_forces.append(sum(
            abs(contacts.hydroelastic_contact_info(i).F_Ac_W().translational()[2])
            for i in range(contacts.num_hydroelastic_contacts())
        ))
    meshcat.StopRecording()
    meshcat.PublishRecording()
    return dict(
        times=times, angles=np.asarray(angles), rates=np.asarray(rates),
        gimbal_angles=np.asarray(gimbal_angles), gimbal_rates=np.asarray(gimbal_rates),
        torques=np.asarray(torques), target=controller.desired_angle,
        speed_limit=controller.speed_limit,
        cube_angles=np.asarray(cube_angles), contact_forces=np.asarray(contact_forces),
    )


def main():
    duration = 5.0
    case = load_scene_case()
    cube = make_scene_cube(case)
    rpm = float(case["flywheel_rpm"])
    initial_gamma_deg = float(case["initial_gamma_deg"])
    target_gamma_deg = float(case["target_gamma_deg"])
    gimbal_rate_deg_s = 0  # Initial angular speed, not a speed command.
    cube_frame_pivoted = True
    meshcat = StartMeshcat()

    result = simulate_scene(
        meshcat, cube, duration, rpm, initial_gamma_deg, target_gamma_deg,
        gimbal_rate_deg_s, pivoted=cube_frame_pivoted,
    )
    
    replay = scene_replay_html(meshcat.StaticHtml(), **result)
    server, replay_url = serve_scene_replay(replay)
    print("Open this replay with telemetry and time slider:", replay_url)
    print("Meshcat scene only:", meshcat.web_url())
    try:
        input("Press Enter to close the viewers. ")
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        server.shutdown()
        server.server_close()


if __name__ == "__main__":
    main()
