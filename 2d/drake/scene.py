"""Step 1: display the plate and flywheel, held still, in Meshcat."""
from pathlib import Path
import numpy as np
from pydrake.all import RevoluteJoint, Simulator

from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    Box,
    DiagramBuilder,
    MeshcatVisualizer,
    Parser,
    RigidTransform,
    StartMeshcat,
    Sphere
)


def build_scene(meshcat=None):
    # 1. Create the physics model and its geometry system.
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser = Parser(plant)
    folder = Path(__file__).resolve().parent

    # 2. Load the two SDF files, including their mass and inertia.
    plate_model = parser.AddModels(str(folder / "plate.sdf"))[0]
    wheel_model = parser.AddModels(str(folder / "flywheel.sdf"))[0]
    plate = plant.GetBodyByName("cube", plate_model)
    wheel = plant.GetBodyByName("flywheel", wheel_model)

    # 3. Hold the plate upright and place the wheel at its centre.
    # This puts the original cube's floor-edge frame at the world origin.
    plant.WeldFrames(plant.world_frame(), plate.body_frame(),
                     RigidTransform([-0.075, 0, 0.075]))
    # Allow the flywheel to spin about local Y.
    # Its centre stays attached to the plate's centre.
    plant.AddJoint(RevoluteJoint("flywheel_spin", plate.body_frame(), wheel.body_frame(), [0, 1, 0],))

    # Floor appearance only; contact dynamics come in a later step.
    plant.RegisterVisualGeometry(
        plant.world_body(), RigidTransform([0, 0, -0.005]),
        Box(0.5, 0.4, 0.01), "floor", [0.8, 0.8, 0.8, 1],
    )
    plant.RegisterVisualGeometry(
        wheel,
        RigidTransform([0.054, -0.0055, 0]),
        Sphere(0.0025),
        "spin_marker",
        [1.0, 0.05, 0.05, 1.0],  # Red: R, G, B, opacity.
)
    plant.Finalize()

    # 4. Connect the geometry to the viewer.
    if meshcat is not None:
        MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
        meshcat.SetCameraPose([0.20, -0.55, 0.25], [-0.075, 0, 0.075])
    return builder.Build(), plant


def main():
    meshcat = StartMeshcat()
    diagram, plant = build_scene(meshcat)

    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()
    plant_context = plant.GetMyMutableContextFromRoot(context)

    # Start the wheel at 60 RPM: one revolution per second.
    rpm = 60.0
    spin_joint = plant.GetJointByName("flywheel_spin")
    spin_joint.set_angular_rate(
        plant_context,
        rpm * 2 * np.pi / 60,
    )

    simulator.set_target_realtime_rate(1.0)
    simulator.Initialize()
    diagram.ForcedPublish(context)

    print("Open this viewer:", meshcat.web_url())
    input("Press Enter to start the 5-second simulation. ")

    simulator.AdvanceTo(5.0)
    diagram.ForcedPublish(context)

    input("Simulation finished. Press Enter to close the viewer. ")

if __name__ == "__main__":
    main()
