"""Build the Drake scene system and adapt the existing gimbal controller."""
from pathlib import Path
import sys

import numpy as np
from pydrake.all import (
    AddContactMaterial,
    AddCompliantHydroelasticProperties,
    AddMultibodyPlantSceneGraph,
    AddRigidHydroelasticProperties,
    BasicVector,
    Box,
    ContactModel,
    CoulombFriction,
    DiagramBuilder,
    DiscreteContactApproximation,
    FixedOffsetFrame,
    HalfSpace,
    LeafSystem,
    Mesh,
    MeshcatVisualizer,
    MeshcatVisualizerParams,
    PrismaticJoint,
    ProximityProperties,
    RevoluteJoint,
    RigidTransform,
    RotationMatrix,
    RotationalInertia,
    SpatialInertia,
    Sphere,
)

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from control.gimbal_control import GimbalControl
from dynamics.state import SystemState


class GimbalController(LeafSystem):
    def __init__(self, controller, plant, cube_joint, gimbal_joint, spin_joint):
        super().__init__()
        self.controller = controller
        self.theta_index = cube_joint.position_start()
        self.theta_rate_index = plant.num_positions() + cube_joint.velocity_start()
        self.gamma_index = gimbal_joint.position_start()
        self.gamma_rate_index = plant.num_positions() + gimbal_joint.velocity_start()
        self.spin_rate_index = plant.num_positions() + spin_joint.velocity_start()
        self.DeclareVectorInputPort("state", plant.num_multibody_states())
        self.DeclareVectorOutputPort("torque", BasicVector(2), self.calculate_torque)

    def calculate_torque(self, context, output):
        values = self.get_input_port().Eval(context)
        state = SystemState(
            theta=values[self.theta_index],
            theta_d=values[self.theta_rate_index],
            gamma=values[self.gamma_index] - np.pi / 2,
            gamma_d=values[self.gamma_rate_index],
        )
        rpm = values[self.spin_rate_index] * 60 / (2 * np.pi)
        torque = self.controller.command(context.get_time(), state.to_vector(), rpm)
        output.SetFromVector([torque, 0.0])


def _make_controller(cube, rpm, initial_gamma_deg, target_gamma_deg):
    return GimbalControl(
        cube=cube,
        t_eval=np.array([0.0]),
        work_rpm=np.array([rpm]),
        initial_angle=np.deg2rad(initial_gamma_deg),
        desired_angle_deg=target_gamma_deg,
        ready_rpm=rpm,
        control_start=0.0,
    )


def _add_bodies_and_joints(plant, cube, pivoted):
    cage_inertia = cube.calculate_cage_inertia()
    plate = plant.AddRigidBody("cube", SpatialInertia.MakeFromCentralInertia(
        cube.cage_mass, np.zeros(3), RotationalInertia(*([cage_inertia] * 3)),
    ))
    wheel = plant.AddRigidBody("flywheel", SpatialInertia.MakeFromCentralInertia(
        cube.flywheel.mass, np.zeros(3), RotationalInertia(
            cube.flywheel.transverse_inertia, cube.flywheel.inertia,
            cube.flywheel.transverse_inertia,
        ),
    ))

    servo = cube.servo_motor
    servo_body = plant.AddRigidBody("servo_mass", SpatialInertia.MakeFromCentralInertia(
        servo.mass, np.zeros(3), RotationalInertia(*([servo.calculate_inertia()] * 3)),
    ))
    x, z = cube.servo_position
    plant.WeldFrames(plate.body_frame(), servo_body.body_frame(), RigidTransform([x, 0, z]))

    ring, motor = cube.gimbal, cube.brushless_motor
    transverse = ring.calculate_transverse_inertia() + motor.calculate_transverse_inertia()
    axial = ring.calculate_inertia() + motor.calculate_inertia()
    tilt = ring.calculate_inertia() + motor.calculate_transverse_inertia() + motor.mass * motor.offset**2
    gimbal = plant.AddRigidBody("gimbal_carrier", SpatialInertia.MakeFromCentralInertia(
        ring.mass + motor.mass, np.zeros(3), RotationalInertia(transverse, axial, tilt),
    ))

    if pivoted:
        pivot_frame = plant.AddFrame(FixedOffsetFrame(
            "floor_edge", plate.body_frame(),
            RigidTransform([cube.size / 2, 0, -cube.size / 2]),
        ))
        cube_joint = plant.AddJoint(RevoluteJoint(
            "cube_rotation", plant.world_frame(), pivot_frame, [0, 1, 0],
        ))
    else:
        horizontal = plant.AddRigidBody("horizontal_carrier", SpatialInertia.Zero())
        vertical = plant.AddRigidBody("vertical_carrier", SpatialInertia.Zero())
        plant.AddJoint(PrismaticJoint(
            "cube_x", plant.world_frame(), horizontal.body_frame(), [1, 0, 0],
        ))
        plant.AddJoint(PrismaticJoint(
            "cube_z", horizontal.body_frame(), vertical.body_frame(), [0, 0, 1],
        ))
        cube_joint = plant.AddJoint(RevoluteJoint(
            "cube_rotation", vertical.body_frame(), plate.body_frame(), [0, 1, 0],
        ))
    gimbal_joint = plant.AddJoint(RevoluteJoint(
        "gimbal_rotation", plate.body_frame(), gimbal.body_frame(), [0, 0, 1],
    ))
    spin_joint = plant.AddJoint(RevoluteJoint(
        "flywheel_spin", gimbal.body_frame(), wheel.body_frame(), [0, 1, 0],
    ))
    return plate, wheel, cube_joint, gimbal_joint, spin_joint


def _add_actuators(plant, cube, gimbal_joint, spin_joint):
    servo = cube.servo_motor
    gimbal_actuator = plant.AddJointActuator("gimbal_motor", gimbal_joint, servo.torque)
    gimbal_actuator.set_default_rotor_inertia(servo.rotor_inertia)
    gimbal_actuator.set_default_gear_ratio(1.0)

    motor = cube.brushless_motor
    spin_actuator = plant.AddJointActuator("spin_motor", spin_joint, motor.nominal_torque)
    spin_actuator.set_default_rotor_inertia(motor.rotor_inertia)
    spin_actuator.set_default_gear_ratio(1.0)


def _add_visual_geometry(plant, plate, wheel):
    folder = Path(__file__).resolve().parent
    mesh_pose = RigidTransform(RotationMatrix.MakeXRotation(-np.pi / 2))
    plant.RegisterVisualGeometry(
        plate, mesh_pose, Mesh(str(folder / "meshes" / "plate.obj")),
        "plate", [0.2, 0.55, 0.85, 1.0],
    )
    plant.RegisterVisualGeometry(
        wheel, mesh_pose, Mesh(str(folder / "meshes" / "flywheel.obj")),
        "wheel", [0.8, 0.55, 0.12, 1.0],
    )
    plant.RegisterVisualGeometry(
        plant.world_body(), RigidTransform([0, 0, -0.005]),
        Box(0.5, 0.4, 0.01), "floor", [0.8, 0.8, 0.8, 1],
    )
    plant.RegisterVisualGeometry(
        wheel, RigidTransform([0.054, -0.0055, 0]),
        Sphere(0.0025), "spin_marker", [1.0, 0.05, 0.05, 1.0],
    )
def _add_contact_geometry(plant, plate, cube):
    friction = CoulombFriction(0.6, 0.5)
    floor_properties = ProximityProperties()
    cube_properties = ProximityProperties()
    for properties in (floor_properties, cube_properties):
        AddContactMaterial(properties, friction=friction)
        properties.AddProperty("material", "relaxation_time", 0.01)
    AddRigidHydroelasticProperties(floor_properties)
    AddCompliantHydroelasticProperties(0.03, 1e7, cube_properties)
    plant.RegisterCollisionGeometry(
        plant.world_body(), RigidTransform(), HalfSpace(), "floor_collision", floor_properties,
    )
    plant.RegisterCollisionGeometry(
        plate, RigidTransform(), Box(cube.size, cube.size, cube.size),
        "cube_collision", cube_properties,
    )


def _connect_controller_and_visualizer(builder, plant, scene_graph, meshcat, cube,
                                       controller, cube_joint, gimbal_joint, spin_joint):
    adapter = builder.AddSystem(
        GimbalController(controller, plant, cube_joint, gimbal_joint, spin_joint),
    )
    builder.Connect(plant.get_state_output_port(), adapter.get_input_port())
    builder.Connect(adapter.get_output_port(), plant.get_actuation_input_port())
    MeshcatVisualizer.AddToBuilder(
        builder, scene_graph, meshcat, MeshcatVisualizerParams(publish_period=1 / 60),
    )
    meshcat.SetCameraPose([0.20, -0.55, 0.25], [-cube.size / 2, 0, cube.size / 2])


def build_scene_system(meshcat, cube, rpm, initial_gamma_deg, target_gamma_deg,
                       pivoted=True):
    controller = _make_controller(cube, rpm, initial_gamma_deg, target_gamma_deg)
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
    plant.set_contact_model(ContactModel.kHydroelastic)
    plant.set_discrete_contact_approximation(DiscreteContactApproximation.kSap)
    plate, wheel, cube_joint, gimbal_joint, spin_joint = _add_bodies_and_joints(
        plant, cube, pivoted,
    )
    _add_actuators(plant, cube, gimbal_joint, spin_joint)
    _add_visual_geometry(plant, plate, wheel)
    _add_contact_geometry(plant, plate, cube)
    plant.Finalize()
    _connect_controller_and_visualizer(
        builder, plant, scene_graph, meshcat, cube,
        controller, cube_joint, gimbal_joint, spin_joint,
    )
    return builder.Build(), plant, controller
