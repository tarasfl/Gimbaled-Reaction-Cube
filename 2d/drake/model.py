from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
import sys

import numpy as np

# Allow `python 2d/drake/run.py` with the repository's existing flat imports.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from hardware.parameters import make_cube
from pydrake.geometry import Box, MeshcatVisualizer, MeshcatVisualizerParams, Sphere
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import (
    FixedOffsetFrame, RevoluteJoint, RotationalInertia, SpatialInertia,
)
from pydrake.systems.framework import DiagramBuilder

# New gamma=0 puts the disc in XZ. In the original planar equations that
# orientation is gamma=-pi/2: gamma_analytical = gamma_Drake - pi/2.
GIMBAL_OFFSET = np.pi / 2


@dataclass
class Model:
    builder: object
    plant: object
    cube: object
    joints: tuple
    visualizer: object = None


def inertia(mass, diagonal):
    return SpatialInertia.MakeFromCentralInertia(
        mass, np.zeros(3), RotationalInertia(*diagonal)
    )


def analytical_state(state):
    """Convert Drake state to the original equations' order and gimbal zero."""
    result = np.asarray(state)[[0, 3, 1, 4]].copy()
    result[2] -= GIMBAL_OFFSET
    return result


def set_state(model, context, x, rpm, phi=0.0):
    """Set Drake from x in the original analytical state convention."""
    theta, theta_d, gamma, gamma_d = x
    model.plant.SetPositionsAndVelocities(
        context, [theta, gamma + GIMBAL_OFFSET, phi, theta_d, gamma_d, rpm * 2*np.pi/60]
    )


def dynamics_terms(plant, context):
    """M vdot = rhs + actuator torque, with no damping/contact."""
    mass = plant.CalcMassMatrix(context)
    rhs = (plant.CalcGravityGeneralizedForces(context)
           - plant.CalcBiasTerm(context))
    return mass, rhs


def held_speed_torque(plant, context, gimbal_torque, cube_fixed=False):
    """Ideal speed source: solve Drake dynamics imposing phi_dd = 0.

    Return the actual internal spin-joint torque needed to hold relative RPM.
    This is deliberately unlimited; it is a validation mode, not a motor model.
    """
    mass, rhs = dynamics_terms(plant, context)
    if cube_fixed:
        # Floor support imposes theta_dd=0 while the cube is resting flat.
        gamma_dd = (rhs[1] + gimbal_torque) / mass[1, 1]
        return float(mass[2, 1]*gamma_dd - rhs[2])
    acceleration = np.linalg.solve(mass[:2, :2], rhs[:2] + [0, gimbal_torque])
    return float(mass[2, :2] @ acceleration - rhs[2])


def add_ring(plant, body, radius, width, thickness, color, rotation=None):
    """Segmented annulus for display only; inertia comes from hardware values."""
    rotation = RotationMatrix() if rotation is None else rotation
    for k in range(48):
        angle = 2*np.pi*k/48
        local_rotation = RotationMatrix.MakeZRotation(angle)
        center = rotation @ np.array([radius*np.cos(angle), radius*np.sin(angle), 0])
        plant.RegisterVisualGeometry(
            body, RigidTransform(rotation @ local_rotation, center),
            Box(width, 2*radius*np.sin(np.pi/48), thickness), f"ring_{k}", color,
        )


def cage_and_flywheel(cube=None):
    """Keep the existing analytical API, but remove all auxiliary inertia.

    Motor settings still supply ideal joint torque/speed limits. Their mass
    and rotor inertia, and the gimbal ring's mass, do not enter this model.
    Copy caller data so the original hardware/optimisation setup is unchanged.
    """
    cube = make_cube() if cube is None else deepcopy(cube)
    for component in (cube.servo_motor, cube.brushless_motor, cube.gimbal):
        component.mass = 0.0
        component.inertia = component.calculate_inertia()
    for component in (cube.brushless_motor, cube.gimbal):
        component.transverse_inertia = component.calculate_transverse_inertia()
    cube.servo_motor.rotor_inertia = 0.0
    cube.brushless_motor.rotor_inertia = 0.0
    cube.brushless_motor.offset = 0.0
    cube.update_total_mass()
    return cube


def build_model(cube=None, meshcat=None, gravity=9.81):
    cube = cage_and_flywheel(cube)
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    plant.mutable_gravity_field().set_gravity_vector([0, 0, -gravity])
    size = cube.size
    cage_I = cube.calculate_cage_inertia()
    cage = plant.AddRigidBody("cube", inertia(cube.cage_mass, [cage_I]*3))

    # Body origin at cube centre; the selected floor edge is (+a/2,0,-a/2).
    edge = plant.AddFrame(FixedOffsetFrame(
        "floor_edge", cage.body_frame(), RigidTransform([size/2, 0, -size/2])
    ))
    theta = plant.AddJoint(RevoluteJoint(
        "theta", plant.world_frame(), edge, [0, 1, 0]
    ))

    # Two independent wheel rotations need an intermediate frame/body in the
    # joint chain. This carrier has exactly zero mass/inertia and no geometry.
    gimbal_body = plant.AddRigidBody("tilt_carrier", SpatialInertia.Zero())
    gamma = plant.AddJoint(RevoluteJoint(
        "gamma", cage.body_frame(), gimbal_body.body_frame(), [0, 0, 1]
    ))
    plant.AddJointActuator("gimbal_motor", gamma, cube.servo_motor.torque)

    wheel = cube.flywheel
    # Disc lies in local XZ, so its axial inertia and spin axis are along Y.
    flywheel_body = plant.AddRigidBody("flywheel", inertia(
        wheel.mass, [wheel.transverse_inertia, wheel.inertia, wheel.transverse_inertia]
    ))
    phi = plant.AddJoint(RevoluteJoint(
        "phi", gimbal_body.body_frame(), flywheel_body.body_frame(), [0, 1, 0]
    ))
    plant.AddJointActuator("spin_motor", phi, cube.brushless_motor.nominal_torque)

    plant.RegisterVisualGeometry(cage, RigidTransform(), Box(size, size, size),
                                 "shell", [0.2, 0.55, 0.85, 0.18])
    add_ring(plant, flywheel_body, (wheel.outer_radii+wheel.inner_radii)/2,
             wheel.outer_radii-wheel.inner_radii, wheel.thickness, [0.8, 0.55, 0.12, 1],
             RotationMatrix.MakeXRotation(-np.pi/2))
    # Put the marker on the visible face, rather than buried inside the rim.
    plant.RegisterVisualGeometry(flywheel_body,
                                 RigidTransform([wheel.outer_radii-0.006, -wheel.thickness/2-0.001, 0]),
                                 Sphere(0.003), "spin_marker", [0.9, 0.1, 0.1, 1])
    plant.RegisterVisualGeometry(plant.world_body(), RigidTransform([0, 0, -0.006]),
                                 Box(0.7, 0.5, 0.012), "floor_visual_only", [0.8, 0.8, 0.8, 1])
    plant.Finalize()
    visualizer = None
    if meshcat is not None:
        meshcat.SetCameraPose([0.25, -0.65, 0.30], [-size/2, 0, size/2])
        params = MeshcatVisualizerParams()
        params.publish_period = 1 / 240  # Resolve the brief initial gimbal motion.
        visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat, params)
    return Model(builder, plant, cube, (theta, gamma, phi), visualizer)
