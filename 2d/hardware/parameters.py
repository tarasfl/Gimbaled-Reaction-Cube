from hardware.brushless_motor import BrushlessMotor
from hardware.servo_motor import ServoMotor
from hardware.flywheel import Flywheel
from hardware.gimbal import Gimbal
from hardware.cube import Cube


def make_cube():
    """Return fresh hardware using the baseline parameters from main.py."""
    # Maxon ECX FLAT 32 L, 24 V, high-torque winding Y330A
    brushless_motor = BrushlessMotor(
        nominal_rpm=8220,
        nominal_torque=0.0985,
        rotor_inertia=30.7e-7,
        mass=0.071,
        radius=0.016,
        length=0.0187,
    )

    # CubeMars AK40-10 V3.0, 24 V
    servo_motor = ServoMotor(
        mass=0.190,
        radius=0.053 / 2,
        length=0.0402,
        rotor_inertia=97.35e-7 * 10**2,  # Assume motor-side inertia.
        nominal_torque=1.3,
        max_rpm=370,
    )
    flywheel = Flywheel(outer_radii=0.063, inner_radii=0.045, thickness=0.0115)
    gimbal = Gimbal(mass=0.2, outter_radii=0.07, inner_radii=0.065)
    cube = Cube(size=0.15, cage_mass=0.5, flywheel=flywheel, brushless_motor=brushless_motor, servo_motor=servo_motor, gimbal=gimbal)

    return cube
