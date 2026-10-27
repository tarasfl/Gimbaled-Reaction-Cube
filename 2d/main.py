from brushless_motor import BrushlessMotor
from servo_motor import ServoMotor
from flywheel import Flywheel
from gimbal import Gimbal
from cube import Cube

brushless_motor = BrushlessMotor(nominal_rpm = 6290, nominal_torque = 0.0598, rotor_inertia = 8.75*10**(-6), mass=0.112, radius=21*10**-3, length=0.05)
servo_motor = ServoMotor(mass=0.112, radius=33*10**-3, length=0.134, rotor_inertia=8.75*10**-6, nominal_torque=0.1, max_rpm=1600)
flywheel = Flywheel()
gimbal = Gimbal(mass=0.2, outter_radii=0.07, inner_radii=0.065)


cube = Cube(size=0.15, flywheel=flywheel, brushless_motor=brushless_motor, servo_motor=servo_motor, gimbal=gimbal)