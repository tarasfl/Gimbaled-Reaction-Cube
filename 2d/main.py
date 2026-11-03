from hardware.brushless_motor import BrushlessMotor
from hardware.servo_motor import ServoMotor
from hardware.flywheel import Flywheel
from hardware.gimbal import Gimbal
from hardware.cube import Cube
from optimisation.flywheel_optimisation import optimise_flywheel
from optimisation.optimise_servo import optimise_servo
from visualization.plotter import plot_mass_vs_flip_time_from_csv, plot_result
from visualization.pareto import plot_pareto_from_csv



brushless_motor = BrushlessMotor(nominal_rpm = 10000, nominal_torque = 0.118, rotor_inertia = 8.75*10**(-6), mass=0.071, radius=21*10**-3, length=0.05)
servo_motor = ServoMotor(mass=0.190, radius=0.053/2, length=0.04, rotor_inertia=97*1e-7, nominal_torque=1.6, max_rpm=130)
flywheel = Flywheel(outer_radii=0.063, inner_radii=0.045, thickness=0.0115)
gimbal = Gimbal(mass=0.2, outter_radii=0.07, inner_radii=0.065)
cube = Cube(size=0.15, cage_mass=0.5, flywheel=flywheel, brushless_motor=brushless_motor, servo_motor=servo_motor, gimbal=gimbal)



if __name__ == "__main__":
    best_run = optimise_servo(
        cube,
        power_limit_w=50.0, # Candidate rated-power ceiling.
        flywheel_rpm=3000,
        steps=3,
    )

    print("Best servo candidate:", best_run.candidate)
    print("Flip time:", best_run.flip_time_s)

    plot_result(best_run.result)