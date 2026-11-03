from hardware.brushless_motor import BrushlessMotor
from hardware.servo_motor import ServoMotor
from hardware.flywheel import Flywheel
from hardware.gimbal import Gimbal
from hardware.cube import Cube
from optimisation.flywheel_optimisation import optimise_flywheel
from optimisation.optimise_servo import optimise_servo
from visualization.plotter import plot_mass_vs_flip_time_from_csv, plot_result
from visualization.pareto import plot_pareto_from_csv



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

initial_degs = [-30, -20, -10, 0]
target_degs = [60, 75, 90, 100]

flywheel_rpms = list(range(1000, 7001, 500))

outer_radius_values = [0.045, 0.050, 0.055, 0.060, 0.063]
inner_radius_values = [0.020, 0.030, 0.035, 0.040, 0.045]
thickness_values = [0.006, 0.008, 0.010, 0.0115, 0.014, 0.016]

if __name__ == "__main__":
    best_flywheel = optimise_flywheel(
        cube,
        csv_specs="FULL_RANGE_TEST",
        steps=3,
        initial_degs=initial_degs,
        target_degs=target_degs,
        outer_radius_values=outer_radius_values,
        inner_radius_values=inner_radius_values,
        thickness_values=thickness_values,
        rpms=flywheel_rpms,
        duration=15.0,
    )

    # Transfer the winning geometry into the next stage.
    cube.flywheel = Flywheel(**best_flywheel.candidate)
    cube.update_total_mass()

    best_run = optimise_servo(
        cube,
        flywheel_rpm=best_flywheel.target_rpm,
        power_limit_w=50.0,
        steps=3,
        initial_degs=initial_degs,
        target_degs=target_degs,
        duration=15.0,
    )

    print("Best flywheel:", best_flywheel.candidate)
    print("Flywheel RPM:", best_flywheel.target_rpm)
    print("Best servo requirements:", best_run.candidate)
    print("Flip time:", best_run.flip_time_s)

    plot_result(best_run.result)