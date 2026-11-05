from hardware.brushless_motor import BrushlessMotor
from hardware.servo_motor import ServoMotor
from hardware.flywheel import Flywheel
from hardware.gimbal import Gimbal
from hardware.cube import Cube
from optimisation.flywheel_optimisation import optimise_flywheel
from optimisation.optimise_servo import optimise_servo
from visualization.plotter import plot_mass_vs_flip_time_from_csv, plot_result
from visualization.pareto import plot_pareto_from_csv



# Shared with the Drake model; each simulation gets fresh hardware.
from hardware.parameters import make_cube

cube = make_cube()
brushless_motor = cube.brushless_motor
servo_motor = cube.servo_motor
flywheel = cube.flywheel
gimbal = cube.gimbal

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