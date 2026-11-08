"""Load the two selected optimisation stages without running another sweep."""
import ast
import csv
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from hardware.flywheel import Flywheel
from hardware.parameters import make_cube
from optimisation.optimisation import select_best_weighted


def selected_row(path, weights):
    with path.open(newline="") as source:
        rows = [SimpleNamespace(**row, csv_line=line)
                for line, row in enumerate(csv.DictReader(source), start=2)]
    return select_best_weighted(rows, weights)


def load_saved_case():
    static = Path(__file__).resolve().parents[2] / "static"
    wheel_path = static / "FULL_RANGE_TEST_2026-11-03_20-08-21.csv"
    servo_path = static / "SERVO_FULL_RANGE_TEST_2026-11-03_20-08-51.csv"
    wheel = selected_row(wheel_path, {"component_mass": 1, "flip_time_s": 1, "target_rpm": 1})
    servo = selected_row(servo_path, {"torque": 1, "max_rpm": 0.75, "flip_time_s": 0.75})
    cube = make_cube()
    cube.flywheel = Flywheel(**ast.literal_eval(wheel.candidate))
    cube.servo_motor.torque = float(servo.torque)
    cube.servo_motor.max_rpm = float(servo.max_rpm)
    cube.update_total_mass()
    if not np.isclose(cube.total_mass, float(servo.cube_total_mass), atol=1e-12, rtol=0):
        raise ValueError("Current hardware masses do not match the saved result.")
    if float(wheel.target_rpm) != float(servo.target_rpm):
        raise ValueError("The two saved stages use different flywheel speeds.")
    reference = {key: float(getattr(servo, key)) for key in (
        "initial_gamma_deg", "target_gamma_deg", "flywheel_rpm", "flip_time_s",
        "gimbal_angle_at_flip_deg", "peak_gimbal_speed_deg_s", "peak_torque_nm",
        "cube_total_mass", "torque", "max_rpm",
    )}
    reference.update(wheel_geometry=ast.literal_eval(wheel.candidate),
                     wheel_source=str(wheel_path), wheel_csv_line=wheel.csv_line,
                     servo_source=str(servo_path), servo_csv_line=servo.csv_line)
    return cube, reference
