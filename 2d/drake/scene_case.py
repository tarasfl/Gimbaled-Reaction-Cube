"""Load the selected optimisation case and apply its hardware to the baseline cube."""
import ast
import csv
from itertools import islice
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from hardware.flywheel import Flywheel
from hardware.parameters import make_cube


CASE_CSV = Path(__file__).resolve().parents[2] / "static" / "FULL_RANGE_TEST_2026-11-03_20-08-21.csv"
SELECTED_ROW = 18786


def load_scene_case(path=CASE_CSV, selected_row=SELECTED_ROW):
    with path.open(newline="") as source:
        return next(islice(csv.DictReader(source), selected_row, selected_row + 1))


def make_scene_cube(case):
    """Apply the selected case's component values to a fresh baseline cube."""
    cube = make_cube()
    cube.size = float(case["cube_size"])
    cube.cage_mass = float(case["cube_cage_mass"])
    cube.servo_position = np.array([0.0, cube.size / 2])
    cube.flywheel = Flywheel(**ast.literal_eval(case["candidate"]))
    cube.gimbal.mass = float(case["cube_gimbal_mass"])
    cube.gimbal.outer_radii = float(case["cube_gimbal_outer_radii"])
    cube.gimbal.inner_radii = float(case["cube_gimbal_inner_radii"])
    cube.gimbal.inertia = cube.gimbal.calculate_inertia()
    cube.gimbal.transverse_inertia = cube.gimbal.calculate_transverse_inertia()
    cube.servo_motor.mass = float(case["cube_servo_motor_mass"])
    cube.servo_motor.torque = float(case["cube_servo_motor_torque"])
    cube.servo_motor.max_rpm = float(case["cube_servo_motor_max_rpm"])
    cube.brushless_motor.mass = float(case["cube_brushless_motor_mass"])
    cube.brushless_motor.nominal_rpm = float(case["cube_brushless_motor_nominal_rpm"])
    cube.brushless_motor.nominal_torque = float(case["cube_brushless_motor_nominal_torque"])
    cube.brushless_motor.inertia = cube.brushless_motor.calculate_inertia()
    cube.brushless_motor.transverse_inertia = cube.brushless_motor.calculate_transverse_inertia()
    cube.update_total_mass()
    return cube
