from hardware.cube import Cube
from dynamics.state import SystemState
import numpy as np



class Dynamics2D:
    def __init__(self, cube):
        self.cube = cube


    def derivatives(self, t, x, rpm, gimbal_torque):
        self.cube.update_total_mass()   

        state = SystemState.from_vector(x)

        theta = state.theta
        theta_d = state.theta_d
        gamma = state.gamma
        gamma_d = state.gamma_d 

        A = self.cube.effective_inertia(gamma)
        A_prime = self.cube.effective_inertia_derivative(gamma)

        B = self.cube.gimbal.effective_inertia(flywheel=self.cube.flywheel, brushless_motor=self.cube.brushless_motor, servo_motor=self.cube.servo_motor)



        work_rpm = rpm
        H = self.cube.flywheel.calculate_angular_momentum(work_rpm)

        gravity = self.cube.gravitational_torque(theta)

        theta_dd = (gravity + H * gamma_d * np.cos(gamma) - A_prime * gamma_d * theta_d) / A
        gamma_dd = (gimbal_torque - H * theta_d * np.cos(gamma) + 0.5 * A_prime * theta_d**2) / B

        return np.array([theta_d, theta_dd, gamma_d, gamma_dd])