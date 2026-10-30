import numpy as np

class Cube:
    def __init__(self, size: float, flywheel, brushless_motor, servo_motor, gimbal,
                 cage_mass: float = 0.3):
        # Require motor/wheel data explicitly rather than silently invent hardware.
        self.size = size

        self.cage_mass = cage_mass

        self.flywheel = flywheel
        self.gimbal = gimbal
        self.brushless_motor = brushless_motor
        self.servo_motor = servo_motor

        self.servo_position = np.array([0, self.size / 2])
        self.update_total_mass()

    def update_total_mass(self):
        masses = [self.cage_mass, self.brushless_motor.mass, self.flywheel.mass,
                  self.gimbal.mass, self.servo_motor.mass]
        self.total_mass = np.sum(masses)
        return self.total_mass

    def calculate_cage_inertia(self):
        return (5 / 18) * self.cage_mass * self.size**2

    def fixed_inertia(self):
        center_distance_sq = self.size**2 / 2
        x, z = self.servo_position
        servo_distance_sq = (x - self.size / 2)**2 + (z + self.size / 2)**2

        servo = self.servo_motor.mass * servo_distance_sq + self.servo_motor.calculate_inertia()
        parallel_axis = center_distance_sq * sum([self.flywheel.mass, self.brushless_motor.mass, self.gimbal.mass, self.cage_mass])

        return servo + parallel_axis + self.calculate_cage_inertia()

    def effective_inertia(self, gamma):
        components = [self.flywheel, self.gimbal, self.brushless_motor]

        Jt = sum(c.calculate_transverse_inertia() for c in components)
        Js = sum(c.calculate_inertia() for c in components)

        return self.fixed_inertia() + Jt + (Js - Jt) * np.sin(gamma)**2

    def effective_inertia_derivative(self, gamma):
        components = [self.flywheel, self.gimbal, self.brushless_motor]

        Jt = sum(c.calculate_transverse_inertia() for c in components)
        Js = sum(c.calculate_inertia() for c in components)

        return (Js - Jt) * np.sin(2 * gamma)

    def spin_inertia(self):
        return self.flywheel.calculate_inertia() + self.brushless_motor.rotor_inertia

    def center_of_mass(self): 
        #  COM in respect to the pivot point 
        return np.array([-self.size / 2, self.size / 2]) + (self.servo_motor.mass / self.total_mass) * self.servo_position

    def gravitational_torque(self, theta):
        xp, zp = self.center_of_mass()

        return self.total_mass * 9.81 * (xp * np.cos(theta) + zp * np.sin(theta))

    def potential_energy(self, theta):
        xp, zp = self.center_of_mass()

        return self.total_mass * 9.81 * (-xp * np.sin(theta) + zp * np.cos(theta))

    def balance_angle(self):
        xp, zp = self.center_of_mass()

        return float(np.arctan2(-xp, zp))
