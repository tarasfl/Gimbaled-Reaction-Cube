import numpy as np



class ServoMotor:
    def __init__(self, mass, radius, length, rotor_inertia, nominal_torque, max_rpm):



        self.mass = mass
        self.radius = radius
        self.length = length

        self.rotor_inertia = rotor_inertia
        self.max_rpm = max_rpm

        self.torque = nominal_torque


    # calculating the inertia of the stepper motor as a solid cylinder
    def calculate_inertia(self):
        return 0.5 * self.mass * self.radius**2
