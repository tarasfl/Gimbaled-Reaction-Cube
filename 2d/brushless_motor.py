import numpy as np

class BrushlessMotor:
    def __init__(self, rotor_inertia, nominal_rpm, nominal_torque, mass=0, radius=0, length=0, offset=0.01):
        self.mass = mass
        self.radius = radius
        self.length = length
        self.offset = offset # offset from rotation axis
    
        self.inertia = self.calculate_inertia()
        self.transverse_inertia = self.calculate_transverse_inertia()
        self.rotor_inertia = rotor_inertia
        
        self.nominal_rpm = nominal_rpm
        self.omega = 0.0  # flywheel speed [rad/s]
        self.nominal_torque = nominal_torque

        

    def angular_momentum(self):
        return self.inertia * self.omega

    def calculate_transverse_inertia(self):
        return (
            self.mass / 12
            * (3 * self.radius**2 + self.length**2)
        )


    def calculate_inertia(self):
        return 0.5 * self.mass * self.radius**2

    def calculate_rpm(self, flywheel):
        inertia = flywheel.inertia + self.rotor_inertia
        target_omega = self.nominal_torque / inertia
        return target_omega * 60 / (2 * np.pi)