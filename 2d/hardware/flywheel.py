import numpy as np
from hardware.brushless_motor import BrushlessMotor

class Flywheel:

    def __init__(self, outer_radii:float=0.065, inner_radii:float=0.045, thikness:float=0.01, density:float=2700):

        # Dimensions of the flywheel Geometry
        self.outer_radii = outer_radii
        self.inner_radii = inner_radii
        self.thickness = thikness

        # Material properties of the flywheel Mass
        self.density = density
        self.volume = np.pi * self.thickness * (self.outer_radii**2 - self.inner_radii**2)
        self.mass = self.update_flywheel_mass()

        # Inertia properties of the flywheel
        self.inertia = self.calculate_inertia()
        self.transverse_inertia = self.calculate_transverse_inertia()

        # RPMs
        self.max_rpm = 12800
        self.work_rpm = 0
    


    def calculate_inertia(self):
        self.inertia = 0.5 * self.mass * (self.outer_radii**2 + self.inner_radii**2)
        return self.inertia

    def update_flywheel_mass(self):
        self.mass = self.density * self.volume
        self.inertia = self.calculate_inertia()
        self.transverse_inertia = self.calculate_transverse_inertia()
        return self.mass
    
    def recalculate_thikness(self, target_inertia):
        self.thickness = 2 * target_inertia / (np.pi * self.density * (self.outer_radii**4 - self.inner_radii**4))
        self.volume = np.pi * self.thickness * (self.outer_radii**2 - self.inner_radii**2)
        self.update_flywheel_mass()
        return self.thickness

    def calculate_transverse_inertia(self):
        return (1/12) * self.mass * (3 * (self.outer_radii**2 + self.inner_radii**2) + self.thickness**2)

    def calculate_angular_momentum(self, work_rpm):
        self.work_rpm = work_rpm
        omega = 2 * np.pi * self.work_rpm / 60  # Convert RPM to rad/s
        return self.inertia * omega

    def calculate_spin_energy(self):
        omega = 2 * np.pi * self.work_rpm / 60  # Convert RPM to rad/s
        return 0.5 * self.inertia * omega**2