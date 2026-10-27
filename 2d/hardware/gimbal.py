class Gimbal:

    def __init__(self, mass, outter_radii, inner_radii):
        self.mass = mass

        self.outer_radii = outter_radii
        self.inner_radii = inner_radii

        self.inertia = self.calculate_inertia()
        self.transverse_inertia = self.calculate_transverse_inertia()   

        self.gamma = 0

    def calculate_inertia(self):
        self.inertia = 0.5 * self.mass * (self.outer_radii**2 + self.inner_radii**2)
        return self.inertia

    def calculate_transverse_inertia(self):
        return self.mass * (self.outer_radii**2 + self.inner_radii**2) * 0.25


    def effective_inertia(self, flywheel, servo_motor, brushless_motor):
        return (
            self.calculate_inertia()
            + flywheel.calculate_transverse_inertia()
            + servo_motor.rotor_inertia
            + brushless_motor.transverse_inertia
            + brushless_motor.offset**2 * brushless_motor.mass
        )