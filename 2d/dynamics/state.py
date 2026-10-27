from dataclasses import dataclass
import numpy as np

@dataclass
class SystemState:
    theta: float = 0.0
    theta_d: float = 0.0
    gamma: float = 0.0
    gamma_d: float = 0.0

    def to_vector(self):
        return np.array([
            self.theta,
            self.theta_d,
            self.gamma,
            self.gamma_d
        ])


    @classmethod
    def from_vector(cls, vector):
        return cls(
            theta=vector[0],
            theta_d=vector[1],
            gamma=vector[2],
            gamma_d=vector[3]
        )