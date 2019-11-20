import numpy as np


class State:
    def __init__(self, pose=np.asmatrix([0.0, 0.0, 0.0]).T, twist=np.asmatrix([0.0, 0.0]).T):
        self.pose = pose.copy()
        self.twist = twist.copy()

class DE2Config:
    def __init__(self, axle_length: float = 0.122):
        self.axle_length = axle_length

    def __str__(self):
        return f"axle_length: {self.axle_length}"

    def __repr__(self):
        return f"axle_length: {self.axle_length}"


class DE2Bot:
    def __init__(self, state: State = State(), config: DE2Config = DE2Config()):
        self.state = State(state.pose)
        self.config = config

        self.G = np.asmatrix([
            [1.0, 1.0],
            [-1 / self.config.axle_length, 1 / self.config.axle_length]
        ])
        self.Ginv = np.linalg.inv(self.G)

    def apply(self, u: np.ndarray, dt: float):
        self.state.twist = self.G * u
        linear, angular = self.state.twist[0, 0], self.state.twist[1, 0]
        heading = self.state.pose[2, 0]

        self.state.pose = self.state.pose + dt * np.asmatrix(
            [[linear * np.cos(heading),
              linear * np.sin(heading),
              angular]]).T

    def left_right(self):
        return self.Ginv * self.state.twist
