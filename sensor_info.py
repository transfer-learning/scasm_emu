from typing import NamedTuple, List


class SensorInfo(NamedTuple):
    sonar: List[float]
    theta: float
    velocity: float
