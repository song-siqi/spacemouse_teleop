"""Device-independent, named processing records; seconds/metres/radians."""

from dataclasses import dataclass
import math
from typing import Protocol


@dataclass
class State:
    time: float
    joints: list[float]
    position: list[float]
    quaternion: list[float]  # wxyz
    gripper: float
    gripper_pulse: float

    def __post_init__(self):
        values = [
            self.time,
            *self.joints,
            *self.position,
            *self.quaternion,
            self.gripper,
            self.gripper_pulse,
        ]
        if (
            not self.joints
            or len(self.position) != 3
            or len(self.quaternion) != 4
            or not all(math.isfinite(v) for v in values)
        ):
            raise ValueError("Malformed canonical state")
        if (
            abs(sum(v * v for v in self.quaternion) - 1) > 1e-5
            or not 0 <= self.gripper <= 1
        ):
            raise ValueError("Invalid orientation or gripper state")


class SourceAdapter(Protocol):
    def read(self, path, metadata): ...


class RobotDataAdapter(Protocol):
    def robot_values(self, row): ...
    def gripper_values(self, row, config): ...
