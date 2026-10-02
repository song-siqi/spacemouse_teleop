from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol, Union

Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]


def finite(values, length=None):
    if (length is not None and len(values) != length) or not all(math.isfinite(v) for v in values):
        raise ValueError("Invalid numeric vector")


@dataclass(frozen=True)
class Stamp:
    seconds: float
    clock: str
    epoch: int = 0

    def __post_init__(self):
        if not math.isfinite(self.seconds) or not self.clock or not isinstance(self.epoch, int) or self.epoch < 0:
            raise ValueError("Invalid clock stamp")

    def remaining(self, now: Stamp) -> float:
        if (self.clock, self.epoch) != (now.clock, now.epoch):
            raise ValueError("Cannot compare deadlines across clocks or epochs")
        return self.seconds - now.seconds


@dataclass(frozen=True)
class Pose:
    position: Vector3
    quaternion: Quaternion
    frame: str = "world"

    def __post_init__(self):
        finite(self.position, 3)
        finite(self.quaternion, 4)
        if not math.isclose(sum(v*v for v in self.quaternion), 1., abs_tol=1e-6):
            raise ValueError("Quaternion must be normalized wxyz")


@dataclass(frozen=True)
class CartesianTwist:
    linear: Vector3 = (0., 0., 0.)
    angular: Vector3 = (0., 0., 0.)
    frame: str = "world"

    def __post_init__(self):
        finite(self.linear, 3)
        finite(self.angular, 3)


@dataclass(frozen=True)
class CartesianPoseTarget:
    pose: Pose


@dataclass(frozen=True)
class JointPositionTarget:
    names: tuple[str, ...]
    positions: tuple[float, ...]

    def __post_init__(self):
        finite(self.positions, len(self.names))
        if len(set(self.names)) != len(self.names):
            raise ValueError("Duplicate joint names")


@dataclass(frozen=True)
class GripperCommand:
    intent: str = "hold"
    position: float | None = None

    def __post_init__(self):
        if self.intent not in ("open", "close", "hold", "position"):
            raise ValueError("Unknown gripper intent")
        if self.intent == "position":
            if self.position is None or not 0 <= self.position <= 1:
                raise ValueError("Normalized position must be in [0, 1]")
        elif self.position is not None:
            raise ValueError("Position requires position intent")


Command = Union[CartesianTwist, CartesianPoseTarget, JointPositionTarget, GripperCommand]


@dataclass(frozen=True)
class GripperState:
    normalized_position: float | None
    native_position: tuple[float, ...] | None
    native_unit: str
    execution: str


@dataclass(frozen=True)
class RobotState:
    stamp: Stamp
    joint_names: tuple[str, ...]
    joint_positions: tuple[float, ...]
    joint_velocities: tuple[float, ...] | None
    tcp: Pose
    gripper: GripperState | None
    execution: str


class Backend(Protocol):
    def connect(self) -> None: ...
    def capabilities(self) -> frozenset[type]: ...
    def observe(self) -> RobotState: ...
    def submit(self, command: Command, deadline: Stamp) -> None: ...
    def stop(self) -> None: ...
    def close(self) -> None: ...


class SimulationBackend(Backend, Protocol):
    def reset(self) -> RobotState: ...
    def advance(self, dt: float | None = None) -> RobotState: ...
    def render(self, camera: str, width: int, height: int): ...
