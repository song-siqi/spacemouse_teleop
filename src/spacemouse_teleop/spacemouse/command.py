from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Dict, Iterable, Tuple


Vector3 = Tuple[float, float, float]


def vector3(values: Iterable[float]) -> Vector3:
    x, y, z = values
    return (float(x), float(y), float(z))


@dataclass(frozen=True)
class RawSpaceMouseState:
    """Raw normalized state from a SpaceMouse-like 6-DoF input device."""

    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    buttons: Tuple[int, ...] = field(default_factory=tuple)
    timestamp: float = 0.0

    def axis(self, name: str) -> float:
        try:
            return float(getattr(self, name))
        except AttributeError as exc:
            raise KeyError(f"unknown SpaceMouse axis: {name}") from exc

    def to_dict(self) -> Dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class TeleopCommand:
    """Device-independent command used by real and simulated robot backends."""

    linear_vel_mps: Vector3
    angular_vel_radps: Vector3
    delta_pos_m: Vector3
    delta_rot_rad: Vector3
    # Target gripper closedness. 0.0 is fully open, 1.0 is fully closed.
    gripper: float
    enabled: bool
    frame: str
    dt: float
    timestamp: float
    # Change in normalized gripper closedness for this command step.
    delta_gripper: float = 0.0

    def to_dict(self) -> Dict[str, object]:
        return asdict(self)
