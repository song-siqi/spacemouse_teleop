from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Dict, Iterable, Literal, Optional, Tuple


Vector3 = Tuple[float, float, float]
GripperIntent = Literal["hold", "open", "close"]


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
    enabled: bool
    frame: str
    dt: float
    timestamp: float
    gripper_intent: GripperIntent = "hold"
    # Optional legacy/backend-internal target gripper closedness.
    # SpaceMouse teleop should prefer gripper_intent.
    gripper: Optional[float] = None
    # Change in normalized gripper closedness for this command step.
    delta_gripper: float = 0.0
    # Normalized gripper closedness velocity; positive closes, negative opens.
    gripper_velocity: float = 0.0

    def to_dict(self) -> Dict[str, object]:
        return asdict(self)
