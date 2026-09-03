from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Mapping, Optional, Tuple

from spacemouse_teleop.spacemouse.command import (
    RawSpaceMouseState,
    TeleopCommand,
    Vector3,
)


def _clamp(value: float, limit: float) -> float:
    limit = abs(float(limit))
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def _deadzone(value: float, threshold: float) -> float:
    return 0.0 if abs(value) < threshold else value


def _scale_vector(values: Vector3, scale: float, limit: float) -> Vector3:
    return tuple(_clamp(value * scale, limit) for value in values)  # type: ignore[return-value]


def _ema(previous: Vector3, current: Vector3, alpha: float) -> Vector3:
    return tuple(alpha * cur + (1.0 - alpha) * prev for prev, cur in zip(previous, current))  # type: ignore[return-value]


@dataclass(frozen=True)
class MappingRule:
    source: str
    sign: float = 1.0

    @classmethod
    def from_mapping(cls, data: Mapping[str, object]) -> "MappingRule":
        return cls(source=str(data["source"]), sign=float(data.get("sign", 1.0)))


@dataclass
class TeleopConfig:
    frame: str = "link_base"
    require_deadman: bool = False
    deadman_button: int = 0
    translation_only_button: Optional[int] = None
    rotation_only_button: Optional[int] = None
    gripper_open_button: Optional[int] = 0
    gripper_close_button: Optional[int] = 1
    gripper_speed_per_s: float = 0.8
    gripper_initial_closedness: float = 0.0
    deadzone: float = 0.08
    filter_alpha: float = 0.35
    linear_scale_mps: float = 0.12
    angular_scale_radps: float = 0.8
    max_linear_speed_mps: float = 0.2
    max_angular_speed_radps: float = 1.0
    timeout_s: float = 0.25
    linear_mapping: Dict[str, MappingRule] = field(
        default_factory=lambda: {
            "x": MappingRule("x"),
            "y": MappingRule("y"),
            "z": MappingRule("z"),
        }
    )
    angular_mapping: Dict[str, MappingRule] = field(
        default_factory=lambda: {
            "x": MappingRule("roll"),
            "y": MappingRule("pitch"),
            "z": MappingRule("yaw"),
        }
    )

    @classmethod
    def from_mapping(cls, data: Mapping[str, object]) -> "TeleopConfig":
        config = cls()
        for key in (
            "frame",
            "require_deadman",
            "deadman_button",
            "translation_only_button",
            "rotation_only_button",
            "gripper_open_button",
            "gripper_close_button",
            "gripper_speed_per_s",
            "gripper_initial_closedness",
            "deadzone",
            "filter_alpha",
            "linear_scale_mps",
            "angular_scale_radps",
            "max_linear_speed_mps",
            "max_angular_speed_radps",
            "timeout_s",
        ):
            if key in data:
                setattr(config, key, data[key])

        gripper = data.get("gripper")
        if isinstance(gripper, Mapping):
            if "open_button" in gripper:
                config.gripper_open_button = gripper["open_button"]  # type: ignore[assignment]
            if "close_button" in gripper:
                config.gripper_close_button = gripper["close_button"]  # type: ignore[assignment]
            if "speed_per_s" in gripper:
                config.gripper_speed_per_s = float(gripper["speed_per_s"])
            if "initial_closedness" in gripper:
                config.gripper_initial_closedness = float(
                    gripper["initial_closedness"]
                )

        mapping = data.get("mapping")
        if isinstance(mapping, Mapping):
            linear = mapping.get("linear")
            angular = mapping.get("angular")
            if isinstance(linear, Mapping):
                config.linear_mapping = {
                    axis: MappingRule.from_mapping(rule)
                    for axis, rule in linear.items()
                    if isinstance(rule, Mapping)
                }
            if isinstance(angular, Mapping):
                config.angular_mapping = {
                    axis: MappingRule.from_mapping(rule)
                    for axis, rule in angular.items()
                    if isinstance(rule, Mapping)
                }

        config.deadman_button = int(config.deadman_button)
        config.translation_only_button = _optional_int(config.translation_only_button)
        config.rotation_only_button = _optional_int(config.rotation_only_button)
        config.gripper_open_button = _optional_int(config.gripper_open_button)
        config.gripper_close_button = _optional_int(config.gripper_close_button)
        config.gripper_speed_per_s = max(0.0, float(config.gripper_speed_per_s))
        config.gripper_initial_closedness = _clamp01(config.gripper_initial_closedness)
        config.deadzone = float(config.deadzone)
        config.filter_alpha = _clamp(float(config.filter_alpha), 1.0)
        config.linear_scale_mps = float(config.linear_scale_mps)
        config.angular_scale_radps = float(config.angular_scale_radps)
        config.max_linear_speed_mps = float(config.max_linear_speed_mps)
        config.max_angular_speed_radps = float(config.max_angular_speed_radps)
        config.timeout_s = float(config.timeout_s)
        config.require_deadman = bool(config.require_deadman)
        config.frame = str(config.frame)
        return config


class TeleopCore:
    """Convert raw SpaceMouse states into stable, backend-neutral commands."""

    def __init__(self, config: Optional[TeleopConfig] = None) -> None:
        self.config = config or TeleopConfig()
        self._last_timestamp: Optional[float] = None
        self._filtered_linear: Vector3 = (0.0, 0.0, 0.0)
        self._filtered_angular: Vector3 = (0.0, 0.0, 0.0)
        self._gripper_closedness = _clamp01(self.config.gripper_initial_closedness)

    def process(self, state: RawSpaceMouseState) -> TeleopCommand:
        now = float(state.timestamp)
        dt = self._compute_dt(now)

        enabled = self._is_enabled(state)
        linear_raw = self._mapped_vector(state, self.config.linear_mapping)
        angular_raw = self._mapped_vector(state, self.config.angular_mapping)

        linear_raw = tuple(_deadzone(v, self.config.deadzone) for v in linear_raw)  # type: ignore[assignment]
        angular_raw = tuple(_deadzone(v, self.config.deadzone) for v in angular_raw)  # type: ignore[assignment]

        if self._button_pressed(state, self.config.translation_only_button):
            angular_raw = (0.0, 0.0, 0.0)
        if self._button_pressed(state, self.config.rotation_only_button):
            linear_raw = (0.0, 0.0, 0.0)
        if not enabled:
            linear_raw = (0.0, 0.0, 0.0)
            angular_raw = (0.0, 0.0, 0.0)

        delta_gripper = self._gripper_delta(state, dt, enabled)
        if delta_gripper:
            self._gripper_closedness = _clamp01(
                self._gripper_closedness + delta_gripper
            )

        linear_vel = _scale_vector(
            linear_raw, self.config.linear_scale_mps, self.config.max_linear_speed_mps
        )
        angular_vel = _scale_vector(
            angular_raw, self.config.angular_scale_radps, self.config.max_angular_speed_radps
        )

        alpha = float(self.config.filter_alpha)
        self._filtered_linear = _ema(self._filtered_linear, linear_vel, alpha)
        self._filtered_angular = _ema(self._filtered_angular, angular_vel, alpha)

        delta_pos = tuple(v * dt for v in self._filtered_linear)  # type: ignore[assignment]
        delta_rot = tuple(v * dt for v in self._filtered_angular)  # type: ignore[assignment]

        return TeleopCommand(
            linear_vel_mps=self._filtered_linear,
            angular_vel_radps=self._filtered_angular,
            delta_pos_m=delta_pos,
            delta_rot_rad=delta_rot,
            gripper=self._gripper_closedness,
            enabled=enabled,
            frame=self.config.frame,
            dt=dt,
            timestamp=now,
            delta_gripper=delta_gripper,
        )

    def zero_command(self, timestamp: float) -> TeleopCommand:
        self._filtered_linear = (0.0, 0.0, 0.0)
        self._filtered_angular = (0.0, 0.0, 0.0)
        self._last_timestamp = timestamp
        return TeleopCommand(
            linear_vel_mps=(0.0, 0.0, 0.0),
            angular_vel_radps=(0.0, 0.0, 0.0),
            delta_pos_m=(0.0, 0.0, 0.0),
            delta_rot_rad=(0.0, 0.0, 0.0),
            gripper=self._gripper_closedness,
            enabled=False,
            frame=self.config.frame,
            dt=0.0,
            timestamp=timestamp,
            delta_gripper=0.0,
        )

    def _compute_dt(self, timestamp: float) -> float:
        if self._last_timestamp is None:
            self._last_timestamp = timestamp
            return 0.0

        dt = max(0.0, timestamp - self._last_timestamp)
        self._last_timestamp = timestamp
        if dt > self.config.timeout_s:
            self._filtered_linear = (0.0, 0.0, 0.0)
            self._filtered_angular = (0.0, 0.0, 0.0)
            return 0.0
        return dt

    def _mapped_vector(
        self, state: RawSpaceMouseState, mapping: Mapping[str, MappingRule]
    ) -> Vector3:
        return (
            mapping["x"].sign * state.axis(mapping["x"].source),
            mapping["y"].sign * state.axis(mapping["y"].source),
            mapping["z"].sign * state.axis(mapping["z"].source),
        )

    def _is_enabled(self, state: RawSpaceMouseState) -> bool:
        if not self.config.require_deadman:
            return True
        return self._button_pressed(state, self.config.deadman_button)

    def _gripper_delta(
        self, state: RawSpaceMouseState, dt: float, enabled: bool
    ) -> float:
        if not enabled or dt <= 0.0:
            return 0.0
        opening = self._button_pressed(state, self.config.gripper_open_button)
        closing = self._button_pressed(state, self.config.gripper_close_button)
        direction = float(int(closing) - int(opening))
        return direction * self.config.gripper_speed_per_s * dt

    @staticmethod
    def _button_pressed(state: RawSpaceMouseState, index: Optional[int]) -> bool:
        if index is None:
            return False
        return 0 <= index < len(state.buttons) and bool(state.buttons[index])


def _optional_int(value: object) -> Optional[int]:
    if value is None:
        return None
    return int(value)
