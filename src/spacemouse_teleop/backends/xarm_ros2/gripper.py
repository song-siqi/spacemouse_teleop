from __future__ import annotations

from dataclasses import dataclass


def clamp_closedness(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


@dataclass(frozen=True)
class XArmGripperMapping:
    """Map backend-neutral gripper closedness into official xArm gripper units."""

    action_open_rad: float = 0.0
    action_closed_rad: float = 0.86
    service_open_pulse: float = 850.0
    service_closed_pulse: float = 0.0

    def to_action_position_rad(self, closedness: float) -> float:
        closedness = clamp_closedness(closedness)
        return self.action_open_rad + (
            self.action_closed_rad - self.action_open_rad
        ) * closedness

    def to_service_position_pulse(self, closedness: float) -> float:
        closedness = clamp_closedness(closedness)
        return self.service_open_pulse + (
            self.service_closed_pulse - self.service_open_pulse
        ) * closedness

    def from_action_position_rad(self, position_rad: float) -> float:
        span = self.action_closed_rad - self.action_open_rad
        if span == 0.0:
            return 0.0
        return clamp_closedness((float(position_rad) - self.action_open_rad) / span)

    def from_service_position_pulse(self, position_pulse: float) -> float:
        span = self.service_closed_pulse - self.service_open_pulse
        if span == 0.0:
            return 0.0
        return clamp_closedness((float(position_pulse) - self.service_open_pulse) / span)
