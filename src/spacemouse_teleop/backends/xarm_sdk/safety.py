from __future__ import annotations

import math
from dataclasses import asdict, dataclass

from .config import XArmConfig


class TeleopFault(RuntimeError):
    pass


@dataclass(frozen=True)
class RobotFeedback:
    received_at: float
    sequence: int
    state: int
    mode: int
    error_code: int
    warn_code: int
    joints_rad: tuple
    tcp_mm_rad: tuple
    tcp_offset_mm_rad: tuple
    payload_kg_com_mm: tuple
    axis: int
    robot_type: int
    motor_enable: int
    motor_brake: int
    version: str

    def to_dict(self):
        return asdict(self)


def check_feedback(feedback, now, timeout, running=False, expected_mode=5):
    if feedback is None or not 0 <= now - feedback.received_at <= timeout:
        raise TeleopFault("Robot feedback missing or stale")
    if any(len(values) != count for values, count in ((feedback.joints_rad, 6), (feedback.tcp_mm_rad, 6), (feedback.tcp_offset_mm_rad, 6), (feedback.payload_kg_com_mm, 4))):
        raise TeleopFault("Malformed robot feedback")
    numbers = (*feedback.joints_rad, *feedback.tcp_mm_rad, *feedback.tcp_offset_mm_rad, *feedback.payload_kg_com_mm)
    if not all(math.isfinite(v) for v in numbers):
        raise TeleopFault("Robot feedback contains nonfinite values")
    if feedback.axis != 6 or feedback.robot_type != 6:
        raise TeleopFault("Connected robot is not xArm6")
    if feedback.error_code or feedback.warn_code:
        raise TeleopFault(f"Robot error={feedback.error_code}, warning={feedback.warn_code}; resolve in Studio")
    if running and (feedback.state not in (0, 1, 2) or feedback.mode != expected_mode
                    or feedback.motor_enable & 63 != 63 or feedback.motor_brake & 63 != 63):
        raise TeleopFault(f"Robot left velocity control: state={feedback.state}, mode={feedback.mode}")
    # State 5 is MODE_CHANGED: a stationary configuration-change latch.
    # Startup may initialize after check_site; a runtime transition still faults.
    if not running and feedback.state not in (0, 2, 4, 5):
        raise TeleopFault(f"Robot is not stationary/ready for startup: state={feedback.state}")


def check_site(config, feedback, initial=True):
    for name, observed, tolerances in (
        ("expected_tcp_offset_mm_rad", feedback.tcp_offset_mm_rad, (1, 1, 1, .01, .01, .01)),
        ("expected_payload_kg_com_mm", feedback.payload_kg_com_mm, (.01, 1, 1, 1)),
        ("initial_joints_rad", feedback.joints_rad, (.02,) * 6),
    ):
        if name == "initial_joints_rad" and not initial:
            continue
        expected = config.site[name]
        if any(abs(a - b) > t for a, b, t in zip(expected, observed, tolerances)):
            raise TeleopFault(f"Robot does not match site.{name}; no automatic reposition/configuration")


def check_homing_site(config, feedback):
    check_site(config, feedback, initial=False)
    if not config.site.get("workspace_limits_enabled", True):
        return
    position = tuple(v / 1000 for v in feedback.tcp_mm_rad[:3])
    if any(p < lo or p > hi for p, lo, hi in zip(
            position, config.site["workspace_min_m"], config.site["workspace_max_m"])):
        raise TeleopFault("TCP left the configured homing workspace")


def vector_limit(values, maximum):
    norm = math.sqrt(sum(v * v for v in values))
    scale = min(1.0, maximum / norm) if norm else 1.0
    return tuple(v * scale for v in values)


def slew(previous, target, acceleration, dt):
    if acceleration is None:
        return target
    delta = vector_limit(tuple(t - p for t, p in zip(target, previous)), acceleration * dt)
    return tuple(p + d for p, d in zip(previous, delta))


class VelocityLimiter:
    def __init__(self, config: XArmConfig):
        self.config = config
        self.reset()

    def reset(self):
        self.linear = self.angular = (0., 0., 0.)
        self.last_time = None

    def convert(self, command, now, duration, feedback=None, neutral=False):
        values = (*command.linear_vel_mps, *command.angular_vel_radps)
        if (command.frame != "link_base" or not all(math.isfinite(v) for v in values)
                or not math.isfinite(command.timestamp) or not 0 <= now - command.timestamp <= self.config.robot["input_timeout_s"]):
            raise TeleopFault("Invalid command frame, values, or timestamp")
        if not command.enabled or neutral:
            self.reset()
            return (0.,) * 6
        dt = 1 / self.config.robot["control_hz"] if self.last_time is None else now - self.last_time
        if not 0 < dt <= self.config.robot["input_timeout_s"]:
            raise TeleopFault("Control loop deadline missed")
        site, t = self.config.site, self.config.teleop
        v = vector_limit(command.linear_vel_mps, min(t.max_linear_speed_mps, site["max_linear_speed_mps"] or t.max_linear_speed_mps))
        w = vector_limit(command.angular_vel_radps, min(t.max_angular_speed_radps, site["max_angular_speed_radps"] or t.max_angular_speed_radps))
        v = slew(self.linear, v, site["max_linear_accel_mps2"], dt)
        w = slew(self.angular, w, site["max_angular_accel_radps2"], dt)
        # Bound the entire command lease, including distance needed to brake.
        lo, hi = site["workspace_min_m"], site["workspace_max_m"]
        if site.get("workspace_limits_enabled", True) and feedback is not None and lo is not None and hi is not None:
            clipped = []
            for speed, p, lower, upper in zip(v, feedback.tcp_mm_rad[:3], lo, hi):
                pos = p / 1000
                margin = upper - pos if speed >= 0 else pos - lower
                if margin <= 0:
                    speed = 0.
                else:
                    acc = site["max_linear_accel_mps2"]
                    horizon = duration + max(0., now - feedback.received_at)
                    cap = margin / horizon
                    if acc is not None:
                        cap = min(cap, math.sqrt((acc * horizon) ** 2 + 2 * acc * margin) - acc * horizon)
                    speed = math.copysign(min(abs(speed), cap), speed)
                clipped.append(speed)
            v = tuple(clipped)
        self.linear, self.angular, self.last_time = v, w, now
        return tuple(x * 1000 for x in v) + w
