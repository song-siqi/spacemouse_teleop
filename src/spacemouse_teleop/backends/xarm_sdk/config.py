from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

from spacemouse_teleop.spacemouse import TeleopConfig

DEFAULT_CONFIG = Path(__file__).resolve().parents[4] / "configs/spacemouse_xarm6_real.json"
SITE_FIELDS = (
    "expected_tcp_offset_mm_rad", "expected_payload_kg_com_mm", "initial_joints_rad",
    "workspace_min_m", "workspace_max_m", "max_linear_speed_mps",
    "max_angular_speed_radps", "max_linear_accel_mps2", "max_angular_accel_radps2",
)


def positive(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be a finite positive number")
    return float(value)


def vector(value, n, name):
    if not isinstance(value, (list, tuple)) or len(value) != n:
        raise ValueError(f"{name} must contain {n} numbers")
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value):
        raise ValueError(f"{name} must contain finite numbers")
    return tuple(float(v) for v in value)


@dataclass
class XArmConfig:
    teleop: TeleopConfig
    reader: dict
    robot: dict
    gripper: dict
    site: dict

    @classmethod
    def load(cls, path=DEFAULT_CONFIG, ip=None):
        with Path(path).open(encoding="utf-8") as file:
            data = json.load(file)
        for key in ("teleop", "reader", "robot", "gripper", "site"):
            if not isinstance(data.get(key), dict):
                raise ValueError(f"Real config requires its own '{key}' section; sim configs cannot be used here")
        t = data["teleop"]
        # Require every adjustable input setting: no inherited sim/debug defaults.
        required = ("frame", "require_deadman", "deadman_button", "translation_only_button", "rotation_only_button", "gripper", "deadzone", "filter_alpha", "linear_scale_mps", "angular_scale_radps", "max_linear_speed_mps", "max_angular_speed_radps", "timeout_s", "mapping")
        for key in required:
            if key not in t:
                raise ValueError(f"Missing teleop.{key}")
        if t["frame"] != "link_base" or t["require_deadman"] is not False:
            raise ValueError("v1 requires link_base and require_deadman=false")
        for name in ("deadzone", "filter_alpha"):
            value = positive(t[name], f"teleop.{name}")
            if value > 1 or (name == "deadzone" and value == 1):
                raise ValueError(f"Invalid teleop.{name}")
        for name in ("linear_scale_mps", "angular_scale_radps", "max_linear_speed_mps", "max_angular_speed_radps", "timeout_s"):
            positive(t[name], f"teleop.{name}")
        for group in ("linear", "angular"):
            if set(t["mapping"][group]) != {"x", "y", "z"}:
                raise ValueError("Each mapping must have x/y/z")
            for rule in t["mapping"][group].values():
                if rule.get("source") not in ("x", "y", "z", "roll", "pitch", "yaw") or rule.get("sign") not in (-1, 1):
                    raise ValueError("Mapping requires a valid source and sign +/-1")
        buttons = [t["gripper"].get(k) for k in ("open_button", "close_button")]
        if any(type(v) is not int or v < 0 for v in buttons) or buttons[0] == buttons[1]:
            raise ValueError("Gripper buttons must be distinct nonnegative indices")
        r, reader, g, site = data["robot"], data["reader"], data["gripper"], data["site"]
        if ip is not None:
            r["ip"] = ip
        if not isinstance(r.get("ip"), str) or not r["ip"]:
            raise ValueError("robot.ip is required")
        for name in ("control_hz", "command_duration_s", "input_timeout_s", "feedback_timeout_s", "socket_timeout_s", "startup_timeout_s"):
            positive(r.get(name), f"robot.{name}")
        if r["command_duration_s"] > r["input_timeout_s"] or r["command_duration_s"] <= 1 / r["control_hz"]:
            raise ValueError("Command duration must exceed the send period and not exceed input timeout")
        if r["socket_timeout_s"] >= r["command_duration_s"]:
            raise ValueError("robot.socket_timeout_s must be shorter than command duration")
        homing = r.get("homing", {"enabled": False})
        if not isinstance(homing, dict) or type(homing.get("enabled")) is not bool:
            raise ValueError("robot.homing.enabled must be boolean")
        if homing["enabled"]:
            for key in ("speed_radps", "accel_radps2", "tolerance_rad", "settle_s", "timeout_s"):
                positive(homing.get(key), f"robot.homing.{key}")
            if homing["settle_s"] < r["command_duration_s"] or homing["timeout_s"] <= homing["settle_s"]:
                raise ValueError("Homing requires settling for at least one command duration and a longer timeout")
        positive(reader.get("poll_hz"), "reader.poll_hz")
        if reader.get("backend") not in ("mock", "pyspacemouse") or reader.get("axis_convention") != "ros":
            raise ValueError("v1 reader requires mock/pyspacemouse and explicit ros axis convention")
        if type(reader.get("device_index")) is not int or reader["device_index"] < 0:
            raise ValueError("Invalid reader.device_index")
        for name in ("poll_hz", "socket_timeout_s"):
            positive(g.get(name), f"gripper.{name}")
        if type(g.get("enabled")) is not bool:
            raise ValueError("gripper.enabled must be boolean")
        for name in ("open_pulse", "close_pulse"):
            if type(g.get(name)) is not int or not 0 <= g[name] <= 850:
                raise ValueError(f"Invalid gripper.{name}")
        if g["open_pulse"] <= g["close_pulse"]:
            raise ValueError("gripper.open_pulse must exceed close_pulse")
        opening = g.get("startup_open")
        if not isinstance(opening, dict):
            raise ValueError("gripper.startup_open settings are required")
        for key in ("timeout_s", "settle_s"):
            positive(opening.get(key), f"gripper.startup_open.{key}")
        tolerance = opening.get("tolerance_pulse")
        if type(tolerance) is not int or not 0 <= tolerance <= 50 or tolerance >= g["open_pulse"] - g["close_pulse"]:
            raise ValueError("gripper.startup_open.tolerance_pulse must be 0..50 and smaller than endpoint separation")
        if opening["settle_s"] < 1 / g["poll_hz"] or opening["timeout_s"] <= opening["settle_s"]:
            raise ValueError("Gripper startup opening needs a settling interval and a longer timeout")
        if g.get("model") not in (None, "xarm_gripper"):
            raise ValueError("v1 supports confirmed xarm_gripper only")
        for name in ("speed_pulse_s", "baudrate"):
            if g.get(name) is not None:
                positive(g[name], f"gripper.{name}")
        if g.get("speed_pulse_s") is not None and (type(g["speed_pulse_s"]) is not int or g["speed_pulse_s"] > 5000):
            raise ValueError("gripper.speed_pulse_s must be an integer in 1..5000")
        if g.get("baudrate") is not None and type(g["baudrate"]) is not int:
            raise ValueError("gripper.baudrate must be an integer")
        if type(site.get("workspace_limits_enabled", True)) is not bool:
            raise ValueError("site.workspace_limits_enabled must be boolean")
        lengths = {"expected_tcp_offset_mm_rad": 6, "expected_payload_kg_com_mm": 4, "initial_joints_rad": 6, "workspace_min_m": 3, "workspace_max_m": 3}
        for name in SITE_FIELDS:
            if name not in site:
                raise ValueError(f"Missing site.{name} (use null until confirmed)")
            if site[name] is not None:
                if name in lengths:
                    vector(site[name], lengths[name], f"site.{name}")
                else:
                    positive(site[name], f"site.{name}")
        if site["expected_payload_kg_com_mm"] is not None and site["expected_payload_kg_com_mm"][0] < 0:
            raise ValueError("Payload mass cannot be negative")
        if site["workspace_min_m"] is not None and site["workspace_max_m"] is not None:
            if any(lo >= hi for lo, hi in zip(site["workspace_min_m"], site["workspace_max_m"])):
                raise ValueError("Workspace minimum must be below maximum")
        return cls(TeleopConfig.from_mapping(t), reader, r, g, site)

    def missing_site_settings(self):
        optional = {"workspace_min_m", "workspace_max_m"} if not self.site.get("workspace_limits_enabled", True) else set()
        missing = [f"site.{k}" for k in SITE_FIELDS if self.site[k] is None and k not in optional]
        if self.gripper["enabled"]:
            missing += [f"gripper.{k}" for k in ("model", "baudrate", "speed_pulse_s") if self.gripper.get(k) is None]
        return missing

    def require_live_settings(self):
        missing = self.missing_site_settings()
        if missing:
            raise ValueError("Live motion unavailable; confirm and configure: " + ", ".join(missing))
