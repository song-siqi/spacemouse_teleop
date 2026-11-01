from __future__ import annotations
import json
import math
from dataclasses import dataclass
from pathlib import Path
from robot_teleop.recording.native_writer import NATIVE_VIDEO_SETTINGS
from robot_teleop.backends.real.xarm_sdk.config import XArmConfig

DEFAULT_CONFIG = Path(__file__).resolve().parents[5] / "configs" / "spacemouse_xarm6_collection.json"
ROLES = ("wrist", "fixed_1", "fixed_2")

def positive(value, name, zero=False):
    if (isinstance(value, bool) or not isinstance(value, (int, float)) or
        not math.isfinite(value) or (value < 0 if zero else value <= 0)):
        raise ValueError(f"{name} must be finite and {'nonnegative' if zero else 'positive'}")

@dataclass(frozen=True)
class CollectionConfig:
    path: Path
    data: dict
    robot: XArmConfig

    @classmethod
    def load(cls, path=DEFAULT_CONFIG, ip=None):
        path = Path(path).expanduser().resolve()
        data = json.loads(path.read_text())
        if not isinstance(data, dict) or data.get("schema_version") not in (2, 3):
            raise ValueError("Raw v4 requires collection schema_version 2 or 3. Migrate the collection config: "
                             "replace sample_hz/robot_poll_hz/quality with native source, buffers and queues. "
                             "Existing v1/v2 data remains readable.")
        obsolete = {"sample_hz", "robot_poll_hz", "quality", "queue_size", "export_fps"} & data.keys()
        if obsolete: raise ValueError(f"Online alignment settings are not supported: {sorted(obsolete)}")
        if data.get("robot_state_source") != "tcp30000":
            raise ValueError("Raw v4 requires passive tcp30000; no silent polling fallback")
        real_path = (path.parent / data["robot_config"]).resolve()
        robot = XArmConfig.load(real_path, ip=ip)
        cameras = data["cameras"]
        if data["schema_version"] == 2 and (len(cameras) != 3 or {c["role"] for c in cameras} != set(ROLES)):
            raise ValueError("Exactly wrist, fixed_1 and fixed_2 cameras are required")
        import re
        roles = [c["role"] for c in cameras]
        if len(set(roles)) != len(roles) or any(not re.fullmatch(r"[A-Za-z0-9_-]+", role) for role in roles):
            raise ValueError("Camera roles must be unique safe source IDs")
        if set(roles) & {"robot", "gripper", "report", "commands"}:
            raise ValueError("Camera role collides with a state source")
        serials = [c.get("serial") for c in cameras if c.get("serial")]
        if len(set(serials)) != len(serials): raise ValueError("Camera serials must be unique")
        for camera in cameras:
            for field in ("width", "height", "fps"):
                value = camera[field]
                if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
                    raise ValueError(f"camera.{field} must be a positive integer")
            if camera["width"] % 2 or camera["height"] % 2:
                raise ValueError("H.264 yuv420p requires even dimensions; resizing is disabled")
        for name in ("pre_roll_s", "post_roll_s"):
            positive(data[name], name, zero=True)
        for name in ("source_timeout_s", "clock_interval_s"):
            positive(data[name], name)
        if set(data["queues"]) != {"video", "states", "session"}:
            raise ValueError("Specify independent video, states and session queues")
        for name, value in data["queues"].items():
            if not isinstance(value, int) or isinstance(value, bool) or value < 4:
                raise ValueError(f"queues.{name} must be an integer >= 4")
        reserve = data["min_free_bytes"]
        if not isinstance(reserve, int) or isinstance(reserve, bool) or reserve < 0:
            raise ValueError("min_free_bytes must be a nonnegative integer")
        settings = data.get("visual_storage", {})
        allowed = {"mode", "crf", "preset", "threads_per_camera", "gop"}
        if set(settings) - allowed or settings.get("mode") != "native_video":
            raise ValueError("visual_storage requires native_video with only supported encoder options")
        settings = {**NATIVE_VIDEO_SETTINGS, **settings}
        if not isinstance(settings["crf"], int) or isinstance(settings["crf"], bool) or not 0 <= settings["crf"] <= 51:
            raise ValueError("H.264 CRF must be an integer in [0,51]")
        if settings["preset"] not in ("ultrafast", "superfast", "veryfast", "faster", "fast", "medium"):
            raise ValueError("Unsupported H.264 preset")
        for key in ("threads_per_camera", "gop"):
            if not isinstance(settings[key], int) or isinstance(settings[key], bool) or settings[key] < 1:
                raise ValueError(f"{key} must be a positive integer")
        for c in cameras:
            if data["queues"]["video"] < math.ceil(data["pre_roll_s"] * c["fps"]) + 2:
                raise ValueError("Video queue must accommodate the entire pre-roll plus live frames")
        data["visual_storage"] = settings
        data["robot_config"] = str(real_path)
        data["output_root"] = str((path.parent / data["output_root"]).resolve())
        return cls(path, data, robot)

    def require_live(self):
        self.robot.require_live_settings()
        if not self.robot.gripper["enabled"]:
            raise ValueError("Collection requires actual gripper feedback")
        if any(not isinstance(c.get("serial"), str) or not c["serial"].strip() for c in self.data["cameras"]):
            raise ValueError("Bind all three camera serials before live collection")
