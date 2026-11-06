import json, math
from pathlib import Path

MODES = ("joint_position", "delta_joint_position", "ee_pose", "delta_ee_pose")
DEFAULTS = dict(
    fps=20,
    action_mode="delta_joint_position",
    max_image_error_s=0.025,
    max_camera_skew_s=0.04,
    max_robot_gap_s=0.02,
    max_gripper_age_s=0.1,
    robot="xarm6",
    gripper_open=850.0,
    gripper_closed=230.0,
)


def load(path=None, **overrides):
    c = {
        **DEFAULTS,
        **(json.loads(Path(path).read_text()) if path else {}),
        **{k: v for k, v in overrides.items() if v is not None},
    }
    if isinstance(c["fps"], bool) or not isinstance(c["fps"], int) or c["fps"] <= 0:
        raise ValueError("fps must be a positive integer")
    if c["action_mode"] not in MODES:
        raise ValueError("Unknown action mode")
    for k in (
        "max_image_error_s",
        "max_camera_skew_s",
        "max_robot_gap_s",
        "max_gripper_age_s",
    ):
        if not math.isfinite(c[k]) or c[k] <= 0:
            raise ValueError("Invalid quality threshold: " + k)
    if c["gripper_open"] == c["gripper_closed"]:
        raise ValueError("Degenerate gripper mapping")
    return c
