"""Explicit JSON references resolve relative to the file that declares them."""
import json
from pathlib import Path


def load_experiment(path):
    path = Path(path).resolve()
    value = json.loads(path.read_text())
    if value.get("schema_version") != 1:
        raise ValueError("Experiment schema_version must be 1")
    for key in ("robot", "end_effector", "backend", "setup", "input", "recording"):
        if key not in value:
            raise ValueError(f"Missing experiment section: {key}")
        section = value[key]
        base = path.parent
        if isinstance(section, str):
            section_path = (path.parent / section).resolve()
            value[key] = json.loads(section_path.read_text())
            base = section_path.parent
        if not isinstance(value[key], dict):
            raise ValueError(f"Expected object for {key}")
        if "config" in value[key]:
            value[key]["config"] = str((base / value[key]["config"]).resolve())
    return value


def make_backend(config):
    combination = (config["robot"]["name"], config["end_effector"]["name"], config["backend"]["name"])
    if combination in (("xarm6", "xarm_gripper", "mujoco"), ("fr3", "robotiq_2f85", "mujoco")):
        if config["setup"]["name"] != "tabletop":
            raise ValueError("Only the explicit tabletop setup is registered")
        from robot_teleop.backends.sim.mujoco.backend import MujocoBackend
        return MujocoBackend(*combination[:2], control_hz=config["backend"].get("control_hz", 60))
    if combination == ("xarm6", "xarm_gripper", "xarm_sdk"):
        from robot_teleop.backends.real.xarm_sdk.backend import XArmBackend
        from robot_teleop.backends.real.xarm_sdk.config import XArmConfig
        return XArmBackend(XArmConfig.load(config["backend"]["config"]))
    raise ValueError(f"Unsupported combination: {combination}")
