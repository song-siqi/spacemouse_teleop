"""Explicit engine names and transmission mappings for supported assemblies."""
from dataclasses import dataclass
from .constants import JOINT_NAMES, HOME_QPOS, GRIPPER_JOINT_NAMES, GRIPPER_PAD_GEOM_NAMES, GRIPPER_GUARD_GEOM_NAMES


@dataclass(frozen=True)
class GripperBinding:
    joints: tuple[str, ...]
    actuators: tuple[str, ...]
    driver: str
    control_range: tuple[float, float]
    driver_range: tuple[float, float]
    pads: tuple[str, ...]
    guards: tuple[str, ...]
    normalized_speed: float = 2.


@dataclass(frozen=True)
class RobotBinding:
    robot: str
    tool: str
    joints: tuple[str, ...]
    actuators: tuple[str, ...]
    home: tuple[float, ...]
    tcp: str
    velocities: tuple[float, ...]
    gripper: GripperBinding


XARM = RobotBinding("xarm6", "xarm_gripper", JOINT_NAMES,
    tuple(f"{j}_pos" for j in JOINT_NAMES), HOME_QPOS, "eef", (2.14,) * 6,
    GripperBinding(GRIPPER_JOINT_NAMES, ("gripper_pos",), "drive_joint", (0., .85), (0., .85),
                   GRIPPER_PAD_GEOM_NAMES, GRIPPER_GUARD_GEOM_NAMES))
FR3_JOINTS = tuple(f"fr3_joint{i}" for i in range(1, 8))
FR3 = RobotBinding("fr3", "robotiq_2f85", FR3_JOINTS, FR3_JOINTS,
    (0., 0., 0., -1.57079, 0., 1.57079, -.7853), "tool_pinch",
    (2., 1., 1.5, 1.25, 3., 1.5, 3.),
    GripperBinding(tuple(f"tool_{side}_{part}_joint" for side in ("right", "left")
                        for part in ("driver", "coupler", "spring_link", "follower")),
                   ("tool_fingers_actuator",), "tool_right_driver_joint", (0., 255.), (0., .8),
                   tuple(f"tool_{side}_pad{i}" for side in ("left", "right") for i in (1, 2)), ()))
BINDINGS = {(b.robot, b.tool): b for b in (XARM, FR3)}
