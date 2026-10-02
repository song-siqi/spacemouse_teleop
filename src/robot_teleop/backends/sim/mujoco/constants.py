from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple


JOINT_NAMES: Tuple[str, ...] = (
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
)

END_EFFECTOR_NONE = "none"
END_EFFECTOR_XARM_GRIPPER = "xarm_gripper"
END_EFFECTOR_NAMES: Tuple[str, ...] = (END_EFFECTOR_NONE, END_EFFECTOR_XARM_GRIPPER)

GRIPPER_DRIVE_JOINT_NAME = "drive_joint"
GRIPPER_JOINT_NAMES: Tuple[str, ...] = (
    GRIPPER_DRIVE_JOINT_NAME,
    "left_finger_joint",
    "left_inner_knuckle_joint",
    "right_outer_knuckle_joint",
    "right_finger_joint",
    "right_inner_knuckle_joint",
)
GRIPPER_ACTUATOR_NAMES: Tuple[str, ...] = ("gripper_pos",)
GRIPPER_BODY_NAMES: Tuple[str, ...] = (
    "xarm_gripper_base_link",
    "left_outer_knuckle",
    "left_finger",
    "left_inner_knuckle",
    "right_outer_knuckle",
    "right_finger",
    "right_inner_knuckle",
)
GRIPPER_PAD_GEOM_NAMES: Tuple[str, ...] = (
    "left_finger_pad_collision",
    "right_finger_pad_collision",
)
GRIPPER_FINGER_MESH_COLLISION_GEOM_NAMES: Tuple[str, ...] = (
    "left_finger_mesh_collision",
    "right_finger_mesh_collision",
)
GRIPPER_GUARD_GEOM_NAMES: Tuple[str, ...] = (
    "gripper_palm_collision",
    "left_outer_knuckle_guard_collision",
    "right_outer_knuckle_guard_collision",
    "left_finger_guard_collision",
    "right_finger_guard_collision",
)
GRIPPER_JOINT_LIMIT_RAD: Tuple[float, float] = (0.0, 0.85)
GRIPPER_ACTION_LIMIT_RAD: Tuple[float, float] = (0.0, 0.86)
GRIPPER_SERVICE_PULSE_RANGE: Tuple[float, float] = (0.0, 850.0)

# The realmove xarm_ros2 launch defaults to limited:=true. These ranges mirror
# xarm6_robot_macro.xacro for that mode.
JOINT_LIMITS: Tuple[Tuple[float, float], ...] = (
    (-math.pi * 0.99, math.pi * 0.99),
    (-2.059, 2.0944),
    (-math.pi * 0.99, 0.19198),
    (-math.pi * 0.99, math.pi * 0.99),
    (-1.69297, math.pi * 0.99),
    (-math.pi * 0.99, math.pi * 0.99),
)

# MoveIt config/xarm6/joint_limits.yaml uses 2.14 rad/s for all six joints.
JOINT_VELOCITY_LIMIT_RADPS = 2.14

# A non-singular tabletop pose that keeps the end effector above the cube.
HOME_QPOS: Tuple[float, ...] = (0.0, -0.65, -0.45, 0.0, 1.05, 0.0)


@dataclass(frozen=True)
class XarmRos2Alignment:
    servo_cartesian_topic: str = "/servo_server/delta_twist_cmds"
    trajectory_topic: str = "/xarm6_traj_controller/joint_trajectory"
    gripper_action_name: str = "/xarm_gripper/gripper_action"
    gripper_enable_service: str = "/xarm/set_gripper_enable"
    gripper_mode_service: str = "/xarm/set_gripper_mode"
    gripper_speed_service: str = "/xarm/set_gripper_speed"
    gripper_position_service: str = "/xarm/set_gripper_position"
    planning_frame: str = "link_base"
    ee_frame: str = "link_tcp"
    ros2_controller_rate_hz: float = 150.0
    moveit_servo_publish_period_s: float = 0.067
    moveit_servo_command_timeout_s: float = 0.2
    sdk_position_call: str = "set_servo_angle_j"
