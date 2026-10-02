"""xArm ROS2 backend adapter package."""

from .gripper import XArmGripperMapping, clamp_closedness

__all__ = ["XArmGripperMapping", "clamp_closedness"]
