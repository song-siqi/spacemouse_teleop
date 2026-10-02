"""Discard legacy integrated deltas at the input boundary."""
from robot_teleop.core import CartesianTwist, GripperCommand


def convert_teleop(command, frame):
    if not command.enabled:
        return CartesianTwist(frame=frame), GripperCommand()
    return CartesianTwist(command.linear_vel_mps,command.angular_vel_radps,frame), GripperCommand(command.gripper_intent)
