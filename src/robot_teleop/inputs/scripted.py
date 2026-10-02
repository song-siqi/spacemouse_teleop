"""Finite, hardware-free Cartesian scripts in SI units."""
from robot_teleop.core import CartesianTwist, GripperCommand


class ScriptedInput:
    def __init__(self, segments):
        self.segments = segments

    def sample(self, elapsed):
        for segment in self.segments:
            if elapsed < segment["duration"]:
                return CartesianTwist(tuple(segment.get("linear", (0.,)*3)), tuple(segment.get("angular", (0.,)*3))), GripperCommand(segment.get("gripper", "hold"))
            elapsed -= segment["duration"]
        return CartesianTwist(), GripperCommand()
