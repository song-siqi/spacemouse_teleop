"""Robot semantics without simulator or SDK imports."""
from dataclasses import dataclass


@dataclass(frozen=True)
class RobotDefinition:
    name: str
    joint_names: tuple[str, ...]
    velocity_limits: tuple[float, ...]
    base_frame: str


ROBOTS = {
    "xarm6": RobotDefinition("xarm6", tuple(f"joint{i}" for i in range(1, 7)), (2.14,) * 6, "link_base"),
    "fr3": RobotDefinition("fr3", tuple(f"fr3_joint{i}" for i in range(1, 8)), (2., 1., 1.5, 1.25, 3., 1.5, 3.), "fr3_link0"),
}
