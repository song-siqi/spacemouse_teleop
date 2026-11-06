"""Robot semantics without simulator or SDK imports."""
from dataclasses import dataclass


@dataclass(frozen=True)
class RobotDefinition:
    name: str
    joint_names: tuple[str, ...]
    velocity_limits: tuple[float, ...]
    base_frame: str
    position_limits: tuple[tuple[float, float], ...] | None = None
    continuous_joints: tuple[str, ...] = ()


ROBOTS = {
    "xarm6": RobotDefinition("xarm6", tuple(f"joint{i}" for i in range(1, 7)), (2.14,) * 6, "link_base", ((-6.283185307179586,6.283185307179586),(-2.059,2.0944),(-3.927,.19198),(-6.283185307179586,6.283185307179586),(-1.69297,3.141592653589793),(-6.283185307179586,6.283185307179586))),
    "fr3": RobotDefinition("fr3", tuple(f"fr3_joint{i}" for i in range(1, 8)), (2., 1., 1.5, 1.25, 3., 1.5, 3.), "fr3_link0"),
}
