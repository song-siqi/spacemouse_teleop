"""Tool identity; transmission and physical models belong to backends."""
from dataclasses import dataclass


@dataclass(frozen=True)
class EndEffectorDefinition:
    name: str
    intents: tuple[str, ...] = ("open", "close", "hold", "position")


END_EFFECTORS = {name: EndEffectorDefinition(name) for name in ("xarm_gripper", "robotiq_2f85")}
