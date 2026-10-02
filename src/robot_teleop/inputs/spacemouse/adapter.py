"""Live input adapter. Constructing it never opens a device."""
from types import SimpleNamespace
import time
from robot_teleop.core import CartesianTwist, GripperCommand
from robot_teleop.inputs.adapter import convert_teleop
from .core import TeleopConfig, TeleopCore


class SpaceMouseInput:
    def __init__(self, config, reader=None):
        # The polling helper has no SDK imports; it enforces report freshness.
        from robot_teleop.inputs.spacemouse.polling import LatestInput
        teleop = TeleopConfig.from_mapping(config.get("teleop", {}))
        self.core = TeleopCore(teleop)
        self.config = SimpleNamespace(teleop=teleop,
            reader={"backend":"pyspacemouse","poll_hz":250,"device":None,"device_index":0,"axis_convention":"ros",**config.get("reader",{})},
            robot={"input_timeout_s":.25,"command_duration_s":.2})
        self.source = LatestInput(self.config,reader=reader)

    def open(self):
        self.source.start()

    def sample(self, elapsed):
        from robot_teleop.inputs.spacemouse.polling import input_deadline
        now = time.monotonic()
        raw, intent = self.source.snapshot(now)
        if raw is None or input_deadline(raw,now,self.config,intent) is None:
            return CartesianTwist(), GripperCommand()
        return convert_teleop(self.core.process(raw),"world")

    def close(self):
        self.source.close()
