import sys
import types
import unittest
from enum import Enum
from unittest.mock import patch

from spacemouse_teleop.spacemouse.readers import PySpaceMouseReader


class ReaderTest(unittest.TestCase):
    def test_pyspacemouse_2x_reads_from_opened_device(self):
        class AxisConvention(Enum):
            ROS = "ros"

        class FakeState:
            x = 0.1
            y = -0.2
            z = 0.3
            roll = -0.4
            pitch = 0.5
            yaw = -0.6
            buttons = (1, 0)

        class FakeDevice:
            def __init__(self):
                self.closed = False

            def read(self):
                return FakeState()

            def close(self):
                self.closed = True

        opened_device = FakeDevice()
        captured_kwargs = {}

        def fake_open(**kwargs):
            captured_kwargs.update(kwargs)
            return opened_device

        fake_module = types.SimpleNamespace(
            AxisConvention=AxisConvention,
            open=fake_open,
        )
        previous = sys.modules.get("pyspacemouse")
        sys.modules["pyspacemouse"] = fake_module
        try:
            reader = PySpaceMouseReader(
                device="SpaceMouseCompact",
                device_index=2,
                axis_convention="ros",
            )
            with patch(
                "spacemouse_teleop.spacemouse.readers._preload_hidapi_on_macos"
            ):
                reader.open()
            state = reader.read()
            reader.close()
        finally:
            if previous is None:
                del sys.modules["pyspacemouse"]
            else:
                sys.modules["pyspacemouse"] = previous

        self.assertEqual(captured_kwargs["device"], "SpaceMouseCompact")
        self.assertEqual(captured_kwargs["device_index"], 2)
        self.assertEqual(captured_kwargs["axis_convention"], AxisConvention.ROS)
        self.assertEqual((state.x, state.y, state.z), (0.1, -0.2, 0.3))
        self.assertEqual((state.roll, state.pitch, state.yaw), (-0.4, 0.5, -0.6))
        self.assertEqual(state.buttons, (1, 0))
        self.assertTrue(opened_device.closed)


if __name__ == "__main__":
    unittest.main()
