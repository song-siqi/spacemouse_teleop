import unittest

from spacemouse_teleop.backends.xarm_ros2 import XArmGripperMapping


class XArmRos2GripperTest(unittest.TestCase):
    def test_closedness_maps_to_action_radians_and_service_pulses(self):
        mapping = XArmGripperMapping()

        self.assertAlmostEqual(mapping.to_action_position_rad(0.0), 0.0)
        self.assertAlmostEqual(mapping.to_action_position_rad(1.0), 0.86)
        self.assertAlmostEqual(mapping.to_action_position_rad(0.5), 0.43)

        self.assertAlmostEqual(mapping.to_service_position_pulse(0.0), 850.0)
        self.assertAlmostEqual(mapping.to_service_position_pulse(1.0), 0.0)
        self.assertAlmostEqual(mapping.to_service_position_pulse(0.5), 425.0)

    def test_backend_specific_values_map_back_to_closedness(self):
        mapping = XArmGripperMapping()

        self.assertAlmostEqual(mapping.from_action_position_rad(0.43), 0.5)
        self.assertAlmostEqual(mapping.from_service_position_pulse(425.0), 0.5)
        self.assertAlmostEqual(mapping.from_action_position_rad(100.0), 1.0)
        self.assertAlmostEqual(mapping.from_service_position_pulse(-100.0), 1.0)


if __name__ == "__main__":
    unittest.main()
