import unittest

from spacemouse_teleop.spacemouse.command import RawSpaceMouseState
from spacemouse_teleop.spacemouse import TeleopConfig, TeleopCore


class TeleopCoreTest(unittest.TestCase):
    def test_scales_and_integrates_cartesian_command(self):
        config = TeleopConfig(
            filter_alpha=1.0,
            deadzone=0.0,
            linear_scale_mps=0.1,
            angular_scale_radps=0.5,
        )
        core = TeleopCore(config)
        core.process(
            RawSpaceMouseState(
                x=0.0,
                y=0.0,
                z=0.0,
                roll=0.0,
                pitch=0.0,
                yaw=0.0,
                timestamp=1.0,
            )
        )
        command = core.process(
            RawSpaceMouseState(
                x=1.0,
                y=-0.5,
                z=0.25,
                roll=0.2,
                pitch=-0.4,
                yaw=0.6,
                timestamp=1.1,
            )
        )

        self.assertEqual(command.linear_vel_mps, (0.1, -0.05, 0.025))
        self.assertEqual(command.angular_vel_radps, (0.1, -0.2, 0.3))
        self.assertAlmostEqual(command.delta_pos_m[0], 0.01)
        self.assertAlmostEqual(command.delta_rot_rad[2], 0.03)

    def test_deadman_disables_motion(self):
        config = TeleopConfig(
            require_deadman=True,
            deadman_button=0,
            filter_alpha=1.0,
            deadzone=0.0,
        )
        core = TeleopCore(config)
        command = core.process(
            RawSpaceMouseState(
                x=1.0,
                y=1.0,
                z=1.0,
                roll=1.0,
                pitch=1.0,
                yaw=1.0,
                buttons=(0, 0),
                timestamp=1.0,
            )
        )

        self.assertFalse(command.enabled)
        self.assertEqual(command.linear_vel_mps, (0.0, 0.0, 0.0))
        self.assertEqual(command.angular_vel_radps, (0.0, 0.0, 0.0))

    def test_gripper_buttons_emit_intent_without_motion_modes(self):
        config = TeleopConfig(
            filter_alpha=1.0,
            deadzone=0.0,
        )
        core = TeleopCore(config)

        core.process(
            RawSpaceMouseState(
                x=0.0,
                y=0.0,
                z=0.0,
                roll=0.0,
                pitch=0.0,
                yaw=0.0,
                buttons=(0, 0),
                timestamp=1.0,
            )
        )
        closing = core.process(
            RawSpaceMouseState(
                x=1.0,
                y=1.0,
                z=1.0,
                roll=1.0,
                pitch=1.0,
                yaw=1.0,
                buttons=(0, 1),
                timestamp=1.2,
            )
        )
        self.assertNotEqual(closing.linear_vel_mps, (0.0, 0.0, 0.0))
        self.assertNotEqual(closing.angular_vel_radps, (0.0, 0.0, 0.0))
        self.assertEqual(closing.gripper_intent, "close")
        self.assertAlmostEqual(closing.delta_gripper, 0.0)
        self.assertAlmostEqual(closing.gripper_velocity, 0.0)
        self.assertIsNone(closing.gripper)

        opening = core.process(
            RawSpaceMouseState(
                x=1.0,
                y=1.0,
                z=1.0,
                roll=1.0,
                pitch=1.0,
                yaw=1.0,
                buttons=(1, 0),
                timestamp=1.4,
            )
        )
        self.assertEqual(opening.gripper_intent, "open")
        self.assertAlmostEqual(opening.delta_gripper, 0.0)
        self.assertAlmostEqual(opening.gripper_velocity, 0.0)
        self.assertIsNone(opening.gripper)

        both = core.process(
            RawSpaceMouseState(
                x=1.0,
                y=1.0,
                z=1.0,
                roll=1.0,
                pitch=1.0,
                yaw=1.0,
                buttons=(1, 1),
                timestamp=1.6,
            )
        )
        self.assertEqual(both.gripper_intent, "hold")


if __name__ == "__main__":
    unittest.main()
