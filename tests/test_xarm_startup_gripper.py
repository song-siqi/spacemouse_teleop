import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import test_xarm_homing as homing_tests
from spacemouse_teleop.backends.xarm_sdk.config import DEFAULT_CONFIG, XArmConfig
from spacemouse_teleop.backends.xarm_sdk.safety import TeleopFault


class StartupGripperTest(unittest.TestCase):
    def rig(self):
        config, world, backend, recorder, rows = homing_tests.HomingTest().rig()
        config.gripper['enabled'] = True
        config.gripper['close_pulse'] = 230
        arm_factory = backend.channel_factory

        class Gripper:
            position = 208.
            target = 208
            speed = 1000.
            blocked = False
            state_override = None
            target_override = None
            init_error = False
            hold_error = False
            error_code = 0
            closed = False
            targets = []

            def initialize_gripper(self, settings):
                world.calls.append('gripper_init')
                if self.init_error:
                    raise TeleopFault('gripper initialization failed')

            def gripper_target(self, pulse):
                if self.hold_error and self.targets:
                    raise TeleopFault('lost hold reply')
                self.targets.append(pulse)
                world.calls.append(('gripper_target', pulse))
                # This device has read back 840 after a requested 850, with a
                # stopped measured position of ~838. Model both observations.
                self.target = min(840, pulse)

            def gripper_status(self):
                moving = abs(self.position - min(838, self.target)) > 1
                return {'position_pulse': round(self.position), 'error_code': self.error_code,
                        'controller_target_pulse': self.target if self.target_override is None else self.target_override,
                        'motion_state': self.state_override or ('moving' if moving else 'stopped'),
                        'received_at': world.now}

            def close(self):
                self.closed = True
                world.calls.append('gripper_channel_close')

        gripper = Gripper()
        sleep = world.sleep

        def advance(seconds):
            if not gripper.blocked:
                goal = min(838, gripper.target)
                delta = max(-gripper.speed * seconds, min(gripper.speed * seconds, goal - gripper.position))
                gripper.position += delta
            sleep(seconds)

        world.sleep = advance
        backend.channel_factory = lambda ip, timeout, **kw: gripper if timeout == config.gripper['socket_timeout_s'] else arm_factory(ip, timeout, **kw)
        return config, world, backend, recorder, rows, gripper

    def assert_no_arm_motion(self, world, backend):
        self.assertNotIn('home_mode', world.calls)
        self.assertFalse(any(isinstance(c, tuple) and c[0] == 'joint' for c in world.calls))
        self.assertFalse(backend.armed)

    def test_open_confirmed_before_any_arm_motion_with_device_endpoint_tolerance(self):
        config, world, backend, recorder, rows, gripper = self.rig()
        with patch('spacemouse_teleop.backends.xarm_sdk.backend.time', world):
            backend.home(recorder, lambda: None)
        events = [r['event'] for r in rows]
        self.assertLess(events.index('startup_gripper_open_completed'), events.index('homing_started'))
        self.assertLess(world.calls.index(('gripper_target', 850)), world.calls.index('home_mode'))
        self.assertLess(world.calls.index('gripper_channel_close'), world.calls.index('home_mode'))
        self.assertEqual(gripper.targets, [850])
        opened = next(r for r in rows if r['event'] == 'startup_gripper_open_completed')
        self.assertEqual(opened['gripper']['position_pulse'], 838)
        self.assertEqual(opened['gripper']['controller_target_pulse'], 840)
        self.assertEqual(events[-1], 'homing_completed')
        self.assertTrue(gripper.closed)

    def test_ack_or_stopped_state_alone_cannot_allow_homing_when_blocked(self):
        config, world, backend, recorder, rows, gripper = self.rig()
        config.gripper['startup_open']['timeout_s'] = .4
        gripper.blocked, gripper.state_override = True, 'stopped'
        with patch('spacemouse_teleop.backends.xarm_sdk.backend.time', world):
            with self.assertRaisesRegex(TeleopFault, 'opening timed out'):
                backend.home(recorder, lambda: None)
        self.assert_no_arm_motion(world, backend)
        self.assertEqual(gripper.targets, [850, 208])
        self.assertEqual(rows[-1]['event'], 'startup_gripper_open_failed')
        self.assertTrue(gripper.closed)

    def test_near_open_position_requires_stopped_state_and_open_target(self):
        for state, target in (('moving', 840), ('grasped', 840), ('stopped', 230)):
            with self.subTest(state=state, target=target):
                config, world, backend, recorder, rows, gripper = self.rig()
                config.gripper['startup_open']['timeout_s'] = .4
                gripper.position = 838
                gripper.state_override, gripper.target_override = state, target
                with patch('spacemouse_teleop.backends.xarm_sdk.backend.time', world):
                    with self.assertRaisesRegex(TeleopFault, 'opening timed out'):
                        backend.home(recorder, lambda: None)
                self.assert_no_arm_motion(world, backend)
                self.assertTrue(gripper.closed)

    def test_initialization_failure_does_not_send_open_or_arm_motion(self):
        config, world, backend, recorder, rows, gripper = self.rig()
        gripper.init_error = True
        with patch('spacemouse_teleop.backends.xarm_sdk.backend.time', world):
            with self.assertRaisesRegex(TeleopFault, 'initialization failed'):
                backend.home(recorder, lambda: None)
        self.assertEqual(gripper.targets, [])
        self.assert_no_arm_motion(world, backend)
        self.assertTrue(gripper.closed)

    def test_input_fault_or_interrupt_holds_gripper_and_never_starts_arm(self):
        for error in (TeleopFault('input disconnected'), KeyboardInterrupt()):
            config, world, backend, recorder, rows, gripper = self.rig()
            def guard():
                if world.now > 10.1:
                    raise error
            with patch('spacemouse_teleop.backends.xarm_sdk.backend.time', world):
                with self.assertRaises(type(error)):
                    backend.home(recorder, guard)
            self.assertEqual(len(gripper.targets), 2)
            self.assertEqual(gripper.targets[-1], round(gripper.position))
            self.assert_no_arm_motion(world, backend)
            self.assertTrue(gripper.closed)

    def test_input_failure_after_init_is_checked_before_open_write(self):
        config, world, backend, recorder, rows, gripper = self.rig()
        def guard():
            if 'gripper_init' in world.calls:
                raise TeleopFault('input disappeared during initialization')
        with patch('spacemouse_teleop.backends.xarm_sdk.backend.time', world):
            with self.assertRaisesRegex(TeleopFault, 'input disappeared'):
                backend.home(recorder, guard)
        self.assertEqual(gripper.targets, [])
        self.assert_no_arm_motion(world, backend)
        self.assertTrue(gripper.closed)

    def test_stale_arm_feedback_during_opening_blocks_homing(self):
        config, world, backend, recorder, rows, gripper = self.rig()
        def guard():
            if world.now > 10.1:
                world.stale = True
        with patch('spacemouse_teleop.backends.xarm_sdk.backend.time', world):
            with self.assertRaisesRegex(TeleopFault, 'feedback missing or stale'):
                backend.home(recorder, guard)
        self.assert_no_arm_motion(world, backend)
        self.assertTrue(gripper.closed)

    def test_opening_error_and_failed_cleanup_are_reported(self):
        config, world, backend, recorder, rows, gripper = self.rig()
        gripper.error_code, gripper.hold_error = 7, True
        with patch('spacemouse_teleop.backends.xarm_sdk.backend.time', world):
            with self.assertRaisesRegex(TeleopFault, 'Startup gripper error: 7; hold failed'):
                backend.home(recorder, lambda: None)
        self.assertEqual(rows[-1]['hold_error'], 'lost hold reply')
        self.assert_no_arm_motion(world, backend)
        self.assertTrue(gripper.closed)

    def test_arm_timeout_starts_after_gripper_opening_completes(self):
        config, world, backend, recorder, rows, gripper = self.rig()
        world.joints = [0.] * 6
        config.robot['homing']['timeout_s'] = .6
        gripper.speed = 300.
        with patch('spacemouse_teleop.backends.xarm_sdk.backend.time', world):
            backend.home(recorder, lambda: None)
        opened = next(r['timestamp'] for r in rows if r['event'] == 'startup_gripper_open_completed')
        home_started = next(r['timestamp'] for r in rows if r['event'] == 'homing_started')
        self.assertGreater(opened - 10., .6)
        self.assertGreaterEqual(home_started, opened)
        self.assertLess(rows[-1]['timestamp'] - home_started, .6)

    def test_invalid_opening_settings_are_rejected_before_connecting(self):
        for key, value in (('timeout_s', 0), ('timeout_s', .1), ('settle_s', .001),
                           ('tolerance_pulse', 850), ('tolerance_pulse', -1)):
            data = json.loads(DEFAULT_CONFIG.read_text())
            data['gripper']['startup_open'][key] = value
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'invalid.json'
                path.write_text(json.dumps(data))
                with self.assertRaises(ValueError):
                    XArmConfig.load(path)
