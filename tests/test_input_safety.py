import time
import types
import unittest
from dataclasses import replace
from unittest.mock import patch

from robot_teleop.inputs.spacemouse import TeleopConfig, TeleopCore
from robot_teleop.inputs.spacemouse.command import RawSpaceMouseState
from robot_teleop.inputs.spacemouse.readers import PySpaceMouseReader, _HIDReportMonitor
from robot_teleop.backends.real.xarm_sdk.config import XArmConfig
from robot_teleop.inputs.spacemouse.polling import LatestInput, input_deadline
from robot_teleop.backends.real.xarm_sdk.safety import TeleopFault


def raw(t, x=0., buttons=(0, 0)):
    return RawSpaceMouseState(x, 0, 0, 0, 0, 0, buttons, t, t, int(t * 1000), True, (t,) * 6)


class InputSafetyTest(unittest.TestCase):
    def assert_zero(self, command):
        self.assertFalse(command.enabled)
        for name in ('linear_vel_mps', 'angular_vel_radps', 'delta_pos_m', 'delta_rot_rad'):
            self.assertEqual(getattr(command, name), (0.,) * 3)
        self.assertEqual(command.gripper_intent, 'hold')

    def test_deadman_release_clears_existing_filter_and_reenable_has_no_tail(self):
        core = TeleopCore(TeleopConfig(require_deadman=True))
        self.assertGreater(core.process(raw(1., 1., (1, 0))).linear_vel_mps[0], 0)
        self.assert_zero(core.process(raw(1.02, 1., (0, 0))))
        self.assertEqual(core.process(raw(1.04, 0., (1, 0))).linear_vel_mps, (0.,) * 3)

    def test_timeout_zeroes_velocity_not_only_delta(self):
        core = TeleopCore()
        core.process(raw(1., 1.))
        self.assert_zero(core.process(raw(2., 1.)))
        self.assertEqual(core.process(raw(2.01)).linear_vel_mps, (0.,) * 3)

    def test_bad_axes_and_time_are_fail_closed(self):
        for bad in (float('nan'), float('inf'), -float('inf')):
            core = TeleopCore()
            core.process(raw(1, 1))
            self.assert_zero(core.process(raw(1.01, bad)))
            self.assert_zero(core.process(replace(raw(1., 1), timestamp=bad)))
        for t in (1., .9):
            core = TeleopCore()
            core.process(raw(1., 1))
            self.assert_zero(core.process(raw(t, 1)))

    def test_disconnection_and_stale_axis_cannot_be_refreshed_by_buttons(self):
        core = TeleopCore()
        self.assert_zero(core.process(replace(raw(1, 1), connected=False)))
        core = TeleopCore()
        state = replace(raw(2, 1, (1, 0)), axis_report_timestamps=(1., 2., 2., 2., 2., 2.))
        self.assert_zero(core.process(state))
        with self.assertRaises(TeleopFault):
            input_deadline(state, 2., XArmConfig.load(), 'open')

    def test_old_idle_report_and_held_button_do_not_require_repeated_packets(self):
        config = XArmConfig.load()
        state = replace(raw(1, buttons=(0, 1)), timestamp=100)
        self.assertAlmostEqual(input_deadline(state, 100, config), 100.2)
        self.assertAlmostEqual(input_deadline(state, 100, config, 'close'), 100.2)

    def test_nonzero_lease_uses_oldest_active_axis_report(self):
        config = XArmConfig.load()
        state = replace(raw(1, 1), timestamp=1.1)
        self.assertAlmostEqual(input_deadline(state, 1.1, config), 1.25)
        with self.assertRaises(TeleopFault):
            input_deadline(state, 1.26, config)

    def test_reader_preserves_cached_report_timestamp(self):
        state = types.SimpleNamespace(x=1, y=0, z=0, roll=0, pitch=0, yaw=0, buttons=(0, 0), t=10.)
        reader = PySpaceMouseReader()
        reader._pyspacemouse = types.SimpleNamespace(read=lambda: state)
        with patch('robot_teleop.inputs.spacemouse.readers.time.monotonic', side_effect=[10., 11., 12.]):
            a, b = reader.read(), reader.read()
            state.t = 12.
            c = reader.read()
        self.assertEqual((a.report_timestamp, b.report_timestamp), (10., 10.))
        self.assertEqual(a.report_sequence, b.report_sequence)
        self.assertGreater(c.report_sequence, b.report_sequence)
        self.assertNotEqual(a.timestamp, b.timestamp)

    def test_channel_monitor_refreshes_identical_motion_but_not_button_packets(self):
        reports = iter((bytes([1]), bytes([3]), bytes([1]), bytes([2])))
        device = types.SimpleNamespace(read=lambda: next(reports))
        mappings = {name: types.SimpleNamespace(channel=1 if i < 3 else 2) for i, name in enumerate(('x', 'y', 'z', 'roll', 'pitch', 'yaw'))}
        timestamps = [None] * 6
        monitor = _HIDReportMonitor(device, mappings, timestamps)
        with patch('robot_teleop.inputs.spacemouse.readers.time.monotonic', side_effect=[1., 2., 3., 4.]):
            monitor.read()
            monitor.read()
            self.assertEqual(timestamps, [1., 1., 1., None, None, None])
            monitor.read()
            monitor.read()
        self.assertEqual(timestamps, [3., 3., 3., 4., 4., 4.])

    def test_released_click_in_same_drain_cannot_trigger_delayed_motion(self):
        import threading
        done = threading.Event()
        now = time.monotonic()
        states = [raw(now), replace(raw(now, buttons=(0, 1)), report_sequence=2), replace(raw(now), report_sequence=3)]
        class Reader:
            _pyspacemouse = types.SimpleNamespace(AxisConvention=object())
            def __enter__(self): return self
            def __exit__(self, *args): pass
            def read(self):
                if states:
                    self.last = states.pop(0)
                else:
                    done.set()
                return self.last
        source = LatestInput(XArmConfig.load(), reader=Reader())
        try:
            source.start()
            self.assertTrue(done.wait(.5))
            value, intent = source.snapshot(time.monotonic())
            self.assertEqual(value.buttons, (0, 0))
            self.assertEqual(intent, 'hold')
            self.assertEqual(source.snapshot(time.monotonic())[1], 'hold')
        finally:
            source.close()

    def test_polling_stall_is_detected_even_with_zero_axes(self):
        source = LatestInput(XArmConfig.load())
        source.raw, source.polled_at = raw(1.), 1.
        with self.assertRaisesRegex(TeleopFault, 'polling stopped'):
            source.snapshot(2.)


class GripperHeldButtonTest(unittest.TestCase):
    def setUp(self):
        self.source = LatestInput(XArmConfig.load())
        self.source.polled_at = 1.

    def intent(self, buttons):
        self.source.raw = raw(1., buttons=buttons)
        return self.source.snapshot(1.)[1]

    def test_held_button_repeats_until_release_and_reverse(self):
        for _ in range(3):
            self.assertEqual(self.intent((1, 0)), 'open')
        self.assertEqual(self.intent((0, 0)), 'hold')
        for _ in range(3):
            self.assertEqual(self.intent((0, 1)), 'close')
        self.assertEqual(self.intent((1, 0)), 'open')
        self.assertEqual(self.intent((0, 0)), 'hold')

    def test_both_buttons_hold_and_remaining_held_button_resumes(self):
        for buttons, expected in (((1, 0), 'open'), ((1, 1), 'hold'),
                                  ((0, 1), 'close'), ((1, 1), 'hold'),
                                  ((1, 0), 'open'), ((0, 0), 'hold')):
            self.assertEqual(self.intent(buttons), expected)

    def test_disconnect_or_stalled_polling_cannot_keep_button_active(self):
        self.source.raw = replace(raw(1., buttons=(0, 1)), connected=False)
        with self.assertRaisesRegex(TeleopFault, 'disconnected'):
            self.source.snapshot(1.)
        self.source.raw = raw(1., buttons=(0, 1))
        with self.assertRaisesRegex(TeleopFault, 'polling stopped'):
            self.source.snapshot(2.)
