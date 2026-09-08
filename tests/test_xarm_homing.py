import contextlib
import io
import json
import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from spacemouse_teleop.backends.xarm_sdk.backend import XArmBackend
from spacemouse_teleop.backends.xarm_sdk.config import XArmConfig, DEFAULT_CONFIG
from spacemouse_teleop.backends.xarm_sdk.homing import JointHomer
from spacemouse_teleop.backends.xarm_sdk.safety import TeleopFault, check_homing_site
from spacemouse_teleop.cli.xarm_teleop import run_loop
from test_xarm_config_safety import feedback, live_config
from test_input_safety import raw


def home_config():
    config = live_config()
    config.gripper['enabled'] = False
    config.robot['homing']['enabled'] = True
    return config


class HomingTest(unittest.TestCase):
    def rig(self, stuck=False, report_period=0., stop_reports_after=None):
        config = home_config()
        class World:
            now, sequence, state, mode = 10., 0, 4, 0
            joints = [.12, -.07, .03, 0, -.05, .09]
            speeds = [0.] * 6
            calls = []
            stale = False
            cached = None
            lease_expires_at = 10.
            def monotonic(self): return self.now
            def sleep(self, seconds):
                seconds = max(seconds, .00001)
                if not stuck:
                    active_seconds = min(seconds, max(0., self.lease_expires_at - self.now))
                    self.joints = [p + v * active_seconds for p, v in zip(self.joints, self.speeds)]
                self.now += seconds
                self.sequence += 1
            def snapshot(self):
                due = self.cached is None or self.now - self.cached.received_at >= report_period
                available = stop_reports_after is None or self.now - 10. < stop_reports_after
                if self.cached is None or (due and available):
                    self.cached = feedback(self.now - (1 if self.stale else 0), sequence=self.sequence,
                                           state=self.state, mode=self.mode, joints_rad=tuple(self.joints))
                return self.cached
            def close(self): pass
        world = World()
        class Channel:
            def activate_homing(self):
                world.calls.append('home_mode')
                world.mode, world.state = 4, 1
                world.sequence += 1
            def activate(self, acc):
                world.calls.append('teleop_mode')
                world.mode, world.state = 5, 2
                world.sequence += 1
            def joint_velocity(self, speeds, duration):
                world.calls.append(('joint', tuple(speeds), duration))
                world.speeds = list(speeds)
                world.lease_expires_at = world.now + duration
            def stop(self):
                world.calls.append('stop')
                world.speeds = [0.] * 6
                world.state = 4
                world.sequence += 1
                return []
            def close(self): pass
        backend = XArmBackend(config, channel_factory=lambda *a, **kw: Channel())
        backend.channel, backend.reports = Channel(), world
        rows = []
        recorder = type('Recorder', (), {'write': lambda self, row: rows.append(row)})()
        return config, world, backend, recorder, rows

    def test_home_reaches_target_settles_stops_then_allows_teleop(self):
        config, world, backend, recorder, rows = self.rig()
        with patch('spacemouse_teleop.backends.xarm_sdk.backend.time', world):
            backend.home(recorder, lambda: None)
            self.assertFalse(backend.armed)
            self.assertEqual(world.state, 4)
            self.assertTrue(all(abs(p) <= .005 for p in world.joints))
            self.assertEqual(rows[-1]['event'], 'homing_completed')
            packets = [c for c in world.calls if isinstance(c, tuple)]
            self.assertTrue(any(any(c[1]) for c in packets))
            self.assertTrue(all(0 < c[2] <= .15 + 1e-10 for c in packets))
            self.assertTrue(all(max(map(abs, c[1])) <= .1 for c in packets))
            self.assertNotIn('teleop_mode', world.calls)
            backend.activate()
            self.assertEqual(world.calls[-1], 'teleop_mode')
            backend.close()

    def test_far_start_can_home_beyond_30_seconds_with_slow_reports(self):
        config, world, backend, recorder, rows = self.rig(report_period=.205)
        world.joints = [3.2, 0., 0., 0., 0., 0.]
        started = world.now
        with patch('spacemouse_teleop.backends.xarm_sdk.backend.time', world):
            backend.home(recorder, lambda: None)
        self.assertGreater(world.now - started, 30.)
        self.assertLess(world.now - started, config.robot['homing']['timeout_s'])
        self.assertEqual(rows[-1]['event'], 'homing_completed')
        self.assertTrue(any(r['event'] == 'homing_waiting_feedback' for r in rows))
        self.assertTrue(all(abs(p) <= .005 for p in world.joints))
        self.assertNotIn('teleop_mode', world.calls)
        self.assertTrue(all(0 < c[2] <= .15 + 1e-10 for c in world.calls if isinstance(c, tuple)))

    def test_waiting_for_report_still_stops_on_real_feedback_timeout(self):
        config, world, backend, recorder, rows = self.rig(report_period=.205, stop_reports_after=.5)
        with patch('spacemouse_teleop.backends.xarm_sdk.backend.time', world):
            with self.assertRaisesRegex(TeleopFault, 'feedback missing or stale'):
                backend.home(recorder, lambda: None)
        self.assertTrue(any(r['event'] == 'homing_waiting_feedback' for r in rows))
        self.assertLess(world.now - world.cached.received_at, config.robot['feedback_timeout_s'] + .02)
        self.assertEqual(world.calls[-1], 'stop')
        self.assertNotIn('teleop_mode', world.calls)

    def test_homing_timeout_stops_without_entering_teleop(self):
        config, world, backend, recorder, rows = self.rig(stuck=True)
        config.robot['homing']['timeout_s'] = .4
        with patch('spacemouse_teleop.backends.xarm_sdk.backend.time', world):
            with self.assertRaisesRegex(TeleopFault, 'timed out'):
                backend.home(recorder, lambda: None)
        self.assertEqual(world.calls[-1], 'stop')
        self.assertNotIn('teleop_mode', world.calls)
        self.assertFalse(any(r['event']=='homing_completed' for r in rows))

    def test_keyboard_interrupt_or_input_fault_stops_homing(self):
        for error in (KeyboardInterrupt(), TeleopFault('input disconnected')):
            config, world, backend, recorder, rows = self.rig()
            def guard():
                if world.now > 10.04:
                    raise error
            with patch('spacemouse_teleop.backends.xarm_sdk.backend.time', world):
                with self.assertRaises(type(error)):
                    backend.home(recorder, guard)
            self.assertEqual(world.calls[-1], 'stop')
            self.assertNotIn('teleop_mode', world.calls)

    def test_stale_feedback_stops_homing(self):
        config, world, backend, recorder, rows = self.rig()
        def guard():
            if world.now > 10.04:
                world.stale = True
        with patch('spacemouse_teleop.backends.xarm_sdk.backend.time', world):
            with self.assertRaisesRegex(TeleopFault, 'stale'):
                backend.home(recorder, guard)
        self.assertEqual(world.calls[-1], 'stop')

    def test_home_requires_matching_tool_and_stays_in_workspace(self):
        for bad in (feedback(tcp_mm_rad=(101, 0, 0, 0, 0, 0)),
                    feedback(tcp_offset_mm_rad=(10,) * 6),
                    feedback(payload_kg_com_mm=(2., 0, 0, 0))):
            with self.assertRaises(TeleopFault):
                check_homing_site(home_config(), bad)
        check_homing_site(home_config(), feedback(joints_rad=(.2,) * 6))

    def test_joint_speed_ramp_and_lease_stop_before_target(self):
        controller = JointHomer(home_config())
        first, done = controller.step(feedback(joints_rad=(1.,) * 6), 10.)
        self.assertFalse(done)
        self.assertAlmostEqual(first[0], -.2 / 60)
        near, done = controller.step(feedback(10.02, joints_rad=(.006,) * 6), 10.02)
        self.assertGreaterEqual(.006 + near[0] * .2, 0.)
        stopped, done = controller.step(feedback(10.04, joints_rad=(.004,) * 6), 10.04)
        self.assertTrue(done)
        self.assertEqual(stopped, (0.,) * 6)

    def test_invalid_homing_config_is_rejected(self):
        for key, value in (('speed_radps', float('nan')), ('timeout_s', 0), ('settle_s', .01)):
            data = json.loads(DEFAULT_CONFIG.read_text())
            data['robot']['homing'][key] = value
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'config.json'
                path.write_text(json.dumps(data))
                with self.assertRaises(ValueError): XArmConfig.load(path)


class HomingSequenceTest(unittest.TestCase):
    def test_homing_finishes_before_a_new_neutral_check_and_teleop(self):
        config = home_config()
        calls = []
        class Source:
            count = 0
            def snapshot(self, now):
                self.count += 1
                calls.append(('input', self.count))
                return raw(now, 1. if self.count < 3 else 0.), 'hold'
        source = Source()
        class Backend:
            gripper = None
            def home(self, recorder, guard):
                calls.append('home_start')
                guard()
                calls.append('home_complete')
            def activate(self): calls.append('teleop')
            def feedback(self):
                import time
                return feedback(time.monotonic())
            def send(self, *args): return {}
        recorder = type('Recorder', (), {'write': lambda self, row: None})()
        with contextlib.redirect_stdout(io.StringIO()):
            run_loop(config, source, Backend(), recorder, duration=.09)
        self.assertLess(calls.index('home_complete'), calls.index(('input', 3)))
        self.assertLess(calls.index(('input', 3)), calls.index('teleop'))
