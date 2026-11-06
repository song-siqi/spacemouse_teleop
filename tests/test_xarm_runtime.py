import contextlib
import io
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from robot_teleop.backends.real.xarm_sdk.backend import GripperWorker, XArmBackend
from robot_teleop.backends.real.xarm_sdk.safety import TeleopFault
from robot_teleop.cli.xarm_teleop import main, run_loop, process_input
from robot_teleop.recording.async_jsonl import AsyncJsonlRecorder
from test_xarm_config_safety import feedback, live_config
from test_input_safety import raw


class FakeChannel:
    def __init__(self):
        self.targets = []
        self.calls = []
        self.closed = False
        self.error_code = 0
        self.position = 100  # Obstructed gripper need not reach commanded endpoint.
    def gripper_target(self, pulse): self.targets.append(pulse)
    def gripper_status(self): return {'position_pulse': self.position, 'error_code': self.error_code, 'received_at': time.monotonic()}
    def velocity(self, speeds, duration): self.calls.append(('velocity', tuple(speeds), duration))
    def stop(self): self.calls.append(('stop',)); return []
    def close(self): self.closed = True


class GripperTest(unittest.TestCase):
    def setUp(self):
        self.config = live_config()
        self.channel = FakeChannel()
        self.worker = GripperWorker(self.channel, self.config.gripper, clock=lambda: 1.)

    def test_held_endpoint_is_not_rewritten_and_release_holds_current_position(self):
        self.worker.submit('close', 2.)
        self.worker.tick()
        self.channel.position = 94
        self.worker.tick()  # A held button remains active without a new edge.
        self.channel.position = 89
        self.worker.submit('hold', 2.)
        self.worker.tick()
        self.worker.tick()
        self.assertEqual(self.channel.targets, [0, 89])
        self.assertEqual(self.worker.snapshot()["target_sequence"], 2)
        self.assertEqual(self.worker.snapshot()['position_pulse'], 89)
        self.assertEqual(self.worker.snapshot()['target_pulse'], 89)

    def test_obstruction_and_renewed_held_input_do_not_rewrite_endpoint(self):
        self.worker.submit('close', 2.)
        for _ in range(10):
            self.worker.submit("close", 2.)
            self.worker.tick()
        self.assertEqual(self.channel.targets, [0])
        self.assertEqual(self.worker.snapshot()["target_sequence"], 1)
        self.worker.submit('hold', 2.)
        self.worker.tick()
        self.assertEqual(self.channel.targets[-1], 100)

    def test_latest_pending_goal_wins_and_reverse_is_immediate_next_tick(self):
        self.worker.submit('close', 2.)
        self.worker.submit('open', 2.)
        self.worker.tick()
        self.worker.submit('close', 2.)
        self.worker.tick()
        self.assertEqual(self.channel.targets, [850, 0])

    def test_release_or_reverse_during_feedback_read_prevents_stale_target(self):
        self.worker.submit('close', 2.)
        self.worker.tick()
        read = self.channel.gripper_status
        for intent, expected in (('hold', 100), ('open', 850)):
            self.worker.submit('close', 2.)
            def update():
                self.worker.submit(intent, 2.)
                return read()
            self.channel.gripper_status = update
            self.worker.tick()
            self.assertEqual(self.channel.targets[-1], expected)

    def test_endpoint_reached_does_not_rewrite_goal_and_retreat_is_allowed(self):
        self.worker.config.update(close_pulse=230, open_pulse=850)
        self.channel.position = 234
        self.worker.submit('close', 2.)
        self.worker.tick()
        self.assertEqual(self.channel.targets[-1], 230)
        self.channel.position = 230
        self.worker.tick()
        self.worker.tick()
        self.assertEqual(self.channel.targets, [230])
        # Sensor obstruction may have left the gripper below the new limit.
        self.channel.position = 208
        self.worker.submit('open', 2.)
        self.worker.tick()
        self.assertEqual(self.channel.targets, [230, 850])
        self.channel.position = 847
        self.worker.tick()
        self.assertEqual(self.channel.targets[-1], 850)

    def test_release_then_repress_sends_same_endpoint_again(self):
        self.worker.submit('open', 2.)
        self.worker.tick()
        self.channel.position = 400
        self.worker.submit('hold', 2.)
        self.worker.tick()
        self.worker.submit('open', 2.)
        for _ in range(5):
            self.worker.tick()
        self.assertEqual(self.channel.targets, [850, 400, 850])
        self.assertEqual(self.worker.snapshot()['target_sequence'], 3)

    def test_new_close_below_limit_cancels_inflight_open_goal(self):
        self.worker.config['close_pulse'] = 230
        self.channel.position = 208
        self.worker.submit('open', 2.)
        self.worker.tick()
        self.worker.submit('close', 2.)
        self.worker.tick()
        self.worker.tick()
        self.assertEqual(self.channel.targets, [850, 208])

    def test_release_replaces_inflight_target_without_waiting_for_completion(self):
        # Model a device that accepts a target immediately and moves later.
        self.channel.device_target = 100
        def accept(pulse):
            self.channel.device_target = pulse
            self.channel.targets.append(pulse)
        self.channel.gripper_target = accept
        self.worker.submit('open', 2.)
        self.worker.tick()
        self.assertEqual(self.channel.device_target, 850)
        self.channel.position = 300  # Still far from the open goal.
        self.worker.submit('hold', 2.)
        self.worker.tick()
        self.assertEqual(self.channel.device_target, 300)
        self.assertEqual(self.channel.targets, [850, 300])

    def test_initial_close_below_limit_cancels_old_goal_without_closing_further(self):
        self.worker.config['close_pulse'] = 230
        self.channel.position = 208
        self.worker.submit('close', 2.)
        self.worker.tick()
        self.assertEqual(self.channel.targets, [208])

    def test_expired_goal_and_error_are_faults(self):
        self.worker.submit('close', .5)
        with self.assertRaises(TeleopFault): self.worker.tick()
        self.assertEqual(self.channel.targets, [])
        self.channel.error_code = 7
        with self.assertRaises(TeleopFault): self.worker.tick()

    def test_short_fresh_lease_defers_target_until_renewed(self):
        # Real incident: 58 ms left on an arm-feedback-capped lease, while
        # gripper I/O requires 100 ms. The button itself is freshly pressed.
        self.worker.config['socket_timeout_s'] = .1
        self.worker.submit('open', 1.058)
        self.worker.tick()
        self.assertEqual(self.channel.targets, [])
        self.assertTrue(self.worker.snapshot()['waiting_for_lease'])
        self.worker.submit('open', 1.2)
        self.worker.tick()
        self.worker.tick()
        self.assertEqual(self.channel.targets, [850])
        self.assertFalse(self.worker.snapshot()['waiting_for_lease'])

    def test_release_after_deferred_target_does_not_open_later(self):
        self.worker.config['socket_timeout_s'] = .1
        self.worker.submit('open', 1.058)
        self.worker.tick()
        self.worker.submit('hold', 1.2)
        self.worker.tick()
        self.assertEqual(self.channel.targets, [])
        self.assertFalse(self.worker.snapshot()['waiting_for_lease'])

    def test_reverse_with_short_lease_holds_before_retrying(self):
        self.worker.config['socket_timeout_s'] = .1
        self.worker.submit('close', 1.2)
        self.worker.tick()
        self.channel.position = 90
        self.worker.submit('open', 1.058)
        self.worker.tick()
        self.assertEqual(self.channel.targets, [0, 90])
        self.worker.submit('open', 1.2)
        self.worker.tick()
        self.assertEqual(self.channel.targets, [0, 90, 850])

    def test_deferred_target_still_faults_if_lease_really_expires(self):
        now = [1.]
        self.worker.clock = lambda: now[0]
        self.worker.config['socket_timeout_s'] = .1
        self.worker.submit('open', 1.058)
        self.worker.tick()
        now[0] = 1.06
        with self.assertRaisesRegex(TeleopFault, 'expired'):
            self.worker.tick()
        self.assertEqual(self.channel.targets, [])

    def test_expiry_while_jogging_holds_current_position_before_fault(self):
        self.worker.submit('close', 2.)
        self.worker.tick()
        self.channel.position = 94
        self.worker.submit('close', .5)
        with self.assertRaisesRegex(TeleopFault, 'expired'):
            self.worker.tick()
        self.assertEqual(self.channel.targets, [0, 94])

    def test_startup_and_unused_exit_do_not_command_positions(self):
        self.worker.tick()
        self.worker.submit('close', 2.)
        self.worker.close()
        self.worker.tick()
        self.assertEqual(self.channel.targets, [])
        self.assertTrue(self.channel.closed)

    def test_exit_during_motion_holds_current_position_without_release(self):
        self.worker.submit('close', 2.)
        self.worker.tick()
        self.channel.position = 96
        self.worker.close()
        self.assertEqual(self.channel.targets, [0, 96])
        self.assertTrue(self.channel.closed)

    def test_exit_hold_failure_is_observable(self):
        self.worker.submit('close', 2.)
        self.worker.tick()
        self.channel.gripper_target = lambda value: (_ for _ in ()).throw(TimeoutError('lost connection'))
        with self.assertRaisesRegex(TeleopFault, 'lost connection'):
            self.worker.close()
        self.assertTrue(self.channel.closed)

    def test_slow_gripper_does_not_hold_arm_channel(self):
        entered, release = threading.Event(), threading.Event()
        def slow_target(pulse):
            entered.set()
            release.wait(1.)
        self.channel.gripper_target = slow_target
        worker = GripperWorker(self.channel, self.config.gripper)
        arm = FakeChannel()
        try:
            worker.submit('close', time.monotonic() + .5)
            worker.start()
            self.assertTrue(entered.wait(.5))
            arm.velocity((1, 0, 0, 0, 0, 0), .2)
            self.assertEqual(len(arm.calls), 1)
            self.assertFalse(release.is_set())
        finally:
            release.set()
            worker.close()


class BackendTest(unittest.TestCase):
    def make_backend(self):
        config = live_config()
        config.gripper['enabled'] = False
        channel = FakeChannel()
        backend = XArmBackend(config)
        backend.channel, backend.armed = channel, True
        backend.reports = type('Reports', (), {'snapshot': lambda self: feedback(time.monotonic()), 'close': lambda self: None})()
        return backend, channel

    def test_feedback_capped_arm_lease_does_not_fault_fresh_gripper_press(self):
        backend, arm = self.make_backend()
        self.addCleanup(backend.close)
        backend.config.robot.update(feedback_timeout_s=.25, socket_timeout_s=.05)
        backend.config.gripper['socket_timeout_s'] = .1
        now = [10.]
        tool = FakeChannel()
        backend.gripper = GripperWorker(tool, backend.config.gripper, clock=lambda:now[0])
        backend.reports.snapshot = lambda:feedback(9.808)
        with patch('robot_teleop.backends.real.xarm_sdk.backend.time.monotonic',lambda:now[0]):
            sent = backend.send((0.,)*6, 10.2, 'open')
            self.assertAlmostEqual(sent['duration_s'], .008)
            backend.gripper.tick()
            self.assertTrue(backend.gripper.snapshot()['waiting_for_lease'])
            self.assertEqual(tool.targets, [])
            now[0] = 10.02
            backend.reports.snapshot = lambda:feedback(now[0])
            backend.send((0.,)*6, 10.22, 'open')
            backend.gripper.tick()
            self.assertEqual(tool.targets, [850])
            self.assertEqual(len(arm.calls), 2)

    def test_expired_send_never_reaches_sdk(self):
        backend, channel = self.make_backend()
        with self.assertRaises(TeleopFault):
            backend.send((1, 0, 0, 0, 0, 0), time.monotonic() + .001)
        self.assertEqual(channel.calls, [])
        backend.close()
        self.assertEqual(channel.calls, [('stop',)])

    def test_send_uses_bounded_duration_and_cleanup_stops_once(self):
        backend, channel = self.make_backend()
        sent = backend.send((1, 0, 0, 0, 0, 0), time.monotonic() + .2)
        self.assertGreater(sent['duration_s'], 0)
        self.assertLessEqual(sent['duration_s'], .15)
        backend.close()
        backend.close()
        self.assertEqual(sum(c[0] == 'stop' for c in channel.calls), 1)
        self.assertTrue(channel.closed)

    def test_stale_feedback_and_changed_tcp_block_send(self):
        for value in (feedback(time.monotonic() - 1), feedback(time.monotonic(), tcp_offset_mm_rad=(10,) * 6)):
            backend, channel = self.make_backend()
            backend.reports.snapshot = lambda: value
            with self.assertRaises(TeleopFault):
                backend.send((1, 0, 0, 0, 0, 0), time.monotonic() + .2)
            self.assertEqual(channel.calls, [])
            backend.close()

    def test_mode_changed_startup_enters_mode_five_after_site_check(self):
        config = live_config()
        config.gripper['enabled'] = False
        old, live = FakeChannel(), FakeChannel()
        reports = type('Reports', (), {'snapshot': lambda self: self.value, 'close': lambda self: None})()
        reports.value = feedback(time.monotonic(), state=5, mode=0)
        def activate(acc):
            live.calls.append(('activate', acc))
            reports.value = feedback(time.monotonic(), sequence=2, state=2, mode=5)
        live.activate = activate
        backend = XArmBackend(config, channel_factory=lambda *a, **kw: live)
        backend.channel, backend.reports = old, reports
        backend.activate()
        self.assertTrue(backend.armed)
        self.assertEqual(live.calls[0], ('activate', .3))
        backend.close()
        self.assertIn(('stop',), live.calls)

    def test_activation_failure_still_sends_stop(self):
        config = live_config()
        config.gripper['enabled'] = False
        read_channel, write_channel = FakeChannel(), FakeChannel()
        write_channel.activate = lambda acc: (_ for _ in ()).throw(TeleopFault('partial enable failure'))
        backend = XArmBackend(config, channel_factory=lambda *args, **kwargs: write_channel)
        backend.channel = read_channel
        backend.reports = type('Reports', (), {'snapshot': lambda self: feedback(time.monotonic()), 'close': lambda self: None})()
        with self.assertRaises(TeleopFault): backend.activate()
        self.assertIn(('stop',), write_channel.calls)
        backend.close()
        self.assertTrue(read_channel.closed and write_channel.closed)

    def test_cleanup_closes_all_resources_even_when_stop_fails(self):
        backend, channel = self.make_backend()
        channel.stop = lambda: (_ for _ in ()).throw(TeleopFault('lost network'))
        errors = backend.close()
        self.assertTrue(errors)
        self.assertTrue(channel.closed)
        self.assertIsNone(backend.reports)


class CliTest(unittest.TestCase):
    def test_missing_site_settings_fail_before_hid_or_network(self):
        with patch('robot_teleop.cli.xarm_teleop.LatestInput') as input_factory, patch('socket.create_connection') as connect:
            with contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(main(['--duration', '.1']), 1)
            input_factory.assert_not_called()
            connect.assert_not_called()

    def test_dry_run_works_without_any_robot_connection(self):
        with tempfile.TemporaryDirectory() as directory, patch('socket.create_connection', side_effect=AssertionError('network forbidden')):
            path = Path(directory) / 'preview.jsonl'
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main(['--dry-run', '--backend', 'mock', '--duration', '.3', '--log', str(path)]), 0)
            import json
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            previews = [row for row in rows if row.get('mode') == 'xarm-sdk-dry-run']
            self.assertTrue(previews)
            self.assertTrue(any(any(row['execution']['would_send_mm_s_rad_s']) for row in previews))
            self.assertEqual(rows[-1]['event'], 'session_end')

    def test_live_loop_waits_for_neutral_before_activation(self):
        config = live_config()
        class Source:
            calls = 0
            def snapshot(self, now):
                self.calls += 1
                return raw(now, 1. if self.calls != 2 else 0.), 'hold'
        source = Source()
        class Backend:
            gripper = None
            activation_at = None
            sends = 0
            def activate(self): self.activation_at = source.calls
            def feedback(self): return feedback(time.monotonic())
            def send(self, *args): self.sends += 1; return {}
        backend = Backend()
        recorder = type('Recorder', (), {'write': lambda self, row: None})()
        with contextlib.redirect_stdout(io.StringIO()):
            run_loop(config, source, backend, recorder, duration=.08)
        self.assertEqual(backend.activation_at, 2)
        self.assertGreater(backend.sends, 0)

    def test_fault_and_keyboard_interrupt_always_close_backend_first(self):
        config = live_config()
        config.gripper['enabled'] = False
        for failure in (TeleopFault('input unplugged'), KeyboardInterrupt()):
            calls = []
            class Backend:
                def __init__(self, config): pass
                def connect(self): pass
                def close(self): calls.append('robot_stop'); return []
            class Input:
                def __init__(self, *args, **kwargs): pass
                def start(self): pass
                def close(self): calls.append('input_close')
            with tempfile.TemporaryDirectory() as directory:
                with patch('robot_teleop.cli.xarm_teleop.XArmConfig.load', return_value=config), patch('robot_teleop.cli.xarm_teleop.XArmBackend', Backend), patch('robot_teleop.cli.xarm_teleop.LatestInput', Input), patch('robot_teleop.cli.xarm_teleop.run_loop', side_effect=failure):
                    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                        main(['--log', str(Path(directory) / 'log.jsonl')])
            self.assertEqual(calls, ['robot_stop', 'input_close'])

    def test_log_overflow_and_disk_error_are_observable(self):
        recorder = AsyncJsonlRecorder('/unused', capacity=1)
        recorder.write({'event': 'one'})
        with self.assertRaisesRegex(RuntimeError, 'queue full'):
            recorder.write({'event': 'two'})
        recorder.error = OSError('disk full')
        with self.assertRaisesRegex(RuntimeError, 'disk full'):
            recorder.write({})

class GripperFreshnessTest(unittest.TestCase):
    def test_polling_hang_is_detected_without_waiting_for_thread(self):
        now = [1.]
        worker = GripperWorker(FakeChannel(), live_config().gripper, clock=lambda: now[0])
        worker.tick()
        now[0] = 2.
        with self.assertRaisesRegex(TeleopFault, 'stalled'):
            worker.snapshot()


class NeutralResumeTest(unittest.TestCase):
    def test_reverse_after_neutral_cannot_revive_previous_velocity(self):
        from robot_teleop.inputs.spacemouse import TeleopCore
        config = live_config()
        core = TeleopCore(config.teleop)
        self.assertGreater(process_input(core, raw(1., 1.), 'hold', config).linear_vel_mps[0], 0)
        stopped = process_input(core, raw(1.02), 'close', config)
        self.assertEqual(stopped.linear_vel_mps, (0., 0., 0.))
        self.assertEqual(stopped.gripper_intent, 'close')
        self.assertLess(process_input(core, raw(1.04, -.1), 'hold', config).linear_vel_mps[0], 0)
