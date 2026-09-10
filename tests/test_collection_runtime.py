"""Collection orchestration regressions; all motion and source I/O are fake."""
import contextlib
import io
import queue
import threading
import types
import unittest
from unittest.mock import patch

from spacemouse_teleop.backends.xarm6_camera.backend import XArmCameraBackend
from spacemouse_teleop.backends.xarm6_camera.coordinator import CollectionCoordinator
from spacemouse_teleop.cli.xarm_teleop import run_loop
from test_input_safety import raw
from test_xarm_config_safety import feedback, live_config


class FakeClock:
    def __init__(self): self.now = 100.
    def monotonic(self): return self.now
    def sleep(self, duration): self.now += max(duration, .001)
    def advance(self, duration=.4): self.now += duration


class EpisodeSink:
    """Tiny recorder state machine; no fake API can command a robot."""
    def __init__(self):
        self.state = 'idle'
        self.calls, self.rows, self.samples = [], [], []
        self.complete = False
        self.healthy = True
        self.error = None
    def status(self): return {'state': self.state, 'complete': self.complete, 'error': self.error}
    def write(self, row): self.rows.append(row)
    def publish_state(self, sample): self.samples.append(sample)
    def start_episode(self):
        self.calls.append('start')
        if not self.healthy or self.state != 'idle': return None
        self.state, self.complete = 'recording', False
        return 'episode'
    def stop_episode(self, reason='user', incomplete=False):
        self.calls.append(('stop', reason))
        self.state, self.complete = 'review', not incomplete
    def disposition(self, value):
        self.calls.append(('disposition', value))
        if self.state not in ('review', 'fault'): return False
        self.state = 'idle'
        return True
    def mark_fault(self, reason):
        self.error = str(reason)
        if self.state == 'recording':
            self.state, self.complete = 'fault', False


class MotionHarness:
    def __init__(self, sink=None, threaded=False):
        self.sink = sink or EpisodeSink()
        self.events, self.instances = [], []
        self.clock = FakeClock()
        self.threaded = threaded
        self.wrapper = XArmCameraBackend(live_config(), self.sink,
                                         backend_factory=self.make_backend,
                                         channel_factory=self.make_channel)

    def make_channel(self, *args, **kwargs):
        harness = self
        class Channel:
            def gripper_status(self):
                return {'position_pulse': 400, 'controller_target_pulse': 850,
                        'error_code': 0, 'received_at': harness.clock.monotonic()}
        return Channel()

    def make_backend(self, config, channel_factory):
        harness, index = self, len(self.instances)
        self.events.append(('construct', index))
        class Motion:
            gripper = None
            startup_feedback = None
            closed = False
            report_sequence = 1
            close_errors = None
            worker = None
            def __init__(self): self.stop_event = threading.Event()
            def connect(self):
                harness.events.append(('connect', index))
                self.channel = channel_factory('fake', .05, writable=False)
                if harness.threaded:
                    self.worker = threading.Thread(target=lambda: self.stop_event.wait(), daemon=True)
                    self.worker.start()
            def home(self, recorder, input_guard=None):
                harness.events.append(('home', index))
                if input_guard: input_guard()
                self.channel.gripper_status()
            def activate(self): harness.events.append(('activate', index))
            def feedback(self): return feedback(harness.clock.monotonic(), sequence=self.report_sequence)
            def send(self, *args, **kwargs):
                harness.events.append(('send', index))
                return {'sent': args}
            def close(self):
                harness.events.append(('close_begin', index))
                self.channel.gripper_status()
                self.stop_event.set()
                if self.worker: self.worker.join(.3)
                self.closed = True
                harness.events.append(('close_end', index))
                return self.close_errors or []
        motion = Motion()
        self.instances.append(motion)
        return motion

    def close(self):
        self.wrapper.close()
        for motion in self.instances:
            motion.stop_event.set()
            if motion.worker: motion.worker.join(.3)


class CompositionLifecycleTests(unittest.TestCase):
    def make(self, threaded=False):
        harness = MotionHarness(threaded=threaded)
        self.addCleanup(harness.close)
        harness.wrapper.connect()
        return harness

    def test_repeated_prepare_closes_and_joins_before_new_connect_and_home(self):
        harness = self.make(threaded=True)
        for _ in range(3): harness.wrapper.home(harness.sink)
        self.assertEqual(harness.events, [
            ('construct', 0), ('connect', 0), ('home', 0),
            ('close_begin', 0), ('close_end', 0), ('construct', 1), ('connect', 1), ('home', 1),
            ('close_begin', 1), ('close_end', 1), ('construct', 2), ('connect', 2), ('home', 2)])
        self.assertTrue(all(m.closed and not m.worker.is_alive() for m in harness.instances[:-1]))
        self.assertTrue(harness.instances[-1].worker.is_alive())
        harness.wrapper.close()
        self.assertTrue(all(not m.worker.is_alive() for m in harness.instances))

    def test_cleanup_ack_failure_prevents_next_construct_connect_and_home(self):
        harness = self.make()
        harness.wrapper.home(harness.sink)
        harness.instances[0].close_errors = ['gripper worker did not stop']
        with self.assertRaisesRegex(RuntimeError, 'cleanup failed'):
            harness.wrapper.home(harness.sink)
        self.assertEqual(len(harness.instances), 1)
        self.assertEqual([e for e in harness.events if e[0] == 'home'], [('home', 0)])

    def test_cleanup_exception_prevents_next_backend(self):
        harness = self.make()
        harness.wrapper.home(harness.sink)
        def fail(): raise RuntimeError('socket cleanup failed')
        harness.instances[0].close = fail
        with self.assertRaisesRegex(RuntimeError, 'socket cleanup failed'):
            harness.wrapper.home(harness.sink)
        self.assertEqual(len(harness.instances), 1)

    def test_gripper_source_sequence_spans_homing_polling_and_backend_replacement(self):
        harness = self.make()
        for _ in range(3):
            harness.wrapper.home(harness.sink)
            harness.wrapper.inner.channel.gripper_status()
        harness.wrapper.close()
        samples = [s for s in harness.sink.samples if s['kind'] == 'gripper']
        self.assertEqual([s['sequence'] for s in samples], list(range(1, len(samples) + 1)))
        self.assertEqual(len(samples), 9)  # home + poll + close in each generation
        self.assertTrue(all(s['values']['position_pulse'] == 400 for s in samples))
        self.assertTrue(all(s['values']['controller_target_pulse'] == 850 for s in samples))
        self.assertTrue(all(s['device_sequence'] is None for s in samples))

    def test_report_cache_is_not_republished_but_new_generation_is(self):
        harness = self.make()
        harness.wrapper.home(harness.sink)
        harness.wrapper.feedback()
        harness.wrapper.feedback()
        harness.instances[0].report_sequence = 2
        harness.wrapper.feedback()
        harness.wrapper.home(harness.sink)
        harness.wrapper.feedback()
        samples = [s for s in harness.sink.samples if s['kind'] == 'report']
        self.assertEqual([s['sequence'] for s in samples], [1, 2, 3])
        self.assertEqual([s['control_report_sequence'] for s in samples], [1, 2, 1])

    def test_gripper_sink_failure_does_not_change_sdk_query_result(self):
        harness = self.make()
        def fail(sample): raise OSError('disk full')
        harness.sink.publish_state = fail
        observed = harness.wrapper._channel('fake', .05)
        result = observed.gripper_status()
        self.assertEqual(result['position_pulse'], 400)
        self.assertEqual(result['controller_target_pulse'], 850)


class CoordinatorTests(unittest.TestCase):
    def setUp(self):
        self.sink = EpisodeSink()
        self.events = queue.Queue()
        self.coordinator = CollectionCoordinator(self.sink, self.events)
        self.clock = FakeClock()
        clock_patch = patch('spacemouse_teleop.backends.xarm6_camera.coordinator.time', self.clock)
        clock_patch.start()
        self.addCleanup(clock_patch.stop)

    def event(self, name, running=True):
        self.clock.advance()
        self.events.put_nowait(name)
        return self.coordinator.poll(running)

    def test_record_requires_running_and_healthy_sources(self):
        self.event('toggle', running=False)
        self.assertEqual(self.sink.calls, [])
        self.assertEqual(self.coordinator.phase, 'waiting_neutral')
        self.sink.healthy = False
        self.event('toggle')
        self.assertEqual(self.sink.state, 'idle')
        self.sink.healthy = True
        self.event('toggle')
        self.assertEqual(self.sink.state, 'recording')

    def test_start_end_keep_and_discard_never_request_motion_preparation(self):
        for disposition, expected in (('keep', 'kept'), ('discard', 'discarded')):
            self.assertIsNone(self.event('toggle'))
            self.assertEqual(self.sink.state, 'recording')
            self.assertIsNone(self.event('toggle'))
            self.assertEqual(self.sink.state, 'review')
            self.assertIsNone(self.event(disposition))
            self.assertEqual(self.sink.state, 'idle')
            self.assertIn(('disposition', expected), self.sink.calls)
        self.assertEqual(self.coordinator.phase, 'teleop')

    def test_camera_fault_does_not_request_motion_and_keep_does_not_make_complete(self):
        self.event('toggle')
        self.sink.mark_fault('camera disconnected')
        self.assertIsNone(self.coordinator.poll(True))
        self.assertEqual(self.coordinator.phase, 'teleop')
        self.assertFalse(self.sink.complete)
        self.assertIsNone(self.event('keep'))
        self.assertFalse(self.sink.complete)
        self.assertEqual(self.sink.state, 'idle')

    def test_next_requires_idle_after_recording_and_disposition(self):
        for state in ('recording', 'review', 'finalizing', 'fault'):
            self.sink.state = state
            self.assertIsNone(self.event('prepare_next'))
            self.assertEqual(self.coordinator.phase, 'teleop')
        self.sink.state = 'idle'
        self.assertEqual(self.event('prepare_next'), 'prepare_next')
        self.assertEqual(self.coordinator.phase, 'preparing')

    def test_quit_is_seen_in_homing_guard_and_nonquit_keys_are_not_deferred_through_reset(self):
        self.coordinator.set_phase('preparing')
        self.events.put_nowait('toggle')
        self.events.put_nowait('prepare_next')
        self.coordinator.check_exit()
        self.coordinator.poll(True)
        self.assertEqual(self.sink.calls, [])
        self.events.put_nowait('quit')
        with self.assertRaises(KeyboardInterrupt): self.coordinator.check_exit()
        self.assertTrue(self.coordinator.quit_requested)

    def test_auto_record_waits_for_neutral_and_starts_once_per_preparation(self):
        self.coordinator.auto_record = True
        self.coordinator.poll(False)
        self.assertFalse(self.sink.calls)
        self.coordinator.poll(True)
        self.coordinator.poll(True)
        self.assertEqual(self.sink.calls, ['start'])
        self.event('toggle')
        self.event('keep')
        self.event('prepare_next')
        self.coordinator.poll(False)
        self.assertEqual(self.sink.calls.count('start'), 1)
        self.coordinator.poll(True)
        self.assertEqual(self.sink.calls.count('start'), 2)


class CollectionRunLoopTests(unittest.TestCase):
    def setup_loop(self, homing=True):
        config = live_config()
        config.gripper['enabled'] = False
        config.robot['homing']['enabled'] = homing
        harness = MotionHarness()
        harness.wrapper.config = config
        harness.wrapper.connect()
        self.addCleanup(harness.close)
        events = queue.Queue()
        coordinator = CollectionCoordinator(harness.sink, events)
        clock_patch = patch('spacemouse_teleop.cli.xarm_teleop.time', harness.clock)
        coordinator_patch = patch('spacemouse_teleop.backends.xarm6_camera.coordinator.time', harness.clock)
        clock_patch.start()
        coordinator_patch.start()
        self.addCleanup(clock_patch.stop)
        self.addCleanup(coordinator_patch.stop)
        return config, harness, events, coordinator

    def test_q_during_home_interrupts_before_activation_or_velocity(self):
        config, harness, events, coordinator = self.setup_loop()
        events.put_nowait('quit')
        source = types.SimpleNamespace(snapshot=lambda now: (raw(now), 'hold'))
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(KeyboardInterrupt):
            run_loop(config, source, harness.wrapper, harness.sink, coordinator=coordinator)
        self.assertFalse(any(e[0] in ('activate', 'send') for e in harness.events))
        self.assertFalse(harness.sink.calls)

    def test_next_event_returns_to_orchestrator_and_next_loop_replaces_backend(self):
        config, harness, events, coordinator = self.setup_loop()
        class Source:
            def __init__(self): self.calls = 0
            def snapshot(self, now):
                self.calls += 1
                # First call is the homing guard; second centers and activates.
                if self.calls % 2 == 0: events.put_nowait('prepare_next')
                return raw(now), 'hold'
        source = Source()
        with contextlib.redirect_stdout(io.StringIO()):
            for _ in range(3):
                result = run_loop(config, source, harness.wrapper, harness.sink, coordinator=coordinator)
                self.assertEqual(result, 'prepare_next')
        self.assertEqual(len(harness.instances), 3)
        for index in (0, 1):
            self.assertLess(harness.events.index(('close_end', index)),
                            harness.events.index(('home', index + 1)))
        self.assertFalse(harness.sink.calls)  # Neither reset nor neutral wait records an episode.

    def test_camera_fault_and_episode_controls_preserve_active_velocity_loop(self):
        config, harness, events, coordinator = self.setup_loop(homing=False)
        class Source:
            calls = 0
            def snapshot(self, now):
                self.calls += 1
                if self.calls == 1: events.put_nowait('toggle')
                if self.calls == 3: harness.sink.mark_fault('RGB timeout')
                if self.calls == 4: events.put_nowait('discard')
                return raw(now, 0. if self.calls == 1 else .2), 'hold'
        with contextlib.redirect_stdout(io.StringIO()):
            run_loop(config, Source(), harness.wrapper, harness.sink, duration=.12, coordinator=coordinator)
        self.assertEqual(harness.sink.state, 'idle')
        self.assertGreaterEqual(sum(e[0] == 'send' for e in harness.events), 4)
        self.assertEqual(sum(e[0] == 'activate' for e in harness.events), 1)
        self.assertFalse(any(e[0] in ('home', 'close_begin') for e in harness.events))

    def test_default_coordinator_none_keeps_plain_teleop(self):
        config, harness, _, _ = self.setup_loop(homing=False)
        source = types.SimpleNamespace(snapshot=lambda now: (raw(now), 'hold'))
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertIsNone(run_loop(config, source, harness.wrapper, harness.sink, duration=.05))
        self.assertTrue(any(e[0] == 'send' for e in harness.events))
        self.assertFalse(harness.sink.calls)


if __name__ == '__main__':
    unittest.main()
