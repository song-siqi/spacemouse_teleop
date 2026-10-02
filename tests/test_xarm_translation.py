import contextlib
from dataclasses import replace
import io
import json
import math
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

from robot_teleop.backends.real.xarm_sdk.backend import XArmBackend
from robot_teleop.inputs.spacemouse.polling import LatestInput, input_deadline, is_neutral
from robot_teleop.backends.real.xarm_sdk.safety import RobotFeedback, TeleopFault, VelocityLimiter
from robot_teleop.cli import xarm_translation as translation
from robot_teleop.cli.xarm_teleop import process_input, run_loop
from robot_teleop.inputs.spacemouse import TeleopCore
from robot_teleop.inputs.spacemouse.command import RawSpaceMouseState


def raw(t=10., axes=(0.,)*6, **kwargs):
    value = RawSpaceMouseState(*axes, buttons=(0, 0), timestamp=t, report_timestamp=t,
                              report_sequence=int(t*1000), connected=True, axis_report_timestamps=(t,)*6)
    return replace(value, **kwargs)


class Source:
    def __init__(self, value):
        self.value = value
    def start(self): pass
    def snapshot(self, now):
        buttons = self.value.buttons
        intent = ('open' if buttons[0] else 'close') if bool(buttons[0]) != bool(buttons[1]) else 'hold'
        return self.value, intent
    def close(self): pass


class Sink:
    def __init__(self): self.rows = []
    def open(self): pass
    def write(self, row): self.rows.append(row)
    def close(self): pass


def feedback(config, rpy=(math.pi, 0., 0.), **kwargs):
    value = RobotFeedback(time.monotonic(), 1, 2, 5, 0, 0, tuple(config.site['initial_joints_rad']),
        (200., 0., 400., *rpy), tuple(config.site['expected_tcp_offset_mm_rad']),
        tuple(config.site['expected_payload_kg_com_mm']), 6, 6, 63, 63, 'v2.5.5')
    return replace(value, **kwargs)


class TranslationConfigTests(unittest.TestCase):
    def test_profile_is_independent_and_retains_demo_motion_settings(self):
        demo = translation.REPO_ROOT / 'configs/spacemouse_xarm6_demo.json'
        before = demo.read_bytes()
        config, tolerance = translation.load_profile(ip='fake-ip')
        data = json.loads(translation.DEFAULT_CONFIG.read_text())
        self.assertEqual(data.pop('orientation_guard'), {'tolerance_deg': 3.})
        self.assertEqual(data, json.loads(before))
        self.assertEqual(config.robot['ip'], 'fake-ip')
        self.assertEqual(tolerance, 3.)
        self.assertEqual(config.robot['control_hz'], 60)
        self.assertEqual(config.reader['poll_hz'], 250)
        self.assertEqual(config.gripper['poll_hz'], 20)
        self.assertEqual(config.teleop.max_linear_speed_mps, .03)
        self.assertEqual(demo.read_bytes(), before)
        self.assertEqual(translation.load_profile()[0].robot['ip'], '192.168.1.209')

    def test_bad_profile_fails_before_any_hardware(self):
        base = json.loads(translation.DEFAULT_CONFIG.read_text())
        changes = [lambda d: d.pop('orientation_guard')]
        changes += [lambda d, v=v: d['orientation_guard'].update(tolerance_deg=v)
                    for v in (0, -1, 90, 180, True, float('nan'), float('inf'))]
        changes += [lambda d: d['teleop']['mapping']['linear']['x'].update(source='roll'),
                    lambda d: d['teleop']['mapping']['angular']['x'].update(source='x'),
                    lambda d: d['teleop'].update(rotation_only_button=0),
                    lambda d: d['robot']['homing'].update(enabled=False),
                    lambda d: d['gripper'].update(enabled=False),
                    lambda d: d['site'].update(expected_tcp_offset_mm_rad=[0, 0, 0, 0, 0, 1])]
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'config.json'
            for change in changes:
                data = json.loads(json.dumps(base))
                change(data)
                path.write_text(json.dumps(data))
                with self.subTest(data=data), self.assertRaises(ValueError):
                    translation.load_profile(path)

    def test_cli_default_is_unlimited_and_independent(self):
        args = translation.build_parser().parse_args([])
        self.assertEqual(Path(args.config), translation.DEFAULT_CONFIG)
        self.assertEqual(args.duration, 0.)
        self.assertIsNone(args.log)

    def test_existing_log_is_never_appended_or_overwritten(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'original.jsonl'
            path.write_text('original')
            link = Path(folder) / 'latest.jsonl'
            link.symlink_to(path)
            broken = Path(folder) / 'broken_latest.jsonl'
            missing = Path(folder) / 'missing.jsonl'
            broken.symlink_to(missing)
            for candidate in (path, link, broken):
                with self.assertRaises(FileExistsError): translation.new_log_path(candidate)
            self.assertEqual(path.read_text(), 'original')
            self.assertTrue(link.is_symlink())
            self.assertFalse(missing.exists())


class TranslationInputTests(unittest.TestCase):
    def setUp(self):
        self.config, _ = translation.load_profile()

    def test_startup_keeps_six_axes_and_buttons_for_neutral_check(self):
        device = Source(raw(axes=(0, 0, 0, .8, 0, 0)))
        source = translation.TranslationInput(device)
        sample, _ = source.snapshot(10.)
        self.assertIs(sample, device.value)
        self.assertFalse(is_neutral(sample, self.config))
        device.value = raw(buttons=(1, 0))
        self.assertFalse(is_neutral(source.snapshot(10.)[0], self.config))
        device.value = raw()
        self.assertTrue(is_neutral(source.snapshot(10.)[0], self.config))

    def test_projection_retains_transport_metadata_and_does_not_mutate_input(self):
        original = raw(axes=(.3, -.4, .5, .6, -.7, .8))
        source = translation.TranslationInput(Source(original))
        source.active = True
        sample, _ = source.snapshot(10.)
        self.assertEqual(sample, replace(original, roll=0., pitch=0., yaw=0.))
        self.assertEqual(source.last_raw, original)
        self.assertEqual(original.roll, .6)
        self.assertEqual(sample.axis_report_timestamps, original.axis_report_timestamps)

    def test_rotation_never_changes_translation_or_produces_rotation_commands(self):
        for linear in ((0, 0, 0), (.5, 0, 0), (0, -.5, 0), (0, 0, .5), (.3, -.4, .5)):
            expected = None
            for angular in ((0, 0, 0), (1, 0, 0), (0, -1, 0), (0, 0, 1), (.8, -.6, .4)):
                source = translation.TranslationInput(Source(raw(axes=linear+angular)))
                source.active = True
                sample, intent = source.snapshot(10.)
                command = process_input(TeleopCore(self.config.teleop), sample, intent, self.config)
                if expected is None: expected = command.linear_vel_mps
                self.assertEqual(command.linear_vel_mps, expected)
                self.assertEqual(command.angular_vel_radps, (0.,)*3)
                self.assertEqual(command.delta_rot_rad, (0.,)*3)

    def test_xyz_neutral_clears_filter_and_limiter_even_while_rotating(self):
        device = Source(raw(axes=(1., 0, 0, 0, 0, 0)))
        source = translation.TranslationInput(device)
        source.active = True
        core, limiter = TeleopCore(self.config.teleop), VelocityLimiter(self.config)
        sample, intent = source.snapshot(10.)
        cmd = process_input(core, sample, intent, self.config)
        self.assertGreater(limiter.convert(cmd, 10., .2)[0], 0.)
        device.value = raw(10.02, axes=(0, 0, 0, 1, .5, -.5))
        sample, intent = source.snapshot(10.02)
        cmd = process_input(core, sample, intent, self.config)
        speeds = limiter.convert(cmd, 10.02, .2, neutral=True)
        self.assertEqual(cmd.linear_vel_mps, (0.,)*3)
        self.assertEqual(speeds, (0.,)*6)
        self.assertEqual(core._filtered_linear, (0.,)*3)
        self.assertEqual(limiter.linear, (0.,)*3)

    def test_rotation_updates_cannot_refresh_expired_translation(self):
        original = raw(10., axes=(1, 0, 0, 1, 0, 0), axis_report_timestamps=(9., 10., 10., 10., 10., 10.))
        source = translation.TranslationInput(Source(original))
        source.active = True
        effective, intent = source.snapshot(10.)
        with self.assertRaisesRegex(TeleopFault, 'expired'):
            input_deadline(effective, 10., self.config, intent)

    def test_stale_ignored_rotation_is_not_a_nonzero_motion_input(self):
        original = raw(10., axes=(0, 0, 0, 1, 0, 0), report_timestamp=9., axis_report_timestamps=(9.,)*6)
        source = translation.TranslationInput(Source(original))
        source.active = True
        effective, intent = source.snapshot(10.)
        self.assertAlmostEqual(input_deadline(effective, 10., self.config, intent), 10.2)

    def test_invalid_axes_and_disconnection_are_not_hidden_by_projection(self):
        cases = [raw(axes=(0, 0, 0, value, 0, 0)) for value in (math.nan, math.inf, -math.inf)]
        cases += [raw(axes=(math.nan, 0, 0, 0, 0, 0)), raw(connected=False)]
        for original in cases:
            source = translation.TranslationInput(Source(original))
            source.active = True
            with self.assertRaises(TeleopFault): source.snapshot(10.)

    def test_existing_polling_stall_remains_a_fault(self):
        source = LatestInput(self.config, backend='mock')
        source.raw, source.polled_at = raw(10.), 9.
        projected = translation.TranslationInput(source)
        projected.active = True
        with self.assertRaisesRegex(TeleopFault, 'polling stopped'): projected.snapshot(10.)

    def test_gripper_intents_survive_projection(self):
        for buttons, expected in (((1,0),'open'), ((0,1),'close'), ((1,1),'hold'), ((0,0),'hold')):
            source = translation.TranslationInput(Source(raw(axes=(0,0,0,1,1,1),buttons=buttons)))
            source.active = True
            sample, intent = source.snapshot(10.)
            command = process_input(TeleopCore(self.config.teleop), sample, intent, self.config)
            self.assertEqual(command.gripper_intent, expected)


class OrientationTests(unittest.TestCase):
    def setUp(self): self.config, _ = translation.load_profile()

    def test_rpy_pi_wrap_and_equivalent_euler_representations(self):
        guard = translation.OrientationGuard()
        for rpy in ((math.pi,0,0),(-math.pi,0,0),(math.pi*3,0,0),(0,math.pi,math.pi)):
            guard.check(feedback(self.config, rpy))
            self.assertAlmostEqual(guard.last['downward_error_deg'], 0., places=5)

    def test_tilt_and_yaw_fail_and_preserve_offending_measurement(self):
        for rpy in ((math.pi+.07,0,0),(math.pi,.07,0),(math.pi,0,.07),(0,0,0)):
            guard = translation.OrientationGuard()
            value = feedback(self.config, rpy)
            with self.assertRaisesRegex(TeleopFault, 'Downward orientation'):
                guard.capture_reference(value)
            self.assertIsNone(guard.reference)
            self.assertEqual(guard.last['actual_rpy_rad'], rpy)

    def test_tolerance_boundary_is_inclusive(self):
        guard = translation.OrientationGuard(3.)
        guard.check(feedback(self.config,(math.pi,0,math.radians(3))))
        with self.assertRaises(TeleopFault):
            guard.check(feedback(self.config,(math.pi,0,math.radians(3.001))))

    def test_reference_is_fixed_and_drift_can_fail_while_still_downward(self):
        guard = translation.OrientationGuard()
        guard.capture_reference(feedback(self.config,(math.pi,0,math.radians(2))))
        original = guard.reference
        with self.assertRaisesRegex(TeleopFault, 'drift'):
            guard.check(feedback(self.config,(math.pi,0,math.radians(-2))))
        self.assertEqual(guard.reference, original)
        with self.assertRaisesRegex(TeleopFault, 'cannot be updated'):
            guard.capture_reference(feedback(self.config))

    def test_reference_drift_within_tolerance_does_not_bypass_downward_limit(self):
        guard = translation.OrientationGuard()
        guard.capture_reference(feedback(self.config,(math.pi,0,math.radians(2))))
        with self.assertRaisesRegex(TeleopFault, 'Downward orientation'):
            guard.check(feedback(self.config,(math.pi,0,math.radians(4))))

    def make_backend(self, value=None):
        value = value or feedback(self.config)
        reports = type('Reports', (), {'value':value, 'snapshot':lambda self:self.value, 'close':lambda self:None})()
        backend = translation.TranslationBackend(self.config, translation.OrientationGuard())
        backend.reports = reports
        return backend

    def test_homing_feedback_can_rotate_before_cartesian_guard_is_enabled(self):
        backend = self.make_backend(feedback(self.config,(0,0,0),mode=4))
        backend.control_mode, backend.armed = 4, True
        self.assertEqual(backend.feedback().mode, 4)
        calls = []
        backend.channel = type('Channel', (), {'joint_velocity':lambda self,*args:calls.append(args)})()
        speeds = (.01,.02,.03,.04,.05,.06)
        backend._send_velocity(speeds,time.monotonic()+.2)
        self.assertEqual(calls[0][0],speeds)
        self.assertIs(translation.TranslationBackend.home, XArmBackend.home)
        self.assertIs(translation.TranslationBackend.stop, XArmBackend.stop)

    def test_bad_start_pose_prevents_cartesian_activation(self):
        backend = self.make_backend(feedback(self.config,(0,0,0)))
        with patch.object(XArmBackend,'activate') as activate:
            with self.assertRaises(TeleopFault): backend.activate()
            activate.assert_not_called()

    def test_activation_checks_fresh_post_activation_pose_and_captures_once(self):
        backend = self.make_backend()
        before = backend.reports.value
        after = replace(before, sequence=2, tcp_mm_rad=(200,0,400,math.pi,0,.01))
        def activate(instance):
            instance.reports.value = after
            instance.armed = True
        with patch.object(XArmBackend,'activate',activate): backend.activate()
        self.assertEqual(backend.guard.reference_rpy, after.tcp_mm_rad[3:])
        self.assertEqual(backend.guard.last['feedback_sequence'], 2)

    def test_bad_post_activation_pose_stops_without_reference_or_velocity(self):
        backend = self.make_backend()
        def activate(instance):
            instance.reports.value = feedback(self.config,(0,0,0),sequence=2)
        with patch.object(XArmBackend,'activate',activate), patch.object(backend,'stop') as stop:
            with self.assertRaises(TeleopFault): backend.activate()
            stop.assert_called_once()
        self.assertIsNone(backend.guard.reference)

    def test_send_checks_latest_orientation_before_sdk_write(self):
        backend = self.make_backend()
        backend.guard.capture_reference(backend.reports.value)
        backend.armed = backend._monitor_orientation = True
        calls = []
        backend.channel = type('Channel', (), {'velocity':lambda self,*args:calls.append(args)})()
        deadline = time.monotonic()+.2
        backend.send((1,0,0,0,0,0),deadline)
        self.assertEqual(len(calls),1)
        self.assertGreater(calls[0][1],0)
        self.assertLessEqual(calls[0][1],.2)
        for update in (feedback(self.config,(0,0,0)), feedback(self.config,received_at=time.monotonic()-1)):
            backend.reports.value = update
            with self.assertRaises(TeleopFault): backend.send((1,0,0,0,0,0),time.monotonic()+.2)
        self.assertEqual(len(calls),1)

    def test_nonzero_rotation_never_reaches_backend_or_becomes_correction(self):
        backend = self.make_backend()
        backend.guard.capture_reference(backend.reports.value)
        for speeds in ((1,0,0,.000000001,0,0),(0,0,0,0,0,math.nan),(1,0,0)):
            with patch.object(XArmBackend,'send') as send:
                with self.assertRaises(TeleopFault): backend.send(speeds,time.monotonic()+.2)
                send.assert_not_called()


class TranslationLifecycleTests(unittest.TestCase):
    def test_existing_loop_waits_for_full_neutral_then_projects_and_logs_original(self):
        config, _ = translation.load_profile()
        config.robot['homing']['enabled'] = False
        values = [(0,0,0,.8,0,0),(0,0,0,0,0,0),(.5,0,0,.8,.3,.4),(0,0,0,.8,.3,.4)]
        class SequenceSource:
            def __init__(self): self.index = 0
            def snapshot(self, now):
                if self.index == len(values): raise KeyboardInterrupt
                sample = raw(now, axes=values[self.index])
                self.index += 1
                return sample,'hold'
        source = translation.TranslationInput(SequenceSource())
        sink = Sink()
        recorder = translation.TranslationRecorder(sink,source,translation.OrientationGuard(),dry_run=True)
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(KeyboardInterrupt):
            run_loop(config,source,None,recorder,dry_run=True)
        rows = [row for row in sink.rows if 'effective_input' in row]
        self.assertEqual(len(rows),2)
        self.assertEqual(rows[0]['raw_input']['roll'],.8)
        self.assertEqual(rows[0]['effective_input']['roll'],0.)
        self.assertGreater(rows[0]['execution']['would_send_mm_s_rad_s'][0],0.)
        self.assertEqual(rows[1]['execution']['would_send_mm_s_rad_s'],(0.,)*6)
        self.assertIsNone(rows[0]['orientation'])
        self.assertEqual(rows[0]['orientation_validation'],'not_evaluated_dry_run')

    def test_cli_dry_run_is_offline_and_outputs_only_translation(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'run.jsonl'
            original = translation.LatestInput
            opened = []
            def make(config,backend):
                opened.append(backend)
                return original(config,backend=backend)
            with patch.object(translation,'LatestInput',make), \
                    patch.object(translation,'TranslationBackend',side_effect=AssertionError('Hardware backend forbidden')), \
                    patch('socket.create_connection',side_effect=AssertionError('Network forbidden')), \
                    contextlib.redirect_stdout(io.StringIO()):
                status = translation.main(['--dry-run','--duration','.3','--log',str(path)])
            self.assertEqual(status,0)
            self.assertEqual(opened,['mock'])
            rows=[json.loads(line) for line in path.read_text().splitlines()]
            commands=[r for r in rows if 'command' in r]
            self.assertTrue(commands)
            self.assertEqual(rows[-1]['status'],0)
            self.assertTrue(all(r['command']['angular_vel_radps']==[0,0,0] for r in commands))
            self.assertTrue(all(r['command']['delta_rot_rad']==[0,0,0] for r in commands))

    def test_cleanup_stops_robot_before_input_join_and_log_close_on_fault(self):
        order=[]
        class Input:
            last_raw=None
            def __init__(self,*a,**kw): pass
            def start(self): order.append('input_start')
            def close(self): order.append('input_close')
        class Backend:
            def __init__(self,*a,**kw): pass
            def connect(self): order.append('connect')
            def close(self): order.append('robot_stop'); return []
        class Recorder:
            def __init__(self,*a,**kw): pass
            def open(self): order.append('log_open')
            def write(self,row): order.append(row.get('event','row'))
            def close(self): order.append('log_close')
        with tempfile.TemporaryDirectory() as folder, \
                patch.object(translation,'TranslationInput',Input), \
                patch.object(translation,'TranslationBackend',Backend), \
                patch.object(translation,'TranslationRecorder',Recorder), \
                patch.object(translation,'run_loop',side_effect=TeleopFault('pose deviation')), \
                contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            status=translation.main(['--log',str(Path(folder)/'test.jsonl')])
        self.assertEqual(status,1)
        self.assertLess(order.index('robot_stop'),order.index('input_close'))
        self.assertLess(order.index('robot_stop'),order.index('session_end'))
        self.assertLess(order.index('robot_stop'),order.index('log_close'))


if __name__ == '__main__':
    unittest.main()
