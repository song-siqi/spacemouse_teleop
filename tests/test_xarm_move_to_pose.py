import contextlib
import copy
import io
import json
import math
from pathlib import Path
import signal
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch

from spacemouse_teleop.cli import xarm_move_to_pose as move


POSE = [250., 0., 300., 180., 0., 0.]


def arguments(*extra):
    return move.build_parser().parse_args(['--pose', *map(str, POSE), *extra])


class Clock:
    def __init__(self): self.t = 10.
    def now(self): return self.t
    def sleep(self, dt): self.t += dt


class Log:
    def __init__(self, events=None):
        self.rows = []
        self.events = events if events is not None else []
        self.path = Path('/tmp/fake-move-log')
    def write(self, event, **data):
        self.rows.append(dict(event=event, **copy.deepcopy(data)))
        self.events.append(event)
    def close(self): self.events.append('log_close')


class Robot:
    def __init__(self, clock=None):
        self.clock = clock or Clock()
        self.events, self.calls = [], []
        self.active = self.sent = False
        self.initial = {}
        self.samples = []
        self.move_result = 0
        self.preflight_error = self.stop_error = None
        self.ready_value = True
        self.sample_index = 0

    def connect(self): self.events.append('connect')
    def snapshot(self):
        s = dict(axis=6, robot_type=6, received_at=self.clock.now(), mode=0 if self.active else 5,
                 report_state=2, pose_mm_deg=POSE.copy(), state=2, cmdnum=0, error_code=0, warn_code=0,
                 tcp_offset_mm_rad=[0]*6, world_offset_mm_rad=[0]*6, payload_kg_com_mm=[.82, 0, 0, 0],
                 motor_enable=63, motor_brake=63, query_before=self.clock.now(), query_after=self.clock.now())
        if not self.sent:
            s.update(self.initial)
        elif self.samples:
            sample = self.samples[min(self.sample_index, len(self.samples)-1)]
            self.sample_index += 1
            if isinstance(sample, BaseException): raise sample
            s.update(sample)
        return s
    def initial_details(self): return dict(joints_rad=[0]*6, firmware='fake', sdk_version='fake')
    def preflight(self, pose):
        self.events.append('preflight')
        if self.preflight_error: raise self.preflight_error
        return dict(target_joints_rad=[0]*6, path_checked=False)
    def activate(self):
        self.events.append('activate')
        self.active = True
    def ready(self): return self.ready_value
    def move(self, kwargs):
        self.events.append('move')
        self.calls.append(kwargs)
        self.sent = True
        if isinstance(self.move_result, BaseException): raise self.move_result
        return self.move_result
    def stop(self):
        self.events.append('stop')
        if self.stop_error: raise self.stop_error
    def close(self): self.events.append('close')


class ArgumentTests(unittest.TestCase):
    def test_defaults_and_exact_position_call(self):
        a = arguments()
        move.validate_args(a)
        self.assertEqual((a.ip, a.speed, a.accel, a.timeout, a.position_tolerance, a.orientation_tolerance),
                         ('192.168.1.209', 10, 50, 120, 2, 1))
        self.assertEqual(move.move_kwargs(a), dict(x=250., y=0., z=300., roll=180., pitch=0., yaw=0.,
                         speed=10., mvacc=50., relative=False, is_radian=False, radius=-1, wait=False))

    def test_invalid_arguments_exit_two_without_hardware(self):
        for option in ('--speed', '--accel', '--timeout', '--position-tolerance', '--orientation-tolerance'):
            for bad in ('0', '-1', 'nan', 'inf'):
                with self.subTest(option=option, bad=bad), patch.object(move, 'SDKRobot') as factory:
                    with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as caught:
                        move.main(['--pose', *map(str, POSE), option, bad])
                    self.assertEqual(caught.exception.code, 2)
                    factory.assert_not_called()
        a = arguments()
        a.pose[0] = math.nan
        with self.assertRaises(move.MoveError): move.validate_args(a)

    def test_dry_run_has_no_sdk_or_network_and_logs_no_completion(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'preview.jsonl'
            with patch.object(move, 'SDKRobot', side_effect=AssertionError('hardware')), \
                 patch('socket.create_connection', side_effect=AssertionError('network')):
                with contextlib.redirect_stdout(io.StringIO()):
                    result = move.main(['--pose', *map(str, POSE), '--dry-run', '--log', str(path)])
            self.assertEqual(result, 0)
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            preview = next(r for r in rows if r['event'] == 'dry_run')
            self.assertFalse(preview['reachability_checked'])
            self.assertFalse(any(r['event'] == 'completed' for r in rows))

    def test_logs_never_overwrite_files_or_symlinks(self):
        with tempfile.TemporaryDirectory() as folder:
            p = Path(folder) / 'existing'
            p.write_text('keep')
            symlink = Path(folder) / 'link'
            symlink.symlink_to(p)
            broken = Path(folder) / 'broken'
            broken.symlink_to(Path(folder) / 'missing')
            for path in (p, symlink, broken):
                with self.assertRaises(FileExistsError): move.JsonLog(path)
            self.assertEqual(p.read_text(), 'keep')


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.robot = Robot(self.clock)
        self.log = Log(self.robot.events)
        self.session = move.MoveSession(self.robot, self.log, self.clock.now, self.clock.sleep)

    def test_success_single_call_settles_and_preserves_position_mode(self):
        final = self.session.run(arguments())
        self.assertEqual(len(self.robot.calls), 1)
        self.assertEqual(final['mode'], 0)
        self.assertGreaterEqual(self.clock.now()-10, .3)
        self.assertFalse(self.session.may_have_motion)
        self.assertNotIn('stop', self.robot.events)
        self.assertLess(self.robot.events.index('preflight'), self.robot.events.index('activate'))
        self.assertLess(self.robot.events.index('activated'), self.robot.events.index('move'))

    def test_ack_and_passing_through_target_are_not_completion(self):
        self.robot.samples = [dict(state=1), dict(state=2, cmdnum=1), dict(state=0), dict(state=2)]
        self.session.run(arguments())
        monitors = [r for r in self.log.rows if r['event'] == 'monitor']
        self.assertTrue(all(not r['arrived'] for r in monitors[:3]))
        self.assertGreaterEqual(self.clock.now()-10, .45)

    def test_wrong_pose_or_wrong_orientation_times_out(self):
        for pose in ([260, 0, 300, 180, 0, 0], [250, 0, 300, 180, 0, 5]):
            with self.subTest(pose=pose):
                self.setUp()
                self.robot.samples = [dict(pose_mm_deg=pose)]
                with self.assertRaisesRegex(move.MoveError, 'timed out'):
                    self.session.run(arguments('--timeout', '.4'))
                self.assertTrue(self.session.may_have_motion)
                self.assertEqual(len(self.robot.calls), 1)

    def test_equivalent_pi_and_pure_rotation_target(self):
        self.robot.initial['pose_mm_deg'] = [250, 0, 300, 175, 0, 0]
        self.robot.samples = [dict(pose_mm_deg=[250, 0, 300, -180, 0, 0])]
        self.session.run(arguments())
        self.assertAlmostEqual(move.orientation_error_deg([180, 0, 0], [-180, 0, 0]), 0)
        self.assertAlmostEqual(move.orientation_error_deg([180, 0, 0], [180, 0, 5]), 5)
        self.assertAlmostEqual(move.orientation_error_deg([0, 90, 0], [180, 90, 180]), 0)

    def test_settle_restarts_after_excursion(self):
        self.robot.samples = [{}]*5 + [dict(pose_mm_deg=[260, 0, 300, 180, 0, 0])] + [{}]*9
        self.session.run(arguments())
        self.assertGreaterEqual(self.clock.now()-10, .6)

    def test_initial_faults_refuse_activation(self):
        changes = [dict(axis=7), dict(robot_type=7), dict(state=1), dict(state=3), dict(cmdnum=1),
                   dict(error_code=10), dict(warn_code=1), dict(tcp_offset_mm_rad=[0, 0, 0, .01, 0, 0]),
                   dict(world_offset_mm_rad=[1, 0, 0, 0, 0, 0]), dict(pose_mm_deg=[math.nan]*6),
                   dict(payload_kg_com_mm=[math.nan]*4)]
        for change in changes:
            with self.subTest(change=change):
                self.setUp()
                self.robot.initial.update(change)
                with self.assertRaises(move.MoveError): self.session.run(arguments())
                self.assertNotIn('activate', self.robot.events)
                self.assertFalse(self.session.may_have_motion)

    def test_mode_confirmation_timeout_never_sends(self):
        self.robot.ready_value = False
        with self.assertRaisesRegex(move.MoveError, 'activation'):
            self.session.run(arguments())
        self.assertEqual(self.robot.calls, [])

    def test_preflight_failure_does_not_enable(self):
        self.robot.preflight_error = move.MoveError('no IK')
        with self.assertRaisesRegex(move.MoveError, 'IK'): self.session.run(arguments())
        self.assertNotIn('activate', self.robot.events)

    def test_move_ack_failure_is_marked_for_stop_and_not_retried(self):
        for result in (-1, OSError('lost ACK')):
            self.setUp()
            self.robot.move_result = result
            with self.assertRaises((move.MoveError, OSError)): self.session.run(arguments())
            self.assertTrue(self.session.may_have_motion)
            self.assertEqual(len(self.robot.calls), 1)

    def test_runtime_mode_fault_or_feedback_loss(self):
        for sample in (dict(mode=5), dict(state=4), dict(motor_enable=0), dict(error_code=3),
                       move.MoveError('stale feedback')):
            self.setUp()
            self.robot.samples = [sample]
            with self.assertRaises(move.MoveError): self.session.run(arguments())
            self.assertTrue(self.session.may_have_motion)

    def test_deadline_includes_query_duration(self):
        original = self.robot.snapshot
        def slow():
            if self.robot.sent: self.clock.sleep(1.)
            return original()
        self.robot.snapshot = slow
        with self.assertRaisesRegex(move.MoveError, 'timed out while querying'):
            self.session.run(arguments('--timeout', '.5'))


class CleanupTests(unittest.TestCase):
    def run_main(self, robot, log):
        cls = move.MoveSession
        with patch.object(move, 'SDKRobot', return_value=robot), patch.object(move, 'JsonLog', return_value=log), \
             patch.object(move, 'MoveSession', side_effect=lambda r, l: cls(r, l, robot.clock.now, robot.clock.sleep)), \
             contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            return move.main(['--pose', *map(str, POSE), '--timeout', '.5'])

    def test_faults_stop_before_disconnect_and_log_close(self):
        for error, code in ((OSError('lost ACK'), 1), (KeyboardInterrupt(), 130), (move.Terminated(), 143)):
            with self.subTest(code=code):
                robot = Robot()
                log = Log(robot.events)
                robot.move_result = error
                old = signal.getsignal(signal.SIGTERM)
                self.assertEqual(self.run_main(robot, log), code)
                self.assertIs(signal.getsignal(signal.SIGTERM), old)
                self.assertLess(robot.events.index('stop'), robot.events.index('close'))
                self.assertLess(robot.events.index('close'), robot.events.index('log_close'))
                self.assertEqual(len(robot.calls), 1)

    def test_failed_stop_is_recorded_and_cleanup_continues(self):
        robot = Robot()
        log = Log(robot.events)
        robot.move_result = -1
        robot.stop_error = OSError('disconnected')
        self.assertEqual(self.run_main(robot, log), 1)
        self.assertFalse(next(r for r in log.rows if r['event'] == 'stop')['confirmed'])
        self.assertIn('close', robot.events)
        self.assertIn('log_close', robot.events)

    def test_disk_error_after_motion_still_stops(self):
        robot = Robot()
        log = Log(robot.events)
        original = log.write
        def fail(event, **fields):
            if event in ('monitor', 'stop'): raise OSError('disk full')
            original(event, **fields)
        log.write = fail
        self.assertEqual(self.run_main(robot, log), 1)
        self.assertLess(robot.events.index('stop'), robot.events.index('close'))

    def test_success_disconnects_without_stop_or_motor_disable(self):
        robot = Robot()
        log = Log(robot.events)
        self.assertEqual(self.run_main(robot, log), 0)
        self.assertNotIn('stop', robot.events)
        self.assertEqual(robot.events[-1], 'log_close')

    def test_failed_connect_is_closed_without_motion_stop(self):
        robot = Robot()
        robot.connect = Mock(side_effect=OSError('connect failed'))
        log = Log(robot.events)
        self.assertEqual(self.run_main(robot, log), 1)
        self.assertNotIn('stop', robot.events)
        self.assertIn('close', robot.events)


class AdapterTests(unittest.TestCase):
    def adapter(self):
        r = move.SDKRobot.__new__(move.SDKRobot)
        r.api = Mock()
        r._lock = threading.Lock()
        r._report = r._report_error = None
        return r

    def test_preflight_checks_codes_finiteness_and_joint_limits(self):
        for ik, limit in (((-1, []), (0, False)), ((0, [math.nan]*7), (0, False)),
                          ((0, [0]*7), (-1, False)), ((0, [0]*7), (0, True))):
            r = self.adapter()
            r.api.get_inverse_kinematics.return_value = ik
            r.api.is_joint_limit.return_value = limit
            with self.assertRaises(move.MoveError): r.preflight(POSE)
        r.api.get_inverse_kinematics.return_value = (0, [0]*7)
        r.api.is_joint_limit.return_value = (0, False)
        self.assertFalse(r.preflight(POSE)['path_checked'])
        r.api.get_inverse_kinematics.assert_called_with(POSE, input_is_radian=False, return_is_radian=True)
        r.api.is_joint_limit.assert_called_with([0]*7, is_radian=True)

    def test_snapshot_checks_every_query_code(self):
        for name in ('get_position', 'get_state', 'get_cmdnum', 'get_err_warn_code'):
            r = self.adapter()
            r.api.get_position.return_value = (0, POSE)
            r.api.get_state.return_value = (0, 2)
            r.api.get_cmdnum.return_value = (0, 0)
            r.api.get_err_warn_code.return_value = (0, [0, 0])
            getattr(r.api, name).return_value = (-1, None)
            with self.assertRaisesRegex(move.MoveError, name): r.snapshot()

    def test_rich_report_requires_real_fields_and_freshness(self):
        # Use the installed SDK parser to check names, units and packet coverage.
        with contextlib.redirect_stdout(io.StringIO()):
            from xarm.x3.report import ReportDataStructure
        import struct
        packet = bytearray(416)
        struct.pack_into('>I', packet, 0, len(packet))
        packet[145], packet[146] = 6, 6
        parser = ReportDataStructure.create(30002, is_radian=True)
        parser.update(bytes(packet))
        r = self.adapter()
        r.api._arm.report_data = parser
        r._on_report({})
        self.assertEqual(r.report()['axis'], 6)
        self.assertEqual(r.report()['tcp_offset_mm_rad'], [0]*6)
        r._report['received_at'] -= 2
        with self.assertRaisesRegex(move.MoveError, 'stale'): r.report()
        struct.pack_into('>I', packet, 0, 181)
        parser.update(bytes(packet[:181]))
        r._on_report({})
        with self.assertRaisesRegex(move.MoveError, 'lacks required'): r.report()

    def test_stop_requires_controller_confirmation(self):
        r = self.adapter()
        r.api.set_state.return_value = 0
        r.api.get_state.return_value = (0, 1)
        with self.assertRaisesRegex(move.MoveError, 'not confirmed'): r.stop()
        r.api.set_state.assert_called_once_with(4)

    def test_sdk_adapter_bounds_enable_and_sync_without_network(self):
        with contextlib.redirect_stdout(io.StringIO()):
            from xarm.wrapper import XArmAPI
        with patch.object(XArmAPI, 'connect'), patch.object(XArmAPI, 'disconnect'):
            r = move.SDKRobot('fake')
            with patch.object(r, 'report', return_value={}), patch.object(r.api, 'set_timeout') as timeout:
                r._report = {}
                r.api._arm.arm_cmd = Mock()
                receive = r.api._arm.arm_cmd.recv_modbus_response
                r.connect()
                self.assertEqual(timeout.call_args_list[0].args, (1.,))
                core = r.api._arm
                core.arm_cmd.motion_en(8, 1)
                self.assertEqual(core.arm_cmd.set_nu8.call_args.kwargs['timeout'], 1.)
                core.arm_cmd.recv_modbus_response(1, 2, 3, 10.)
                receive.assert_called_once_with(1, 2, 3, 1.)
                core._is_sync, core._need_sync = False, True
                self.assertNotEqual(core._XArm__wait_sync(), 0)
                core._is_sync, core._need_sync = True, False
                self.assertEqual(core._XArm__wait_sync(), 0)

    def test_real_sdk_position_conversion_reaches_line_codec_once(self):
        with contextlib.redirect_stdout(io.StringIO()):
            from xarm.wrapper import XArmAPI
        from unittest.mock import PropertyMock
        with patch.object(XArmAPI, 'connect'), patch.object(XArmAPI, 'disconnect'):
            r = move.SDKRobot('fake')
            r._report = {}
            core = r.api._arm
            core.arm_cmd = Mock()
            with patch.object(r, 'report', return_value={}): r.connect()
            core._major_version_number, core._minor_version_number, core._revision_version_number = 2, 5, 5
            core._is_sync, core._need_sync = True, False
            core.arm_cmd.move_line_common.return_value = [0, 0, 0, 0]
            with patch.object(type(core), 'connected', new_callable=PropertyMock, return_value=True):
                self.assertEqual(r.move(move.move_kwargs(arguments())), 0)
            call = core.arm_cmd.move_line_common.call_args
            self.assertEqual(call.args[0][:3], POSE[:3])
            self.assertAlmostEqual(call.args[0][3], math.pi)
            self.assertEqual(call.args[1:3], (10., 50.))
            self.assertEqual(call.kwargs['coord'], 0)
            self.assertFalse(call.kwargs['is_axis_angle'])
            core.arm_cmd.move_line_common.assert_called_once()
            # Existing SDK's movement function was not patched globally.
            second = XArmAPI('fake', do_not_open=True)
            self.assertNotIn('_XArm__wait_sync', vars(second._arm))


if __name__ == '__main__':
    unittest.main()
