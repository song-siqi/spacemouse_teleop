import contextlib
import copy
from dataclasses import replace
import io
import json
import math
import os
from pathlib import Path
import pty
import signal
import tempfile
import termios
import unittest
from unittest.mock import patch, Mock

from robot_teleop.cli import xarm_demo as demo
from robot_teleop.backends.real.xarm_sdk.backend import XArmBackend
from robot_teleop.backends.real.xarm_sdk.config import XArmConfig
from robot_teleop.backends.real.xarm_sdk.safety import RobotFeedback, TeleopFault, check_site
from robot_teleop.inputs.spacemouse.command import RawSpaceMouseState


def config():
    return XArmConfig.load(demo.DEFAULT_CONFIG)


def feedback(c, now=10., **kwargs):
    f = RobotFeedback(now, 1, 2, 5, 0, 0, tuple(c.site['initial_joints_rad']),
                      (200, 0, 300, math.pi, 0, 0), tuple(c.site['expected_tcp_offset_mm_rad']),
                      tuple(c.site['expected_payload_kg_com_mm']), 6, 6, 63, 63, 'v2.5.5')
    return replace(f, **kwargs)


class Clock:
    def __init__(self): self.t = 10.
    def now(self): return self.t
    def sleep(self, dt): self.t += dt


class Source:
    def __init__(self, clock, calls):
        self.clock, self.calls = clock, calls
        self.values = lambda: ((0.,)*6, (0, 0))
        self.fault = None
    def snapshot(self, now):
        if self.fault: raise self.fault
        axes, buttons = self.values()
        raw = RawSpaceMouseState(*axes, buttons=buttons, timestamp=now, report_timestamp=now,
                                 report_sequence=int(now*1000), connected=True, axis_report_timestamps=(now,)*6)
        intent = 'open' if buttons[0] and not buttons[1] else 'close' if buttons[1] and not buttons[0] else 'hold'
        return raw, intent
    def close(self): self.calls.append('input_close')


class Log:
    def __init__(self, calls): self.rows, self.calls = [], calls
    def write(self, row): self.rows.append(copy.deepcopy(row))
    def close(self): self.calls.append('log_close')


class Keys:
    def __init__(self, clock, times=()):
        self.clock, self.times = clock, list(times)
        self.polls = []
        self.failure = None
    def poll(self, accept_home=True):
        self.polls.append(accept_home)
        if self.failure: raise self.failure
        if self.times and self.clock.now() >= self.times[0]:
            self.times.pop(0)
            return accept_home
        return False


class Backend:
    def __init__(self, rig, c):
        self.rig, self.config = rig, c
        self.number = len(rig.backends)
        rig.backends.append(self)
        self.gripper = None
        self.startup_feedback = None
        self.active = False
    def call(self, name): self.rig.calls.append((self.number, name))
    def connect(self): self.call('connect')
    def feedback(self, running=True):
        if self.rig.feedback_error: raise self.rig.feedback_error
        return feedback(self.config, self.rig.clock.now(), joints_rad=(.1,)*6)
    def activate(self):
        self.call('activate')
        self.active = True
        self.startup_feedback = self.feedback()
    def send(self, speeds, expires_at, intent):
        self.call('send')
        if self.rig.send_error: raise self.rig.send_error
        self.rig.sent.append((self.number, speeds, intent, expires_at-self.rig.clock.now()))
        return {'speeds_mm_s_rad_s': speeds}
    def home(self, recorder, guard):
        self.call('open_gripper')
        self.rig.home_targets.append(list(self.config.site['initial_joints_rad']))
        if self.rig.home_error: raise self.rig.home_error
        self.call('home')
        for _ in range(10):
            guard()
            self.rig.clock.sleep(.1)
        self.rig.after_home(self.rig)
    def close(self):
        self.call('stop')
        self.active = False
        self.call('gripper_close')
        self.call('close')
        return ['stop failed'] if self.rig.close_error else []


class Rig:
    def __init__(self, times=(), dry_run=False):
        self.clock, self.config = Clock(), config()
        self.calls, self.backends, self.sent, self.home_targets = [], [], [], []
        self.close_error = False
        self.feedback_error = self.home_error = self.send_error = None
        self.after_home = lambda rig: None
        self.source = Source(self.clock, self.calls)
        self.log = Log(self.calls)
        self.keys = Keys(self.clock, times)
        self.session = demo.DemoSession(self.config, self.source, self.log, self.keys, dry_run=dry_run,
             backend_factory=lambda c: Backend(self, c), clock=self.clock.now, sleep=self.clock.sleep)
    def run(self, duration=.3):
        with contextlib.redirect_stdout(io.StringIO()): self.session.run(duration)


class ActivationTests(unittest.TestCase):
    def test_current_position_used_only_for_activation(self):
        c = config()
        original = copy.deepcopy(c)
        backend = demo.DemoBackend(c)
        f = feedback(c, joints_rad=(.4,)*6)
        backend.feedback = lambda **kw: f
        def activate(instance):
            self.assertIsNot(instance.config, c)
            self.assertEqual(instance.config.site['initial_joints_rad'], [.4]*6)
            check_site(instance.config, f)
        with patch.object(XArmBackend, 'activate', activate): backend.activate()
        self.assertIs(backend.config, c)
        self.assertEqual(c, original)
        self.assertIs(demo.DemoBackend.home, XArmBackend.home)

    def test_failed_activation_restores_original_config(self):
        c = config()
        backend = demo.DemoBackend(c)
        backend.feedback = lambda **kw: feedback(c)
        with patch.object(XArmBackend, 'activate', side_effect=TeleopFault('activate failed')):
            with self.assertRaises(TeleopFault): backend.activate()
        self.assertIs(backend.config, c)

    def test_tool_mismatch_still_rejected_before_activation(self):
        c = config()
        backend = demo.DemoBackend(c)
        backend.feedback = lambda **kw: feedback(c, tcp_offset_mm_rad=(0.,)*6)
        with patch.object(XArmBackend, 'activate') as activate:
            with self.assertRaises(TeleopFault): backend.activate()
            activate.assert_not_called()


class SessionTests(unittest.TestCase):
    def test_start_does_not_home_or_open_and_six_axis_motion_is_retained(self):
        r = Rig()
        r.source.values = lambda: (((.4,)*6 if r.clock.now() > 10.05 else (0.,)*6), (1, 0) if r.clock.now()>10.05 else (0, 0))
        r.run()
        self.assertNotIn((0,'home'), r.calls)
        self.assertNotIn((0,'open_gripper'), r.calls)
        self.assertTrue(any(any(v for v in row[1][3:]) for row in r.sent))
        self.assertTrue(any(row[2]=='open' for row in r.sent))
        self.assertTrue(all(0 < row[3] <= r.config.robot['command_duration_s']+1e-9 for row in r.sent))

    def test_waits_for_all_axes_and_both_buttons(self):
        for axes, buttons in (((0,0,0,0,0,.5),(0,0)), ((0,)*6,(1,0)), ((0,)*6,(0,1))):
            r=Rig()
            r.source.values=lambda: (axes,buttons)
            with self.assertRaisesRegex(TeleopFault,'neutral'): r.run(.1)
            self.assertNotIn((0,'activate'),r.calls)
            self.assertFalse(r.sent)

    def test_two_homes_close_old_instances_and_preserve_target(self):
        r=Rig(times=[10.1,11.25])
        original=copy.deepcopy(r.config)
        r.run(.5)
        self.assertEqual(len(r.backends),3)
        self.assertEqual(r.home_targets,[original.site['initial_joints_rad']]*2)
        self.assertEqual(r.config,original)
        for old in (0,1):
            self.assertLess(r.calls.index((old,'stop')),r.calls.index((old,'gripper_close')))
            self.assertLess(r.calls.index((old,'close')),r.calls.index((old+1,'connect')))
            self.assertLess(r.calls.index((old+1,'open_gripper')),r.calls.index((old+1,'home')))
            self.assertLess(r.calls.index((old+1,'home')),r.calls.index((old+1,'activate')))
        self.assertEqual(sum(row.get('event')=='session_start' for row in r.log.rows),1)
        self.assertEqual(sum(row.get('event')=='teleop_started' for row in r.log.rows),3)
        self.assertAlmostEqual(r.session.active_elapsed,.5,delta=.025)
        self.assertGreaterEqual(r.clock.now()-10,2.5)

    def test_h_in_initial_wait_and_h_during_home_is_discarded(self):
        r=Rig(times=[10.,10.2])
        r.run(.2)
        self.assertEqual(len(r.backends),2)
        self.assertNotIn((0,'activate'),r.calls)
        self.assertEqual(len(r.home_targets),1)
        self.assertIn(False,r.keys.polls)

    def test_home_requires_neutral_again_and_resets_filter_history(self):
        r=Rig(times=[10.15])
        def values():
            t=r.clock.now()
            return ((.8,0,0,0,0,0) if 10.04<t<11.3 else (0.,)*6), (0,0)
        r.source.values=values
        r.run(.5)
        activations=[row for row in r.log.rows if row.get('event')=='teleop_started']
        self.assertGreaterEqual(activations[1]['timestamp'],11.3)
        after=[row for row in r.sent if row[0]==1]
        self.assertTrue(after)
        self.assertEqual(after[0][1],(0.,)*6)

    def test_close_failure_prevents_new_connection_and_home(self):
        r=Rig(times=[10.1]);r.close_error=True
        with self.assertRaisesRegex(TeleopFault,'cleanup'):r.run()
        self.assertEqual(len(r.backends),1)
        self.assertFalse(r.home_targets)

    def test_home_failure_does_not_resume(self):
        for reason in ('gripper open failed','home timed out','feedback stale'):
            r=Rig(times=[10.1]);r.home_error=TeleopFault(reason)
            with self.assertRaisesRegex(TeleopFault,reason):r.run()
            self.assertNotIn((1,'activate'),r.calls)
            r.session.close()
            self.assertIn((1,'stop'),r.calls)

    def test_home_input_or_terminal_failure_stops_progress(self):
        for field, error in (('source',TeleopFault('HID disconnected')),('keys',TeleopFault('terminal closed'))):
            r=Rig(times=[10.1])
            original=Backend.home
            def home(backend,log,guard):
                if field=='source':r.source.fault=error
                else:r.keys.failure=error
                original(backend,log,guard)
            with patch.object(Backend,'home',home),self.assertRaises(TeleopFault):r.run()
            self.assertNotIn((1,'activate'),r.calls)

    def test_nonfinite_input_is_rejected_during_home(self):
        r=Rig(times=[10.])
        r.source.values=lambda: ((math.nan,0,0,0,0,0),(0,0))
        with self.assertRaisesRegex(TeleopFault,'nonfinite'):r.run()
        self.assertFalse(r.home_targets)

    def test_neutral_wait_timeout_is_retained(self):
        r=Rig();r.config.robot['startup_timeout_s']=.05
        r.source.values=lambda: ((.5,0,0,0,0,0),(0,0))
        with self.assertRaisesRegex(TeleopFault,'Startup timed out'):r.run(0)

    def test_dry_run_home_has_no_backend(self):
        r=Rig(times=[10.1],dry_run=True);r.run(.3)
        self.assertFalse(r.backends)
        self.assertEqual(sum(row.get('event')=='home_dry_run' for row in r.log.rows),1)


class TerminalTests(unittest.TestCase):
    def setUp(self):
        self.master,self.slave=pty.openpty()
        self.stream=os.fdopen(os.dup(self.slave),'r')
        self.original=termios.tcgetattr(self.slave)
        self.clock=Clock()
        self.keys=demo.TerminalKeys(self.stream,self.clock.now)
        self.addCleanup(self.cleanup)
        self.keys.open()
    def cleanup(self):
        try:self.keys.close()
        finally:
            self.stream.close();os.close(self.slave)
            if self.master is not None:os.close(self.master)
    def type(self,s):os.write(self.master,s)
    def test_cbreak_preserves_signals_and_restores_terminal(self):
        attrs=termios.tcgetattr(self.slave)
        self.assertFalse(attrs[3]&termios.ICANON)
        self.assertTrue(attrs[3]&termios.ISIG)
        self.keys.close()
        self.assertEqual(termios.tcgetattr(self.slave),self.original)
    def test_h_without_enter_and_uppercase(self):
        self.type(b'h');self.assertTrue(self.keys.poll())
        self.clock.sleep(1);self.type(b'H');self.assertTrue(self.keys.poll())
        self.type(b'x');self.assertFalse(self.keys.poll())
    def test_repeats_ignored_including_across_home(self):
        self.type(b'h');self.assertTrue(self.keys.poll())
        for _ in range(20):
            self.clock.sleep(.1);self.type(b'h');self.assertFalse(self.keys.poll(False))
        self.clock.sleep(.1);self.type(b'h');self.assertFalse(self.keys.poll())
        self.clock.sleep(1);self.type(b'h');self.assertTrue(self.keys.poll())
    def test_eof_is_fault(self):
        os.close(self.master);self.master=None
        with self.assertRaisesRegex(TeleopFault,'Terminal'):self.keys.poll()
        # PTY hangup prevents tcsetattr; main reports this but still closes logs.
        self.keys.saved=None
    def test_non_tty_live_rejected_and_offline_allowed(self):
        keys=demo.TerminalKeys(io.StringIO())
        with self.assertRaisesRegex(TeleopFault,'interactive'):keys.open()
        keys.open(required=False);self.assertFalse(keys.poll())


class MainTests(unittest.TestCase):
    def test_defaults(self):
        a=demo.build_parser().parse_args([])
        self.assertEqual(a.duration,0)
        self.assertEqual(Path(a.config),demo.DEFAULT_CONFIG)
    def test_non_tty_rejected_before_hardware(self):
        with patch.object(demo,'TerminalKeys',return_value=demo.TerminalKeys(io.StringIO())), \
             patch.object(demo,'LatestInput') as source,patch.object(demo,'DemoBackend') as backend, \
             contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(demo.main([]),1)
            source.assert_not_called();backend.assert_not_called()
    def test_real_main_dry_run_no_sdk_hid_network(self):
        with tempfile.TemporaryDirectory() as folder:
            log=Path(folder)/'dry.jsonl'
            with patch.object(demo,'TerminalKeys',return_value=demo.TerminalKeys(io.StringIO())), \
                 patch('socket.create_connection',side_effect=AssertionError('network')), \
                 patch('robot_teleop.backends.real.xarm_sdk.backend.XArmBackend.connect',side_effect=AssertionError('robot')), \
                 contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(demo.main(['--dry-run','--duration','.2','--log',str(log)]),0)
            rows=[json.loads(line) for line in log.read_text().splitlines()]
            self.assertEqual(rows[-1]['event'],'session_end')
            self.assertEqual(rows[-1]['status'],0)
            self.assertTrue(any('execution' in row for row in rows))
            self.assertFalse(any(row.get('event')=='homing_started' for row in rows))
    def test_signals_and_faults_stop_before_input_terminal_and_log(self):
        for error,expected in ((KeyboardInterrupt(),130),(demo.SignalExit(signal.SIGTERM),143),
                               (demo.SignalExit(signal.SIGHUP),129),(TeleopFault('lost feedback'),1)):
            calls=[]
            keys=Mock(fd=1);keys.close.side_effect=lambda:calls.append('terminal')
            source=Mock();source.close.side_effect=lambda:calls.append('input')
            recorder=Mock();recorder.close.side_effect=lambda:calls.append('log')
            session=Mock();session.run.side_effect=error;session.close.side_effect=lambda:calls.append('robot')
            old=signal.getsignal(signal.SIGTERM)
            with patch.object(demo,'TerminalKeys',return_value=keys),patch.object(demo,'LatestInput',return_value=source), \
                 patch.object(demo,'AsyncJsonlRecorder',return_value=recorder),patch.object(demo,'DemoSession',return_value=session), \
                 contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(demo.main([]),expected)
            self.assertEqual(calls,['robot','input','terminal','log'])
            self.assertIs(signal.getsignal(signal.SIGTERM),old)


if __name__=='__main__':unittest.main()
