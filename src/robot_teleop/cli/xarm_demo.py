"""Demo-only orchestration: start here, press H to open the gripper and home."""
from __future__ import annotations

import copy
from dataclasses import asdict
import math
import os
from pathlib import Path
import select
import signal
import sys
import termios
import time
import tty

from robot_teleop.backends.real.xarm_sdk.backend import XArmBackend
from robot_teleop.backends.real.xarm_sdk.config import XArmConfig
from robot_teleop.inputs.spacemouse.polling import LatestInput, axes_neutral, input_deadline, is_neutral
from robot_teleop.backends.real.xarm_sdk.safety import TeleopFault, VelocityLimiter, check_homing_site
from robot_teleop.cli.xarm_teleop import build_parser as base_parser, process_input
from robot_teleop.recording.async_jsonl import AsyncJsonlRecorder
from robot_teleop.inputs.spacemouse import TeleopCore

DEFAULT_CONFIG = Path(__file__).resolve().parents[3] / 'configs/spacemouse_xarm6_demo.json'


class DemoBackend(XArmBackend):
    def activate(self):
        # The configured home target remains immutable. Only this activation's
        # initial-joint check uses a fresh stationary starting position.
        original = self.config
        feedback = self.feedback(running=False)
        check_homing_site(original, feedback)
        local = copy.deepcopy(original)
        local.site['initial_joints_rad'] = list(feedback.joints_rad)
        self.config = local
        try:
            return super().activate()
        finally:
            self.config = original


class TerminalKeys:
    """Main-thread terminal polling; no robot calls and no background key queue."""
    def __init__(self, stream=None, clock=time.monotonic):
        self.stream = sys.stdin if stream is None else stream
        self.clock = clock
        self.fd = self.saved = None
        self.last_h = -math.inf

    def open(self, required=True):
        if not self.stream.isatty():
            if required:
                raise TeleopFault('Demo live mode requires an interactive terminal for H and Ctrl-C')
            return
        self.fd = self.stream.fileno()
        self.saved = termios.tcgetattr(self.fd)
        try:
            tty.setcbreak(self.fd, termios.TCSANOW)  # Retain ISIG / Ctrl-C.
        except BaseException:
            self.close()
            raise

    def poll(self, accept_home=True):
        if self.fd is None:
            return False
        requested = False
        # Bound terminal draining just like HID draining; never block control.
        for _ in range(8):
            try:
                readable, _, _ = select.select([self.fd], [], [], 0)
                if not readable:
                    return requested
                data = os.read(self.fd, 4096)
            except (OSError, ValueError) as exc:
                raise TeleopFault(f'Terminal disconnected: {exc}') from exc
            if not data:
                raise TeleopFault('Terminal closed')
            if b'h' in data.lower():
                now = self.clock()
                if now - self.last_h >= .8 and accept_home:
                    requested = True
                # Update even while homing: held-key repeats never build requests.
                self.last_h = now
        raise TeleopFault('Terminal input backlog exceeded limit')

    def close(self):
        if self.saved is not None:
            try:
                termios.tcsetattr(self.fd, termios.TCSANOW, self.saved)
            finally:
                self.saved = None
                self.fd = None


class DemoSession:
    def __init__(self, config, source, recorder, keys, *, dry_run=False,
                 backend_factory=DemoBackend, clock=time.monotonic, sleep=time.sleep):
        self.config, self.source, self.recorder, self.keys = config, source, recorder, keys
        self.dry_run, self.backend_factory = dry_run, backend_factory
        self.clock, self.sleep = clock, sleep
        self.backend = None
        self.phase = None
        self.active_elapsed = 0.

    def event(self, event, **fields):
        self.recorder.write({'event': event, 'timestamp': self.clock(), **fields})

    def transition(self, phase):
        previous, self.phase = self.phase, phase
        self.event('demo_phase', previous=previous, phase=phase)

    def connect(self):
        if not self.dry_run:
            self.backend = self.backend_factory(self.config)
            self.backend.connect()
            check_homing_site(self.config, self.backend.feedback(running=False))

    def close(self):
        if self.backend is not None:
            # Retain ownership if close raises, so outer cleanup can try again.
            errors = self.backend.close()
            self.backend = None
            if errors:
                raise TeleopFault('Demo backend cleanup failed: ' + '; '.join(errors))

    def home_guard(self):
        self.keys.poll(accept_home=False)
        raw, intent = self.source.snapshot(self.clock())
        if raw is None or input_deadline(raw, raw.timestamp, self.config, intent) is None:
            raise TeleopFault('SpaceMouse input unavailable during home')
        return raw, intent

    def home(self):
        self.event('home_requested', target_joints_rad=list(self.config.site['initial_joints_rad']))
        self.transition('stopping')
        self.close()  # Includes arm stop BEFORE gripper worker join.
        self.transition('homing')
        print('H: teleop stopped; opening gripper and returning to configured home.', flush=True)
        self.home_guard()
        if self.dry_run:
            self.event('home_dry_run', hardware_connected=False)
        else:
            self.connect()
            self.backend.home(self.recorder, self.home_guard)
        self.keys.poll(accept_home=False)  # Discard any queued H before rearming.
        self.transition('waiting_neutral')
        print('Home complete. Center all axes and release both buttons to resume.', flush=True)

    def run(self, duration=0.):
        self.event('session_start', config=asdict(self.config), dry_run=self.dry_run,
                   startup_homing=False, duration=duration,
                   home_joints_rad=list(self.config.site['initial_joints_rad']))
        self.connect()
        self.transition('waiting_neutral')
        print('Starting at current pose. Center all axes and release both buttons. H: open gripper + home; Ctrl-C: stop.', flush=True)
        core, limiter = TeleopCore(self.config.teleop), VelocityLimiter(self.config)
        period = 1 / self.config.robot['control_hz']
        last_budget = wait_started = next_tick = self.clock()
        previous_tick = None
        while True:
            now = self.clock()
            self.active_elapsed += now - last_budget
            last_budget = now
            if duration and self.active_elapsed >= duration:
                if self.phase != 'teleop':
                    raise TeleopFault('Duration ended before neutral input allowed teleop')
                return
            if previous_tick is not None and self.phase == 'teleop' and now - previous_tick > self.config.robot['input_timeout_s']:
                raise TeleopFault('Control loop stalled beyond input timeout')
            previous_tick = now
            if self.keys.poll():
                self.home()
                core, limiter = TeleopCore(self.config.teleop), VelocityLimiter(self.config)
                last_budget = wait_started = next_tick = self.clock()
                previous_tick = None
                continue
            feedback = None
            if not self.dry_run:
                feedback = self.backend.feedback(running=self.phase == 'teleop')
                if self.phase == 'waiting_neutral':
                    check_homing_site(self.config, feedback)
            raw, intent = self.source.snapshot(self.clock())
            if raw is not None:
                expires_at = input_deadline(raw, raw.timestamp, self.config, intent)
                if self.phase == 'waiting_neutral':
                    if expires_at is not None and is_neutral(raw, self.config):
                        if not self.dry_run:
                            self.backend.activate()
                        core.zero_command(self.clock())
                        limiter.reset()
                        previous_tick = self.clock()
                        self.transition('teleop')
                        initial = None if self.dry_run else self.backend.startup_feedback
                        self.event('teleop_started', startup_feedback=None if initial is None else initial.to_dict(),
                                   activation_joints_rad=None if initial is None else list(initial.joints_rad))
                        print('Teleop active. H: open gripper + home; Ctrl-C: stop.', flush=True)
                else:
                    if expires_at is None:
                        raise TeleopFault('Input report metadata disappeared')
                    command = process_input(core, raw, intent, self.config)
                    speeds = limiter.convert(command, raw.timestamp, expires_at - raw.timestamp,
                                             feedback=feedback, neutral=axes_neutral(raw, self.config))
                    sent = ({'would_send_mm_s_rad_s': speeds, 'duration_s': expires_at - raw.timestamp,
                             'gripper_intent': intent} if self.dry_run else self.backend.send(speeds, expires_at, intent))
                    gripper = None if self.dry_run or self.backend.gripper is None else self.backend.gripper.snapshot()
                    self.recorder.write({'mode': 'xarm-demo-dry-run' if self.dry_run else 'xarm-demo-velocity',
                                         'raw': raw.to_dict(), 'command': command.to_dict(), 'execution': sent,
                                         'feedback': None if feedback is None else feedback.to_dict(), 'gripper': gripper})
            if self.phase == 'waiting_neutral' and self.clock() - wait_started >= self.config.robot['startup_timeout_s']:
                raise TeleopFault('Startup timed out waiting for neutral SpaceMouse input')
            next_tick += period
            if next_tick < self.clock():
                next_tick = self.clock() + period
            self.sleep(max(0., next_tick - self.clock()))


class SignalExit(BaseException):
    def __init__(self, signum): self.signum = signum


def build_parser():
    parser = base_parser()
    parser.description = __doc__
    parser.set_defaults(config=str(DEFAULT_CONFIG), duration=0., log='logs/xarm_demo.jsonl')
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('duration must be finite and nonnegative')
    source = recorder = session = keys = None
    old_signals = {}
    status, reason, cleanup_errors = 0, 'completed', []
    try:
        config = XArmConfig.load(args.config, ip=args.ip)
        if not config.robot['homing']['enabled'] or not config.gripper['enabled']:
            raise ValueError('Demo H requires configured homing and gripper support')
        if not args.dry_run:
            config.require_live_settings()
            if (args.backend or config.reader['backend']) != 'pyspacemouse':
                raise ValueError('Mock input is allowed only with --dry-run')
        print(f'Limits: {min(config.teleop.max_linear_speed_mps, config.site["max_linear_speed_mps"] or config.teleop.max_linear_speed_mps)*1000:g} mm/s, '
              f'{min(config.teleop.max_angular_speed_radps, config.site["max_angular_speed_radps"] or config.teleop.max_angular_speed_radps):g} rad/s; gripper buttons unchanged.', flush=True)
        keys = TerminalKeys()
        keys.open(required=not args.dry_run)  # Before HID, robot, or log initialization.
        if args.dry_run and keys.fd is None and not args.duration:
            raise ValueError('Noninteractive dry-run requires a positive --duration')
        def terminate(signum, frame):
            raise SignalExit(signum)
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            old_signals[sig] = signal.signal(sig, terminate)
        recorder = AsyncJsonlRecorder(args.log)
        recorder.open()
        source = LatestInput(config, backend='mock' if args.dry_run else 'pyspacemouse')
        source.start()
        session = DemoSession(config, source, recorder, keys, dry_run=args.dry_run)
        session.run(args.duration)
    except KeyboardInterrupt:
        status, reason = 130, 'Ctrl-C'
    except SignalExit as exc:
        status, reason = 128 + exc.signum, signal.Signals(exc.signum).name
    except Exception as exc:
        status, reason = 1, str(exc)
    finally:
        for sig in old_signals:
            signal.signal(sig, signal.SIG_IGN)
        # Do not let terminal/HID/log cleanup delay an arm stop attempt.
        for label, component in (('robot', session), ('input', source), ('terminal', keys)):
            if component is not None:
                try:
                    component.close()
                except Exception as exc:
                    cleanup_errors.append(f'{label}: {exc}')
        if cleanup_errors and status == 0:
            status, reason = 1, 'cleanup failed'
        if recorder is not None:
            try:
                recorder.write({'event': 'session_end', 'timestamp': time.monotonic(), 'reason': reason,
                                'status': status, 'cleanup_errors': list(cleanup_errors),
                                'phase': None if session is None else session.phase,
                                'active_elapsed_s': None if session is None else session.active_elapsed})
            except Exception as exc:
                cleanup_errors.append(f'final log: {exc}')
                status = status or 1
            try:
                recorder.close()
            except Exception as exc:
                cleanup_errors.append(f'log close: {exc}')
                status = status or 1
        for sig, handler in old_signals.items():
            signal.signal(sig, handler)
    if status:
        print(f'Demo stopped: {reason}', file=sys.stderr)
    for error in cleanup_errors:
        print(f'Cleanup: {error}', file=sys.stderr)
    return status


if __name__ == '__main__':
    raise SystemExit(main())
