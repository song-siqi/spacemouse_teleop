"""Independent downward-facing XYZ teleop using the unchanged SDK loop."""
from __future__ import annotations

import argparse
from dataclasses import replace
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
import uuid

from robot_teleop.backends.real.xarm_sdk.backend import XArmBackend
from robot_teleop.backends.real.xarm_sdk.config import XArmConfig
from robot_teleop.inputs.spacemouse.polling import LatestInput
from robot_teleop.backends.real.xarm_sdk.safety import TeleopFault
from robot_teleop.cli.xarm_teleop import run_loop
from robot_teleop.recording.async_jsonl import AsyncJsonlRecorder

REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CONFIG = REPO_ROOT / 'configs/spacemouse_xarm6_translation.json'
DOWNWARD_RPY_RAD = (math.pi, 0., 0.)


def load_profile(path=DEFAULT_CONFIG, ip=None):
    config = XArmConfig.load(path, ip=ip)
    settings = json.loads(Path(path).read_text(encoding='utf-8')).get('orientation_guard')
    if not isinstance(settings, dict) or set(settings) != {'tolerance_deg'}:
        raise ValueError('Translation config requires orientation_guard.tolerance_deg')
    tolerance = settings['tolerance_deg']
    if (isinstance(tolerance, bool) or not isinstance(tolerance, (int, float))
            or not math.isfinite(tolerance) or not 0 < tolerance < 90):
        raise ValueError('orientation_guard.tolerance_deg must be finite and between 0 and 90 degrees')
    if any(rule.source not in ('x', 'y', 'z') for rule in config.teleop.linear_mapping.values()):
        raise ValueError('Translation mapping may use only SpaceMouse x/y/z')
    if any(rule.source not in ('roll', 'pitch', 'yaw') for rule in config.teleop.angular_mapping.values()):
        raise ValueError('Angular mapping must use the rotation axes that will be masked')
    if config.teleop.translation_only_button is not None or config.teleop.rotation_only_button is not None:
        raise ValueError('Translation profile reserves buttons for the gripper, not mode switching')
    if not config.robot['homing']['enabled'] or not config.gripper['enabled']:
        raise ValueError('Translation profile requires startup homing and the official gripper')
    if (config.site['expected_tcp_offset_mm_rad'] is None
            or config.site['expected_tcp_offset_mm_rad'][3:] != [0] * 3):
        raise ValueError('Translation profile requires zero TCP rotation offset for its downward convention')
    return config, float(tolerance)


def rotation_matrix(rpy):
    """Base-frame RPY convention: Rz(yaw) Ry(pitch) Rx(roll)."""
    if len(rpy) != 3 or not all(math.isfinite(v) for v in rpy):
        raise TeleopFault('Invalid RPY orientation feedback')
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
            (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr),
            (-sp, cp*sr, cp*cr))


def rotation_error_deg(reference, actual):
    # Frobenius inner product equals trace(reference.T @ actual).
    trace = sum(a*b for ra, rb in zip(reference, actual) for a, b in zip(ra, rb))
    return math.degrees(math.acos(max(-1., min(1., (trace-1.)/2.))))


class OrientationGuard:
    def __init__(self, tolerance_deg=3.):
        self.tolerance_deg = tolerance_deg
        self.downward = rotation_matrix(DOWNWARD_RPY_RAD)
        self.reference = self.reference_rpy = None
        self.last = None

    def check(self, feedback, phase='teleop'):
        rpy = tuple(feedback.tcp_mm_rad[3:])
        actual = rotation_matrix(rpy)
        downward_error = rotation_error_deg(self.downward, actual)
        hold_error = None if self.reference is None else rotation_error_deg(self.reference, actual)
        self.last = {'phase': phase, 'source': 'existing_sdk_rich_report', 'frame': 'link_base',
                     'representation': 'RPY radians, Rz(yaw) Ry(pitch) Rx(roll)',
                     'feedback_sequence': feedback.sequence, 'feedback_received_at': feedback.received_at,
                     'checked_at': time.monotonic(), 'target_rpy_rad': DOWNWARD_RPY_RAD,
                     'reference_rpy_rad': self.reference_rpy, 'actual_rpy_rad': rpy,
                     'downward_error_deg': downward_error, 'hold_error_deg': hold_error,
                     'tolerance_deg': self.tolerance_deg, 'active_correction': False}
        # Small numerical allowance only for the acos calculation at the boundary.
        if downward_error > self.tolerance_deg + 1e-9:
            raise TeleopFault(f'Downward orientation exceeded tolerance: {downward_error:.4f} > {self.tolerance_deg:g} deg')
        if hold_error is not None and hold_error > self.tolerance_deg + 1e-9:
            raise TeleopFault(f'Held orientation drift exceeded tolerance: {hold_error:.4f} > {self.tolerance_deg:g} deg')
        return feedback

    def capture_reference(self, feedback):
        if self.reference is not None:
            raise TeleopFault('The held orientation reference cannot be updated during a run')
        self.check(feedback, 'after_activation')
        self.reference_rpy = tuple(feedback.tcp_mm_rad[3:])
        self.reference = rotation_matrix(self.reference_rpy)
        self.check(feedback, 'reference_captured')


def require_translation_velocity(speeds):
    if len(speeds) != 6 or not all(math.isfinite(v) for v in speeds):
        raise TeleopFault('Translation command must contain six finite speeds')
    if any(v != 0. for v in speeds[3:]):
        raise TeleopFault('Translation-only invariant violated: nonzero angular velocity')


class TranslationBackend(XArmBackend):
    """Guard inherited feedback, including the check immediately before sending."""
    def __init__(self, config, guard, **kwargs):
        super().__init__(config, **kwargs)
        self.guard = guard
        self._monitor_orientation = False

    def activate(self):
        # Homing may rotate freely; the guard starts only at Cartesian activation.
        self.guard.check(super().feedback(running=False), 'before_activation')
        self._monitor_orientation = True
        try:
            super().activate()
            self.guard.capture_reference(self.feedback())
        except BaseException:
            self.stop()
            raise

    def feedback(self, running=True):
        feedback = super().feedback(running=running)
        if self._monitor_orientation:
            self.guard.check(feedback)
        return feedback

    def send(self, speeds, expires_at, intent='hold'):
        require_translation_velocity(speeds)
        if self.guard.reference is None:
            raise TeleopFault('Translation teleop requires a verified orientation reference')
        return super().send(speeds, expires_at, intent)


class TranslationInput:
    """Project only after the original loop has accepted six-axis neutral input."""
    def __init__(self, source):
        self.source = source
        self.active = False
        self.last_raw = self.last_effective = None

    def start(self):
        self.source.start()

    def snapshot(self, now):
        raw, intent = self.source.snapshot(now)
        self.last_raw = raw
        if raw is None:
            self.last_effective = None
            return None, intent
        # Never hide corrupt raw axes by replacing them with zeros.
        if not all(math.isfinite(v) for v in (raw.x, raw.y, raw.z, raw.roll, raw.pitch, raw.yaw, raw.timestamp)):
            raise TeleopFault('SpaceMouse returned nonfinite input')
        if not raw.connected:
            raise TeleopFault('SpaceMouse disconnected')
        effective = replace(raw, roll=0., pitch=0., yaw=0.) if self.active else raw
        # All original sequence, connection and per-axis time fields are retained.
        self.last_effective = effective
        return effective, intent

    def close(self):
        self.source.close()


def json_safe(value):
    """Represent invalid fault evidence as null rather than nonstandard JSON NaN."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_safe(v) for v in value]
    return value


class TranslationRecorder:
    def __init__(self, sink, source, guard, dry_run=False):
        self.sink, self.source, self.guard = sink, source, guard
        self.dry_run = dry_run

    def open(self):
        self.sink.open()

    def write(self, event):
        row = dict(event)
        if row.get('event') == 'session_start':
            row['translation_profile'] = {'target_rpy_rad': DOWNWARD_RPY_RAD,
                'orientation_tolerance_deg': self.guard.tolerance_deg,
                'reference_policy': 'capture once after activation', 'active_correction': False,
                'feedback_policy': 'existing rich report cadence; no additional report connection'}
        if row.get('event') == 'teleop_started':
            if not self.dry_run and self.guard.reference is None:
                raise TeleopFault('Cannot start input projection without an orientation reference')
            self.source.active = True
        if 'raw' in row:
            effective = row.pop('raw')
            if self.source.last_effective is None or effective != self.source.last_effective.to_dict():
                raise TeleopFault('Input log provenance does not match the control snapshot')
            row['raw_input'] = self.source.last_raw.to_dict()
            row['effective_input'] = effective
            command = row['command']
            if any(v != 0. for name in ('angular_vel_radps', 'delta_rot_rad') for v in command[name]):
                raise TeleopFault('Translation-only command contains a rotation component')
            execution = row['execution']
            require_translation_velocity(execution.get('speeds_mm_s_rad_s', execution.get('would_send_mm_s_rad_s')))
            row['mode'] = 'xarm-translation-dry-run' if self.dry_run else 'xarm-translation-velocity'
        row['orientation'] = None if self.guard.last is None else dict(self.guard.last)
        row['orientation_validation'] = 'not_evaluated_dry_run' if self.dry_run else 'feedback_guard'
        self.sink.write(row)

    def close(self):
        self.sink.close()


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(DEFAULT_CONFIG))
    parser.add_argument('--ip')
    parser.add_argument('--duration', type=float, default=0., help='Seconds after homing; 0 runs until Ctrl-C')
    parser.add_argument('--log', help='New JSONL path; existing paths are refused')
    parser.add_argument('--dry-run', action='store_true', help='Synthetic input only; no robot, HID, cameras or collection')
    return parser


def new_log_path(requested=None):
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    path = Path(requested).expanduser() if requested else REPO_ROOT / 'logs' / f'xarm_translation_{stamp}_{os.getpid()}_{uuid.uuid4().hex[:8]}.jsonl'
    if path.exists() or path.is_symlink():
        raise FileExistsError(f'Refusing an existing log path: {path}')
    path = path.resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    # The shared recorder appends, so reserve a new file before handing it over.
    # This also refuses existing demo logs or symlink destinations.
    with path.open('x', encoding='utf-8'):
        pass
    return path


def main(argv=None):
    args = build_parser().parse_args(argv)
    source = backend = recorder = None
    guard = None
    status, reason, cleanup_errors = 0, 'completed', []
    old_term = None
    try:
        if not math.isfinite(args.duration) or args.duration < 0:
            raise ValueError('duration must be finite and nonnegative')
        config, tolerance = load_profile(args.config, args.ip)
        if not args.dry_run:
            config.require_live_settings()
            if config.reader['backend'] != 'pyspacemouse':
                raise ValueError('Live translation teleop requires pyspacemouse input')
        guard = OrientationGuard(tolerance)
        path = new_log_path(args.log)
        print(f'Translation-only log: {path}', flush=True)
        linear_limit = min(config.teleop.max_linear_speed_mps, config.site['max_linear_speed_mps'] or config.teleop.max_linear_speed_mps)
        print(f'Target base RPY: [180, 0, 0] deg; orientation tolerance: {tolerance:g} deg; no active correction.')
        print(f'XYZ limit: {linear_limit*1000:g} mm/s; angular command: exactly zero; gripper buttons unchanged.')
        if args.dry_run:
            print('Dry-run: synthetic input; no hardware. Orientation tracking is not verified.')
        else:
            print('Startup opens the gripper and homes joints; after six-axis neutral, downward orientation is verified before teleop.')
        if hasattr(signal, 'SIGTERM'):
            old_term = signal.getsignal(signal.SIGTERM)
            def terminate(signum, frame):
                raise KeyboardInterrupt
            signal.signal(signal.SIGTERM, terminate)
        source = TranslationInput(LatestInput(config, backend='mock' if args.dry_run else 'pyspacemouse'))
        recorder = TranslationRecorder(AsyncJsonlRecorder(path), source, guard, args.dry_run)
        recorder.open()
        if not args.dry_run:
            backend = TranslationBackend(config, guard)
            backend.connect()
        source.start()
        # Reuse the existing lifecycle, neutral handling, limiter and finite sends.
        run_loop(config, source, backend, recorder, args.duration, args.dry_run)
    except KeyboardInterrupt:
        reason = 'interrupted'
        print('Stopped.')
    except Exception as exc:
        status, reason = 1, str(exc)
        print(f'Translation teleop failed: {exc}', file=sys.stderr)
    finally:
        # No input join or log flush precedes the robot stop attempt.
        if backend:
            try:
                cleanup_errors.extend(backend.close() or [])
            except Exception as exc:
                cleanup_errors.append(f'Robot cleanup: {exc}')
        if source:
            try:
                source.close()
            except Exception as exc:
                cleanup_errors.append(f'Input cleanup: {exc}')
        if cleanup_errors:
            status = 1
            print('Cleanup failed: ' + '; '.join(cleanup_errors), file=sys.stderr)
        if recorder:
            try:
                try:
                    recorder.write({'event': 'session_end', 'timestamp': time.monotonic(),
                        'reason': reason, 'status': status, 'cleanup_errors': cleanup_errors,
                        'last_raw_input': json_safe(source.last_raw.to_dict()) if source.last_raw else None})
                finally:
                    recorder.close()
            except Exception as exc:
                status = 1
                print(f'Log cleanup failed: {exc}', file=sys.stderr)
        if old_term is not None:
            signal.signal(signal.SIGTERM, old_term)
    return status


if __name__ == '__main__':
    raise SystemExit(main())
