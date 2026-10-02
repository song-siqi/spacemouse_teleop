"""One absolute, controller-planned TCP move; independent of teleop and recording."""
from __future__ import annotations

import argparse
import contextlib
import json
import math
from pathlib import Path
import signal
import sys
import threading
import time
from uuid import uuid4


REPO_ROOT = Path(__file__).resolve().parents[3]
SDK_VERSION = "1.18.4"
REPORT_TIMEOUT = 1.0
SETUP_TIMEOUT = 5.0
PERIOD = 0.05
SETTLE_TIME = 0.3


class MoveError(RuntimeError):
    pass


class Terminated(BaseException):
    pass


def checked(code, name):
    if code != 0:
        raise MoveError(f"{name} failed: SDK code {code}")


def query(result, name):
    if not isinstance(result, (list, tuple)) or len(result) != 2:
        raise MoveError(f"{name}: malformed SDK response")
    checked(result[0], name)
    return result[1]


def vector(values, count, name):
    if not isinstance(values, (tuple, list)) or len(values) != count:
        raise MoveError(f"{name}: expected {count} values")
    if not all(isinstance(v, (float, int)) and not isinstance(v, bool) and math.isfinite(v) for v in values):
        raise MoveError(f"{name}: nonfinite or invalid value")
    return list(values)


def orientation_error_deg(a, b):
    def rotation(rpy):
        r, p, y = map(math.radians, rpy)
        cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
        return (cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr,
                sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr, -sp, cp*sr, cp*cr)
    cosine = (sum(x*y for x, y in zip(rotation(a), rotation(b))) - 1) / 2
    return math.degrees(math.acos(max(-1., min(1., cosine))))


def build_parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--pose', nargs=6, type=float, required=True, metavar=('X', 'Y', 'Z', 'ROLL', 'PITCH', 'YAW'),
                   help='Absolute base-frame TCP pose: mm and degrees; zero TCP/world offsets required.')
    p.add_argument('--ip', default='192.168.1.209')
    p.add_argument('--speed', type=float, default=10., help='SDK Cartesian speed, mm/s (not an angular speed limit).')
    p.add_argument('--accel', type=float, default=50., help='SDK Cartesian acceleration, mm/s^2.')
    p.add_argument('--timeout', type=float, default=120., help='Motion deadline, seconds.')
    p.add_argument('--position-tolerance', type=float, default=2., help='Final XYZ distance tolerance, mm.')
    p.add_argument('--orientation-tolerance', type=float, default=1., help='Final rotation angle tolerance, degrees.')
    p.add_argument('--log', help='New JSONL file; existing paths are refused.')
    p.add_argument('--dry-run', action='store_true', help='Offline argument/call preview only; no reachability check.')
    return p


def validate_args(args):
    vector(args.pose, 6, 'pose')
    for name in ('speed', 'accel', 'timeout', 'position_tolerance', 'orientation_tolerance'):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            raise MoveError(f'{name} must be finite and positive')
    if args.orientation_tolerance >= 180:
        raise MoveError('orientation_tolerance must be less than 180 degrees')
    if not args.ip.strip():
        raise MoveError('ip must not be empty')


def move_kwargs(args):
    return dict(zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'), args.pose),
                speed=args.speed, mvacc=args.accel, relative=False, is_radian=False, radius=-1, wait=False)


class JsonLog:
    def __init__(self, path=None):
        self.path = Path(path) if path else REPO_ROOT / 'logs' / f'xarm_move_to_pose_{time.time_ns()}_{uuid4().hex[:8]}.jsonl'
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.path.open('x', encoding='utf-8')  # Also refuses existing/broken symlinks.

    def write(self, event, **fields):
        self.file.write(json.dumps(dict(event=event, monotonic=time.monotonic(), realtime_ns=time.time_ns(), **fields),
                                   allow_nan=False) + '\n')
        self.file.flush()

    def close(self):
        self.file.close()


class SDKRobot:
    """All dependencies on pinned SDK internals stay inside this adapter."""
    def __init__(self, ip):
        from importlib.metadata import version
        if version('xarm-python-sdk') != SDK_VERSION:
            raise MoveError(f'This script requires xarm-python-sdk=={SDK_VERSION}')
        with contextlib.redirect_stdout(sys.stderr):
            from xarm.wrapper import XArmAPI
        self.api = XArmAPI(ip, is_radian=True, do_not_open=True, report_type='rich',
                           timed_comm=False, enable_heartbeat=False, max_callback_thread_count=0,
                           check_is_pause=False, check_cmdnum_limit=False)
        self._lock = threading.Lock()
        self._report = None
        self._report_error = None
        self.api.register_report_callback(self._on_report)

    def _on_report(self, _):
        # Inline SDK report callback: the parser has just consumed this packet.
        # Do not use default-zero SDK properties before a complete rich report.
        try:
            p = self.api._arm.report_data
            if p is None or p.data_size < 312:
                raise MoveError('Rich report lacks required TCP/world configuration fields')
            report = dict(received_at=time.monotonic(), axis=p.robot_axis, robot_type=p.robot_type,
                          mode=p.mode, report_state=p.state, tcp_offset_mm_rad=list(p.tcp_offset),
                          world_offset_mm_rad=list(p.world_offset), payload_kg_com_mm=list(p.tcp_payload),
                          motor_enable=p.motor_enable_status, motor_brake=p.motor_brake_status)
            with self._lock:
                self._report = report
        except Exception as exc:
            with self._lock:
                self._report_error = str(exc)

    def connect(self):
        self.api.set_timeout(1.)
        self.api.connect()
        self.api.set_timeout(1.)
        core = self.api._arm
        from xarm.core.config.x_config import XCONF
        from xarm.x3.code import APIState
        # SDK motion_en otherwise forces a minimum five-second transaction.
        core.arm_cmd.motion_en = lambda axis, enable: core.arm_cmd.set_nu8(
            XCONF.UxbusReg.MOTION_EN, [axis, int(enable)], 2, timeout=1.)
        receive = core.arm_cmd.recv_modbus_response

        def bounded_receive(unit, transaction, count, timeout, *args, **kwargs):
            # move_line_common hardcodes ten seconds even after set_timeout(1).
            return receive(unit, transaction, count, min(timeout, 1.), *args, **kwargs)
        core.arm_cmd.recv_modbus_response = bounded_receive

        def ready_without_wait():
            # set_position's private sync loop has no deadline. We explicitly
            # wait for readiness before sending, and refuse races at the call.
            return 0 if core._is_sync and not core._need_sync else APIState.NOT_READY
        core._XArm__wait_sync = ready_without_wait
        deadline = time.monotonic() + SETUP_TIMEOUT
        while True:
            with self._lock:
                present = self._report is not None or self._report_error is not None
            if present:
                self.report()
                return
            if not self.api.connected or time.monotonic() >= deadline:
                raise MoveError('No initial rich feedback')
            time.sleep(PERIOD)

    def report(self):
        with self._lock:
            result, error = self._report, self._report_error
        if error:
            raise MoveError(error)
        if not self.api.connected or result is None or not 0 <= time.monotonic() - result['received_at'] <= REPORT_TIMEOUT:
            raise MoveError('Robot disconnected or rich feedback stale')
        return dict(result)

    def snapshot(self):
        before = time.monotonic()
        pose = vector(query(self.api.get_position(is_radian=False), 'get_position'), 6, 'actual pose')
        state = query(self.api.get_state(), 'get_state')
        cmdnum = query(self.api.get_cmdnum(), 'get_cmdnum')
        errors = vector(query(self.api.get_err_warn_code(), 'get_err_warn_code'), 2, 'error/warning')
        report = self.report()
        return dict(report, pose_mm_deg=pose, state=state, cmdnum=cmdnum, error_code=errors[0], warn_code=errors[1],
                    query_before=before, query_after=time.monotonic())

    def initial_details(self):
        joints = query(self.api.get_servo_angle(is_radian=True), 'get_servo_angle')
        if not isinstance(joints, (list, tuple)) or len(joints) not in (6, 7):
            raise MoveError('Malformed joint feedback')
        vector(joints, len(joints), 'joints')
        return dict(joints_rad=list(joints[:6]), firmware=query(self.api.get_version(), 'get_version'), sdk_version=SDK_VERSION)

    def preflight(self, pose):
        joints = query(self.api.get_inverse_kinematics(pose, input_is_radian=False, return_is_radian=True), 'inverse kinematics')
        if not isinstance(joints, (tuple, list)) or len(joints) not in (6, 7):
            raise MoveError('Inverse kinematics returned malformed joints')
        vector(joints, len(joints), 'inverse kinematics')
        limited = query(self.api.is_joint_limit(joints, is_radian=True), 'joint limit check')
        if limited not in (False, 0):
            raise MoveError('Target joint solution violates joint limits')
        return dict(target_joints_rad=list(joints[:6]), target_only=True, path_checked=False)

    def activate(self):
        checked(self.api.motion_enable(enable=True), 'motion_enable')
        checked(self.api.set_mode(0), 'set_mode')
        checked(self.api.set_state(0), 'set_state')

    def ready(self):
        core = self.api._arm
        return core._is_sync and not core._need_sync

    def move(self, kwargs):
        core = self.api._arm
        # High-level SDK silently clamps these values. Refuse a different speed
        # than requested, especially an increase above a requested slow speed.
        if not core._min_tcp_speed <= kwargs['speed'] <= 1000:
            raise MoveError('Requested speed is outside SDK supported range')
        if not core._min_tcp_acc <= kwargs['mvacc'] <= 50000:
            raise MoveError('Requested acceleration is outside SDK supported range')
        return self.api.set_position(**kwargs)

    def stop(self):
        checked(self.api.set_state(4), 'stop')
        state = query(self.api.get_state(), 'confirm stop')
        if state != 4:
            raise MoveError(f'Stop not confirmed: state={state}')

    def close(self):
        self.api.disconnect()


def check_snapshot(s, *, stationary=False, running=False):
    if s['axis'] != 6 or s['robot_type'] != 6:
        raise MoveError('Connected robot is not xArm6')
    vector(s['pose_mm_deg'], 6, 'actual pose')
    vector(s['payload_kg_com_mm'], 4, 'payload')
    for name in ('tcp_offset_mm_rad', 'world_offset_mm_rad'):
        if any(abs(v) > 1e-6 for v in vector(s[name], 6, name)):
            raise MoveError(f'{name} must be zero; robot configuration was not changed')
    for name in ('state', 'mode', 'cmdnum', 'error_code', 'warn_code'):
        if not isinstance(s[name], int) or s[name] < 0:
            raise MoveError(f'Invalid {name}')
    if s['error_code'] or s['warn_code']:
        raise MoveError(f"Robot error={s['error_code']}, warning={s['warn_code']}; not cleared automatically")
    if stationary and (s['state'] not in (0, 2, 4, 5) or s['cmdnum'] != 0):
        raise MoveError('Robot must be stationary with no pending motion')
    if running and (s['mode'] != 0 or s['state'] not in (0, 1, 2)
                    or s['motor_enable'] & 63 != 63 or s['motor_brake'] & 63 != 63):
        raise MoveError('Robot left enabled position control')


class MoveSession:
    def __init__(self, robot, log, clock=time.monotonic, sleep=time.sleep):
        self.robot, self.log, self.clock, self.sleep = robot, log, clock, sleep
        self.may_have_motion = False

    def run(self, args):
        self.robot.connect()
        initial = self.robot.snapshot()
        self.log.write('initial', snapshot=initial, **self.robot.initial_details())
        check_snapshot(initial, stationary=True)
        self.log.write('preflight', **self.robot.preflight(args.pose))
        # Recheck after preflight: no blind mode switch based on an old snapshot.
        check_snapshot(self.robot.snapshot(), stationary=True)
        self.robot.activate()
        activated_at = self.clock()
        while True:
            s = self.robot.snapshot()
            check_snapshot(s, stationary=True)
            if (s['received_at'] >= activated_at and s['mode'] == 0 and s['state'] in (0, 2)
                    and s['motor_enable'] & 63 == 63 and s['motor_brake'] & 63 == 63 and self.robot.ready()):
                break
            if self.clock() - activated_at >= SETUP_TIMEOUT:
                raise MoveError('Position mode activation not confirmed')
            self.sleep(PERIOD)
        self.log.write('activated', snapshot=s)
        kwargs = move_kwargs(args)
        self.log.write('move_call', kwargs=kwargs)
        deadline = self.clock() + args.timeout
        self.may_have_motion = True  # Set BEFORE SDK call: an ACK may be lost.
        code = self.robot.move(kwargs)
        self.log.write('move_return', return_code=code)
        checked(code, 'set_position')
        settled_since = None
        while True:
            cycle_started = self.clock()
            if self.clock() >= deadline:
                raise MoveError('Motion timed out before verified arrival')
            s = self.robot.snapshot()
            if self.clock() >= deadline:
                raise MoveError('Motion timed out while querying feedback')
            check_snapshot(s, running=True)
            position_error = math.dist(s['pose_mm_deg'][:3], args.pose[:3])
            rotation_error = orientation_error_deg(s['pose_mm_deg'][3:], args.pose[3:])
            # State 0 means START, not completion. Require STOP(2) and empty queue.
            arrived = (s['state'] == 2 and s['cmdnum'] == 0
                       and position_error <= args.position_tolerance and rotation_error <= args.orientation_tolerance)
            now = self.clock()
            settled_since = (now if settled_since is None else settled_since) if arrived else None
            self.log.write('monitor', snapshot=s, position_error_mm=position_error,
                           orientation_error_deg=rotation_error, arrived=arrived)
            if settled_since is not None and now - settled_since >= SETTLE_TIME:
                self.log.write('completed', snapshot=s, position_error_mm=position_error, orientation_error_deg=rotation_error)
                self.may_have_motion = False
                return s
            self.sleep(max(0., min(cycle_started + PERIOD, deadline) - self.clock()))


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        validate_args(args)
    except MoveError as exc:
        parser.error(str(exc))
    robot = log = session = None
    status, reason = 0, 'completed'
    old_signals = {}

    def terminate(signum, _frame):
        if signum == signal.SIGINT:
            raise KeyboardInterrupt
        raise Terminated

    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            old_signals[sig] = signal.signal(sig, terminate)
        log = JsonLog(args.log)
        log.write('session_start', arguments=vars(args), frame='robot_base', pose_units='mm,deg',
                  orientation='RPY (Rz(yaw) Ry(pitch) Rx(roll))', dry_run=args.dry_run)
        print(f'Log: {log.path.resolve()}')
        if args.dry_run:
            preview = move_kwargs(args)
            log.write('dry_run', sdk_call='set_position', kwargs=preview, reachability_checked=False)
            print('Dry-run: no hardware connection; reachability and path are NOT checked.')
            print('set_position(' + json.dumps(preview) + ')')
        else:
            robot = SDKRobot(args.ip)
            session = MoveSession(robot, log)
            final = session.run(args)
            print(f"Arrived: {final['pose_mm_deg']} (mm, deg)")
    except KeyboardInterrupt:
        status, reason = 130, 'Ctrl-C'
    except Terminated:
        status, reason = 143, 'SIGTERM'
    except Exception as exc:
        status, reason = 1, str(exc)
    finally:
        # A repeated signal must not skip the stop/close path.
        for sig in old_signals:
            signal.signal(sig, signal.SIG_IGN)
        cleanup_errors = []
        if session is not None and session.may_have_motion:
            stop_error = None
            try:
                robot.stop()
            except Exception as exc:
                stop_error = str(exc)
                cleanup_errors.append(f'Stop NOT confirmed: {exc}')
            try:
                log.write('stop', confirmed=stop_error is None, error=stop_error)
            except Exception as exc:
                cleanup_errors.append(f'Stop log: {exc}')
        if robot is not None:
            try:
                robot.close()
            except Exception as exc:
                cleanup_errors.append(f'Disconnect: {exc}')
        if cleanup_errors and status == 0:
            status = 1
        if log is not None:
            try:
                log.write('session_end', status=status, reason=reason, cleanup_errors=cleanup_errors)
            except Exception as exc:
                cleanup_errors.append(f'Final log: {exc}')
                status = status or 1
            try:
                log.close()
            except Exception as exc:
                cleanup_errors.append(f'Log close: {exc}')
                status = status or 1
        for sig, handler in old_signals.items():
            signal.signal(sig, handler)
        if status:
            print(f'Move failed: {reason}', file=sys.stderr)
        for error in cleanup_errors:
            print(error, file=sys.stderr)
    return status


if __name__ == '__main__':
    raise SystemExit(main())
