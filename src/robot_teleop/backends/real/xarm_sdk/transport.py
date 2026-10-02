"""Pinned SDK codecs over separate, bounded sockets; no XArmAPI.connect side effects.

Read-only sessions expose only a query whitelist. Rich reports are received on
30002 without subscribing, enabling, clearing warnings, or changing controller
configuration. Motion/gripper channels each own a UxbusCmdTcp and its lock.
"""
from __future__ import annotations

import re
import socket
import struct
import threading
import time

from .safety import RobotFeedback, TeleopFault


def sdk_types():
    try:
        import contextlib
        import sys
        # The pinned SDK prints its version on import; keep CLI JSON on stdout.
        with contextlib.redirect_stdout(sys.stderr):
            from xarm.core.wrapper import UxbusCmdTcp
            from xarm.x3.report import ReportDataStructure
    except ImportError as exc:
        raise RuntimeError("Install the xarm extra: uv pip install -e '.[xarm]'") from exc
    return UxbusCmdTcp, ReportDataStructure


class PacketSocket:
    """Minimal SDK port contract, assembling fragmented/coalesced MBAP replies."""
    def __init__(self, ip, timeout, port=502):
        self.timeout = timeout
        self.socket = socket.create_connection((ip, port), timeout=timeout)
        self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.buffer = bytearray()
        self.connected = True

    def flush(self):
        # Keep partial replies; SDK transaction IDs discard older responses.
        return 0

    def write(self, data):
        try:
            self.socket.settimeout(self.timeout)
            self.socket.sendall(data)
        except OSError as exc:
            self.close()
            raise TeleopFault(f"Robot send failed: {exc}") from exc
        return 0

    def read(self, timeout=None):
        deadline = time.monotonic() + min(self.timeout, timeout or self.timeout)
        while self.connected:
            if len(self.buffer) >= 6:
                length = 6 + struct.unpack_from(">H", self.buffer, 4)[0]
                if not 8 <= length <= 4096:
                    raise TeleopFault(f"Invalid SDK reply length: {length}")
                if len(self.buffer) >= length:
                    reply = bytes(self.buffer[:length])
                    del self.buffer[:length]
                    return reply
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return -1
            try:
                self.socket.settimeout(remaining)
                data = self.socket.recv(4096)
                if not data:
                    self.close()
                    raise TeleopFault("Robot control socket disconnected")
                self.buffer.extend(data)
            except socket.timeout:
                return -1
            except OSError as exc:
                self.close()
                raise TeleopFault(f"Robot receive failed: {exc}") from exc
        raise TeleopFault("Robot control socket closed")

    def close(self):
        self.connected = False
        try:
            self.socket.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.socket.close()


def checked(result, name, allow_robot_fault=False):
    # SDK private protocol codes 1/2 indicate a valid reply with robot error/warn.
    if not result or result[0] not in ((0, 1, 2) if allow_robot_fault else (0,)):
        raise TeleopFault(f"{name} failed: SDK code {result[0] if result else 'empty reply'}")
    return result[1:]


class SDKChannel:
    READS = frozenset(("get_version", "get_state", "get_err_code", "get_joint_pos", "get_tcp_pose"))

    def __init__(self, ip, timeout, writable=False, port_factory=PacketSocket):
        codec, _ = sdk_types()
        self.port = port_factory(ip, timeout)
        self.core = codec(self.port)
        self.core.set_timeout(timeout)  # Changes local SDK deadlines only.
        self.writable = writable

    def query(self, name):
        if name not in self.READS:
            raise ValueError(f"Not a read-only query: {name}")
        return checked(getattr(self.core, name)(), name, allow_robot_fault=True)

    def identity(self):
        value = bytes(self.query("get_version")).split(b"\0", 1)[0].decode("ascii", errors="replace")
        match = re.search(r"[vV](\d+)\.(\d+)\.(\d+)", value)
        return {"version_string": value, "firmware": tuple(map(int, match.groups())) if match else None}

    def _write(self, name, *args, **kwargs):
        if not self.writable:
            raise TeleopFault("Control write rejected by read-only SDK session")
        return checked(getattr(self.core, name)(*args, **kwargs), name)

    def activate(self, max_accel_mps2):
        self.control_mode = 5
        self._write("set_tcp_maxacc", max_accel_mps2 * 1000)
        # motion_en() forces >=5s in SDK; use its exact encoding with our deadline.
        from xarm.core.config.x_config import XCONF
        self._write("set_nu8", XCONF.UxbusReg.MOTION_EN, [8, 1], 2)
        self._write("set_mode", 5)
        self._write("set_state", 0)

    def activate_homing(self):
        from xarm.core.config.x_config import XCONF
        self.control_mode = 4
        self._write("set_nu8", XCONF.UxbusReg.MOTION_EN, [8, 1], 2)
        self._write("set_mode", 4)
        self._write("set_state", 0)

    def joint_velocity(self, speeds, duration):
        import math
        if not math.isfinite(duration) or duration <= 0 or len(speeds) != 6 or not all(math.isfinite(v) for v in speeds):
            raise TeleopFault("Joint velocity requires six finite values and a positive finite duration")
        self._write("vc_set_jointv", list(speeds) + [0.], 1, duration=duration)

    def velocity(self, speeds, duration):
        import math
        if not math.isfinite(duration) or duration <= 0 or len(speeds) != 6 or not all(math.isfinite(v) for v in speeds):
            raise TeleopFault("Velocity packets require six finite values and a positive finite duration")
        # Official vc_set_cartesian_velocity reaches this codec with mm/s, rad/s.
        self._write("vc_set_linev", list(speeds), 0, duration=duration)

    def stop(self):
        errors = []
        try:
            send_zero = self.joint_velocity if getattr(self, "control_mode", 5) == 4 else self.velocity
            send_zero((0.,) * 6, .1)
        except Exception as exc:
            errors.append(str(exc))
        try:
            self._write("set_state", 4)
        except Exception as exc:
            errors.append(str(exc))
        return errors

    def gripper_baudrate(self):
        from xarm.core.config.x_config import XCONF
        result = checked(self.core.tgpio_addr_r16(XCONF.ServoConf.MODBUS_BAUDRATE & 0x0FFF), "read tool baudrate")
        index = int(result[0])
        if not 0 <= index < len(self.core.BAUDRATES):
            raise TeleopFault("Unknown tool baudrate")
        return self.core.BAUDRATES[index]

    def _gripper_ack(self, name, value):
        from xarm.core.config.x_config import XCONF
        result = self._write(name, value)
        # SDK's gripper setters return host-id + Modbus write-multiple response.
        addresses = {
            "gripper_modbus_set_mode": (XCONF.ServoConf.CON_MODE, 1),
            "gripper_modbus_set_en": (XCONF.ServoConf.CON_EN, 1),
            "gripper_modbus_set_posspd": (XCONF.ServoConf.POS_SPD, 1),
            "gripper_modbus_set_pos": (XCONF.ServoConf.TAGET_POS, 2),
        }
        address, count = addresses[name]
        expected = [9, 8, 16, address >> 8, address & 255, 0, count]
        if result != expected:
            raise TeleopFault(f"Invalid gripper acknowledgement for {name}: {result}")

    def initialize_gripper(self, config):
        if not self.writable:
            raise TeleopFault("Gripper initialization requires a live session")
        if self.gripper_baudrate() != config["baudrate"]:
            raise TeleopFault("Tool baudrate differs from gripper config; configure it in Studio first")
        status = self.gripper_status()
        if status["error_code"]:
            raise TeleopFault(f"Gripper error: {status['error_code']}")
        self._gripper_ack("gripper_modbus_set_mode", 0)
        self._gripper_ack("gripper_modbus_set_en", 1)
        self._gripper_ack("gripper_modbus_set_posspd", int(config["speed_pulse_s"]))

    def gripper_target(self, pulse):
        # Direct codec: no wait_move, baud auto-setting, auto-enable, or wait-for-goal.
        self._gripper_ack("gripper_modbus_set_pos", int(pulse))

    def gripper_status(self):
        # Same register as SDK get_gripper_status, without its auto-baud setter.
        motion = checked(self.core.gripper_modbus_r16s(0x0000, 1), "gripper motion status")
        if len(motion) != 6 or motion[:4] != [9, 8, 3, 2]:
            raise TeleopFault("Invalid gripper motion status response")
        status_code = (motion[4] << 8) | motion[5]
        from xarm.core.config.x_config import XCONF
        target = checked(self.core.gripper_modbus_r16s(XCONF.ServoConf.TAGET_POS, 2), "gripper target readback")
        if len(target) != 8 or target[:4] != [9, 8, 3, 4]:
            raise TeleopFault("Invalid gripper target readback response")
        controller_target = int.from_bytes(bytes(target[4:]), 'big', signed=True)
        error = checked(self.core.gripper_modbus_get_errcode(), "gripper error")
        # Read position last: release/hold should use the most recent measurement.
        position = checked(self.core.gripper_modbus_get_pos(), "gripper position")
        if len(position) != 1 or len(error) != 1:
            raise TeleopFault("Invalid gripper status response")
        return {"position_pulse": position[0], "error_code": error[0],
                "status_code": status_code, "controller_target_pulse": controller_target,
                "motion_state": {0: "stopped", 1: "moving", 2: "grasped"}.get(status_code & 3, "unknown"),
                "received_at": time.monotonic()}

    def close(self):
        self.port.close()


class ReportReceiver:
    def __init__(self, ip, timeout):
        self.ip, self.timeout = ip, timeout
        self.latest = None
        self.error = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.socket = None
        self.thread = None

    def start(self):
        _, report_type = sdk_types()
        self.parser = report_type.create(30002, is_radian=True)
        self.socket = socket.create_connection((self.ip, 30002), timeout=self.timeout)
        self.socket.settimeout(self.timeout)
        self.thread = threading.Thread(target=self._run, name="xarm-feedback", daemon=True)
        self.thread.start()

    def _run(self):
        buffer = bytearray()
        sequence = 0
        try:
            while not self._stop.is_set():
                try:
                    chunk = self.socket.recv(8192)
                except socket.timeout:
                    continue
                if not chunk:
                    raise TeleopFault("Robot report socket disconnected")
                received = time.monotonic()
                buffer.extend(chunk)
                if len(buffer) > 65536:
                    raise TeleopFault("Robot feedback backlog exceeded limit")
                while len(buffer) >= 4:
                    length = struct.unpack_from(">I", buffer)[0]
                    if not 181 <= length <= 4096:
                        raise TeleopFault(f"Unsupported rich report length: {length}")
                    # SDK supports legacy rich packets advertising 233 but using 245.
                    # This implementation requires modern firmware reports instead.
                    if length == 233:
                        raise TeleopFault("Legacy rich reports unsupported; use a compatible firmware")
                    if len(buffer) < length:
                        break
                    data = bytes(buffer[:length])
                    del buffer[:length]
                    self.parser.update(data)
                    p = self.parser
                    sequence += 1
                    feedback = RobotFeedback(received, sequence, p.state, p.mode, p.error_code, p.warn_code,
                        tuple(p.actual_joint_angle[:6]), tuple(p.actual_tcp_pose), tuple(p.tcp_offset), tuple(p.tcp_payload),
                        p.robot_axis, p.robot_type, p.motor_enable_status, p.motor_brake_status,
                        p.version.decode("ascii", errors="replace") if isinstance(p.version, bytes) else str(p.version))
                    with self._lock:
                        self.latest = feedback
        except Exception as exc:
            if not self._stop.is_set():
                self.error = exc

    def snapshot(self):
        if self.error:
            raise TeleopFault(f"Feedback failed: {self.error}")
        with self._lock:
            return self.latest

    def wait_first(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            value = self.snapshot()
            if value is not None:
                return value
            self._stop.wait(.005)
        raise TeleopFault("No robot feedback received")

    def close(self):
        self._stop.set()
        if self.socket:
            try:
                self.socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self.socket.close()
        if self.thread:
            self.thread.join(self.timeout + .2)
