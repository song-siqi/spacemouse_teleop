"""Passive TCP 30000 acquisition. No SDK initialization or socket writes."""
from __future__ import annotations
import math
import socket
import struct
import threading
import time

PACKET_SIZES = (784, 804)
FIELD_LAYOUT = {
    "target_joints_rad": (32, 7, "rad"),
    "target_joint_velocity_rad_s": (60, 7, "rad/s"),
    "target_joint_acceleration_rad_s2": (88, 7, "rad/s^2"),
    "joints_rad": (116, 7, "rad"),
    "joint_velocity_rad_s": (144, 7, "rad/s"),
    "joint_acceleration_rad_s2": (172, 7, "rad/s^2"),
    "joint_current_a": (200, 7, "A"),
    "estimated_joint_torque_nm": (228, 7, "N m"),
    "target_tcp_mm_rotvec_rad": (424, 6, "mm,rad (rotation vector)"),
    "target_tcp_speed_mm_s_rad_s": (448, 6, "mm/s,rad/s"),
    "tcp_mm_rotvec_rad": (472, 6, "mm,rad (rotation vector)"),
    "tcp_speed_mm_s_rad_s": (496, 6, "mm/s,rad/s"),
    "estimated_tcp_wrench_n_nm": (520, 6, "N,N m"),
    "target_tcp_acceleration_mm_s2_rad_s2": (544, 6, "mm/s^2,rad/s^2"),
    "tcp_acceleration_mm_s2_rad_s2": (568, 6, "mm/s^2,rad/s^2"),
}
REPORT_SEMANTICS = {
    "layout": "xarm_tcp30000_prefix_v1", "supported_packet_bytes": list(PACKET_SIZES),
    "device_timestamp_unit": "us", "device_timestamp_epoch": "unknown",
    "device_timestamp_event": "unknown; not verified as encoder measurement time",
    "measurement_refresh_verified": False, "clock_mapping": "one_way_receive_observations",
    "fixed_offset_uncertainty_s": None, "tcp_frame": "robot base, configured TCP",
    "tcp_orientation": "rotation vector; not RPY",
    "joint_order": [f"joint{i}" for i in range(1, 7)],
    "units": {key: spec[2] for key, spec in FIELD_LAYOUT.items()},
    "reserved_and_unverified_fields": "retained only in original packet",
}

def decode_report(packet):
    if len(packet) not in PACKET_SIZES or struct.unpack_from(">I", packet)[0] != len(packet):
        raise ValueError(f"Unsupported TCP 30000 report length: {len(packet)}")
    values = {"state": packet[12] & 15, "mode": packet[12] >> 4,
              "command_count": struct.unpack_from(">H", packet, 13)[0]}
    invalid = []
    for key, (offset, count, _) in FIELD_LAYOUT.items():
        data = list(struct.unpack_from(f"<{count}f", packet, offset))
        if count == 7:
            data = data[:6]  # The seventh protocol slot remains in the binary.
        if not all(math.isfinite(v) for v in data):
            invalid.append(key)
        values[key] = [v if math.isfinite(v) else None for v in data]
    # Model/firmware-dependent units and unsupported zeros are not measurements.
    values["monitor_device_type"] = packet[736]
    values["end_effector_monitor"] = None
    if packet[736]:
        values["end_effector_monitor"] = {
            "state_raw": packet[737], "position_raw": struct.unpack_from(">h", packet, 738)[0],
            "speed_raw": struct.unpack_from(">h", packet, 740)[0],
            "current_raw": struct.unpack_from(">h", packet, 742)[0],
            "units_verified": False}
    return {"device_timestamp_raw": struct.unpack_from(">Q", packet, 4)[0],
            "device_timestamp_unit": "us", "values": values, "valid": not invalid,
            "invalid_fields": invalid, "packet_bytes": len(packet),
            "layout": REPORT_SEMANTICS["layout"]}

class ReportFramingError(ValueError):
    def __init__(self, message, completed_packets=()):
        super().__init__(message)
        self.completed_packets = list(completed_packets)

class PacketFramer:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, chunk):
        self.buffer.extend(chunk)
        if len(self.buffer) > 262144:
            raise ValueError("TCP 30000 receive buffer limit exceeded")
        packets = []
        while len(self.buffer) >= 4:
            size = struct.unpack_from(">I", self.buffer)[0]
            if size not in PACKET_SIZES:
                raise ReportFramingError(f"Unsupported TCP 30000 advertised length: {size}", packets)
            if len(self.buffer) < size:
                break
            packets.append(bytes(self.buffer[:size]))
            del self.buffer[:size]
        return packets

class TimestampedReportReceiver:
    def __init__(self, ip, on_sample, on_error, *, on_event=None, timeout=1.,
                 socket_factory=socket.create_connection):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("Report timeout must be positive")
        self.ip, self.on_sample, self.on_error = ip, on_sample, on_error
        self.on_event = on_event or (lambda event: None)
        self.timeout, self.socket_factory = timeout, socket_factory
        self.stop_event = threading.Event()
        self.thread = self.socket = None
        self.sequence = self.epoch = self.batch = 0
        self.last_timestamp = None
        self.error = None
        self.framer = PacketFramer()

    def start(self):
        if self.thread is not None:
            raise RuntimeError("Report receiver can only start once")
        self.socket = self.socket_factory((self.ip, 30000), timeout=self.timeout)
        self.socket.settimeout(min(.1, self.timeout))
        self.thread = threading.Thread(target=self._run, name="robot-native-report", daemon=True)
        self.thread.start()
        return self

    def accept(self, packet, received_at, batch):
        row = decode_report(packet)
        stamp = row["device_timestamp_raw"]
        if self.last_timestamp is not None and stamp < self.last_timestamp:
            self.epoch += 1
            self.on_event({"event": "source_clock_reset", "source": "robot",
                           "received_at": received_at, "clock_epoch": self.epoch,
                           "previous_device_timestamp_raw": self.last_timestamp,
                           "device_timestamp_raw": stamp, "boundary": True})
        elif stamp == self.last_timestamp:
            self.on_event({"event": "repeated_device_timestamp", "source": "robot",
                           "received_at": received_at, "device_timestamp_raw": stamp})
        self.last_timestamp = stamp
        self.sequence += 1
        row.update(kind="robot", sequence=self.sequence, received_at=received_at,
                   receive_batch=batch, source="tcp30000", clock_id="robot_tcp30000",
                   clock_epoch=self.epoch, device_sequence=None, packet=packet)
        self.on_sample(row)

    def _run(self):
        last = time.monotonic()
        try:
            while not self.stop_event.is_set():
                try:
                    chunk = self.socket.recv(65536)
                except socket.timeout:
                    if time.monotonic() - last >= self.timeout:
                        raise TimeoutError("TCP 30000 source stopped")
                    continue
                received = time.monotonic()
                if not chunk:
                    raise ConnectionError("TCP 30000 disconnected")
                self.batch += 1
                framing_error = None
                try:
                    packets = self.framer.feed(chunk)
                except ReportFramingError as exc:
                    packets, framing_error = exc.completed_packets, exc
                for packet in packets:
                    self.accept(packet, received, self.batch)
                    last = received
                if framing_error:
                    raise framing_error
                if received - last >= self.timeout:
                    raise TimeoutError("TCP 30000 received no complete reports")
        except Exception as exc:
            if not self.stop_event.is_set():
                self.error = str(exc)
                self.on_error({"source": "robot", "received_at": time.monotonic(),
                               "reason": str(exc), "partial_packet_hex": self.framer.buffer.hex()})
        finally:
            if self.socket:
                self.socket.close()

    def close(self):
        self.stop_event.set()
        if self.socket:
            try: self.socket.shutdown(socket.SHUT_RDWR)
            except OSError: pass
            self.socket.close()
        if self.thread:
            self.thread.join(self.timeout + .5)
            if self.thread.is_alive():
                raise RuntimeError("Native report receiver did not stop")

def mock_packet(stamp, value=0., size=784):
    packet = bytearray(size)
    struct.pack_into(">IQ", packet, 0, size, stamp)
    packet[12] = 5 << 4
    struct.pack_into("<7f", packet, 116, value, -.8, -.5, 0., 1.5, 0., 0.)
    struct.pack_into("<6f", packet, 472, 300. + value, 0., 200., math.pi, 0., 0.)
    return bytes(packet)

class MockReportReceiver(TimestampedReportReceiver):
    def start(self):
        if self.thread is not None:
            raise RuntimeError("Report receiver can only start once")
        self.thread = threading.Thread(target=self._synthetic, name="mock-native-report", daemon=True)
        self.thread.start()
        return self

    def _synthetic(self):
        tick = time.monotonic()
        try:
            while not self.stop_event.is_set():
                now = time.monotonic()
                self.batch += 1
                self.accept(mock_packet(int(now * 1e6), math.sin(now) * .02), now, self.batch)
                tick += .004
                if tick < now: tick = now + .004
                self.stop_event.wait(max(0., tick - time.monotonic()))
        except Exception as exc:
            self.error = str(exc)
            self.on_error({"source": "robot", "reason": str(exc), "received_at": time.monotonic()})
