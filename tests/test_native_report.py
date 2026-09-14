import math
import socket
import struct
import time
import unittest
from spacemouse_teleop.backends.xarm6_camera.report import (
    decode_report, PacketFramer, TimestampedReportReceiver, mock_packet, FIELD_LAYOUT)

class NativeReportTests(unittest.TestCase):
    def test_known_layouts_preserve_actual_values_and_raw_representation(self):
        for size in (784, 804):
            packet = bytearray(mock_packet(123456, .25, size))
            for n, (key, (offset, count, unit)) in enumerate(FIELD_LAYOUT.items()):
                struct.pack_into(f"<{count}f", packet, offset, *[n + i / 4 for i in range(count)])
            row = decode_report(packet)
            self.assertEqual(row["packet_bytes"], size)
            self.assertEqual(row["device_timestamp_raw"], 123456)
            self.assertEqual(row["values"]["joints_rad"], [3+i/4 for i in range(6)])
            self.assertEqual(row["values"]["tcp_mm_rotvec_rad"], [10+i/4 for i in range(6)])
            self.assertNotIn("tcp_mm_rad", row["values"])
            self.assertNotIn("cgpio_analog_input", row["values"])

    def test_unused_monitor_bytes_cannot_become_gripper_state(self):
        packet = bytearray(mock_packet(123))
        struct.pack_into(">h", packet, 738, 777)
        self.assertIsNone(decode_report(packet)["values"]["end_effector_monitor"])
        packet[736] = 1
        monitor = decode_report(packet)["values"]["end_effector_monitor"]
        self.assertEqual(monitor["position_raw"], 777)
        self.assertFalse(monitor["units_verified"])

    def test_nonfinite_values_are_explicit_invalid_and_json_compatible(self):
        packet = bytearray(mock_packet(10))
        struct.pack_into("<f", packet, 116, math.nan)
        row = decode_report(packet)
        self.assertFalse(row["valid"])
        self.assertIsNone(row["values"]["joints_rad"][0])
        self.assertIn("joints_rad", row["invalid_fields"])

    def test_split_and_coalesced_packets_are_not_lost(self):
        a, b = mock_packet(100), mock_packet(200, size=804)
        for step in (1, 3, 100, 784, 1588):
            parser, result = PacketFramer(), []
            data = a+b+a
            for i in range(0, len(data), step):
                result.extend(parser.feed(data[i:i+step]))
            self.assertEqual(result, [a,b,a])
            self.assertFalse(parser.buffer)

    def test_unknown_lengths_fail_without_padding(self):
        for size in (0, 32, 783, 805, 999999):
            with self.assertRaises(ValueError):
                PacketFramer().feed(struct.pack(">I", size))
        with self.assertRaises(ValueError): decode_report(mock_packet(100)[:-1])

    def test_same_receive_batch_and_clock_reset_are_preserved(self):
        rows, events = [], []
        source = TimestampedReportReceiver("unused", rows.append, self.fail, on_event=events.append)
        for stamp in (100, 100, 90):
            source.accept(mock_packet(stamp), 10., 1)
        self.assertEqual([r["sequence"] for r in rows], [1,2,3])
        self.assertEqual([r["received_at"] for r in rows], [10.]*3)
        self.assertEqual([r["clock_epoch"] for r in rows], [0,0,1])
        self.assertEqual([e["event"] for e in events], ["repeated_device_timestamp", "source_clock_reset"])
        self.assertTrue(events[-1]["boundary"])

    def test_passive_receiver_never_sends_and_reports_disconnect(self):
        calls, rows, errors = [], [], []
        class Socket:
            def __init__(self): self.chunks = [mock_packet(1)[:19], mock_packet(1)[19:]+mock_packet(2), b""]
            def settimeout(self, timeout): pass
            def recv(self, length): return self.chunks.pop(0)
            def close(self): calls.append("close")
            def shutdown(self, how): pass
            def send(self, data): raise AssertionError("No sends permitted")
        def connect(address, timeout):
            calls.append(address)
            return Socket()
        source = TimestampedReportReceiver("fake", rows.append, errors.append, socket_factory=connect)
        source.start()
        source.thread.join(1)
        source.close()
        self.assertEqual(calls[0], ("fake",30000))
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0]["received_at"], rows[1]["received_at"])
        self.assertEqual(len(errors), 1)

    def test_truncated_stream_retains_partial_packet_evidence(self):
        errors = []
        class Socket:
            chunks = [mock_packet(123)[:99], b""]
            def settimeout(self, timeout): pass
            def recv(self, length): return self.chunks.pop(0)
            def close(self): pass
            def shutdown(self, how): pass
        source = TimestampedReportReceiver("fake", self.fail, errors.append,
                                           socket_factory=lambda *a, **kw: Socket())
        source.start(); source.thread.join(1); source.close()
        self.assertEqual(bytes.fromhex(errors[0]["partial_packet_hex"]), mock_packet(123)[:99])

    def test_valid_packets_before_unknown_layout_are_still_published(self):
        rows, errors = [], []
        good = mock_packet(456)
        unknown = struct.pack(">I", 900) + b"unknown"
        class Socket:
            def settimeout(self, timeout): pass
            def recv(self, length): return good + unknown
            def close(self): pass
            def shutdown(self, how): pass
        source = TimestampedReportReceiver("fake", rows.append, errors.append,
                                           socket_factory=lambda *a, **kw: Socket())
        source.start(); source.thread.join(1); source.close()
        self.assertEqual([r["packet"] for r in rows], [good])
        self.assertEqual(bytes.fromhex(errors[0]["partial_packet_hex"]), unknown)

    def test_captured_784_byte_packet_matches_independent_diagnostic_decode(self):
        import json, hashlib
        from pathlib import Path
        fixture = Path(__file__).parent / "fixtures" / "xarm30000_784"
        packet = fixture.with_suffix(".bin").read_bytes()
        evidence = json.loads(fixture.with_suffix(".json").read_text())
        self.assertEqual(hashlib.sha256(packet).hexdigest(), evidence["sha256"])
        row, expected = decode_report(packet), evidence["expected"]
        self.assertEqual(row["device_timestamp_raw"], expected["device_timestamp_raw"])
        for field in ("joints_rad", "tcp_mm_rotvec_rad", "state", "mode", "monitor_device_type"):
            self.assertEqual(row["values"][field], expected[field])
        self.assertEqual(row["values"]["joint_velocity_rad_s"], expected["joint_velocity"])
        self.assertEqual(row["values"]["tcp_speed_mm_s_rad_s"], expected["tcp_speed"])

    def test_804_layout_offsets_agree_with_installed_sdk_structure(self):
        try:
            from spacemouse_teleop.backends.xarm_sdk.transport import sdk_types
            _, report_type = sdk_types()
        except RuntimeError:
            self.skipTest("SDK codec not installed")
        structure = report_type.create(30000, is_radian=True)
        self.assertEqual(len(structure), 804)
        packet = mock_packet(5678, .75, size=804)
        structure.update(packet)
        current = decode_report(packet)
        self.assertEqual(current["values"]["joints_rad"], structure.actual_joint_angle[:6])
        self.assertEqual(current["values"]["tcp_mm_rotvec_rad"], structure.actual_tcp_pose)
        self.assertEqual(type(structure)._timestamp.offset, 4)
        self.assertEqual(type(structure)._actual_joint_angle.offset, 116)
        self.assertEqual(type(structure)._actual_tcp_pose.offset, 472)
