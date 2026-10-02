import importlib.util
import socket
import struct
import threading
import unittest
from unittest.mock import patch

from robot_teleop.backends.real.xarm_sdk.transport import PacketSocket, SDKChannel
from robot_teleop.backends.real.xarm_sdk.safety import TeleopFault


class PacketSocketTest(unittest.TestCase):
    def packet_port(self, chunks):
        class Socket:
            def settimeout(self, value): pass
            def recv(self, count): return chunks.pop(0)
            def shutdown(self, how): pass
            def close(self): pass
        port = PacketSocket.__new__(PacketSocket)
        port.socket, port.timeout, port.connected, port.buffer = Socket(), .1, True, bytearray()
        return port

    def test_fragmented_and_coalesced_replies_are_framed(self):
        a = struct.pack('>HHHBB', 1, 2, 3, 13, 0) + b'\x02'
        b = struct.pack('>HHHBB', 2, 2, 4, 15, 0) + b'\x00\x00'
        port = self.packet_port([a[:3], a[3:] + b])
        self.assertEqual(port.read(), a)
        self.assertEqual(port.read(), b)

    def test_disconnect_and_bad_frame_raise(self):
        with self.assertRaises(TeleopFault): self.packet_port([b'']).read()
        with self.assertRaises(TeleopFault): self.packet_port([struct.pack('>HHH', 1, 2, 65535)]).read()


@unittest.skipUnless(importlib.util.find_spec('xarm'), 'SDK codec checks run in the xarm environment')
class SDKCodecTest(unittest.TestCase):
    def make_channel(self, writable=False, pause=None):
        from xarm.core.config.x_config import XCONF
        class Port:
            connected = True
            bad_gripper_ack = False
            baud_index = 11
            gripper_motion_status = 0
            gripper_target_position = 0
            bad_target_readback = False
            bad_gripper_status = False
            def __init__(self, *args): self.requests = []; self.response = None
            def flush(self): pass
            def close(self): self.connected = False
            def write(self, data):
                self.requests.append(data)
                func = data[6]
                payload = b''
                if func == XCONF.UxbusReg.GET_VERSION:
                    payload = b'6,6,XI1304,AC1302,v2.5.5'.ljust(40, b'\0')
                elif func == XCONF.UxbusReg.GET_STATE: payload = b'\x02'
                elif func == XCONF.UxbusReg.GET_ERROR: payload = b'\x00\x00'
                elif func == XCONF.UxbusReg.TGPIO_R16B:
                    payload = struct.pack('>i', self.baud_index)
                elif func == XCONF.UxbusReg.RS485_RTU:
                    body = data[7:]
                    host, device, function = body[:3]
                    address, count = struct.unpack('>HH', body[3:7])
                    if function == 3:
                        values = struct.pack('>i', self.gripper_target_position if address == XCONF.ServoConf.TAGET_POS else 207) if count == 2 else struct.pack('>H', self.gripper_motion_status if address == 0 else 0)
                        payload = bytes((host, device, function, len(values))) + values
                        if (address == 0 and self.bad_gripper_status) or (address == XCONF.ServoConf.TAGET_POS and self.bad_target_readback):
                            payload = payload[:-1]
                    elif function == 16:
                        if address == XCONF.ServoConf.TAGET_POS:
                            self.gripper_target_position = struct.unpack('>i', body[8:12])[0]
                        payload = body[:7]
                        if self.bad_gripper_ack:
                            payload = payload[:-1] + bytes((99,))
                self.response = data[:4] + struct.pack('>H', len(payload) + 2) + bytes((func, 0)) + payload
                return 0
            def read(self, timeout):
                if pause:
                    pause[0].set()
                    pause[1].wait(.5)
                return self.response
        return SDKChannel('unused', .1, writable=writable, port_factory=Port)

    def test_connect_is_silent_and_query_whitelist_cannot_write(self):
        channel = self.make_channel()
        self.assertEqual(channel.port.requests, [])
        self.assertEqual(channel.identity()['firmware'], (2, 5, 5))
        self.assertEqual(channel.query('get_state'), [2])
        before = len(channel.port.requests)
        with self.assertRaises(ValueError): channel.query('clean_war')
        with self.assertRaises(TeleopFault): channel.activate(.1)
        with self.assertRaises(TeleopFault): channel.velocity((0,) * 6, .2)
        with self.assertRaises(TeleopFault): channel.initialize_gripper({})
        self.assertEqual(len(channel.port.requests), before)
        channel.close()

    def test_sdk_velocity_codec_serializes_mm_and_radians_with_duration(self):
        channel = self.make_channel(writable=True)
        channel.velocity((10., -20., 30., .1, -.2, .3), .15)
        data = channel.port.requests[-1]
        self.assertEqual(struct.unpack('<6f', data[7:31])[:3], (10., -20., 30.))
        self.assertEqual(data[31], 0)  # base frame
        self.assertAlmostEqual(struct.unpack('<f', data[32:36])[0], .15)
        channel.close()

    def test_joint_homing_codec_has_finite_duration_and_seventh_axis_zero(self):
        channel = self.make_channel(writable=True)
        channel.activate_homing()
        channel.joint_velocity((.1, -.1, 0., .02, 0., 0.), .15)
        data = channel.port.requests[-1]
        self.assertEqual(len(data), 40)
        speeds = struct.unpack('<7f', data[7:35])
        self.assertAlmostEqual(speeds[0], .1)
        self.assertAlmostEqual(speeds[1], -.1)
        self.assertEqual(speeds[6], 0.)
        self.assertEqual(data[35], 1)
        self.assertAlmostEqual(struct.unpack('<f', data[36:40])[0], .15)
        before = len(channel.port.requests)
        for duration in (0, -1, float('nan')):
            with self.assertRaises(TeleopFault): channel.joint_velocity((0,) * 6, duration)
        self.assertEqual(len(channel.port.requests), before)
        channel.stop()
        from xarm.core.config.x_config import XCONF
        self.assertEqual(channel.port.requests[-2][6], XCONF.UxbusReg.VC_SET_JOINTV)
        self.assertEqual(channel.port.requests[-1][6], XCONF.UxbusReg.SET_STATE)
        channel.close()

    def test_gripper_init_does_not_send_goal_and_targets_use_official_endpoint_encoding(self):
        from xarm.core.config.x_config import XCONF
        channel = self.make_channel(writable=True)
        self.assertEqual(channel.gripper_baudrate(), 2000000)
        self.assertEqual(channel.gripper_status()['position_pulse'], 207)
        channel.initialize_gripper({'baudrate': 2000000, 'speed_pulse_s': 500})
        writes = [p[7:] for p in channel.port.requests if p[6] == XCONF.UxbusReg.RS485_RTU and p[9] == 16]
        self.assertEqual([struct.unpack('>H', p[3:5])[0] for p in writes],
                         [XCONF.ServoConf.CON_MODE, XCONF.ServoConf.CON_EN, XCONF.ServoConf.POS_SPD])
        for target in (850, 0):
            channel.gripper_target(target)
            body = channel.port.requests[-1][7:]
            self.assertEqual(body[:3], bytes((9, 8, 16)))
            self.assertEqual(struct.unpack('>HHB', body[3:8]), (XCONF.ServoConf.TAGET_POS, 2, 4))
            self.assertEqual(struct.unpack('>i', body[8:12])[0], target)
        channel.close()

    def test_gripper_status_is_read_only_and_reports_moving_and_grasped(self):
        from xarm.core.config.x_config import XCONF
        channel = self.make_channel()
        for code, name in ((0, 'stopped'), (1, 'moving'), (2, 'grasped'), (3, 'unknown')):
            channel.port.gripper_motion_status = code
            status = channel.gripper_status()
            self.assertEqual(status['status_code'], code)
            self.assertEqual(status['motion_state'], name)
        self.assertTrue(all(p[6] == XCONF.UxbusReg.RS485_RTU and p[9] == 3 for p in channel.port.requests))
        channel.port.bad_gripper_status = True
        with self.assertRaisesRegex(TeleopFault, 'motion status'):
            channel.gripper_status()
        channel.close()

    def test_gripper_target_readback_tracks_latest_register_write(self):
        channel = self.make_channel(writable=True)
        for target in (850, 407, 230):
            channel.gripper_target(target)
            status = channel.gripper_status()
            self.assertEqual(status['controller_target_pulse'], target)
            self.assertEqual(status['position_pulse'], 207)
        channel.port.bad_target_readback = True
        with self.assertRaisesRegex(TeleopFault, 'target readback'):
            channel.gripper_status()
        channel.close()

    def test_gripper_baud_mismatch_and_bad_ack_fail_without_auto_baud_setting(self):
        from xarm.core.config.x_config import XCONF
        channel = self.make_channel(writable=True)
        channel.port.baud_index = 5
        with self.assertRaisesRegex(TeleopFault, 'baudrate differs'):
            channel.initialize_gripper({'baudrate': 2000000, 'speed_pulse_s': 500})
        self.assertEqual([p[6] for p in channel.port.requests], [XCONF.UxbusReg.TGPIO_R16B])
        channel.port.bad_gripper_ack = True
        with self.assertRaisesRegex(TeleopFault, 'acknowledgement'):
            channel.gripper_target(850)
        channel.close()

    def test_unbounded_velocity_is_rejected_before_serialization(self):
        channel = self.make_channel(writable=True)
        for duration in (0., -1., float('nan'), float('inf')):
            with self.assertRaises(TeleopFault):
                channel.velocity((0,) * 6, duration)
        self.assertEqual(channel.port.requests, [])
        channel.close()

    def test_sdk_locks_are_not_shared_across_channels(self):
        entered, release, done = threading.Event(), threading.Event(), threading.Event()
        slow = self.make_channel(pause=(entered, release))
        arm = self.make_channel(writable=True)
        thread = threading.Thread(target=lambda: (slow.query('get_state'), done.set()))
        try:
            thread.start()
            self.assertTrue(entered.wait(.2))
            arm.velocity((0,) * 6, .1)
            self.assertFalse(done.is_set())
            self.assertEqual(len(arm.port.requests), 1)
        finally:
            release.set()
            thread.join(1.)
            slow.close()
            arm.close()
