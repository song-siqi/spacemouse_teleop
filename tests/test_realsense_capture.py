import importlib.util
import threading
import time
import types
import unittest

from robot_teleop.sensors.realsense import CameraGroup, MockCameraGroup, REQUIRED_ROLES, enumerate_devices


class Array:
    shape = (2, 3, 3)
    dtype = "uint8"
    writable = True

    def copy(self): return Array()
    def setflags(self, *, write): self.writable = write


class FakeSDK:
    camera_info = types.SimpleNamespace(serial_number="serial", name="name", usb_type_descriptor="usb", firmware_version="firmware")
    stream = types.SimpleNamespace(color="color")
    format = types.SimpleNamespace(rgb8="rgb8")

    def __init__(self, frame_ids=(1, 2, 3), pause=.004):
        self.frame_ids = list(frame_ids)
        self.pause = pause
        self.configs = []
        self.pipelines = []
        self.fail_stream = False
        self.stop_gate = None

    def context(self):
        class Device:
            def __init__(self, serial): self.serial = serial
            def supports(self, token): return True
            def get_info(self, token): return {"serial": self.serial, "name": "D455", "usb": "3.2", "firmware": "test"}[token]
        return types.SimpleNamespace(query_devices=lambda: [Device("c"), Device("a"), Device("b")])

    def config(self):
        class Config:
            def enable_device(self, serial): self.serial = serial
            def enable_stream(self, *stream): self.stream = stream
        config = Config()
        self.configs.append(config)
        return config

    def pipeline(self):
        sdk = self
        class Pipeline:
            stopped = False
            def __init__(self): self.ids = list(sdk.frame_ids)
            def start(self, config): self.config = config
            def try_wait_for_frames(self, timeout_ms):
                time.sleep(sdk.pause)
                if sdk.fail_stream: raise RuntimeError("USB disconnected")
                if not self.ids: return False, None
                frame_id = self.ids.pop(0)
                if isinstance(frame_id, tuple):
                    frame_id, stamp, domain = frame_id
                else:
                    stamp, domain = frame_id * 33.3, "hardware_clock"
                color = types.SimpleNamespace(get_frame_number=lambda: frame_id,
                    get_timestamp=lambda: stamp,
                    get_frame_timestamp_domain=lambda: domain,
                    get_data=lambda: Array())
                return True, types.SimpleNamespace(get_color_frame=lambda: color)
            def stop(self):
                if sdk.stop_gate: sdk.stop_gate.wait(.5)
                self.stopped = True
        pipe = Pipeline()
        self.pipelines.append(pipe)
        return pipe


def cameras():
    return [{"role": role, "serial": serial, "width": 3, "height": 2, "frame_timeout_s": .06}
            for role, serial in zip(REQUIRED_ROLES, ("b", "c", "a"))]


def wait_until(check, timeout=.6):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if check(): return
        time.sleep(.002)
    raise AssertionError("Condition did not become true")


class CameraCaptureTests(unittest.TestCase):
    def make(self, sdk=None, on_frame=None, on_error=None, history_size=120):
        frames, errors = [], []
        sdk = sdk or FakeSDK()
        group = CameraGroup(cameras(), on_frame or frames.append, on_error or errors.append,
                            rs_module=sdk, numpy_module=types.SimpleNamespace(asanyarray=lambda value: value),
                            history_size=history_size)
        self.addCleanup(group.close)
        return group, sdk, frames, errors

    def test_enumeration_never_opens_pipeline(self):
        sdk = FakeSDK()
        devices = enumerate_devices(sdk)
        self.assertEqual([d["serial"] for d in devices], ["a", "b", "c"])
        self.assertEqual(devices[0]["usb_type"], "3.2")
        self.assertEqual(sdk.pipelines, [])

    def test_three_explicit_unique_bindings_are_required(self):
        for specs in (cameras()[:2], [dict(c, serial="a") for c in cameras()],
                      [dict(c, serial=None) for c in cameras()],
                      [dict(c, role="wrist") for c in cameras()]):
            with self.assertRaises(ValueError): CameraGroup(specs, lambda f: None, lambda e: None)
        for value in (0, -1, float("nan"), True):
            specs = cameras()
            specs[0]["frame_timeout_s"] = value
            with self.assertRaises(ValueError): CameraGroup(specs, lambda f: None, lambda e: None)

    def test_schema_v2_fps_reaches_the_camera_stream(self):
        specs = [dict(c, fps=15) for c in cameras()]
        group = CameraGroup(specs, lambda row: None, lambda error: None)
        self.assertTrue(all(c["capture_fps"] == 15 for c in group.cameras))

    def test_missing_serial_fails_before_any_stream_opens(self):
        group, sdk, _, _ = self.make()
        group.cameras[0]["serial"] = "missing"
        with self.assertRaisesRegex(RuntimeError, "not connected"): group.start()
        self.assertEqual(sdk.pipelines, [])

    def test_binding_rgb_copy_device_time_and_bounded_history(self):
        group, sdk, frames, errors = self.make(history_size=2)
        group.start()
        wait_until(lambda: len(frames) == 9)
        group.close()
        self.assertFalse(errors)
        self.assertEqual({c.serial for c in sdk.configs}, {"a", "b", "c"})
        self.assertTrue(all(c.stream == ("color", 3, 2, "rgb8", 30) for c in sdk.configs))
        self.assertTrue(all(not f["rgb"].writable for f in frames))
        self.assertTrue(all(f["timestamp_domain"] == "hardware_clock" for f in frames))
        self.assertEqual(len(group.history("wrist")), 2)
        latest = group.latest_frames()
        self.assertEqual(latest["wrist"]["serial"], "b")
        self.assertEqual(latest["wrist"]["sequence"], 3)
        latest["wrist"]["sequence"] = 999
        self.assertEqual(group.latest_frames()["wrist"]["sequence"], 3)
        self.assertTrue(all(p.stopped for p in sdk.pipelines))
        with self.assertRaises(RuntimeError): group.start()

    def test_timestamp_domain_change_keeps_raw_times_and_opens_boundary(self):
        group, sdk, frames, errors = self.make(FakeSDK([(1,33.3,"hardware_clock"),
                                                      (1,100033.3,"global_time")]))
        events=[]
        group.on_event=events.append
        group.start()
        wait_until(lambda:len(frames)==6)
        group.close()
        rows=[f for f in frames if f["role"]=="wrist"]
        self.assertEqual([f["clock_epoch"] for f in rows],[0,1])
        self.assertEqual([f["device_timestamp_ms"] for f in rows],[33.3,100033.3])
        self.assertEqual(len(events),3)
        self.assertTrue(all(e["boundary"] for e in events))
        self.assertEqual(events[0]["previous_timestamp_domain"],"hardware_clock")
        self.assertEqual(events[0]["timestamp_domain"],"global_time")

    def test_duplicate_frames_not_fresh_and_timeout_is_reported(self):
        group, _, frames, errors = self.make(FakeSDK([1] * 100))
        group.start()
        wait_until(lambda: len(errors) == 3)
        group.close()
        self.assertEqual(len(frames), 3)
        self.assertTrue(all("No fresh RGB" in e["reason"] for e in errors))
        stats = group.diagnostics()
        self.assertTrue(all(c["duplicate_frames"] > 0 for c in stats["cameras"].values()))

    def test_backwards_device_sequence_opens_clock_segment_without_rewriting_time(self):
        group, _, frames, errors = self.make(FakeSDK([10, 9]))
        events = []
        group.on_event = events.append
        group.start()
        wait_until(lambda: len(frames) == 6)
        group.close()
        self.assertFalse(errors)
        self.assertEqual(len(events), 3)
        self.assertTrue(all(e["boundary"] for e in events))
        for role in REQUIRED_ROLES:
            rows = [f for f in frames if f["role"] == role]
            self.assertEqual([f["clock_epoch"] for f in rows], [0, 1])
            self.assertEqual([f["frame_id"] for f in rows], [10, 9])

    def test_disconnect_and_callback_exception_are_surfaced(self):
        sdk = FakeSDK()
        sdk.fail_stream = True
        group, _, frames, errors = self.make(sdk)
        group.start()
        wait_until(lambda: len(errors) == 3)
        self.assertFalse(frames)
        self.assertTrue(all("USB disconnected" in e["reason"] for e in errors))
        def fail(frame): raise RuntimeError("writer unavailable")
        group2, _, _, errors2 = self.make(on_frame=fail)
        group2.start()
        wait_until(lambda: len(errors2) == 3)
        self.assertTrue(all("writer unavailable" in e["reason"] for e in errors2))

    def test_shutdown_is_bounded_and_reports_blocked_native_stop(self):
        sdk = FakeSDK()
        sdk.stop_gate = threading.Event()
        group, _, frames, _ = self.make(sdk)
        group.start()
        wait_until(lambda: len(frames) >= 3)
        started = time.monotonic()
        try:
            with self.assertRaisesRegex(RuntimeError, "did not stop"): group.close(.02)
            self.assertLess(time.monotonic() - started, .1)
        finally:
            sdk.stop_gate.set()
            group.close()

    @unittest.skipUnless(importlib.util.find_spec("numpy"), "Mock RGB generation requires collection NumPy extra")
    def test_mock_provides_same_frames_without_camera_sdk(self):
        frames = []
        group = MockCameraGroup(cameras(), frames.append, self.fail)
        self.addCleanup(group.close)
        group.start()
        wait_until(lambda: len(frames) >= 3)
        group.close()
        self.assertEqual(set(group.latest_frames()), set(REQUIRED_ROLES))
        self.assertTrue(all(f["timestamp_domain"] == "mock_host_monotonic" for f in frames))
        self.assertTrue(all(not f["rgb"].flags.writeable for f in frames))


if __name__ == "__main__":
    unittest.main()
