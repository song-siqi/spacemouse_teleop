import types
import unittest
from robot_teleop.recording.clocks import ClockSampler
from robot_teleop.sensors.realsense.capture import read_frame_metadata, capture_identity

class ClockEvidenceTests(unittest.TestCase):
    def test_brackets_are_preserved_and_wall_step_is_not_corrected(self):
        mono = iter([100, 120, 1_000_000_100, 1_000_000_140])
        wall = iter([10_000_000_110, 16_000_000_120])
        rows, events = [], []
        clock = ClockSampler(rows.append, events.append, monotonic_ns=lambda:next(mono),
                             realtime_ns=lambda:next(wall))
        clock.sample("start"); clock.sample("periodic")
        self.assertEqual(rows[0]["read_interval_ns"], 20)
        self.assertEqual(rows[1]["realtime_ns"], 16_000_000_120)
        self.assertEqual(rows[1]["clock_epoch"], 1)
        self.assertEqual(len(events), 1)
        self.assertNotIn("corrected_timestamp", rows[1])

    def test_read_jitter_with_overlapping_brackets_is_not_clock_step(self):
        mono = iter([0, 20_000_000, 100_000_000, 120_000_000])
        wall = iter([1_005_000_000, 1_115_000_000])
        events = []
        clock = ClockSampler(lambda x:None, events.append, monotonic_ns=lambda:next(mono),
                             realtime_ns=lambda:next(wall))
        clock.sample("one"); clock.sample("two")
        self.assertFalse(events)

    def test_camera_raw_metadata_availability_and_unknown_units(self):
        names = ("frame_timestamp","sensor_timestamp","actual_exposure","time_of_arrival","backend_timestamp","actual_fps")
        rs = types.SimpleNamespace(frame_metadata_value=types.SimpleNamespace(**dict(zip(names,names))))
        frame = types.SimpleNamespace(supports_frame_metadata=lambda key:key != "actual_exposure",
                                      get_frame_metadata=lambda key:123)
        result = read_frame_metadata(rs, frame)
        self.assertEqual(result["sensor_timestamp"]["raw"],123)
        self.assertEqual(result["sensor_timestamp"]["unit"],"us")
        self.assertIsNone(result["backend_timestamp"]["unit"])
        self.assertFalse(result["actual_exposure"]["available"])
        self.assertIsNone(result["actual_exposure"]["raw"])

    def test_missing_metadata_does_not_become_zero(self):
        result = read_frame_metadata(types.SimpleNamespace(), object())
        self.assertTrue(all(v["raw"] is None and not v["available"] for v in result.values()))

    def test_clock_worker_failure_is_reported(self):
        errors = []
        clock = ClockSampler(lambda row: None, lambda row: None, interval_s=.001,
                             on_error=errors.append)
        def broken(reason): raise OSError("clock unavailable")
        clock.sample = broken
        clock._run()
        self.assertIn("clock unavailable", errors[0])
