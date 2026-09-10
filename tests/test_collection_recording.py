import json
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from spacemouse_teleop.recording.collection import (
    CollectionFault, CollectionRecorder, DEFAULT_QUALITY, ROLES, align_observation,
)


def state(kind, sequence=1, received_at=10., value=0.):
    values = {"joints": {"joints_rad": [value] * 6},
              "tcp": {"tcp_mm_rad": [value] * 6},
              "gripper": {"position_pulse": value, "controller_target_pulse": 840,
                          "motion_status": 0, "error_code": 0},
              "report": {"state": 0, "mode": 5}}[kind]
    return {"kind": kind, "sequence": sequence, "received_at": received_at,
            "query_started_at": received_at - .002, "query_finished_at": received_at,
            "values": values, "valid": True}


def frame(role, sequence=1, received_at=10.):
    rgb = np.zeros((3, 4, 3), dtype=np.uint8)
    rgb[:, :, 0] = 201
    rgb[:, :, 2] = 19
    return {"role": role, "serial": f"serial_{role}", "frame_id": sequence,
            "sequence": sequence, "received_at": received_at,
            "device_timestamp_ms": 2000 + sequence * 33.33,
            "timestamp_domain": "hardware_clock", "width": 4, "height": 3,
            "capture_fps": 30, "rgb": rgb}


def histories(received_at=10.):
    return ({kind: [state(kind, received_at=received_at)] for kind in ("joints", "tcp", "gripper")},
            {role: [frame(role, received_at=received_at)] for role in ROLES})


def wait_until(predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.005)
    raise AssertionError("timed out waiting for asynchronous recorder")


class AlignmentTests(unittest.TestCase):
    def test_fixed_50_ms_grid_uses_history_and_not_future_latest(self):
        states, frames = histories(9.99)
        for kind in states:
            states[kind].append(state(kind, 2, 10.06, 2.))
        for role in frames:
            frames[role].append(frame(role, 2, 10.06))
        for index in range(2):
            row, _ = align_observation(10. + index / 20, 10., index, states, frames, DEFAULT_QUALITY)
            self.assertEqual(row['sources']['joints']['sequence'], 1)
            self.assertEqual(row['images']['wrist']['sequence'], 1)
            self.assertAlmostEqual(row['relative_time_s'], index * .05)
        row, _ = align_observation(10.1, 10., 2, states, frames, DEFAULT_QUALITY)
        self.assertEqual(row['joints_rad'], [2.] * 6)
        self.assertEqual(row['sources']['joints']['sequence'], 2)

    def test_stationary_values_are_fresh_when_source_sequence_is_new(self):
        states, frames = histories(10.)
        states['joints'].append(state('joints', 2, 10.049))
        row, _ = align_observation(10.05, 10., 1, states, frames, DEFAULT_QUALITY)
        self.assertEqual(row['sources']['joints']['sequence'], 2)
        self.assertAlmostEqual(row['sources']['joints']['age_s'], .001)

    def test_missing_required_view_and_stale_sources_are_faults(self):
        states, frames = histories(10.)
        frames['fixed_2'] = []
        with self.assertRaisesRegex(CollectionFault, 'missing causal camera fixed_2'):
            align_observation(10., 10., 0, states, frames, DEFAULT_QUALITY)
        states, frames = histories(10.)
        with self.assertRaisesRegex(CollectionFault, 'stale joints'):
            align_observation(10.2, 10., 4, states, frames, DEFAULT_QUALITY)

    def test_gripper_target_cannot_substitute_for_actual_position(self):
        states, frames = histories()
        states['gripper'][0]['values'] = {'target_pulse': 850}
        with self.assertRaisesRegex(CollectionFault, 'actual position_pulse'):
            align_observation(10., 10., 0, states, frames, DEFAULT_QUALITY)

    def test_nonfinite_and_wrong_joint_mapping_are_rejected(self):
        for values in ([float('nan')] * 6, [0.] * 7, [None] * 6):
            states, frames = histories()
            states['joints'][0]['values']['joints_rad'] = values
            with self.assertRaises(CollectionFault):
                align_observation(10., 10., 0, states, frames, DEFAULT_QUALITY)

    def test_camera_skew_uses_host_clock_not_device_clock(self):
        states, frames = histories(10.)
        frames['fixed_1'][0]['received_at'] = 9.94
        frames['wrist'][0]['device_timestamp_ms'] = 9000000000.
        with self.assertRaisesRegex(CollectionFault, 'receive-time skew'):
            align_observation(10., 10., 0, states, frames, DEFAULT_QUALITY)

    def test_query_interval_is_preserved_and_must_end_before_receive(self):
        states, frames = histories()
        row, _ = align_observation(10., 10., 0, states, frames, DEFAULT_QUALITY)
        self.assertEqual(row['sources']['tcp']['query_started_at'], 9.998)
        states['tcp'][0]['query_finished_at'] = 11.
        with self.assertRaisesRegex(CollectionFault, 'query time interval'):
            align_observation(10., 10., 0, states, frames, DEFAULT_QUALITY)


class RecorderTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.recorder = CollectionRecorder(self.tmp.name, 'pick object', {}, min_free_bytes=0)
        self.recorder.open()
        self.addCleanup(self.recorder.close)
        self.sequence = 0
        self.producing = threading.Event()
        self.producer = None

    def publish(self):
        self.sequence += 1
        now = time.monotonic()
        for kind in ('joints', 'tcp', 'gripper', 'report'):
            self.recorder.publish_state(state(kind, self.sequence, now, float(self.sequence)))
        for role in ROLES:
            self.recorder.publish_frame(frame(role, self.sequence, now))

    def feed(self):
        self.publish()
        self.producing.set()
        def run():
            while self.producing.wait(0) and not self.recorder._stop.is_set():
                time.sleep(1 / 30)
                self.publish()
        self.producer = threading.Thread(target=run, daemon=True)
        self.producer.start()
        self.addCleanup(self.stop_feed)

    def stop_feed(self):
        self.producing.clear()
        if self.producer:
            self.producer.join(1)

    def finish(self):
        self.recorder.stop_episode()
        wait_until(lambda: self.recorder.status()['state'] in ('review', 'fault'))
        path = Path(self.recorder.status()['path'])
        return path, json.loads((path / 'meta.json').read_text())

    def test_complete_episode_has_native_provenance_lossless_images_and_20hz_grid(self):
        self.feed()
        self.recorder.write({'phase': 'homing'})
        self.assertIsNotNone(self.recorder.start_episode())
        wait_until(lambda: self.recorder.status()['frame_count'] >= 4)
        path, meta = self.finish()
        self.assertEqual(meta['status'], 'complete')
        self.assertEqual(meta['disposition'], 'pending')
        rows = [json.loads(line) for line in (path / 'observations.jsonl').read_text().splitlines()]
        raw = [json.loads(line) for line in (path / 'raw_states.jsonl').read_text().splitlines()]
        raw_keys = {(row['kind'], row['sequence']) for row in raw}
        self.assertGreater(len(raw), len(rows) * 3)
        import cv2
        for index, row in enumerate(rows):
            self.assertEqual(row['index'], index)
            self.assertAlmostEqual(row['timestamp'] - rows[0]['timestamp'], index / 20)
            for kind, source in row['sources'].items():
                self.assertIn((kind, source['sequence']), raw_keys)
                self.assertLessEqual(source['received_at'], row['timestamp'])
            for image in row['images'].values():
                bgr = cv2.imread(str(path / image['file']))
                self.assertEqual(tuple(bgr[0, 0]), (19, 0, 201))
        self.recorder.close()
        control = [json.loads(line) for line in (self.recorder.session_path / 'control.jsonl').read_text().splitlines()]
        self.assertEqual(control, [{'phase': 'homing'}])

    def test_review_requires_disposition_and_next_episode_restarts_index(self):
        self.feed()
        first = self.recorder.start_episode()
        wait_until(lambda: self.recorder.status()['frame_count'] >= 1)
        first_path, _ = self.finish()
        self.assertIsNone(self.recorder.start_episode())
        self.assertTrue(self.recorder.disposition('discarded'))
        wait_until(lambda: self.recorder.status()['state'] == 'idle')
        second = self.recorder.start_episode()
        self.assertNotEqual(first, second)
        wait_until(lambda: self.recorder.status()['frame_count'] >= 1)
        second_path, _ = self.finish()
        self.assertTrue(self.recorder.disposition('kept'))
        wait_until(lambda: self.recorder.status()['state'] == 'idle')
        self.assertTrue((first_path / 'observations.jsonl').exists())
        self.assertEqual(json.loads((first_path / 'meta.json').read_text())['disposition'], 'discarded')
        self.assertEqual(json.loads((second_path / 'meta.json').read_text())['disposition'], 'kept')
        rows = [json.loads(line) for line in (second_path / 'observations.jsonl').read_text().splitlines()]
        self.assertEqual(rows[0]['index'], 0)

    def test_stale_camera_fault_ends_only_episode_and_keep_cannot_make_it_complete(self):
        self.publish()
        self.assertIsNotNone(self.recorder.start_episode())
        wait_until(lambda: self.recorder.status()['state'] == 'fault')
        self.recorder.write({'arm_still_teleoperating': True})
        self.assertTrue(self.recorder.disposition('kept'))
        wait_until(lambda: self.recorder.status()['state'] == 'idle')
        path = Path(self.recorder.status()['path'])
        meta = json.loads((path / 'meta.json').read_text())
        self.assertEqual(meta['status'], 'incomplete')
        self.assertEqual(meta['disposition'], 'kept')
        self.assertIsNone(self.recorder.start_episode())
        self.feed()
        self.assertIsNotNone(self.recorder.start_episode())

    def test_republishing_cached_source_does_not_refresh_age(self):
        self.publish()
        original = self.recorder._states['joints'][-1]['received_at']
        self.recorder.publish_state(state('joints', self.sequence, original + 1))
        self.assertEqual(self.recorder._states['joints'][-1]['received_at'], original)
        original_frame = self.recorder._frames['wrist'][-1]['received_at']
        self.recorder.publish_frame(frame('wrist', self.sequence + 1, original_frame + 1))
        self.assertNotEqual(self.recorder._frames['wrist'][-1]['sequence'], self.sequence)
        duplicate = frame('wrist', self.sequence + 2, original_frame + 2)
        duplicate['frame_id'] = self.sequence + 1
        self.recorder.publish_frame(duplicate)
        self.assertEqual(self.recorder._frames['wrist'][-1]['received_at'], original_frame + 1)

    def test_malformed_input_never_throws_into_sdk_caller(self):
        self.publish()
        self.recorder.start_episode()
        malformed = state('joints', 2, time.monotonic(), float('nan'))
        self.recorder.publish_state(malformed)
        self.recorder.publish_frame({'role': 'wrist'})
        self.recorder.write({'invalid': float('inf')})
        wait_until(lambda: self.recorder.status()['state'] == 'fault')
        self.assertFalse(self.recorder.status()['complete'])

    def test_writer_failure_preserves_incomplete_meta_and_does_not_raise_in_publishers(self):
        self.feed()
        with patch.object(self.recorder, '_save_rgb', side_effect=OSError('disk full')):
            self.recorder.start_episode()
            wait_until(lambda: self.recorder.status()['fatal_error'] is not None)
            self.publish()
            self.recorder.write({'control': 'continues'})
        path = Path(self.recorder.status()['path'])
        self.assertNotEqual(json.loads((path / 'meta.json').read_text())['status'], 'complete')
        self.assertIsNone(self.recorder.start_episode())

    def test_missing_image_is_detected_before_complete_status(self):
        self.feed()
        with patch.object(self.recorder, '_save_rgb', return_value=None):
            self.recorder.start_episode()
            wait_until(lambda: self.recorder.status()['frame_count'] >= 1)
            _, meta = self.finish()
        self.assertEqual(meta['status'], 'incomplete')
        self.assertIn('observation references missing images', meta['integrity_errors'])

    def test_low_disk_reserve_prevents_start_without_harming_log(self):
        self.publish()
        with self.recorder._lock:
            self.recorder._free_bytes = -1
        self.assertIsNone(self.recorder.start_episode())
        self.recorder.write({'still': 'alive'})
        self.assertEqual(self.recorder.status()['state'], 'idle')

    def test_start_uses_cached_space_and_never_touches_filesystem_on_calling_thread(self):
        self.feed()
        import shutil
        real_usage = shutil.disk_usage
        caller = threading.get_ident()
        checked_threads = []
        def background_only(path):
            checked_threads.append(threading.get_ident())
            if threading.get_ident() == caller:
                raise AssertionError("filesystem called from motion thread")
            return real_usage(path)
        with patch('spacemouse_teleop.recording.collection.shutil.disk_usage', side_effect=background_only):
            started = time.monotonic()
            self.assertIsNotNone(self.recorder.start_episode())
            self.assertLess(time.monotonic() - started, .02)
            wait_until(lambda: self.recorder.status()['frame_count'] >= 1)
            self.finish()
        self.assertTrue(checked_threads)
        self.assertNotIn(caller, checked_threads)

    def test_begin_rechecks_space_in_background_and_marks_episode_incomplete(self):
        self.publish()
        with patch('spacemouse_teleop.recording.collection.shutil.disk_usage') as usage:
            usage.return_value.free = -1
            self.assertIsNotNone(self.recorder.start_episode())
            wait_until(lambda: self.recorder.status()['state'] == 'fault')
        meta = json.loads((Path(self.recorder.status()['path']) / 'meta.json').read_text())
        self.assertEqual(meta['status'], 'incomplete')
        self.assertIn('disk free space', meta['end_reason'])

    def test_even_a_completely_full_queue_cannot_raise_from_stop_or_control_write(self):
        self.recorder.close()
        self.recorder = CollectionRecorder(self.tmp.name, 'full', {}, min_free_bytes=0, queue_size=4)
        self.recorder.open()
        self.addCleanup(self.recorder.close)
        self.publish()
        entered, release = threading.Event(), threading.Event()
        original = self.recorder._save_rgb
        def slow(path, rgb):
            entered.set()
            release.wait(2)
            original(path, rgb)
        with patch.object(self.recorder, '_save_rgb', side_effect=slow):
            self.recorder.start_episode()
            self.assertTrue(entered.wait(1))
            # Simulate a future caller accidentally consuming the reserved slot.
            with self.recorder._lock:
                while not self.recorder.queue.full():
                    self.recorder.queue.put_nowait(('control', {'injected': True}))
            began = time.monotonic()
            self.assertTrue(self.recorder.stop_episode())
            for _ in range(20):
                self.recorder.write({'control': 'continues'})
            self.assertLess(time.monotonic() - began, .1)
            self.assertIsNotNone(self.recorder._pending_finish)
            release.set()
            wait_until(lambda: self.recorder.status()['state'] == 'fault')
        self.assertFalse(self.recorder.status()['complete'])
        self.assertIsNone(self.recorder._pending_finish)

    def test_queue_overflow_is_bounded_and_retains_finish_fence(self):
        self.recorder.close()
        self.recorder = CollectionRecorder(self.tmp.name, 'bounded', {}, min_free_bytes=0, queue_size=4)
        self.recorder.open()
        self.addCleanup(self.recorder.close)
        self.publish()
        entered, release = threading.Event(), threading.Event()
        original = self.recorder._save_rgb
        def slow(path, rgb):
            entered.set()
            release.wait(2)
            original(path, rgb)
        with patch.object(self.recorder, '_save_rgb', side_effect=slow):
            self.recorder.start_episode()
            self.assertTrue(entered.wait(1))
            began = time.monotonic()
            for i in range(100):
                self.recorder.write({'sequence': i})
            self.assertLess(time.monotonic() - began, .1)
            self.assertLessEqual(self.recorder.queue.qsize(), 4)
            self.assertEqual(self.recorder.status()['state'], 'finalizing')
            release.set()
            wait_until(lambda: self.recorder.status()['state'] == 'fault')
        self.assertGreater(self.recorder.status()['dropped_events'], 0)
        meta = json.loads((Path(self.recorder.status()['path']) / 'meta.json').read_text())
        self.assertEqual(meta['status'], 'incomplete')

    def test_missing_native_reference_prevents_complete_status(self):
        self.feed()
        original = self.recorder._raw
        def omit_joint(episode, sample):
            if sample['kind'] != 'joints':
                original(episode, sample)
        with patch.object(self.recorder, '_raw', side_effect=omit_joint):
            self.recorder.start_episode()
            wait_until(lambda: self.recorder.status()['frame_count'] >= 1)
            _, meta = self.finish()
        self.assertEqual(meta['status'], 'incomplete')
        self.assertIn('observation references missing native state samples', meta['integrity_errors'])

    def test_missed_scheduler_grid_faults_instead_of_compressing_time(self):
        self.recorder.close()
        now = [10.]
        self.recorder = CollectionRecorder(self.tmp.name, 'clock', {}, min_free_bytes=0, clock=lambda: now[0])
        self.recorder.open()
        self.addCleanup(self.recorder.close)
        for kind in ('joints', 'tcp', 'gripper'):
            self.recorder.publish_state(state(kind, 1, 10.))
        for role in ROLES:
            self.recorder.publish_frame(frame(role, 1, 10.))
        self.recorder.start_episode()
        wait_until(lambda: self.recorder.status()['frame_count'] == 1)
        now[0] = 10.2
        self.recorder._wake.set()
        wait_until(lambda: self.recorder.status()['state'] == 'fault')
        meta = json.loads((Path(self.recorder.status()['path']) / 'meta.json').read_text())
        self.assertEqual(meta['frame_count'], 1)
        self.assertIn('missed observation grid', meta['end_reason'])

    def test_close_joins_threads_and_active_segment_is_incomplete(self):
        self.feed()
        self.recorder.start_episode()
        wait_until(lambda: self.recorder.status()['frame_count'] >= 1)
        self.recorder.close()
        self.assertFalse(self.recorder.thread.is_alive())
        self.assertFalse(self.recorder.scheduler_thread.is_alive())
        self.assertFalse(self.recorder.status()['complete'])


if __name__ == '__main__':
    unittest.main()
