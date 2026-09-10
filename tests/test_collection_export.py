import contextlib
import csv
import hashlib
import io
import json
import math
import struct
import tempfile
import time
import unittest
import zlib
from pathlib import Path

from spacemouse_teleop.cli.collection_check import main as check_main
from spacemouse_teleop.cli.collection_export import main as export_main
from spacemouse_teleop.recording.export import check_episode, export_episode, rotation_delta_base


def png(color):
    def chunk(kind, value):
        return struct.pack('>I', len(value)) + kind + value + struct.pack('>I', zlib.crc32(kind + value) & 0xffffffff)
    return b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', 1, 1, 8, 2, 0, 0, 0)) + chunk(b'IDAT', zlib.compress(b'\0' + bytes(color))) + chunk(b'IEND', b'')


def write_json(path, value):
    path.write_text(json.dumps(value), encoding='utf-8')


def write_rows(path, rows):
    path.write_text(''.join(json.dumps(row) + '\n' for row in rows), encoding='utf-8')


def read_rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def fixture(root, count=5):
    root.mkdir()
    observations, raw = [], []
    for index in range(count):
        timestamp = 100. + index / 20
        joints = [.1 * index, 0., 0., 0., 0., 0.]
        tcp = [300. + index, 20., 100., .1, -.2, .01 * index]
        grip = {'position_pulse': 230. + 100 * index, 'controller_target_pulse': 850, 'status_code': 1,
                'motion_state': 'moving', 'error_code': 0, 'received_at': timestamp - .01}
        images, sources = {}, {}
        for role, color in [('wrist', [255, 0, 0]), ('fixed_1', [0, 255, 0]), ('fixed_2', [0, 0, 255])]:
            file = f'images/{role}/{index:06d}.png'
            path = root / file
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(png(color))
            images[role] = {'file': file, 'role': role, 'serial': f'serial-{role}', 'sequence': index,
                            'frame_id': index, 'received_at': timestamp - .005, 'device_timestamp_ms': index * 50.,
                            'timestamp_domain': 'hardware_clock', 'width': 1, 'height': 1, 'capture_fps': 30, 'age_s': .005}
        for kind, values in [('joints', {'joints_rad': joints}), ('tcp', {'tcp_mm_rad': tcp}), ('gripper', grip), ('report', {'state': 2})]:
            received = timestamp - .01
            source = {'kind': kind, 'sequence': index, 'received_at': received, 'values': values, 'valid': True}
            if kind in ('joints', 'tcp'):
                source.update(query_started_at=received-.002, query_finished_at=received)
            raw.append(source)
            sources[kind] = {'sequence': index, 'received_at': received, 'age_s': .01}
        observations.append({'index': index, 'timestamp': timestamp, 'relative_time_s': index / 20,
                             'images': images, 'joints_rad': joints, 'tcp_mm_rad': tcp, 'gripper': grip,
                             'sources': sources, 'camera_skew_s': 0., 'valid': True})
    write_json(root / 'meta.json', {'format_version': 1, 'task': 'pick object', 'sample_hz': 20, 'status': 'complete',
                                  'disposition': 'kept', 'start_time': 100., 'end_time': 100. + count / 20,
                                  'frame_count': count, 'config': {}, 'metadata': {}, 'stats': {}})
    write_rows(root / 'raw_states.jsonl', raw)
    write_rows(root / 'observations.jsonl', observations)
    return observations, raw


def digests(root):
    return {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in root.rglob('*') if p.is_file()}


class CollectionExportTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.root = self.base / 'episode'
        self.rows, self.raw = fixture(self.root)

    def export(self, **kwargs):
        return export_episode(self.root, self.base / 'export', **kwargs)

    def test_complete_episode_validates_and_pending_is_not_exportable(self):
        report = check_episode(self.root)
        self.assertTrue(report['valid'], report)
        self.assertEqual(report['sample_count'], 5)
        self.assertEqual(report['raw_state_count'], 20)
        meta = json.loads((self.root / 'meta.json').read_text())
        meta['disposition'] = 'pending'
        write_json(self.root / 'meta.json', meta)
        self.assertTrue(check_episode(self.root)['valid'])
        self.assertFalse(check_episode(self.root, require_kept=True)['valid'])
        with self.assertRaisesRegex(ValueError, 'kept'):
            self.export(format='light', fixed_view='fixed_1')

    def test_light_downsampling_recomputes_next_state_and_drops_terminal_action(self):
        before = digests(self.root)
        meta = self.export(format='light', fps=10, fixed_view='fixed_2')
        output = self.base / 'export'
        rows = read_rows(output / 'train.jsonl')
        self.assertEqual(meta['source_indices'], [0, 2, 4])
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0]['state'], [0., 0., 0., 0., 0., 0., 230.])
        self.assertEqual(rows[0]['actions'], [.2, 0., 0., 0., 0., 0., 430.])
        self.assertEqual(rows[1]['actions'], [.4, 0., 0., 0., 0., 0., 630.])
        self.assertAlmostEqual(rows[0]['dt_s'], .1)
        self.assertEqual([r['timestamp'] for r in rows], [0., .1])
        self.assertEqual((output / rows[0]['image']).read_bytes(), png([0, 0, 255]))
        self.assertEqual((output / rows[0]['wrist_image']).read_bytes(), png([255, 0, 0]))
        self.assertEqual(before, digests(self.root))

    def test_legacy_exact_columns_roles_and_historical_units(self):
        meta = self.export(format='legacy', fixed_view='fixed_1')
        with (self.base / 'export' / 'robot_log.csv').open() as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), 5)
        self.assertEqual(float(rows[0]['tcp_x_m']), 300.)
        self.assertEqual(float(rows[0]['gripper_mm']), 230.)
        self.assertEqual(float(rows[1]['j1_rad']), .1)
        self.assertIn('tcp_rx_rad', rows[0])
        self.assertEqual(meta['camera_mapping'], {'realsense_0': 'fixed_1', 'realsense_1': 'wrist'})
        self.assertIn('raw pulse', meta['legacy_unit_mapping']['gripper_mm'])
        self.assertEqual(float(rows[4]['ts']), .2)

    def test_derived_joint_wrap_and_so3_rotation_are_not_rpy_subtraction(self):
        yaw0, yaw1 = math.radians(179), math.radians(-179)
        self.rows[0]['tcp_mm_rad'][3:] = [0., 0., yaw0]
        self.rows[1]['tcp_mm_rad'][3:] = [0., 0., yaw1]
        self.rows[0]['joints_rad'][0] = yaw0
        self.rows[1]['joints_rad'][0] = yaw1
        # The fixture shares lists until serialized; refresh native references.
        write_rows(self.root / 'observations.jsonl', self.rows)
        write_rows(self.root / 'raw_states.jsonl', self.raw)
        meta = self.export(format='derived', gripper_open=850, gripper_close=230, binary_threshold=.25)
        rows = read_rows(self.base / 'export' / 'derived.jsonl')
        self.assertEqual(len(rows), 4)
        self.assertAlmostEqual(rows[0]['delta_joint_pose'][0], math.radians(2))
        self.assertAlmostEqual(rows[0]['delta_tcp_rotation_base_rad'][2], math.radians(2))
        self.assertEqual(rows[0]['delta_tcp_translation_base_m'], [.001, 0., 0.])
        self.assertEqual(rows[0]['gripper']['absolute_raw'], 230.)
        self.assertEqual(rows[0]['gripper']['delta_raw'], 100.)
        self.assertAlmostEqual(rows[0]['gripper']['next_normalized'], 100/620)
        self.assertEqual(rows[1]['gripper']['next_binary_open'], 1)
        self.assertIn('principal', meta['joint_delta_convention'])
        self.assertIn('R_next', meta['tcp_rotation_convention'])

    def test_rotation_delta_is_base_frame_for_nontrivial_orientation(self):
        # At yaw=pi/2, adding roll rotates around base Y, not base X.
        result = rotation_delta_base([0, 0, math.pi/2], [.1, 0, math.pi/2])
        self.assertAlmostEqual(result[0], 0., places=12)
        self.assertAlmostEqual(result[1], .1, places=12)
        self.assertAlmostEqual(result[2], 0., places=12)
        equivalent = rotation_delta_base([math.pi, 0., math.pi], [0., math.pi, 0.])
        self.assertLess(sum(v*v for v in equivalent), 1e-20)

    def test_derived_ten_hz_uses_selected_dt_and_gripper_delta(self):
        self.export(format='derived', fps=10, gripper_open=850, gripper_close=230)
        rows = read_rows(self.base / 'export' / 'derived.jsonl')
        self.assertEqual(rows[0]['next_source_index'], 2)
        self.assertAlmostEqual(rows[0]['delta_joint_pose'][0], .2)
        self.assertAlmostEqual(rows[0]['dt_s'], .1)
        self.assertEqual(rows[0]['gripper']['delta_raw'], 200.)

    def test_incomplete_and_discarded_are_rejected(self):
        original = json.loads((self.root / 'meta.json').read_text())
        for changes in ({'status': 'incomplete'}, {'disposition': 'discarded'}):
            with self.subTest(changes=changes):
                write_json(self.root / 'meta.json', dict(original, **changes))
                self.assertFalse(check_episode(self.root)['valid'])
                with self.assertRaises(ValueError):
                    self.export(format='light', fixed_view='fixed_1')

    def test_missing_or_reordered_sample_cannot_be_relabelled_as_regular_data(self):
        original = json.loads((self.root / 'meta.json').read_text())
        for rows in (self.rows[:2]+self.rows[3:], list(reversed(self.rows))):
            with self.subTest(indices=[r['index'] for r in rows]):
                write_rows(self.root / 'observations.jsonl', rows)
                write_json(self.root / 'meta.json', dict(original, frame_count=len(rows)))
                with self.assertRaisesRegex(ValueError, 'gap or reordering'):
                    self.export(format='legacy', fps=10, fixed_view='fixed_1')

    def test_timestamp_gap_even_with_contiguous_indices_is_rejected(self):
        self.rows[2]['timestamp'] += .01
        write_rows(self.root / 'observations.jsonl', self.rows)
        self.assertIn('continuous fixed grid', check_episode(self.root)['errors'][0])

    def test_missing_native_reference_and_value_mismatch_are_rejected(self):
        self.rows[1]['sources']['joints']['sequence'] = 999
        write_rows(self.root / 'observations.jsonl', self.rows)
        self.assertIn('native source', check_episode(self.root)['errors'][0])
        self.rows[1]['sources']['joints']['sequence'] = 1
        self.rows[1]['joints_rad'][0] = .99
        write_rows(self.root / 'observations.jsonl', self.rows)
        self.assertIn('differs from native', check_episode(self.root)['errors'][0])

    def test_missing_image_and_corrupt_png_are_rejected(self):
        image = self.root / self.rows[0]['images']['wrist']['file']
        image.unlink()
        self.assertIn('Missing image', check_episode(self.root)['errors'][0])
        image.write_bytes(png([1, 2, 3])[:-3])
        self.assertIn('PNG', check_episode(self.root)['errors'][0])

    def test_outside_image_path_and_symlink_are_rejected(self):
        self.rows[0]['images']['wrist']['file'] = '../outside.png'
        (self.base / 'outside.png').write_bytes(png([1, 2, 3]))
        write_rows(self.root / 'observations.jsonl', self.rows)
        self.assertIn('outside episode', check_episode(self.root)['errors'][0])
        linked = self.root / 'outside.png'
        linked.symlink_to(self.base / 'outside.png')
        self.rows[0]['images']['wrist']['file'] = 'outside.png'
        write_rows(self.root / 'observations.jsonl', self.rows)
        self.assertIn('outside episode', check_episode(self.root)['errors'][0])

    def test_source_age_and_future_data_are_rejected(self):
        self.rows[0]['sources']['tcp']['age_s'] = -.01
        write_rows(self.root / 'observations.jsonl', self.rows)
        self.assertIn('time/age mismatch', check_episode(self.root)['errors'][0])

    def test_nonfinite_state_is_rejected(self):
        self.rows[0]['joints_rad'][0] = float('nan')
        write_rows(self.root / 'observations.jsonl', self.rows)
        self.assertIn('Nonfinite', check_episode(self.root)['errors'][0])

    def test_single_terminal_sample_does_not_fabricate_action(self):
        meta = json.loads((self.root / 'meta.json').read_text())
        write_json(self.root / 'meta.json', dict(meta, frame_count=1))
        write_rows(self.root / 'observations.jsonl', self.rows[:1])
        self.assertTrue(check_episode(self.root)['valid'])
        with self.assertRaisesRegex(ValueError, 'no final action'):
            self.export(format='light', fixed_view='fixed_1')

    def test_roles_fps_and_mapping_require_explicit_valid_choices(self):
        for options in ({'format': 'legacy'}, {'format': 'light'}, {'format': 'derived'},
                        {'format': 'derived', 'gripper_open': 230, 'gripper_close': 850},
                        {'format': 'derived', 'gripper_open': 850, 'gripper_close': 230, 'binary_threshold': 2},
                        {'format': 'light', 'fixed_view': 'fixed_1', 'fps': 30}):
            with self.subTest(options=options):
                with self.assertRaises(ValueError):
                    self.export(**options)

    def test_output_nonempty_and_raw_subdirectory_are_rejected_without_changes(self):
        output = self.base / 'export'
        output.mkdir()
        marker = output / 'keep'
        marker.write_text('keep me')
        with self.assertRaisesRegex(ValueError, 'nonempty'):
            self.export(format='legacy', fixed_view='fixed_1')
        self.assertEqual(marker.read_text(), 'keep me')
        before = digests(self.root)
        with self.assertRaisesRegex(ValueError, 'separate'):
            export_episode(self.root, self.root / 'derived', format='derived', gripper_open=850, gripper_close=230)
        self.assertEqual(before, digests(self.root))

    def test_empty_existing_output_is_allowed(self):
        (self.base / 'export').mkdir()
        self.export(format='legacy', fixed_view='fixed_1')
        self.assertTrue((self.base / 'export' / 'robot_log.csv').is_file())

    def test_native_recorder_round_trip_is_accepted_without_schema_adaptation(self):
        import numpy as np
        from spacemouse_teleop.recording.collection import CollectionRecorder
        now = [100.]
        recorder = CollectionRecorder(self.base / "recorded", "test task", {}, min_free_bytes=0, clock=lambda: now[0])
        recorder.open()
        self.addCleanup(recorder.close)
        def wait_for(predicate):
            deadline = time.monotonic() + 2
            while not predicate() and time.monotonic() < deadline:
                time.sleep(.002)
            self.assertTrue(predicate(), recorder.status())
        for index in range(5):
            for source in self.raw[index*4:(index+1)*4]:
                recorder.publish_state(source)
            for frame in self.rows[index]["images"].values():
                recorder.publish_frame(dict(frame, rgb=np.zeros((1, 1, 3), dtype=np.uint8)))
            now[0] = 100. + index / 20
            if index == 0:
                self.assertIsNotNone(recorder.start_episode())
            recorder._wake.set()
            wait_for(lambda: recorder.status()["frame_count"] == index + 1)
        self.assertTrue(recorder.stop_episode())
        wait_for(lambda: recorder.status()["state"] == "review")
        self.assertTrue(recorder.disposition("kept"))
        wait_for(lambda: recorder.status()["state"] == "idle")
        path = recorder.status()["path"]
        recorder.close()
        report = check_episode(path, require_kept=True)
        self.assertTrue(report["valid"], report)
        result = export_episode(path, self.base / "round-trip", format="light", fixed_view="fixed_2", fps=10)
        self.assertEqual(result["training_pair_count"], 2)

    def test_recorded_age_limit_is_rechecked(self):
        meta = json.loads((self.root / 'meta.json').read_text())
        meta['quality'] = {'max_robot_age_s': .001}
        write_json(self.root / 'meta.json', meta)
        self.assertIn('age limit', check_episode(self.root)['errors'][0])

    def test_cli_returns_machine_readable_check_and_explicit_export_errors(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            status = check_main([str(self.root)])
        self.assertEqual(status, 0)
        self.assertTrue(json.loads(output.getvalue())['valid'])
        with contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(export_main([str(self.root), str(self.base/'bad'), '--format', 'legacy']), 1)
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(export_main([str(self.root), str(self.base/'ok'), '--format', 'light', '--fixed-view', 'fixed_2', '--fps', '10']), 0)


if __name__ == '__main__':
    unittest.main()
