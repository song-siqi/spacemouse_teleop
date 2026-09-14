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
from spacemouse_teleop.recording.raw import check_episode


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


class LegacyRawCheckTests(unittest.TestCase):
    def setUp(self):
            self.temp = tempfile.TemporaryDirectory()
            self.addCleanup(self.temp.cleanup)
            self.base = Path(self.temp.name)
            self.root = self.base / 'episode'
            self.rows, self.raw = fixture(self.root)

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

    def test_recorded_age_limit_is_rechecked(self):
            meta = json.loads((self.root / 'meta.json').read_text())
            meta['quality'] = {'max_robot_age_s': .001}
            write_json(self.root / 'meta.json', meta)
            self.assertIn('age limit', check_episode(self.root)['errors'][0])
