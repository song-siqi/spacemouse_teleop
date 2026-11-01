"""xArm protocol semantics added to generic raw-v4 storage validation; no SDK IO."""
import json
import math
from robot_teleop.recording.visual import file_sha256
from .report import decode_report


def validate_sources(root, meta):
    session = root.parent.parent
    evidence = meta['metadata'].get('session_evidence', {})
    expected = {'session.json', 'control.jsonl', 'events.jsonl', 'clock_anchors.jsonl'}
    if set(evidence) != expected:
        raise ValueError('Missing shared session evidence')
    for name, relative in evidence.items():
        path = (root / relative).resolve()
        if path != (session / name).resolve() or not path.is_file():
            raise ValueError(f'Invalid session evidence reference: {name}')
    session_meta = json.loads((session/'session.json').read_text())
    if session_meta.get('status') == 'incomplete':
        raise ValueError('Session evidence is incomplete')
    if session_meta.get('status') == 'closed':
        checksums = session_meta.get('checksums', {})
        if set(checksums) != expected-{'session.json'}:
            raise ValueError('Session evidence manifest incomplete')
        for name, checksum in checksums.items():
            if file_sha256(session/name) != checksum:
                raise ValueError(f'Session checksum mismatch: {name}')
    anchors = 0
    with (session/'clock_anchors.jsonl').open() as handle:
        for line in handle:
            row = json.loads(line)
            before, after = row['monotonic_before_ns'], row['monotonic_after_ns']
            if before > after or after-before != row['read_interval_ns']:
                raise ValueError('Invalid host clock bracket')
            anchors += 1
    if not anchors:
        raise ValueError('Missing host clock brackets')
    sources = {s['id']: s for s in meta['sources']}
    if not {'robot', 'gripper'} <= sources.keys():
        raise ValueError('Missing required xArm robot/gripper evidence')
    source = sources['robot']
    with (root / source['binary_path']).open('rb') as binary, (root / source['path']).open() as index:
        for line in index:
            row = json.loads(line)
            native = row['values']
            binary.seek(row['offset'])
            decoded = decode_report(binary.read(row['length']))
            for key in ('values', 'device_timestamp_raw', 'device_timestamp_unit', 'valid'):
                if native.get(key) != decoded[key]:
                    raise ValueError(f'xArm original packet differs from decoded {key}')
            if row['valid'] != native['valid'] or row['host_received_at'] != native['received_at']:
                raise ValueError('xArm source header differs from native evidence')
            if row['source_timestamp'] != native['device_timestamp_raw'] / 1e6:
                raise ValueError('xArm device timestamp conversion differs')
    for source in sources.values():
        with (root / source['path']).open() as index:
            for line in index:
                row = json.loads(line)
                v = row['values']
                if source['type'] == 'rgb':
                    if (v['role'] != source['id'] or v['serial'] != source['clock'].split(':',1)[1]
                            or v['capture_fps'] != source['video']['fps']
                            or v['width'] != source['video']['width']
                            or v['height'] != source['video']['height']
                            or row['source_timestamp'] != v['device_timestamp_ms'] / 1000
                            or row['host_received_at'] != v['received_at']):
                        raise ValueError('Camera source header differs from frame evidence')
                if row['clock_epoch'] != v.get('clock_epoch',0):
                    raise ValueError('Source clock segment differs from native evidence')
                if source['id'] == 'gripper':
                    if row['valid']:
                        pos = v.get('values',{}).get('position_pulse')
                        if isinstance(pos,bool) or not isinstance(pos,(int,float)) or not math.isfinite(pos):
                            raise ValueError('Missing measured gripper position')
                    if not v['query_started_at'] <= v['query_finished_at'] <= v['received_at']:
                        raise ValueError('Invalid gripper query interval')
