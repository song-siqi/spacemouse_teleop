"""Native xArm evidence and operational boundaries adapted to raw v4."""
from datetime import datetime, timezone
import copy
import itertools
import math
from pathlib import Path
import re
import shutil
import threading
import time
import uuid
from robot_teleop.core import Stamp
from robot_teleop.recording.v4 import Source
from robot_teleop.recording.archive import atomic_json
from robot_teleop.recording.native_writer import SessionWriter, NATIVE_VIDEO_SETTINGS
from robot_teleop.recording.clocks import ClockSampler
from robot_teleop.recording.visual import video_preflight, file_sha256
from robot_teleop.runtime.episodes import EpisodeSession
from .report import REPORT_SEMANTICS


class XArmCollectionRecorder:
    def __init__(self, output_root, task, config_snapshot, *, cameras=None, pre_roll_s=.5,
                 post_roll_s=.5, source_timeout_s=1., clock_interval_s=1., video_queue_size=64,
                 state_queue_size=4096, log_queue_size=4096, min_free_bytes=10*1024**3,
                 metadata=None, visual_storage=None, clock=time.monotonic, video_factory=None):
        self.root, self.task = Path(output_root), task
        self.config = copy.deepcopy(config_snapshot)
        self.cameras = copy.deepcopy(cameras or [])
        self.metadata = copy.deepcopy(metadata or {})
        self.pre, self.post, self.timeout = pre_roll_s, post_roll_s, source_timeout_s
        self.capacity = max(state_queue_size, video_queue_size*max(1,len(self.cameras)))
        self.video_capacity, self.state_capacity = video_queue_size, state_queue_size
        self.log_capacity, self.clock_interval = log_queue_size, clock_interval_s
        self.visual_storage = {**NATIVE_VIDEO_SETTINGS, **(visual_storage or {})}
        self.min_free_bytes, self.clock, self.video_factory = min_free_bytes, clock, video_factory
        self.session = self.log = self.clocks = None
        self.latest = {}
        self.error = self.fatal_error = None
        self._stop = threading.Event()
        self._sequence = itertools.count()
        self._closed = False
        self._state = None
        self._video_tools = None
        self._continuity = {}
        self._free_bytes = self._space_checked_at = None
        self.control_stats = dict(sent=0, waiting_for_feedback=0, failed=0)

    def preflight(self):
        if self.cameras and self._video_tools is None:
            self._video_tools = video_preflight()

    def open(self):
        if self.session is not None or self._closed:
            raise RuntimeError('Recorder can only be opened once')
        self.preflight()
        task = re.sub(r'[^\w.-]+', '_', self.task).strip('._') or 'task'
        self.session_path = self.root/task/f'session_{datetime.now(timezone.utc):%Y%m%dT%H%M%S}_{uuid.uuid4().hex[:8]}'
        self.session_path.mkdir(parents=True)
        self._check_space()
        if self._free_bytes < self.min_free_bytes:
            raise OSError('Insufficient disk reserve')
        self.sources = [Source('robot', 'robot_state', 'xarm_tcp30000_prefix_v1', REPORT_SEMANTICS['units'], 'link_base', 'robot_tcp30000', binary=True),
                        Source('gripper', 'gripper_state', 'xarm-pulse/v1', {'position':'pulse'}, 'tool', 'host_monotonic'),
                        Source('report', 'robot_state', 'xarm-rich-report/v1', {'tcp':'mm,rad(RPY)'}, 'link_base', 'host_monotonic', required=False),
                        Source('commands', 'command', 'xarm-control-log/v1', {'linear':'mm/s','angular':'rad/s'}, 'link_base', 'host_monotonic', required=False)]
        mock = self.metadata.get('capture_mode') == 'mock'
        self.sources += [Source(c['role'], 'rgb', 'realsense-rgb/v1', {'source_timestamp':'s','native_device_timestamp':'ms'}, c['role'],
                                f"{'mock_camera' if mock else 'camera'}:{c['serial']}",
                                video={k:c[k] for k in ('width','height','fps')}) for c in self.cameras]
        self.log = SessionWriter(self.session_path, self.mark_fault, self.log_capacity)
        self.log.start()
        self.session = EpisodeSession(self.session_path/'episodes', self.sources, clock='host_monotonic',
            pre_roll=self.pre, post_roll=self.post, capacity=self.capacity,
            source_capacities={s.id:self.video_capacity if s.video else self.state_capacity for s in self.sources},
            metadata={'task':self.task,'session_id':self.session_path.name,
                      'config':self.config,'provenance':self.metadata,'report_semantics':REPORT_SEMANTICS,
                      'clock_mapping_status':'evidence_only_not_calibrated','fixed_offset_uncertainty_s':None,
                      'session_evidence':{name:'../../'+name for name in ('session.json','control.jsonl','events.jsonl','clock_anchors.jsonl')}},
            visual_storage=self.visual_storage, validation_profile='xarm',
            on_event=self._session_event, video_factory=self.video_factory, prepare_validation=self.log.flush)
        self.clocks = ClockSampler(lambda r:self.log.submit('clock',r), self.publish_event,
                                   self.clock_interval, on_error=self.mark_fault)
        self.clocks.start()
        self.meta = dict(format_version=4, status='recording', task=self.task,
                         metadata=self.metadata, config=self.config, visual_storage=self.visual_storage,
                         video_tools=self._video_tools, robot_report_semantics=REPORT_SEMANTICS,
                         clock_semantics={'host':'monotonic seconds, same process session',
                                          'cross_device_mapping':'evidence only; not calibrated',
                                          'fixed_offset_uncertainty_s':None})
        atomic_json(self.session_path/'session.json', self.meta)
        self.thread = threading.Thread(target=self._supervise, name='raw4-xarm-supervisor')
        self.thread.start()

    def _stamp(self):
        return Stamp(self.clock(), 'host_monotonic', self.session.epoch)

    def _check_space(self):
        self._free_bytes = shutil.disk_usage(self.session_path).free
        self._space_checked_at = self.clock()

    def publish_state(self, row):
        if not self.session or self._closed:
            return
        source, received = row['kind'], row['received_at']
        if row.get('valid') is True:
            self.latest[source] = received
        values = {k:v for k,v in row.items() if k != 'packet'}
        stamp = row['device_timestamp_raw']/1e6 if source == 'robot' else received
        if not row.get('valid', True):
            self.publish_event(dict(event='invalid_state', source=source,
                                    received_at=received, sequence=row['sequence']))
        if not self.session.publish(source, int(row['sequence']), stamp, values, session_time=received,
                source_epoch=row.get('clock_epoch',0), received_at=received,
                valid=row.get('valid',True), packet=row.get('packet')):
            self.mark_fault(self.session.error or 'State publish failed')

    def publish_frame(self, row):
        if not self.session or self._closed:
            return
        received, role = row['received_at'], row['role']
        spec = next((c for c in self.cameras if c['role'] == role), None)
        rgb = row['rgb']
        if (spec is None or row['serial'] != spec['serial'] or
                getattr(rgb,'shape',None) != (spec['height'],spec['width'],3) or
                str(getattr(rgb,'dtype','')) != 'uint8'):
            self.mark_fault('Camera identity/dimensions differ from configuration')
            return
        self.latest[role] = received
        if not self.session.publish(role, int(row['sequence']), row['device_timestamp_ms']/1000,
                {k:v for k,v in row.items() if k != 'rgb'}, session_time=received,
                source_epoch=row.get('clock_epoch',0), received_at=received, rgb=rgb):
            self.mark_fault(self.session.error or 'Frame publish failed')

    def write(self, row):
        if self.log:
            self.log.submit('control', row)
        execution = row.get('execution', {})
        if execution.get('waiting_for_feedback'):
            self.control_stats['waiting_for_feedback'] += 1
        elif 'send_started_at' in execution:
            self.control_stats['sent' if execution.get('return_code') == 0 else 'failed'] += 1
        if self.session and not self._closed:
            t = row.get('timestamp', self.clock())
            if 'command' in row:
                t = row['command']['timestamp']
            if not self.session.publish('commands', next(self._sequence), t, row, session_time=t):
                self.mark_fault(self.session.error or 'Control evidence publish failed')

    def _session_event(self, row):
        if self.log:
            self.log.submit('event', row)
        if row['event'] == 'episode_start':
            self._continuity = {}
        elif row['event'] == 'capture_closed' and self.session.last:
            self.session.last.meta['continuity'] = dict(events=dict(self._continuity), has_events=bool(self._continuity))

    def publish_event(self, row):
        if self.log:
            self.log.submit('event', row)
        if self.session and self.session.active:
            name = row.get('event','source_event')
            self._continuity[name] = self._continuity.get(name,0)+1
        if row.get('boundary'):
            self.mark_boundary(row.get('event','source_reset'))

    def mark_boundary(self, reason):
        if self.session:
            self.latest.clear()
            self.session.boundary(self._stamp(), reason)

    def mark_fault(self, error):
        self.error = str(error)
        if isinstance(error, dict) and error.get('source'):
            self.latest.pop(error['source'], None)
        if self.log and not self.log.error:
            self.log.submit('event', dict(event='recording_fault', received_at=self.clock(), reason=self.error))
        if self.session:
            self.session.fail(self._stamp(), self.error)

    def start_episode(self):
        if self.status()['state'] != 'idle':
            return None
        now = self.clock()
        self._check_space()
        if (self._free_bytes < self.min_free_bytes or self.log.error or self.clocks.error or
                any(not 0 <= now-self.latest.get(s.id,-1e9) <= self.timeout for s in self.sources if s.required)):
            return None
        self.error = None
        self._state = 'recording'
        if self.session.start(self._stamp()):
            return 'queued'
        self.mark_fault(self.session.error or 'Cannot queue episode start')
        return None

    def stop_episode(self, reason='user', incomplete=False):
        if self.session and self.status()['state'] in ('recording','postroll'):
            self.session.stop(self._stamp(), incomplete, reason)
            self._state = 'finalizing' if incomplete else 'postroll'

    def disposition(self, value):
        if self.status()['state'] not in ('review','fault') or not self.session.last or self.session.finalizing:
            return False
        self.session.disposition(value)
        self._state = 'idle'
        self.error = None
        return True

    def status(self):
        session = self.session
        last = session.last if session else None
        complete = last is not None and last.meta['status'] == 'complete'
        state = session.state if session else 'idle'
        if state == 'recording' and self._state in ('postroll','finalizing'):
            state = self._state
        if session and session.finalizing:
            state = 'finalizing'
        elif state == 'idle':
            if last and last.meta['disposition'] == 'pending':
                state = 'review' if complete else 'fault'
            elif self._state in ('recording','postroll','finalizing'):
                state = self._state
        fatal = self.fatal_error or (session.fatal_error if session else None)
        if fatal:
            state = 'fault'
        counts = dict(last.counts) if last else {}
        frame_counts = {c['role']:counts.get(c['role'],0) for c in self.cameras}
        deadline = getattr(last,'finalization_deadline',None)
        return dict(state=state, complete=complete, error=self.error or (session.error if session else None) or (self.log.error if self.log else None),
            fatal_error=fatal, episode_id=last.path.name if last else None,
            episode_path=str(last.path) if last else None, path=str(last.path) if last else None,
            session_path=str(self.session_path) if session else None,
            disposition=last.meta['disposition'] if last else None,
            elapsed_s=max(0., (last.meta.get('end_time') or self.clock())-last.meta['start_time']) if last else 0.,
            frame_count=sum(frame_counts.values()), frame_counts=frame_counts,
            state_counts={k:counts.get(k,0) for k in ('robot','gripper','report')},
            captured_frame_counts={c['role']:last.accepted[c['role']] if last else 0 for c in self.cameras},
            queue_high_water=dict(last.high_water) if last else {}, control_stats=dict(self.control_stats),
            free_bytes=self._free_bytes, space_checked_at=self._space_checked_at,
            finalization_deadline=deadline,
            finalization_remaining_s=max(0.,deadline-time.monotonic()) if deadline else 120.)

    def _supervise(self):
        try:
            while not self._stop.wait(.02):
                now = self.clock()
                self.session.tick(self._stamp())
                episode = self.session.active
                if episode:
                    end = episode.meta.get('capture_requested_end_time')
                    when = min(now,end) if end is not None else now
                    if episode.errors:
                        self.mark_fault('; '.join(episode.errors))
                    else:
                        for source in self.sources:
                            if source.required and when-self.latest.get(source.id,-1e9)>self.timeout:
                                self.mark_fault(f'Source stopped: {source.id}')
                                break
                if self._space_checked_at is None or now-self._space_checked_at >= .5:
                    self._check_space()
                    if episode and self._free_bytes < self.min_free_bytes:
                        self.mark_fault('Disk reserve breached')
        except Exception as exc:
            self.fatal_error = f'Recording supervisor failed: {exc}'
            self.mark_fault(self.fatal_error)

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._stop.set()
        if hasattr(self,'thread'):
            self.thread.join(2)
        if self.clocks:
            self.clocks.close()
        try:
            if self.session:
                self.session.close()
        finally:
            if self.log:
                self.log.close()
                error = self.fatal_error or (self.session.fatal_error if self.session else None) or self.log.error
                self.meta.update(status='incomplete' if error else 'closed', error=error,
                    recording_error=self.error,
                    checksums={p.name:file_sha256(p) for p in self.session_path.glob('*.jsonl')})
                atomic_json(self.session_path/'session.json',self.meta)
