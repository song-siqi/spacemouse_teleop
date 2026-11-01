"""Session-clock buffers; independent event consumption and bounded finalization."""
from collections import deque
from pathlib import Path
import copy
import queue
import threading
import time
from robot_teleop.core import Stamp
from robot_teleop.recording.v4 import RawEpisode
from robot_teleop.recording.archive import atomic_json


class EpisodeSession:
    def __init__(self, path, sources, *, clock="simulation", epoch=0, pre_roll=.5,
                 post_roll=.5, capacity=4096, metadata=None, source_capacities=None,
                 visual_storage=None, validation_profile=None, on_event=None,
                 video_factory=None, prepare_validation=None):
        if pre_roll < 0 or post_roll < 0 or capacity < 1:
            raise ValueError("Invalid buffer settings")
        self.path, self.sources = Path(path), sources
        self.clock, self.epoch = clock, epoch
        self.pre_roll, self.post_roll = pre_roll, post_roll
        self.metadata = metadata or {}
        self.episode_options = dict(capacity=source_capacities or 256,
                                    visual_storage=visual_storage,
                                    validation_profile=validation_profile)
        if video_factory is not None:
            self.episode_options['video_factory'] = video_factory
        self.on_event = on_event or (lambda event: None)
        self.prepare_validation = prepare_validation or (lambda: None)
        self.queue = queue.Queue(capacity)
        self.history = deque(maxlen=capacity)
        self.active = self.last = None
        self.deadline = None
        self.number = 0
        self.state = "idle"
        self.error = self.fatal_error = None
        self.boundary_time = float('-inf')
        self.finalizing = None
        self.finalizers = []
        self._closing = False
        self.thread = threading.Thread(target=self._run, name="raw4-session")
        self.thread.start()

    def _enqueue(self, kind, value):
        if self._closing:
            return False
        try:
            self.queue.put_nowait((kind, value))
            return True
        except queue.Full:
            self.error = "Session queue overflow"
            if self.active:
                self.active.fault(self.error)
            return False

    def publish(self, source_id, sequence, source_time, values, **kwargs):
        item = (source_id, sequence, source_time, copy.deepcopy(values), copy.deepcopy(kwargs))
        return self._enqueue("sample", item)

    def start(self, stamp):
        self._check_clock(stamp)
        return self._enqueue("start", stamp.seconds)

    def stop(self, stamp, incomplete=False, reason="user"):
        self._check_clock(stamp)
        return self._enqueue("stop", (stamp.seconds, incomplete, str(reason)))

    def tick(self, stamp):
        self._check_clock(stamp, epoch=False)
        return self._enqueue("tick", stamp)

    def boundary(self, stamp, reason):
        self._check_clock(stamp, epoch=False)
        return self._enqueue("boundary", (stamp.seconds, str(reason)))

    def fail(self, stamp, reason):
        self._check_clock(stamp, epoch=False)
        return self._enqueue("fault", (stamp.seconds, str(reason)))

    def _check_clock(self, stamp, epoch=True):
        if stamp.clock != self.clock or epoch and stamp.epoch != self.epoch:
            raise ValueError("Session clock mismatch")

    def disposition(self, value):
        if value not in ("kept", "discarded"):
            raise ValueError("Invalid disposition")
        return self._enqueue("disposition", value)

    def _event(self, name, at, **fields):
        self.on_event(dict(event=name, received_at=at,
                           episode_id=self.last.path.name if self.last else None, **fields))

    def _finish(self, complete=True, at=None, reason=None):
        episode, self.active = self.active, None
        if episode:
            at = at if at is not None else episode.meta.get('end_time', 0.)
            if episode.meta.get('end_time') is None:
                episode.meta['end_time'] = at
            if reason is not None:
                episode.meta['end_reason'] = reason
            if not complete:
                episode.meta['capture_requested_end_time'] = at
            self.boundary_time = max(self.boundary_time, at)
            self.history.clear()
            duration = max(0., episode.meta['end_time'] - episode.meta['start_time'])
            episode.finalization_deadline = time.monotonic() + max(120., 2 * duration)
            self.finalizing = episode
            self.state = 'finalizing'
            self._event('capture_closed', at, reason=episode.meta['end_reason'], incomplete=not complete)

            eligible = complete and not self.error

            def finalize():
                try:
                    can_commit = eligible
                    try:
                        self.prepare_validation()
                    except Exception as exc:
                        episode.fault(f'Session evidence flush: {exc}')
                        can_commit = False
                    episode.close(complete=can_commit, disposition='pending',
                                  deadline=episode.finalization_deadline)
                except Exception as exc:
                    self.fatal_error = f'Finalization failed: {exc}'
                    episode.fault(self.fatal_error)
                    episode.meta.update(status='incomplete', capture_complete=False,
                                        errors=list(episode.errors))
                    try:
                        atomic_json(episode.path/'meta.json', episode.meta)
                    except Exception:
                        pass
                if episode.fatal_error:
                    self.fatal_error = episode.fatal_error
                if self.finalizing is episode:
                    self.finalizing = None
                    if self.active is None:
                        self.state = 'fault' if self.fatal_error else 'idle'
                self._event('episode_finalized', time.monotonic(), status=episode.meta['status'])

            thread = threading.Thread(target=finalize, name='raw4-finalizer')
            self.finalizers.append(thread)
            thread.start()
        self.deadline = None

    def _advance(self, stamp):
        if stamp.epoch != self.epoch:
            if stamp.epoch < self.epoch:
                raise ValueError('Stale session epoch')
            self._finish(False, stamp.seconds, 'session_clock_reset')
            self.history.clear()
            self.boundary_time = stamp.seconds
            self.epoch = stamp.epoch
        if self.deadline is not None and stamp.seconds > self.deadline:
            self._finish(at=self.deadline)

    def _run(self):
        try:
            while True:
                kind, value = self.queue.get()
                if kind == 'close':
                    if self.active:
                        self._finish(False, value, 'session_closed_during_recording')
                    return
                if self.error and self.active:
                    self.active.fault(self.error)
                    at = value.seconds if kind == 'tick' else self.active.meta['start_time']
                    self._finish(False, at, self.error)
                if kind == 'tick':
                    self._advance(value)
                    if self.deadline is not None and value.seconds >= self.deadline:
                        self._finish(at=self.deadline)
                elif kind == 'sample':
                    t = value[4]['session_time']
                    epoch = value[4].setdefault('epoch', self.epoch)
                    self._advance(Stamp(t, self.clock, epoch))
                    if t < self.boundary_time:
                        continue
                    while self.history and self.history[0][4]['session_time'] < t-self.pre_roll:
                        self.history.popleft()
                    if len(self.history) == self.history.maxlen:
                        self.error = 'Pre-roll capacity exceeded'
                    self.history.append(value)
                    if self.active:
                        if not self.active.publish(*value[:4], **value[4]):
                            self.error = '; '.join(self.active.errors) or 'Raw source queue overflow'
                elif kind == 'start':
                    if self.active or self.finalizing or self.fatal_error:
                        continue
                    self.error = None
                    self.number += 1
                    self.active = RawEpisode(self.path/f'episode_{self.number:06d}', self.sources,
                        clock=self.clock, epoch=self.epoch, metadata=self.metadata, **self.episode_options)
                    self.last = self.active
                    self.active.meta.update(start_time=value, end_time=None, end_reason=None,
                        pre_roll_s=self.pre_roll, post_roll_s=self.post_roll,
                        capture_requested_start_time=max(self.boundary_time, value-self.pre_roll),
                        capture_requested_end_time=None)
                    atomic_json(self.active.path/'meta.json', self.active.meta)
                    self.state = 'recording'
                    for row in self.history:
                        if row[4]['session_time'] >= max(self.boundary_time, value-self.pre_roll):
                            if not self.active.publish(*row[:4], **row[4]):
                                self.error = '; '.join(self.active.errors)
                    self._event('episode_start', value)
                elif kind == 'stop' and self.active:
                    now, incomplete, reason = value
                    self.active.meta.update(end_time=now, end_reason=reason,
                        capture_requested_end_time=now if incomplete else now+self.post_roll)
                    if incomplete:
                        self._finish(False, now)
                    else:
                        self.deadline = now+self.post_roll
                        self.state = 'postroll'
                        self._event('episode_stop_requested', now)
                        if self.post_roll == 0:
                            self._finish(at=now)
                elif kind in ('boundary', 'fault'):
                    now, reason = value
                    if self.active:
                        if kind == 'fault':
                            self.active.fault(reason)
                        self._finish(False, now, reason)
                    self.history.clear()
                    self.boundary_time = now
                    if kind == 'boundary':
                        self.epoch += 1
                        self._event(reason, now, boundary=True)
                elif kind == 'disposition' and self.last and not self.active and not self.finalizing:
                    self.last.meta['disposition'] = value
                    atomic_json(self.last.path/'meta.json', self.last.meta)
                    self.error = None
                    self.state = 'idle'
                elif kind == 'barrier':
                    value.set()
        except Exception as exc:
            self.error = self.fatal_error = str(exc)
            if self.active:
                self.active.fault(self.error)
                self._finish(False, self.active.meta['start_time'], self.error)
            self.state = 'fault'

    def flush(self, wait_finalization=True):
        event = threading.Event()
        if not self._enqueue('barrier', event) or not event.wait(15):
            raise RuntimeError(self.error or 'Session flush timed out')
        if wait_finalization:
            for thread in self.finalizers:
                episode = self.finalizing
                limit = episode.finalization_deadline if episode else time.monotonic()+5.
                thread.join(max(0., limit-time.monotonic())+3.)
                if thread.is_alive():
                    raise RuntimeError('Finalization did not finish before its deadline')

    def close(self):
        if self.thread.is_alive():
            self._closing = True
            last_time = max((r[4]['session_time'] for r in self.history), default=0.)
            self.queue.put(('close', last_time), timeout=10)
            self.thread.join(10)
            if self.thread.is_alive():
                raise RuntimeError('Session event consumer failed to stop')
        for thread in self.finalizers:
            episode = self.finalizing
            limit = episode.finalization_deadline if episode else time.monotonic()+5.
            thread.join(max(0., limit-time.monotonic())+3.)
            if thread.is_alive():
                raise RuntimeError('Finalization failed to stop')
