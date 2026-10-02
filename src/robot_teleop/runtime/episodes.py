"""Source-independent session-clock pre/post buffers and epoch boundaries."""
from collections import deque
from pathlib import Path
import queue
import threading
from robot_teleop.recording.v4 import RawEpisode
from robot_teleop.recording.archive import atomic_json


class EpisodeSession:
    def __init__(self, path, sources, *, clock="simulation", epoch=0, pre_roll=.5,
                 post_roll=.5, capacity=4096, metadata=None):
        if pre_roll < 0 or post_roll < 0 or capacity < 1:
            raise ValueError("Invalid buffer settings")
        self.path, self.sources = Path(path), sources
        self.clock, self.epoch = clock, epoch
        self.pre_roll, self.post_roll = pre_roll, post_roll
        self.metadata = metadata or {}
        self.queue = queue.Queue(capacity)
        self.history = deque(maxlen=capacity)
        self.active = self.last = None
        self.deadline = None
        self.number = 0
        self.state = "idle"
        self.error = None
        self.thread = threading.Thread(target=self._run, name="raw4-session")
        self.thread.start()

    def _enqueue(self, kind, value):
        try:
            self.queue.put_nowait((kind, value))
            return True
        except queue.Full:
            self.error = "Session queue overflow"
            if self.active:
                self.active.fault(self.error)
            return False

    def publish(self, source_id, sequence, source_time, values, **kwargs):
        import copy
        item = (source_id, sequence, source_time, copy.deepcopy(values), copy.deepcopy(kwargs))
        return self._enqueue("sample", item)

    def start(self, stamp):
        if stamp.clock != self.clock or stamp.epoch != self.epoch:
            raise ValueError("Session clock mismatch")
        return self._enqueue("start", stamp.seconds)

    def stop(self, stamp, incomplete=False):
        if stamp.clock != self.clock or stamp.epoch != self.epoch:
            raise ValueError("Session clock mismatch")
        return self._enqueue("stop", (stamp.seconds, incomplete))

    def tick(self, stamp):
        if stamp.clock != self.clock:
            raise ValueError("Session clock mismatch")
        return self._enqueue("tick", stamp)

    def disposition(self, value):
        if value not in ("kept", "discarded"):
            raise ValueError("Invalid disposition")
        return self._enqueue("disposition", value)

    def _finish(self, complete=True):
        if self.active:
            self.state = "finalizing"
            self.active.close(complete=complete and self.error is None, disposition="pending")
            self.last, self.active = self.active, None
            self.state = "idle"
        self.deadline = None

    def _advance(self, stamp):
        if stamp.epoch != self.epoch:
            self._finish(False)
            self.history.clear()
            self.epoch = stamp.epoch
        if self.deadline is not None and stamp.seconds >= self.deadline:
            self._finish()

    def _run(self):
        try:
            while True:
                kind, value = self.queue.get()
                if kind == "close":
                    self._finish(False)
                    return
                if kind == "tick":
                    self._advance(value)
                elif kind == "sample":
                    t = value[4]["session_time"]
                    epoch = value[4].get("epoch", self.epoch)
                    from robot_teleop.core import Stamp
                    self._advance(Stamp(t, self.clock, epoch))
                    while self.history and self.history[0][4]["session_time"] < t-self.pre_roll:
                        self.history.popleft()
                    if len(self.history) == self.history.maxlen:
                        self.error = "Pre-roll capacity exceeded"
                    self.history.append(value)
                    if self.active:
                        self.active.publish(*value[:4], **value[4])
                elif kind == "start":
                    if self.active:
                        continue
                    self.number += 1
                    self.active = RawEpisode(self.path/f"episode_{self.number:06d}", self.sources,
                        clock=self.clock, epoch=self.epoch, metadata=self.metadata)
                    self.state = "recording"
                    if self.error:
                        self.active.fault(self.error)
                    for row in self.history:
                        if row[4]["session_time"] >= value-self.pre_roll:
                            self.active.publish(*row[:4], **row[4])
                elif kind == "stop" and self.active:
                    now, incomplete = value
                    if incomplete:
                        self._finish(False)
                    else:
                        self.deadline = now+self.post_roll
                        self.state = "postroll"
                        if self.post_roll == 0:
                            self._finish()
                elif kind == "disposition" and self.last:
                    self.last.meta["disposition"] = value
                    atomic_json(self.last.path/"meta.json", self.last.meta)
                elif kind == "barrier":
                    value.set()
        except Exception as exc:
            self.error = str(exc)
            self._finish(False)
            self.state = "fault"

    def flush(self):
        event = threading.Event()
        if not self._enqueue("barrier", event) or not event.wait(15):
            raise RuntimeError(self.error or "Session flush timed out")

    def close(self):
        if self.thread.is_alive():
            self.queue.put(("close", None), timeout=10)
            self.thread.join(30)
            if self.thread.is_alive():
                raise RuntimeError("Session writer failed to stop")
