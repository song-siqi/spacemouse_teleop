"""Event-driven raw v3 recording. No observation grid or training processing."""
from __future__ import annotations
import copy
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import queue
import re
import shutil
import threading
import time
import uuid

from .archive import atomic_json, sync_directory
from .clocks import ClockSampler
from .native_writer import (SessionWriter, StateStreamWriter, VideoStreamWriter,
                            StreamingVideo, NATIVE_VIDEO_SETTINGS, inspect_surviving_video)
from .visual import video_preflight, VideoRunner, file_sha256
from spacemouse_teleop.backends.xarm6_camera.report import REPORT_SEMANTICS

ROLES = ("wrist", "fixed_1", "fixed_2")

class CollectionFault(RuntimeError):
    pass

def _number(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)

def _json_safe(value):
    if isinstance(value, float) and not math.isfinite(value): return None
    if isinstance(value, dict): return {k: _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)): return [_json_safe(v) for v in value]
    return value

@dataclass
class _Episode:
    identifier: str
    path: Path
    meta: dict
    videos: dict = field(default_factory=dict)
    states: object = None
    incomplete: bool = False
    errors: list = field(default_factory=list)
    gaps: dict = field(default_factory=dict)
    deadline: float | None = None
    finish_queued: bool = False

class CollectionRecorder:
    """Publishers enqueue owned frames or small records; only workers touch disk."""
    def __init__(self, output_root, task, config_snapshot, *, cameras=None,
                 pre_roll_s=.5, post_roll_s=.5, source_timeout_s=1., clock_interval_s=1.,
                 video_queue_size=64, state_queue_size=4096, log_queue_size=4096,
                 min_free_bytes=10 * 1024**3, metadata=None, visual_storage=None,
                 clock=time.monotonic, video_factory=StreamingVideo):
        self.cameras = copy.deepcopy(cameras or config_snapshot.get("collection", {}).get("cameras", []))
        if len(self.cameras) != 3 or {c["role"] for c in self.cameras} != set(ROLES):
            raise ValueError("Raw v3 requires wrist, fixed_1, fixed_2 cameras")
        for value in (pre_roll_s, post_roll_s):
            if not _number(value) or value < 0: raise ValueError("Buffer durations must be nonnegative")
        for value in (source_timeout_s, clock_interval_s):
            if not _number(value) or value <= 0: raise ValueError("Source/clock intervals must be positive")
        for value in (video_queue_size, state_queue_size, log_queue_size):
            if not isinstance(value, int) or isinstance(value, bool) or value < 4:
                raise ValueError("Independent queue sizes must be integers >= 4")
        if min_free_bytes < 0: raise ValueError("min_free_bytes must be nonnegative")
        self.output_root, self.task = Path(output_root), str(task)
        if not self.task.strip(): raise ValueError("Task must be nonempty")
        self.config_snapshot, self.metadata = copy.deepcopy(config_snapshot), copy.deepcopy(metadata or {})
        self.pre_roll_s, self.post_roll_s = float(pre_roll_s), float(post_roll_s)
        self.source_timeout_s, self.clock_interval_s = source_timeout_s, clock_interval_s
        self.video_queue_size, self.state_queue_size = video_queue_size, state_queue_size
        self.log_queue_size, self.min_free_bytes = log_queue_size, min_free_bytes
        self.visual_storage = {**NATIVE_VIDEO_SETTINGS, **(visual_storage or {})}
        self.clock, self.video_factory = clock, video_factory
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._ops = queue.Queue(16)
        self._history = {}
        self._latest = {}
        self._last_identity = {}
        self._boundary_time = -math.inf
        self._state, self._error, self._fatal_error = "idle", None, None
        self._active = self._last = None
        self._episode_number = 0
        self._free_bytes = self._space_checked_at = None
        self._opened = self._closed = False
        self._video_tools = None
        self.session_path = self.thread = self.session_writer = self.clocks = None

    def preflight(self):
        if self._video_tools is None:
            self._video_tools = video_preflight()

    def open(self):
        self.preflight()
        with self._lock:
            if self._opened or self._closed: raise RuntimeError("Recorder can only be opened once")
            task = re.sub(r"[^\w.-]+", "_", self.task).strip("._") or "task"
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
            self.session_path = self.output_root / task / f"session_{stamp}_{uuid.uuid4().hex[:8]}"
            (self.session_path / "episodes").mkdir(parents=True)
            self._free_bytes = shutil.disk_usage(self.session_path).free
            self._space_checked_at = self.clock()
            if self._free_bytes < self.min_free_bytes:
                raise CollectionFault("Insufficient disk reserve")
            self._session_meta = {
                "format_version": 3, "status": "recording", "task": self.task,
                "session_id": self.session_path.name, "config": self.config_snapshot,
                "metadata": self.metadata, "robot_report_semantics": REPORT_SEMANTICS,
                "clock_semantics": {"host": "monotonic seconds, same process session",
                    "anchors": "bracketed monotonic ns / realtime ns readings",
                    "cross_device_mapping": "evidence only; not calibrated",
                    "fixed_offset_uncertainty_s": None},
                "visual_storage": self.visual_storage, "video_tools": self._video_tools,
            }
            atomic_json(self.session_path / "session.json", self._session_meta)
            self._opened = True
            self.session_writer = SessionWriter(self.session_path, self._session_fault, self.log_queue_size)
            self.session_writer.start()
            self.clocks = ClockSampler(lambda r: self.session_writer.submit("clock", r),
                                       self.publish_event, self.clock_interval_s, on_error=self._session_fault)
            self.clocks.start()
            self.thread = threading.Thread(target=self._supervise, name="collection-lifecycle", daemon=True)
            self.thread.start()

    def _session_fault(self, reason):
        with self._lock:
            self._fatal_error = str(reason)
            self._fault_locked(reason)

    def write(self, control_event):
        try:
            row = _json_safe(copy.deepcopy(control_event))
            if self.session_writer: self.session_writer.submit("control", row)
        except Exception as exc:
            self.mark_fault(f"Control log: {exc}")

    def publish_event(self, event):
        try:
            with self._lock:
                row = _json_safe(copy.deepcopy(event))
                row.setdefault("received_at", self.clock())
                episode = self._active
                row.setdefault("episode_id", episode.identifier if episode else None)
                name = row.get("event", "source_event")
                if episode and name not in ("episode_start", "episode_stop_requested", "capture_closed", "episode_finalized"):
                    episode.gaps[name] = episode.gaps.get(name, 0) + 1
                if self.session_writer: self.session_writer.submit("event", row)
                if row.get("boundary"):
                    self._boundary_locked(row.get("event", "source_boundary"), row["received_at"])
        except Exception as exc:
            self.mark_fault(f"Event log: {exc}")

    def _boundary_locked(self, reason, now):
        if self._active: self._finish_locked(reason, True)
        self._boundary_time = max(self._boundary_time, now)
        self._history.clear()
        self._latest.clear()

    def mark_boundary(self, reason="reset"):
        self.publish_event({"event": reason, "boundary": True, "received_at": self.clock()})

    def _remember_locked(self, key, value):
        now = value["received_at"]
        identity = (value.get("clock_epoch", 0), value["sequence"])
        previous = self._last_identity.get(key)
        if previous == identity:
            self.publish_event({"event": "duplicate_source", "source": key,
                                "sequence": value["sequence"], "received_at": now})
            return False
        if previous and identity[0] == previous[0] and identity[1] < previous[1]:
            self._boundary_locked(f"{key}_sequence_reset", now)
            self.publish_event({"event": "source_sequence_reset", "source": key, "received_at": now})
        self._last_identity[key] = identity
        if key in ROLES or value.get("valid") is True:
            self._latest[key] = value
        history = self._history.setdefault(key, deque())
        history.append(value)
        cutoff = max(self._boundary_time, now - self.pre_roll_s)
        while history and history[0]["received_at"] < cutoff:
            history.popleft()
        # A defective clock must not create an unbounded history.
        limit = self.state_queue_size if key not in ROLES else max(2, self.video_queue_size)
        while len(history) > limit: history.popleft()
        return True

    def publish_state(self, sample):
        try:
            packet = sample.get("packet")
            value = _json_safe(copy.deepcopy({k: v for k, v in sample.items() if k != "packet"}))
            kind = value.get("kind")
            if kind not in ("robot", "gripper", "report"):
                raise CollectionFault(f"Unsupported v3 state source: {kind}")
            if not _number(value.get("received_at")) or not isinstance(value.get("sequence"), int):
                raise CollectionFault("Invalid state source identity/time")
            if kind == "robot":
                if not isinstance(packet, bytes): raise CollectionFault("Native robot report bytes missing")
                value["packet"] = packet
            if kind == "gripper" and value.get("valid") and not _number(value.get("values", {}).get("position_pulse")):
                value.update(valid=False, error="Invalid actual gripper position")
            value.setdefault("clock_id", "host_monotonic")
            value.setdefault("clock_epoch", 0)
            value.setdefault("device_timestamp_raw", None)
            value.setdefault("device_timestamp_unit", None)
            with self._lock:
                if not self._opened or self._closed: return
                if not self._remember_locked(kind, value): return
                if not value.get("valid", False):
                    self.publish_event({"event": "invalid_state", "source": kind,
                                        "sequence": value["sequence"], "received_at": value["received_at"]})
                episode = self._active
                if self._admit_locked(episode, value["received_at"]):
                    if not episode.states.submit(value):
                        self._overflow_locked(kind, value)
        except Exception as exc:
            self.mark_fault(f"State publisher: {exc}")

    def publish_frame(self, frame):
        try:
            value = {k: (_json_safe(copy.deepcopy(v)) if k != "rgb" else v) for k, v in frame.items()}
            role, rgb = value.get("role"), value.get("rgb")
            spec = next((c for c in self.cameras if c["role"] == role), None)
            if spec is None or value.get("serial") != spec["serial"]:
                raise CollectionFault("Camera role/serial differs from capture configuration")
            if not _number(value.get("received_at")) or not _number(value.get("device_timestamp_ms")):
                raise CollectionFault("Invalid camera timestamps")
            if any(not isinstance(value.get(k), int) for k in ("sequence", "frame_id")):
                raise CollectionFault("Invalid camera sequence")
            if (getattr(rgb, "shape", None) != (spec["height"], spec["width"], 3)
                    or str(getattr(rgb, "dtype", "")) != "uint8"):
                raise CollectionFault("Camera RGB dimensions/type differ from configuration")
            value.setdefault("clock_id", f"camera:{value['serial']}")
            value.setdefault("clock_epoch", 0)
            with self._lock:
                if not self._opened or self._closed: return
                if not self._remember_locked(role, value): return
                episode = self._active
                if self._admit_locked(episode, value["received_at"]):
                    if not episode.videos[role].submit(value):
                        self._overflow_locked(role, value)
        except Exception as exc:
            self.mark_fault(f"Camera publisher: {exc}")

    def _admit_locked(self, episode, received):
        return (episode is not None and received >= episode.meta["capture_requested_start_time"]
                and (episode.meta["capture_requested_end_time"] is None or
                     received <= episode.meta["capture_requested_end_time"]))

    def _overflow_locked(self, source, value):
        self.publish_event({"event": "queue_overflow", "source": source,
                            "sequence": value["sequence"], "received_at": self.clock()})
        self._fault_locked(f"{source} recording queue full")

    def start_episode(self):
        with self._lock:
            if not self._opened or self._closed or self._fatal_error:
                self._error = self._fatal_error or "Recorder is not open"
                return None
            if self._state != "idle":
                self._error = "End and keep/discard the current episode before starting another"
                return None
            if self._free_bytes is None or self._free_bytes < self.min_free_bytes:
                self._error = "Insufficient disk reserve"
                return None
            now = self.clock()
            for key in (*ROLES, "robot", "gripper"):
                item = self._latest.get(key)
                if item is None or item["received_at"] < self._boundary_time or not 0 <= now - item["received_at"] <= self.source_timeout_s:
                    self._error = f"Waiting for live source: {key}"
                    return None
                if key not in ROLES and not item.get("valid"):
                    self._error = f"Waiting for valid source: {key}"
                    return None
            self._episode_number += 1
            identifier = f"episode_{self._episode_number:06d}"
            meta = {"format_version": 3, "episode_id": identifier, "task": self.task,
                    "status": "recording", "disposition": "pending", "start_time": now,
                    "end_time": None, "end_reason": None, "capture_start_time": None,
                    "capture_end_time": None,
                    "capture_requested_start_time": max(self._boundary_time, now - self.pre_roll_s),
                    "capture_requested_end_time": None, "pre_roll_s": self.pre_roll_s,
                    "post_roll_s": self.post_roll_s, "cameras": self.cameras,
                    "visual_storage": self.visual_storage, "video_tools": self._video_tools, "metadata": self.metadata,
                    "config": self.config_snapshot, "robot_report_semantics": REPORT_SEMANTICS,
                    "session_metadata": "../../session.json", "session_log": "../../control.jsonl",
                    "session_events": "../../events.jsonl", "clock_anchors": "../../clock_anchors.jsonl",
                    "clock_mapping_status": "evidence_only_not_calibrated",
                    "fixed_offset_uncertainty_s": None}
            episode = _Episode(identifier, self.session_path / "episodes" / identifier, meta)
            failed = lambda reason: self._writer_fault(episode, reason)
            episode.states = StateStreamWriter(episode, failed, self.state_queue_size)
            episode.videos = {c["role"]: VideoStreamWriter(episode, c["role"], c,
                self.visual_storage, failed, self.video_queue_size, self.video_factory) for c in self.cameras}
            self._active = self._last = episode
            self._state, self._error = "recording", None
            self._ops.put_nowait(("begin", episode))
            for key, history in self._history.items():
                for value in history:
                    if not self._admit_locked(episode, value["received_at"]): continue
                    writer = episode.videos[key] if key in ROLES else episode.states
                    if not writer.submit(value):
                        self._overflow_locked(key, value)
                        return None
            self.publish_event({"event": "episode_start", "received_at": now})
            return identifier

    def stop_episode(self, reason="user", incomplete=False):
        with self._lock:
            if not self._active: return False
            episode = self._active
            if incomplete:
                self._finish_locked(reason, True)
            elif episode.meta["end_time"] is None:
                now = self.clock()
                episode.meta.update(end_time=now, end_reason=str(reason),
                                    capture_requested_end_time=now + self.post_roll_s)
                self._state = "postroll"
                self.publish_event({"event": "episode_stop_requested", "received_at": now})
                if self.post_roll_s == 0: self._finish_locked(reason, False)
            return True

    def _finish_locked(self, reason, incomplete):
        episode = self._active
        if episode is None: return
        now = self.clock()
        episode.incomplete |= incomplete
        if episode.meta["end_time"] is None:
            episode.meta.update(end_time=now, end_reason=str(reason))
        if incomplete:
            episode.meta.update(end_reason=str(reason), capture_requested_end_time=now)
        if episode.meta["capture_requested_end_time"] is None:
            episode.meta["capture_requested_end_time"] = now
        episode.deadline = time.monotonic() + max(120., 2 * (now - episode.meta["start_time"]))
        self._active = None
        self._boundary_time = max(self._boundary_time, now)
        self._history.clear()
        self._state = "finalizing"
        self.publish_event({"event": "capture_closed", "episode_id": episode.identifier,
                            "received_at": now, "reason": str(reason), "incomplete": incomplete})
        if not episode.finish_queued:
            episode.finish_queued = True
            self._ops.put_nowait(("finish", episode))

    def _writer_fault(self, episode, reason):
        with self._lock:
            episode.incomplete = True
            episode.errors.append(str(reason))
            self._error = str(reason)
            if self._active is episode: self._finish_locked(reason, True)

    def _fault_locked(self, reason):
        self._error = str(reason)
        if self._active:
            self._active.errors.append(str(reason))
            self._finish_locked(reason, True)

    def mark_fault(self, reason):
        with self._lock:
            self.publish_event({"event": "source_fault", "reason": _json_safe(reason), "received_at": self.clock()})
            self._fault_locked(str(reason))
            self._boundary_time = self.clock()
            self._history.clear()
            self._latest.clear()

    def disposition(self, value):
        if value not in ("kept", "discarded"): raise ValueError("Expected kept/discarded")
        with self._lock:
            if self._state not in ("review", "fault") or self._last is None: return False
            self._state = "finalizing"
            self._ops.put_nowait(("disposition", (self._last, value)))
            return True

    def status(self):
        with self._lock:
            episode = self._last
            frames = {r: w.count for r, w in episode.videos.items()} if episode else dict.fromkeys(ROLES, 0)
            captured = {r: w.accepted for r, w in episode.videos.items()} if episode else dict.fromkeys(ROLES, 0)
            return {"state": self._state, "error": self._error, "fatal_error": self._fatal_error,
                    "episode_id": episode.identifier if episode else None,
                    "path": str(episode.path) if episode else None,
                    "session_path": str(self.session_path) if self.session_path else None,
                    "frame_counts": frames, "captured_frame_counts": captured,
                    "frame_count": sum(frames.values()),
                    "state_counts": dict(episode.states.counts) if episode else {},
                    "gaps": dict(episode.gaps) if episode else {},
                    "complete": bool(episode and episode.meta["status"] == "complete"),
                    "disposition": episode.meta["disposition"] if episode else None,
                    "free_bytes": self._free_bytes, "space_checked_at": self._space_checked_at,
                    "finalization_deadline": episode.deadline if episode else None,
                    "finalization_remaining_s": max(0., episode.deadline - time.monotonic()) if episode and episode.deadline else 30.}

    def _begin(self, episode):
        episode.path.mkdir(parents=True)
        (episode.path / "videos").mkdir()
        (episode.path / "frames").mkdir()
        atomic_json(episode.path / "meta.json", episode.meta)
        episode.states.thread.start()
        for writer in episode.videos.values(): writer.thread.start()

    def _finish(self, episode):
        writers = [episode.states, *episode.videos.values()]
        for writer in writers: writer.stop.set()
        deadline = time.monotonic() + 30.
        for writer in writers:
            if writer.thread.ident is not None:
                writer.thread.join(max(0., deadline - time.monotonic()))
        for writer in episode.videos.values():
            if writer.thread.is_alive(): writer.abort()
        for writer in writers:
            if writer.thread.is_alive(): writer.thread.join(6.)
            if writer.thread.is_alive():
                self._fatal_error = "Recording writer did not stop"
                episode.errors.append(self._fatal_error)
        errors = list(episode.errors) + [w.error for w in writers if w.error]
        frame_counts = {role: writer.count for role, writer in episode.videos.items()}
        if any(w.count != w.accepted for w in episode.videos.values()):
            errors.append("Accepted camera frames were not all written")
        if sum(episode.states.counts.values()) != episode.states.accepted:
            errors.append("Accepted state records were not all written")
        first = [w.first_time for w in episode.videos.values()] + list(episode.states.first_times.values())
        last = [w.last_time for w in episode.videos.values()] + list(episode.states.last_times.values())
        episode.meta.update(frame_counts=frame_counts, state_counts=dict(episode.states.counts),
            accepted_frame_counts={r: w.accepted for r, w in episode.videos.items()},
            accepted_state_count=episode.states.accepted,
            capture_start_time=min((v for v in first if v is not None), default=None),
            capture_end_time=max((v for v in last if v is not None), default=None),
            source_ranges={**{r: {"first": w.first_time, "last": w.last_time} for r, w in episode.videos.items()},
                           **{k: {"first": v, "last": episode.states.last_times[k]}
                              for k, v in episode.states.first_times.items()}},
            continuity={"events": dict(episode.gaps), "has_events": bool(episode.gaps)},
            queue_high_water={**{r: w.high_water for r, w in episode.videos.items()},
                              "states": episode.states.high_water},
            errors=errors, status="incomplete" if errors or episode.incomplete else "finalizing")
        runner = VideoRunner(episode.deadline)
        if not errors and not episode.incomplete:
            try:
                from .raw import validate_native_episode
                self.session_writer.flush()
                report = validate_native_episode(episode.path, meta=episode.meta, check_hashes=False,
                                                 allow_uncommitted=True, runner=runner)
                episode.meta["validation"] = report
                episode.meta["files"] = {str(p.relative_to(episode.path)):
                    {"bytes": p.stat().st_size, "sha256": file_sha256(p, runner)}
                    for p in sorted(episode.path.rglob("*")) if p.is_file() and p.name != "meta.json"}
                sync_directory(episode.path / "videos")
                sync_directory(episode.path / "frames")
                if self._fatal_error:
                    raise CollectionFault(self._fatal_error)
                episode.meta["status"] = "complete"
            except Exception as exc:
                episode.meta["status"] = "incomplete"
                episode.meta["errors"].append(str(exc))
        if episode.meta["status"] == "incomplete":
            episode.meta["surviving_video_checks"] = {
                role: inspect_surviving_video(episode.path / "videos" / f"{role}.mp4", writer.spec, runner)
                for role, writer in episode.videos.items() if not writer.thread.is_alive()}
            partial_files = {}
            if not any(writer.thread.is_alive() for writer in writers):
                try:
                    for path in sorted(episode.path.rglob("*")):
                        if path.is_file() and path.name != "meta.json":
                            partial_files[str(path.relative_to(episode.path))] = {
                                "bytes": path.stat().st_size, "sha256": file_sha256(path, runner)}
                except Exception as exc:
                    episode.meta["errors"].append(f"Partial file inventory: {exc}")
            episode.meta["files"] = partial_files
        atomic_json(episode.path / "meta.json", episode.meta)
        with self._lock:
            self._state = "review" if episode.meta["status"] == "complete" else "fault"
            if episode.meta["errors"]: self._error = "; ".join(episode.meta["errors"])
        self.publish_event({"event": "episode_finalized", "episode_id": episode.identifier,
                            "status": episode.meta["status"], "received_at": self.clock()})

    def _supervise(self):
        while not self._stop.is_set() or not self._ops.empty():
            try:
                now = self.clock()
                with self._lock:
                    episode = self._active
                    if episode:
                        if self._state == "postroll" and now >= episode.meta["capture_requested_end_time"]:
                            self._finish_locked(episode.meta["end_reason"], False)
                        else:
                            for key in (*ROLES, "robot", "gripper"):
                                value = self._latest.get(key)
                                if value is None or now - value["received_at"] > self.source_timeout_s:
                                    self._fault_locked(f"Source stopped: {key}")
                                    break
                if self._space_checked_at is None or now - self._space_checked_at >= .5:
                    free = shutil.disk_usage(self.session_path).free
                    with self._lock:
                        self._free_bytes, self._space_checked_at = free, now
                        if free < self.min_free_bytes: self._fault_locked("Disk reserve breached")
                try: kind, value = self._ops.get(timeout=.02)
                except queue.Empty: continue
                if kind == "begin": self._begin(value)
                elif kind == "finish": self._finish(value)
                elif kind == "disposition":
                    episode, choice = value
                    episode.meta["disposition"] = choice
                    atomic_json(episode.path / "meta.json", episode.meta)
                    with self._lock: self._state = "idle"
            except Exception as exc:
                with self._lock:
                    self._fatal_error = self._error = f"Collection lifecycle: {exc}"
                    if self._active: self._finish_locked(self._error, True)
                    else: self._state = "fault"

    def close(self):
        with self._lock:
            if not self._opened or self._closed: return
            self._closed = True
            if self._active:
                self._finish_locked("session_closed", True)
            self._stop.set()
        if self.thread:
            deadline = self._last.deadline if self._last and self._last.deadline else time.monotonic() + 30
            self.thread.join(max(0., deadline - time.monotonic()) + 7)
            if self.thread.is_alive(): self._fatal_error = "Collection lifecycle did not stop"
        try:
            self.clocks.close()
        except Exception as exc:
            self._session_fault(f"Final clock observation failed: {exc}")
        self.session_writer.close()
        self._session_meta.update(status="incomplete" if self._fatal_error else "closed",
                                  closed_at=self.clock(), error=self._fatal_error,
                                  files={name: {"sha256": file_sha256(self.session_path / name),
                                                "bytes": (self.session_path / name).stat().st_size}
                                         for name in ("control.jsonl", "events.jsonl", "clock_anchors.jsonl")
                                         if (self.session_path / name).is_file()})
        atomic_json(self.session_path / "session.json", self._session_meta)
