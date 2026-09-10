"""Independent, bounded raw collection; publishers never perform filesystem work.

Times in this module are host ``time.monotonic()`` seconds. Camera device clocks
are retained as provenance, never subtracted from the host clock. The recorder
does not own, enable, reset or stop a robot.
"""
from __future__ import annotations

import copy
import json
import math
import os
import queue
import re
import shutil
import threading
import time
import uuid
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .visual import storage_config, video_preflight, VIDEO_SETTINGS
from .archive import ArchiveJob, atomic_json


ROLES = ("wrist", "fixed_1", "fixed_2")
REQUIRED_STATES = ("joints", "tcp", "gripper")
DEFAULT_QUALITY = {
    "max_camera_age_s": 0.1,
    "max_robot_age_s": 0.1,
    "max_gripper_age_s": 0.15,
    "max_camera_skew_s": 0.05,
}


class CollectionFault(RuntimeError):
    """An episode cannot represent a complete continuous observation stream."""


def _finite(value):
    if isinstance(value, float):
        return math.isfinite(value)
    if isinstance(value, dict):
        return all(_finite(item) for item in value.values())
    if isinstance(value, (list, tuple)):
        return all(_finite(item) for item in value)
    return True


def _json_safe(value):
    """Keep malformed-input evidence JSON compatible without inventing zeros."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


def _number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _latest_before(history, timestamp):
    for value in reversed(history):
        if value["received_at"] <= timestamp:
            return value
    return None


def validate_state(sample):
    """Validate measured quantities rather than trusting a producer's valid flag."""
    kind = sample.get("kind")
    if kind not in (*REQUIRED_STATES, "report"):
        raise CollectionFault(f"unknown state kind: {kind}")
    if not isinstance(sample.get("sequence"), int) or sample["sequence"] < 0:
        raise CollectionFault(f"{kind}: invalid source sequence")
    if not _number(sample.get("received_at")):
        raise CollectionFault(f"{kind}: invalid receive time")
    if not sample.get("valid", False) or not _finite(sample):
        raise CollectionFault(f"{kind}: invalid/non-finite state: {sample.get('error', '')}")
    values = sample.get("values")
    if not isinstance(values, dict):
        raise CollectionFault(f"{kind}: missing values")
    vector = {"joints": "joints_rad", "tcp": "tcp_mm_rad"}.get(kind)
    if vector and (not isinstance(values.get(vector), (list, tuple))
                   or len(values[vector]) != 6 or not all(_number(v) for v in values[vector])):
        raise CollectionFault(f"{kind}: {vector} must contain six finite numbers")
    if kind == "gripper" and not _number(values.get("position_pulse")):
        raise CollectionFault("gripper: missing actual position_pulse")
    begin, end = sample.get("query_started_at"), sample.get("query_finished_at")
    if begin is not None or end is not None:
        if not (_number(begin) and _number(end) and begin <= end <= sample["received_at"]):
            raise CollectionFault(f"{kind}: invalid query time interval")


def align_observation(timestamp, start_time, index, states, frames, quality):
    """Select a causal fixed-grid observation without touching arrays or the disk.

    Returns the JSON row and the three selected RGB frame dictionaries. A stale
    source is an error; identical values with a new source sequence remain valid.
    """
    row = {"index": index, "timestamp": timestamp,
           "relative_time_s": timestamp - start_time, "images": {}, "sources": {}, "valid": True}
    chosen_frames = {}
    for kind in REQUIRED_STATES:
        sample = _latest_before(states.get(kind, ()), timestamp)
        if sample is None:
            raise CollectionFault(f"missing causal {kind} sample")
        validate_state(sample)
        age = timestamp - sample["received_at"]
        limit = quality["max_gripper_age_s" if kind == "gripper" else "max_robot_age_s"]
        if age > limit + 1e-9:
            raise CollectionFault(f"stale {kind}: age {age:.6f}s > {limit:.6f}s")
        source = {key: sample[key] for key in (
            "sequence", "received_at", "query_started_at", "query_finished_at") if key in sample}
        source["age_s"] = age
        row["sources"][kind] = source
        if kind == "gripper":
            row["gripper"] = copy.deepcopy(sample["values"])
        else:
            key = "joints_rad" if kind == "joints" else "tcp_mm_rad"
            row[key] = list(sample["values"][key])
    for role in ROLES:
        frame = _latest_before(frames.get(role, ()), timestamp)
        if frame is None:
            raise CollectionFault(f"missing causal camera {role}")
        age = timestamp - frame["received_at"]
        if age > quality["max_camera_age_s"] + 1e-9:
            raise CollectionFault(f"stale camera {role}: age {age:.6f}s")
        metadata = {key: value for key, value in frame.items() if key != "rgb"}
        metadata.update(file=f"images/{role}/{index:06d}.png", age_s=age)
        row["images"][role] = metadata
        chosen_frames[role] = frame
    times = [frame["received_at"] for frame in chosen_frames.values()]
    row["camera_skew_s"] = max(times) - min(times)
    if row["camera_skew_s"] > quality["max_camera_skew_s"] + 1e-9:
        raise CollectionFault(f"camera receive-time skew {row['camera_skew_s']:.6f}s")
    return row, chosen_frames


@dataclass
class _Episode:
    identifier: str
    path: Path
    meta: dict
    incomplete: bool = False
    raw_keys: set = field(default_factory=set)
    references: set = field(default_factory=set)
    image_files: list = field(default_factory=list)
    counts: dict = field(default_factory=dict)
    native_times: dict = field(default_factory=dict)
    source_sequences: dict = field(default_factory=dict)
    reused: dict = field(default_factory=dict)
    ages: dict = field(default_factory=dict)
    skews: list = field(default_factory=list)
    lags: list = field(default_factory=list)
    observations: Any = None
    raw: Any = None


class CollectionRecorder:
    """Session logs plus fixed-rate three-camera raw episodes.

    ``publish_*`` and ``write`` only copy small metadata and enqueue work. Camera
    producers must transfer owned immutable RGB arrays. Collection faults are
    exposed through ``status()`` and never raised into a robot control caller.
    Start/stop/disposition are asynchronous UI operations; watch ``state`` before
    starting another episode or resetting the arm.
    """
    def __init__(self, output_root, task, config_snapshot, sample_hz=20, quality=None,
                 min_free_bytes=10 * 1024**3, queue_size=128, metadata=None,
                 *, clock=time.monotonic, frame_history=64, state_history=4096, visual_storage=None):
        if not _number(sample_hz) or sample_hz <= 0:
            raise ValueError("sample_hz must be finite and positive")
        if queue_size < 4 or frame_history < 2 or state_history < 2:
            raise ValueError("queue_size >= 4 and histories >= 2 are required")
        self.quality = dict(DEFAULT_QUALITY if quality is None else quality)
        if set(self.quality) != set(DEFAULT_QUALITY) or not all(
                _number(value) and value > 0 for value in self.quality.values()):
            raise ValueError("quality must explicitly provide four positive age/skew limits")
        if min_free_bytes < 0:
            raise ValueError("min_free_bytes cannot be negative")
        self.output_root, self.task = Path(output_root), str(task)
        self.config_snapshot, self.metadata = copy.deepcopy(config_snapshot), copy.deepcopy(metadata or {})
        self.sample_hz, self.min_free_bytes, self.clock = float(sample_hz), int(min_free_bytes), clock
        self.queue = queue.Queue(queue_size)
        self._lock = threading.RLock()
        self._wake, self._stop = threading.Event(), threading.Event()
        self._states = {kind: deque(maxlen=state_history) for kind in (*REQUIRED_STATES, "report")}
        self._frames = {role: deque(maxlen=frame_history) for role in ROLES}
        self._state, self._error, self._fatal_error = "idle", None, None
        self._active, self._last = None, None
        self._pending_finish = None
        self._free_bytes, self._space_checked_at = None, None
        self._episode_number, self._next_index = 0, 0
        self._queue_high_water, self._dropped_events = 0, 0
        self._opened = False
        self.thread = self.scheduler_thread = None
        self._image_pool = None
        self.visual_storage = storage_config(visual_storage)
        self._video_tools = None
        self._archive_job = None
        self.session_path = None

    def preflight(self):
        if self.visual_storage['mode'] == 'video_after_episode' and self._video_tools is None:
            self._video_tools = video_preflight()

    def open(self):
        self.preflight()
        with self._lock:
            if self._opened:
                raise RuntimeError("recorder is already open")
            safe_task = re.sub(r"[^\w.-]+", "_", self.task).strip("._") or "task"
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
            self.session_path = self.output_root / safe_task / f"session_{stamp}_{uuid.uuid4().hex[:8]}"
            (self.session_path / "episodes").mkdir(parents=True, exist_ok=False)
            anchor = {"monotonic": self.clock(), "utc": datetime.now(timezone.utc).isoformat()}
            self._atomic_json(self.session_path / "session.json", {
                "format_version": 2 if self.visual_storage["mode"] == "video_after_episode" else 1,
                "visual_storage": self.visual_storage, "task": self.task, "clock_anchor": anchor,
                "sample_hz": self.sample_hz, "quality": self.quality,
                "config": self.config_snapshot, "metadata": self.metadata,
                "time_semantics": "host_monotonic_seconds; device timestamps have separate clock domains",
            })
            self._free_bytes = shutil.disk_usage(self.session_path).free
            self._space_checked_at = self.clock()
            self._opened = True
            self.thread = threading.Thread(target=self._writer, name="collection-writer", daemon=True)
            self.scheduler_thread = threading.Thread(target=self._scheduler, name="collection-grid", daemon=True)
            self.thread.start()
            self.scheduler_thread.start()

    def _enqueue_locked(self, event):
        # One slot belongs exclusively to an episode's finalization fence. All
        # accepted observations/states therefore precede its finish event.
        if self._fatal_error or self._pending_finish or self.queue.qsize() >= self.queue.maxsize - 1:
            self._dropped_events += 1
            self._fault_locked(self._fatal_error or "collection queue full")
            return False
        try:
            self.queue.put_nowait(event)
        except queue.Full:
            self._dropped_events += 1
            self._fault_locked("collection queue full")
            return False
        self._queue_high_water = max(self._queue_high_water, self.queue.qsize())
        return True

    def write(self, control_event):
        try:
            event = copy.deepcopy(control_event)
            if not _finite(event):
                raise CollectionFault("non-finite control log event")
            with self._lock:
                if self._opened and not self._stop.is_set():
                    self._enqueue_locked(("control", event))
        except Exception as exc:
            self.mark_fault(f"control log: {exc}")

    def publish_state(self, sample):
        try:
            value = copy.deepcopy(sample)
            error = None
            try:
                validate_state(value)
            except Exception as exc:
                error = str(exc)
                value = _json_safe(value)
                value.update(valid=False, error=error)
            kind = value.get("kind")
            with self._lock:
                if kind not in self._states or not _number(value.get("received_at")):
                    self._fault_locked(error or "malformed state source")
                    return
                history = self._states[kind]
                if history and value.get("sequence") == history[-1]["sequence"]:
                    return  # A cached sample must not acquire a new timestamp.
                if history and (not isinstance(value.get("sequence"), int)
                                or value["sequence"] < history[-1]["sequence"]
                                or value["received_at"] < history[-1]["received_at"]):
                    self._fault_locked(f"{kind}: source sequence/time went backwards")
                    return
                history.append(value)
                if self._active:
                    self._enqueue_locked(("raw", self._active, value))
                if error:
                    self._fault_locked(error)
        except Exception as exc:
            self.mark_fault(f"state publisher: {exc}")

    def publish_frame(self, frame):
        try:
            value = dict(frame)
            role, rgb = value.get("role"), value.get("rgb")
            required = ("serial", "frame_id", "sequence", "received_at", "device_timestamp_ms",
                        "timestamp_domain", "width", "height", "capture_fps")
            if role not in ROLES or any(key not in value for key in required):
                raise CollectionFault("camera frame metadata is incomplete")
            if not _number(value["received_at"]) or not _number(value["device_timestamp_ms"]):
                raise CollectionFault(f"{role}: invalid camera timestamp")
            if not isinstance(value["sequence"], int) or not isinstance(value["frame_id"], int):
                raise CollectionFault(f"{role}: invalid frame sequence")
            if getattr(rgb, "shape", None) != (value["height"], value["width"], 3) or str(getattr(rgb, "dtype", "")) != "uint8":
                raise CollectionFault(f"{role}: frame must be owned RGB uint8 HxWx3")
            with self._lock:
                history = self._frames[role]
                if history:
                    previous = history[-1]
                    if previous["serial"] != value["serial"]:
                        raise CollectionFault(f"{role}: camera serial changed during session")
                    if value["sequence"] == previous["sequence"] or value["frame_id"] == previous["frame_id"]:
                        return
                    if value["sequence"] < previous["sequence"] or value["received_at"] < previous["received_at"]:
                        raise CollectionFault(f"{role}: camera sequence/time went backwards")
                history.append(value)
        except Exception as exc:
            self.mark_fault(f"camera publisher: {exc}")

    def start_episode(self):
        """Return an episode id, or None and an explanatory status error."""
        try:
            with self._lock:
                if not self._opened or self._stop.is_set() or self._fatal_error:
                    self._error = self._fatal_error or "recorder is not open"
                    return None
                if self._state != "idle":
                    self._error = "finish and keep/discard the current episode before starting another"
                    return None
                # The coordinator calls this method in the motion loop. Disk
                # space is read only at open() and by the writer, never here.
                if self._free_bytes is None or self._free_bytes < self.min_free_bytes:
                    self._error = "insufficient disk space to begin recording"
                    return None
                now = self.clock()
                align_observation(now, now, 0, self._states, self._frames, self.quality)
                self._episode_number += 1
                identifier = f"episode_{self._episode_number:06d}"
                meta = {
                    "format_version": 2 if self.visual_storage["mode"] == "video_after_episode" else 1,
                    "episode_id": identifier, "task": self.task,
                    "sample_hz": self.sample_hz, "required_camera_roles": list(ROLES),
                    "status": "recording", "disposition": "pending", "start_time": now,
                    "end_time": None, "end_reason": None, "frame_count": 0,
                    "session_log": "../../control.jsonl", "session_metadata": "../../session.json",
                    "config": self.config_snapshot, "metadata": self.metadata, "quality": self.quality,
                    "units": {"joints_rad": "rad", "tcp_mm_rad": ["mm", "mm", "mm", "rad", "rad", "rad"],
                              "gripper.position_pulse": "SDK raw pulse", "timestamp": "host monotonic seconds"},
                    "tcp_frame": "robot base; TCP definition in captured real config", "stats": {},
                }
                if self.visual_storage['mode'] == 'video_after_episode':
                    meta.update(capture_complete=False, visual_storage={**VIDEO_SETTINGS,'fps':self.sample_hz},
                                archive={'phase':'recording','cleanup_pending':True})
                episode = _Episode(identifier, self.session_path / "episodes" / identifier, meta)
                self._active = self._last = episode
                self._state, self._error, self._next_index = "recording", None, 0
                predecessors = [copy.deepcopy(sample) for history in self._states.values()
                                if (sample := _latest_before(history, now)) is not None]
                if not self._enqueue_locked(("begin", episode, predecessors)):
                    return None
                self._wake.set()
                return identifier
        except Exception as exc:
            with self._lock:
                self._error = str(exc)
            return None

    def _end_locked(self, reason, incomplete):
        episode = self._active
        if episode is None:
            return False
        episode.incomplete |= bool(incomplete)
        episode.meta.update(end_time=self.clock(), end_reason=str(reason))
        self._active = None
        self._state = "finalizing"
        # Reserved slot normally guarantees this fence follows accepted events.
        # Keep one out-of-band fence if that invariant is ever violated: future
        # producers are then rejected until the queue drains, preserving order.
        try:
            self.queue.put_nowait(("finish", episode))
        except queue.Full:
            episode.incomplete = True
            episode.meta["end_reason"] = "collection queue full during finalization"
            self._error = episode.meta["end_reason"]
            self._pending_finish = episode
        self._wake.set()
        return True

    def stop_episode(self, reason="user", incomplete=False):
        try:
            with self._lock:
                return self._end_locked(reason, incomplete)
        except Exception as exc:
            self.mark_fault(f"episode finalization: {exc}")
            return False

    def _fault_locked(self, reason):
        self._error = str(reason)
        self._end_locked(reason, True)

    def mark_fault(self, reason):
        try:
            with self._lock:
                self._fault_locked(reason)
        except Exception:
            # This method is deliberately safe in SDK query/control callbacks.
            self._error = str(reason)

    def disposition(self, value):
        if value not in ("kept", "discarded"):
            raise ValueError("disposition must be kept or discarded")
        with self._lock:
            if self._state not in ("review", "fault") or self._last is None or self._fatal_error:
                return False
            if not self._enqueue_locked(("disposition", self._last, value)):
                return False
            self._state = "finalizing"
            return True

    def status(self):
        with self._lock:
            episode = self._last
            return {"state": self._state, "error": self._error, "fatal_error": self._fatal_error,
                    "episode_id": episode.identifier if episode else None,
                    "path": str(episode.path) if episode else None,
                    "session_path": str(self.session_path) if self.session_path else None,
                    "frame_count": episode.meta["frame_count"] if episode else 0,
                    "disposition": episode.meta["disposition"] if episode else None,
                    "complete": episode.meta["status"] == "complete" if episode else False,
                    "queue_size": self.queue.qsize(), "queue_high_water": self._queue_high_water,
                    "dropped_events": self._dropped_events, "free_bytes": self._free_bytes,
                    "space_checked_at": self._space_checked_at,
                    "archive": self._archive_job.snapshot() if self._archive_job and not self._archive_job.finished.is_set()
                               else copy.deepcopy(episode.meta.get('archive',{})) if episode else {},
                    "finalization_deadline": self._archive_job.runner.deadline if self._archive_job and not self._archive_job.finished.is_set() else None,
                    "finalization_remaining_s": self._archive_job.remaining if self._archive_job and not self._archive_job.finished.is_set()
                                                else 30.}

    def _scheduler(self):
        period = 1.0 / self.sample_hz
        while not self._stop.is_set():
            delay = min(period, 0.05)
            try:
                with self._lock:
                    episode = self._active
                    if episode is not None:
                        grid = episode.meta["start_time"] + self._next_index * period
                        now = self.clock()
                        if now < grid:
                            delay = min(grid - now, .05)
                        else:
                            delay = 0
                            # A late tick may use history, but never squeeze several
                            # missed grid points into a burst or relabel time.
                            if now - grid >= period:
                                raise CollectionFault(f"missed observation grid: {now - grid:.6f}s late")
                            row, frames = align_observation(grid, episode.meta["start_time"], self._next_index,
                                                            self._states, self._frames, self.quality)
                            row["scheduler_lag_s"] = now - grid
                            if self._enqueue_locked(("observation", episode, row, frames)):
                                self._next_index += 1
            except Exception as exc:
                self.mark_fault(str(exc))
            if delay:
                self._wake.wait(delay)
                self._wake.clear()

    _atomic_json = staticmethod(atomic_json)

    @staticmethod
    def _line(handle, value):
        handle.write(json.dumps(value, allow_nan=False, sort_keys=True) + "\n")

    def _refresh_disk_space(self):
        # Filesystem work belongs to the writer (or pre-motion open phase).
        available = shutil.disk_usage(self.session_path).free
        with self._lock:
            self._free_bytes, self._space_checked_at = available, self.clock()
            if available < self.min_free_bytes:
                self._fault_locked("disk free space fell below recording reserve")

    def _begin(self, episode, predecessors):
        episode.path.mkdir(parents=True, exist_ok=False)
        self._atomic_json(episode.path / "meta.json", episode.meta)
        episode.observations = (episode.path / "observations.jsonl").open("w", encoding="utf-8")
        episode.raw = (episode.path / "raw_states.jsonl").open("w", encoding="utf-8")
        for role in ROLES:
            (episode.path / ('_staging/images' if episode.meta['format_version'] == 2 else 'images') / role).mkdir(parents=True)
        for sample in predecessors:
            self._raw(episode, sample)
        self._refresh_disk_space()

    def _raw(self, episode, sample):
        key = sample["kind"], sample["sequence"]
        if key not in episode.raw_keys:
            self._line(episode.raw, sample)
            episode.raw_keys.add(key)
            episode.counts[sample["kind"]] = episode.counts.get(sample["kind"], 0) + 1
            episode.native_times.setdefault(sample["kind"], []).append(sample["received_at"])

    @staticmethod
    def _save_rgb(path, rgb):
        import cv2
        if not cv2.imwrite(str(path), rgb[:, :, ::-1], [cv2.IMWRITE_PNG_COMPRESSION, 1]):
            raise OSError(f"PNG encoder failed: {path}")

    def _observation(self, episode, row, frames):
        if row["index"] != episode.meta["frame_count"]:
            raise CollectionFault("observation index is not continuous")
        # Three fixed encoders share this one observation. The writer never
        # queues another observation until all three are done, bounding images
        # in flight and preserving image-before-index ordering.
        pending = []
        for role, frame in frames.items():
            relative = row["images"][role]["file"]
            if episode.meta['format_version'] == 2:
                relative = f'_staging/images/{role}/{row["index"]:06d}.png'
                row['images'][role].update(file=f'videos/{role}.mp4',video_frame_index=row['index'],
                                           video_pts_s=row['index']/self.sample_hz)
            pending.append((relative, self._image_pool.submit(self._save_rgb, episode.path / relative, frame["rgb"])))
        for relative, future in pending:
            future.result()
            episode.image_files.append(relative)
        # An index row only becomes visible after its images have been written.
        self._line(episode.observations, row)
        for kind, source in row["sources"].items():
            episode.references.add((kind, source["sequence"]))
            if episode.source_sequences.get(kind) == source["sequence"]:
                episode.reused[kind] = episode.reused.get(kind, 0) + 1
            episode.source_sequences[kind] = source["sequence"]
            episode.ages.setdefault(kind, []).append(source["age_s"])
        for role, source in row["images"].items():
            if episode.source_sequences.get(role) == source["sequence"]:
                episode.reused[role] = episode.reused.get(role, 0) + 1
            episode.source_sequences[role] = source["sequence"]
            episode.ages.setdefault(role, []).append(source["age_s"])
        episode.skews.append(row["camera_skew_s"])
        episode.lags.append(row["scheduler_lag_s"])
        episode.meta["frame_count"] += 1

    @staticmethod
    def _distribution(values):
        if not values:
            return {"count": 0}
        ordered = sorted(values)
        def percentile(fraction):
            return ordered[min(len(ordered) - 1, math.ceil(len(ordered) * fraction) - 1)]
        return {"count": len(values), "median": percentile(.5), "p95": percentile(.95),
                "p99": percentile(.99), "max": ordered[-1]}

    def _finish(self, episode):
        for handle in (episode.observations, episode.raw):
            if handle and not handle.closed:
                handle.flush()
                os.fsync(handle.fileno())
                handle.close()
        failures = []
        if not episode.meta["frame_count"]:
            failures.append("episode has no observations")
        if episode.references - episode.raw_keys:
            failures.append("observation references missing native state samples")
        if any(not (episode.path / relative).is_file() for relative in episode.image_files):
            failures.append("observation references missing images")
        if failures:
            episode.incomplete = True
            episode.meta["integrity_errors"] = failures
        episode.meta["status"] = "incomplete" if episode.incomplete else "complete"
        count = episode.meta["frame_count"]
        episode.meta["stats"] = {
            "raw_sample_count": episode.counts,
            "native_source_timing": {
                kind: {"response_hz": (len(times) - 1) / (times[-1] - times[0])
                       if len(times) > 1 and times[-1] > times[0] else None,
                       "interval_s": self._distribution([b - a for a, b in zip(times, times[1:])]),
                       "device_update_hz": None,
                       "note": "Host response rate does not prove internal device measurement rate"}
                for kind, times in episode.native_times.items()
            },
            "sample_hz": self.sample_hz,
            "saved_grid_span_s": max(0, count - 1) / self.sample_hz,
            "source_age_s": {key: self._distribution(values) for key, values in episode.ages.items()},
            "camera_skew_s": self._distribution(episode.skews),
            "scheduler_lag_s": self._distribution(episode.lags),
            "reused_source_count": episode.reused,
            "reused_source_fraction": {key: episode.reused.get(key, 0) / max(1, count - 1)
                                       for key in (*REQUIRED_STATES, *ROLES)},
            "queue_high_water_session": self._queue_high_water,
            "dropped_events_session": self._dropped_events,
        }
        if episode.meta['format_version'] == 2 and not episode.incomplete:
            episode.meta.update(status='finalizing',capture_complete=False)
            episode.meta['archive']['phase'] = 'checking_capture'
            self._atomic_json(episode.path / 'meta.json', episode.meta)
            def completed(meta, error):
                with self._lock:
                    if meta is not None: episode.meta = meta
                    if error:
                        self._error = f'video archive: {error}'
                        self._state = 'fault'
                    else:
                        self._error = episode.meta.get('archive',{}).get('cleanup_warning')
                        self._state = 'fault' if self._fatal_error else 'review'
                self._wake.set()
            job = ArchiveJob(episode.path,episode.meta,completed,self._video_tools)
            with self._lock: self._archive_job = job
            job.start()
        else:
            self._atomic_json(episode.path / "meta.json", episode.meta)
            with self._lock:
                self._state = "fault" if episode.incomplete else "review"

    def _writer(self):
        control = None
        current = None
        try:
            self._image_pool = ThreadPoolExecutor(max_workers=3, thread_name_prefix="collection-png")
            control = (self.session_path / "control.jsonl").open("w", encoding="utf-8")
            last_flush = self.clock()
            while not self._stop.is_set() or not self.queue.empty() or self._pending_finish:
                now = self.clock()
                if now - self._space_checked_at >= 1:
                    self._refresh_disk_space()
                dequeued = True
                try:
                    event = self.queue.get(timeout=.05)
                except queue.Empty:
                    with self._lock:
                        pending, self._pending_finish = self._pending_finish, None
                    if pending is None:
                        continue
                    event, dequeued = ("finish", pending), False
                try:
                    kind = event[0]
                    if kind == "control":
                        self._line(control, event[1])
                    elif kind == "begin":
                        current = event[1]
                        self._begin(current, event[2])
                    elif kind == "raw":
                        self._raw(event[1], event[2])
                    elif kind == "observation":
                        self._observation(event[1], event[2], event[3])
                    elif kind == "finish":
                        control.flush()
                        self._finish(event[1])
                        current = None
                    elif kind == "disposition":
                        event[1].meta["disposition"] = event[2]
                        self._atomic_json(event[1].path / "meta.json", event[1].meta)
                        with self._lock:
                            self._state = "idle"
                    now = self.clock()
                    if now - last_flush >= 1:
                        control.flush()
                        if current:
                            current.raw.flush()
                            current.observations.flush()
                        last_flush = now
                finally:
                    if dequeued:
                        self.queue.task_done()
        except Exception as exc:
            with self._lock:
                self._fatal_error = self._error = f"collection writer failed: {exc}"
                if self._active:
                    self._active.incomplete = True
                    self._active.meta.update(end_time=self.clock(), end_reason=self._fatal_error)
                    current = self._active
                    self._active = None
                self._state = "fault"
            if current:
                current.incomplete = True
                current.meta.update(status="incomplete", end_reason=self._fatal_error)
                try:
                    self._finish(current)
                except Exception:
                    # Initial meta remains 'recording' if a full/unavailable disk
                    # also prevents this update. Such an episode is not complete.
                    pass
        finally:
            if self._image_pool:
                self._image_pool.shutdown(wait=True, cancel_futures=True)
            for handle in (control, current.raw if current else None, current.observations if current else None):
                if handle and not handle.closed:
                    try:
                        handle.close()
                    except Exception:
                        pass
            if self._fatal_error:
                self._pending_finish = None
                while True:
                    try:
                        self.queue.get_nowait()
                        self.queue.task_done()
                    except queue.Empty:
                        break

    def close(self):
        """Call after stopping motion; flushing cannot delay robot shutdown."""
        with self._lock:
            if not self._opened or self._stop.is_set():
                return
            self._end_locked("session_closed", True)
            self._stop.set()
            self._wake.set()
        if self.scheduler_thread:
            self.scheduler_thread.join(2)
        if self.thread:
            self.thread.join(30)
            if self.thread.is_alive():
                with self._lock:
                    self._fatal_error = self._error = "collection writer did not finish within 30 seconds"
                    self._state = "fault"
        job = self._archive_job
        if job and not job.finished.is_set():
            if self._fatal_error: job.cancel()
            job.thread.join(job.remaining + 4)
            if job.thread.is_alive():
                job.cancel()
                job.thread.join(4)
                with self._lock:
                    self._fatal_error = self._error = 'video archive did not stop after its deadline'
                    self._state = 'fault'


