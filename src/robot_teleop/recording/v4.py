"""Source-native raw v4. No robot protocol, resampling, or training actions."""
from __future__ import annotations
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import queue
import re
import threading
import time

from .archive import atomic_json
from .native_writer import StreamingVideo, NATIVE_VIDEO_SETTINGS, line, sync_close
from .visual import file_sha256, validate_video


@dataclass(frozen=True)
class Source:
    id: str
    type: str
    schema: str
    units: dict
    frame: str
    clock: str
    required: bool = True
    video: dict | None = None
    binary: bool = False

    def __post_init__(self):
        if not re.fullmatch(r"[A-Za-z0-9_-]+", self.id):
            raise ValueError("Source ID must be a safe filename component")
        if not self.schema or not self.clock:
            raise ValueError("Source schema and clock are required")
        if self.video:
            for key in ("width", "height", "fps"):
                if not math.isfinite(self.video[key]) or self.video[key] <= 0:
                    raise ValueError("Invalid video dimensions/rate")


class RawEpisode:
    """One bounded queue and independent writer per source; fault evidence survives."""
    def __init__(self, path, sources, *, clock="simulation", epoch=0, capacity=256,
                 metadata=None, video_factory=StreamingVideo, visual_storage=None,
                 validation_profile=None):
        capacities = {s.id: capacity[s.id] if isinstance(capacity, dict) else capacity for s in sources}
        if any(isinstance(v, bool) or not isinstance(v, int) or v < 1 for v in capacities.values()):
            raise ValueError("Queue capacities must be positive integers")
        self.path = Path(path)
        self.path.mkdir(parents=True, exist_ok=False)
        self.sources = {s.id: s for s in sources}
        if len(self.sources) != len(sources):
            raise ValueError("Duplicate source IDs")
        self.queues = {s.id: queue.Queue(capacities[s.id]) for s in sources}
        self.clock, self.epoch = clock, epoch
        self.errors, self.counts, self.checksums = [], {}, {}
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.closed = False
        self.video_factory = video_factory
        self.visual_storage = {**NATIVE_VIDEO_SETTINGS, **(visual_storage or {})}
        self.encoders = {}
        self.accepted = dict.fromkeys(self.sources, 0)
        self.high_water = dict.fromkeys(self.sources, 0)
        self.ranges = {}
        self.fatal_error = None
        self.meta = {"format_version": 4, "status": "recording", "capture_complete": False,
                     "disposition": "pending", "session_clock": clock, "epoch": epoch,
                     "metadata": metadata or {}, "sources": [], "errors": [],
                     "validation_profile": validation_profile,
                     "visual_storage": self.visual_storage, "queue_capacities": capacities}
        for s in sources:
            entry = asdict(s)
            entry["path"] = f"{s.id}.jsonl"
            if s.video:
                entry["video_path"] = f"{s.id}.mp4"
                entry["pts_semantics"] = "ordinal/fps; source timestamps are in the index"
            if s.binary:
                entry["binary_path"] = f"{s.id}.bin"
            self.meta["sources"].append(entry)
        atomic_json(self.path / "meta.json", self.meta)
        self.threads = [threading.Thread(target=self._write, args=(s,), name=f"raw4-{s.id}") for s in sources]
        for thread in self.threads:
            thread.start()

    def fault(self, error):
        with self.lock:
            if str(error) not in self.errors:
                self.errors.append(str(error))

    def publish(self, source_id, sequence, source_time, values, *, session_time,
                epoch=None, source_epoch=0, received_at=None, valid=True, rgb=None, packet=None):
        if self.closed or self.stop_event.is_set():
            return False
        s = self.sources[source_id]
        epoch = self.epoch if epoch is None else epoch
        if epoch != self.epoch:
            self.fault("Session reset crossed episode boundary")
            return False
        if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0 or not all(math.isfinite(v) for v in (source_time, session_time)):
            self.fault(f"Invalid sample header: {source_id}")
            return False
        row = {"sequence": sequence, "source_timestamp": source_time, "clock_id": s.clock,
               "clock_epoch": source_epoch, "session_time": session_time, "session_epoch": epoch,
               "host_received_at": time.monotonic() if received_at is None else received_at,
               "valid": bool(valid), "values": values}
        try:
            # Own JSON and image buffers before returning to the producer.
            row = json.loads(json.dumps(row, allow_nan=False))
            item = (row, None if rgb is None else rgb.copy(), None if packet is None else bytes(packet))
            with self.lock:
                if self.stop_event.is_set():
                    return False
                self.queues[source_id].put_nowait(item)
                self.accepted[source_id] += 1
                self.high_water[source_id] = max(self.high_water[source_id], self.queues[source_id].qsize())
            return True
        except (queue.Full, ValueError, TypeError) as exc:
            self.fault(f"Publish failed for {source_id}: {type(exc).__name__}")
            return False

    def _write(self, source):
        index = binary = encoder = None
        count, previous, previous_time = 0, None, None
        try:
            index = (self.path / f"{source.id}.jsonl").open("w")
            if source.binary:
                binary = (self.path / f"{source.id}.bin").open("wb")
            if source.video:
                encoder = self.video_factory(self.path / f"{source.id}.mp4", source.video, self.visual_storage)
                self.encoders[source.id] = encoder
            q = self.queues[source.id]
            while not self.stop_event.is_set() or not q.empty():
                try:
                    row, rgb, packet = q.get(timeout=.05)
                except queue.Empty:
                    continue
                identity = (row["clock_epoch"], row["sequence"])
                if previous is not None and identity[0] == previous[0] and identity[1] != previous[1] + 1:
                    self.fault(f"Noncontiguous sequence in {source.id}: {previous} -> {identity}")
                if previous is not None and identity[0] == previous[0] and row["source_timestamp"] < previous_time:
                    self.fault(f"Unmarked source clock reset in {source.id}")
                previous = identity
                previous_time = row["source_timestamp"]
                if encoder:
                    if rgb is None or rgb.shape != (source.video["height"], source.video["width"], 3) or str(rgb.dtype) != "uint8":
                        raise ValueError("RGB frame must match declared uint8 shape")
                    encoder.write(rgb)
                    row.update(video_frame_index=count, video_pts_s=count/source.video["fps"])
                if binary:
                    if packet is None:
                        raise ValueError("Declared binary source requires original bytes")
                    row.update(offset=binary.tell(), length=len(packet), packet_sha256=hashlib.sha256(packet).hexdigest())
                    binary.write(packet)
                line(index, row)
                count += 1
                self.counts[source.id] = count
                if source.id not in self.ranges:
                    self.ranges[source.id] = {"first": row["session_time"], "last": row["session_time"]}
                self.ranges[source.id]["last"] = row["session_time"]
            if encoder:
                encoder.finish()
                encoder = None
        except Exception as exc:
            self.fault(f"Writer {source.id}: {exc}")
        finally:
            if encoder:
                encoder.abort()
            for handle in (index, binary):
                try:
                    sync_close(handle)
                except OSError as exc:
                    self.fault(f"Close {source.id}: {exc}")
            self.counts[source.id] = count

    def close(self, *, complete=True, disposition="kept", deadline=None):
        if self.closed:
            return self.meta
        if disposition not in ("kept", "discarded", "pending"):
            raise ValueError("Invalid disposition")
        deadline = deadline if deadline is not None else time.monotonic() + 120.
        with self.lock:
            self.stop_event.set()
        for thread in self.threads:
            thread.join(max(0., deadline - time.monotonic()))
        if any(t.is_alive() for t in self.threads):
            for encoder in list(self.encoders.values()):
                try:
                    encoder.abort()
                except Exception as exc:
                    self.fault(f"Encoder abort: {exc}")
            for thread in self.threads:
                thread.join(2.)
            self.fault("Writer drain timed out")
        if any(t.is_alive() for t in self.threads):
            self.fatal_error = "Recording writer did not stop"
            self.fault(self.fatal_error)
        for source in self.sources.values():
            if source.required and not self.counts.get(source.id):
                self.fault(f"Required source empty: {source.id}")
            if self.accepted[source.id] != self.counts.get(source.id, 0):
                self.fault(f"Accepted samples were not all written: {source.id}")
        starts = [r["first"] for r in self.ranges.values()]
        ends = [r["last"] for r in self.ranges.values()]
        self.meta.update(status="finalizing", capture_complete=False,
                         disposition=disposition, errors=list(self.errors), counts=dict(self.counts),
                         accepted_counts=dict(self.accepted), queue_high_water=dict(self.high_water),
                         source_ranges=dict(self.ranges),
                         capture_start_time=min(starts, default=None),
                         capture_end_time=max(ends, default=None))
        try:
            if not self.fatal_error:
                self.meta["checksums"] = {p.name: file_sha256(p) for p in self.path.iterdir() if p.name != "meta.json"}
            atomic_json(self.path / "meta.json", self.meta)
            if complete and not self.errors:
                from .validation import validate_before_commit
                report = validate_before_commit(self.path, deadline)
                self.meta["validation"] = report
                if not report["valid"]:
                    self.fault("Validation: " + "; ".join(report["errors"]))
        except Exception as exc:
            self.fault(f"Finalization: {exc}")
        self.meta.update(status="complete" if complete and not self.errors else "incomplete",
                         capture_complete=bool(complete and not self.errors), errors=list(self.errors))
        atomic_json(self.path / "meta.json", self.meta)
        self.closed = True
        return self.meta


def check_episode(path, require_kept=False, *, _allow_uncommitted=False, _runner=None):
    root = Path(path).resolve()
    errors, meta = [], {}
    try:
        meta = json.loads((root / "meta.json").read_text())
        if meta["format_version"] != 4:
            raise ValueError("Expected raw v4")
        if _allow_uncommitted:
            if meta["status"] != "finalizing" or meta.get("errors"):
                raise ValueError("Only fault-free finalizing episodes can be validated for commit")
        elif meta["status"] != "complete" or not meta["capture_complete"]:
            raise ValueError("Episode is not a complete raw v4 capture")
        if require_kept and meta["disposition"] != "kept":
            raise ValueError("Episode was not kept")
        if 'start_time' in meta:
            names = ('start_time','end_time','capture_requested_start_time','capture_requested_end_time')
            if any(isinstance(meta.get(k),bool) or not isinstance(meta.get(k),(int,float))
                   or not math.isfinite(meta[k]) for k in names):
                raise ValueError('Invalid operation/capture boundaries')
            if not meta['capture_requested_start_time'] <= meta['start_time'] <= meta['end_time'] <= meta['capture_requested_end_time']:
                raise ValueError('Operation interval differs from capture window')
        def safe(name):
            p = (root / name).resolve()
            if not p.is_relative_to(root) or not p.is_file():
                raise ValueError(f"Invalid source path: {name}")
            if name not in meta["checksums"] or file_sha256(p) != meta["checksums"][name]:
                raise ValueError(f"Checksum mismatch: {name}")
            return p
        for source in meta["sources"]:
            rows = [json.loads(s) for s in safe(source["path"]).read_text().splitlines()]
            if (len(rows) != meta["counts"][source["id"]] or (source["required"] and not rows)
                    or meta.get("accepted_counts", {}).get(source["id"], len(rows)) != len(rows)):
                raise ValueError("Sample count mismatch")
            previous = previous_time = previous_session_time = None
            binary = safe(source["binary_path"]).read_bytes() if source["binary"] else None
            offset = 0
            for i, row in enumerate(rows):
                if row["session_epoch"] != meta["epoch"] or row["clock_id"] != source["clock"]:
                    raise ValueError("Clock identity mismatch")
                if not all(math.isfinite(row[k]) for k in ("source_timestamp", "session_time", "host_received_at")):
                    raise ValueError("Nonfinite timestamp")
                start, end = meta.get("capture_requested_start_time"), meta.get("capture_requested_end_time")
                if start is not None and row["session_time"] < start or end is not None and row["session_time"] > end:
                    raise ValueError("Sample outside capture window")
                if previous_session_time is not None and row['session_time'] < previous_session_time:
                    raise ValueError('Session time moved backwards within source')
                previous_session_time = row['session_time']
                identity = (row["clock_epoch"], row["sequence"])
                if previous and identity[0] == previous[0] and identity[1] != previous[1] + 1:
                    raise ValueError("Sequence gap or duplicate")
                if previous and identity[0] == previous[0] and row["source_timestamp"] < previous_time:
                    raise ValueError("Unmarked source clock reset")
                previous = identity
                previous_time = row["source_timestamp"]
                if binary is not None:
                    if row["offset"] != offset or row["length"] < 0:
                        raise ValueError("Binary offset mismatch")
                    data = binary[offset:offset+row["length"]]
                    if len(data) != row["length"] or hashlib.sha256(data).hexdigest() != row["packet_sha256"]:
                        raise ValueError("Binary packet mismatch")
                    offset += row["length"]
                if source["video"] and (row["video_frame_index"] != i or abs(row["video_pts_s"]-i/source["video"]["fps"]) > 1e-8):
                    raise ValueError("Video index mismatch")
            if binary is not None and offset != len(binary):
                raise ValueError("Unindexed binary bytes")
            if source["video"]:
                v = source["video"]
                validate_video(safe(source["video_path"]), len(rows), v["fps"], v["width"], v["height"], runner=_runner)
        if meta.get("validation_profile"):
            from .validation import validate_profile
            validate_profile(root, meta)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        errors.append(str(exc))
    warnings = ["Cross-device clock mapping is not calibrated"] if isinstance(meta, dict) and meta.get("validation_profile") == "xarm" else []
    return {"valid": not errors, "errors": errors, "warnings": warnings, "format_version": 4}
