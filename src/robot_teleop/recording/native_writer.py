"""Independent bounded writers for raw v3. No motion interfaces."""
from __future__ import annotations
import json
import os
from pathlib import Path
import queue
import selectors
import subprocess
import tempfile
import threading
import time

from .visual import VideoError

NATIVE_VIDEO_SETTINGS = {
    "mode": "native_video", "container": "fragmented_mp4", "encoder": "libx264",
    "crf": 18, "preset": "veryfast", "pix_fmt": "yuv420p", "threads_per_camera": 2,
    "b_frames": 0, "gop": 30,
    "pts_semantics": "ordinal frame / configured fps; source times live in frame index",
}

def line(handle, value):
    handle.write(json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(",", ":")) + "\n")

def sync_close(handle):
    if handle and not handle.closed:
        handle.flush()
        os.fsync(handle.fileno())
        handle.close()

class SessionWriter:
    def __init__(self, path, on_error, size=4096):
        self.path, self.on_error = Path(path), on_error
        self.queue = queue.Queue(size)
        self.stop = threading.Event()
        self.error = None
        self.high_water = 0
        self.thread = threading.Thread(target=self._run, name="collection-session-log", daemon=True)

    def start(self):
        self.thread.start()

    def submit(self, kind, value):
        if self.error or self.stop.is_set():
            return False
        try:
            self.queue.put_nowait((kind, value))
            self.high_water = max(self.high_water, self.queue.qsize())
            return True
        except queue.Full:
            self.error = "Session log queue full"
            self.on_error(self.error)
            return False

    def _run(self):
        handles = {}
        try:
            handles = {kind: (self.path / filename).open("w", encoding="utf-8")
                       for kind, filename in (("control", "control.jsonl"), ("event", "events.jsonl"),
                                               ("clock", "clock_anchors.jsonl"))}
            flushed = time.monotonic()
            while not self.stop.is_set() or not self.queue.empty():
                try:
                    kind, row = self.queue.get(timeout=.05)
                except queue.Empty:
                    kind = None
                if kind == "flush":
                    for handle in handles.values():
                        handle.flush()
                        os.fsync(handle.fileno())
                    row.set()
                elif kind:
                    line(handles[kind], row)
                if time.monotonic() - flushed >= .5:
                    for handle in handles.values(): handle.flush()
                    flushed = time.monotonic()
        except Exception as exc:
            self.error = f"Session log failed: {exc}"
            self.on_error(self.error)
        finally:
            for handle in handles.values():
                try: sync_close(handle)
                except Exception as exc:
                    self.error = f"Session log close failed: {exc}"
                    self.on_error(self.error)

    def flush(self):
        fence = threading.Event()
        if not self.submit("flush", fence) or not fence.wait(10):
            raise RuntimeError(self.error or "Session flush did not complete")

    def close(self):
        self.stop.set()
        self.thread.join(10)
        if self.thread.is_alive():
            self.error = "Session log did not drain"
            self.on_error(self.error)

class StreamingVideo:
    def __init__(self, path, spec, settings, executable="ffmpeg"):
        self.path, self.spec = Path(path), spec
        self.cancelled = threading.Event()
        self.errors = tempfile.TemporaryFile()
        args = [executable, "-nostdin", "-v", "error", "-y", "-f", "rawvideo",
                "-pixel_format", "rgb24", "-video_size", f"{spec['width']}x{spec['height']}",
                "-framerate", str(spec["fps"]), "-i", "pipe:0", "-an",
                "-c:v", "libx264", "-crf", str(settings["crf"]), "-preset", settings["preset"],
                "-pix_fmt", "yuv420p", "-threads", str(settings["threads_per_camera"]),
                "-bf", "0", "-g", str(settings["gop"]), "-sc_threshold", "0",
                "-vsync", "0", "-movflags", "+frag_keyframe+empty_moov+default_base_moof",
                "-flush_packets", "1", "-f", "mp4", str(path)]
        try:
            self.process = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                                            stderr=self.errors, bufsize=0)
        except Exception:
            self.errors.close()
            raise
        os.set_blocking(self.process.stdin.fileno(), False)

    def write(self, rgb):
        data = memoryview(rgb.tobytes())
        deadline = time.monotonic() + 5.
        with selectors.DefaultSelector() as selector:
            selector.register(self.process.stdin, selectors.EVENT_WRITE)
            while data:
                if self.cancelled.is_set() or time.monotonic() >= deadline:
                    raise VideoError("Video encoder input deadline exceeded")
                if self.process.poll() is not None:
                    raise VideoError("Video encoder exited while recording")
                for _, _ in selector.select(.05):
                    try: written = os.write(self.process.stdin.fileno(), data[:262144])
                    except BlockingIOError: continue
                    if written <= 0: raise VideoError("Video encoder pipe closed")
                    data = data[written:]

    def finish(self):
        self.process.stdin.close()
        try:
            code = self.process.wait(timeout=10)
            if code:
                self.errors.seek(0)
                raise VideoError(self.errors.read(4096).decode(errors="replace") or f"Encoder exit {code}")
            with self.path.open("rb") as handle: os.fsync(handle.fileno())
        finally:
            if self.process.poll() is None: self.abort()
            self.errors.close()

    def abort(self):
        self.cancelled.set()
        if self.process.poll() is None:
            self.process.terminate()
            try: self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)
        if self.process.stdin and not self.process.stdin.closed:
            self.process.stdin.close()
        self.errors.close()

class VideoStreamWriter:
    def __init__(self, episode, role, spec, settings, on_error, capacity, video_factory=StreamingVideo):
        self.episode, self.role, self.spec = episode, role, spec
        self.settings, self.on_error, self.video_factory = settings, on_error, video_factory
        self.queue = queue.Queue(capacity)
        self.stop, self.cancel = threading.Event(), threading.Event()
        self.count = self.accepted = self.high_water = 0
        self.first_time = self.last_time = None
        self.error = None
        self.encoder = None
        self.thread = threading.Thread(target=self._run, name=f"native-video-{role}", daemon=True)

    def submit(self, frame):
        if self.error or self.stop.is_set(): return False
        try:
            self.queue.put_nowait(frame)
            self.accepted += 1
            self.high_water = max(self.high_water, self.queue.qsize())
            return True
        except queue.Full:
            return False

    def _run(self):
        index = None
        try:
            index = (self.episode.path / "frames" / f"{self.role}.jsonl").open("w", encoding="utf-8")
            self.encoder = self.video_factory(self.episode.path / "videos" / f"{self.role}.mp4",
                                              self.spec, self.settings)
            flushed = time.monotonic()
            while not self.cancel.is_set() and (not self.stop.is_set() or not self.queue.empty()):
                try: frame = self.queue.get(timeout=.05)
                except queue.Empty: continue
                self.encoder.write(frame["rgb"])
                row = {k: v for k, v in frame.items() if k != "rgb"}
                row.update(file=f"videos/{self.role}.mp4", video_frame_index=self.count,
                           video_pts_s=self.count / self.spec["fps"])
                line(index, row)
                self.count += 1
                self.first_time = frame["received_at"] if self.first_time is None else self.first_time
                self.last_time = frame["received_at"]
                if time.monotonic() - flushed >= .5:
                    index.flush()
                    flushed = time.monotonic()
            if self.cancel.is_set():
                raise VideoError("Video writer cancelled")
            self.encoder.finish()
        except Exception as exc:
            detail = ""
            stream = getattr(self.encoder, "errors", None)
            if stream is not None and not stream.closed:
                stream.seek(0)
                detail = stream.read(4096).decode(errors="replace").strip()
            self.error = f"{self.role} video writer: {exc}" + (f"; {detail}" if detail else "")
            self.on_error(self.error)
            if self.encoder:
                try: self.encoder.abort()
                except Exception: pass
        finally:
            try: sync_close(index)
            except Exception as exc:
                self.error = f"{self.role} index close: {exc}"
                self.on_error(self.error)

    def abort(self):
        self.cancel.set()
        if self.encoder:
            self.encoder.cancelled.set()

class StateStreamWriter:
    def __init__(self, episode, on_error, capacity):
        self.episode, self.on_error = episode, on_error
        self.queue = queue.Queue(capacity)
        self.stop = threading.Event()
        self.counts = {"robot": 0, "gripper": 0, "auxiliary": 0}
        self.accepted = self.high_water = 0
        self.first_times, self.last_times = {}, {}
        self.error = None
        self.thread = threading.Thread(target=self._run, name="native-state-writer", daemon=True)

    def submit(self, sample):
        if self.error or self.stop.is_set(): return False
        try:
            self.queue.put_nowait(sample)
            self.accepted += 1
            self.high_water = max(self.high_water, self.queue.qsize())
            return True
        except queue.Full: return False

    def _run(self):
        handles = {}
        binary = None
        try:
            handles = {kind: (self.episode.path / filename).open("w", encoding="utf-8")
                       for kind, filename in (("robot", "robot_states.jsonl"),
                                               ("gripper", "gripper_states.jsonl"),
                                               ("auxiliary", "auxiliary_states.jsonl"))}
            binary = (self.episode.path / "robot_reports.bin").open("wb")
            flushed = time.monotonic()
            while not self.stop.is_set() or not self.queue.empty():
                try: sample = self.queue.get(timeout=.05)
                except queue.Empty: sample = None
                if sample is not None:
                    row = {k: v for k, v in sample.items() if k != "packet"}
                    kind = row["kind"] if row["kind"] in ("robot", "gripper") else "auxiliary"
                    if kind == "robot":
                        packet = sample["packet"]
                        row.update(packet_file="robot_reports.bin", packet_offset=binary.tell(),
                                   packet_bytes=len(packet))
                        binary.write(packet)
                    line(handles[kind], row)
                    self.counts[kind] += 1
                    self.first_times.setdefault(kind, row["received_at"])
                    self.last_times[kind] = row["received_at"]
                if time.monotonic() - flushed >= .5:
                    for handle in (*handles.values(), binary): handle.flush()
                    flushed = time.monotonic()
        except Exception as exc:
            self.error = f"Native state writer: {exc}"
            self.on_error(self.error)
        finally:
            for handle in (*handles.values(), binary):
                try: sync_close(handle)
                except Exception as exc:
                    self.error = f"Native state close: {exc}"
                    self.on_error(self.error)


def inspect_surviving_video(path, spec, runner):
    """Best-effort decode evidence for an incomplete file; never repairs indexes."""
    from fractions import Fraction
    count, time_base, last_pts, pts_match = 0, None, None, True
    def decoded(line):
        nonlocal count, time_base, last_pts, pts_match
        if line.startswith("#tb 0:"):
            time_base = Fraction(line.split(":", 1)[1].strip())
        elif line and not line.startswith("#"):
            fields = [part.strip() for part in line.split(",")]
            if len(fields) != 6 or time_base is None:
                raise VideoError("Incomplete decoded frame record")
            pts = float(int(fields[2]) * time_base)
            pts_match &= abs(pts - count / spec["fps"]) <= 1e-6
            last_pts = pts
            count += 1
    error = None
    try:
        runner.run(["ffmpeg", "-nostdin", "-v", "error", "-xerror", "-threads", "2",
                    "-copyts", "-i", str(path), "-map", "0:v:0", "-an", "-vsync", "0",
                    "-pix_fmt", "rgb24", "-threads", "2", "-f", "framemd5", "pipe:1"], decoded)
    except Exception as exc:
        error = str(exc)
    return {"decoded_prefix_frames": count, "last_decoded_pts_s": last_pts,
            "ordinal_pts_match": pts_match, "decode_error": error,
            "meaning": "surviving decode evidence only; capture remains incomplete"}
