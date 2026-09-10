from __future__ import annotations

from collections import deque
import importlib
import math
import threading
import time
from typing import Callable


REQUIRED_ROLES = ("wrist", "fixed_1", "fixed_2")


def _dependency(name: str):
    try:
        return importlib.import_module(name)
    except ImportError as exc:
        raise RuntimeError(f"Missing {name}; install spacemouse-teleop[collection].") from exc


def enumerate_devices(rs_module=None) -> list[dict]:
    """Read device identities without creating or starting a stream."""
    rs = rs_module if rs_module is not None else _dependency("pyrealsense2")
    result = []
    for device in rs.context().query_devices():
        def info(key):
            token = getattr(rs.camera_info, key, None)
            return device.get_info(token) if token is not None and device.supports(token) else None
        result.append({"serial": info("serial_number"), "model": info("name"),
                       "usb_type": info("usb_type_descriptor"), "firmware": info("firmware_version")})
    return sorted(result, key=lambda item: item["serial"] or "")


def _specs(cameras: list[dict], required_roles) -> list[dict]:
    if not cameras:
        raise ValueError("At least one RGB camera must be configured.")
    specs = []
    for camera in cameras:
        if not isinstance(camera, dict):
            raise ValueError("Each camera must be an object with role and serial.")
        role, serial = camera.get("role"), camera.get("serial")
        if not isinstance(role, str) or not role.strip():
            raise ValueError("Every camera needs an explicit role.")
        if not isinstance(serial, str) or not serial.strip():
            raise ValueError(f"Camera role {role!r} has no serial binding; use the camera preview to bind it.")
        spec = {"role": role, "serial": serial}
        for key, default in (("width", 640), ("height", 480), ("capture_fps", 30)):
            value = camera.get(key, default)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"Camera {role}: {key} must be a positive integer.")
            spec[key] = value
        timeout = camera.get("frame_timeout_s", 1.0)
        if isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError(f"Camera {role}: frame_timeout_s must be finite and positive.")
        spec["frame_timeout_s"] = float(timeout)
        specs.append(spec)
    roles, serials = [s["role"] for s in specs], [s["serial"] for s in specs]
    if len(set(roles)) != len(roles) or len(set(serials)) != len(serials):
        raise ValueError("Camera roles and serials must each be unique.")
    if required_roles and set(roles) != set(required_roles):
        raise ValueError(f"All three RGB views are required: {', '.join(required_roles)}.")
    return specs


def _percentiles(values):
    values = sorted(values)
    if not values:
        return {"median": None, "p95": None, "p99": None, "max": None}
    def at(p):
        index = (len(values) - 1) * p
        lo, hi = math.floor(index), math.ceil(index)
        return values[lo] + (values[hi] - values[lo]) * (index - lo)
    return {"median": at(.5), "p95": at(.95), "p99": at(.99), "max": values[-1]}


class CameraGroup:
    """One RGB capture worker per serial; callbacks must be quick and nonblocking.

    Arrays are owned, read-only RGB copies. Device times are never interpreted as
    host monotonic times. A repeated device frame is counted but not published as
    a fresh sample. Errors are sticky for diagnostics; other camera workers remain
    alive so the preview can show which source stopped.
    """

    def __init__(self, cameras: list[dict], on_frame: Callable[[dict], None],
                 on_error: Callable[[dict], None], *, required_roles=REQUIRED_ROLES,
                 history_size=120, rs_module=None, numpy_module=None):
        self.cameras = _specs(cameras, required_roles)
        if isinstance(history_size, bool) or not isinstance(history_size, int) or history_size < 1:
            raise ValueError("history_size must be a positive integer.")
        self.on_frame, self.on_error = on_frame, on_error
        self._rs, self._np = rs_module, numpy_module
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._threads = []
        self._started = False
        self._closed = False
        self._errors = []
        self._history = {c["role"]: deque(maxlen=history_size) for c in self.cameras}
        self._times = {c["role"]: deque(maxlen=4096) for c in self.cameras}
        self._counts = {c["role"]: {"frames": 0, "duplicate_frames": 0, "device_frame_gaps": 0}
                        for c in self.cameras}

    def start(self):
        if self._started or self._closed:
            raise RuntimeError("CameraGroup instances can only be started once.")
        self._rs = self._rs if self._rs is not None else _dependency("pyrealsense2")
        self._np = self._np if self._np is not None else _dependency("numpy")
        available = {d["serial"] for d in enumerate_devices(self._rs)}
        missing = [c["serial"] for c in self.cameras if c["serial"] not in available]
        if missing:
            raise RuntimeError(f"Configured RealSense serials are not connected: {', '.join(missing)}")
        self._started = True
        for spec in self.cameras:
            thread = threading.Thread(target=self._capture, args=(spec,),
                                      name=f"rgb-{spec['role']}", daemon=True)
            self._threads.append(thread)
            thread.start()
        return self

    def _fail(self, spec, reason):
        event = {"source": "camera", "role": spec["role"], "serial": spec["serial"],
                 "received_at": time.monotonic(), "reason": str(reason)}
        with self._lock:
            self._errors.append(event)
        try:
            self.on_error(dict(event))
        except Exception:
            # A recorder error must never escape into another subsystem.
            pass

    def _publish(self, frame):
        with self._lock:
            self._history[frame["role"]].append(dict(frame))
            self._times[frame["role"]].append(frame["received_at"])
            self._counts[frame["role"]]["frames"] += 1
        self.on_frame(dict(frame))

    def _capture(self, spec):
        pipeline, started = None, False
        try:
            pipeline = self._rs.pipeline()
            config = self._rs.config()
            config.enable_device(spec["serial"])
            config.enable_stream(self._rs.stream.color, spec["width"], spec["height"],
                                 self._rs.format.rgb8, spec["capture_fps"])
            pipeline.start(config)
            started = True
            last_received, last_id, sequence = time.monotonic(), None, 0
            while not self._stop.is_set():
                found, frames = pipeline.try_wait_for_frames(100)
                received_at = time.monotonic()
                if self._stop.is_set():
                    break
                if received_at - last_received > spec["frame_timeout_s"]:
                    raise RuntimeError(f"No fresh RGB frame for {received_at - last_received:.3f}s")
                if not found:
                    continue
                color = frames.get_color_frame()
                if not color:
                    continue
                frame_id = int(color.get_frame_number())
                if last_id is not None and frame_id == last_id:
                    with self._lock:
                        self._counts[spec["role"]]["duplicate_frames"] += 1
                    continue
                if last_id is not None and frame_id < last_id:
                    raise RuntimeError(f"Device frame sequence moved backwards: {last_id} -> {frame_id}")
                timestamp = float(color.get_timestamp())
                if not math.isfinite(timestamp):
                    raise RuntimeError("Camera device timestamp is not finite")
                rgb = self._np.asanyarray(color.get_data()).copy()
                if rgb.shape != (spec["height"], spec["width"], 3) or str(rgb.dtype) != "uint8":
                    raise RuntimeError(f"Unexpected RGB frame shape/dtype: {rgb.shape}/{rgb.dtype}")
                rgb.setflags(write=False)
                if last_id is not None:
                    with self._lock:
                        self._counts[spec["role"]]["device_frame_gaps"] += max(0, frame_id - last_id - 1)
                sequence += 1
                frame = {key: spec[key] for key in ("role", "serial", "width", "height", "capture_fps")}
                frame.update(frame_id=frame_id, sequence=sequence, received_at=received_at,
                             device_timestamp_ms=timestamp,
                             timestamp_domain=str(color.get_frame_timestamp_domain()), rgb=rgb)
                last_received, last_id = received_at, frame_id
                self._publish(frame)
        except Exception as exc:
            if not self._stop.is_set():
                self._fail(spec, exc)
        finally:
            if pipeline is not None and started:
                try:
                    pipeline.stop()
                except Exception as exc:
                    self._fail(spec, f"Stream shutdown failed: {exc}")

    def latest_frames(self) -> dict[str, dict]:
        with self._lock:
            return {role: dict(items[-1]) for role, items in self._history.items() if items}

    def history(self, role: str) -> tuple[dict, ...]:
        with self._lock:
            return tuple(dict(frame) for frame in self._history[role])

    def diagnostics(self) -> dict:
        now = time.monotonic()
        with self._lock:
            cameras = {}
            for spec in self.cameras:
                role, times = spec["role"], list(self._times[spec["role"]])
                intervals = [b - a for a, b in zip(times, times[1:])]
                cameras[role] = {**spec, **self._counts[role],
                                 "observed_fps": ((len(times) - 1) / (times[-1] - times[0]))
                                 if len(times) > 1 and times[-1] > times[0] else None,
                                 "interval_s": _percentiles(intervals),
                                 "latest_age_s": now - times[-1] if times else None,
                                 "statistics_window_frames": len(times)}
            return {"cameras": cameras, "errors": [dict(e) for e in self._errors],
                    "running_workers": [thread.name for thread in self._threads if thread.is_alive()]}

    def close(self, timeout_s: float = 2.0):
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("Shutdown timeout must be finite and positive.")
        self._stop.set()
        self._closed = True
        deadline = time.monotonic() + timeout_s
        for thread in self._threads:
            thread.join(max(0., deadline - time.monotonic()))
        alive = [thread.name for thread in self._threads if thread.is_alive()]
        if alive:
            raise RuntimeError(f"Camera workers did not stop within {timeout_s}s: {', '.join(alive)}")


class MockCameraGroup(CameraGroup):
    """Synthetic RGB frames for offline capture tests; never imports RealSense."""

    def start(self):
        if self._started or self._closed:
            raise RuntimeError("CameraGroup instances can only be started once.")
        self._np = self._np if self._np is not None else _dependency("numpy")
        self._started = True
        for index, spec in enumerate(self.cameras):
            thread = threading.Thread(target=self._synthetic, args=(spec, index),
                                      name=f"mock-rgb-{spec['role']}", daemon=True)
            self._threads.append(thread)
            thread.start()
        return self

    def _synthetic(self, spec, index):
        try:
            sequence, deadline = 0, time.monotonic()
            while not self._stop.is_set():
                sequence += 1
                rgb = self._np.zeros((spec["height"], spec["width"], 3), dtype="uint8")
                rgb[:, :, index % 3] = sequence % 256
                rgb.setflags(write=False)
                received_at = time.monotonic()
                frame = {key: spec[key] for key in ("role", "serial", "width", "height", "capture_fps")}
                frame.update(frame_id=sequence, sequence=sequence, received_at=received_at,
                             device_timestamp_ms=received_at * 1000,
                             timestamp_domain="mock_host_monotonic", rgb=rgb)
                self._publish(frame)
                deadline += 1. / spec["capture_fps"]
                if deadline < time.monotonic():
                    deadline = time.monotonic()
                self._stop.wait(max(0., deadline - time.monotonic()))
        except Exception as exc:
            self._fail(spec, exc)
