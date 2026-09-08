from __future__ import annotations

import threading
import time
from dataclasses import replace

from spacemouse_teleop.spacemouse.command import RawSpaceMouseState
from spacemouse_teleop.spacemouse.readers import make_reader
from .safety import TeleopFault


class LatestInput:
    """Drain HID reports independently and expose the current held buttons."""
    def __init__(self, config, backend=None, reader=None):
        self.config = config
        self.backend = backend or config.reader["backend"]
        r = config.reader
        self.reader = reader or make_reader(self.backend, hz=r["poll_hz"], device=r["device"],
                                           device_index=r["device_index"], axis_convention=r["axis_convention"])
        self.raw = None
        self.polled_at = 0.
        self.error = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.thread = None

    def start(self):
        self.thread = threading.Thread(target=self._run, name="spacemouse-input", daemon=True)
        self.thread.start()

    def _run(self):
        try:
            with self.reader:
                if self.backend == "pyspacemouse" and not hasattr(getattr(self.reader, "_pyspacemouse", None), "AxisConvention"):
                    raise TeleopFault("Real input requires explicit axis-convention support; install hardware extra")
                started = time.monotonic()
                while not self._stop.is_set():
                    for _ in range(64):
                        raw = self.reader.read()
                        now = time.monotonic()
                        if raw is None:
                            with self._lock:
                                self.polled_at = now
                            break
                        # Deterministic neutral bootstrap makes mock runs reproducible.
                        if self.backend == "mock" and now - started < .1:
                            raw = RawSpaceMouseState(0, 0, 0, 0, 0, 0, (0, 0), now, now, 0)
                        with self._lock:
                            previous = None if self.raw is None else self.raw.report_sequence
                            self.raw, self.polled_at = raw, now
                        if self.backend == "mock" or raw.report_sequence == previous:
                            break
                    else:
                        raise TeleopFault("HID report backlog exceeds bounded drain limit")
                    self._stop.wait(1 / self.config.reader["poll_hz"])
        except Exception as exc:
            self.error = exc

    def snapshot(self, now):
        if self.error:
            raise TeleopFault(f"Input failed: {self.error}")
        with self._lock:
            raw, polled_at = self.raw, self.polled_at
        if raw is None:
            return None, "hold"
        now = max(now, polled_at)  # Producer may publish after the caller sampled its clock.
        if not 0 <= now - polled_at <= self.config.robot["input_timeout_s"]:
            raise TeleopFault("SpaceMouse polling stopped")
        if not raw.connected:
            raise TeleopFault("SpaceMouse disconnected")
        t = self.config.teleop
        opening = t.gripper_open_button < len(raw.buttons) and bool(raw.buttons[t.gripper_open_button])
        closing = t.gripper_close_button < len(raw.buttons) and bool(raw.buttons[t.gripper_close_button])
        intent = ('open' if opening else 'close') if opening != closing else 'hold'
        return replace(raw, timestamp=now), intent

    def close(self):
        self._stop.set()
        if self.thread:
            self.thread.join(.5)


def axes_neutral(raw, config):
    return all(abs(v) < config.teleop.deadzone for v in (raw.x, raw.y, raw.z, raw.roll, raw.pitch, raw.yaw))


def is_neutral(raw, config):
    return axes_neutral(raw, config) and not any(raw.buttons)


def input_deadline(raw, now, config, intent="hold"):
    import math
    if not all(math.isfinite(v) for v in (raw.x, raw.y, raw.z, raw.roll, raw.pitch, raw.yaw)):
        raise TeleopFault("SpaceMouse returned nonfinite axes")
    if not raw.connected:
        raise TeleopFault("SpaceMouse disconnected")
    if raw.report_timestamp is None:
        return None  # Wait for a real report before enabling.
    age = now - raw.report_timestamp
    if not math.isfinite(age) or age < 0:
        raise TeleopFault("Invalid HID report timestamp")
    if axes_neutral(raw, config):
        # HID buttons report edges; a held button stays valid while polling stays
        # healthy. It need not generate repeated packets to continue gripper jog.
        return now + config.robot["command_duration_s"]
    ages = [age]
    axes = (raw.x, raw.y, raw.z, raw.roll, raw.pitch, raw.yaw)
    if not axes_neutral(raw, config):
        if raw.axis_report_timestamps is None or len(raw.axis_report_timestamps) != 6:
            raise TeleopFault("Per-axis HID report timestamps unavailable")
        for value, reported in zip(axes, raw.axis_report_timestamps):
            if abs(value) >= config.teleop.deadzone:
                if reported is None or not math.isfinite(reported) or reported > now:
                    raise TeleopFault("Invalid axis report timestamp")
                ages.append(now - reported)
    remaining = config.robot["input_timeout_s"] - max(ages)
    if remaining <= 0:
        raise TeleopFault("Nonzero SpaceMouse input expired")
    return now + min(remaining, config.robot["command_duration_s"])
