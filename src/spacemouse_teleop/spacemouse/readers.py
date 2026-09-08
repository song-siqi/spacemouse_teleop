from __future__ import annotations

import ctypes
import math
import os
import platform
import time
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Optional

from spacemouse_teleop.spacemouse.command import RawSpaceMouseState


class SpaceMouseReader(ABC):
    @abstractmethod
    def open(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def read(self) -> Optional[RawSpaceMouseState]:
        raise NotImplementedError

    def close(self) -> None:
        return None

    def __enter__(self) -> "SpaceMouseReader":
        self.open()
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()


class _HIDReportMonitor:
    """Track each axis report channel without changing PySpaceMouse parsing."""
    def __init__(self, device, mappings, timestamps):
        self.device, self.mappings, self.timestamps = device, mappings, timestamps

    def __getattr__(self, name):
        return getattr(self.device, name)

    def read(self, *args, **kwargs):
        report = self.device.read(*args, **kwargs)
        if report:
            now = time.monotonic()
            for index, name in enumerate(("x", "y", "z", "roll", "pitch", "yaw")):
                if self.mappings[name].channel == report[0]:
                    self.timestamps[index] = now
        return report


class PySpaceMouseReader(SpaceMouseReader):
    """Reader backed by the optional pyspacemouse package."""

    def __init__(
        self,
        device: Optional[str] = None,
        device_index: int = 0,
        axis_convention: Optional[str] = "ros",
    ) -> None:
        self.device = device
        self.device_index = int(device_index)
        self.axis_convention = axis_convention
        self._pyspacemouse = None
        self._device = None
        self._report_sequence = 0
        self._last_report_timestamp = None
        self._axis_report_timestamps = None

    def open(self) -> None:
        _preload_hidapi_on_macos()
        try:
            import pyspacemouse
        except ImportError as exc:
            raise RuntimeError(
                "pyspacemouse is not installed. Run: uv pip install pyspacemouse"
            ) from exc

        self._pyspacemouse = pyspacemouse
        open_kwargs = {
            "device": self.device,
            "device_index": self.device_index,
        }
        convention = _resolve_axis_convention(pyspacemouse, self.axis_convention)
        if convention is not None:
            open_kwargs["axis_convention"] = convention

        try:
            opened = pyspacemouse.open(**open_kwargs)
        except TypeError as exc:
            if "axis_convention" not in str(exc):
                raise
            open_kwargs.pop("axis_convention", None)
            opened = pyspacemouse.open(**open_kwargs)
        except RuntimeError as exc:
            message = str(exc)
            if "HID API" in message or "hid_enumerate" in message:
                raise RuntimeError(_hidapi_install_hint(message)) from exc
            raise RuntimeError(_device_open_hint(message)) from exc
        if not opened:
            raise RuntimeError("pyspacemouse.open() failed; check SpaceMouse connection.")
        if hasattr(opened, "read"):
            self._device = opened
            # Version-pinned 2.1 API: separate translation/rotation/button reports.
            # A button packet must not renew a cached nonzero translation.
            hid_device = getattr(opened, "_device", None)
            info = getattr(opened, "_info", None)
            if hid_device is not None and info is not None:
                self._axis_report_timestamps = [None] * 6
                opened._device = _HIDReportMonitor(hid_device, info.mappings, self._axis_report_timestamps)

    def read(self) -> Optional[RawSpaceMouseState]:
        if self._pyspacemouse is None:
            raise RuntimeError("reader is not open")
        read_source = self._device if self._device is not None else self._pyspacemouse
        if getattr(read_source, "connected", True) is False:
            raise RuntimeError("SpaceMouse disconnected")
        try:
            state = read_source.read()
        except Exception as exc:
            raise RuntimeError(f"SpaceMouse read failed: {exc}") from exc
        if state is None:
            return None

        # PySpaceMouse 2.1 updates t only when a real HID report is processed.
        # It uses timeit.default_timer, the same monotonic clock as Python here.
        report_timestamp = getattr(state, "t", None)
        if report_timestamp is not None:
            report_timestamp = float(report_timestamp)
            if report_timestamp <= 0:
                report_timestamp = None
            elif report_timestamp != self._last_report_timestamp:
                self._report_sequence += 1
                self._last_report_timestamp = report_timestamp
        return RawSpaceMouseState(
            report_timestamp=report_timestamp,
            axis_report_timestamps=None if self._axis_report_timestamps is None else tuple(self._axis_report_timestamps),
            report_sequence=self._report_sequence if report_timestamp is not None else None,
            x=float(getattr(state, "x", 0.0)),
            y=float(getattr(state, "y", 0.0)),
            z=float(getattr(state, "z", 0.0)),
            roll=float(getattr(state, "roll", 0.0)),
            pitch=float(getattr(state, "pitch", 0.0)),
            yaw=float(getattr(state, "yaw", 0.0)),
            buttons=tuple(int(v) for v in getattr(state, "buttons", ())),
            timestamp=time.monotonic(),
        )

    def close(self) -> None:
        if self._device is not None and hasattr(self._device, "close"):
            self._device.close()
        elif self._pyspacemouse is not None and hasattr(self._pyspacemouse, "close"):
            self._pyspacemouse.close()
        self._device = None
        self._pyspacemouse = None


class MockSpaceMouseReader(SpaceMouseReader):
    """Deterministic reader used to verify the pipeline without hardware."""

    def __init__(self, hz: float = 60.0) -> None:
        self.hz = float(hz)
        self._start = 0.0
        self._last = 0.0

    def open(self) -> None:
        self._start = time.monotonic()
        self._last = self._start

    def read(self) -> Optional[RawSpaceMouseState]:
        now = time.monotonic()
        period = 1.0 / self.hz
        sleep_s = period - (now - self._last)
        if sleep_s > 0:
            time.sleep(sleep_s)
            now = time.monotonic()
        self._last = now

        t = now - self._start
        phase = 2.0 * math.pi * 0.25 * t
        return RawSpaceMouseState(
            x=math.sin(phase),
            y=0.5 * math.sin(phase + math.pi / 2.0),
            z=0.25 * math.sin(phase + math.pi),
            roll=0.3 * math.sin(phase * 0.5),
            pitch=0.2 * math.sin(phase * 0.5 + math.pi / 3.0),
            yaw=0.4 * math.sin(phase * 0.5 + math.pi / 5.0),
            buttons=(0, 0),
            timestamp=now,
            report_timestamp=now,
            axis_report_timestamps=(now,) * 6,
            report_sequence=round(t * self.hz),
        )


def make_reader(
    backend: str,
    hz: float = 60.0,
    device: Optional[str] = None,
    device_index: int = 0,
    axis_convention: Optional[str] = "ros",
) -> SpaceMouseReader:
    if backend == "mock":
        return MockSpaceMouseReader(hz=hz)
    if backend == "pyspacemouse":
        return PySpaceMouseReader(
            device=device,
            device_index=device_index,
            axis_convention=axis_convention,
        )
    raise ValueError(f"unknown SpaceMouse backend: {backend}")


def _preload_hidapi_on_macos() -> None:
    if platform.system() != "Darwin":
        return

    for path in _candidate_hidapi_paths():
        if not path.exists():
            continue
        ctypes.CDLL(str(path), mode=getattr(ctypes, "RTLD_GLOBAL", 0))
        return


def _candidate_hidapi_paths():
    env_path = os.environ.get("SPACEMOUSE_HIDAPI_DYLIB")
    if env_path:
        yield Path(env_path)

    for prefix in ("/opt/homebrew", "/usr/local"):
        yield Path(prefix) / "opt" / "hidapi" / "lib" / "libhidapi.dylib"
        yield Path(prefix) / "lib" / "libhidapi.dylib"


def _hidapi_install_hint(original_message: str) -> str:
    if platform.system() == "Linux":
        return (f"{original_message}\nInstall the native HIDAPI runtime:\n"
                "  sudo apt-get install libhidapi-hidraw0 libhidapi-libusb0\n"
                "Then install this project's hardware extra in the venv.")
    return (
        f"{original_message}\n\n"
        "On macOS, pyspacemouse/easyhid also needs the native hidapi library.\n"
        "Install it with:\n"
        "  brew install hidapi\n\n"
        "Then retry:\n"
        "  python scripts/spacemouse_probe.py --backend pyspacemouse --print-rate 10\n\n"
        "If hidapi is installed in a custom location, set:\n"
        "  export SPACEMOUSE_HIDAPI_DYLIB=/path/to/libhidapi.dylib"
    )


def _device_open_hint(original_message: str) -> str:
    return (
        f"{original_message}\n\n"
        "The SpaceMouse was detected, but this process could not open it.\n"
        "If you are running inside an app sandbox, retry from a normal Terminal.\n"
        "If Terminal also fails, unplug/replug the SpaceMouse and check whether a "
        "3Dconnexion driver or helper process is exclusively grabbing the device."
    )


def _resolve_axis_convention(pyspacemouse, value: Optional[str]):
    if value is None:
        return None
    enum = getattr(pyspacemouse, "AxisConvention", None)
    if enum is None:
        return None
    key = value.upper().replace("-", "_")
    try:
        return enum[key]
    except KeyError as exc:
        valid = ", ".join(item.value for item in enum)
        raise RuntimeError(
            f"unknown axis convention '{value}'. Valid values: {valid}"
        ) from exc
