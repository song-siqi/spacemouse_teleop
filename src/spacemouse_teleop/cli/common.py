from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Optional

from spacemouse_teleop.spacemouse import TeleopConfig


class LoopRate:
    def __init__(self, hz: float) -> None:
        self.period = 1.0 / hz if hz > 0.0 else 0.0
        self._next_time: Optional[float] = None

    def sleep(self) -> None:
        if self.period <= 0.0:
            return

        now = time.monotonic()
        if self._next_time is None:
            self._next_time = now + self.period
            return

        sleep_s = self._next_time - now
        if sleep_s > 0.0:
            time.sleep(sleep_s)
            now = time.monotonic()

        while self._next_time <= now:
            self._next_time += self.period


def add_reader_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--backend",
        choices=("pyspacemouse", "mock"),
        default="pyspacemouse",
        help="Input backend to use.",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=60.0,
        help="Target SpaceMouse polling/command rate in Hz.",
    )
    parser.add_argument(
        "--device",
        default=None,
        help="pyspacemouse device name, for example SpaceMouseCompact.",
    )
    parser.add_argument(
        "--device-index",
        type=int,
        default=0,
        help="pyspacemouse device index when multiple HID entries match.",
    )
    parser.add_argument(
        "--axis-convention",
        choices=("ros", "hid_z_up", "hid", "legacy", "unity"),
        default="ros",
        help="pyspacemouse axis convention for real-device reads.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Seconds to run. Use 0 to run until Ctrl-C.",
    )
    parser.add_argument(
        "--print-rate",
        type=float,
        default=10.0,
        help="Maximum terminal print rate in Hz.",
    )


def load_config(path: Optional[str]) -> TeleopConfig:
    if not path:
        return TeleopConfig()
    with Path(path).open("r", encoding="utf-8") as config_file:
        data = json.load(config_file)
    return TeleopConfig.from_mapping(data)


def should_stop(start_time: float, duration: float) -> bool:
    return duration > 0 and time.monotonic() - start_time >= duration


def format_vec(values) -> str:
    return "[" + ", ".join(f"{value:+0.4f}" for value in values) + "]"
