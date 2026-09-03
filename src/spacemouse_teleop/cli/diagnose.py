from __future__ import annotations

import argparse
import sys
import time
from typing import Iterable, Optional

from spacemouse_teleop.spacemouse.readers import (
    _preload_hidapi_on_macos,
    _resolve_axis_convention,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Diagnose SpaceMouse HID access.")
    parser.add_argument(
        "--device",
        default=None,
        help="pyspacemouse device name to open, for example SpaceMouseCompact.",
    )
    parser.add_argument(
        "--max-index",
        type=int,
        default=3,
        help="Try device indices in [0, max-index).",
    )
    parser.add_argument(
        "--axis-convention",
        choices=("ros", "hid_z_up", "hid", "legacy", "unity"),
        default="ros",
        help="pyspacemouse axis convention to use during open/read.",
    )
    parser.add_argument(
        "--skip-open",
        action="store_true",
        help="Only list devices; do not try to open/read.",
    )
    return parser


def main() -> None:
    try:
        _main()
    except KeyboardInterrupt:
        print("\nStopped.")
    except RuntimeError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc


def _main() -> None:
    args = build_parser().parse_args()
    pyspacemouse = _import_pyspacemouse()

    print(f"pyspacemouse: {getattr(pyspacemouse, '__version__', 'unknown')}")
    print(f"pyspacemouse file: {getattr(pyspacemouse, '__file__', 'unknown')}")

    _print_pyspacemouse_devices(pyspacemouse)
    _print_hidapi_devices()

    if args.skip_open:
        return

    targets = [args.device] if args.device else _unique(pyspacemouse.get_connected_devices())
    if not targets:
        print("No pyspacemouse-supported devices found.")
        return

    convention = _resolve_axis_convention(pyspacemouse, args.axis_convention)
    for device in targets:
        for index in range(max(1, args.max_index)):
            _try_open(pyspacemouse, device, index, convention)


def _import_pyspacemouse():
    _preload_hidapi_on_macos()
    try:
        import pyspacemouse
    except ImportError as exc:
        raise RuntimeError(
            "pyspacemouse is not installed. Run: uv pip install pyspacemouse"
        ) from exc
    return pyspacemouse


def _print_pyspacemouse_devices(pyspacemouse) -> None:
    try:
        print(f"connected: {pyspacemouse.get_connected_devices()}")
        print(f"connected by path: {pyspacemouse.get_connected_devices_by_path()}")
    except RuntimeError as exc:
        print(f"connected: failed: {exc}")

    try:
        supported = pyspacemouse.get_supported_devices()
        print(f"supported count: {len(supported)}")
    except RuntimeError as exc:
        print(f"supported: failed: {exc}")


def _print_hidapi_devices() -> None:
    try:
        import hid
    except ImportError:
        print("hidapi python binding: not installed, skipping low-level list")
        return

    print("3Dconnexion HID entries:")
    found = False
    for dev in hid.enumerate():
        if dev.get("vendor_id") != 0x256F:
            continue
        found = True
        path = dev.get("path")
        if isinstance(path, bytes):
            path = path.decode("utf-8", errors="replace")
        print(
            "  "
            f"vid=0x{dev.get('vendor_id'):04x} "
            f"pid=0x{dev.get('product_id'):04x} "
            f"usage_page={dev.get('usage_page')} "
            f"usage={dev.get('usage')} "
            f"interface={dev.get('interface_number')} "
            f"product={dev.get('product_string')} "
            f"path={path}"
        )
    if not found:
        print("  none")


def _try_open(pyspacemouse, device: str, index: int, convention) -> None:
    print(f"try open: device={device} index={index}")
    opened = None
    try:
        opened = pyspacemouse.open(
            device=device,
            device_index=index,
            axis_convention=convention,
        )
        state = opened.read()
        time.sleep(0.02)
        state = opened.read()
        print(
            "  success: "
            f"x={state.x:+0.4f} y={state.y:+0.4f} z={state.z:+0.4f} "
            f"roll={state.roll:+0.4f} pitch={state.pitch:+0.4f} "
            f"yaw={state.yaw:+0.4f} buttons={tuple(state.buttons)}"
        )
    except Exception as exc:
        print(f"  failed: {type(exc).__name__}: {exc}")
    finally:
        if opened is not None and hasattr(opened, "close"):
            opened.close()


def _unique(values: Iterable[str]) -> list[str]:
    result = []
    for value in values:
        if value not in result:
            result.append(value)
    return result


if __name__ == "__main__":
    main()
