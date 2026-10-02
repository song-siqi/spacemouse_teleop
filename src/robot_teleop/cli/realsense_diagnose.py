"""Camera-only enumeration and RGB diagnostics. This module never imports xArm."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import importlib
import json
import math
import os
from pathlib import Path
import sys
import time

from robot_teleop.sensors.realsense import CameraGroup, MockCameraGroup, REQUIRED_ROLES, enumerate_devices


def _parser():
    parser = argparse.ArgumentParser(description="Enumerate RealSense devices; RGB streams open only with --preview or --duration.")
    parser.add_argument("--config", help="Collection JSON containing an explicit cameras list.")
    parser.add_argument("--camera", action="append", default=[], metavar="ROLE=SERIAL",
                        help="Explicit binding; repeat for each view. Cannot be combined with --config.")
    parser.add_argument("--preview", action="store_true", help="Open RGB preview; Q/Esc or window close ends it.")
    parser.add_argument("--duration", type=float, help="Capture for this many seconds, without enabling robot motion.")
    parser.add_argument("--output", type=Path, help="Save diagnostic JSON and one final PNG per view to this directory.")
    parser.add_argument("--mock", action="store_true", help="Use three synthetic RGB sources without opening cameras.")
    return parser


def _bindings(args, devices):
    if args.config and args.camera:
        raise ValueError("Use --config or --camera, not both.")
    if args.config:
        document = json.loads(Path(args.config).read_text())
        cameras = document.get("cameras")
        if not isinstance(cameras, list):
            raise ValueError("Collection config must contain a cameras list.")
        return cameras, True
    if args.camera:
        cameras = []
        for item in args.camera:
            role, separator, serial = item.partition("=")
            if not separator:
                raise ValueError("Expected --camera ROLE=SERIAL.")
            cameras.append({"role": role, "serial": serial})
        return cameras, True
    if args.mock:
        return [{"role": role, "serial": f"mock-{role}"} for role in REQUIRED_ROLES], True
    # Enumeration order is only a preview layout. It never assigns physical roles.
    return [{"role": f"unbound:{item['serial']}", "serial": item["serial"]} for item in devices], False


def _show(frames, title):
    cv2, np = importlib.import_module("cv2"), importlib.import_module("numpy")
    panels = []
    for role, frame in sorted(frames.items()):
        panel = cv2.resize(frame["rgb"][:, :, ::-1], (480, 360))
        cv2.putText(panel, f"{role} / {frame['serial']}", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1)
        cv2.putText(panel, f"frame {frame['frame_id']}  age {time.monotonic()-frame['received_at']:.3f}s",
                    (8, 43), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1)
        panels.append(panel)
    if panels:
        cv2.imshow(title, np.concatenate(panels, axis=1))
    key = cv2.waitKey(1) & 0xff
    return key not in (ord("q"), 27) and cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) >= 1


def _save(output, result, frames):
    output.mkdir(parents=True, exist_ok=True)
    if frames:
        cv2 = importlib.import_module("cv2")
        for index, (role, frame) in enumerate(sorted(frames.items())):
            # A user-defined role must not become an unchecked filesystem path.
            filename = f"view_{index}_{frame['sequence']:08d}.png"
            if not cv2.imwrite(str(output / filename), frame["rgb"][:, :, ::-1]):
                raise RuntimeError(f"Could not write PNG for {role}.")
            result.setdefault("snapshots", {})[role] = filename
    (output / "diagnostic.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")


def main(argv=None):
    parser = _parser()
    args = parser.parse_args(argv)
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
        parser.error("--duration must be finite and positive.")
    if args.output and not (args.preview or args.duration is not None):
        parser.error("--output requires --preview or --duration.")
    group, preview_open = None, False
    result = {"read_only": True, "robot_connected": False,
              "observed_at_utc": datetime.now(timezone.utc).isoformat(), "mock": args.mock}
    exit_code = 0
    try:
        devices = ([{"serial": f"mock-{role}", "model": "synthetic", "usb_type": None}
                    for role in REQUIRED_ROLES] if args.mock else enumerate_devices())
        cameras, roles_bound = _bindings(args, devices)
        result.update(devices=devices, cameras=cameras, roles_explicitly_bound=roles_bound)
        if args.preview or args.duration is not None:
            group_type = MockCameraGroup if args.mock else CameraGroup
            group = group_type(cameras, lambda frame: None, lambda error: None,
                               required_roles=REQUIRED_ROLES if roles_bound else ())
            title = "RealSense RGB — bind by serial; Q to close"
            if args.preview:
                if sys.platform.startswith("linux") and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
                    raise RuntimeError("RGB preview needs a desktop display; use --duration for headless diagnostics.")
                cv2 = importlib.import_module("cv2")
                cv2.namedWindow(title, cv2.WINDOW_NORMAL)
                preview_open = True
            group.start()
            started = time.monotonic()
            while args.duration is None or time.monotonic() - started < args.duration:
                frames = group.latest_frames()
                if group.diagnostics()["errors"]:
                    exit_code = 1
                    break
                if args.preview and not _show(frames, title):
                    break
                time.sleep(.02)
            measured_at = time.monotonic()
            diagnostics = group.diagnostics()
            frames = group.latest_frames()
            result.update(duration_s=measured_at - started, measured_at_monotonic=measured_at,
                          diagnostics=diagnostics,
                          latest_frame_metadata={role: {k: v for k, v in frame.items() if k != "rgb"}
                                                 for role, frame in frames.items()})
            group.close()
            result["shutdown_duration_s"] = time.monotonic() - measured_at
            result["workers_after_shutdown"] = group.diagnostics()["running_workers"]
            if args.output:
                _save(args.output, result, group.latest_frames())
        print(json.dumps(result, indent=2, allow_nan=False))
    except KeyboardInterrupt:
        exit_code = 130
    except (RuntimeError, ValueError, OSError, ImportError, KeyError) as exc:
        print(f"Camera diagnosis failed: {exc}", file=sys.stderr)
        exit_code = 1
    finally:
        if group:
            try:
                group.close()
            except RuntimeError as exc:
                print(str(exc), file=sys.stderr)
                exit_code = 1
        if preview_open:
            importlib.import_module("cv2").destroyAllWindows()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
