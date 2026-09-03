from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Optional

from spacemouse_teleop.cli.common import (
    LoopRate,
    add_reader_args,
    format_vec,
    load_config,
    should_stop,
)
from spacemouse_teleop.recording import JsonlRecorder
from spacemouse_teleop.spacemouse import TeleopCore
from spacemouse_teleop.spacemouse.readers import make_reader


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Print and optionally log normalized TeleopCommand output."
    )
    add_reader_args(parser)
    parser.add_argument(
        "--config",
        default="configs/spacemouse_xarm6.json",
        help="JSON config path.",
    )
    parser.add_argument("--log", default=None, help="Optional JSONL log path.")
    parser.add_argument(
        "--mode",
        choices=("delta-ee", "delta-joint-placeholder"),
        default="delta-ee",
        help="Dataset action mode label to record for this run.",
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
    config = load_config(args.config)
    core = TeleopCore(config)
    recorder: Optional[JsonlRecorder] = (
        JsonlRecorder(Path(args.log)) if args.log else None
    )

    start = time.monotonic()
    rate = LoopRate(args.hz)
    next_print = 0.0
    print_period = 1.0 / max(args.print_rate, 0.1)

    try:
        if recorder:
            recorder.open()
        with make_reader(
            args.backend,
            hz=args.hz,
            device=args.device,
            device_index=args.device_index,
            axis_convention=args.axis_convention,
        ) as reader:
            while not should_stop(start, args.duration):
                raw = reader.read()
                if raw is None:
                    rate.sleep()
                    continue
                command = core.process(raw)
                if recorder:
                    recorder.write(raw, command, mode=args.mode)

                now = time.monotonic()
                if now >= next_print:
                    next_print = now + print_period
                    gripper = "none" if command.gripper is None else f"{command.gripper:0.3f}"
                    print(
                        f"enabled={int(command.enabled)} frame={command.frame} "
                        f"v={format_vec(command.linear_vel_mps)} "
                        f"w={format_vec(command.angular_vel_radps)} "
                        f"dpos={format_vec(command.delta_pos_m)} "
                        f"drot={format_vec(command.delta_rot_rad)} "
                        f"gintent={command.gripper_intent} "
                        f"gripper={gripper} "
                        f"dgripper={command.delta_gripper:+0.4f} "
                        f"gripper_vel={command.gripper_velocity:+0.3f} "
                        f"dt={command.dt:0.4f}"
                    )
                rate.sleep()
    finally:
        if recorder:
            recorder.close()


if __name__ == "__main__":
    main()
