from __future__ import annotations

import argparse
import sys
import time

from spacemouse_teleop.cli.common import (
    LoopRate,
    add_reader_args,
    format_vec,
    should_stop,
)
from spacemouse_teleop.spacemouse.readers import make_reader


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Print raw SpaceMouse axes/buttons.")
    add_reader_args(parser)
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
    start = time.monotonic()
    rate = LoopRate(args.hz)
    next_print = 0.0
    print_period = 1.0 / max(args.print_rate, 0.1)

    with make_reader(
        args.backend,
        hz=args.hz,
        device=args.device,
        device_index=args.device_index,
        axis_convention=args.axis_convention,
    ) as reader:
        while not should_stop(start, args.duration):
            state = reader.read()
            if state is None:
                rate.sleep()
                continue

            now = time.monotonic()
            if now >= next_print:
                next_print = now + print_period
                linear = (state.x, state.y, state.z)
                angular = (state.roll, state.pitch, state.yaw)
                print(
                    f"raw linear={format_vec(linear)} "
                    f"angular={format_vec(angular)} buttons={state.buttons}"
                )
            rate.sleep()


if __name__ == "__main__":
    main()
