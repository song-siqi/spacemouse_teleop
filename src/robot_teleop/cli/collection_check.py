"""Inspect one native episode without touching its files or any hardware."""
from __future__ import annotations

import argparse
import json

from robot_teleop.recording.raw import check_episode


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode")
    parser.add_argument("--require-kept", action="store_true", help="Also require the operator to have marked the episode kept.")
    args = parser.parse_args(argv)
    report = check_episode(args.episode, require_kept=args.require_kept)
    print(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False))
    return 0 if report["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
