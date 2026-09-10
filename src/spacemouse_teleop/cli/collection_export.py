"""Export complete accepted episodes offline; no robot or camera connection."""
from __future__ import annotations

import argparse
import json
import sys

from spacemouse_teleop.recording.export import export_episode


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode")
    parser.add_argument("output")
    parser.add_argument("--format", choices=("legacy", "light", "derived"), default="legacy")
    parser.add_argument("--fps", type=int, choices=(20, 10), default=20)
    parser.add_argument("--fixed-view", choices=("fixed_1", "fixed_2"), help="Required for legacy/light: select the fixed view paired with wrist.")
    parser.add_argument("--gripper-open", type=float, help="Required for derived: calibrated open endpoint in raw pulse.")
    parser.add_argument("--gripper-close", type=float, help="Required for derived: calibrated close endpoint in raw pulse.")
    parser.add_argument("--binary-threshold", type=float, default=.5, help="Derived binary-open threshold in normalized [0,1] space.")
    args = parser.parse_args(argv)
    try:
        result = export_episode(args.episode, args.output, format=args.format, fps=args.fps,
                                fixed_view=args.fixed_view, gripper_open=args.gripper_open,
                                gripper_close=args.gripper_close, binary_threshold=args.binary_threshold)
    except (OSError, ValueError, TypeError, KeyError, AttributeError) as exc:
        print(f"Export failed: {exc}", file=sys.stderr)
        return 1
    print(json.dumps({"output": args.output, "format": result["format"], "fps": result["fps"],
                      "observations": result["observation_count"], "training_pairs": result["training_pair_count"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
