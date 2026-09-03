from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Tuple

from spacemouse_teleop.backends.mujoco import (
    DEFAULT_END_EFFECTOR,
    END_EFFECTOR_NAMES,
    XArm6TableCubeEnv,
)
from spacemouse_teleop.cli.common import format_vec
from spacemouse_teleop.spacemouse.command import TeleopCommand


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Probe xArm gripper/cube lift behavior in MuJoCo."
    )
    parser.add_argument("--hz", type=float, default=60.0)
    parser.add_argument("--model", default=None, help="Optional MuJoCo XML model path.")
    parser.add_argument(
        "--end-effector",
        choices=END_EFFECTOR_NAMES,
        default=DEFAULT_END_EFFECTOR,
    )
    parser.add_argument("--target-x", type=float, default=0.450)
    parser.add_argument("--target-y", type=float, default=0.0)
    parser.add_argument("--target-z", type=float, default=0.755)
    parser.add_argument("--approach-duration", type=float, default=2.0)
    parser.add_argument("--close-duration", type=float, default=3.0)
    parser.add_argument("--lift-distance", type=float, default=0.12)
    parser.add_argument("--lift-duration", type=float, default=1.5)
    parser.add_argument("--min-lift", type=float, default=0.08)
    return parser


def main() -> None:
    try:
        _main()
    except RuntimeError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc


def _main() -> None:
    args = build_parser().parse_args()
    env = XArm6TableCubeEnv(
        model_path=Path(args.model) if args.model else None,
        end_effector=args.end_effector,
        control_hz=args.hz,
        target_mode="velocity",
        arm_control_mode="kinematic",
        ik_position_gain=1.0,
        ik_orientation_gain=0.35,
    )
    initial = env.reset()
    target = (args.target_x, args.target_y, args.target_z)

    approach_steps = _steps(args.approach_duration, args.hz)
    start_pos = initial.ee_pos
    preclose = initial
    for _ in range(approach_steps):
        delta = tuple((target[i] - start_pos[i]) / approach_steps for i in range(3))
        preclose = env.step_command(_command(delta, gripper=0.0, hz=args.hz))

    close_steps = _steps(args.close_duration, args.hz)
    closed = preclose
    for index in range(1, close_steps + 1):
        gripper = min(1.0, index / close_steps)
        closed = env.step_command(
            _command((0.0, 0.0, 0.0), gripper=gripper, hz=args.hz)
        )

    lift_steps = _steps(args.lift_duration, args.hz)
    lifted = closed
    lift_delta = (0.0, 0.0, args.lift_distance / lift_steps)
    for _ in range(lift_steps):
        lifted = env.step_command(_command(lift_delta, gripper=1.0, hz=args.hz))

    cube_lift = lifted.cube_pos[2] - closed.cube_pos[2]
    print(
        f"initial_ee={format_vec(initial.ee_pos)} "
        f"cube={format_vec(initial.cube_pos)}"
    )
    print(
        f"preclose_ee={format_vec(preclose.ee_pos)} "
        f"cube={format_vec(preclose.cube_pos)}"
    )
    print(
        f"closed_ee={format_vec(closed.ee_pos)} cube={format_vec(closed.cube_pos)} "
        f"gripper={closed.gripper_closedness:0.3f}/"
        f"{closed.target_gripper_closedness:0.3f}"
    )
    print(
        f"lifted_ee={format_vec(lifted.ee_pos)} cube={format_vec(lifted.cube_pos)} "
        f"gripper={lifted.gripper_closedness:0.3f}/"
        f"{lifted.target_gripper_closedness:0.3f}"
    )
    print(f"cube_lift={cube_lift:+0.4f} min_lift={args.min_lift:0.4f}")
    if cube_lift < args.min_lift:
        raise RuntimeError("gripper lift probe failed")


def _command(
    delta_pos: Tuple[float, float, float],
    gripper: float,
    hz: float,
) -> TeleopCommand:
    dt = 1.0 / hz
    return TeleopCommand(
        linear_vel_mps=tuple(value / dt for value in delta_pos),
        angular_vel_radps=(0.0, 0.0, 0.0),
        delta_pos_m=delta_pos,
        delta_rot_rad=(0.0, 0.0, 0.0),
        gripper=gripper,
        enabled=True,
        frame="link_base",
        dt=dt,
        timestamp=0.0,
    )


def _steps(duration: float, hz: float) -> int:
    return max(1, int(round(duration * hz)))


if __name__ == "__main__":
    main()
