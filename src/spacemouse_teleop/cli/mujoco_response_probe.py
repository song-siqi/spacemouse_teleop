from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Tuple

import numpy as np

from spacemouse_teleop.backends.mujoco import (
    DEFAULT_END_EFFECTOR,
    END_EFFECTOR_NAMES,
    XArm6TableCubeEnv,
)
from spacemouse_teleop.cli.common import format_vec
from spacemouse_teleop.spacemouse.command import TeleopCommand


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Probe MuJoCo xArm6 command response without SpaceMouse input."
    )
    parser.add_argument("--hz", type=float, default=60.0)
    parser.add_argument("--move-duration", type=float, default=0.5)
    parser.add_argument("--hold-duration", type=float, default=1.0)
    parser.add_argument("--linear-x", type=float, default=0.18)
    parser.add_argument("--linear-y", type=float, default=0.0)
    parser.add_argument("--linear-z", type=float, default=0.0)
    parser.add_argument("--angular-x", type=float, default=0.0)
    parser.add_argument("--angular-y", type=float, default=0.0)
    parser.add_argument("--angular-z", type=float, default=0.0)
    parser.add_argument(
        "--target-mode",
        choices=("velocity", "integrated"),
        default="velocity",
    )
    parser.add_argument(
        "--arm-control-mode",
        choices=("kinematic", "actuator"),
        default="kinematic",
    )
    parser.add_argument("--position-gain", type=float, default=0.6)
    parser.add_argument("--orientation-gain", type=float, default=0.35)
    parser.add_argument("--model", default=None, help="Optional MuJoCo XML model path.")
    parser.add_argument(
        "--end-effector",
        choices=END_EFFECTOR_NAMES,
        default=DEFAULT_END_EFFECTOR,
        help="Generated MuJoCo model end effector when --model is not provided.",
    )
    return parser


def main() -> None:
    try:
        _main()
    except RuntimeError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc


def _main() -> None:
    args = build_parser().parse_args()
    dt = 1.0 / args.hz
    env = XArm6TableCubeEnv(
        model_path=Path(args.model) if args.model else None,
        end_effector=args.end_effector,
        control_hz=args.hz,
        target_mode=args.target_mode,
        arm_control_mode=args.arm_control_mode,
        ik_position_gain=args.position_gain,
        ik_orientation_gain=args.orientation_gain,
    )
    initial = env.reset()

    linear = (args.linear_x, args.linear_y, args.linear_z)
    angular = (args.angular_x, args.angular_y, args.angular_z)
    for _ in range(_steps(args.move_duration, args.hz)):
        env.step_command(_command(linear, angular, dt, enabled=True))
    stop_start = env.observe()

    for _ in range(_steps(args.hold_duration, args.hz)):
        env.step_command(_command((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), dt, enabled=True))
    final = env.observe()

    move_delta = np.asarray(stop_start.ee_pos) - np.asarray(initial.ee_pos)
    drift_after_stop = np.asarray(final.ee_pos) - np.asarray(stop_start.ee_pos)
    target_error = np.asarray(final.target_ee_pos) - np.asarray(final.ee_pos)

    print(
        f"target_mode={args.target_mode} "
        f"arm_control_mode={args.arm_control_mode} "
        f"position_gain={args.position_gain:0.3f} "
        f"orientation_gain={args.orientation_gain:0.3f} "
        f"hz={args.hz:0.1f}"
    )
    print(f"command_linear={format_vec(linear)} command_angular={format_vec(angular)}")
    print(f"move_duration_s={args.move_duration:0.3f} hold_duration_s={args.hold_duration:0.3f}")
    print(f"initial_ee={format_vec(initial.ee_pos)}")
    print(f"stop_start_ee={format_vec(stop_start.ee_pos)}")
    print(f"final_ee={format_vec(final.ee_pos)}")
    print(f"move_delta={format_vec(move_delta)}")
    print(f"drift_after_stop={format_vec(drift_after_stop)}")
    print(f"final_target_error={format_vec(target_error)}")
    print(f"final_q={format_vec(final.joint_pos)}")


def _steps(duration: float, hz: float) -> int:
    return max(1, int(round(duration * hz)))


def _command(
    linear_vel: Tuple[float, float, float],
    angular_vel: Tuple[float, float, float],
    dt: float,
    enabled: bool,
) -> TeleopCommand:
    return TeleopCommand(
        linear_vel_mps=linear_vel,
        angular_vel_radps=angular_vel,
        delta_pos_m=tuple(value * dt for value in linear_vel),
        delta_rot_rad=tuple(value * dt for value in angular_vel),
        gripper=0.0,
        enabled=enabled,
        frame="link_base",
        dt=dt,
        timestamp=0.0,
    )


if __name__ == "__main__":
    main()
