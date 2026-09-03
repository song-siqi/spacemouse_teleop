from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

from spacemouse_teleop.backends.mujoco import (
    DEFAULT_END_EFFECTOR,
    END_EFFECTOR_NAMES,
    XArm6TableCubeEnv,
    ensure_official_xarm6_table_cube_mjcf,
)
from spacemouse_teleop.cli.common import format_vec
from spacemouse_teleop.spacemouse.command import TeleopCommand


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Diagnose whether the generated MuJoCo xArm6 model can hold pose."
    )
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--hz", type=float, default=60.0)
    parser.add_argument("--model", default=None, help="Optional MuJoCo XML model path.")
    parser.add_argument(
        "--end-effector",
        choices=END_EFFECTOR_NAMES,
        default=DEFAULT_END_EFFECTOR,
        help="Generated MuJoCo model end effector when --model is not provided.",
    )
    parser.add_argument(
        "--arm-control-mode",
        choices=("kinematic", "actuator"),
        default="kinematic",
        help="Hold mode to diagnose.",
    )
    parser.add_argument(
        "--force-regenerate",
        action="store_true",
        help="Regenerate the official-derived xArm6 tabletop MJCF before testing.",
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
    model_path = Path(args.model) if args.model else None
    if model_path is None:
        model_path = ensure_official_xarm6_table_cube_mjcf(
            end_effector=args.end_effector,
            force=args.force_regenerate,
        )

    env = XArm6TableCubeEnv(
        model_path=model_path,
        end_effector=args.end_effector,
        control_hz=args.hz,
        arm_control_mode=args.arm_control_mode,
    )
    before = env.reset()
    steps = max(1, int(round(args.duration * args.hz)))
    for _ in range(steps):
        env.step_command(_zero_command(1.0 / args.hz))
    after = env.observe()

    q_delta = np.asarray(after.joint_pos) - np.asarray(before.joint_pos)
    ee_delta = np.asarray(after.ee_pos) - np.asarray(before.ee_pos)
    actuator_force = env.data.qfrc_actuator[env.ik.dof_ids]
    bias_force = env.data.qfrc_bias[env.ik.dof_ids]
    gravcomp_force = env.data.qfrc_gravcomp[env.ik.dof_ids]

    print(f"model={env.model_path}")
    print(f"duration_s={args.duration:0.3f} hz={args.hz:0.1f}")
    print(f"end_effector={args.end_effector} arm_control_mode={args.arm_control_mode}")
    print(f"initial_ee={format_vec(before.ee_pos)}")
    print(f"final_ee={format_vec(after.ee_pos)}")
    print(f"ee_delta={format_vec(ee_delta)}")
    print(
        "gripper="
        f"{after.gripper_closedness:0.3f}/{after.target_gripper_closedness:0.3f}"
    )
    print(f"initial_q={format_vec(before.joint_pos)}")
    print(f"final_q={format_vec(after.joint_pos)}")
    print(f"q_delta={format_vec(q_delta)}")
    print(f"ctrl={format_vec(env.data.ctrl[env.actuator_ids])}")
    print(f"qfrc_actuator={format_vec(actuator_force)}")
    print(f"qfrc_bias={format_vec(bias_force)}")
    print(f"qfrc_gravcomp={format_vec(gravcomp_force)}")


def _zero_command(dt: float) -> TeleopCommand:
    return TeleopCommand(
        linear_vel_mps=(0.0, 0.0, 0.0),
        angular_vel_radps=(0.0, 0.0, 0.0),
        delta_pos_m=(0.0, 0.0, 0.0),
        delta_rot_rad=(0.0, 0.0, 0.0),
        gripper=0.0,
        enabled=True,
        frame="link_base",
        dt=dt,
        timestamp=0.0,
    )


if __name__ == "__main__":
    main()
