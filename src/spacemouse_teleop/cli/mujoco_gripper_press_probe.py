from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Optional, Tuple

from spacemouse_teleop.backends.mujoco import (
    DEFAULT_END_EFFECTOR,
    END_EFFECTOR_NAMES,
    XArm6TableCubeEnv,
)
from spacemouse_teleop.cli.common import format_vec
from spacemouse_teleop.spacemouse.command import TeleopCommand


TABLE_TOP_Z = 0.72
CUBE_HALF_SIZE_M = 0.025


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Probe cube/table penetration while pressing with the gripper."
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
    parser.add_argument("--target-z", type=float, default=0.775)
    parser.add_argument("--approach-duration", type=float, default=2.0)
    parser.add_argument("--close-duration", type=float, default=1.5)
    parser.add_argument("--press-distance", type=float, default=0.04)
    parser.add_argument("--press-duration", type=float, default=1.0)
    parser.add_argument("--max-table-penetration", type=float, default=0.002)
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

    min_cube_bottom_gap = _cube_bottom_gap(initial.cube_pos)
    min_table_contact_dist = _cube_table_contact_dist(env)

    approach_steps = _steps(args.approach_duration, args.hz)
    start_pos = initial.ee_pos
    closed = initial
    for _ in range(approach_steps):
        delta = tuple((target[i] - start_pos[i]) / approach_steps for i in range(3))
        closed = env.step_command(_command(delta, gripper=0.0, hz=args.hz))
        min_cube_bottom_gap = min(
            min_cube_bottom_gap, _cube_bottom_gap(closed.cube_pos)
        )
        min_table_contact_dist = _min_optional(
            min_table_contact_dist, _cube_table_contact_dist(env)
        )

    close_steps = _steps(args.close_duration, args.hz)
    for index in range(1, close_steps + 1):
        gripper = min(1.0, index / close_steps)
        closed = env.step_command(
            _command((0.0, 0.0, 0.0), gripper=gripper, hz=args.hz)
        )
        min_cube_bottom_gap = min(
            min_cube_bottom_gap, _cube_bottom_gap(closed.cube_pos)
        )
        min_table_contact_dist = _min_optional(
            min_table_contact_dist, _cube_table_contact_dist(env)
        )

    press_steps = _steps(args.press_duration, args.hz)
    pressed = closed
    press_delta = (0.0, 0.0, -abs(args.press_distance) / press_steps)
    for _ in range(press_steps):
        pressed = env.step_command(
            _command(press_delta, gripper=1.0, hz=args.hz)
        )
        min_cube_bottom_gap = min(
            min_cube_bottom_gap, _cube_bottom_gap(pressed.cube_pos)
        )
        min_table_contact_dist = _min_optional(
            min_table_contact_dist, _cube_table_contact_dist(env)
        )

    contact_penetration = max(0.0, -(min_table_contact_dist or 0.0))
    bottom_penetration = max(0.0, -min_cube_bottom_gap)
    worst_penetration = max(contact_penetration, bottom_penetration)
    print(
        f"initial_ee={format_vec(initial.ee_pos)} "
        f"cube={format_vec(initial.cube_pos)}"
    )
    print(
        f"pressed_ee={format_vec(pressed.ee_pos)} cube={format_vec(pressed.cube_pos)} "
        f"gripper={pressed.gripper_closedness:0.3f}/"
        f"{pressed.target_gripper_closedness:0.3f}"
    )
    print(
        f"min_cube_bottom_gap={min_cube_bottom_gap:+0.5f} "
        f"min_table_contact_dist={_format_optional(min_table_contact_dist)}"
    )
    print(
        f"worst_table_penetration={worst_penetration:0.5f} "
        f"max_table_penetration={args.max_table_penetration:0.5f}"
    )
    if worst_penetration > args.max_table_penetration:
        raise RuntimeError("gripper press probe exceeded table penetration limit")


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


def _cube_bottom_gap(cube_pos: Tuple[float, ...]) -> float:
    return float(cube_pos[2]) - CUBE_HALF_SIZE_M - TABLE_TOP_Z


def _cube_table_contact_dist(env: XArm6TableCubeEnv) -> Optional[float]:
    distances = []
    for index in range(env.data.ncon):
        contact = env.data.contact[index]
        geom1 = env.mujoco.mj_id2name(
            env.model, env.mujoco.mjtObj.mjOBJ_GEOM, contact.geom1
        )
        geom2 = env.mujoco.mj_id2name(
            env.model, env.mujoco.mjtObj.mjOBJ_GEOM, contact.geom2
        )
        if {str(geom1), str(geom2)} == {"table", "cube_geom"}:
            distances.append(float(contact.dist))
    return min(distances) if distances else None


def _min_optional(first: Optional[float], second: Optional[float]) -> Optional[float]:
    if first is None:
        return second
    if second is None:
        return first
    return min(first, second)


def _format_optional(value: Optional[float]) -> str:
    if value is None:
        return "none"
    return f"{value:+0.5f}"


if __name__ == "__main__":
    main()
