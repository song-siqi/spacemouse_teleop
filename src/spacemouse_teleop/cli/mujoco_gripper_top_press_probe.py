from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

from spacemouse_teleop.backends.mujoco import (
    DEFAULT_END_EFFECTOR,
    END_EFFECTOR_NAMES,
    GRIPPER_GUARD_GEOM_NAMES,
    XArm6TableCubeEnv,
)
from spacemouse_teleop.cli.common import format_vec
from spacemouse_teleop.spacemouse.command import TeleopCommand

TABLE_TOP_Z = 0.72
CUBE_HALF_SIZE_M = 0.025


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Probe open-gripper shell contact while pressing a cube from above."
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
    parser.add_argument("--approach-z", type=float, default=0.820)
    parser.add_argument("--press-z", type=float, default=0.755)
    parser.add_argument("--approach-duration", type=float, default=2.0)
    parser.add_argument("--press-duration", type=float, default=1.0)
    parser.add_argument("--max-guard-penetration", type=float, default=0.0005)
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
    approach_target = (args.target_x, args.target_y, args.approach_z)

    approach_steps = _steps(args.approach_duration, args.hz)
    start_pos = initial.ee_pos
    approached = initial
    for _ in range(approach_steps):
        delta = tuple(
            (approach_target[index] - start_pos[index]) / approach_steps
            for index in range(3)
        )
        approached = env.step_command(_command(delta, hz=args.hz))

    press_steps = _steps(args.press_duration, args.hz)
    press_delta = (
        0.0,
        0.0,
        (args.press_z - args.approach_z) / press_steps,
    )
    pressed = approached
    guard_contact_seen = False
    guard_blocked_seen = False
    min_guard_contact_dist: Optional[float] = None
    min_table_contact_dist: Optional[float] = None
    min_cube_bottom_gap = _cube_bottom_gap(pressed.cube_pos)
    max_guard_normal_force = 0.0
    for _ in range(press_steps):
        pressed = env.step_command(_command(press_delta, hz=args.hz))
        guard_blocked_seen = (
            guard_blocked_seen or env.last_kinematic_guard_blocked
        )
        guard_dist, guard_force = _guard_cube_contact(env)
        guard_contact_seen = guard_contact_seen or guard_dist is not None
        min_guard_contact_dist = _min_optional(
            min_guard_contact_dist, guard_dist
        )
        max_guard_normal_force = max(max_guard_normal_force, guard_force)
        min_table_contact_dist = _min_optional(
            min_table_contact_dist, _cube_table_contact_dist(env)
        )
        min_cube_bottom_gap = min(
            min_cube_bottom_gap, _cube_bottom_gap(pressed.cube_pos)
        )

    guard_penetration = max(0.0, -(min_guard_contact_dist or 0.0))
    table_penetration = max(
        max(0.0, -(min_table_contact_dist or 0.0)),
        max(0.0, -min_cube_bottom_gap),
    )
    print(
        f"initial_ee={format_vec(initial.ee_pos)} "
        f"cube={format_vec(initial.cube_pos)}"
    )
    print(
        f"approached_ee={format_vec(approached.ee_pos)} "
        f"pressed_ee={format_vec(pressed.ee_pos)} "
        f"cube={format_vec(pressed.cube_pos)}"
    )
    print(
        f"guard_contact={int(guard_contact_seen)} "
        f"guard_blocked={int(guard_blocked_seen)} "
        f"min_guard_contact_dist={_format_optional(min_guard_contact_dist)} "
        f"max_guard_normal_force={max_guard_normal_force:0.3f}"
    )
    print(
        f"guard_penetration={guard_penetration:0.5f}/"
        f"{args.max_guard_penetration:0.5f} "
        f"table_penetration={table_penetration:0.5f}/"
        f"{args.max_table_penetration:0.5f}"
    )
    if not (guard_contact_seen or guard_blocked_seen):
        raise RuntimeError("top press probe did not reach a gripper guard")
    if guard_penetration > args.max_guard_penetration:
        raise RuntimeError("top press probe exceeded guard penetration limit")
    if table_penetration > args.max_table_penetration:
        raise RuntimeError("top press probe exceeded table penetration limit")


def _command(delta_pos: Tuple[float, float, float], hz: float) -> TeleopCommand:
    dt = 1.0 / hz
    return TeleopCommand(
        linear_vel_mps=tuple(value / dt for value in delta_pos),
        angular_vel_radps=(0.0, 0.0, 0.0),
        delta_pos_m=delta_pos,
        delta_rot_rad=(0.0, 0.0, 0.0),
        gripper=0.0,
        enabled=True,
        frame="link_base",
        dt=dt,
        timestamp=0.0,
    )


def _steps(duration: float, hz: float) -> int:
    return max(1, int(round(duration * hz)))


def _cube_bottom_gap(cube_pos: Tuple[float, ...]) -> float:
    return float(cube_pos[2]) - CUBE_HALF_SIZE_M - TABLE_TOP_Z


def _guard_cube_contact(
    env: XArm6TableCubeEnv,
) -> Tuple[Optional[float], float]:
    distances = []
    max_normal_force = 0.0
    guard_names = set(GRIPPER_GUARD_GEOM_NAMES)
    for index in range(env.data.ncon):
        contact = env.data.contact[index]
        names = _contact_geom_names(env, contact)
        if "cube_geom" not in names or not names.intersection(guard_names):
            continue
        distances.append(float(contact.dist))
        force = np.zeros(6, dtype=float)
        env.mujoco.mj_contactForce(env.model, env.data, index, force)
        max_normal_force = max(max_normal_force, abs(float(force[0])))
    return (min(distances) if distances else None), max_normal_force


def _cube_table_contact_dist(env: XArm6TableCubeEnv) -> Optional[float]:
    distances = []
    for index in range(env.data.ncon):
        contact = env.data.contact[index]
        if _contact_geom_names(env, contact) == {"table", "cube_geom"}:
            distances.append(float(contact.dist))
    return min(distances) if distances else None


def _contact_geom_names(env: XArm6TableCubeEnv, contact) -> set[str]:
    return {
        str(
            env.mujoco.mj_id2name(
                env.model, env.mujoco.mjtObj.mjOBJ_GEOM, geom_id
            )
        )
        for geom_id in (contact.geom1, contact.geom2)
    }


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
