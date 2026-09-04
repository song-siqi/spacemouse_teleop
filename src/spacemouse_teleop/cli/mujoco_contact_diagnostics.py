from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

from spacemouse_teleop.backends.mujoco import (
    DEFAULT_END_EFFECTOR,
    END_EFFECTOR_NAMES,
    XArm6TableCubeEnv,
)
from spacemouse_teleop.backends.mujoco.contact_diagnostics import (
    ContactSnapshot,
    PhaseContactSummary,
    capture_contact_snapshot,
    summarize_contact_pairs,
    summarize_contact_snapshots,
)
from spacemouse_teleop.spacemouse.command import TeleopCommand


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Measure MuJoCo manipulation-object contacts during repeatable "
            "grasp, rotate, outer-push, and retreat phases."
        )
    )
    parser.add_argument("--hz", type=float, default=60.0)
    parser.add_argument("--model", default=None, help="Optional MuJoCo XML model path.")
    parser.add_argument(
        "--object-body",
        default="cube",
        help="Tracked manipulation-object body name.",
    )
    parser.add_argument(
        "--object-geom",
        action="append",
        default=None,
        help=(
            "Tracked object geom name; repeat for multi-geom objects. "
            "Defaults to all geoms in --object-body's subtree."
        ),
    )
    parser.add_argument(
        "--end-effector",
        choices=END_EFFECTOR_NAMES,
        default=DEFAULT_END_EFFECTOR,
    )
    parser.add_argument(
        "--arm-control-mode",
        choices=("kinematic", "actuator"),
        default="kinematic",
    )
    parser.add_argument(
        "--target-mode",
        choices=("velocity", "integrated"),
        default="velocity",
    )
    parser.add_argument(
        "--max-ee-angular-speed",
        type=float,
        default=0.25,
        help="MuJoCo EE target rotation limit in rad/s; use 0 to disable.",
    )
    parser.add_argument("--target-x", type=float, default=0.450)
    parser.add_argument("--target-y", type=float, default=0.0)
    parser.add_argument("--target-z", type=float, default=0.755)
    parser.add_argument("--approach-duration", type=float, default=2.0)
    parser.add_argument("--close-duration", type=float, default=2.0)
    parser.add_argument("--lift-distance", type=float, default=0.08)
    parser.add_argument("--lift-duration", type=float, default=1.5)
    parser.add_argument("--rotate-degrees", type=float, default=60.0)
    parser.add_argument("--rotate-duration", type=float, default=1.5)
    parser.add_argument("--hold-duration", type=float, default=0.75)
    parser.add_argument("--outer-approach-y-offset", type=float, default=0.14)
    parser.add_argument("--outer-approach-duration", type=float, default=2.0)
    parser.add_argument("--outer-push-distance", type=float, default=0.16)
    parser.add_argument("--outer-push-duration", type=float, default=2.0)
    parser.add_argument("--outer-retreat-distance", type=float, default=0.05)
    parser.add_argument("--outer-retreat-duration", type=float, default=0.75)
    parser.add_argument("--log", default=None, help="Optional JSONL output path.")
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
        cube_body_name=args.object_body,
        control_hz=args.hz,
        target_mode=args.target_mode,
        arm_control_mode=args.arm_control_mode,
        ik_position_gain=1.0,
        ik_orientation_gain=0.35,
        max_ee_angular_speed_radps=args.max_ee_angular_speed,
    )
    snapshots: List[ContactSnapshot] = []

    initial = env.reset()
    target = (args.target_x, args.target_y, args.target_z)
    approach_steps = _steps(args.approach_duration, args.hz)
    approach_delta = tuple(
        (target[index] - initial.ee_pos[index]) / approach_steps
        for index in range(3)
    )
    _run_phase(
        env,
        snapshots,
        "approach",
        approach_steps,
        args.hz,
        delta_pos=approach_delta,
        first_intent="open",
        object_body_name=args.object_body,
        object_geom_names=args.object_geom,
    )
    _run_phase(
        env,
        snapshots,
        "close",
        _steps(args.close_duration, args.hz),
        args.hz,
        first_intent="close",
        object_body_name=args.object_body,
        object_geom_names=args.object_geom,
    )
    lift_steps = _steps(args.lift_duration, args.hz)
    _run_phase(
        env,
        snapshots,
        "lift",
        lift_steps,
        args.hz,
        delta_pos=(0.0, 0.0, args.lift_distance / lift_steps),
        object_body_name=args.object_body,
        object_geom_names=args.object_geom,
    )
    rotate_steps = _steps(args.rotate_duration, args.hz)
    _run_phase(
        env,
        snapshots,
        "rotate",
        rotate_steps,
        args.hz,
        delta_rot=(0.0, 0.0, math.radians(args.rotate_degrees) / rotate_steps),
        object_body_name=args.object_body,
        object_geom_names=args.object_geom,
    )
    _run_phase(
        env,
        snapshots,
        "hold",
        _steps(args.hold_duration, args.hz),
        args.hz,
        object_body_name=args.object_body,
        object_geom_names=args.object_geom,
    )
    grasp_final = env.observe()

    outer_initial = env.reset()
    outer_target = (
        args.target_x,
        args.target_y + args.outer_approach_y_offset,
        args.target_z,
    )
    outer_approach_steps = _steps(args.outer_approach_duration, args.hz)
    outer_approach_delta = tuple(
        (outer_target[index] - outer_initial.ee_pos[index])
        / outer_approach_steps
        for index in range(3)
    )
    _run_phase(
        env,
        snapshots,
        "outer_approach",
        outer_approach_steps,
        args.hz,
        delta_pos=outer_approach_delta,
        first_intent="open",
        object_body_name=args.object_body,
        object_geom_names=args.object_geom,
    )
    outer_push_steps = _steps(args.outer_push_duration, args.hz)
    _run_phase(
        env,
        snapshots,
        "outer_push",
        outer_push_steps,
        args.hz,
        delta_pos=(0.0, -abs(args.outer_push_distance) / outer_push_steps, 0.0),
        object_body_name=args.object_body,
        object_geom_names=args.object_geom,
    )
    outer_pushed = env.observe()
    outer_retreat_steps = _steps(args.outer_retreat_duration, args.hz)
    _run_phase(
        env,
        snapshots,
        "outer_retreat",
        outer_retreat_steps,
        args.hz,
        delta_pos=(
            0.0,
            abs(args.outer_retreat_distance) / outer_retreat_steps,
            0.0,
        ),
        object_body_name=args.object_body,
        object_geom_names=args.object_geom,
    )
    outer_final = env.observe()
    outer_retreat_drag = math.dist(outer_pushed.cube_pos, outer_final.cube_pos)

    print(
        "phase          n  obj% lpad% rpad% guard% table%   ee_w   obj_v "
        "  obj_w   rel_w ee_deg ob_deg slip_deg    max_ke  max_pen"
    )
    for summary in summarize_contact_snapshots(snapshots):
        print(
            f"{summary.phase:<14} {summary.sample_count:>3d} "
            f"{100.0 * summary.object_contact_fraction:>5.1f} "
            f"{100.0 * summary.left_pad.contact_fraction:>5.1f} "
            f"{100.0 * summary.right_pad.contact_fraction:>5.1f} "
            f"{100.0 * summary.guards.contact_fraction:>6.1f} "
            f"{100.0 * summary.table.contact_fraction:>6.1f} "
            f"{summary.max_ee_angular_speed_radps:>6.3f} "
            f"{summary.max_object_linear_speed_mps:>7.3f} "
            f"{summary.max_object_angular_speed_radps:>7.3f} "
            f"{summary.max_relative_angular_speed_radps:>7.3f} "
            f"{math.degrees(summary.net_ee_rotation_rad):>6.1f} "
            f"{math.degrees(summary.net_object_rotation_rad):>6.1f} "
            f"{math.degrees(summary.relative_rotation_drift_rad):>8.1f} "
            f"{summary.max_object_kinetic_energy_j:>9.5f} "
            f"{1000.0 * summary.max_penetration_m:>8.3f}mm"
        )
        _print_role_contacts(summary)

    print("contact_pairs:")
    pair_summaries = summarize_contact_pairs(snapshots)
    for pair_name, stats in sorted(
        pair_summaries.items(),
        key=lambda item: item[1]["max_normal_force_n"],
        reverse=True,
    ):
        print(
            f"  {pair_name}: samples={stats['samples']} "
            f"fn={stats['max_normal_force_n']:.2f}N "
            f"ft={stats['max_tangential_force_n']:.2f}N "
            f"tau={stats['max_torsional_torque_nm']:.4f}Nm "
            f"pen={1000.0 * stats['max_penetration_m']:.3f}mm"
        )
    print(
        "grasp_final_object="
        f"[{', '.join(f'{value:+.4f}' for value in grasp_final.cube_pos)}] "
        f"gripper={grasp_final.gripper_closedness:.3f}/"
        f"{grasp_final.target_gripper_closedness:.3f}"
    )
    print(
        "outer_final_object="
        f"[{', '.join(f'{value:+.4f}' for value in outer_final.cube_pos)}] "
        f"retreat_drag={1000.0 * outer_retreat_drag:.3f}mm"
    )

    if args.log:
        log_path = Path(args.log)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        with log_path.open("w", encoding="utf-8") as stream:
            for snapshot in snapshots:
                stream.write(json.dumps(snapshot.to_dict(), sort_keys=True) + "\n")
        print(f"wrote {len(snapshots)} samples to {log_path}")


def _run_phase(
    env: XArm6TableCubeEnv,
    snapshots: List[ContactSnapshot],
    phase: str,
    steps: int,
    hz: float,
    delta_pos: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    delta_rot: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    first_intent: str = "hold",
    object_body_name: str = "cube",
    object_geom_names: Optional[Sequence[str]] = None,
) -> None:
    for index in range(steps):
        intent = first_intent if index == 0 else "hold"
        env.step_command(_command(delta_pos, delta_rot, intent, hz))
        snapshots.append(
            capture_contact_snapshot(
                env,
                phase,
                object_body_name=object_body_name,
                object_geom_names=object_geom_names,
            )
        )


def _print_role_contacts(summary: PhaseContactSummary) -> None:
    roles = (
        ("left_pad", summary.left_pad),
        ("right_pad", summary.right_pad),
        ("guards", summary.guards),
        ("table", summary.table),
    )
    active = []
    for name, role in roles:
        if role.contact_count == 0:
            continue
        active.append(
            f"{name}:n={role.contact_count},fn={role.max_normal_force_n:.2f}N,"
            f"ft={role.max_tangential_force_n:.2f}N,"
            f"tau={role.max_torsional_torque_nm:.4f}Nm,"
            f"pen={1000.0 * role.max_penetration_m:.3f}mm"
        )
    if active:
        print("  " + " | ".join(active))


def _command(
    delta_pos: Tuple[float, float, float],
    delta_rot: Tuple[float, float, float],
    gripper_intent: str,
    hz: float,
) -> TeleopCommand:
    dt = 1.0 / hz
    return TeleopCommand(
        linear_vel_mps=tuple(value / dt for value in delta_pos),
        angular_vel_radps=tuple(value / dt for value in delta_rot),
        delta_pos_m=delta_pos,
        delta_rot_rad=delta_rot,
        enabled=True,
        frame="link_base",
        dt=dt,
        timestamp=0.0,
        gripper_intent=gripper_intent,
    )


def _steps(duration: float, hz: float) -> int:
    return max(1, int(round(duration * hz)))


if __name__ == "__main__":
    main()
