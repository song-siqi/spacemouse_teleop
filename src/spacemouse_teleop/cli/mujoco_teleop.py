from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Optional, Sequence

from spacemouse_teleop.backends.mujoco import (
    CAMERA_NAMES,
    DEFAULT_CAMERA,
    DEFAULT_END_EFFECTOR,
    END_EFFECTOR_NAMES,
    XArm6TableCubeEnv,
)
from spacemouse_teleop.backends.mujoco.multiview import (
    DEFAULT_MULTIVIEW_CAMERAS,
    ViewerCameraOverlay,
)
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
        description="Teleoperate the MuJoCo xArm6 table/cube scene."
    )
    add_reader_args(parser)
    parser.add_argument(
        "--config",
        default="configs/spacemouse_xarm6.json",
        help="SpaceMouse command config path.",
    )
    parser.add_argument(
        "--model",
        default=None,
        help="Optional MuJoCo XML model path.",
    )
    parser.add_argument(
        "--end-effector",
        choices=END_EFFECTOR_NAMES,
        default=DEFAULT_END_EFFECTOR,
        help="Generated MuJoCo model end effector when --model is not provided.",
    )
    parser.add_argument(
        "--viewer",
        action="store_true",
        help="Open the MuJoCo passive viewer.",
    )
    parser.add_argument(
        "--show-collision-geoms",
        action="store_true",
        help="Show translucent gripper pad and shell collision geoms.",
    )
    parser.add_argument(
        "--camera",
        default=DEFAULT_CAMERA,
        help=(
            "Initial MuJoCo fixed camera name. Use 'free' for the default free "
            f"camera. Generated scene cameras: {', '.join(CAMERA_NAMES)}."
        ),
    )
    parser.add_argument(
        "--list-cameras",
        action="store_true",
        help="Print available MuJoCo cameras and exit.",
    )
    parser.add_argument(
        "--multiview",
        action="store_true",
        help="Overlay live thumbnails from multiple fixed cameras in the viewer.",
    )
    parser.add_argument(
        "--multiview-cameras",
        default=",".join(DEFAULT_MULTIVIEW_CAMERAS),
        help="Comma-separated fixed camera names to show in multiview thumbnails.",
    )
    parser.add_argument(
        "--multiview-layout",
        choices=("grid", "column"),
        default="grid",
        help="Thumbnail layout for --multiview.",
    )
    parser.add_argument(
        "--multiview-rate",
        type=float,
        default=15.0,
        help="Maximum multiview thumbnail refresh rate in Hz.",
    )
    parser.add_argument(
        "--multiview-width",
        type=int,
        default=380,
        help="Requested thumbnail width in pixels.",
    )
    parser.add_argument(
        "--multiview-height",
        type=int,
        default=260,
        help="Requested thumbnail height in pixels.",
    )
    parser.add_argument(
        "--target-mode",
        choices=("velocity", "integrated"),
        default="velocity",
        help=(
            "MuJoCo EE target semantics. 'velocity' applies each command delta "
            "from the observed EE pose; 'integrated' accumulates a persistent "
            "EE target."
        ),
    )
    parser.add_argument(
        "--arm-control-mode",
        choices=("kinematic", "actuator"),
        default="kinematic",
        help=(
            "How the MuJoCo arm executes IK joint targets. 'kinematic' gives "
            "responsive teleop; 'actuator' uses MuJoCo position actuator dynamics."
        ),
    )
    parser.add_argument(
        "--position-gain",
        type=float,
        default=0.6,
        help="Controller weight for translational EE error in the MuJoCo IK step.",
    )
    parser.add_argument(
        "--orientation-gain",
        type=float,
        default=0.35,
        help="Controller weight for rotational EE error in the MuJoCo IK step.",
    )
    parser.add_argument(
        "--max-ee-angular-speed",
        type=float,
        default=0.25,
        help=(
            "MuJoCo EE target rotation limit in rad/s. Use 0 to disable the limit."
        ),
    )
    parser.add_argument(
        "--log",
        default=None,
        help="Optional JSONL path for raw command and MuJoCo observations.",
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
    if args.multiview and not args.viewer:
        raise RuntimeError("--multiview requires --viewer")
    if args.show_collision_geoms and not args.viewer:
        raise RuntimeError("--show-collision-geoms requires --viewer")

    config = load_config(args.config)
    core = TeleopCore(config)
    env = XArm6TableCubeEnv(
        model_path=Path(args.model) if args.model else None,
        end_effector=args.end_effector,
        control_hz=args.hz,
        target_mode=args.target_mode,
        arm_control_mode=args.arm_control_mode,
        ik_position_gain=args.position_gain,
        ik_orientation_gain=args.orientation_gain,
        max_ee_angular_speed_radps=args.max_ee_angular_speed,
    )
    observation = env.reset()
    env.set_gripper_collision_debug(args.show_collision_geoms)
    if args.list_cameras:
        print("available cameras: " + ", ".join(_camera_names(env)))
        return

    recorder: Optional[JsonlRecorder] = (
        JsonlRecorder(Path(args.log)) if args.log else None
    )
    viewer = _open_viewer(env, args.camera) if args.viewer else None
    multiview = (
        _open_multiview(viewer, env, _parse_camera_list(args.multiview_cameras), args)
        if viewer is not None and args.multiview
        else None
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
                if viewer is not None and not viewer.is_running():
                    break

                raw = reader.read()
                if raw is None:
                    env.step_physics(1.0 / max(args.hz, 1.0))
                    if viewer is not None:
                        _sync_viewer(viewer, multiview)
                    rate.sleep()
                    continue

                command = core.process(raw)
                observation = env.step_command(command)
                if recorder:
                    recorder.write(
                        raw,
                        command,
                        mode=(
                            f"mujoco-delta-ee-{args.target_mode}-"
                            f"{args.arm_control_mode}"
                        ),
                        extras={
                            "mujoco": observation.to_dict(),
                            "controller": {
                                "position_gain": args.position_gain,
                                "orientation_gain": args.orientation_gain,
                                "max_ee_angular_speed_radps": (
                                    args.max_ee_angular_speed
                                ),
                            },
                            "end_effector": args.end_effector,
                        },
                    )

                now = time.monotonic()
                if now >= next_print:
                    next_print = now + print_period
                    _print_status(command, observation)

                if viewer is not None:
                    _sync_viewer(viewer, multiview)
                rate.sleep()
    finally:
        if recorder:
            recorder.close()
        if multiview is not None:
            multiview.close()
        if viewer is not None:
            viewer.close()


def _open_viewer(env, camera_name: str):
    try:
        import mujoco.viewer
    except ImportError as exc:
        raise RuntimeError("MuJoCo viewer is unavailable in this environment") from exc
    viewer = mujoco.viewer.launch_passive(env.model, env.data)
    _set_viewer_camera(viewer, env, camera_name)
    return viewer


def _set_viewer_camera(viewer, env, camera_name: str) -> None:
    if not camera_name or camera_name == "free":
        return

    camera_id = env.mujoco.mj_name2id(
        env.model, env.mujoco.mjtObj.mjOBJ_CAMERA, camera_name
    )
    if camera_id < 0:
        available = ", ".join(_camera_names(env))
        raise RuntimeError(
            f"MuJoCo camera not found: {camera_name}. Available cameras: {available}"
        )
    with viewer.lock():
        viewer.cam.type = env.mujoco.mjtCamera.mjCAMERA_FIXED
        viewer.cam.fixedcamid = camera_id
    viewer.sync()


def _open_multiview(
    viewer,
    env,
    cameras: Sequence[str],
    args: argparse.Namespace,
) -> ViewerCameraOverlay:
    return ViewerCameraOverlay(
        env=env,
        viewer=viewer,
        cameras=cameras,
        layout=args.multiview_layout,
        pane_width=args.multiview_width,
        pane_height=args.multiview_height,
        update_rate_hz=args.multiview_rate,
    )


def _sync_viewer(viewer, multiview: Optional[ViewerCameraOverlay]) -> None:
    if multiview is not None:
        multiview.sync()
    viewer.sync()


def _parse_camera_list(value: str) -> tuple[str, ...]:
    cameras = tuple(item.strip() for item in value.split(",") if item.strip())
    if not cameras:
        raise RuntimeError("--multiview-cameras must contain at least one camera name")
    return cameras


def _camera_names(env) -> tuple[str, ...]:
    names = []
    for index in range(env.model.ncam):
        name = env.mujoco.mj_id2name(
            env.model, env.mujoco.mjtObj.mjOBJ_CAMERA, index
        )
        if name:
            names.append(name)
    return tuple(names)


def _print_status(command, observation) -> None:
    print(
        f"enabled={int(command.enabled)} "
        f"ee={format_vec(observation.ee_pos)} "
        f"target_ee={format_vec(observation.target_ee_pos)} "
        f"cube={format_vec(observation.cube_pos)} "
        f"gripper={observation.gripper_closedness:0.3f}/"
        f"{observation.target_gripper_closedness:0.3f} "
        f"q={format_vec(observation.joint_pos)} "
        f"ik_error={observation.ik_error_norm:0.4f}"
    )


if __name__ == "__main__":
    main()
