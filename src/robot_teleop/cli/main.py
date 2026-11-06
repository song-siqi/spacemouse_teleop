"""Unified, lazy-importing command line."""
from __future__ import annotations
import argparse
from dataclasses import asdict
import importlib
import json
from pathlib import Path
import sys


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if "--config" in argv and "--backend" not in argv:
        i = argv.index("--config")
        if i+1 < len(argv):
            path = Path(argv[i+1])
            raw = json.loads(path.read_text())
            if raw.get("schema_version") == 1 and "backend" in raw:
                from robot_teleop.runtime.experiment import load_experiment
                experiment = load_experiment(path)
                if experiment["backend"]["name"] == "xarm_sdk":
                    section = experiment["recording"] if argv[0] == "collect" else experiment["backend"]
                    argv[i+1] = section["config"]
                    argv.extend(("--backend", "xarm_sdk"))
    # Preserve mature real-robot lifecycle checks behind the unified entry point.
    dispatch = {"process": "process", "dataset-check": "dataset_check", "replay": "replay", "move-to-pose": "xarm_move_to_pose", "check": "collection_check", "finalize": "collection_finalize"}
    if argv and argv[0] in dispatch:
        return importlib.import_module(f"robot_teleop.cli.{dispatch[argv[0]]}").main(argv[1:])
    if "--backend" in argv:
        i = argv.index("--backend")
        if i + 1 < len(argv) and argv[i+1] == "xarm_sdk":
            del argv[i:i+2]
            if "--input" in argv:
                argv[argv.index("--input")] = "--backend"
            modules = {"teleop": "xarm_teleop", "collect": "xarm_collect", "diagnose": "xarm_diagnose"}
            if "--profile" in argv:
                i = argv.index("--profile")
                if argv[i+1] not in ("translation", "demo") or argv[0] != "teleop":
                    raise ValueError("Only demo and translation teleop profiles are supported")
                profile = argv[i+1]
                del argv[i:i+2]
                modules["teleop"] = "xarm_" + profile
            return importlib.import_module(f"robot_teleop.cli.{modules[argv[0]]}").main(argv[1:])
    parser = argparse.ArgumentParser(prog="robot-teleop")
    parser.add_argument("command", choices=("teleop", "collect", "diagnose", "move-to-pose", "check", "finalize", "process", "dataset-check", "replay"))
    parser.add_argument("--config", default="configs/xarm_sim.json")
    parser.add_argument("--duration", type=float, default=3.)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--render", action="store_true")
    parser.add_argument("--collision-debug", action="store_true")
    parser.add_argument("--scenario", choices=("grasp", "collision"), default="grasp")
    parser.add_argument("--input", choices=("scripted", "keyboard", "spacemouse"))
    parser.add_argument("--viewer", action="store_true", help="Interactive MuJoCo view; use mjpython on macOS")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    from robot_teleop.runtime.experiment import load_experiment, make_backend
    config = load_experiment(args.config)
    if args.dry_run:
        print(json.dumps(config, indent=2))
        return 0
    backend = make_backend(config)
    if args.collision_debug:
        backend.set_collision_debug(True)
    recorder = None
    source = viewer = None
    try:
        if args.command == "diagnose":
            from robot_teleop.backends.sim.mujoco.diagnostics import grasp_probe, collision_probe
            if args.scenario == "grasp":
                summary, rows = grasp_probe(backend)
                passed = summary["passed_grasp"]
            else:
                summary, rows = collision_probe(backend), []
                top, side = summary["top"], summary["side"]
                passed = (top["shell_contact_count"] > 0 and top["shell_penetration_m"] <= .0005
                          and top["table_penetration_m"] <= .002 and top["retreat_contacts"] == 0
                          and side["displacement_m"] >= .04 and side["shell_penetration_m"] <= .0025
                          and side["retreat_drag_m"] <= .005 and side["retreat_contacts"] == 0)
                summary["passed_collision"] = bool(passed)
            print(json.dumps(summary, indent=2))
            if args.output:
                args.output.mkdir(parents=True, exist_ok=True)
                (args.output / "summary.json").write_text(json.dumps(summary, indent=2))
                with (args.output / "grasp.jsonl").open("w") as handle:
                    for row in rows:
                        handle.write(json.dumps(row) + "\n")
            result = 0 if passed else 1
        else:
            import math
            from robot_teleop.core import Stamp
            from robot_teleop.inputs.scripted import ScriptedInput
            if not math.isfinite(args.duration) or args.duration <= 0:
                raise ValueError("duration must be positive")
            input_name = args.input or config["input"]["name"]
            if input_name == "scripted":
                source = ScriptedInput(config["input"].get("segments", []))
            elif input_name == "keyboard":
                from robot_teleop.inputs.keyboard import KeyboardInput
                source = KeyboardInput()
                source.open()
            elif input_name == "spacemouse":
                from robot_teleop.inputs.spacemouse.adapter import SpaceMouseInput
                source = SpaceMouseInput(config["input"])
                source.open()
            else:
                raise ValueError(f"Unsupported input: {input_name}")
            if args.viewer:
                import mujoco.viewer
                viewer = mujoco.viewer.launch_passive(backend.model, backend.data)
            cameras = config["recording"].get("cameras", [])
            if args.command == "collect":
                from robot_teleop.recording.v4 import RawEpisode, Source
                specs = [Source("robot", "robot_state", "robot-state/v1", {"position": "m", "joints": "rad"}, "world", "simulation"),
                         Source("commands", "command", "command/v1", {"linear": "m/s", "angular": "rad/s"}, "world", "simulation")]
                specs += [Source(c["id"], "rgb", "rgb/v1", {}, c.get("frame", "world"), "simulation",
                                 video={k: c[k] for k in ("width", "height", "fps")}) for c in cameras]
                recorder = RawEpisode(args.output or Path("recordings") / f"{backend.binding.robot}-episode", specs,
                                      epoch=backend.epoch, metadata={"experiment": config})
            from robot_teleop.runtime.simulation import run
            run(backend,source,args.duration,recorder=recorder,cameras=cameras,viewer=viewer,
                realtime=bool(viewer) or input_name != "scripted")
            print(json.dumps(asdict(backend.observe()), indent=2))
            result = 0
            if recorder:
                recorder.close()
                from robot_teleop.recording.v4 import check_episode
                report = check_episode(recorder.path)
                print(json.dumps(report))
                result = 0 if report["valid"] else 1
        if args.render:
            import cv2
            output = args.output or Path(".generated") / backend.binding.robot
            output.mkdir(parents=True, exist_ok=True)
            for camera in ("overview", "side"):
                image = backend.render(camera)
                if image.std() < 1:
                    raise RuntimeError("Blank rendered view")
                cv2.imwrite(str(output / f"{camera}.png"), image[:, :, ::-1])
        return result
    finally:
        backend.close()
        if source and hasattr(source, "close"):
            source.close()
        if viewer:
            viewer.close()
        if recorder and not recorder.closed:
            recorder.close(complete=False)


if __name__ == "__main__":
    raise SystemExit(main())
