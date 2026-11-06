import argparse, json
import signal
from pathlib import Path


def main(argv=None):
    p = argparse.ArgumentParser(
        description="Dataset replay: offline by default, hardware requires --execute"
    )
    p.add_argument("dataset", type=Path)
    p.add_argument("--episode", type=int, default=0)
    p.add_argument("--mode", choices=["offline", "preview", "robot"], default="offline")
    p.add_argument(
        "--prepare-start",
        action="store_true",
        help="Move slowly to the episode first joint state before replay",
    )
    p.add_argument(
        "--prepare-gripper",
        action="store_true",
        help="Explicitly move gripper to its first measured position after arm preparation",
    )
    p.add_argument(
        "--prepare-timeout",
        type=float,
        help="Arm preparation timeout in seconds (default: homing timeout)",
    )
    p.add_argument("--execute", action="store_true")
    p.add_argument(
        "--plot", type=Path, help="Write offline state/action/time-offset curves"
    )
    p.add_argument("--config")
    p.add_argument("--log", type=Path, default=Path("logs/replay.jsonl"))
    a = p.parse_args(argv)
    if (
        a.prepare_start or a.prepare_gripper or a.prepare_timeout is not None
    ) and a.mode != "robot":
        p.error("Preparation options require --mode robot")
    if a.prepare_gripper and not a.prepare_start:
        p.error("--prepare-gripper requires --prepare-start")
    if a.prepare_timeout is not None and not a.prepare_start:
        p.error("--prepare-timeout requires --prepare-start")
    if a.episode != 0:
        p.error(
            "Each processed output contains one episode; select its output directory"
        )
    from robot_teleop.processing.validation import check_dataset

    result = check_dataset(a.dataset)
    if a.plot:
        from robot_teleop.processing.preview import plot

        plot(a.dataset, a.plot)
    if a.mode == "offline":
        print(json.dumps(result))
        return 0
    if a.mode == "preview":
        from robot_teleop.processing.preview import show

        show(a.dataset)
        return 0
    if not a.execute or not a.config:
        p.error("Robot replay requires --execute and --config")
    cpath = Path(a.config).resolve()
    c = json.loads(cpath.read_text())
    from robot_teleop.backends.real.xarm_sdk.config import XArmConfig
    from robot_teleop.backends.real.xarm_sdk.replay import XArmReplay
    from robot_teleop.replay.runner import load_reference, run

    states, mode = load_reference(a.dataset)
    manifest = json.loads((a.dataset / "processing_manifest.json").read_text())
    if manifest["config"]["robot"] != "xarm6":
        raise ValueError("Only xArm6 hardware adapter implemented")
    config = XArmConfig.load(cpath.parent / c["robot_config"])
    config.require_live_settings()
    if (
        config.gripper["open_pulse"] != manifest["config"]["gripper_open"]
        or config.gripper["close_pulse"] != manifest["config"]["gripper_closed"]
    ):
        raise ValueError("Dataset gripper mapping differs from hardware configuration")
    recorded = manifest.get("raw_configuration", {}).get("robot", {}).get("site", {})
    for key in ("expected_tcp_offset_mm_rad", "expected_payload_kg_com_mm"):
        if recorded.get(key) != config.site[key]:
            raise ValueError("Replay tool calibration differs: " + key)
    a.log.parent.mkdir(parents=True, exist_ok=True)
    with a.log.open("x") as log:
        old_term = signal.getsignal(signal.SIGTERM)

        def terminate(*args):
            raise KeyboardInterrupt

        signal.signal(signal.SIGTERM, terminate)
        try:
            log.write(json.dumps(dict(event="replay_start", config=c)) + "\n")
            run(
                states,
                mode,
                XArmReplay(
                    config,
                    tracking_limits=c.get("tracking_limits"),
                    prepare_start=a.prepare_start,
                    prepare_gripper=a.prepare_gripper,
                    prepare_timeout_s=a.prepare_timeout,
                    log=lambda row: log.write(json.dumps(row) + "\n"),
                ),
                speed=c.get("speed", 1.0),
                hz=c.get("control_hz", 60),
                log=lambda row: log.write(json.dumps(row) + "\n"),
            )
        except BaseException as exc:
            log.write(
                json.dumps(dict(event="replay_end", reason=str(exc), complete=False))
                + "\n"
            )
            raise
        else:
            log.write(json.dumps(dict(event="replay_end", complete=True)) + "\n")
        finally:
            signal.signal(signal.SIGTERM, old_term)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
