import json, hashlib, platform
from importlib.metadata import version
from dataclasses import asdict
from robot_teleop.robots import ROBOTS
from pathlib import Path
import numpy as np
from .sources import raw_v4, legacy_xarm_v3, ADAPTERS
from .timing.mapping import map_source
from .timing.resample import align
from .validation import reconstruction_error
from ..replay.reconstruction import actions


def sha(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for b in iter(lambda: f.read(1024 * 1024), b""):
            h.update(b)
    return h.hexdigest()


def process_episode(path, output, c):
    from robot_teleop.recording.raw import check_episode

    p = Path(path).resolve()
    out = Path(output).resolve()
    if out.exists():
        raise ValueError("Output must be new")
    if out == p or p in out.parents:
        raise ValueError("Output cannot be inside raw episode")
    if c["robot"] not in ADAPTERS:
        raise ValueError(
            "Only the native xArm source adapter is registered; action math supports arbitrary joint dimensions"
        )
    report = check_episode(p, require_kept=True)
    if not report["valid"]:
        raise ValueError(report["errors"])
    meta = json.loads((p / "meta.json").read_text())
    if meta["format_version"] not in (3, 4):
        raise ValueError("Only raw v3/v4 supported")
    session = p.parent.parent
    if json.loads((session / "session.json").read_text()).get("status") != "closed":
        raise ValueError("Close the raw session before processing shared evidence")
    paths = [x for x in p.rglob("*") if x.is_file()] + list(session.glob("*.json*"))
    hashes = {str(x): sha(x) for x in paths}
    streams, cameras = (raw_v4 if meta["format_version"] == 4 else legacy_xarm_v3).read(
        p, meta
    )
    if not set(c.get("required_cameras", [])) <= cameras.keys():
        raise ValueError("Missing required camera role")
    if not cameras:
        raise ValueError("At least one RGB source is required")
    if c["fps"] > min(v["fps"] for v in cameras.values()):
        raise ValueError("Output fps exceeds slowest camera capture mode")
    for k in ["robot", "gripper", *cameras]:
        if not streams.get(k) or any(not r["valid"] for r in streams[k]):
            raise ValueError("Invalid required source: " + k)
        if any(
            b["sequence"] != a["sequence"] + 1
            for a, b in zip(streams[k], streams[k][1:])
        ):
            raise ValueError("Source sequence gap")
        if k in cameras and any(
            b["values"]["frame_id"] != a["values"]["frame_id"] + 1
            for a, b in zip(streams[k], streams[k][1:])
        ):
            raise ValueError("Camera device frame gap")
    anchors = raw_v4.rows(session / "clock_anchors.jsonl")
    times = {}
    fits = {}
    for k in ["robot", "gripper", *cameras]:
        times[k], fits[k] = map_source(
            streams[k],
            "camera" if k in cameras else k,
            anchors,
            c.get("source_offsets_s", {}).get(k, 0.0),
        )
    profile = ROBOTS[c["robot"]]
    c = {**c, "joint_names": list(profile.joint_names)}
    states, refs = align(streams, cameras, times, meta, c, adapter=ADAPTERS[c["robot"]])
    profile = ROBOTS[c["robot"]]
    if any(
        len(s.joints) != len(profile.joint_names)
        or any(
            not lo <= q <= hi
            for q, (lo, hi) in zip(s.joints, profile.position_limits or [])
        )
        for s in states
    ):
        raise ValueError("Robot joint layout/limits mismatch")
    values = actions(states, c["action_mode"])
    reconstruction = reconstruction_error(states, values, c["action_mode"])
    staging = out.with_name(out.name + ".partial")
    staging.mkdir(parents=True, exist_ok=False)
    manifest = dict(
        status="processing",
        schema_version=1,
        config=c,
        raw_episode=str(p),
        input_hashes=hashes,
        task=meta.get("task", session.parent.name),
        units=dict(
            position="m",
            joint="rad",
            orientation="wxyz quaternion",
            action_rotation="rotvec rad",
        ),
        frame="link_base",
        action_source="adjacent_measured_states",
        action_reference="per_step",
        terminal_action_fabricated=False,
        robot_definition=asdict(profile),
        python=platform.python_version(),
        raw_configuration=json.loads((session / "session.json").read_text()).get(
            "config"
        ),
        packages={
            name: version(name) for name in ("lerobot", "numpy", "scipy", "av", "torch")
        },
    )
    try:
        (staging / "processing_manifest.json").write_text(
            json.dumps(manifest, indent=2)
        )
        (staging / "reference_trajectory.json").write_text(
            json.dumps([asdict(s) for s in states])
        )
        (staging / "source_refs.jsonl").write_text(
            "".join(json.dumps(x) + "\n" for x in refs[:-1])
        )
        np.save(staging / "actions.npy", values)
        quality = dict(
            clock_mappings=fits,
            reconstruction=reconstruction,
            frames=len(values),
            image_errors={
                k: dict(
                    max_abs_s=max(abs(x["images"][k]["error_s"]) for x in refs),
                    reused=len(refs) - len({x["images"][k]["sequence"] for x in refs}),
                )
                for k in cameras
            },
            camera_sources=cameras,
            motion_statistics=dict(
                max_joint_speed_rad_s=float(
                    np.max(abs(np.diff([s.joints for s in states], axis=0) * c["fps"]))
                ),
                max_joint_accel_rad_s2=float(
                    np.max(
                        abs(
                            np.diff([s.joints for s in states], n=2, axis=0)
                            * c["fps"] ** 2
                        )
                    )
                ),
            )
            if len(states) > 2
            else {},
            physical_synchronization_verified=False,
        )
        (staging / "alignment_report.json").write_text(json.dumps(quality, indent=2))
        from .exporters.lerobot import export

        export(staging / "lerobot", states, refs, cameras, values, c, manifest["task"])
        from .validation import check_dataset

        check = check_dataset(staging, allow_pending=True)
        (staging / "validation_report.json").write_text(json.dumps(check, indent=2))
        if any(sha(x) != h for x, h in hashes.items()):
            raise ValueError("Raw input changed while processing")
        manifest.update(
            status="complete",
            input_unchanged=True,
            output_hashes={
                str(f.relative_to(staging)): sha(f)
                for f in staging.rglob("*")
                if f.is_file() and f.name != "processing_manifest.json"
            },
        )
        (staging / "processing_manifest.json").write_text(
            json.dumps(manifest, indent=2)
        )
        staging.rename(out)
        return dict(output=str(out), frames=len(values), validation=check)
    except BaseException as exc:
        manifest.update(status="incomplete", error=str(exc))
        (staging / "processing_manifest.json").write_text(
            json.dumps(manifest, indent=2)
        )
        raise
