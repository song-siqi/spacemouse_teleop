"""Offline validation and versioned exports of native collection episodes.

No camera, robot, training framework, or array library is imported here.
"""
from __future__ import annotations

import csv
import json
import math
import shutil
import tempfile
from pathlib import Path

from .visual import (safe_file as _file, png_size as _png_size, video_reference,
                     validate_video, VisualReader, file_sha256)

FORMAT_VERSION = 1
JOINT_COLUMNS = tuple(f"j{i}_rad" for i in range(1, 7))
TCP_COLUMNS = ("tcp_x_m", "tcp_y_m", "tcp_z_m", "tcp_rx_rad", "tcp_ry_rad", "tcp_rz_rad")
ROLES = ("wrist", "fixed_1", "fixed_2")


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return float(value)


def _vector(value, length, name):
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"{name} must contain {length} values")
    return [_finite(v, name) for v in value]


def _integer(value, name):
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a nonnegative integer")
    return value


def _read_json(path):
    def invalid(value):
        raise ValueError(f"Nonfinite JSON value in {path}: {value}")
    return json.loads(path.read_text(encoding="utf-8"), parse_constant=invalid)


def _rows(path):
    result = []
    with path.open(encoding="utf-8") as stream:
        for number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                value = json.loads(line, parse_constant=lambda v: (_ for _ in ()).throw(ValueError(f"Nonfinite JSON: {v}")))
            except ValueError as exc:
                raise ValueError(f"{path.name}:{number}: {exc}") from exc
            if not isinstance(value, dict):
                raise ValueError(f"{path.name}:{number}: expected an object")
            result.append(value)
    return result


def _load_episode(path, *, require_kept=False, _archive=False, _staged=False, _runner=None, _progress=None):
    root = Path(path).resolve()
    meta = _read_json(root / "meta.json")
    if not isinstance(meta, dict) or meta.get("format_version") not in (1, 2):
        raise ValueError("Unsupported native episode format_version")
    if not _archive and meta.get("status") != "complete":
        raise ValueError("Episode is not complete")
    if not _archive and (meta.get("disposition") == "discarded" or require_kept and meta.get("disposition") != "kept"):
        raise ValueError("Only kept episodes can be exported; discarded episodes are invalid")
    fps = _finite(meta.get("sample_hz"), "sample_hz")
    if fps <= 0:
        raise ValueError("sample_hz must be positive")
    if not isinstance(meta.get("task"), str) or not meta["task"].strip():
        raise ValueError("Episode task must be nonempty")
    start = _finite(meta.get("start_time"), "start_time")
    observations = _rows(root / "observations.jsonl")
    native = _rows(root / "raw_states.jsonl")
    if not observations or _integer(meta.get("frame_count"), "frame_count") != len(observations):
        raise ValueError("Episode frame_count does not match nonempty observations")
    quality = meta.get("quality", {})
    for key, limit in quality.items():
        if key in ("max_camera_age_s", "max_robot_age_s", "max_gripper_age_s", "max_camera_skew_s") and _finite(limit, key) <= 0:
            raise ValueError(f"{key} must be positive")
    records, last_received = {}, {}
    for row in native:
        kind = row.get("kind")
        if kind not in ("joints", "tcp", "gripper", "report"):
            raise ValueError(f"Unknown native state kind: {kind}")
        key = kind, _integer(row.get("sequence"), "native sequence")
        received = _finite(row.get("received_at"), "native received_at")
        if key in records:
            raise ValueError(f"Duplicate native state reference: {key}")
        if received < last_received.get(kind, -math.inf):
            raise ValueError(f"Out-of-order native {kind} timestamps")
        if "query_started_at" in row or "query_finished_at" in row:
            begun = _finite(row.get("query_started_at"), "query_started_at")
            finished = _finite(row.get("query_finished_at"), "query_finished_at")
            if not begun <= finished <= received + 1e-6:
                raise ValueError(f"Invalid {kind} query interval")
        records[key], last_received[kind] = row, received
    video_mode = meta['format_version'] == 2
    if video_mode and meta.get('visual_storage', {}).get('mode') != 'video_after_episode':
        raise ValueError('Version 2 requires video_after_episode visual storage')
    if video_mode and not _archive and meta.get('capture_complete') is not True:
        raise ValueError('Video episode lacks successful capture completion')
    checked_images = {}
    video_specs = {}
    source_repeats = {k: 0 for k in ("joints", "tcp", "gripper", *ROLES)}
    last_source = {}
    max_age = {k: 0. for k in source_repeats}
    for index, row in enumerate(observations):
        if row.get("valid") is not True:
            raise ValueError(f"Invalid observation {index}")
        if _integer(row.get("index"), "observation index") != index:
            raise ValueError("Observation index gap or reordering; time cannot be compressed")
        timestamp = _finite(row.get("timestamp"), "observation timestamp")
        relative = _finite(row.get("relative_time_s"), "relative_time_s")
        if abs(relative - index / fps) > 1e-6 or abs(timestamp - start - relative) > 1e-6:
            raise ValueError("Observation timestamps are not a continuous fixed grid")
        joints = _vector(row.get("joints_rad"), 6, "joints_rad")
        tcp = _vector(row.get("tcp_mm_rad"), 6, "tcp_mm_rad")
        grip = row.get("gripper", {})
        position = _finite(grip.get("position_pulse"), "gripper.position_pulse")
        sources = row.get("sources", {})
        if not all(kind in sources for kind in ("joints", "tcp", "gripper")):
            raise ValueError(f"Missing required source references at observation {index}")
        for kind, ref in sources.items():
            key = kind, _integer(ref.get("sequence"), "source sequence")
            actual = records.get(key)
            if actual is None or actual.get("valid") is not True:
                raise ValueError(f"Missing or invalid native source: {key}")
            received = _finite(ref.get("received_at"), "source received_at")
            age = _finite(ref.get("age_s"), "source age_s")
            if abs(received - actual["received_at"]) > 1e-6 or age < -1e-6 or abs(timestamp - received - age) > 1e-6:
                raise ValueError(f"Source time/age mismatch: {key}")
            expected = {"joints": ("joints_rad", joints), "tcp": ("tcp_mm_rad", tcp), "gripper": ("position_pulse", position)}.get(kind)
            if expected and actual.get("values", {}).get(expected[0]) != expected[1]:
                raise ValueError(f"Observation differs from native source values: {key}")
            if kind == "gripper" and actual.get("values") != grip:
                raise ValueError("Gripper observation differs from native source feedback")
            limit_key = "max_gripper_age_s" if kind == "gripper" else "max_robot_age_s"
            if kind in ("joints", "tcp", "gripper") and age > quality.get(limit_key, math.inf) + 1e-6:
                raise ValueError(f"Source exceeds recorded age limit: {kind}")
            if kind in source_repeats:
                source_repeats[kind] += int(last_source.get(kind) == key)
                last_source[kind], max_age[kind] = key, max(max_age[kind], age)
        images = row.get("images", {})
        if not all(role in images for role in ROLES):
            raise ValueError(f"Missing required camera role at observation {index}")
        camera_times = []
        for role in ROLES:
            frame = images[role]
            if _runner: _runner.check()
            if video_mode:
                video_reference(frame, role, index, fps)
                spec = (frame.get('width'), frame.get('height'))
                if role in video_specs and video_specs[role] != spec:
                    raise ValueError(f'Video dimensions changed within episode: {role}')
                video_specs[role] = spec
            if not video_mode or _staged:
                relative = f'_staging/images/{role}/{index:06d}.png' if _staged else frame.get('file')
                image_path = _file(root, relative)
                if image_path not in checked_images:
                    checked_images[image_path] = _png_size(image_path)
                if checked_images[image_path] != (frame.get("width"), frame.get("height")):
                    raise ValueError(f"Image dimensions differ from metadata: {frame['file']}")
            if frame.get("role") != role or not frame.get("serial"):
                raise ValueError(f"Camera role/serial missing: {role}")
            received = _finite(frame.get("received_at"), "image received_at")
            age = _finite(frame.get("age_s"), "image age_s")
            if age < -1e-6 or abs(timestamp - received - age) > 1e-6:
                raise ValueError(f"Image source time/age mismatch: {role}")
            if age > quality.get("max_camera_age_s", math.inf) + 1e-6:
                raise ValueError(f"Image exceeds recorded age limit: {role}")
            camera_times.append(received)
            key = (frame["serial"], _integer(frame.get("sequence"), "image sequence"))
            source_repeats[role] += int(last_source.get(role) == key)
            last_source[role], max_age[role] = key, max(max_age[role], age)
        skew = max(camera_times) - min(camera_times)
        if abs(_finite(row.get("camera_skew_s"), "camera_skew_s") - skew) > 1e-6 or skew > quality.get("max_camera_skew_s", math.inf) + 1e-6:
            raise ValueError("Camera skew differs from sources or exceeds recorded limit")
    video_checks = {}
    if video_mode and not _staged:
        for name in ('observations.jsonl','raw_states.jsonl'):
            if meta.get('capture_sha256', {}).get(name) != file_sha256(root/name, _runner):
                raise ValueError(f'Capture JSON checksum mismatch: {name}')
        for role, (width, height) in video_specs.items():
            if _progress: _progress(role, 0)
            video_checks[role] = validate_video(_file(root, f'videos/{role}.mp4'), len(observations), fps,
                                                width, height, runner=_runner,
                                                progress=(lambda n, role=role: _progress(role,n)) if _progress else None)
            if meta.get('archive', {}).get('videos', {}).get(role, {}).get('sha256') != video_checks[role]['sha256']:
                raise ValueError(f'Archived video checksum mismatch: {role}')
    summary = {"valid": True, "errors": [], "warnings": [], "sample_count": len(observations),
               "sample_hz": fps, "raw_state_count": len(native), "image_count": len(observations)*len(ROLES) if video_mode else len(checked_images),
               "video_file_count": len(video_checks), "video_checks": video_checks,
               "source_repeat_counts": source_repeats, "max_source_age_s": max_age,
               "status": meta["status"], "disposition": meta.get("disposition")}
    if any(source_repeats.values()):
        summary["warnings"].append("Repeated source samples exist; inspect counts and age limits before training")
    return root, meta, observations, summary


def check_episode(path, *, require_kept=False):
    """Return a machine-readable report. This function never changes an episode."""
    try:
        return _load_episode(path, require_kept=require_kept)[3]
    except (OSError, ValueError, TypeError, KeyError, AttributeError) as exc:
        return {"valid": False, "errors": [str(exc)], "warnings": []}


def _multiply(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return (w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j, w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v)


def _rpy_quaternion(rpy):
    roll, pitch, yaw = (v / 2 for v in rpy)
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return (cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy)


def rotation_delta_base(current_rpy, next_rpy):
    """SO(3) log(R_next R_current^T), base-frame rotation vector in radians.

    Input uses R = Rz(yaw) Ry(pitch) Rx(roll). The principal rotation is chosen;
    half-turn axis sign is inherently ambiguous, and no winding is inferred.
    """
    before = _rpy_quaternion(current_rpy)
    after = _rpy_quaternion(next_rpy)
    q = _multiply(after, (before[0], -before[1], -before[2], -before[3]))
    norm = math.sqrt(sum(v*v for v in q))
    q = tuple(v / norm * (-1 if q[0] < 0 else 1) for v in q)
    sine = math.sqrt(sum(v*v for v in q[1:]))
    scale = 2. if sine < 1e-12 else 2 * math.atan2(sine, q[0]) / sine
    return [scale * v for v in q[1:]]


def _write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def _write_rows(path, rows):
    with path.open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")


def _state(row):
    return list(row["joints_rad"]) + [row["gripper"]["position_pulse"]]


def export_episode(path, output, *, format="legacy", fps=20, fixed_view=None,
                   gripper_open=None, gripper_close=None, binary_threshold=.5):
    """Export one complete kept episode to a new directory, preserving native data.

    Legacy layout is an episode directory; place it under TASK/EPISODE for the
    external converter's ``raw_root/*/*/meta.json`` discovery convention.
    Derived mappings require explicit raw gripper calibration endpoints.
    """
    root, meta, rows, validation = _load_episode(path, require_kept=True)
    if format not in ("legacy", "light", "derived"):
        raise ValueError("format must be legacy, light, or derived")
    if fps not in (10, 20) or meta["sample_hz"] not in (10, 20) or fps > meta["sample_hz"]:
        raise ValueError("Only native 20/10 Hz and explicit 20-to-10 Hz downsampling are supported")
    if format in ("legacy", "light") and fixed_view not in ("fixed_1", "fixed_2"):
        raise ValueError("Choose fixed_view=fixed_1 or fixed_2 for a two-view export")
    stride = int(meta["sample_hz"] / fps)
    selected = rows[::stride]
    if len(selected) < 2:
        raise ValueError("At least two selected samples are needed; no final action is fabricated")
    mapping = None
    if format == "derived":
        opened, closed = _finite(gripper_open, "gripper_open"), _finite(gripper_close, "gripper_close")
        threshold = _finite(binary_threshold, "binary_threshold")
        if opened <= closed or not 0 <= threshold <= 1:
            raise ValueError("Require gripper_open > gripper_close and binary_threshold in [0, 1]")
        mapping = {"open_pulse": opened, "close_pulse": closed, "binary_open_threshold_normalized": threshold,
                   "normalized": "clip((raw-close)/(open-close),0,1)", "binary": "1=open when normalized>=threshold; 0=closed"}
    target = Path(output).absolute()
    if target.is_symlink():
        raise ValueError("Output cannot be a symlink")
    resolved = target.resolve()
    if resolved == root or resolved.is_relative_to(root) or root.is_relative_to(resolved):
        raise ValueError("Export output must be separate from the native episode")
    if target.exists() and (not target.is_dir() or any(target.iterdir())):
        raise ValueError("Refusing to overwrite nonempty output")
    target.parent.mkdir(parents=True, exist_ok=True)
    export_meta = {"format": f"spacemouse_{format}_v1", "format_version": 1, "task": meta["task"],
                   "source_episode": str(root), "source_format_version": meta["format_version"],
                   "source_sample_hz": meta["sample_hz"], "fps": fps, "sample_hz": fps, "save_hz": fps,
                   "source_indices": [row["index"] for row in selected], "observation_count": len(selected),
                   "training_pair_count": len(selected)-1, "status": "complete", "disposition": "kept",
                   "time_alignment": "select original fixed-grid samples; no interpolation or gap compression",
                   "state_columns": list(JOINT_COLUMNS) + ["gripper_mm"],
                   "state_units": ["rad"] * 6 + ["raw pulse"], "gripper_mapping": mapping,
                   "camera_mapping": {"realsense_0": fixed_view, "realsense_1": "wrist"} if fixed_view else None,
                   "action_definition": "actions[t] = state[t+1] after downsampling; terminal observation has no action",
                   "native_validation": validation, "source_metadata": meta}
    if fixed_view:
        export_meta["cameras"] = [
            {"name": camera, "role": role, "kind": "realsense", **{
                key: selected[0]["images"][role][key] for key in ("serial", "width", "height", "capture_fps")},
             "saved_fps": fps}
            for camera, role in (("realsense_0", fixed_view), ("realsense_1", "wrist"))]
    if format == "legacy":
        export_meta["legacy_unit_mapping"] = {"gripper_mm": "raw pulse, not geometric millimeters",
                                               "tcp_x_m,tcp_y_m,tcp_z_m": "millimeters despite historical names",
                                               "tcp_rx_rad,tcp_ry_rad,tcp_rz_rad": "radians; Rz(yaw) Ry(pitch) Rx(roll)"}
        export_meta["external_converter"] = {"layout": "RAW_ROOT/TASK/EPISODE", "fps": fps,
                                             "note": "Pass the matching fps explicitly to the external LeRobot converter"}
    if format == "derived":
        export_meta["action_definition"] = "measured state differences over dt_s, not commanded control intent"
        export_meta["joint_delta_convention"] = "principal atan2(sin(q_next-q),cos(q_next-q)); assumes each joint moves less than pi per sample"
        export_meta["tcp_rotation_convention"] = "base-frame SO(3) rotation vector log(R_next R_current^T); R=Rz(yaw)Ry(pitch)Rx(roll)"
    with tempfile.TemporaryDirectory(prefix=f".{target.name}-", dir=target.parent) as temporary:
        staging = Path(temporary) / "export"
        staging.mkdir()
        copied = []
        if format in ("legacy", "light"):
            with VisualReader(root) as reader:
                for index, row in enumerate(selected):
                    paths = {}
                    for camera, role in (("realsense_0", fixed_view), ("realsense_1", "wrist")):
                        relative = f"{camera}/{index:06d}.png"
                        destination = staging / relative
                        destination.parent.mkdir(exist_ok=True)
                        reader.export_png(row["images"][role], destination)
                        paths[camera] = relative
                    copied.append(paths)
        if format == "legacy":
            columns = ("ts", *JOINT_COLUMNS, *TCP_COLUMNS, "gripper_mm", "realsense_0_file", "realsense_1_file")
            with (staging / "robot_log.csv").open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=columns)
                writer.writeheader()
                for row, files in zip(selected, copied):
                    item = dict(zip(JOINT_COLUMNS, row["joints_rad"]))
                    item.update(zip(TCP_COLUMNS, row["tcp_mm_rad"]))
                    item.update(ts=row["relative_time_s"], gripper_mm=row["gripper"]["position_pulse"],
                                realsense_0_file=files["realsense_0"], realsense_1_file=files["realsense_1"])
                    writer.writerow(item)
        else:
            converted = []
            for index, (before, after) in enumerate(zip(selected, selected[1:])):
                row = {"episode_index": 0, "frame_index": index, "timestamp": before["relative_time_s"],
                       "source_index": before["index"], "next_source_index": after["index"],
                       "dt_s": after["relative_time_s"]-before["relative_time_s"], "task": meta["task"]}
                if format == "light":
                    row.update(image=copied[index]["realsense_0"], wrist_image=copied[index]["realsense_1"],
                               state=_state(before), actions=_state(after), prompt=meta["task"])
                else:
                    current, future = before["gripper"]["position_pulse"], after["gripper"]["position_pulse"]
                    normalize = lambda value: max(0., min(1., (value-closed)/(opened-closed)))
                    n0, n1 = normalize(current), normalize(future)
                    row.update(joints_rad=before["joints_rad"], next_joints_rad=after["joints_rad"],
                               delta_joint_pose=[math.atan2(math.sin(b-a), math.cos(b-a)) for a, b in zip(before["joints_rad"], after["joints_rad"])],
                               tcp_mm_rad=before["tcp_mm_rad"], next_tcp_mm_rad=after["tcp_mm_rad"],
                               delta_tcp_translation_base_m=[(b-a)/1000 for a,b in zip(before["tcp_mm_rad"][:3],after["tcp_mm_rad"][:3])],
                               delta_tcp_rotation_base_rad=rotation_delta_base(before["tcp_mm_rad"][3:],after["tcp_mm_rad"][3:]),
                               gripper={"absolute_raw": current, "next_absolute_raw": future, "delta_raw": future-current,
                                        "normalized": n0, "next_normalized": n1,
                                        "binary_open": int(n0>=threshold), "next_binary_open": int(n1>=threshold)})
                converted.append(row)
            _write_rows(staging / ("train.jsonl" if format == "light" else "derived.jsonl"), converted)
        _write_json(staging / "meta.json", export_meta)
        if target.exists():
            if not target.is_dir() or any(target.iterdir()) or target.is_symlink():
                raise ValueError("Output became nonempty while exporting; refusing overwrite")
            target.rmdir()
        staging.rename(target)
    return export_meta
