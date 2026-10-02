"""Read-only validation of native collection episodes (v1/v2/v3).

No camera, robot, training framework, or array library is imported here.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

from .visual import (safe_file as _file, png_size as _png_size, video_reference,
                     validate_video, file_sha256)

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
    if require_kept and meta.get("disposition") != "kept":
        raise ValueError("Operator has not kept this episode")
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
        summary["warnings"].append("Repeated source samples exist; inspect counts and age limits without changing raw records")
    return root, meta, observations, summary


def check_episode(path, *, require_kept=False):
    """Return a machine-readable report. This function never changes an episode."""
    try:
        meta = _read_json(Path(path) / "meta.json")
        if meta.get("format_version") == 4:
            from .v4 import check_episode as check_v4
            return check_v4(path, require_kept=require_kept)
        if meta.get("format_version") == 3:
            if require_kept and meta.get("disposition") != "kept":
                raise ValueError("Operator has not kept this episode")
            return validate_native_episode(path, meta=meta)
        return _load_episode(path, require_kept=require_kept)[3]
    except (OSError, ValueError, TypeError, KeyError, AttributeError) as exc:
        return {"valid": False, "errors": [str(exc)], "warnings": []}


def iter_source(path, filename):
    """Read raw rows in stored order. Never select, interpolate or derive actions."""
    root = Path(path).resolve()
    source = _file(root, filename)
    with source.open(encoding="utf-8") as handle:
        for number, line in enumerate(handle, 1):
            if not line.strip(): continue
            try:
                row = json.loads(line, parse_constant=lambda v: (_ for _ in ()).throw(ValueError(v)))
                if not isinstance(row, dict): raise ValueError("expected object")
            except ValueError as exc:
                raise ValueError(f"{filename}:{number}: invalid JSON: {exc}") from exc
            yield row

def _ordered(row, previous, source):
    sequence = _integer(row.get("sequence"), f"{source} sequence")
    received = _finite(row.get("received_at"), f"{source} received_at")
    epoch = _integer(row.get("clock_epoch"), f"{source} clock_epoch")
    if not isinstance(row.get("clock_id"), str) or not row["clock_id"]:
        raise ValueError(f"{source} clock identity missing")
    if previous is not None and (received < previous[1] or sequence <= previous[0]):
        raise ValueError(f"{source} sequence/receive time is out of order")
    return sequence, received

def validate_native_episode(path, *, meta=None, check_hashes=True,
                            allow_uncommitted=False, runner=None):
    """Check storage integrity, not synchronization or training suitability."""
    from robot_teleop.backends.real.xarm_sdk.report import decode_report
    root = Path(path).resolve()
    meta = _read_json(root / "meta.json") if meta is None else meta
    if meta.get("format_version") != 3:
        raise ValueError("Expected raw format_version 3")
    if not allow_uncommitted and meta.get("status") != "complete":
        raise ValueError("Raw capture has no successful completion marker")
    if (root / "observations.jsonl").exists():
        raise ValueError("Raw v3 must not contain an observation grid")
    start = _finite(meta.get("capture_requested_start_time"), "capture window start")
    end = _finite(meta.get("capture_requested_end_time"), "capture window end")
    if end < start: raise ValueError("Capture window is backwards")
    def in_window(row):
        if not start <= row["received_at"] <= end:
            raise ValueError("Source lies outside recorded capture window")
    cameras = meta.get("cameras", [])
    if len(cameras) != 3 or {c.get("role") for c in cameras} != set(ROLES):
        raise ValueError("Camera bindings missing")
    expected_files = {"robot_states.jsonl", "robot_reports.bin", "gripper_states.jsonl",
                      "auxiliary_states.jsonl"}
    frames, video_checks = {}, {}
    for spec in cameras:
        role, fps = spec["role"], _finite(spec["fps"], "camera fps")
        if fps <= 0: raise ValueError("Camera fps must be positive")
        filename = f"frames/{role}.jsonl"
        expected_files.update((filename, f"videos/{role}.mp4"))
        count, previous = 0, None
        for row in iter_source(root, filename):
            previous = _ordered(row, previous, role)
            in_window(row)
            if row.get("role") != role or row.get("serial") != spec.get("serial"):
                raise ValueError(f"{role} identity mismatch")
            if (row.get("width"), row.get("height")) != (spec["width"], spec["height"]):
                raise ValueError(f"{role} dimensions mismatch")
            _integer(row.get("frame_id"), "device frame number")
            _finite(row.get("device_timestamp_ms"), "SDK camera timestamp")
            if not isinstance(row.get("timestamp_domain"), str):
                raise ValueError("Camera timestamp domain missing")
            video_reference(row, role, count, fps)
            count += 1
        if not count or meta.get("frame_counts", {}).get(role) != count:
            raise ValueError(f"{role} frame index count mismatch/empty")
        frames[role] = count
        video_checks[role] = validate_video(_file(root, f"videos/{role}.mp4"), count, fps,
                                            spec["width"], spec["height"], runner=runner)
    counts = {"robot": 0, "gripper": 0, "auxiliary": 0}
    invalid = dict.fromkeys(counts, 0)
    offset, previous = 0, None
    with _file(root, "robot_reports.bin").open("rb") as binary:
        for row in iter_source(root, "robot_states.jsonl"):
            previous = _ordered(row, previous, "robot")
            in_window(row)
            if row.get("kind") != "robot" or row.get("packet_file") != "robot_reports.bin":
                raise ValueError("Robot source/packet reference missing")
            if _integer(row.get("packet_offset"), "packet offset") != offset:
                raise ValueError("Robot packet offset gap or overlap")
            size = _integer(row.get("packet_bytes"), "packet bytes")
            if size not in (784, 804): raise ValueError("Unsupported packet length")
            packet = binary.read(size)
            decoded = decode_report(packet)
            if (row.get("values") != decoded["values"] or
                    row.get("device_timestamp_raw") != decoded["device_timestamp_raw"] or
                    row.get("device_timestamp_unit") != "us" or row.get("valid") != decoded["valid"]):
                raise ValueError("Decoded robot record differs from original packet")
            offset += size
            counts["robot"] += 1
            invalid["robot"] += int(not row["valid"])
        if binary.read(1): raise ValueError("Unindexed trailing robot packet bytes")
    for kind, filename in (("gripper", "gripper_states.jsonl"), ("auxiliary", "auxiliary_states.jsonl")):
        previous_by_kind = {}
        for row in iter_source(root, filename):
            key = row.get("kind")
            if key != ("gripper" if kind == "gripper" else "report"):
                raise ValueError("Invalid state stream kind")
            previous_by_kind[key] = _ordered(row, previous_by_kind.get(key), key)
            in_window(row)
            if "query_started_at" in row or "query_finished_at" in row:
                if not (_finite(row.get("query_started_at"), "query start") <=
                        _finite(row.get("query_finished_at"), "query finish") <= row["received_at"]):
                    raise ValueError("Invalid query interval")
            if kind == "gripper" and row.get("valid"):
                _finite(row.get("values", {}).get("position_pulse"), "actual gripper position")
            counts[kind] += 1
            invalid[kind] += int(not row.get("valid"))
    if not counts["robot"] or not counts["gripper"]:
        raise ValueError("Required native robot/gripper streams are empty")
    if meta.get("state_counts") != counts:
        raise ValueError("State count mismatch")
    session = root.parent.parent
    for key, expected in (("session_metadata", "../../session.json"),
                          ("session_log", "../../control.jsonl"),
                          ("session_events", "../../events.jsonl"),
                          ("clock_anchors", "../../clock_anchors.jsonl")):
        if meta.get(key) != expected or not (root / expected).is_file():
            raise ValueError(f"Missing session evidence: {key}")
    anchor_count = 0
    for row in iter_source(session, "clock_anchors.jsonl"):
        before = _integer(row.get("monotonic_before_ns"), "clock bracket start")
        after = _integer(row.get("monotonic_after_ns"), "clock bracket end")
        _integer(row.get("realtime_ns"), "realtime")
        if before > after or row.get("read_interval_ns") != after-before:
            raise ValueError("Invalid host clock bracket")
        anchor_count += 1
    if not anchor_count: raise ValueError("Clock anchors missing")
    if check_hashes:
        session_meta = _read_json(session / "session.json")
        if session_meta.get("format_version") != 3:
            raise ValueError("Wrong session format version")
        if session_meta.get("status") == "incomplete":
            raise ValueError("Session evidence is incomplete")
        if session_meta.get("status") == "closed":
            shared = session_meta.get("files", {})
            if set(shared) != {"control.jsonl", "events.jsonl", "clock_anchors.jsonl"}:
                raise ValueError("Session file manifest incomplete")
            for filename, info in shared.items():
                file = _file(session, filename)
                if file.stat().st_size != info.get("bytes") or file_sha256(file, runner) != info.get("sha256"):
                    raise ValueError(f"Session checksum mismatch: {filename}")
        files = meta.get("files", {})
        if set(files) != expected_files: raise ValueError("Raw file manifest incomplete")
        for filename in expected_files:
            file = _file(root, filename)
            if file.stat().st_size != files[filename].get("bytes") or file_sha256(file, runner) != files[filename].get("sha256"):
                raise ValueError(f"Raw checksum mismatch: {filename}")
    return {"valid": True, "errors": [], "format_version": 3,
            "warnings": ["Cross-device clock mapping is not calibrated; integrity does not establish training quality"],
            "frame_counts": frames, "image_count": sum(frames.values()), "video_file_count": 3,
            "video_checks": video_checks, "state_counts": counts, "invalid_state_counts": invalid,
            "clock_anchor_count": anchor_count, "continuity": meta.get("continuity", {}),
            "status": meta["status"], "disposition": meta.get("disposition")}
