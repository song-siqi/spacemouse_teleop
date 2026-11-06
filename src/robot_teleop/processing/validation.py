import numpy as np
from ..replay.reconstruction import reconstruct
from .actions.cartesian import rotation


def reconstruction_error(states, values, mode):
    restored = reconstruct(states[0], values, mode)
    errors = []
    for a, b in zip(states, restored):
        if "joint" in mode:
            e = float(np.max(abs(np.asarray(a.joints) - b.joints)))
        else:
            e = max(
                float(np.linalg.norm(np.asarray(a.position) - b.position)),
                float((rotation(a) * rotation(b).inv()).magnitude()),
            )
        errors.append(max(e, abs(a.gripper - b.gripper)))
    error = max(errors)
    if error > 1e-5:
        raise ValueError(f"Action reconstruction mismatch: {error}")
    return dict(max_reconstruction_error=error, samples=len(values))


def check_dataset(path, allow_pending=False):
    import json
    from pathlib import Path
    from .contracts import State
    from .exporters.lerobot import state_vector
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    root = Path(path)
    m = json.loads((root / "processing_manifest.json").read_text())
    if not allow_pending and m["status"] != "complete":
        raise ValueError("Incomplete processed dataset")
    from .pipeline import sha

    for name, digest in m.get("output_hashes", {}).items():
        f = (root / name).resolve()
        if not f.is_relative_to(root.resolve()) or sha(f) != digest:
            raise ValueError("Processed output checksum mismatch: " + name)
    states = [
        State(**s) for s in json.loads((root / "reference_trajectory.json").read_text())
    ]
    values = np.load(root / "actions.npy", allow_pickle=False)
    ds = LeRobotDataset("local/teleop", root=root / "lerobot", video_backend="pyav")
    if len(ds) != len(values) or ds.fps != m["config"]["fps"]:
        raise ValueError("Dataset count/fps mismatch")
    numbers = ds.select_columns(["action", "observation.state", "timestamp"])
    for i, (a, s) in enumerate(zip(values, states)):
        row = numbers[i]
        if not np.array_equal(np.asarray(row["action"]), a) or not np.array_equal(
            np.asarray(row["observation.state"]),
            state_vector(s, m["config"]["action_mode"]),
        ):
            raise ValueError("LeRobot numeric readback differs")
        if abs(float(np.asarray(row["timestamp"]).item()) - i / ds.fps) > 1e-4:
            raise ValueError("LeRobot timestamp differs")
    from robot_teleop.recording.visual import validate_video

    for video in (root / "lerobot" / "videos").rglob("*.mp4"):
        feature = next(
            v
            for k, v in ds.features.items()
            if k.startswith("observation.images.") and k in str(video)
        )
        shape = feature["shape"]
        validate_video(video, len(values), ds.fps, shape[1], shape[0])
    refs = [
        json.loads(l) for l in (root / "source_refs.jsonl").read_text().splitlines()
    ]
    if len(refs) != len(values):
        raise ValueError("Source reference count mismatch")
    indices = sorted(
        {0, len(ds) - 1, len(ds) // 2, *range(0, len(ds), max(1, len(ds) // 8))}
    )
    max_mae = 0.0
    source_check = False
    import cv2

    camera_sources = json.loads((root / "alignment_report.json").read_text())[
        "camera_sources"
    ]
    for i in indices:
        row = ds[i]
        for k in row:
            if k.startswith("observation.images."):
                image = np.asarray(row[k])
                if (
                    image.ndim != 3
                    or image.shape[0] != 3
                    or not np.isfinite(image).all()
                ):
                    raise ValueError("Invalid decoded image")
                role = k.split(".")[-1]
                source = camera_sources[role]["video_path"]
                if Path(source).exists():
                    cap = cv2.VideoCapture(source)
                    try:
                        wanted = refs[i]["images"][role]["video_frame_index"]
                        for _ in range(wanted + 1):
                            ok, original = cap.read()
                        if not ok:
                            raise ValueError("Missing source comparison frame")
                        original = cv2.cvtColor(original, cv2.COLOR_BGR2RGB).transpose(
                            2, 0, 1
                        )
                        mae = float(np.mean(abs(image * 255 - original.astype(float))))
                        max_mae = max(mae, max_mae)
                        source_check = True
                        if mae > 20:
                            raise ValueError("Image source/encoding mismatch")
                    finally:
                        cap.release()
    result = reconstruction_error(states, values, m["config"]["action_mode"])
    result.update(
        valid=True,
        decoded_sample_frames=len(indices),
        clock_mapping_calibrated=False,
        max_image_mae_8bit=max_mae,
        source_image_comparison=source_check,
    )
    return result
