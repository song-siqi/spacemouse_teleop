from pathlib import Path
from .raw_v4 import rows


def read(path, meta):
    p = Path(path)
    streams = {}
    for k in ("robot", "gripper"):
        streams[k] = [
            dict(
                source_timestamp=r.get("device_timestamp_raw", 0) / 1e6
                if k == "robot"
                else r["received_at"],
                host_received_at=r["received_at"],
                values=r,
                valid=r["valid"],
                sequence=r["sequence"],
                clock_epoch=r.get("clock_epoch", 0),
            )
            for r in rows(p / f"{k}_states.jsonl")
        ]
    cameras = {}
    for c in meta["cameras"]:
        role = c["role"]
        cameras[role] = dict(c, video_path=str(p / "videos" / f"{role}.mp4"))
        streams[role] = [
            dict(
                source_timestamp=r["device_timestamp_ms"] / 1000,
                host_received_at=r["received_at"],
                values=r,
                valid=True,
                sequence=r["sequence"],
                clock_epoch=r.get("clock_epoch", 0),
                video_frame_index=r["video_frame_index"],
            )
            for r in rows(p / "frames" / f"{role}.jsonl")
        ]
    return streams, cameras
