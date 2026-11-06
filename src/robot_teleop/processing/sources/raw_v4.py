import json
from pathlib import Path


def rows(path):
    return [json.loads(l) for l in Path(path).read_text().splitlines() if l.strip()]


def read(path, meta):
    sources = {s["id"]: s for s in meta["sources"]}
    streams = {k: rows(Path(path) / s["path"]) for k, s in sources.items()}
    cameras = {
        k: dict(s["video"], video_path=str(Path(path) / s["video_path"]))
        for k, s in sources.items()
        if s["type"] == "rgb"
    }
    return streams, cameras
