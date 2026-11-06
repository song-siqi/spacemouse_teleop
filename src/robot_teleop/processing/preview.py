"""Offline aligned-video and state viewer. No robot imports."""


def show(path):
    import cv2, numpy as np
    from pathlib import Path
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    ds = LeRobotDataset(
        "local/teleop", root=Path(path) / "lerobot", video_backend="pyav"
    )
    i = 0
    paused = False
    last_shown = 0
    cv2.namedWindow("Aligned dataset: Space pause, A/D step, Q quit")
    cv2.createTrackbar(
        "frame",
        "Aligned dataset: Space pause, A/D step, Q quit",
        0,
        max(1, len(ds) - 1),
        lambda x: None,
    )
    try:
        while i < len(ds):
            requested = cv2.getTrackbarPos(
                "frame", "Aligned dataset: Space pause, A/D step, Q quit"
            )
            if requested != last_shown:
                i = requested
                paused = True
            cv2.setTrackbarPos(
                "frame", "Aligned dataset: Space pause, A/D step, Q quit", i
            )
            last_shown = i
            row = ds[i]
            images = [
                (row[k].numpy().transpose(1, 2, 0) * 255).astype("uint8")[:, :, ::-1]
                for k in row
                if k.startswith("observation.images.")
            ]
            canvas = np.concatenate(images, axis=1)
            cv2.putText(
                canvas,
                f"frame {i} state {row['observation.state'].numpy().round(2)}",
                (8, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
            )
            cv2.putText(
                canvas,
                f"action {row['action'].numpy().round(3)}",
                (8, 48),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
            )
            cv2.imshow("Aligned dataset: Space pause, A/D step, Q quit", canvas)
            key = cv2.waitKey(30 if paused else max(1, int(1000 / ds.fps))) & 255
            if key in (ord("q"), 27):
                break
            if key == 32:
                paused = not paused
            if key == ord("a"):
                i = max(0, i - 1)
                paused = True
            elif key == ord("d"):
                i = min(len(ds) - 1, i + 1)
                paused = True
            elif not paused:
                i += 1
    finally:
        cv2.destroyAllWindows()


def plot(path, output):
    """Standalone diagnostic curves, no GUI or hardware required."""
    import json
    from pathlib import Path
    import numpy as np
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    root = Path(path)
    states = json.loads((root / "reference_trajectory.json").read_text())
    refs = [
        json.loads(l) for l in (root / "source_refs.jsonl").read_text().splitlines()
    ]
    action = np.load(root / "actions.npy", allow_pickle=False)
    t = np.array([s["time"] for s in states])
    t = t - t[0]
    fig, axes = plt.subplots(4, 1, figsize=(12, 11), sharex=True)
    axes[0].plot(t, np.array([s["joints"] for s in states]))
    axes[0].set_ylabel("Joint rad")
    axes[1].plot(t, np.array([s["position"] for s in states]))
    axes[1].set_ylabel("TCP m")
    axes[2].plot(t[:-1], action)
    axes[2].set_ylabel("Action (mode units)")
    for role in refs[0]["images"]:
        axes[3].plot(
            t[:-1], [r["images"][role]["error_s"] * 1000 for r in refs], label=role
        )
    axes[3].set_ylabel("Image offset ms")
    axes[3].set_xlabel("Episode seconds")
    axes[3].legend()
    fig.tight_layout()
    fig.savefig(output)
    plt.close(fig)
