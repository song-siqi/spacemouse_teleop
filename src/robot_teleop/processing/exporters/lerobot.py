"""Optional official LeRobot writer; imported only in the processing environment."""

import numpy as np


def state_vector(s, mode):
    return np.asarray(
        [*(s.joints if "joint" in mode else s.position + s.quaternion), s.gripper],
        np.float32,
    )


def export(root, states, refs, cameras, values, c, task):
    import cv2
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.configs.video import RGBEncoderConfig

    names = (
        c.get("joint_names", [f"joint{i + 1}" for i in range(len(states[0].joints))])
        if "joint" in c["action_mode"]
        else ["x", "y", "z", "qw", "qx", "qy", "qz"]
    )
    anames = (
        c.get("joint_names", [f"joint{i + 1}" for i in range(len(states[0].joints))])
        if "joint" in c["action_mode"]
        else ["x", "y", "z", "rx", "ry", "rz"]
    )
    features = {
        "observation.state": dict(
            dtype="float32", shape=(len(names) + 1,), names=names + ["gripper"]
        ),
        "action": dict(
            dtype="float32", shape=(len(anames) + 1,), names=anames + ["gripper"]
        ),
        "observation.joints": dict(
            dtype="float32",
            shape=(len(states[0].joints),),
            names=c.get(
                "joint_names", [f"joint{i + 1}" for i in range(len(states[0].joints))]
            ),
        ),
        "observation.tcp": dict(
            dtype="float32", shape=(7,), names=["x", "y", "z", "qw", "qx", "qy", "qz"]
        ),
        "observation.gripper_pulse": dict(dtype="float32", shape=(1,), names=["pulse"]),
    }
    for k, v in cameras.items():
        features["observation.images." + k] = dict(
            dtype="video",
            shape=(v["height"], v["width"], 3),
            names=["height", "width", "channels"],
        )
    ds = LeRobotDataset.create(
        "local/teleop",
        root=root,
        fps=c["fps"],
        features=features,
        robot_type=c["robot"],
        video_backend="pyav",
        image_writer_threads=3,
        rgb_encoder=RGBEncoderConfig(vcodec="h264", crf=18, preset="veryfast", g=40),
    )
    caps = {k: cv2.VideoCapture(v["video_path"]) for k, v in cameras.items()}
    positions = {k: -1 for k in caps}
    buffers = {}
    try:
        for i, (s, ref, a) in enumerate(zip(states, refs, values)):
            frame = {
                "observation.state": state_vector(s, c["action_mode"]),
                "action": a,
                "observation.joints": np.array(s.joints, np.float32),
                "observation.tcp": np.array(s.position + s.quaternion, np.float32),
                "observation.gripper_pulse": np.array([s.gripper_pulse], np.float32),
                "task": task,
            }
            for role, cap in caps.items():
                wanted = ref["images"][role]["video_frame_index"]
                if wanted < positions[role]:
                    raise ValueError("Non-monotonic selected frame")
                while positions[role] < wanted:
                    ok, bgr = cap.read()
                    if not ok:
                        raise ValueError("Missing source image")
                    buffers[role] = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                    positions[role] += 1
                frame["observation.images." + role] = buffers[role].copy()
            ds.add_frame(frame)
        ds.save_episode(parallel_encoding=False)
        ds.finalize()
    finally:
        for cap in caps.values():
            cap.release()
        if getattr(ds, "writer", None) is not None and getattr(
            ds.writer, "image_writer", None
        ):
            ds.writer.image_writer.stop()
