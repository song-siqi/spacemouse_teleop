import numpy as np
from scipy.spatial.transform import Slerp
from ..contracts import State
from .quality import nearest, bracket


def align(streams, cameras, times, meta, c, *, adapter):
    start, end = meta["start_time"], meta["end_time"]
    grid = start + np.arange(int(np.floor((end - start) * c["fps"])) + 1) / c["fps"]
    if len(grid) < 2:
        raise ValueError("Episode too short")
    rr = streams["robot"]
    qs = []
    ps = []
    rot = []
    for r in rr:
        q, p, R = adapter.robot_values(r)
        qs.append(q)
        ps.append(p)
        rot.append(R)
    qs = np.array(qs)
    ps = np.array(ps)
    states = []
    refs = []
    for t in grid:
        i, j, w = bracket(times["robot"], t, c["max_robot_gap_s"])
        q = (1 - w) * qs[i] + w * qs[j]
        p = (1 - w) * ps[i] + w * ps[j]
        from scipy.spatial.transform import Rotation

        R = Slerp([0, 1], Rotation.concatenate([rot[i], rot[j]]))([w])[0]
        gidx = (
            int(
                np.searchsorted(
                    [r["host_received_at"] for r in streams["gripper"]], t, side="right"
                )
            )
            - 1
        )
        if gidx < 0 or not 0 <= t - times["gripper"][gidx] <= c["max_gripper_age_s"]:
            raise ValueError("Gripper feedback missing/stale")
        g, pulse = adapter.gripper_values(streams["gripper"][gidx], c)
        quat = R.as_quat()
        states.append(
            State(
                float(t), q.tolist(), p.tolist(), quat[[3, 0, 1, 2]].tolist(), g, pulse
            )
        )
        images = {}
        selected = []
        for role in cameras:
            ix = nearest(times[role], t)
            dt = float(times[role][ix] - t)
            if abs(dt) > c["max_image_error_s"]:
                raise ValueError("Image too far from grid: " + role)
            row = streams[role][ix]
            selected.append(times[role][ix])
            images[role] = dict(
                source_index=ix,
                sequence=row["sequence"],
                frame_id=row["values"]["frame_id"],
                video_frame_index=row["video_frame_index"],
                mapped_time=float(times[role][ix]),
                error_s=dt,
            )
        if max(selected) - min(selected) > c["max_camera_skew_s"]:
            raise ValueError("Camera skew exceeds threshold")
        refs.append(
            dict(
                time=float(t),
                robot_indices=[i, j],
                robot_weight=w,
                gripper_index=gidx,
                gripper_query_interval=[
                    streams["gripper"][gidx]["values"][k]
                    for k in ("query_started_at", "query_finished_at")
                ],
                images=images,
            )
        )
    return states, refs
