"""Scripted manipulation metrics from the generic dynamics backend."""
from dataclasses import asdict
import numpy as np
from robot_teleop.core import CartesianPoseTarget, Pose, GripperCommand, Stamp


def contacts(backend):
    result = {role: {"count": 0, "max_penetration_m": 0., "max_normal_force_n": 0., "force_n": 0., "torque_nm": 0.}
              for role in ("left_pad", "right_pad", "shell", "table")}
    model, data, mj = backend.model, backend.data, backend.mj
    objects = {int(model.jnt_bodyid[j]) for j in range(model.njnt) if model.jnt_type[j] == mj.mjtJoint.mjJNT_FREE}
    for index, c in enumerate(data.contact[:data.ncon]):
        if not any(int(model.geom_bodyid[g]) in objects for g in (c.geom1, c.geom2)):
            continue
        role = None
        for g in (c.geom1, c.geom2):
            name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, g) or ""
            if name in backend.binding.gripper.pads:
                role = "left_pad" if "left" in name else "right_pad"
            elif model.geom_contype[g] == 4:
                role = "shell"
            elif name == "table":
                role = "table"
        if role is not None:
            force = np.zeros(6)
            mj.mj_contactForce(model, data, index, force)
            r = result[role]
            r["count"] += 1
            r["max_penetration_m"] = max(r["max_penetration_m"], -float(c.dist))
            r["max_normal_force_n"] = max(r["max_normal_force_n"], abs(float(force[0])))
            r["force_n"] += float(np.linalg.norm(force[:3]))
            r["torque_nm"] += float(np.linalg.norm(force[3:]))
    return result


def grasp_probe(backend, callback=None):
    backend.reset()
    hz = round(1 / backend.dt)
    rows = []
    def run(phase, position, duration, intent="hold", quaternion=(0., 1., 0., 0.)):
        for _ in range(round(duration * hz)):
            deadline = Stamp(backend.data.time + .25, "simulation", backend.epoch)
            backend.submit(CartesianPoseTarget(Pose(tuple(position), quaternion)), deadline)
            # Do not repeatedly recapture hold: only transition from motion to hold.
            if intent != "hold" or backend.tool_intent != "hold":
                backend.submit(GripperCommand(intent), deadline)
            peaks = {}
            state = backend.advance(on_step=lambda b: _contact_peaks(peaks, b))
            row = {"phase": phase, "state": asdict(state), "objects": {k: asdict(v) for k, v in backend.scene_state().items()}, "contacts": peaks}
            rows.append(row)
            if callback:
                callback(backend, row)
    obj = next(iter(backend.scene_state().values()))
    x, y = obj.position[:2]
    run("approach", (x, y, .88), 3, "open")
    run("descend", (x, y, .755), 3, "open")
    run("close", (x, y, .755), 2, "close")
    before = next(iter(backend.scene_state().values())).position[2]
    start = backend.observe().tcp.position
    run("lift", (start[0], start[1], start[2] + .12), 3, "close")
    lifted = next(iter(backend.scene_state().values())).position[2]
    run("hold", backend.observe().tcp.position, 2, "close")
    relative = [np.asarray(next(iter(r["objects"].values()))["position"]) - r["state"]["tcp"]["position"] for r in rows if r["phase"] == "hold"]
    def relative_quat(row):
        from .math_utils import quat_multiply
        ee = np.asarray(row["state"]["tcp"]["quaternion"])
        return quat_multiply(ee * (1, -1, -1, -1), next(iter(row["objects"].values()))["quaternion"])
    def angle(a, b):
        return float(np.degrees(2*np.arccos(np.clip(abs(np.dot(a, b)), 0, 1))))
    held = [relative_quat(r) for r in rows if r["phase"] == "hold"]
    reference = held[-1]
    from .math_utils import quat_multiply, quat_from_rotvec
    rotated = tuple(quat_multiply(quat_from_rotvec((0, 0, .5)), (0, 1, 0, 0)))
    run("rotate", backend.observe().tcp.position, 3, "close", rotated)
    summary = {"robot": backend.binding.robot, "object_lift_m": lifted-before,
               "hold_relative_slip_m": float(max(np.linalg.norm(p-relative[0]) for p in relative)),
               "hold_orientation_drift_deg": max(angle(held[0], q) for q in held),
               "rotation_relative_drift_deg": max(angle(reference, relative_quat(r)) for r in rows if r["phase"] == "rotate"),
               "max_penetration_m": max(r["contacts"][p]["max_penetration_m"] for r in rows for p in ("left_pad", "right_pad")),
               "max_normal_force_n": max(r["contacts"][p]["max_normal_force_n"] for r in rows for p in ("left_pad", "right_pad"))}
    summary["passed_lift_and_slip"] = bool(summary["object_lift_m"] >= .08 and summary["hold_relative_slip_m"] <= .005)
    loaded = [r for r in rows if r["phase"] in ("lift", "hold", "rotate")]
    summary["pad_contact_fraction"] = {p:sum(r["contacts"][p]["count"] > 0 for r in loaded)/len(loaded) for p in ("left_pad", "right_pad")}
    summary["passed_grasp"] = bool(summary["passed_lift_and_slip"] and summary["hold_orientation_drift_deg"] <= 5 and summary["rotation_relative_drift_deg"] <= 10 and summary["max_penetration_m"] <= .002 and summary["max_normal_force_n"] <= 80 and min(summary["pad_contact_fraction"].values()) >= .8)
    return summary, rows


def _contact_peaks(peaks, backend):
    # Contacts/forces correspond to this solver step. Sampling only at control
    # ticks can miss an impact entirely, particularly with a 30 Hz input loop.
    for role, values in contacts(backend).items():
        previous = peaks.setdefault(role, dict(values))
        for key, value in values.items():
            previous[key] = max(previous[key], value)


def collision_probe(backend, tilt=0.):
    """Top press and lateral push followed by retreat, without pose rollback."""
    hz = round(1 / backend.dt)
    def run(pos, seconds, quaternion=(0., 1., 0., 0.), ramp=False):
        samples = []
        start = np.asarray(backend.observe().tcp.position)
        steps = round(seconds * hz)
        for step in range(steps):
            target = start + (np.asarray(pos)-start)*(step+1)/steps if ramp else pos
            backend.submit(CartesianPoseTarget(Pose(tuple(target), quaternion)), Stamp(backend.data.time+.25, "simulation", backend.epoch))
            peaks = {}
            backend.advance(on_step=lambda b: _contact_peaks(peaks, b))
            samples.append(peaks)
        return samples
    backend.reset()
    obj = next(iter(backend.scene_state().values()))
    x, y = obj.position[:2]
    top_x = x
    top_quat = (0., 1., 0., 0.)
    if tilt:
        from .math_utils import quat_multiply, quat_from_rotvec
        top_quat = tuple(quat_multiply(quat_from_rotvec((tilt,0,0)),top_quat))
    top_y = y + (.055 if backend.binding.robot == "fr3" else 0.)
    # A large initial FR3 wrist reorientation must settle before contact; otherwise
    # the test mixes shell pushing with a continuing gripper yaw maneuver.
    run((top_x, top_y, .9), 10 if backend.binding.robot == "fr3" else 4, top_quat)
    top = run((top_x, top_y, .65), 4, top_quat)
    z = backend.observe().tcp.position[2]
    pressed_tcp = backend.observe().tcp.position
    retreat = run((top_x, top_y, z+.08), 2, top_quat)
    top_summary = {"shell_penetration_m": max(r["shell"]["max_penetration_m"] for r in top),
                   "table_penetration_m": max(r["table"]["max_penetration_m"] for r in top),
                   "shell_contact_count": sum(r["shell"]["count"] for r in top),
                   "pressed_tcp": pressed_tcp,
                   "retreat_m": backend.observe().tcp.position[2]-z,
                   "retreat_contacts": retreat[-1]["shell"]["count"]}
    backend.reset()
    if backend.binding.robot == "fr3":
        backend.submit(GripperCommand("close"), Stamp(30, "simulation", backend.epoch))
    side_z = .755
    run((x, y+.16, .9), 10 if backend.binding.robot == "fr3" else 3)
    run((x, y+.16, side_z), 4)
    start = np.array(next(iter(backend.scene_state().values())).position)
    side = run((x, y-.06, side_z), 4, ramp=True)
    side += run((x, y-.06, side_z), 1)
    end = np.array(next(iter(backend.scene_state().values())).position)
    retreat = run((x, y+.16, side_z), 2, ramp=True)
    after = np.array(next(iter(backend.scene_state().values())).position)
    side_summary = {"displacement_m": float(np.linalg.norm(end-start)),
                    "pad_contact_count": sum(r[p]["count"] for r in side for p in ("left_pad", "right_pad")),
                    "shell_penetration_m": max(r["shell"]["max_penetration_m"] for r in side),
                    "retreat_drag_m": float(np.linalg.norm(after-end)), "retreat_delta_m": tuple(after-end), "retreat_contacts": retreat[-1]["shell"]["count"]}
    return {"top": top_summary, "side": side_summary}
