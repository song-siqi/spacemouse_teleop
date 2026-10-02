"""Independent control and camera schedules on the simulation timeline."""
from dataclasses import asdict
import math
import time
from robot_teleop.core import Stamp


def run(backend, source, duration, *, recorder=None, cameras=(), viewer=None, realtime=False):
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError("Duration must be finite and positive")
    start = backend.scheduled_time
    logical = start
    wall_start = time.monotonic()
    tick = 0
    sequences = {c["id"]: 0 for c in cameras}
    for c in cameras:
        if not math.isfinite(c["fps"]) or c["fps"] <= 0 or c["fps"] > 1/backend.model.opt.timestep:
            raise ValueError("Camera rate must fit the physics timestep")
    while logical < start+duration-1e-10:
        control_at = start+tick*backend.dt
        camera_at = {c["id"]:start+sequences[c["id"]]/c["fps"] for c in cameras} if recorder else {}
        event_at = min(control_at, *camera_at.values(), start+duration)
        if event_at > logical+1e-12:
            backend.advance(event_at-logical)
            logical = event_at
        if event_at >= start+duration-1e-10:
            break
        state = backend.observe()
        t = state.stamp.seconds
        if control_at <= event_at+1e-10:
            twist, gripper = source.sample(logical-start)
            deadline = Stamp(t+.25,"simulation",backend.epoch)
            backend.submit(twist,deadline)
            if gripper.intent != "hold" or backend.tool_intent != "hold":
                backend.submit(gripper,deadline)
            if recorder:
                recorder.publish("robot",tick,t,asdict(state),session_time=t)
                recorder.publish("commands",tick,t,{"twist":asdict(twist),"gripper":asdict(gripper)},session_time=t)
            tick += 1
        for camera in cameras if recorder else ():
            name = camera["id"]
            if camera_at[name] <= event_at+1e-10:
                rgb = backend.render(camera.get("camera","overview"),camera["width"],camera["height"])
                recorder.publish(name,sequences[name],t,{},session_time=t,rgb=rgb)
                sequences[name] += 1
        if viewer:
            if not viewer.is_running():
                break
            viewer.sync()
        if realtime:
            time.sleep(max(0.,wall_start+logical-start-time.monotonic()))
    return backend.observe()
