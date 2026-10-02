"""xArm source schemas and legacy runtime callbacks adapted to generic raw v4."""
from datetime import datetime, timezone
import itertools
from pathlib import Path
import re
import shutil
import threading
import time
import uuid
from robot_teleop.core import Stamp
from robot_teleop.recording.v4 import Source, check_episode
from robot_teleop.recording.archive import atomic_json
from robot_teleop.recording.native_writer import SessionWriter
from robot_teleop.recording.clocks import ClockSampler
from robot_teleop.recording.visual import video_preflight, file_sha256
from robot_teleop.runtime.episodes import EpisodeSession
from .report import REPORT_SEMANTICS


class XArmCollectionRecorder:
    def __init__(self, output_root, task, config_snapshot, *, cameras=None, pre_roll_s=.5,
                 post_roll_s=.5, source_timeout_s=1., clock_interval_s=1., video_queue_size=64,
                 state_queue_size=4096, log_queue_size=4096, min_free_bytes=10*1024**3,
                 metadata=None, visual_storage=None):
        self.root, self.task, self.config = Path(output_root), task, config_snapshot
        self.cameras = cameras or []
        self.metadata = metadata or {}
        self.pre, self.post, self.timeout = pre_roll_s, post_roll_s, source_timeout_s
        self.capacity = max(state_queue_size, video_queue_size*max(1,len(self.cameras)))
        self.log_capacity, self.clock_interval = log_queue_size, clock_interval_s
        self.min_free_bytes = min_free_bytes
        self.session = self.log = self.clocks = None
        self.latest = {}
        self.error = None
        self._stop = threading.Event()
        self._sequence = itertools.count()
        self._closed = False
        self._state = None

    def preflight(self):
        if self.cameras:
            video_preflight()

    def open(self):
        self.preflight()
        task = re.sub(r"[^\w.-]+", "_", self.task).strip("._") or "task"
        self.session_path = self.root/task/f"session_{datetime.now(timezone.utc):%Y%m%dT%H%M%S}_{uuid.uuid4().hex[:8]}"
        self.session_path.mkdir(parents=True)
        if shutil.disk_usage(self.session_path).free < self.min_free_bytes:
            raise OSError("Insufficient disk reserve")
        self.sources = [Source("robot", "robot_state", "xarm_tcp30000_prefix_v1", REPORT_SEMANTICS["units"], "link_base", "robot_tcp30000", binary=True),
                        Source("gripper", "gripper_state", "xarm-pulse/v1", {"position": "pulse"}, "tool", "host_monotonic"),
                        Source("report", "robot_state", "xarm-rich-report/v1", {"tcp": "mm,rad(RPY)"}, "link_base", "host_monotonic", required=False),
                        Source("commands", "command", "xarm-control-log/v1", {"linear": "mm/s", "angular": "rad/s"}, "link_base", "host_monotonic", required=False)]
        mock = self.metadata.get("capture_mode") == "mock"
        self.sources += [Source(c["role"], "rgb", "realsense-rgb/v1", {"source_timestamp": "s", "native_device_timestamp": "ms"}, c["role"],
                                f"{'mock_camera' if mock else 'camera'}:{c['serial']}",
                                video={k: c[k] for k in ("width", "height", "fps")}) for c in self.cameras]
        self.session = EpisodeSession(self.session_path/"episodes", self.sources, clock="host_monotonic",
            pre_roll=self.pre, post_roll=self.post, capacity=self.capacity, metadata={"config":self.config,"provenance":self.metadata,"report_semantics":REPORT_SEMANTICS})
        self.log = SessionWriter(self.session_path,self.mark_fault,self.log_capacity)
        self.log.start()
        self.clocks = ClockSampler(lambda r:self.log.submit("clock",r),self.publish_event,self.clock_interval,on_error=self.mark_fault)
        self.clocks.start()
        self.meta = {"format_version":4,"status":"recording","metadata":self.metadata,"config":self.config}
        atomic_json(self.session_path/"session.json",self.meta)
        self.thread = threading.Thread(target=self._supervise,name="raw4-xarm-supervisor")
        self.thread.start()

    def _stamp(self):
        return Stamp(time.monotonic(),"host_monotonic",self.session.epoch)

    def publish_state(self,row):
        if not self.session:
            return
        source = row["kind"]
        received = row["received_at"]
        self.latest[source] = received
        values = {k:v for k,v in row.items() if k != "packet"}
        stamp = row.get("device_timestamp_raw",received*1e6)/1e6 if source == "robot" else received
        self.session.publish(source,int(row["sequence"]),stamp,values,session_time=received,
            epoch=self.session.epoch,source_epoch=row.get("clock_epoch",0),received_at=received,
            valid=row.get("valid",True),packet=row.get("packet"))

    def publish_frame(self,row):
        received = row["received_at"]
        role = row["role"]
        self.latest[role] = received
        self.session.publish(role,int(row["sequence"]),row["device_timestamp_ms"]/1000,
            {k:v for k,v in row.items() if k != "rgb"},session_time=received,epoch=self.session.epoch,
            source_epoch=row.get("clock_epoch",0),received_at=received,rgb=row["rgb"])

    def write(self,row):
        if self.log:
            self.log.submit("control",row)
        if self.session:
            t = time.monotonic()
            self.session.publish("commands",next(self._sequence),t,row,session_time=t,epoch=self.session.epoch)

    def publish_event(self,row):
        if self.log:
            self.log.submit("event",row)
        if row.get("boundary"):
            self.mark_boundary("source_reset")

    def mark_boundary(self,reason):
        if self.session:
            self.session.tick(Stamp(time.monotonic(),"host_monotonic",self.session.epoch+1))

    def mark_fault(self,error):
        self.error = str(error)
        if self.session:
            self.session.error = self.error
            if self.session.active:
                self.session.active.fault(self.error)
            self.session.stop(self._stamp(),incomplete=True)

    def start_episode(self):
        now = time.monotonic()
        if self.error or any(now-self.latest.get(s.id,-1e9)>self.timeout for s in self.sources if s.required):
            return None
        if self.status()["state"] != "idle":
            return None
        self._state = "recording"
        self.session.start(self._stamp())
        return "queued"

    def stop_episode(self,reason="user",incomplete=False):
        if self.session and self.status()["state"] in ("recording","postroll"):
            self._state = "finalizing" if incomplete else "postroll"
            self.session.stop(self._stamp(),incomplete)

    def disposition(self,value):
        self.session.disposition(value)
        self._state = "idle"

    def status(self):
        last = self.session.last if self.session else None
        complete = last is not None and last.meta["status"] == "complete"
        state = self.session.state if self.session else "idle"
        if state == "recording" and self._state in ("postroll", "finalizing"):
            state = self._state
        if self.error or (self.session and self.session.error):
            state = "fault"
        elif state == "idle" and last and last.meta["disposition"] == "pending":
            state = "review" if complete else "fault"
        elif self._state and state == "idle":
            state = self._state
        return {"state":state,"complete":complete,"error":self.error or (self.session.error if self.session else None),
                "fatal_error":self.error,"episode_id":None if last is None else last.path.name,
                "episode_path":None if last is None else str(last.path),"disposition":None if last is None else last.meta["disposition"],
                "elapsed_s":0.,"frame_counts":{} }

    def _supervise(self):
        try:
            while not self._stop.wait(.05):
                now = time.monotonic()
                self.session.tick(self._stamp())
                if self.session.active:
                    for s in self.sources:
                        if s.required and now-self.latest.get(s.id,-1e9)>self.timeout:
                            self.mark_fault(f"Source stopped: {s.id}")
                    if shutil.disk_usage(self.session_path).free < self.min_free_bytes:
                        self.mark_fault("Disk reserve breached")
        except Exception as exc:
            self.mark_fault(exc)

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._stop.set()
        if hasattr(self,"thread"):
            self.thread.join(2)
        if self.clocks:
            self.clocks.close()
        if self.session:
            self.session.close()
            if self.session.last and self.session.last.meta["status"] == "complete":
                report = check_episode(self.session.last.path)
                if not report["valid"]:
                    self.error = str(report["errors"])
        if self.log:
            self.log.close()
            self.meta.update(status="incomplete" if self.error else "closed",error=self.error,
                checksums={p.name:file_sha256(p) for p in self.session_path.glob("*.jsonl")})
            atomic_json(self.session_path/"session.json",self.meta)
