import copy
import json
from pathlib import Path
import shutil
import struct
import tempfile
import threading
import time
import types
import unittest
from unittest.mock import patch

import numpy as np
from spacemouse_teleop.recording.collection import CollectionRecorder, ROLES
from spacemouse_teleop.recording.raw import check_episode, iter_source
from spacemouse_teleop.recording.native_writer import StreamingVideo
from spacemouse_teleop.backends.xarm6_camera.report import MockReportReceiver
from spacemouse_teleop.backends.xarm6_camera.state import StatePoller
from spacemouse_teleop.sensors.realsense import MockCameraGroup

def wait_until(check, timeout=10):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if check(): return
        time.sleep(.005)
    raise AssertionError("Condition did not become true")

class Harness:
    def __init__(self, root, **options):
        self.specs = [{"role":role,"serial":f"mock-{role}","width":32,"height":24,"fps":fps}
                      for role,fps in zip(ROLES,(30,25,20))]
        self.seen = {r: [] for r in ROLES}
        self.rec = CollectionRecorder(root,"native test",{},cameras=self.specs, min_free_bytes=0,
                   **{"pre_roll_s":.08,"post_roll_s":.08,**options})
        self.rec.open()
        def frame(row):
            self.seen[row["role"]].append({k:v for k,v in row.items() if k != "rgb"})
            self.rec.publish_frame(row)
        self.cameras = MockCameraGroup(self.specs, frame, self.rec.mark_fault,
                                        on_event=self.rec.publish_event)
        self.cameras.start()
        self.robot = MockReportReceiver("unused",self.rec.publish_state,self.rec.mark_fault,
                                        on_event=self.rec.publish_event)
        self.robot.start()
        class Channel:
            def __init__(self,*a,**kw):
                assert kw["writable"] is False
            def gripper_status(self):
                return {"position_pulse":500., "controller_target_pulse":700.,
                        "received_at":time.monotonic(), "error_code":0}
            def close(self): pass
        self.gripper=StatePoller("unused",self.rec.publish_state,self.rec.mark_fault,hz=0,
                                gripper_hz=20,channel_factory=Channel)
        self.gripper.start()
        wait_until(lambda:all(k in self.rec._latest for k in (*ROLES,"robot","gripper")))
    def record(self,duration=.2):
        self.start()
        time.sleep(duration)
        return self.finish()
    def start(self):
        assert self.rec.start_episode() is not None,self.rec.status()
    def finish(self):
        self.rec.stop_episode()
        wait_until(lambda:self.rec.status()["state"] in ("review","fault"))
        return Path(self.rec.status()["path"])
    def close(self):
        self.cameras.close();self.robot.close();self.gripper.close();self.rec.close()

@unittest.skipUnless(shutil.which("ffmpeg") and shutil.which("ffprobe"),"Requires local FFmpeg")
class NativeRecordingTests(unittest.TestCase):
    def harness(self,**options):
        temp=tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        h=Harness(temp.name,**options)
        self.addCleanup(h.close)
        return h

    def test_every_native_frame_and_report_is_written_without_a_grid(self):
        h=self.harness()
        time.sleep(.1)
        path=h.record(.4)
        status=h.rec.status()
        self.assertTrue(status["complete"],status)
        meta=json.loads((path/"meta.json").read_text())
        self.assertEqual(meta["format_version"],3)
        self.assertNotIn("sample_hz",meta)
        self.assertFalse((path/"observations.jsonl").exists())
        self.assertFalse(list(path.rglob("*.png")))
        for role in ROLES:
            saved=list(iter_source(path,f"frames/{role}.jsonl"))
            source=[f for f in h.seen[role] if meta["capture_requested_start_time"] <= f["received_at"] <= meta["capture_requested_end_time"]]
            self.assertEqual([f["sequence"] for f in saved],[f["sequence"] for f in source])
            self.assertEqual([f["video_frame_index"] for f in saved],list(range(len(saved))))
            self.assertEqual([f["device_timestamp_ms"] for f in saved],[f["device_timestamp_ms"] for f in source])
        self.assertGreater(status["frame_counts"]["wrist"],status["frame_counts"]["fixed_2"])
        self.assertGreater(status["state_counts"]["robot"],status["frame_counts"]["wrist"]*5)
        self.assertTrue(check_episode(path)["valid"],check_episode(path))
        self.assertEqual(meta["continuity"]["events"],{})
        self.assertLess(meta["capture_start_time"],meta["start_time"])
        self.assertGreater(meta["capture_end_time"],meta["end_time"])

    def test_two_episodes_restart_file_indices_without_crossing_reset(self):
        h=self.harness()
        first=h.record()
        last=list(iter_source(first,"robot_states.jsonl"))[-1]
        self.assertTrue(h.rec.disposition("kept"))
        wait_until(lambda:h.rec.status()["state"]=="idle")
        boundary=time.monotonic()
        h.rec.mark_boundary("preparation_start")
        time.sleep(.08)
        h.rec.mark_boundary("preparation_end")
        end_boundary=time.monotonic()
        wait_until(lambda:all(k in h.rec._latest for k in (*ROLES,"robot","gripper")))
        second=h.record()
        second_meta=json.loads((second/"meta.json").read_text())
        rows=list(iter_source(second,"robot_states.jsonl"))
        self.assertGreater(rows[0]["sequence"],last["sequence"])
        self.assertEqual(rows[0]["packet_offset"],0)
        self.assertGreaterEqual(second_meta["capture_start_time"],end_boundary-.002)
        self.assertEqual(next(iter_source(second,"frames/wrist.jsonl"))["video_frame_index"],0)
        self.assertTrue(check_episode(first,require_kept=True)["valid"])

    def test_postroll_blocks_keep_and_new_episode(self):
        h=self.harness(post_roll_s=.2)
        h.start();time.sleep(.1);h.rec.stop_episode()
        self.assertEqual(h.rec.status()["state"],"postroll")
        self.assertIsNone(h.rec.start_episode())
        self.assertFalse(h.rec.disposition("kept"))
        wait_until(lambda:h.rec.status()["state"]=="review")
        self.assertTrue(h.rec.status()["complete"])

    def test_fault_cuts_tail_and_keep_does_not_repair_integrity(self):
        h=self.harness(post_roll_s=.5)
        h.start();time.sleep(.1)
        at=time.monotonic()
        h.rec.mark_fault("camera disconnected")
        wait_until(lambda:h.rec.status()["state"]=="fault")
        path=Path(h.rec.status()["path"])
        meta=json.loads((path/"meta.json").read_text())
        self.assertEqual(meta["status"],"incomplete")
        self.assertLess(meta["capture_requested_end_time"]-at,.1)
        self.assertTrue(list((path/"videos").glob("*.mp4")))
        h.rec.disposition("kept");wait_until(lambda:h.rec.status()["state"]=="idle")
        self.assertFalse(check_episode(path)["valid"])

    def test_invalid_report_is_preserved_instead_of_becoming_a_quality_gate(self):
        h=self.harness()
        h.start()
        original=h.robot.accept
        done=threading.Event()
        def corrupt(packet, received, batch):
            if not done.is_set():
                value=bytearray(packet)
                struct.pack_into("<f",value,116,float("nan"))
                packet=bytes(value);done.set()
            original(packet,received,batch)
        h.robot.accept=corrupt
        self.assertTrue(done.wait(1))
        time.sleep(.1)
        path=h.finish()
        self.assertTrue(h.rec.status()["complete"],h.rec.status())
        rows=list(iter_source(path,"robot_states.jsonl"))
        invalid=[r for r in rows if not r["valid"]]
        self.assertEqual(len(invalid),1)
        self.assertIsNone(invalid[0]["values"]["joints_rad"][0])
        self.assertEqual(check_episode(path)["invalid_state_counts"]["robot"],1)

    def test_queue_overflow_is_bounded_and_preserves_failure_evidence(self):
        h=self.harness(pre_roll_s=0,video_queue_size=4)
        h.cameras.close()
        entered,release=threading.Event(),threading.Event()
        original=StreamingVideo.write
        def blocked(writer,rgb):
            entered.set()
            if not release.wait(2): raise OSError("test writer wait")
            original(writer,rgb)
        try:
            with patch.object(StreamingVideo,"write",blocked):
                h.start()
                spec=h.specs[0]
                for i in range(20):
                    row={"role":spec["role"],"serial":spec["serial"],"width":32,"height":24,
                         "capture_fps":30,"sequence":10000+i,"frame_id":10000+i,
                         "received_at":time.monotonic(),"device_timestamp_ms":i*33.,
                         "timestamp_domain":"mock","rgb":np.zeros((24,32,3),np.uint8)}
                    started=time.monotonic();h.rec.publish_frame(row)
                    self.assertLess(time.monotonic()-started,.1)
                release.set()
                wait_until(lambda:h.rec.status()["state"]=="fault")
        finally: release.set()
        status=h.rec.status()
        self.assertFalse(status["complete"])
        self.assertGreater(status["gaps"].get("queue_overflow",0),0)
        self.assertLessEqual(h.rec._last.videos["wrist"].high_water,4)

    def test_start_and_stop_do_not_touch_filesystem_on_calling_thread(self):
        h=self.harness()
        owner=threading.get_ident()
        import spacemouse_teleop.recording.collection as collection
        original=collection.atomic_json
        def checked(*args,**kwargs):
            self.assertNotEqual(threading.get_ident(),owner)
            return original(*args,**kwargs)
        with patch.object(collection,"atomic_json",checked):
            h.start();time.sleep(.1);h.finish()
        self.assertTrue(h.rec.status()["complete"],h.rec.status())

    def test_slow_validation_keeps_control_log_live(self):
        h=self.harness()
        from spacemouse_teleop.recording import raw
        entered,release=threading.Event(),threading.Event()
        original=raw.validate_native_episode
        def slow(*a,**kw):
            entered.set()
            if not release.wait(3):raise ValueError("test timeout")
            return original(*a,**kw)
        try:
            with patch.object(raw,"validate_native_episode",slow):
                h.start();time.sleep(.1);h.rec.stop_episode()
                self.assertTrue(entered.wait(2))
                for n in range(30): h.rec.write({"tick":n})
                h.rec.session_writer.flush()
                self.assertIn('"tick":29',(h.rec.session_path/"control.jsonl").read_text())
                self.assertIsNone(h.rec.start_episode())
                release.set()
                wait_until(lambda:h.rec.status()["state"]=="review")
        finally:release.set()

    def test_closed_session_evidence_checksum_is_checked(self):
        h=self.harness()
        path=h.record()
        h.close()
        self.assertTrue(check_episode(path)["valid"])
        with (path.parent.parent/"events.jsonl").open("a") as handle:
            handle.write('{"event":"unexpected"}\n')
        report=check_episode(path)
        self.assertFalse(report["valid"])
        self.assertIn("Session checksum mismatch",report["errors"][0])

    def test_isolated_camera_gap_is_logged_without_filtering_frames(self):
        h=self.harness()
        h.start()
        h.rec.publish_event({"event":"device_frame_gap", "source":"wrist",
                             "missing_count":2, "received_at":time.monotonic()})
        time.sleep(.1)
        path=h.finish()
        self.assertTrue(h.rec.status()["complete"],h.rec.status())
        self.assertEqual(check_episode(path)["continuity"]["events"]["device_frame_gap"],1)

    def test_original_packet_corruption_is_detected(self):
        h=self.harness()
        path=h.record()
        binary=path/"robot_reports.bin"
        with binary.open("r+b") as stream:
            stream.seek(116);stream.write(struct.pack("<f",999.))
        report=check_episode(path)
        self.assertFalse(report["valid"])
        self.assertIn("differs from original packet",report["errors"][0])

    def test_video_index_reordering_is_detected(self):
        h=self.harness()
        path=h.record()
        index=path/"frames/wrist.jsonl"
        rows=list(iter_source(path,"frames/wrist.jsonl"))
        rows[0]["video_frame_index"]=1
        index.write_text("".join(json.dumps(r)+"\n" for r in rows))
        report=check_episode(path)
        self.assertFalse(report["valid"])
        self.assertIn("frame index",report["errors"][0])

    def test_disk_reserve_failure_preserves_data_and_does_not_raise_into_publishers(self):
        h=self.harness()
        h.rec.min_free_bytes=1
        h.start()
        with patch("spacemouse_teleop.recording.collection.shutil.disk_usage",
                   return_value=types.SimpleNamespace(free=0)):
            wait_until(lambda:h.rec.status()["state"]=="fault")
        path=Path(h.rec.status()["path"])
        self.assertEqual(json.loads((path/"meta.json").read_text())["status"],"incomplete")

    def test_state_disk_full_preserves_partial_files_as_incomplete(self):
        h=self.harness()
        from spacemouse_teleop.recording import native_writer
        original=native_writer.line
        def full(handle,row):
            if row.get("kind") == "robot":
                raise OSError(28,"No space left on device")
            return original(handle,row)
        with patch.object(native_writer,"line",full):
            h.start()
            wait_until(lambda:h.rec.status()["state"]=="fault")
        path=Path(h.rec.status()["path"])
        self.assertTrue((path/"robot_reports.bin").exists())
        meta=json.loads((path/"meta.json").read_text())
        self.assertEqual(meta["status"],"incomplete")
        self.assertGreater(meta["accepted_state_count"],sum(meta["state_counts"].values()))
        self.assertIn("No space left",h.rec.status()["error"])

    def test_encoder_process_failure_preserves_other_streams(self):
        h=self.harness()
        h.start()
        wait_until(lambda:h.rec._last.videos["wrist"].count >= 3)
        h.rec._last.videos["wrist"].encoder.process.kill()
        wait_until(lambda:h.rec.status()["state"]=="fault")
        path=Path(h.rec.status()["path"])
        self.assertTrue((path/"videos/fixed_1.mp4").is_file())
        meta=json.loads((path/"meta.json").read_text())
        self.assertIn("wrist",meta["surviving_video_checks"])
        self.assertGreater(meta["surviving_video_checks"]["fixed_1"]["decoded_prefix_frames"],0)
        self.assertTrue((path/"robot_reports.bin").stat().st_size > 0)
        self.assertFalse(check_episode(path)["valid"])

    def test_close_active_recording_is_incomplete_and_workers_join(self):
        h=self.harness()
        h.start();time.sleep(.1)
        h.rec.close()
        self.assertFalse(h.rec.thread.is_alive())
        self.assertTrue(all(not w.thread.is_alive() for w in h.rec._last.videos.values()))
        self.assertFalse(h.rec._last.states.thread.is_alive())
        self.assertFalse(h.rec.status()["complete"])
        self.assertFalse(check_episode(h.rec.status()["path"])["valid"])

if __name__ == "__main__": unittest.main()
