import json
from pathlib import Path
import shutil
import tempfile
import unittest
import numpy as np
from robot_teleop.recording.v4 import RawEpisode, Source, check_episode
from robot_teleop.recording.raw import check_episode as dispatch_check


class RawV4Tests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

    def source(self, binary=False):
        return Source("robot", "robot_state", "test/v1", {"joint": "rad"}, "base", "simulation", binary=binary)

    def test_zero_cameras_variable_dof_and_binary_checksum(self):
        for dof in (6, 7):
            path = self.root / str(dof)
            writer = RawEpisode(path, [self.source(True)])
            for i in range(3):
                writer.publish("robot", i, i*.01, {"joints": [0.]*dof}, session_time=i*.01, packet=bytes([i]))
            writer.close()
            self.assertTrue(dispatch_check(path)["valid"])
            (path / "robot.bin").write_bytes(b"corrupt")
            self.assertFalse(check_episode(path)["valid"])

    def test_reset_duplicates_and_missing_samples_are_not_complete(self):
        for name, sequence, epoch in (("duplicate", [0, 0], 0), ("gap", [0, 2], 0), ("reset", [0, 1], 1)):
            writer = RawEpisode(self.root/name, [self.source()])
            for i in sequence:
                writer.publish("robot", i, i, {}, session_time=i, epoch=epoch)
            writer.close()
            self.assertFalse(check_episode(writer.path)["valid"])

    def test_writer_failure_and_queue_overflow(self):
        import threading
        entered, release = threading.Event(), threading.Event()
        class BlockedEncoder:
            def __init__(self, *args): pass
            def write(self, rgb):
                entered.set()
                release.wait(2)
                raise OSError("injected disk full")
            def abort(self): pass
        source = Source("camera", "rgb", "rgb/v1", {}, "world", "simulation", video={"width": 32, "height": 24, "fps": 30})
        writer = RawEpisode(self.root/"fault", [source], capacity=1, video_factory=BlockedEncoder)
        rgb = np.zeros((24,32,3),np.uint8)
        try:
            writer.publish("camera", 0, 0, {}, session_time=0, rgb=rgb)
            self.assertTrue(entered.wait(1))
            writer.publish("camera", 1, .1, {}, session_time=.1, rgb=rgb)
            self.assertFalse(writer.publish("camera", 2, .2, {}, session_time=.2, rgb=rgb))
        finally:
            release.set()
            writer.close()
        self.assertEqual(writer.meta["status"], "incomplete")
        self.assertFalse(check_episode(writer.path)["valid"])

    def test_checker_rejects_unmarked_clock_reset_even_with_valid_checksum(self):
        from robot_teleop.recording.visual import file_sha256
        writer = RawEpisode(self.root/"clock", [self.source()])
        for i in range(2):
            writer.publish("robot", i, i, {}, session_time=i)
        writer.close()
        index = writer.path/"robot.jsonl"
        rows = [json.loads(line) for line in index.read_text().splitlines()]
        rows[1]["source_timestamp"] = -1
        index.write_text("".join(json.dumps(row)+"\n" for row in rows))
        meta = writer.meta
        meta["checksums"][index.name] = file_sha256(index)
        (writer.path/"meta.json").write_text(json.dumps(meta))
        self.assertFalse(check_episode(writer.path)["valid"])

    def test_session_buffers_reset_and_consecutive_episodes(self):
        from robot_teleop.runtime.episodes import EpisodeSession
        from robot_teleop.core import Stamp
        session = EpisodeSession(self.root/"session",[self.source()],pre_roll=.1,post_roll=.1)
        self.addCleanup(session.close)
        for i in range(5):
            session.publish("robot",i,i*.05,{},session_time=i*.05)
        session.start(Stamp(.2,"simulation"))
        session.stop(Stamp(.2,"simulation"))
        session.publish("robot",5,.25,{},session_time=.25)
        session.tick(Stamp(.31,"simulation"))
        session.disposition("kept")
        session.flush()
        first = session.last.path
        self.assertTrue(check_episode(first)["valid"])
        rows = [json.loads(s) for s in (first/"robot.jsonl").read_text().splitlines()]
        self.assertEqual([r["sequence"] for r in rows],[2,3,4,5])
        session.tick(Stamp(0,"simulation",1))
        session.flush()
        session.start(Stamp(0,"simulation",1))
        session.publish("robot",0,0,{},session_time=0,epoch=1)
        session.stop(Stamp(.1,"simulation",1))
        session.tick(Stamp(.21,"simulation",1))
        session.flush()
        self.assertTrue(check_episode(session.last.path)["valid"])
        self.assertEqual(session.last.meta["counts"]["robot"],1)

    @unittest.skipUnless(shutil.which("ffmpeg") and shutil.which("ffprobe"), "FFmpeg required")
    def test_one_and_three_cameras_real_encoder_consecutive_episodes(self):
        for count in (1, 3):
            sources = [Source(f"cam{i}", "rgb", "rgb/v1", {}, "world", "simulation",
                              video={"width":32,"height":24,"fps":10+i*5}) for i in range(count)]
            for episode in range(2):
                writer = RawEpisode(self.root/f"{count}-{episode}", sources)
                for s in sources:
                    for i in range(5):
                        rgb = np.full((24,32,3), i*40, np.uint8)
                        writer.publish(s.id,i,i/s.video["fps"],{},session_time=i/s.video["fps"],rgb=rgb)
                writer.close()
                report = check_episode(writer.path)
                self.assertTrue(report["valid"],report)
