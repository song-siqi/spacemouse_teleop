"""Official writer/reader integration, synthetic video, no hardware or Hub."""

import unittest, importlib.util, tempfile, json
from pathlib import Path
from dataclasses import asdict
from unittest.mock import patch

if (
    importlib.util.find_spec("lerobot") is None
    or importlib.util.find_spec("scipy") is None
):
    raise unittest.SkipTest("Requires independent processing environment")
import cv2, numpy as np
from robot_teleop.processing.config import load, MODES
from robot_teleop.processing.exporters.lerobot import export
from robot_teleop.processing.validation import check_dataset
from robot_teleop.replay.reconstruction import actions
from test_processing import state


class DatasetIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.tmp.name)
        source = cls.root / "source.mp4"
        video = cv2.VideoWriter(
            str(source), cv2.VideoWriter_fourcc(*"mp4v"), 20, (32, 24)
        )
        for i in range(8):
            video.write(np.full((24, 32, 3), 30 + i * 20, np.uint8))
        video.release()
        cls.cameras = dict(
            view=dict(width=32, height=24, fps=20, video_path=str(source))
        )
        cls.states = [state(i * 0.05) for i in range(6)]
        cls.refs = [
            dict(time=s.time, images=dict(view=dict(video_frame_index=i, error_s=0)))
            for i, s in enumerate(cls.states)
        ]
        cls.outputs = []
        for mode in MODES:
            root = cls.root / mode
            root.mkdir()
            c = load(action_mode=mode)
            a = actions(cls.states, mode)
            with patch(
                "socket.create_connection", side_effect=AssertionError("No network")
            ):
                export(
                    root / "lerobot",
                    cls.states,
                    cls.refs,
                    cls.cameras,
                    a,
                    c,
                    "synthetic",
                )
            (root / "processing_manifest.json").write_text(
                json.dumps(dict(status="complete", config=c))
            )
            (root / "reference_trajectory.json").write_text(
                json.dumps([asdict(s) for s in cls.states])
            )
            (root / "source_refs.jsonl").write_text(
                "".join(json.dumps(r) + "\n" for r in cls.refs[:-1])
            )
            (root / "alignment_report.json").write_text(
                json.dumps(dict(camera_sources=cls.cameras))
            )
            np.save(root / "actions.npy", a)
            cls.outputs.append(root)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_all_modes_official_readback_and_full_video(self):
        for p in self.outputs:
            with (
                self.subTest(mode=p.name),
                patch(
                    "socket.create_connection", side_effect=AssertionError("No network")
                ),
            ):
                r = check_dataset(p)
                self.assertTrue(r["valid"])
                self.assertTrue(r["source_image_comparison"])

    def test_action_tamper_rejected(self):
        p = self.outputs[0]
        a = np.load(p / "actions.npy")
        bad = a.copy()
        bad[0, 0] += 1
        np.save(p / "actions.npy", bad)
        try:
            with self.assertRaises(ValueError):
                check_dataset(p)
        finally:
            np.save(p / "actions.npy", a)

    def test_history_future_padding_and_workers(self):
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        from torch.utils.data import DataLoader

        ds = LeRobotDataset(
            "local/teleop",
            root=self.outputs[0] / "lerobot",
            video_backend="pyav",
            delta_timestamps={"observation.state": [-0.05, 0.0], "action": [0.0, 0.05]},
        )
        self.assertEqual(tuple(ds[0]["observation.state"].shape), (2, 7))
        self.assertTrue(bool(ds[0]["observation.state_is_pad"][0]))
        self.assertTrue(bool(ds[len(ds) - 1]["action_is_pad"][-1]))
        for n in (0, 2):
            options = dict(multiprocessing_context="spawn") if n else {}
            batches = list(DataLoader(ds, batch_size=2, num_workers=n, **options))
            self.assertEqual(sum(len(b["index"]) for b in batches), len(ds))


class FailureSubmission(unittest.TestCase):
    def test_disk_failure_keeps_incomplete_staging_and_raw_unchanged(self):
        from robot_teleop.processing.pipeline import process_episode, sha
        from test_processing import Resampling

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            session = root / "session"
            ep = session / "episodes" / "ep"
            ep.mkdir(parents=True)
            meta = dict(format_version=4, start_time=0, end_time=0.1, task="failure")
            (ep / "meta.json").write_text(json.dumps(meta))
            (session / "session.json").write_text('{"config":{},"status":"closed"}')
            (session / "clock_anchors.jsonl").write_text("")
            _, streams, cameras, times = Resampling().fixture()
            cameras["view"].update(fps=30, width=32, height=24)
            for name, rows in streams.items():
                for i, row in enumerate(rows):
                    row.update(sequence=i, valid=True)
            original = sha(ep / "meta.json")
            by_identity = {id(streams[k]): times[k] for k in times}
            with (
                patch(
                    "robot_teleop.recording.raw.check_episode",
                    return_value={"valid": True},
                ),
                patch(
                    "robot_teleop.processing.pipeline.raw_v4.read",
                    return_value=(streams, cameras),
                ),
                patch(
                    "robot_teleop.processing.pipeline.map_source",
                    side_effect=lambda rows, *a: (by_identity[id(rows)], {}),
                ),
                patch(
                    "robot_teleop.processing.exporters.lerobot.export",
                    side_effect=OSError("disk full"),
                ),
            ):
                with self.assertRaisesRegex(OSError, "disk full"):
                    process_episode(ep, root / "output", load())
            self.assertFalse((root / "output").exists())
            failure = json.loads(
                (root / "output.partial/processing_manifest.json").read_text()
            )
            self.assertEqual(failure["status"], "incomplete")
            self.assertEqual(original, sha(ep / "meta.json"))
