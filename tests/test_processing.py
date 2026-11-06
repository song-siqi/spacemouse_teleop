import unittest
from unittest.mock import patch
import importlib.util

if importlib.util.find_spec("scipy") is None:
    raise unittest.SkipTest("Requires independent processing environment")
import numpy as np
from scipy.spatial.transform import Rotation
from robot_teleop.processing.contracts import State
from robot_teleop.processing.config import load, MODES
from robot_teleop.processing.timing.mapping import affine, map_source
from robot_teleop.processing.timing.quality import nearest, bracket
from robot_teleop.replay.reconstruction import actions
from robot_teleop.processing.validation import reconstruction_error
from robot_teleop.replay.runner import run


def state(t, n=6):
    q = Rotation.from_rotvec([0.1 * t, 0.2 * t, -0.3 * t]).as_quat()[[3, 0, 1, 2]]
    return State(
        t,
        [0.01 * t * (i + 1) for i in range(n)],
        [0.01 * t, 0.02 * t, 0],
        q.tolist(),
        0.2 + 0.01 * t,
        700.0,
    )


class Actions(unittest.TestCase):
    def test_all_modes_and_joint_counts(self):
        for n in (6, 7):
            for mode in MODES:
                with self.subTest(n=n, mode=mode):
                    states = [state(t, n) for t in np.arange(0, 5, 0.05)]
                    a = actions(states, mode)
                    self.assertEqual(len(a), len(states) - 1)
                    self.assertLess(
                        reconstruction_error(states, a, mode)[
                            "max_reconstruction_error"
                        ],
                        1e-5,
                    )

    def test_noncommuting_rotations(self):
        states = [state(0), state(0), state(0)]
        for s, angles in zip(states, ([3.13, 0.1, 0], [-3.13, 0.2, 0.3], [0.2, 1, 2])):
            s.quaternion = Rotation.from_rotvec(angles).as_quat()[[3, 0, 1, 2]].tolist()
        self.assertLess(
            reconstruction_error(
                states, actions(states, "delta_ee_pose"), "delta_ee_pose"
            )["max_reconstruction_error"],
            1e-5,
        )

    def test_fps_recomputes_delta(self):
        for hz in (10, 20, 30):
            a = actions([state(0), state(1 / hz)], "delta_joint_position")
            self.assertAlmostEqual(a[0, 0], 0.01 / hz)

    def test_bad_config(self):
        for fps in (True, 0, -1, 1.5):
            with self.assertRaises(ValueError):
                load(fps=fps)


class Timing(unittest.TestCase):
    def test_offset_drift_and_outliers(self):
        x = np.linspace(0, 100, 1000)
        true = 1000 + 1.0001 * x
        y = true.copy()
        y[::50] += 0.1
        mapped, fit = affine(x, y)
        self.assertLess(np.max(abs(mapped - true)), 0.001)
        self.assertIsNone(fit["fixed_offset_uncertainty_s"])

    def test_selection_and_gap(self):
        for hz in (30, 60, 57.76):
            times = np.arange(0, 2, 1 / hz)
            for t in np.arange(0.1, 1.8, 0.05):
                self.assertLessEqual(abs(times[nearest(times, t)] - t), 1 / hz)
        self.assertEqual(bracket([0, 0.004], 0.002, 0.02), (0, 1, 0.5))
        with self.assertRaises(ValueError):
            bracket([0, 0.1], 0.05, 0.02)

    def test_epoch_refused(self):
        with self.assertRaises(ValueError):
            map_source([dict(clock_epoch=0), dict(clock_epoch=1)], "robot", [])

    def test_degenerate_clock(self):
        with self.assertRaises(ValueError):
            affine([0, 0], [1, 2])


class Scheduling(unittest.TestCase):
    def test_cleanup_and_no_hardware(self):
        now = [0.0]
        seen = []

        class Fake:
            def connect(self, s, mode):
                seen.append("connect")

            def step(self, s, v, mode, dt):
                seen.append(s.time)
                return {}

            def close(self):
                seen.append("stop")

        with patch(
            "socket.create_connection", side_effect=AssertionError("hardware forbidden")
        ):
            run(
                [state(0), state(0.1)],
                "delta_joint_position",
                Fake(),
                clock=lambda: now[0],
                sleep=lambda dt: now.__setitem__(0, now[0] + dt),
                speed=0.25,
            )
        self.assertEqual(seen[0], "connect")
        self.assertEqual(seen[-1], "stop")
        self.assertAlmostEqual(seen[-2], 0.1)
        self.assertGreaterEqual(now[0], 0.4)

    def test_fault_stops(self):
        closed = []

        class Fake:
            def connect(self, *a):
                pass

            def step(self, *a):
                raise RuntimeError("feedback lost")

            def close(self):
                closed.append(True)

        with self.assertRaisesRegex(RuntimeError, "feedback lost"):
            run([state(0), state(1)], "joint_position", Fake())
        self.assertEqual(closed, [True])


class Resampling(unittest.TestCase):
    def fixture(self):
        from robot_teleop.processing.timing.resample import align as resample
        from robot_teleop.processing.sources import xarm
        from functools import partial

        align = partial(resample, adapter=xarm)

        ts = np.arange(-0.1, 0.301, 0.004)
        robot = [
            dict(
                values=dict(
                    values=dict(
                        joints_rad=[float(t)] * 6,
                        tcp_mm_rotvec_rad=[t * 1000, 0, 0, 0, 0, t],
                    )
                )
            )
            for t in ts
        ]
        gs = np.arange(-0.1, 0.31, 0.05)
        grip = [
            dict(
                host_received_at=float(t),
                values=dict(
                    values=dict(position_pulse=600),
                    query_started_at=float(t - 0.001),
                    query_finished_at=float(t),
                ),
            )
            for t in gs
        ]
        ct = np.arange(-0.1, 0.31, 1 / 30)
        camera = [
            dict(sequence=i, video_frame_index=i, values=dict(frame_id=i))
            for i in range(len(ct))
        ]
        return (
            align,
            dict(robot=robot, gripper=grip, view=camera),
            dict(view={}),
            dict(robot=ts, gripper=gs, view=ct),
        )

    def test_state_interpolation_before_difference(self):
        align, streams, cameras, times = self.fixture()
        ss, refs = align(
            streams, cameras, times, dict(start_time=0, end_time=0.2), load()
        )
        self.assertEqual(len(ss), 5)
        for s in ss:
            self.assertAlmostEqual(s.joints[0], s.time)
        self.assertTrue(np.allclose(actions(ss, "delta_joint_position")[:, 0], 0.05))
        self.assertEqual(len(refs), len(ss))

    def test_image_error_refused(self):
        align, streams, cameras, times = self.fixture()
        times["view"] = times["view"] + 10
        with self.assertRaisesRegex(ValueError, "Image too far"):
            align(streams, cameras, times, dict(start_time=0, end_time=0.2), load())

    def test_stale_gripper_refused(self):
        align, streams, cameras, times = self.fixture()
        times["gripper"] = times["gripper"] - 1
        with self.assertRaisesRegex(ValueError, "Gripper"):
            align(streams, cameras, times, dict(start_time=0, end_time=0.2), load())


class GenericAdapter(unittest.TestCase):
    def test_seven_joint_source_without_xarm_decoder(self):
        from robot_teleop.processing.timing.resample import align
        from scipy.spatial.transform import Rotation
        import types

        _, streams, cameras, times = Resampling().fixture()
        adapter = types.SimpleNamespace(
            robot_values=lambda row: (
                np.arange(7, dtype=float),
                np.zeros(3),
                Rotation.identity(),
            ),
            gripper_values=lambda row, c: (0.5, 500.0),
        )
        states, _ = align(
            streams,
            cameras,
            times,
            dict(start_time=0, end_time=0.2),
            load(),
            adapter=adapter,
        )
        self.assertEqual(len(states[0].joints), 7)
        self.assertEqual(actions(states, "delta_joint_position").shape[1], 8)


class Following(unittest.TestCase):
    def test_closed_loop_reference_following(self):
        now = [0.0]
        actual = np.zeros(6)
        errors = []

        class Plant:
            def connect(self, *args):
                pass

            def step(self, s, v, mode, dt):
                error = np.asarray(s.joints) - actual
                errors.append(float(np.max(abs(error))))
                actual[:] += dt * (v + 2 * error)
                return {"max_error": errors[-1]}

            def close(self):
                pass

        states = [state(t) for t in np.arange(0, 1.01, 0.05)]
        run(
            states,
            "delta_joint_position",
            Plant(),
            speed=0.25,
            clock=lambda: now[0],
            sleep=lambda dt: now.__setitem__(0, now[0] + dt),
        )
        self.assertLess(max(errors), 0.002)
        self.assertLess(
            float(np.max(abs(actual - np.asarray(states[-1].joints)))), 0.002
        )


class PreparationClock(unittest.TestCase):
    def test_reference_time_starts_after_connect_preparation(self):
        now = [0.0]
        rows = []

        class Prepared:
            def connect(self, *args):
                now[0] += 10

            def step(self, *args):
                return {}

            def close(self):
                pass

        run(
            [state(0), state(0.1)],
            "joint_position",
            Prepared(),
            clock=lambda: now[0],
            sleep=lambda dt: now.__setitem__(0, now[0] + dt),
            log=rows.append,
        )
        self.assertEqual(rows[0]["elapsed_s"], 0.0)
        self.assertEqual(rows[0]["reference_time"], 0.0)
        self.assertGreater(now[0], 10.4)
