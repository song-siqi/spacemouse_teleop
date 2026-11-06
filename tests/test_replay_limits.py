"""Regression for full-speed replay accidentally inheriting 0.1 rad/s homing cap."""

import unittest, importlib.util, json, types
from pathlib import Path
from unittest.mock import patch

if importlib.util.find_spec("scipy") is None:
    raise unittest.SkipTest("Requires processing environment")
import numpy as np
from robot_teleop.replay.limits import joint_limits, preflight
from robot_teleop.replay.runner import run
from robot_teleop.backends.real.xarm_sdk.replay import XArmReplay
from robot_teleop.processing.contracts import State
from test_xarm_config_safety import live_config, feedback


def reference():
    return [
        State(
            float(t),
            [0.0, 0.0, 0.0, 0.0, 0.24 * (1 - np.cos(t)), 0.0],
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            0.0,
            850.0,
        )
        for t in np.arange(0, 6.01, 0.05)
    ]


class LimitsTest(unittest.TestCase):
    def test_reference_accepts_1x_and_rejects_old_cap(self):
        report = preflight(reference(), "joint_position", 1.0)
        self.assertGreater(report["reference_peak_joint_speed_radps"], 0.23)
        with self.assertRaisesRegex(ValueError, "Reference requires"):
            preflight(reference(), "joint_position", 1.0, dict(joint_speed_radps=0.1))

    def test_rejects_acceleration_and_bad_limits(self):
        with self.assertRaisesRegex(ValueError, "acceleration"):
            preflight(reference(), "joint_position", 1.0, dict(joint_accel_radps2=0.1))
        for limits in [
            dict(joint_speed_radps=float("nan")),
            dict(joint_speed_radps=0),
            dict(joint_speed_radps=3),
            dict(typo=1),
        ]:
            with self.assertRaises(ValueError):
                joint_limits(limits)

    def test_preflight_failure_precedes_connect(self):
        calls = []

        class Fake:
            def preflight(self, *args):
                raise ValueError("infeasible reference")

            def connect(self, *args):
                calls.append("connect")

            def close(self):
                calls.append("close")

        with self.assertRaisesRegex(ValueError, "infeasible"):
            run(reference(), "joint_position", Fake())
        self.assertEqual(calls, [])

    def test_defaults_and_preparation_are_separate(self):
        config = live_config()
        config.robot["homing"].update(speed_radps=0.1, accel_radps2=0.2)
        backend = XArmReplay(config)
        self.assertEqual(backend.tracking_limits, joint_limits())
        self.assertEqual(config.robot["homing"]["speed_radps"], 0.1)
        self.assertEqual(config.robot["homing"]["accel_radps2"], 0.2)
        c = json.loads(Path("configs/replay/xarm6.json").read_text())
        self.assertEqual(c["speed"], 1.0)

    def test_full_speed_real_controller_with_5hz_feedback_and_finite_leases(self):
        now = [0.0]
        q = np.zeros(6)
        velocity = np.zeros(6)
        lease = [0.0]
        last = [0.0]
        sample = [None]
        last_report = [-1.0]
        config = live_config()
        config.robot.update(feedback_timeout_s=0.25, socket_timeout_s=0.05)
        config.robot["homing"].update(speed_radps=0.1, accel_radps2=0.2)
        backend = XArmReplay(config)
        backend.mode = 4

        def snapshot():
            dt = max(0, min(now[0], lease[0]) - last[0])
            q[:] += velocity * dt
            last[0] = now[0]
            if sample[0] is None or now[0] - last_report[0] >= 0.2 - 1e-9:
                last_report[0] = now[0]
                sample[0] = feedback(now[0], mode=4, joints_rad=tuple(q))
            return sample[0]

        def send(v, d):
            velocity[:] = v
            lease[0] = now[0] + d

        backend.channel = types.SimpleNamespace(
            joint_velocity=send, stop=lambda: [], close=lambda: None
        )
        backend.reports = types.SimpleNamespace(snapshot=snapshot, close=lambda: None)
        backend.gripper = types.SimpleNamespace(
            error=None,
            status=None,
            started_at=0,
            submit=lambda *a: None,
            close=lambda: None,
        )
        original_step = backend.step

        def step(*args):
            backend.gripper.status = {"received_at": now[0]}
            return original_step(*args)

        backend.step = step
        backend.connect = lambda *a: None
        rows = []
        with (
            patch(
                "robot_teleop.backends.real.xarm_sdk.replay.time.monotonic",
                lambda: now[0],
            ),
            patch(
                "socket.create_connection", side_effect=AssertionError("No hardware")
            ),
        ):
            run(
                reference(),
                "joint_position",
                backend,
                clock=lambda: now[0],
                sleep=lambda dt: now.__setitem__(0, now[0] + dt),
                log=rows.append,
            )
        steps = [r for r in rows if "feedback" in r]
        self.assertGreater(
            max(np.linalg.norm(r["feedback"]["velocity"]) for r in steps), 0.1
        )
        self.assertLess(
            max(max(abs(np.array(r["feedback"]["joint_error_rad"]))) for r in steps),
            0.15,
        )
        self.assertGreater(steps[-1]["elapsed_s"], 6.0)
        self.assertLess(steps[-1]["elapsed_s"], 6.5)  # default is 1x, not 24s
        self.assertTrue(
            all(np.linalg.norm(r["feedback"]["velocity"]) <= 0.5 + 1e-9 for r in steps)
        )
