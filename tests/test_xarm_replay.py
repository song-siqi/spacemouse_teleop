import unittest, time
from dataclasses import replace
from unittest.mock import patch
import importlib.util

if importlib.util.find_spec("scipy") is None:
    raise unittest.SkipTest("Requires processing environment")
import numpy as np
from robot_teleop.backends.real.xarm_sdk.replay import XArmReplay, PositionGripper
from robot_teleop.backends.real.xarm_sdk.safety import TeleopFault
from test_xarm_config_safety import live_config, feedback
from robot_teleop.processing.contracts import State


class XArmReplayTest(unittest.TestCase):
    def backend(self):
        c = live_config()
        c.gripper.update(open_pulse=850, close_pulse=230)
        self.mode = 5
        self.sequence = 0
        self.calls = []
        owner = self

        class Reports:
            def __init__(self, *a):
                pass

            def start(self):
                pass

            def wait_first(self, *a):
                return self.snapshot()

            def snapshot(self):
                owner.sequence += 1
                return feedback(
                    time.monotonic(), mode=owner.mode, sequence=owner.sequence
                )

            def close(self):
                owner.calls.append("reports_close")

        class Channel:
            def __init__(self, *a, writable=False):
                self.writable = writable
                owner.calls.append(("connect", writable))

            def identity(self):
                return dict(firmware=(2, 5, 5))

            def gripper_status(self):
                return dict(position_pulse=850, error_code=0)

            def activate_homing(self):
                owner.mode = 4
                owner.calls.append("activate4")

            def activate(self, *a):
                owner.mode = 5
                owner.calls.append("activate5")

            def gripper_target(self, p):
                owner.calls.append(("gripper", p))

            def joint_velocity(self, v, d):
                owner.calls.append(("joint", v, d))

            def velocity(self, v, d):
                owner.calls.append(("tcp", v, d))

            def stop(self):
                owner.calls.append("stop")
                return []

            def close(self):
                owner.calls.append("close")

        return XArmReplay(c, Channel, Reports)

    def initial(self):
        return State(0, [0.0] * 6, [0.0] * 3, [1.0, 0.0, 0.0, 0.0], 0.0, 850.0)

    def test_initial_mismatch_does_not_enable(self):
        b = self.backend()
        with patch(
            "socket.create_connection", side_effect=AssertionError("hardware forbidden")
        ):
            with self.assertRaisesRegex(TeleopFault, "initial state"):
                b.connect(replace(self.initial(), joints=[0.1] * 6), "joint_position")
            b.close()
        self.assertNotIn(("connect", True), self.calls)

    def test_joint_and_tcp_limits_stop_and_no_release(self):
        for mode in ("joint_position", "ee_pose"):
            with self.subTest(mode=mode):
                b = self.backend()
                with patch(
                    "socket.create_connection",
                    side_effect=AssertionError("hardware forbidden"),
                ):
                    try:
                        b.connect(self.initial(), mode)
                        target = replace(
                            self.initial(), joints=[0.01] * 6, position=[0.001, 0, 0]
                        )
                        b.step(target, np.ones(6), mode, 1 / 60)
                        sent = [
                            c
                            for c in self.calls
                            if isinstance(c, tuple) and c[0] in ("joint", "tcp")
                        ]
                        self.assertEqual(len(sent), 1)
                        self.assertGreater(sent[0][2], 0)
                        self.assertLessEqual(sent[0][2], 0.2)
                        b.reports.snapshot = lambda: feedback(
                            time.monotonic() - 1, mode=self.mode
                        )
                        with self.assertRaisesRegex(TeleopFault, "stale"):
                            b.step(target, np.zeros(6), mode, 1 / 60)
                    finally:
                        b.close()
                self.assertIn("stop", self.calls)
                self.assertNotIn(("gripper", 230), self.calls)

    def test_tracking_error_and_slow_ack_fault(self):
        b = self.backend()
        try:
            b.connect(self.initial(), "joint_position")
            with self.assertRaisesRegex(TeleopFault, "tracking"):
                b.step(
                    replace(self.initial(), joints=[0.16] * 6),
                    np.zeros(6),
                    "joint_position",
                    1 / 60,
                )

            def slow(*args):
                time.sleep(0.09)

            b.channel.joint_velocity = slow
            with self.assertRaisesRegex(TeleopFault, "deadline"):
                b.step(self.initial(), np.zeros(6), "joint_position", 1 / 60)
        finally:
            b.close()

    def test_repeat_cleanup_has_no_worker_leak(self):
        import threading

        for _ in range(2):
            b = self.backend()
            b.connect(self.initial(), "joint_position")
            b.close()
            b.close()
        self.assertFalse(any(t.name == "replay-gripper" for t in threading.enumerate()))


class ReplayGripperTest(unittest.TestCase):
    def test_deduplicate_and_hold_on_exit(self):
        calls = []

        class Channel:
            def gripper_status(self):
                return dict(position_pulse=500, error_code=0)

            def gripper_target(self, p):
                calls.append(p)

            def close(self):
                pass

        c = live_config().gripper
        c.update(open_pulse=850, close_pulse=230, poll_hz=100)
        w = PositionGripper(Channel(), c)
        w.submit(0.5, time.monotonic() + 1)
        w.start()
        time.sleep(0.04)
        w.close()
        self.assertEqual(calls, [540, 500])


class PrepareStartTest(unittest.TestCase):
    backend = XArmReplayTest.backend
    initial = XArmReplayTest.initial

    def preparing(self, moving=True):
        b = self.backend()
        b.prepare_start = True
        self.now = [100.0]
        self.q = np.zeros(6)
        self.speed = np.zeros(6)
        self.last = 100.0
        base_reports = b.make_reports
        base_channel = b.make_channel
        owner = self

        class Reports(base_reports):
            def snapshot(self):
                dt = owner.now[0] - owner.last
                owner.last = owner.now[0]
                if moving:
                    owner.q += owner.speed * dt
                return replace(super().snapshot(), joints_rad=tuple(owner.q))

        class Channel(base_channel):
            def joint_velocity(self, v, d):
                owner.speed = np.asarray(v)
                super().joint_velocity(v, d)

        b.make_reports = Reports
        b.make_channel = Channel
        return b

    def clocks(self):
        import contextlib

        stack = contextlib.ExitStack()
        stack.enter_context(
            patch(
                "robot_teleop.backends.real.xarm_sdk.replay.time.monotonic",
                lambda: self.now[0],
            )
        )
        stack.enter_context(
            patch(
                "robot_teleop.backends.real.xarm_sdk.replay.time.sleep",
                lambda dt: self.now.__setitem__(0, self.now[0] + dt),
            )
        )
        stack.enter_context(
            patch(
                "socket.create_connection",
                side_effect=AssertionError("hardware forbidden"),
            )
        )
        return stack

    def test_moves_to_first_joint_state_before_tcp_activation(self):
        b = self.preparing()
        events = []
        b.log = events.append
        target = replace(self.initial(), joints=[0.05] * 6)
        with self.clocks():
            try:
                b.connect(target, "delta_ee_pose")
            finally:
                b.close()
        self.assertLess(max(abs(self.q - 0.05)), 0.005)
        self.assertLessEqual(
            max(
                abs(v)
                for c in self.calls
                if isinstance(c, tuple) and c[0] == "joint"
                for v in c[1]
            ),
            b.config.robot["homing"]["speed_radps"],
        )
        complete = next(e for e in events if e["event"] == "prepare_complete")
        self.assertGreater(
            complete["timestamp"] - events[0]["timestamp"],
            b.config.robot["homing"]["settle_s"],
        )
        self.assertLess(self.calls.index("activate4"), self.calls.index("activate5"))
        self.assertNotIn(("gripper", 230), self.calls)
        self.assertIn("stop", self.calls)

    def test_preparation_timeout_stops_without_replay(self):
        b = self.preparing(moving=False)
        b.prepare_timeout_s = 0.1
        with self.clocks():
            try:
                with self.assertRaisesRegex(TeleopFault, "Preparation timed out"):
                    b.connect(replace(self.initial(), joints=[0.05] * 6), "ee_pose")
            finally:
                b.close()
        self.assertNotIn("activate5", self.calls)
        self.assertIn("stop", self.calls)

    def test_gripper_mismatch_without_opt_in_does_not_move_arm(self):
        b = self.preparing()
        with self.clocks():
            try:
                with self.assertRaisesRegex(TeleopFault, "gripper mismatch"):
                    b.connect(
                        replace(self.initial(), gripper_pulse=500, gripper=0.5),
                        "joint_position",
                    )
            finally:
                b.close()
        self.assertNotIn(("connect", True), self.calls)

    def test_interruption_during_preparation_stops(self):
        b = self.preparing()
        with (
            self.clocks(),
            patch(
                "robot_teleop.backends.real.xarm_sdk.replay.time.sleep",
                side_effect=KeyboardInterrupt,
            ),
        ):
            try:
                with self.assertRaises(KeyboardInterrupt):
                    b.connect(replace(self.initial(), joints=[0.05] * 6), "ee_pose")
            finally:
                b.close()
        self.assertNotIn("activate5", self.calls)
        self.assertIn("stop", self.calls)

    def test_tcp_mismatch_after_joint_preparation_rejects_replay(self):
        b = self.preparing()
        with self.clocks():
            try:
                with self.assertRaisesRegex(TeleopFault, "initial state"):
                    b.connect(replace(self.initial(), position=[0.01, 0, 0]), "ee_pose")
            finally:
                b.close()
        self.assertNotIn("activate5", self.calls)

    def test_explicit_gripper_preparation_and_hold_on_failure(self):
        b = self.preparing()
        b.prepare_gripper = True
        base = b.make_channel
        owner = self
        owner.pulse = 850

        class Channel(base):
            def gripper_status(self):
                return dict(
                    position_pulse=owner.pulse, error_code=0, motion_state="stopped"
                )

            def gripper_target(self, p):
                owner.pulse = p
                super().gripper_target(p)

        b.make_channel = Channel
        with self.clocks():
            try:
                b.connect(
                    replace(self.initial(), gripper_pulse=540, gripper=0.5),
                    "joint_position",
                )
            finally:
                b.close()
        self.assertIn(("gripper", 540), self.calls)

    def test_invalid_options(self):
        from robot_teleop.cli.replay import main

        with self.assertRaises(SystemExit):
            main(["missing", "--prepare-start"])
        with self.assertRaises(SystemExit):
            main(["missing", "--mode", "robot", "--prepare-gripper"])
        with self.assertRaises(ValueError):
            XArmReplay(live_config(), prepare_timeout_s=float("nan"))

    def test_gripper_obstruction_times_out_and_holds(self):
        b = self.preparing()
        b.prepare_gripper = True
        b.config.gripper["startup_open"]["timeout_s"] = 0.1
        base = b.make_channel

        class Channel(base):
            def gripper_status(self):
                return dict(position_pulse=850, error_code=0, motion_state="stopped")

        b.make_channel = Channel
        with self.clocks():
            try:
                with self.assertRaisesRegex(TeleopFault, "gripper timed out"):
                    b.connect(
                        replace(self.initial(), gripper_pulse=540, gripper=0.5),
                        "ee_pose",
                    )
            finally:
                b.close()
        targets = [
            c[1] for c in self.calls if isinstance(c, tuple) and c[0] == "gripper"
        ]
        self.assertEqual(targets, [540, 850])
        self.assertIn("stop", self.calls)
        self.assertNotIn("activate5", self.calls)

    def test_stale_feedback_during_prepare_does_not_replay(self):
        b = self.preparing()
        base = b.make_reports
        owner = self

        class Reports(base):
            def snapshot(self):
                f = super().snapshot()
                return (
                    replace(f, received_at=f.received_at - 1) if owner.mode == 4 else f
                )

        b.make_reports = Reports
        with self.clocks():
            try:
                with self.assertRaisesRegex(TeleopFault, "stale"):
                    b.connect(self.initial(), "ee_pose")
            finally:
                b.close()
        self.assertNotIn("activate5", self.calls)
        self.assertIn("stop", self.calls)
