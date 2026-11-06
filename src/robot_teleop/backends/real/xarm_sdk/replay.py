"""Explicit xArm reference tracking. Never used by offline readers or teleop."""

import threading, time
import copy, math
import numpy as np
from scipy.spatial.transform import Rotation
from .safety import (
    check_feedback,
    check_site,
    check_homing_site,
    TeleopFault,
    vector_limit,
    slew,
)
from ....processing.contracts import State
from ....processing.actions.cartesian import rotation


class PositionGripper:
    def __init__(self, channel, config):
        self.channel = channel
        self.config = config
        self.lock = threading.Lock()
        self.pending = None
        self.error = None
        self.last = None
        self.status = None
        self.started_at = time.monotonic()
        self.stop = threading.Event()
        self.thread = None

    def start(self):
        self.thread = threading.Thread(target=self._run, name="replay-gripper")
        self.thread.start()

    def submit(self, g, deadline):
        if not np.isfinite(g) or not 0 <= g <= 1:
            raise ValueError("Invalid gripper position")
        with self.lock:
            self.pending = (
                round(
                    self.config["open_pulse"]
                    + g * (self.config["close_pulse"] - self.config["open_pulse"])
                ),
                deadline,
            )

    def _run(self):
        try:
            while not self.stop.is_set():
                status = self.channel.gripper_status()
                now = time.monotonic()
                self.status = dict(status, received_at=now)
                if status["error_code"]:
                    raise TeleopFault("Replay gripper fault")
                with self.lock:
                    pending = self.pending
                if self.stop.is_set():
                    break
                if pending:
                    pulse, deadline = pending
                    if now >= deadline:
                        raise TeleopFault("Replay gripper target expired")
                    if (
                        pulse != self.last
                        and deadline - now > self.config["socket_timeout_s"]
                    ):
                        self.channel.gripper_target(pulse)
                        self.last = pulse
                self.stop.wait(1 / self.config["poll_hz"])
        except Exception as e:
            self.error = e
        finally:
            try:
                status = self.channel.gripper_status()
                self.channel.gripper_target(status["position_pulse"])
            except Exception as e:
                self.error = self.error or e

    def close(self):
        self.stop.set()
        if self.thread:
            self.thread.join(10 * self.config["socket_timeout_s"] + 0.2)
        self.channel.close()
        if self.thread and self.thread.is_alive():
            self.thread.join(0.2)
        if self.thread and self.thread.is_alive():
            raise TeleopFault("Replay gripper did not stop")
        if self.error:
            raise TeleopFault(str(self.error))


class XArmReplay:
    def __init__(
        self,
        config,
        channel_factory=None,
        report_factory=None,
        *,
        tracking_limits=None,
        prepare_start=False,
        prepare_gripper=False,
        prepare_timeout_s=None,
        log=lambda row: None,
    ):
        from .transport import SDKChannel, ReportReceiver

        from robot_teleop.replay.limits import joint_limits

        self.tracking_limits = joint_limits(tracking_limits)
        self.config = config
        self.prepare_start = prepare_start
        self.prepare_gripper = prepare_gripper
        self.prepare_timeout_s = (
            prepare_timeout_s
            if prepare_timeout_s is not None
            else config.robot["homing"]["timeout_s"]
        )
        if not math.isfinite(self.prepare_timeout_s) or self.prepare_timeout_s <= 0:
            raise ValueError("Preparation timeout must be positive and finite")
        if prepare_gripper and not prepare_start:
            raise ValueError("Gripper preparation requires prepare_start")
        self.log = log
        self.make_channel = channel_factory or SDKChannel
        self.make_reports = report_factory or ReportReceiver
        self.channel = self.reports = self.gripper = None
        self.active = False
        self.previous = None

    def preflight(self, states, mode, speed):
        from robot_teleop.replay.limits import preflight

        return preflight(states, mode, speed, self.tracking_limits)

    def _state(self, f, g):
        p = f.tcp_mm_rad
        quat = Rotation.from_euler("xyz", p[3:]).as_quat()[[3, 0, 1, 2]]
        c = self.config.gripper
        return State(
            f.received_at,
            list(f.joints_rad),
            [v / 1000 for v in p[:3]],
            quat.tolist(),
            (g - c["open_pulse"]) / (c["close_pulse"] - c["open_pulse"]),
            g,
        )

    def connect(self, initial, mode):
        c = self.config
        r = c.robot
        self.mode = 4 if "joint" in mode else 5
        read = self.make_channel(r["ip"], r["socket_timeout_s"], writable=False)
        try:
            if tuple(read.identity()["firmware"]) < (1, 8, 0):
                raise TeleopFault("Unsupported finite-duration firmware")
            self.reports = self.make_reports(r["ip"], r["socket_timeout_s"])
            self.reports.start()
            f = self.reports.wait_first(2)
            check_feedback(f, time.monotonic(), r["feedback_timeout_s"])
            check_site(c, f, initial=False)
            initial_gripper = read.gripper_status()
            if initial_gripper["error_code"]:
                raise TeleopFault("Replay initial gripper fault")
            g = initial_gripper["position_pulse"]
            s = self._state(f, g)
            from robot_teleop.robots import ROBOTS

            if len(initial.joints) != 6 or any(
                not lo <= q <= hi
                for q, (lo, hi) in zip(initial.joints, ROBOTS["xarm6"].position_limits)
            ):
                raise TeleopFault("Replay initial target exceeds joint limits")
            if self.prepare_start:
                if abs(initial.gripper_pulse - g) > 15 and not self.prepare_gripper:
                    raise TeleopFault(
                        "Initial gripper mismatch; set manually or explicitly use --prepare-gripper"
                    )
            else:
                self._check_initial(initial, s)
        finally:
            read.close()
        self.channel = self.make_channel(r["ip"], r["socket_timeout_s"], writable=True)
        self.active = True
        if self.prepare_start:
            self.log(
                dict(
                    event="prepare_start",
                    timestamp=time.monotonic(),
                    target_joints=initial.joints,
                )
            )
            print(
                "Preparing to episode first state: low-speed joint motion; replay clock has not started.",
                flush=True,
            )
            self._activate(4)
            self._prepare_arm(initial)
            if self.prepare_gripper:
                self._prepare_tool(initial)
            # Query the final gripper independently; no implicit gripper motion.
            tool = self.make_channel(
                r["ip"], c.gripper["socket_timeout_s"], writable=False
            )
            try:
                status = tool.gripper_status()
                if status["error_code"]:
                    raise TeleopFault("Preparation gripper fault")
            finally:
                tool.close()
            f = self.reports.snapshot()
            check_feedback(
                f,
                time.monotonic(),
                r["feedback_timeout_s"],
                running=True,
                expected_mode=4,
            )
            check_site(c, f, initial=False)
            self._check_initial(initial, self._state(f, status["position_pulse"]))
            self.log(
                dict(
                    event="prepare_complete",
                    timestamp=time.monotonic(),
                    feedback=f.to_dict(),
                )
            )
            print("First state reached and checked. Starting replay.", flush=True)
        self._activate(self.mode)
        self.previous = None
        gc = self.make_channel(r["ip"], c.gripper["socket_timeout_s"], writable=True)
        self.gripper = PositionGripper(gc, c.gripper)
        self.gripper.start()

    @staticmethod
    def _check_initial(initial, measured):
        if (
            max(abs(np.asarray(initial.joints) - measured.joints)) > 0.02
            or np.linalg.norm(np.asarray(initial.position) - measured.position) > 0.005
            or (rotation(initial) * rotation(measured).inv()).magnitude() > 0.05
            or abs(initial.gripper_pulse - measured.gripper_pulse) > 15
        ):
            raise TeleopFault(
                "Replay initial state mismatch; position robot manually before retrying"
            )

    def _activate(self, mode):
        r = self.config.robot
        before = self.reports.snapshot()
        if (
            before is None
            or not 0 <= time.monotonic() - before.received_at <= r["feedback_timeout_s"]
        ):
            raise TeleopFault("Replay activation feedback missing or stale")
        if mode == 4:
            self.channel.activate_homing()
        else:
            self.channel.activate(self.config.site["max_linear_accel_mps2"])
        deadline = time.monotonic() + 1.0
        while True:
            f = self.reports.snapshot()
            now = time.monotonic()
            if (
                f
                and f.sequence > before.sequence
                and f.mode == mode
                and f.state in (0, 1, 2)
            ):
                check_feedback(
                    f, now, r["feedback_timeout_s"], running=True, expected_mode=mode
                )
                return
            if now >= deadline:
                raise TeleopFault("Replay activation timeout")
            time.sleep(0.005)

    def _prepare_arm(self, initial):
        from .homing import JointHomer

        c = copy.deepcopy(self.config)
        c.site["initial_joints_rad"] = list(initial.joints)
        controller = JointHomer(c)
        r = c.robot
        settings = r["homing"]
        period = 1 / r["control_hz"]
        deadline = time.monotonic() + self.prepare_timeout_s
        settled = None
        previous_tick = time.monotonic()
        while True:
            f = self.reports.snapshot()
            now = time.monotonic()
            if now >= deadline:
                raise TeleopFault("Preparation timed out before first state")
            if now - previous_tick > r["input_timeout_s"]:
                raise TeleopFault("Preparation scheduler stalled")
            previous_tick = now
            check_feedback(
                f, now, r["feedback_timeout_s"], running=True, expected_mode=4
            )
            check_homing_site(c, f)
            available = min(
                r["command_duration_s"], f.received_at + r["feedback_timeout_s"] - now
            )
            if available <= r["socket_timeout_s"] + period:
                # Do not let the controller accumulate a fictitious long dt while
                # waiting for the next rich report. Position error is recomputed.
                controller.last_time = now
                time.sleep(period)
                continue
            speeds, at_target = controller.step(f, now)
            sent = time.monotonic()
            duration = min(available, deadline - sent) - r["socket_timeout_s"]
            if duration <= 0:
                raise TeleopFault("Preparation timed out before send")
            self.channel.joint_velocity(speeds, duration)
            if time.monotonic() - sent > r[
                "socket_timeout_s"
            ] * 1.5 or time.monotonic() >= min(deadline, now + available):
                raise TeleopFault("Preparation send deadline exceeded")
            self.log(
                dict(
                    event="prepare_step",
                    timestamp=now,
                    joints=list(f.joints_rad),
                    velocity=list(speeds),
                    duration_s=duration,
                )
            )
            settled = (now if settled is None else settled) if at_target else None
            if settled is not None and now - settled >= settings["settle_s"]:
                self.channel.joint_velocity((0.0,) * 6, min(0.1, duration))
                return
            time.sleep(max(0.0, period - (time.monotonic() - now)))

    def _prepare_tool(self, initial):
        c = self.config
        g = c.gripper
        r = c.robot
        channel = self.make_channel(r["ip"], g["socket_timeout_s"], writable=True)
        issued = False
        settled = None
        timeout = g["startup_open"]["timeout_s"]
        deadline = time.monotonic() + timeout
        self.log(
            dict(
                event="prepare_gripper_start",
                timestamp=time.monotonic(),
                target_pulse=initial.gripper_pulse,
            )
        )
        try:
            issued = True
            channel.gripper_target(round(initial.gripper_pulse))
            while True:
                status = channel.gripper_status()
                f = self.reports.snapshot()
                now = time.monotonic()
                check_feedback(
                    f, now, r["feedback_timeout_s"], running=True, expected_mode=4
                )
                check_homing_site(c, f)
                if now >= deadline:
                    raise TeleopFault("Preparation gripper timed out")
                if status["error_code"]:
                    raise TeleopFault("Preparation gripper fault")
                if max(abs(np.asarray(initial.joints) - f.joints_rad)) > 0.02:
                    raise TeleopFault("Arm moved during gripper preparation")
                ready = abs(
                    status["position_pulse"] - initial.gripper_pulse
                ) <= 15 and status.get("motion_state") in ("stopped", "grasped")
                settled = (now if settled is None else settled) if ready else None
                if settled is not None and now - settled >= r["homing"]["settle_s"]:
                    return
                time.sleep(1 / g["poll_hz"])
        except BaseException:
            if issued:
                current = channel.gripper_status()
                channel.gripper_target(current["position_pulse"])
            raise
        finally:
            channel.close()

    def step(self, target, velocity, mode, dt):
        c = self.config
        r = c.robot
        f = self.reports.snapshot()
        now = time.monotonic()
        from robot_teleop.robots import ROBOTS

        if any(
            not lo <= q <= hi
            for q, (lo, hi) in zip(target.joints, ROBOTS["xarm6"].position_limits)
        ):
            raise TeleopFault("Replay target exceeds joint limits")
        check_feedback(
            f, now, r["feedback_timeout_s"], running=True, expected_mode=self.mode
        )
        if self.gripper.error:
            raise TeleopFault(str(self.gripper.error))
        if (
            now
            - (
                self.gripper.status["received_at"]
                if self.gripper.status
                else self.gripper.started_at
            )
            > 0.6
        ):
            raise TeleopFault("Replay gripper feedback stalled")
        measured = self._state(f, target.gripper_pulse)
        if self.mode == 4:
            error = np.asarray(target.joints) - measured.joints
            if max(abs(error)) > 0.15:
                index = int(np.argmax(abs(error)))
                self.log(
                    dict(
                        event="replay_tracking_fault",
                        reference_time=target.time,
                        joint=index + 1,
                        error_rad=float(error[index]),
                        threshold_rad=0.15,
                        target_joints=target.joints,
                        measured_joints=measured.joints,
                        feedback_age_s=now - f.received_at,
                    )
                )
                raise TeleopFault(
                    f"Replay joint tracking error: joint{index + 1} {error[index]:.4f} rad exceeds 0.15 rad"
                )
            requested = np.asarray(velocity + 2 * error)
            desired = np.asarray(
                vector_limit(requested, self.tracking_limits["joint_speed_radps"])
            )
            acceleration = self.tracking_limits["joint_accel_radps2"]
        else:
            pe = np.asarray(target.position) - measured.position
            re = (rotation(target) * rotation(measured).inv()).as_rotvec()
            if np.linalg.norm(pe) > 0.03 or np.linalg.norm(re) > 0.15:
                raise TeleopFault("Replay TCP tracking error")
            requested = np.r_[velocity[:3] + 2 * pe, velocity[3:] + 2 * re]
            desired = np.r_[
                vector_limit(velocity[:3] + 2 * pe, c.site["max_linear_speed_mps"]),
                vector_limit(velocity[3:] + 2 * re, c.site["max_angular_speed_radps"]),
            ]
            acceleration = None
        speed_limited = not np.allclose(requested, desired, atol=1e-10, rtol=0)
        bounded = desired.copy()
        if self.previous is None:
            self.previous = np.zeros_like(desired)
        if self.mode == 4:
            desired = np.array(slew(self.previous, desired, acceleration, dt))
        else:
            desired = np.r_[
                slew(
                    self.previous[:3], desired[:3], c.site["max_linear_accel_mps2"], dt
                ),
                slew(
                    self.previous[3:],
                    desired[3:],
                    c.site["max_angular_accel_radps2"],
                    dt,
                ),
            ]
        accel_limited = not np.allclose(bounded, desired, atol=1e-10, rtol=0)
        deadline = min(
            now + r["command_duration_s"], f.received_at + r["feedback_timeout_s"]
        )
        duration = deadline - time.monotonic() - r["socket_timeout_s"]
        sent = False
        if duration > 0:
            speeds = (
                desired if self.mode == 4 else np.r_[desired[:3] * 1000, desired[3:]]
            )
            started = time.monotonic()
            (self.channel.joint_velocity if self.mode == 4 else self.channel.velocity)(
                speeds.tolist(), duration
            )
            if (
                time.monotonic() >= deadline
                or time.monotonic() - started > r["socket_timeout_s"] * 1.5
            ):
                raise TeleopFault("Replay send deadline exceeded")
            sent = True
            self.previous = desired.copy()
        self.gripper.submit(target.gripper, now + r["command_duration_s"])
        return dict(
            sent=sent,
            velocity=desired.tolist(),
            requested_velocity=requested.tolist(),
            speed_limited=speed_limited,
            acceleration_limited=accel_limited,
            target_joints=target.joints,
            joint_error_rad=(np.asarray(target.joints) - measured.joints).tolist(),
            feedback_received_at=f.received_at,
            feedback_age_s=now - f.received_at,
            tracking_limits=self.tracking_limits,
            joints=list(f.joints_rad),
            tcp=list(f.tcp_mm_rad),
        )

    def close(self):
        errors = []
        if self.channel and self.active:
            try:
                errors.extend(self.channel.stop())
            except Exception as e:
                errors.append(str(e))
        self.active = False
        for name in ("gripper", "channel", "reports"):
            obj = getattr(self, name)
            setattr(self, name, None)
            if obj:
                try:
                    obj.close()
                except Exception as e:
                    errors.append(str(e))
        if errors:
            raise TeleopFault("; ".join(map(str, errors)))
