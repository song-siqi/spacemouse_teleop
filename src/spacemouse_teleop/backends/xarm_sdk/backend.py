from __future__ import annotations

import threading
import time

from .safety import TeleopFault, check_feedback, check_site, check_homing_site
from .transport import SDKChannel, ReportReceiver


class GripperWorker:
    """Send one endpoint per held direction; release replaces it with a hold."""
    def __init__(self, channel, config, clock=time.monotonic):
        self.channel, self.config, self.clock = channel, config, clock
        self.pending = None
        self.last_sent = None
        self.target_sent_at = self.target_ack_received_at = None
        self.feedback = None
        self.error = None
        self._jogging = False
        self._applied_intent = "hold"
        self.target_sequence = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.thread = None
        self.started_at = self.clock()

    def start(self):
        self.thread = threading.Thread(target=self._run, name="xarm-gripper", daemon=True)
        self.thread.start()

    def submit(self, intent, expires_at):
        if intent not in ("open", "close", "hold"):
            raise TeleopFault(f"Unknown gripper intent: {intent}")
        with self._lock:
            self.pending = (intent, expires_at)

    def _send_target(self, pulse):
        self.target_sent_at = self.clock()
        self.channel.gripper_target(int(pulse))
        self.target_ack_received_at = self.clock()
        self.last_sent = int(pulse)
        self.target_sequence += 1

    def _hold(self, status, force=False):
        if self._jogging or force:
            self._send_target(status['position_pulse'])
            self._jogging = False
        self._applied_intent = 'hold'

    def tick(self):
        if self._stop.is_set():
            return
        status = dict(self.channel.gripper_status(), received_at=self.clock())
        if status["error_code"]:
            raise TeleopFault(f"Gripper fault: {status['error_code']}")
        # Re-read the latest button state after I/O, so release/reverse while
        # reading feedback takes effect before another target can be sent.
        with self._lock:
            pending = self.pending
        if self._stop.is_set():
            return
        intent, deadline = pending if pending is not None else ('hold', self.clock())
        expired = intent != 'hold' and self.clock() >= deadline
        if expired:
            self._hold(status)
            raise TeleopFault('Gripper held input expired')
        if intent == 'hold':
            self._hold(status)
        elif intent != self._applied_intent:
            if deadline - self.clock() <= self.config['socket_timeout_s']:
                self._hold(status)
                raise TeleopFault('Gripper held input expired before send')
            position = status['position_pulse']
            endpoint = self.config[f'{intent}_pulse']
            direction = 1 if intent == 'open' else -1
            if direction * (endpoint - position) <= 0:
                # A new direction may start beyond its allowed endpoint. Cancel
                # an existing goal without commanding further into that boundary.
                self._hold(status, force=True)
            else:
                # The SDK writes a position target, not a streamed velocity.
                # Leave it unchanged while held; the device controls motion speed.
                # Set this before I/O so cleanup holds even if the ACK is lost.
                self._jogging = True
                self._send_target(endpoint)
            self._applied_intent = intent
        with self._lock:
            self.feedback = dict(status, target_pulse=self.last_sent, jog_intent=intent,
                                 target_sequence=self.target_sequence,
                                 target_sent_at=self.target_sent_at, target_ack_received_at=self.target_ack_received_at)

    def _run(self):
        try:
            while not self._stop.is_set():
                started = self.clock()
                self.tick()
                self._stop.wait(max(0, 1 / self.config["poll_hz"] - (self.clock() - started)))
        except Exception as exc:
            if not self._stop.is_set():
                self.error = exc
        finally:
            if self._jogging:
                try:
                    self._hold(self.channel.gripper_status())
                except Exception as exc:
                    self.error = self.error or TeleopFault(f'Gripper hold failed: {exc}')

    def snapshot(self):
        if self.error:
            raise TeleopFault(f"Gripper communication failed: {self.error}")
        with self._lock:
            feedback = self.feedback
        received = self.started_at if feedback is None else feedback["received_at"]
        budget = 5 * self.config["socket_timeout_s"] + 2 / self.config["poll_hz"]
        if self.clock() - received > budget:
            raise TeleopFault("Gripper feedback/polling stalled")
        return feedback

    def cancel(self):
        self._stop.set()
        with self._lock:
            self.pending = None

    def close(self):
        self.cancel()
        if self.thread:
            # The arm has already been stopped by backend.close. Give this
            # independent worker bounded time to send a current-position hold.
            self.thread.join(10 * self.config['socket_timeout_s'] + .2)
        else:
            if self._jogging:
                try:
                    self._hold(self.channel.gripper_status())
                except Exception as exc:
                    self.error = self.error or exc
        self.channel.close()
        if self.thread and self.thread.is_alive():
            self.thread.join(.2)
            if self.thread.is_alive():
                raise TeleopFault('Gripper worker did not stop within its I/O budget')
        if self.error:
            raise TeleopFault(str(self.error))

class XArmBackend:
    def __init__(self, config, channel_factory=SDKChannel, report_factory=ReportReceiver):
        self.config = config
        self.channel_factory, self.report_factory = channel_factory, report_factory
        self.channel = self.reports = self.gripper = None
        self.identity = None
        self.control_mode = 5
        self.startup_feedback = None
        self.armed = False

    def connect(self):
        r = self.config.robot
        try:
            self.channel = self.channel_factory(r["ip"], r["socket_timeout_s"], writable=False)
            self.identity = self.channel.identity()
            firmware = self.identity["firmware"]
            if firmware is None or tuple(firmware) < (1, 8, 0):
                raise TeleopFault("Firmware must support finite velocity duration (>=1.8.0)")
            self.reports = self.report_factory(r["ip"], r["socket_timeout_s"])
            self.reports.start()
            feedback = self.reports.wait_first(1.)
            check_feedback(feedback, time.monotonic(), r["feedback_timeout_s"])
            return feedback
        except Exception:
            self.close()
            raise

    def _open_gripper_before_home(self, recorder, input_guard):
        g = self.config.gripper
        settings = g['startup_open']
        started = time.monotonic()
        deadline = started + settings['timeout_s']
        channel = None
        issued = False

        def guard():
            input_guard()
            check_homing_site(self.config, self.feedback(running=False))
            if time.monotonic() >= deadline:
                raise TeleopFault('Startup gripper opening timed out; arm homing was not started')

        recorder.write({'event': 'startup_gripper_open_started', 'timestamp': started,
                        'target_pulse': g['open_pulse'], 'timeout_s': settings['timeout_s']})
        try:
            guard()
            channel = self.channel_factory(self.config.robot['ip'], g['socket_timeout_s'], writable=True)
            channel.initialize_gripper(g)
            guard()  # Initialization includes bounded I/O; recheck before motion.
            issued = True  # An ACK can be lost after the device accepted the goal.
            sent_at = time.monotonic()
            channel.gripper_target(g['open_pulse'])
            recorder.write({'event': 'startup_gripper_open_sent', 'timestamp': time.monotonic(),
                            'target_pulse': g['open_pulse'], 'send_started_at': sent_at})
            settled_since = None
            while True:
                guard()
                status = channel.gripper_status()
                guard()  # Do not accept completion after stale feedback/deadline.
                if status['error_code']:
                    raise TeleopFault(f"Startup gripper error: {status['error_code']}")
                now = time.monotonic()
                recorder.write({'event': 'startup_gripper_open_feedback', 'timestamp': now, 'gripper': status})
                tolerance = settings['tolerance_pulse']
                opened = (status['motion_state'] == 'stopped'
                          and abs(status['position_pulse'] - g['open_pulse']) <= tolerance
                          and abs(status['controller_target_pulse'] - g['open_pulse']) <= tolerance)
                settled_since = (now if settled_since is None else settled_since) if opened else None
                if settled_since is not None and now - settled_since >= settings['settle_s']:
                    recorder.write({'event': 'startup_gripper_open_completed', 'timestamp': now, 'gripper': status})
                    return
                time.sleep(min(1 / g['poll_hz'], max(0., deadline - time.monotonic())))
        except BaseException as exc:
            hold_error = None
            if issued and channel is not None:
                try:
                    position = channel.gripper_status()['position_pulse']
                    channel.gripper_target(position)
                except Exception as cleanup_exc:
                    hold_error = str(cleanup_exc)
            recorder.write({'event': 'startup_gripper_open_failed', 'timestamp': time.monotonic(),
                            'reason': str(exc) or type(exc).__name__, 'hold_error': hold_error})
            if hold_error:
                raise TeleopFault(f'Startup gripper opening failed: {exc}; hold failed: {hold_error}') from exc
            raise
        finally:
            if channel is not None:
                channel.close()

    def home(self, recorder, input_guard):
        from .homing import JointHomer
        self.config.require_live_settings()
        r, settings = self.config.robot, self.config.robot["homing"]
        initial = self.feedback(running=False)
        check_homing_site(self.config, initial)
        input_guard()
        if self.config.gripper['enabled']:
            self._open_gripper_before_home(recorder, input_guard)
            # Opening may take several seconds. Use fresh robot feedback before
            # enabling any joint motion; the arm timeout starts after opening.
            initial = self.feedback(running=False)
            check_homing_site(self.config, initial)
            input_guard()
        controller = JointHomer(self.config)
        started = time.monotonic()
        recorder.write({"event": "homing_started", "timestamp": started,
                        "target_joints_rad": self.config.site["initial_joints_rad"], "feedback": initial.to_dict()})
        self.channel.close()
        self.channel = self.channel_factory(r["ip"], r["socket_timeout_s"], writable=True)
        self.control_mode, self.armed = 4, True
        try:
            self.channel.activate_homing()
            mode_deadline = time.monotonic() + 1.
            while True:
                input_guard()
                f = self.reports.snapshot()
                if f.sequence > initial.sequence and f.mode == 4 and f.state in (0, 1, 2):
                    self.feedback()
                    break
                if time.monotonic() >= mode_deadline:
                    raise TeleopFault("Robot did not enter homing control")
                time.sleep(.005)
            next_tick, settled_since = time.monotonic(), None
            waiting_feedback_sequence = None
            period = 1 / r["control_hz"]
            while True:
                now = time.monotonic()
                if now - started >= settings["timeout_s"]:
                    raise TeleopFault("Homing timed out before reaching initial joints")
                input_guard()
                f = self.feedback()
                check_homing_site(self.config, f)
                now = time.monotonic()
                available = min(r["command_duration_s"], r["feedback_timeout_s"] - (now - f.received_at))
                if available <= r["socket_timeout_s"] + period:
                    # Rich feedback can arrive at ~5 Hz. An otherwise valid old
                    # report may leave no useful lease after the transit reserve.
                    # Wait within the existing feedback deadline instead of
                    # attempting an expired send; the previous velocity expires
                    # on the controller if no new report arrives.
                    if waiting_feedback_sequence != f.sequence:
                        recorder.write({"event": "homing_waiting_feedback", "timestamp": now,
                                        "feedback_sequence": f.sequence, "feedback_age_s": now - f.received_at,
                                        "available_lease_s": available})
                        waiting_feedback_sequence = f.sequence
                    time.sleep(period)
                    next_tick = time.monotonic()
                    continue
                speeds, at_target = controller.step(f, now)
                sent = self._send_velocity(speeds, now + available)
                recorder.write({"event": "homing_step", "timestamp": now, "joint_speeds_rad_s": speeds,
                                "execution": sent, "feedback": f.to_dict()})
                settled_since = (now if settled_since is None else settled_since) if at_target else None
                if settled_since is not None and now - settled_since >= settings["settle_s"]:
                    break
                next_tick = max(next_tick + 1 / r["control_hz"], time.monotonic())
                time.sleep(max(0, next_tick - time.monotonic()))
            errors = self.stop()
            if errors:
                raise TeleopFault("Homing stop failed: " + "; ".join(errors))
            stop_deadline = time.monotonic() + 1.
            while True:
                final = self.reports.snapshot()
                if final.sequence > f.sequence and final.state == 4:
                    check_feedback(final, time.monotonic(), r["feedback_timeout_s"])
                    check_site(self.config, final)
                    recorder.write({"event": "homing_completed", "timestamp": time.monotonic(), "feedback": final.to_dict()})
                    return
                if time.monotonic() >= stop_deadline:
                    raise TeleopFault("Homing stop feedback missing")
                time.sleep(.005)
        except BaseException:
            self.stop()
            raise

    def activate(self):
        self.config.require_live_settings()
        r = self.config.robot
        feedback = self.feedback(running=False)
        check_site(self.config, feedback)
        self.startup_feedback = feedback
        self.channel.close()
        self.channel = self.channel_factory(r["ip"], r["socket_timeout_s"], writable=True)
        self.armed = True  # Also clean up when only part of initialization succeeds.
        try:
            self.control_mode = 5
            self.channel.activate(self.config.site["max_linear_accel_mps2"])
            if self.config.gripper["enabled"]:
                gchannel = self.channel_factory(r["ip"], self.config.gripper["socket_timeout_s"], writable=True)
                self.gripper = GripperWorker(gchannel, self.config.gripper)
                gchannel.initialize_gripper(self.config.gripper)
                self.gripper.start()
            deadline = time.monotonic() + 1.
            while time.monotonic() < deadline:
                candidate = self.reports.snapshot()
                if candidate.sequence > feedback.sequence and candidate.mode == 5 and candidate.state in (0, 1, 2):
                    self.feedback()
                    return
                time.sleep(.005)
            raise TeleopFault("Robot did not enter velocity control")
        except Exception:
            self.stop()
            raise

    def feedback(self, running=True):
        result = self.reports.snapshot()
        check_feedback(result, time.monotonic(), self.config.robot["feedback_timeout_s"], running=running and self.armed, expected_mode=self.control_mode)
        if self.armed:
            check_site(self.config, result, initial=False)
        if self.gripper:
            self.gripper.snapshot()
        return result

    def send(self, speeds, expires_at, intent="hold"):
        if self.control_mode != 5:
            raise TeleopFault("Teleop cannot send before homing completes")
        return self._send_velocity(speeds, expires_at, intent)

    def _send_velocity(self, speeds, expires_at, intent="hold"):
        import math
        if len(speeds) != 6 or not all(math.isfinite(v) for v in speeds) or not math.isfinite(expires_at):
            raise TeleopFault("Invalid serialized velocity command")
        if not self.armed:
            raise TeleopFault("Backend has not been activated")
        self.feedback()
        duration = expires_at - time.monotonic()
        # Reserve one bounded transaction for transit/ack; never send duration<=0.
        duration -= self.config.robot["socket_timeout_s"]
        if duration <= 0:
            raise TeleopFault("Velocity command expired before send")
        started = time.monotonic()
        send = self.channel.joint_velocity if self.control_mode == 4 else self.channel.velocity
        send(speeds, min(duration, self.config.robot["command_duration_s"]))
        finished = time.monotonic()
        if finished - started > self.config.robot["socket_timeout_s"] * 1.5 or finished >= expires_at:
            raise TeleopFault("Velocity send missed its deadline")
        if self.gripper:
            self.gripper.submit(intent, expires_at)
        return {("joint_speeds_rad_s" if self.control_mode == 4 else "speeds_mm_s_rad_s"): speeds, "duration_s": duration, "send_started_at": started, "ack_received_at": finished, "return_code": 0}

    def stop(self):
        if self.gripper:
            self.gripper.cancel()
        errors = []
        if self.armed and self.channel:
            try:
                errors = self.channel.stop()
            except Exception as exc:
                errors = [str(exc)]
            finally:
                self.armed = False
        return errors

    def close(self):
        errors = self.stop()
        for name in ("gripper", "channel", "reports"):
            resource = getattr(self, name)
            if resource:
                try:
                    resource.close()
                except Exception as exc:
                    errors.append(f"{name} close: {exc}")
                setattr(self, name, None)
        return errors
