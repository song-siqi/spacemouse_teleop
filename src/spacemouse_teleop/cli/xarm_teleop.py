from __future__ import annotations

import argparse
import math
import signal
import sys
import time
from dataclasses import asdict, replace
from datetime import datetime, timezone

from spacemouse_teleop.backends.xarm_sdk.backend import XArmBackend
from spacemouse_teleop.backends.xarm_sdk.config import DEFAULT_CONFIG, XArmConfig
from spacemouse_teleop.backends.xarm_sdk.input import LatestInput, input_deadline, is_neutral, axes_neutral
from spacemouse_teleop.backends.xarm_sdk.safety import TeleopFault, VelocityLimiter
from spacemouse_teleop.recording.async_jsonl import AsyncJsonlRecorder
from spacemouse_teleop.spacemouse import TeleopCore


def build_parser():
    parser = argparse.ArgumentParser(description="SpaceMouse xArm6 teleop with an independent real-robot config.")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--ip", default=None)
    parser.add_argument("--backend", choices=("mock", "pyspacemouse"), default=None, help="Override input reader only.")
    parser.add_argument("--dry-run", action="store_true", help="Offline command preview; never connect to or initialize a robot.")
    parser.add_argument("--duration", type=float, default=0., help="Run for this many seconds; 0 means until Ctrl-C.")
    parser.add_argument("--log", default="logs/xarm_teleop.jsonl")
    return parser


def process_input(core, raw, intent, config):
    # Real neutral is an immediate hold, including clearing EMA history so a
    # subsequent reversal cannot revive velocity from before the neutral sample.
    if axes_neutral(raw, config):
        command = replace(core.zero_command(raw.timestamp), enabled=True)
    else:
        command = core.process(raw)
    return replace(command, gripper_intent=intent if command.enabled else "hold")


def run_loop(config, source, backend, recorder, duration=0., dry_run=False):
    core, limiter = TeleopCore(config.teleop), VelocityLimiter(config)
    started = time.monotonic()
    period = 1 / config.robot["control_hz"]
    next_tick, previous_tick, next_print = started, None, started
    running = False
    last_print_intent = "hold"
    recorder.write({"event": "session_start", "timestamp": started, "dry_run": dry_run,
                    "wall_time_utc": datetime.now(timezone.utc).isoformat(), "config": asdict(config), "unverified_site_settings": config.missing_site_settings()})
    if config.robot.get("homing", {}).get("enabled", False):
        if dry_run:
            print("Dry-run: startup gripper opening and homing skipped; no robot connection.")
        else:
            if config.gripper['enabled']:
                print(f"Opening gripper first (timeout {config.gripper['startup_open']['timeout_s']:g} s); joint homing waits for confirmed opening.")
            target_deg = tuple(round(math.degrees(v), 2) for v in config.site["initial_joints_rad"])
            print(f"Homing to initial joints {target_deg} deg; timeout {config.robot['homing']['timeout_s']:g} s; SpaceMouse control starts after reset.")
            backend.home(recorder, lambda: source.snapshot(time.monotonic()))
            print("Homing complete. Center the SpaceMouse and release both buttons.")
        started = time.monotonic()  # The demo duration starts after homing.
        next_tick, previous_tick, next_print = started, None, started
    print("Waiting for a SpaceMouse report with all axes centered and both buttons released.")
    while True:
        now = time.monotonic()
        if duration and now - started >= duration:
            if not running:
                raise TeleopFault("Duration ended before a neutral input was observed")
            return
        if previous_tick is not None and running and now - previous_tick > config.robot["input_timeout_s"]:
            raise TeleopFault("Control loop stalled beyond input timeout")
        previous_tick = now
        raw, intent = source.snapshot(now)
        if raw is not None:
            now = raw.timestamp
            expires_at = input_deadline(raw, now, config, intent)
            if not running:
                if expires_at is not None and is_neutral(raw, config):
                    if not dry_run:
                        backend.activate()
                    previous_tick = time.monotonic()
                    core.zero_command(previous_tick)
                    running = True
                    startup_feedback = None if dry_run else getattr(backend, "startup_feedback", None)
                    recorder.write({"event": "teleop_started", "timestamp": time.monotonic(),
                                    "startup_feedback": None if startup_feedback is None else startup_feedback.to_dict()})
                    print("Dry-run active (no robot connection)." if dry_run else "Teleop active; Ctrl-C to stop.")
            elif expires_at is None:
                raise TeleopFault("Input report metadata disappeared")
            else:
                feedback = None if dry_run else backend.feedback()
                command = process_input(core, raw, intent, config)
                neutral = axes_neutral(raw, config)
                speeds = limiter.convert(command, now, expires_at - now, feedback=feedback, neutral=neutral)
                if dry_run:
                    sent = {"would_send_mm_s_rad_s": speeds, "duration_s": expires_at - now, "gripper_intent": intent}
                else:
                    sent = backend.send(speeds, expires_at, intent)
                gripper_feedback = None if dry_run or backend.gripper is None else backend.gripper.snapshot()
                recorder.write({"mode": "xarm-sdk-dry-run" if dry_run else "xarm-sdk-velocity", "raw": raw.to_dict(),
                                "command": command.to_dict(), "execution": sent, "feedback": None if feedback is None else feedback.to_dict(),
                                "gripper": gripper_feedback})
                if now >= next_print or intent != last_print_intent:
                    next_print = now + .2
                    last_print_intent = intent
                    gripper_text = "" if gripper_feedback is None else f" position={gripper_feedback['position_pulse']} target={gripper_feedback['target_pulse']} device_target={gripper_feedback.get('controller_target_pulse')} goal_seq={gripper_feedback.get('target_sequence')} state={gripper_feedback.get('motion_state', 'unknown')}"
                    print(f"{'preview' if dry_run else 'sent'} mm/s,rad/s={tuple(round(v, 4) for v in speeds)} gripper={intent}{gripper_text}")
        if not running and now - started >= config.robot["startup_timeout_s"]:
            raise TeleopFault("Startup timed out waiting for a reported neutral SpaceMouse state")
        next_tick += period
        after = time.monotonic()
        if next_tick < after:
            next_tick = after + period  # Skip missed slots, never burst old commands.
        time.sleep(max(0, next_tick - after))


def main(argv=None):
    args = build_parser().parse_args(argv)
    source = backend = recorder = None
    old_term = None
    status = 0
    reason = "completed"
    try:
        if not math.isfinite(args.duration) or args.duration < 0:
            raise ValueError("duration must be finite and nonnegative")
        config = XArmConfig.load(args.config, ip=args.ip)
        reader_backend = args.backend or config.reader["backend"]
        if not args.dry_run:
            config.require_live_settings()  # Fail before opening HID/network or enabling anything.
            if reader_backend != "pyspacemouse":
                raise ValueError("Mock input is allowed only with --dry-run")
        else:
            print("Dry-run: no robot writes or connections; site/feedback checks require later live validation.")
        linear_limit = min(config.teleop.max_linear_speed_mps, config.site["max_linear_speed_mps"] or config.teleop.max_linear_speed_mps)
        angular_limit = min(config.teleop.max_angular_speed_radps, config.site["max_angular_speed_radps"] or config.teleop.max_angular_speed_radps)
        print(f"Limits: {linear_limit * 1000:g} mm/s, {angular_limit:g} rad/s; gripper={'enabled' if config.gripper['enabled'] else 'disabled'}.")
        if config.gripper['enabled']:
            g = config.gripper
            print(f"Gripper: hold button {config.teleop.gripper_open_button} to open, button {config.teleop.gripper_close_button} to close; release/both to hold. Range {g['close_pulse']}..{g['open_pulse']} pulse; send endpoint once per held direction, device speed={g['speed_pulse_s']}.")
        bounds = [config.site[k] for k in ("workspace_min_m", "workspace_max_m")]
        if not config.site.get("workspace_limits_enabled", True):
            print("TCP workspace: no additional software position boundary.")
        elif all(bound is not None for bound in bounds):
            print(f"TCP workspace mm: min={tuple(round(v * 1000, 3) for v in bounds[0])}, max={tuple(round(v * 1000, 3) for v in bounds[1])}")
        if config.robot.get("homing", {}).get("enabled", False):
            prefix = "open gripper and confirm, then " if config.gripper['enabled'] else ""
            print(f"Startup: {prefix}home to configured initial joints, then wait for neutral input.")
        else:
            print("Start pose: match configured joints.")
        recorder = AsyncJsonlRecorder(args.log)
        recorder.open()
        if not args.dry_run:
            backend = XArmBackend(config)
            backend.connect()
        source = LatestInput(config, backend=reader_backend)
        source.start()
        if hasattr(signal, "SIGTERM"):
            old_term = signal.getsignal(signal.SIGTERM)
            def terminate(signum, frame):
                raise KeyboardInterrupt
            signal.signal(signal.SIGTERM, terminate)
        run_loop(config, source, backend, recorder, args.duration, args.dry_run)
    except KeyboardInterrupt:
        reason = "interrupted"
        print("Stopped.")
    except Exception as exc:
        reason, status = str(exc), 1
        print(f"Teleop failed: {exc}", file=sys.stderr)
    finally:
        # Stop the robot first, before thread joins or log flushing.
        if backend:
            try:
                errors = backend.close()
                if errors:
                    status = 1
                    print("Stop acknowledgement failed: " + "; ".join(errors), file=sys.stderr)
            except Exception as exc:
                status = 1
                print(f"Robot cleanup failed: {exc}", file=sys.stderr)
        if source:
            source.close()
        if recorder:
            try:
                try:
                    recorder.write({"event": "session_end", "timestamp": time.monotonic(), "reason": reason, "status": status})
                finally:
                    recorder.close()
            except Exception as exc:
                status = 1
                print(f"Log cleanup failed: {exc}", file=sys.stderr)
        if old_term is not None:
            signal.signal(signal.SIGTERM, old_term)
    return status


if __name__ == "__main__":
    raise SystemExit(main())
