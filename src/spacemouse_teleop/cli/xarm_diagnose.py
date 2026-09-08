from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone

from spacemouse_teleop.backends.xarm_sdk.config import DEFAULT_CONFIG, XArmConfig
from spacemouse_teleop.backends.xarm_sdk.transport import SDKChannel, ReportReceiver


def main(argv=None):
    parser = argparse.ArgumentParser(description="Read-only xArm identity, feedback, TCP and payload diagnostics.")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--ip", default=None)
    parser.add_argument("--gripper", action="store_true", help="Also query the attached xArm Gripper; never initialize or change its baudrate.")
    args = parser.parse_args(argv)
    channel = reports = None
    try:
        config = XArmConfig.load(args.config, ip=args.ip)
        r = config.robot
        channel = SDKChannel(r["ip"], r["socket_timeout_s"], writable=False)
        identity = channel.identity()
        reports = ReportReceiver(r["ip"], r["socket_timeout_s"])
        reports.start()
        feedback = reports.wait_first(1.)
        result = {"observed_at_utc": datetime.now(timezone.utc).isoformat(), "read_only": True, "ip": r["ip"], **identity, "feedback": feedback.to_dict(),
                  "feedback_age_s": time.monotonic() - feedback.received_at,
                  "units": {"tcp_mm_rad": "mm, rad", "tcp_offset_mm_rad": "mm, rad", "joints_rad": "rad", "payload_kg_com_mm": "kg, mm"},
                  "missing_live_settings": config.missing_site_settings()}
        if args.gripper:
            result["gripper_baudrate"] = channel.gripper_baudrate()
            result["gripper"] = channel.gripper_status()
        print(json.dumps(result, indent=2, allow_nan=False))
        return 0
    except (RuntimeError, ValueError, OSError, KeyError) as exc:
        print(f"Diagnosis failed: {exc}", file=sys.stderr)
        return 1
    finally:
        if reports:
            reports.close()
        if channel:
            channel.close()


if __name__ == "__main__":
    raise SystemExit(main())
