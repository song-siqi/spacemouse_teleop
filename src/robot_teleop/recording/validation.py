"""Read-only pre-commit validation in an owned, bounded subprocess group."""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from .visual import VideoRunner


def validate_profile(root, meta):
    profile = meta.get('validation_profile')
    if profile is None:
        return
    if profile != 'xarm':
        raise ValueError(f'Unknown validation profile: {profile}')
    from robot_teleop.backends.real.xarm_sdk.validation import validate_sources
    validate_sources(root, meta)


def validate_before_commit(path, deadline):
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise ValueError('Finalization timed out before validation')
    args = [sys.executable, '-m', 'robot_teleop.recording.validation', str(path), '--deadline', str(deadline)]
    process = subprocess.Popen(args, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, text=True, start_new_session=os.name == 'posix')
    try:
        output, error = process.communicate(timeout=remaining)
        if process.returncode:
            raise ValueError(error[-4000:] or output[-4000:] or f'Validator exit {process.returncode}')
        return json.loads(output)
    finally:
        # Only this validator's process group, including its FFmpeg children.
        if process.poll() is None:
            if os.name == 'posix':
                os.killpg(process.pid, signal.SIGKILL)
            else:
                process.kill()
            process.communicate()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episode', type=Path)
    parser.add_argument('--deadline', type=float, required=True)
    args = parser.parse_args(argv)
    from .v4 import check_episode
    report = check_episode(args.episode, _allow_uncommitted=True, _runner=VideoRunner(args.deadline))
    print(json.dumps(report, allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
