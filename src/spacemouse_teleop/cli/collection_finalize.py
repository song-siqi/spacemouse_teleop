"""Retry video archival for a normally completed raw capture; no hardware access."""
from __future__ import annotations
import argparse
import json
import sys
from spacemouse_teleop.recording.archive import finalize_episode


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episode')
    args = parser.parse_args(argv)
    previous = None
    def progress(value):
        nonlocal previous
        if value['phase'] != previous:
            print(f"Archive: {value['phase']}", file=sys.stderr, flush=True)
            previous = value['phase']
    try:
        result = finalize_episode(args.episode,progress=progress)
    except (ValueError, OSError, KeyError, TypeError) as exc:
        print(json.dumps({'ok':False,'error':str(exc)},ensure_ascii=False))
        return 1
    print(json.dumps({'ok':True,'status':result['status'],'disposition':result['disposition'],
                      'archive':result['archive']},ensure_ascii=False,allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
