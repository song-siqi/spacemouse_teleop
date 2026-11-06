import argparse, json
from pathlib import Path


def main(argv=None):
    p = argparse.ArgumentParser(
        description="Offline raw-to-LeRobot processing; never connects hardware"
    )
    p.add_argument("input", type=Path)
    p.add_argument("--output", required=True, type=Path)
    p.add_argument("--config")
    p.add_argument("--fps", type=int)
    p.add_argument("--action-mode")
    a = p.parse_args(argv)
    if a.output.exists():
        raise ValueError("Output must be a new directory")
    from robot_teleop.processing.config import load
    from robot_teleop.processing.pipeline import process_episode

    c = load(a.config, fps=a.fps, action_mode=a.action_mode)
    episodes = (
        [a.input]
        if (a.input / "meta.json").exists()
        else sorted((a.input / "episodes").glob("*"))
    )
    if not episodes:
        raise ValueError("No episodes")
    results = []
    for ep in episodes:
        try:
            m = json.loads((ep / "meta.json").read_text())
            if m.get("disposition") != "kept":
                results.append(dict(episode=str(ep), skipped=True, reason="not kept"))
                continue
            target = a.output if len(episodes) == 1 else a.output / ep.name
            results.append(dict(episode=str(ep), result=process_episode(ep, target, c)))
        except Exception as e:
            results.append(dict(episode=str(ep), error=str(e)))
    if len(episodes) > 1:
        a.output.mkdir(parents=True, exist_ok=True)
        (a.output / "batch_report.json").write_text(json.dumps(results, indent=2))
    print(json.dumps(results, indent=2))
    return int(any("error" in r for r in results))


if __name__ == "__main__":
    raise SystemExit(main())
