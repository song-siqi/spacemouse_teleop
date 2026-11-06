import argparse, json


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("dataset")
    a = p.parse_args(argv)
    from robot_teleop.processing.validation import check_dataset

    print(json.dumps(check_dataset(a.dataset), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
