"""CLI: print Onero H1 LeRobot observations."""

from __future__ import annotations

import argparse
import time
from pprint import pprint

import numpy as np

from onero_h1_lerobot import OneroH1Config, OneroH1Robot


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Print cached observations from an Onero H1 robot.")
    parser.add_argument("--id", default="onero_h1", help="LeRobot robot id")
    parser.add_argument("--no-cameras", action="store_true", help="Disable camera subscriptions")
    parser.add_argument(
        "--cameras",
        default="head,left,right",
        help="Comma-separated camera names from config.camera_topics",
    )
    parser.add_argument("--rate", type=float, default=1.0, help="Print rate in Hz")
    parser.add_argument("--count", type=int, default=0, help="Number of samples to print; 0 means forever")
    return parser.parse_args()


def summarize_value(value):
    if isinstance(value, np.ndarray):
        return {"shape": value.shape, "dtype": str(value.dtype), "min": int(value.min()), "max": int(value.max())}
    return value


def main() -> None:
    args = parse_args()
    camera_names = tuple(name.strip() for name in args.cameras.split(",") if name.strip())
    config = OneroH1Config(id=args.id, use_cameras=not args.no_cameras, camera_names=camera_names)
    robot = OneroH1Robot(config)

    with robot:
        interval = 1.0 / max(args.rate, 1e-6)
        i = 0
        while args.count <= 0 or i < args.count:
            obs = robot.get_observation()
            pprint({key: summarize_value(value) for key, value in obs.items()})
            print("-" * 80)
            i += 1
            time.sleep(interval)


if __name__ == "__main__":
    main()
