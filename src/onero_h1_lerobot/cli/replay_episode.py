"""CLI: replay a recorded LeRobot episode on the Onero H1 robot."""

from __future__ import annotations

import argparse
import logging
import time
from pathlib import Path

from onero_h1_lerobot import OneroH1Config, OneroH1Robot

_logger = logging.getLogger("onero_h1.replay")


def _setup_logging() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
        force=True,
    )


def _import_lerobot_dataset():
    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        from lerobot.utils.constants import ACTION
    except Exception as exc:
        raise RuntimeError("LeRobotDataset is not available. Install LeRobot first.") from exc
    return LeRobotDataset, ACTION


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Replay a recorded LeRobot episode on the Onero H1 robot.")
    parser.add_argument("--repo-id", required=True, help="Dataset repo id, e.g. user/onero_h1_test")
    parser.add_argument("--root", default=None, help="Optional local dataset root")
    parser.add_argument("--episode", type=int, default=0, help="Episode index to replay")
    parser.add_argument("--fps", type=int, default=None, help="Replay frame rate (default: use dataset fps)")
    parser.add_argument("--id", default="onero_h1", help="Robot id")
    parser.add_argument(
        "--arm-command-mode",
        default="record_data",
        choices=["record_data", "movej"],
        help="Arm command mode: record_data (Float64MultiArray) or movej (JSON)",
    )
    parser.add_argument(
        "--send-gripper",
        action="store_true",
        default=True,
        help="Send gripper commands during replay. Default on.",
    )
    parser.add_argument(
        "--no-gripper",
        action="store_false",
        dest="send_gripper",
        help="Disable gripper commands during replay.",
    )
    parser.add_argument("--no-cameras", action="store_true", help="Disable camera features")
    parser.add_argument("--cameras", default="head,left,right", help="Comma-separated camera names")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    _setup_logging()
    try:
        LeRobotDataset, ACTION = _import_lerobot_dataset()
    except RuntimeError as exc:
        raise SystemExit(
            f"ERROR: {exc}\nInstall LeRobot support with: python3 -m pip install -e '.[lerobot]'"
        ) from None

    camera_names = tuple(name.strip() for name in args.cameras.split(",") if name.strip())
    config = OneroH1Config(
        id=args.id,
        use_cameras=not args.no_cameras,
        camera_names=camera_names,
        send_action=True,
        send_gripper_action=args.send_gripper,
        arm_command_mode=args.arm_command_mode,
    )
    robot = OneroH1Robot(config)

    dataset = LeRobotDataset(args.repo_id, root=args.root, episodes=[args.episode])

    if ACTION not in dataset.features:
        raise SystemExit(f"ERROR: Dataset {args.repo_id} does not contain action features.")

    action_names = dataset.features[ACTION]["names"]
    fps = args.fps if args.fps is not None else dataset.fps
    period = 1.0 / max(fps, 1)

    _logger.info(
        "Replaying: repo=%s episode=%d frames=%d fps=%d",
        args.repo_id, args.episode, dataset.num_frames, fps,
    )

    with robot:
        for i in range(dataset.num_frames):
            start = time.perf_counter()

            action_array = dataset[i][ACTION]
            action = {}
            for j, name in enumerate(action_names):
                action[name] = float(action_array[j])

            robot.send_action(action)

            if i % max(1, fps) == 0 or i == dataset.num_frames - 1:
                _logger.info("Replayed frame %d/%d (%.1fs)", i + 1, dataset.num_frames, (i + 1) * period)
                print(f"Replayed frame {i + 1}/{dataset.num_frames}", end="\r")

            sleep_s = period - (time.perf_counter() - start)
            if sleep_s > 0:
                time.sleep(sleep_s)

    _logger.info("Replay finished: %d frames", dataset.num_frames)
    print(f"\nReplay finished: {dataset.num_frames} frames")


if __name__ == "__main__":
    main()