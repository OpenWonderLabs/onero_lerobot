"""CLI: minimal LeRobot dataset recorder for Onero H1."""

from __future__ import annotations

import argparse
import logging
import time

from onero_h1_lerobot import OneroH1Config, OneroH1Robot

_logger = logging.getLogger("onero_h1.record")


def _setup_logging() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
        force=True,
    )


def _import_lerobot_dataset_tools():
    try:
        from lerobot.datasets.feature_utils import build_dataset_frame, hw_to_dataset_features
    except Exception:  # pragma: no cover - version compatibility
        try:
            from lerobot.utils.feature_utils import build_dataset_frame, hw_to_dataset_features
        except Exception as exc:
            raise RuntimeError("LeRobot dataset utilities are not available. Install LeRobot first.") from exc

    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except Exception as exc:
        raise RuntimeError("LeRobotDataset is not available. Install LeRobot first.") from exc

    return LeRobotDataset, build_dataset_frame, hw_to_dataset_features


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record one Onero H1 episode into a LeRobot dataset.")
    parser.add_argument("--repo-id", required=True, help="Dataset repo id, e.g. user/onero_h1_test")
    parser.add_argument("--root", default=None, help="Optional local dataset root")
    parser.add_argument("--task", required=True, help="Task description stored in each frame")
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Episode duration in seconds. If not set, records until Ctrl+C.",
    )
    parser.add_argument("--fps", type=int, default=30, help="Recording frame rate")
    parser.add_argument("--id", default="onero_h1", help="Robot id")
    parser.add_argument("--no-cameras", action="store_true", help="Disable camera features")
    parser.add_argument("--cameras", default="head,left,right", help="Comma-separated camera names")
    parser.add_argument(
        "--send-hold-action",
        action="store_true",
        help="Publish the current state as a hold action at every frame. Off by default for safety.",
    )
    parser.add_argument("--finalize", action="store_true", help="Call dataset.finalize() after saving")
    parser.add_argument(
        "--action-left-arm-topic",
        default=None,
        help="Left arm action topic for teleoperator (default depends on teleop type)",
    )
    parser.add_argument(
        "--action-right-arm-topic",
        default=None,
        help="Right arm action topic for teleoperator (default depends on teleop type)",
    )
    parser.add_argument(
        "--action-gripper-topic",
        default=None,
        help="Gripper action topic for teleoperator (default depends on teleop type)",
    )
    parser.add_argument(
        "--action-left-gripper-topic",
        default=None,
        help="Left gripper action topic for VR teleop (Float32)",
    )
    parser.add_argument(
        "--action-right-gripper-topic",
        default=None,
        help="Right gripper action topic for VR teleop (Float32)",
    )
    parser.add_argument(
        "--gripper-type",
        default="int32",
        choices=["int32", "float32"],
        help="Gripper message type: int32 (joint teleop) or float32 (VR teleop)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    _setup_logging()
    try:
        LeRobotDataset, build_dataset_frame, hw_to_dataset_features = _import_lerobot_dataset_tools()
    except RuntimeError as exc:
        raise SystemExit(
            f"ERROR: {exc}\nInstall LeRobot support with: python3 -m pip install -e '.[lerobot]'"
        ) from None

    camera_names = tuple(name.strip() for name in args.cameras.split(",") if name.strip())
    config = OneroH1Config(id=args.id, use_cameras=not args.no_cameras, camera_names=camera_names)
    robot = OneroH1Robot(config)

    obs_features = hw_to_dataset_features(robot.observation_features, "observation")
    action_features = hw_to_dataset_features(robot.action_features, "action")
    dataset_features = {**obs_features, **action_features}

    create_kwargs = {
        "repo_id": args.repo_id,
        "fps": args.fps,
        "features": dataset_features,
        "robot_type": robot.name,
        "use_videos": not args.no_cameras,
    }
    if args.root:
        create_kwargs["root"] = args.root
    dataset = LeRobotDataset.create(**create_kwargs)

    frame_count = max(1, int(args.duration * args.fps)) if args.duration is not None else None
    period = 1.0 / max(args.fps, 1)

    _logger.info(
        "Recording started: repo=%s task=%s fps=%d duration=%s cameras=%s",
        args.repo_id, args.task, args.fps,
        f"{args.duration}s" if args.duration else "indefinite",
        args.cameras,
    )

    with robot:
        i = 0
        try:
            while True:
                if frame_count is not None and i >= frame_count:
                    break

                start = time.perf_counter()
                obs = robot.get_observation()
                action = {key: float(obs.get(key, 0.0)) for key in robot.action_feature_names}
                if args.send_hold_action:
                    action = robot.send_action(action)

                observation_frame = build_dataset_frame(dataset.features, obs, prefix="observation")
                action_frame = build_dataset_frame(dataset.features, action, prefix="action")
                dataset.add_frame({**observation_frame, **action_frame, "task": args.task})

                sleep_s = period - (time.perf_counter() - start)
                if sleep_s > 0:
                    time.sleep(sleep_s)

                i += 1
                if frame_count is not None:
                    if i % max(1, args.fps) == 0:
                        _logger.info("Recorded frame %d/%d (%.1fs)", i, frame_count, i * period)
                    print(f"Recorded frame {i}/{frame_count}", end="\r")
                else:
                    if i % (args.fps * 5) == 0:
                        _logger.info("Recorded frame %d (%.1fs)", i, i * period)
                    print(f"Recorded frame {i} ({i * period:.1f}s)  |  Ctrl+C to stop", end="\r")
        except KeyboardInterrupt:
            print(f"\n\nStopped by user after {i} frames ({i * period:.1f}s).")
            _logger.info("Recording stopped by user: %d frames (%.1fs)", i, i * period)

    print("Saving episode...")
    _logger.info("Saving episode to %s (%d frames)...", args.repo_id, i)
    dataset.save_episode()
    if args.finalize and hasattr(dataset, "finalize"):
        dataset.finalize()
    _logger.info("Episode saved successfully")
    print("Done.")


if __name__ == "__main__":
    main()
