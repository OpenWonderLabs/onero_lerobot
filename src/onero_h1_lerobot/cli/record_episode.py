"""CLI: minimal LeRobot dataset recorder for Onero H1."""

from __future__ import annotations

import argparse
import time

from onero_h1_lerobot import OneroH1Config, OneroH1Robot


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
    parser.add_argument("--duration", type=float, default=10.0, help="Episode duration in seconds")
    parser.add_argument("--fps", type=int, default=10, help="Recording frame rate")
    parser.add_argument("--id", default="onero_h1", help="Robot id")
    parser.add_argument("--no-cameras", action="store_true", help="Disable camera features")
    parser.add_argument("--cameras", default="head,left,right", help="Comma-separated camera names")
    parser.add_argument(
        "--send-hold-action",
        action="store_true",
        help="Publish the current state as a hold action at every frame. Off by default for safety.",
    )
    parser.add_argument("--finalize", action="store_true", help="Call dataset.finalize() after saving")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
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

    frame_count = max(1, int(args.duration * args.fps))
    period = 1.0 / max(args.fps, 1)

    with robot:
        for i in range(frame_count):
            start = time.perf_counter()
            obs = robot.get_observation()
            # Store a hold-position action by default. If requested, also send it
            # through the adapter so the saved action equals the clipped command.
            action = {key: float(obs.get(key, 0.0)) for key in robot.action_feature_names}
            if args.send_hold_action:
                action = robot.send_action(action)

            observation_frame = build_dataset_frame(dataset.features, obs, prefix="observation")
            action_frame = build_dataset_frame(dataset.features, action, prefix="action")
            dataset.add_frame({**observation_frame, **action_frame, "task": args.task})

            sleep_s = period - (time.perf_counter() - start)
            if sleep_s > 0:
                time.sleep(sleep_s)
            print(f"Recorded frame {i + 1}/{frame_count}", end="\r")

    print("\nSaving episode...")
    dataset.save_episode()
    if args.finalize and hasattr(dataset, "finalize"):
        dataset.finalize()
    print("Done.")


if __name__ == "__main__":
    main()
