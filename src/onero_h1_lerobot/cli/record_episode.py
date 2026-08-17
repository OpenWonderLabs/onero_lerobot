"""CLI: minimal LeRobot dataset recorder for Onero H1."""

from __future__ import annotations

import argparse
import logging
import threading
import time

from onero_h1_lerobot import OneroH1Config, OneroH1Robot
from onero_h1_lerobot.teleoperator import OneroH1RosJointTeleop, OneroH1RosJointTeleopConfig

_logger = logging.getLogger("onero_h1.record")


def _setup_logging() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
        force=True,
    )


def _start_stop_subscriber(
    topic: str,
    stop_event: threading.Event,
    logger: logging.Logger,
) -> None:
    """Create a ROS2 subscriber that sets *stop_event* when True is received.

    The subscriber runs in a background thread so the recording loop can check
    *stop_event* between frames without blocking.
    """
    try:
        import rclpy
        from std_msgs.msg import Bool
    except ImportError:
        logger.warning("rclpy not available; stop topic disabled")
        return

    if not rclpy.ok():
        logger.warning("rclpy not initialized; stop topic disabled")
        return

    try:
        node = rclpy.create_node("_record_stop_listener")
        node.create_subscription(
            Bool,
            topic,
            lambda msg: stop_event.set() if msg.data else None,
            10,
        )
    except Exception as exc:
        logger.warning("Failed to create stop subscriber on '%s': %s", topic, exc)
        return

    def _spin():
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        try:
            while not stop_event.is_set():
                executor.spin_once(timeout_sec=0.5)
        finally:
            executor.remove_node(node)
            node.destroy_node()

    thread = threading.Thread(target=_spin, name="record_stop_subscriber", daemon=True)
    thread.start()
    logger.info("Stop subscriber active on '%s' (publish True to stop)", topic)


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
    parser.add_argument(
        "--teleop-type",
        default="homogeneous",
        choices=["homogeneous", "heterogeneous", "vr"],
        help="Teleop strategy: homogeneous | heterogeneous | vr",
    )
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
        default=None,
        choices=["int32", "float32"],
        help="Gripper message type: int32 (joint teleop) or float32 (VR teleop). Default depends on --teleop-type.",
    )
    parser.add_argument(
        "--no-gripper",
        action="store_true",
        help="Disable gripper action subscription (use when gripper topic is not available)",
    )
    parser.add_argument(
        "--control-hz",
        type=float,
        default=None,
        help="Control publish frequency in Hz. For homogeneous teleop, defaults to 100; "
             "otherwise defaults to the recording fps. Only relevant with --send-hold-action.",
    )
    parser.add_argument(
        "--stop-topic",
        default="/stop_recording",
        help="ROS2 Bool topic to gracefully stop recording (publish True to stop between frames)",
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
    config = OneroH1Config(
        id=args.id,
        use_cameras=not args.no_cameras,
        camera_names=camera_names,
        send_action=args.send_hold_action,
        # Homogeneous teleoperation: the leader arm directly controls the follower
        # via the same /record_data path, so we don't need to send gripper commands
        # from the recording pipeline.
        send_gripper_action=args.teleop_type != "homogeneous",
    )
    robot = OneroH1Robot(config)

    # 创建 Teleoperator，根据遥操类型自动设置话题和夹爪处理策略
    teleop_config = OneroH1RosJointTeleopConfig(
        teleop_type=args.teleop_type,
        use_lift=False,
        use_head=False,
        use_base_velocity_action=False,
        use_gripper=not args.no_gripper,
    )
    # 手动覆盖默认话题（如果用户指定了）
    if args.action_left_arm_topic:
        teleop_config.left_arm_topic = args.action_left_arm_topic
    if args.action_right_arm_topic:
        teleop_config.right_arm_topic = args.action_right_arm_topic
    if args.action_gripper_topic:
        teleop_config.gripper_topic = args.action_gripper_topic
    if args.action_left_gripper_topic:
        teleop_config.left_gripper_topic = args.action_left_gripper_topic
    if args.action_right_gripper_topic:
        teleop_config.right_gripper_topic = args.action_right_gripper_topic
    if args.gripper_type is not None:
        teleop_config.gripper_type = args.gripper_type

    teleop = OneroH1RosJointTeleop(teleop_config)

    obs_features = hw_to_dataset_features(robot.observation_features, "observation")
    action_features = hw_to_dataset_features(teleop.action_features, "action")
    dataset_features = {**obs_features, **action_features}

    create_kwargs = {
        "repo_id": args.repo_id,
        "fps": args.fps,
        "features": dataset_features,
        "robot_type": robot.name,
        "use_videos": not args.no_cameras,
    }
    import os as _os

    if args.root:
        create_kwargs["root"] = args.root
        dataset_root = _os.path.join(args.root, args.repo_id)
    else:
        dataset_root = _os.path.join(_os.environ.get("HF_LEROBOT_HOME", _os.path.join(_os.path.expanduser("~"), "lerobot_datasets")), args.repo_id)

    if _os.path.exists(dataset_root):
        _logger.info("Dataset already exists, resuming (will append new episode)")
        dataset = LeRobotDataset.resume(args.repo_id, root=str(dataset_root))
    else:
        dataset = LeRobotDataset.create(**create_kwargs)

    frame_count = max(1, int(args.duration * args.fps)) if args.duration is not None else None
    period = 1.0 / max(args.fps, 1)

    # Set up graceful stop via ROS2 topic. Publish True to /stop_recording
    # (or --stop-topic) to stop cleanly between frames, avoiding the truncated
    # image / partial frame issues that Ctrl+C can cause.
    stop_event = threading.Event()

    _logger.info(
        "Recording started: repo=%s task=%s fps=%d duration=%s cameras=%s",
        args.repo_id, args.task, args.fps,
        f"{args.duration}s" if args.duration else "indefinite",
        args.cameras,
    )

    with robot:
        # Stop subscriber must be created after rclpy.init() (called by robot.connect())
        _start_stop_subscriber(args.stop_topic, stop_event, _logger)

        with teleop:
            # ── Control frequency ──────────────────────────────────────────
            # Homogeneous teleop needs high-frequency control (100 Hz) for
            # smooth motion, matching the original leader arm's publish rate.
            # Recording stays at the configured fps to keep dataset size
            # manageable.  The control loop runs in a background thread,
            # continuously sending the latest teleop action to the robot.
            if args.control_hz is not None:
                control_hz = args.control_hz
            elif args.teleop_type == "homogeneous":
                control_hz = 100.0
            else:
                control_hz = float(args.fps)
            control_period = 1.0 / max(control_hz, 1.0)

            sent_action_ref: list[dict | None] = [None]
            sent_action_lock = threading.Lock()

            def _control_loop() -> None:
                while not stop_event.is_set():
                    loop_start = time.perf_counter()
                    try:
                        action = teleop.get_action()
                        if args.send_hold_action:
                            action = robot.send_action(action)
                        with sent_action_lock:
                            sent_action_ref[0] = dict(action)
                    except Exception:
                        _logger.exception("Control loop error")
                    elapsed = time.perf_counter() - loop_start
                    sleep_s = control_period - elapsed
                    if sleep_s > 0:
                        time.sleep(sleep_s)

            if args.send_hold_action:
                control_thread = threading.Thread(
                    target=_control_loop, name="control_loop", daemon=True,
                )
                control_thread.start()
                _logger.info("Control loop started at %.0f Hz (recording at %d fps)", control_hz, args.fps)

            # ── Recording loop (fixed fps) ────────────────────────────────
            i = 0
            while True:
                if stop_event.is_set():
                    print(f"\n\nStop signal received after {i} frames ({i * period:.1f}s).")
                    _logger.info("Recording stopped by stop topic: %d frames (%.1fs)", i, i * period)
                    break
                if frame_count is not None and i >= frame_count:
                    break

                start = time.perf_counter()
                obs = robot.get_observation()
                if args.send_hold_action:
                    with sent_action_lock:
                        action = sent_action_ref[0] if sent_action_ref[0] is not None else teleop.get_action()
                else:
                    action = teleop.get_action()

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
                    print(f"Recorded frame {i} ({i * period:.1f}s)  |  Publish to {args.stop_topic} to stop", end="\r")

            # ── Stop control thread before disconnecting ──────────────────
            if args.send_hold_action:
                stop_event.set()
                control_thread.join(timeout=2.0)
                _logger.info("Control loop stopped")

    if i == 0:
        print("No frames recorded. Skipping save.")
        _logger.warning("No frames recorded; skipping save")
        return

    print("Saving episode...")
    _logger.info("Saving episode to %s (%d frames)...", args.repo_id, i)

    dataset.save_episode()
    if args.finalize and hasattr(dataset, "finalize"):
        dataset.finalize()
    _logger.info("Episode saved successfully")
    print("Done.")


if __name__ == "__main__":
    main()
