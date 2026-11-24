"""CLI: minimal LeRobot dataset recorder for Onero H1."""

from __future__ import annotations

import argparse
import logging
import os
import signal
import threading
import time
from datetime import datetime


from onero_h1_lerobot import OneroH1Config, OneroH1Robot
from onero_h1_lerobot.teleoperator import OneroH1RosJointTeleop, OneroH1RosJointTeleopConfig


def resolve_dataset_target(
    repo_id: str,
    dataset_home: str,
    *,
    resume: bool = False,
    no_stamp: bool = False,
    now: datetime | None = None,
) -> tuple[str, str]:
    """Resolve the final repo id and root using LeRobot's startup semantics.

    New sessions get a timestamp suffix; ``--resume`` appends to an existing
    dataset and ``--no-stamp`` keeps the id but never overwrites.
    """
    root = os.path.join(dataset_home, repo_id)
    if resume:
        if not os.path.exists(root):
            raise SystemExit(f"--resume 指定的数据集不存在: {root}")
        return repo_id, root
    if no_stamp:
        if os.path.exists(root):
            raise SystemExit(
                f"数据集已存在: {root}。继续录制请加 --resume；"
                "或去掉 --no-stamp 以生成带时间戳的新数据集。"
            )
        return repo_id, root
    stamped = f"{repo_id}_{(now or datetime.now()).strftime('%Y%m%d_%H%M%S')}"
    stamped_root = os.path.join(dataset_home, stamped)
    if os.path.exists(stamped_root):
        raise SystemExit(f"时间戳数据集已存在: {stamped_root}，请稍后重试")
    return stamped, stamped_root

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


def _install_stop_on_sigint(stop_event: threading.Event) -> object:
    """Turn the first Ctrl+C into the same graceful stop as the stop topic.

    The recording loop checks ``stop_event`` between frames, so the current
    frame finishes, the episode is saved, and the teleop exit handshake (21)
    still runs. A second Ctrl+C restores the default handler so the user can
    force an immediate exit if needed.
    """
    previous = signal.getsignal(signal.SIGINT)

    def _handler(signum, frame) -> None:  # noqa: ARG001
        if stop_event.is_set():
            signal.signal(signal.SIGINT, signal.SIG_DFL)
            print("\n\n再次收到 Ctrl+C，已恢复强制退出。")
            return
        stop_event.set()
        print("\n\n收到 Ctrl+C，正在结束当前帧并保存，请稍候...")

    signal.signal(signal.SIGINT, _handler)
    return previous


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
        detail = f"{type(exc).__name__}: {exc}"
        raise RuntimeError(
            "LeRobotDataset import failed. LeRobot may be installed, but one of its "
            f"dataset dependencies could not be loaded ({detail})."
        ) from exc

    return LeRobotDataset, build_dataset_frame, hw_to_dataset_features


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record one Onero H1 episode into a LeRobot dataset.")
    parser.add_argument("--repo-id", required=True, help="Dataset repo id, e.g. user/onero_h1_test")
    parser.add_argument("--root", default=None, help="Optional local dataset root")
    parser.add_argument("--task", required=True, help="Task description stored in each frame")
    parser.add_argument(
        "--resume",
        action="store_true",
        help="Continue recording an existing dataset; --repo-id must be its full existing id.",
    )
    parser.add_argument(
        "--no-stamp",
        action="store_true",
        help="Keep --repo-id exactly; fail if the dataset directory already exists.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Episode duration in seconds. If not set, records until /stop_recording or Ctrl+C.",
    )
    parser.add_argument("--fps", type=int, default=30, help="Recording frame rate")
    parser.add_argument("--id", default="onero_h1", help="Robot id")
    parser.add_argument(
        "--teleop-type",
        default="homogeneous",
        choices=["homogeneous", "vr"],
        help="Teleop strategy: homogeneous (leader arm) | vr",
    )
    parser.add_argument("--no-cameras", action="store_true", help="Disable camera features")
    parser.add_argument("--cameras", default="head,left,right", help="Comma-separated camera names")
    parser.add_argument(
        "--passive",
        action="store_true",
        help="Passive recording: do not publish control commands to the robot.",
    )
    parser.add_argument(
        "--send-hold-action",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--finalize",
        action="store_true",
        help="Deprecated compatibility flag; LeRobot 0.6.1 datasets are always finalized after saving.",
    )
    parser.add_argument(
        "--action-left-arm-topic",
        default=None,
        help="Left arm action topic (default: /left/joint_states)",
    )
    parser.add_argument(
        "--action-right-arm-topic",
        default=None,
        help="Right arm action topic (default: /right/joint_states)",
    )
    parser.add_argument(
        "--action-gripper-topic",
        default=None,
        help="Gripper action topic (default: /joystick_info)",
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
        "--no-gripper",
        action="store_true",
        help="Disable gripper action subscription (use when gripper topic is not available)",
    )
    parser.add_argument(
        "--no-gripper-action",
        action="store_true",
        help="Do not publish gripper commands to /joystick_info.",
    )
    parser.add_argument(
        "--control-hz",
        type=float,
        default=None,
        help="Control publish frequency in Hz. Defaults to 100; only used in active mode.",
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
        send_action=not args.passive,
        # The leader arm program is only used by homogeneous teleop; VR reads
        # from the headset topics instead.
        auto_start_arm_node=args.teleop_type != "vr",
        # Active control is the default; passive recording starts the leader
        # normally without remapping its command topic.
        arm_node_mode="record" if args.passive else "teleop",
        # onero_leader gripper behavior: publish Int32 commands by default.
        send_gripper_action=not args.no_gripper_action,
    )
    robot = OneroH1Robot(config)

    # 创建 Teleoperator：支持主从同构和 VR，默认话题 onero_leader
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

    teleop = OneroH1RosJointTeleop(teleop_config)

    obs_features = hw_to_dataset_features(robot.observation_features, "observation")
    action_features = hw_to_dataset_features(teleop.action_features, "action")
    dataset_features = {**obs_features, **action_features}

    create_kwargs = {
        "fps": args.fps,
        "features": dataset_features,
        "robot_type": robot.name,
        "use_videos": not args.no_cameras,
    }
    dataset_home = args.root or os.environ.get(
        "HF_LEROBOT_HOME",
        os.path.join(os.path.expanduser("~"), "lerobot_datasets"),
    )
    repo_id, dataset_root = resolve_dataset_target(
        args.repo_id, dataset_home, resume=args.resume, no_stamp=args.no_stamp
    )
    create_kwargs["repo_id"] = repo_id

    if args.resume:
        _logger.info("继续录制已有数据集: %s，检查 feature 兼容性...", dataset_root)
        existing = LeRobotDataset.resume(repo_id, root=str(dataset_root))

        # LeRobot 0.6.1 adds bookkeeping fields such as timestamp/index to
        # ``existing.features``. Compare only non-default dataset fields, and
        # validate their semantic signature rather than keys alone.
        from lerobot.utils.constants import DEFAULT_FEATURES

        default_keys = set(DEFAULT_FEATURES)
        new_keys = set(dataset_features)
        existing_keys = set(existing.features) - default_keys
        missing = new_keys - existing_keys
        extra = existing_keys - new_keys

        def _feature_signature(feature: dict) -> tuple:
            return (
                feature.get("dtype"),
                tuple(feature.get("shape", ())),
                tuple(feature.get("names", ())),
            )

        incompatible = sorted(
            key
            for key in new_keys & existing_keys
            if _feature_signature(dataset_features[key])
            != _feature_signature(existing.features[key])
        )
        if missing or extra or incompatible:
            _logger.error(
                "Feature mismatch with existing dataset:%s%s%s."
                " Use a different --repo-id to avoid data corruption.",
                f" missing={sorted(missing)}" if missing else "",
                f" extra={sorted(extra)}" if extra else "",
                f" incompatible={incompatible}" if incompatible else "",
            )
            existing.finalize()
            raise SystemExit(1)
        dataset = existing
    else:
        _logger.info("创建新数据集: %s (%s)", repo_id, dataset_root)
        dataset = LeRobotDataset.create(**create_kwargs)

    frame_count = max(1, int(args.duration * args.fps)) if args.duration is not None else None
    period = 1.0 / max(args.fps, 1)

    # Graceful stop: publish True to /stop_recording (or --stop-topic), or press
    # Ctrl+C once (converted to the same stop event below). Both stop between
    # frames so the episode is saved cleanly.
    stop_event = threading.Event()

    _logger.info(
        "Recording started: repo=%s root=%s task=%s fps=%d duration=%s cameras=%s",
        repo_id, dataset_root, args.task, args.fps,
        f"{args.duration}s" if args.duration else "indefinite",
        args.cameras,
    )

    with robot:
        # Send the teleop handshake (20); enter_teleop_mode() waits briefly for
        # the arms to settle before the recording/control loop starts.
        robot.enter_teleop_mode()
        # Stop subscriber must be created after rclpy.init() (called by robot.connect())
        _start_stop_subscriber(args.stop_topic, stop_event, _logger)
        # Ctrl+C is handled here so the first press behaves exactly like
        # publishing True to the stop topic.
        previous_sigint = _install_stop_on_sigint(stop_event)

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
                        if not args.passive:
                            action = robot.send_action(action)
                        with sent_action_lock:
                            sent_action_ref[0] = dict(action)
                    except Exception:
                        _logger.exception("Control loop error")
                    elapsed = time.perf_counter() - loop_start
                    sleep_s = control_period - elapsed
                    if sleep_s > 0:
                        time.sleep(sleep_s)

            if not args.passive:
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
                if not args.passive:
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
            # Set the event unconditionally so the stop-topic subscriber also
            # exits cleanly in passive mode.
            stop_event.set()
            if not args.passive:
                control_thread.join(timeout=2.0)
                _logger.info("Control loop stopped")

        # The recording loop is done; restore the default Ctrl+C behavior.
        signal.signal(signal.SIGINT, previous_sigint)

    if i == 0:
        print("No frames recorded. Skipping save.")
        _logger.warning("No frames recorded; skipping save")
        return

    print("Saving episode...")
    _logger.info("Saving episode to %s (%d frames)...", repo_id, i)

    dataset.save_episode()
    # LeRobot 0.6.1 requires finalize() to flush writer/parquet metadata. A
    # finalized dataset remains appendable in a later process via resume().
    dataset.finalize()
    _logger.info("Episode saved and finalized successfully")
    print(f"Done. Dataset: {repo_id}")
    print(f"Path: {dataset_root}")


if __name__ == "__main__":
    main()
