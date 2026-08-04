"""ROS2 client for the Onero H1 SDK topics documented in this repository."""

from __future__ import annotations

import json
import logging
import math
import threading
import time
from datetime import datetime
from types import SimpleNamespace
from typing import Any

import numpy as np

from .config import OneroH1Config
from .utils import now_monotonic, quaternion_to_yaw

_logger = logging.getLogger("onero_h1.ros_client")


class RosImportError(RuntimeError):
    pass


def _quaternion_to_euler(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    """Convert quaternion to roll, pitch, yaw."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def _import_ros() -> SimpleNamespace:
    """Import ROS2 modules lazily so the package remains importable off-robot."""

    try:
        import rclpy
        from diagnostic_msgs.msg import DiagnosticStatus
        from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
        from nav_msgs.msg import Odometry
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import BatteryState, CompressedImage, JointState
        from std_msgs.msg import Bool, Float64MultiArray, Int32, String, UInt8
    except Exception as exc:  # pragma: no cover - depends on ROS2 installation
        raise RosImportError(
            "ROS2 Python packages are not available. Source your ROS2 Jazzy environment "
            "before using OneroH1Robot, for example: `source /opt/ros/jazzy/setup.bash`."
        ) from exc

    return SimpleNamespace(
        rclpy=rclpy,
        ReentrantCallbackGroup=ReentrantCallbackGroup,
        MultiThreadedExecutor=MultiThreadedExecutor,
        DurabilityPolicy=DurabilityPolicy,
        HistoryPolicy=HistoryPolicy,
        QoSProfile=QoSProfile,
        ReliabilityPolicy=ReliabilityPolicy,
        JointState=JointState,
        CompressedImage=CompressedImage,
        Odometry=Odometry,
        BatteryState=BatteryState,
        DiagnosticStatus=DiagnosticStatus,
        PoseWithCovarianceStamped=PoseWithCovarianceStamped,
        Twist=Twist,
        Bool=Bool,
        Float64MultiArray=Float64MultiArray,
        Int32=Int32,
        String=String,
        UInt8=UInt8,
    )


class H1RosClient:
    """Small ROS2 abstraction around the H1 documented topics."""

    def __init__(self, config: OneroH1Config):
        self.config = config
        self.ros: SimpleNamespace | None = None
        self.node = None
        self.executor = None
        self._spin_thread: threading.Thread | None = None
        self._owns_rclpy = False
        self._connected = False
        self._lock = threading.RLock()

        self._joint_pos: dict[str, float] = {}
        self._joint_vel: dict[str, float] = {}
        self._joint_effort: dict[str, float] = {}
        self._base: dict[str, float] = {}
        self._battery: dict[str, float] = {}
        self._bumper_pressed: bool | None = None
        self._arm_diagnostics: dict[str, Any] = {}

        # Gripper state (observation) — UInt8 0-255 mapped to 0.0-1.0
        self._left_gripper_state: float = 0.0
        self._right_gripper_state: float = 0.0

        # Gripper pose (observation) — PoseWithCovarianceStamped
        self._left_gripper_pose: list[float] | None = None  # [x, y, z, roll, pitch, yaw]
        self._right_gripper_pose: list[float] | None = None

        # Effort from arm joint state subscriptions (observation)
        self._arm_effort: dict[str, float] = {}

        # Cached positions for diff computation (observation)
        self._cached_obs_positions: dict[str, float] = {}
        self._cached_obs_time: float = 0.0
        # Cached gripper poses for diff_pos computation
        self._cached_gripper_poses: dict[str, Any] | None = None

        self._compressed_images: dict[str, bytes] = {}
        self._compressed_image_stamps: dict[str, float] = {}
        self._images: dict[str, np.ndarray] = {}
        self._image_decode_stamps: dict[str, float] = {}
        self._image_decode_counts: dict[str, int] = {}
        self._image_condition = threading.Condition(self._lock)
        self._image_decoder_threads: list[threading.Thread] = []
        self._stop_image_decoders = False
        self._stamps: dict[str, float] = {}

        self._pub_left_arm = None
        self._pub_right_arm = None
        self._pub_record_data = None
        self._pub_lift = None
        self._pub_head = None
        self._pub_base = None
        self._subscriptions: list[Any] = []
        self._callback_groups: list[Any] = []

        # Camera FPS monitoring
        self._camera_frame_counts: dict[str, int] = {}
        self._camera_fps_cache: dict[str, float] = {}
        self._fps_running = False
        self._fps_thread: threading.Thread | None = None

        # Data freshness warnings (debounce: only warn once per interval)
        self._last_warn_time: dict[str, float] = {}
        self._warn_interval_s: float = 10.0

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        if self._connected:
            return

        self.ros = _import_ros()
        if self.config.auto_init_rclpy and not self.ros.rclpy.ok():
            self.ros.rclpy.init(args=None)
            self._owns_rclpy = True

        namespace = self.config.ros_namespace or None
        self.node = self.ros.rclpy.create_node(self.config.ros_node_name, namespace=namespace)
        self.executor = self.ros.MultiThreadedExecutor(num_threads=self.config.executor_threads)
        self.executor.add_node(self.node)

        self._create_publishers()
        self._create_subscribers()
        self._start_image_decoders()
        self._start_fps_monitor()

        self._spin_thread = threading.Thread(target=self._spin, name="onero_h1_ros_spin", daemon=True)
        self._spin_thread.start()
        self._connected = True
        _logger.info("H1RosClient connected")

    def _spin(self) -> None:
        assert self.executor is not None
        try:
            self.executor.spin()
        except Exception:
            if self._connected:
                raise

    def _create_publishers(self) -> None:
        assert self.ros is not None and self.node is not None
        if not self.config.send_action:
            return
        arm_command_mode = self.config.arm_command_mode.lower()
        if arm_command_mode == "record_data":
            if self.config.use_left_arm and self.config.use_right_arm:
                self._pub_record_data = self.node.create_publisher(
                    self.ros.Float64MultiArray, self.config.record_data_topic, 1
                )
            elif self.config.use_left_arm or self.config.use_right_arm:
                raise ValueError("record_data arm command mode requires both left and right arms")
        elif arm_command_mode == "movej":
            if self.config.use_left_arm:
                self._pub_left_arm = self.node.create_publisher(
                    self.ros.String, self.config.left_arm_movej_topic, 10
                )
            if self.config.use_right_arm:
                self._pub_right_arm = self.node.create_publisher(
                    self.ros.String, self.config.right_arm_movej_topic, 10
                )
        else:
            raise ValueError(f"Unsupported arm_command_mode: {self.config.arm_command_mode}")

        if self.config.use_lift:
            self._pub_lift = self.node.create_publisher(
                self.ros.JointState, self.config.lift_command_topic, 10
            )
        if self.config.use_head:
            self._pub_head = self.node.create_publisher(
                self.ros.JointState, self.config.head_command_topic, 10
            )
        if self.config.use_base_velocity_action:
            self._pub_base = self.node.create_publisher(
                self.ros.Twist, self.config.base_velocity_topic, 10
            )

    def _create_subscribers(self) -> None:
        assert self.ros is not None and self.node is not None
        joint_state_topics: dict[str, list[str]] = {}
        if self.config.joint_states_topic:
            joint_state_topics.setdefault(self.config.joint_states_topic, []).append("joint_states")
        if self.config.use_left_arm and self.config.left_arm_state_topic:
            joint_state_topics.setdefault(self.config.left_arm_state_topic, []).append("left_arm")
        if self.config.use_right_arm and self.config.right_arm_state_topic:
            joint_state_topics.setdefault(self.config.right_arm_state_topic, []).append("right_arm")
        if self.config.use_head and self.config.head_state_topic:
            joint_state_topics.setdefault(self.config.head_state_topic, []).append("head")
        for topic, source_keys in joint_state_topics.items():
            self._subscriptions.append(
                self.node.create_subscription(
                    self.ros.JointState,
                    topic,
                    lambda msg, keys=tuple(source_keys): self._on_joint_state(msg, keys),
                    20,
                )
            )

        if self.config.use_lift:
            self._subscriptions.append(
                self.node.create_subscription(
                    self.ros.JointState, self.config.lift_state_topic, self._on_lift_state, 10
                )
            )
        if self.config.use_base_observation:
            self._subscriptions.append(
                self.node.create_subscription(self.ros.Odometry, self.config.odom_topic, self._on_odom, 20)
            )
        if self.config.use_battery_observation:
            self._subscriptions.append(
                self.node.create_subscription(
                    self.ros.BatteryState, self.config.battery_topic, self._on_battery, 5
                )
            )
        if self.config.use_bumper_observation:
            self._subscriptions.append(
                self.node.create_subscription(self.ros.Bool, self.config.front_bumper_topic, self._on_bumper, 5)
            )

        if self.config.use_gripper:
            self._subscriptions.append(
                self.node.create_subscription(
                    self.ros.UInt8, self.config.left_gripper_state_topic, self._on_left_gripper_state, 10
                )
            )
            self._subscriptions.append(
                self.node.create_subscription(
                    self.ros.UInt8, self.config.right_gripper_state_topic, self._on_right_gripper_state, 10
                )
            )

        if self.config.use_gripper_pose:
            self._subscriptions.append(
                self.node.create_subscription(
                    self.ros.PoseWithCovarianceStamped,
                    self.config.left_gripper_pose_topic,
                    self._on_left_gripper_pose,
                    10,
                )
            )
            self._subscriptions.append(
                self.node.create_subscription(
                    self.ros.PoseWithCovarianceStamped,
                    self.config.right_gripper_pose_topic,
                    self._on_right_gripper_pose,
                    10,
                )
            )

        if self.config.use_cameras:
            camera_callback_group = self.ros.ReentrantCallbackGroup()
            self._callback_groups.append(camera_callback_group)
            # 与图像发布端的 QoS 保持一致：X1/H1 相机发布端用 SensorDataQoS(BEST_EFFORT)，
            # RELIABLE 订阅端 QoS 不兼容，会静默收不到任何帧。
            camera_qos = self.ros.QoSProfile(
                history=self.ros.HistoryPolicy.KEEP_LAST,
                depth=max(1, int(self.config.camera_subscription_depth)),
                reliability=self.ros.ReliabilityPolicy.BEST_EFFORT,
                durability=self.ros.DurabilityPolicy.VOLATILE,
            )
            for camera_name in self.config.camera_names:
                topic = self.config.camera_topics.get(camera_name)
                if not topic:
                    continue
                self._subscriptions.append(
                    self.node.create_subscription(
                        self.ros.CompressedImage,
                        topic,
                        lambda msg, name=camera_name: self._on_compressed_image(name, msg),
                        camera_qos,
                        callback_group=camera_callback_group,
                    )
                )

    def _stamp(self, key: str) -> None:
        self._stamps[key] = now_monotonic()

    def _on_joint_state(self, msg: Any, source_keys: tuple[str, ...] = ("joint_states",)) -> None:
        with self._lock:
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    self._joint_pos[name] = float(msg.position[i])
                if i < len(msg.velocity):
                    self._joint_vel[name] = float(msg.velocity[i])
                if i < len(msg.effort):
                    self._joint_effort[name] = float(msg.effort[i])
                    if self.config.use_arm_effort:
                        self._arm_effort[name] = float(msg.effort[i])
            for key in source_keys:
                self._stamp(key)
            self._stamp("joint_states")

    def _on_lift_state(self, msg: Any) -> None:
        with self._lock:
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    self._joint_pos[name] = float(msg.position[i])
                if i < len(msg.velocity):
                    self._joint_vel[name] = float(msg.velocity[i])
            self._stamp("lift")

    def _on_odom(self, msg: Any) -> None:
        pose = msg.pose.pose
        twist = msg.twist.twist
        yaw = quaternion_to_yaw(
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )
        with self._lock:
            self._base.update(
                {
                    "base.x": float(pose.position.x),
                    "base.y": float(pose.position.y),
                    "base.yaw": yaw,
                    "base.vx": float(twist.linear.x),
                    "base.vy": float(twist.linear.y),
                    "base.wz": float(twist.angular.z),
                }
            )
            self._stamp("odom")

    def _on_battery(self, msg: Any) -> None:
        with self._lock:
            self._battery = {
                "battery.voltage": float(msg.voltage),
                "battery.current": float(msg.current),
                "battery.percentage": float(msg.percentage),
            }
            self._stamp("battery")

    def _on_bumper(self, msg: Any) -> None:
        with self._lock:
            self._bumper_pressed = bool(msg.data)
            self._stamp("front_bumper")

    def _on_left_gripper_state(self, msg: Any) -> None:
        with self._lock:
            self._left_gripper_state = float(msg.data) / 255.0
            self._stamp("gripper")

    def _on_right_gripper_state(self, msg: Any) -> None:
        with self._lock:
            self._right_gripper_state = float(msg.data) / 255.0

    def _on_left_gripper_pose(self, msg: Any) -> None:
        with self._lock:
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            roll, pitch, yaw = _quaternion_to_euler(float(o.x), float(o.y), float(o.z), float(o.w))
            self._left_gripper_pose = [float(p.x), float(p.y), float(p.z), roll, pitch, yaw]
            self._stamp("gripper_pose")

    def _on_right_gripper_pose(self, msg: Any) -> None:
        with self._lock:
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            roll, pitch, yaw = _quaternion_to_euler(float(o.x), float(o.y), float(o.z), float(o.w))
            self._right_gripper_pose = [float(p.x), float(p.y), float(p.z), roll, pitch, yaw]

    def _on_compressed_image(self, camera_name: str, msg: Any) -> None:
        stamp = now_monotonic()
        with self._image_condition:
            self._compressed_images[camera_name] = bytes(msg.data)
            self._compressed_image_stamps[camera_name] = stamp
            self._stamps[f"camera.{camera_name}"] = stamp
            self._image_condition.notify_all()
        self._camera_frame_counts[camera_name] = self._camera_frame_counts.get(camera_name, 0) + 1

    def _decode_compressed_image(self, camera_name: str, data: bytes) -> np.ndarray | None:
        try:
            import cv2
        except Exception as exc:  # pragma: no cover - depends on optional OpenCV runtime
            raise RuntimeError("OpenCV is required for decoding ROS CompressedImage camera topics") from exc

        # Multi-camera recording is more stable when OpenCV does not spawn a
        # large thread pool per decode path and starve DDS receive callbacks.
        if cv2.getNumThreads() != 1:
            cv2.setNumThreads(1)

        image_data = np.frombuffer(data, dtype=np.uint8)
        image_bgr = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
        if image_bgr is None:
            _logger.warning("Failed to decode image for camera '%s'", camera_name)
            return None
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

        if self.config.resize_camera_images:
            shape = self.config.camera_shapes.get(camera_name)
            if shape is not None:
                height, width, _channels = shape
                if image_rgb.shape[:2] != (height, width):
                    image_rgb = cv2.resize(image_rgb, (width, height), interpolation=cv2.INTER_AREA)

        return image_rgb

    def _start_image_decoders(self) -> None:
        if not self.config.use_cameras:
            return

        self._stop_image_decoders = False
        self._image_decoder_threads = []
        for camera_name in self.config.camera_names:
            thread = threading.Thread(
                target=self._image_decoder_loop,
                args=(camera_name,),
                name=f"onero_h1_image_decode_{camera_name}",
                daemon=True,
            )
            thread.start()
            self._image_decoder_threads.append(thread)

    def _stop_image_decoder_threads(self) -> None:
        with self._image_condition:
            self._stop_image_decoders = True
            self._image_condition.notify_all()
        for thread in self._image_decoder_threads:
            thread.join(timeout=2.0)
        self._image_decoder_threads.clear()

    def _image_decoder_loop(self, camera_name: str) -> None:
        last_decoded_stamp: float | None = None
        decode_hz = float(self.config.camera_decode_hz)
        period_s = 1.0 / decode_hz if decode_hz > 0.0 else 0.0
        next_decode_at = 0.0
        while True:
            with self._image_condition:
                while not self._stop_image_decoders:
                    stamp = self._compressed_image_stamps.get(camera_name)
                    data = self._compressed_images.get(camera_name)
                    now = time.monotonic()
                    if stamp is not None and data is not None and stamp != last_decoded_stamp:
                        if period_s <= 0.0 or now >= next_decode_at:
                            break
                        timeout_s = max(0.0, next_decode_at - now)
                    else:
                        timeout_s = 0.5

                    self._image_condition.wait(timeout=timeout_s)

                if self._stop_image_decoders:
                    return

                stamp = self._compressed_image_stamps.get(camera_name)
                data = self._compressed_images.get(camera_name)
                if stamp is None or data is None or stamp == last_decoded_stamp:
                    continue

            image = self._decode_compressed_image(camera_name, data)

            with self._image_condition:
                last_decoded_stamp = stamp
                if period_s > 0.0:
                    next_decode_at = time.monotonic() + period_s
                if image is None:
                    continue
                self._images[camera_name] = image
                self._image_decode_stamps[camera_name] = stamp
                self._image_decode_counts[camera_name] = self._image_decode_counts.get(camera_name, 0) + 1
                self._stamps[f"camera.{camera_name}.decoded"] = stamp
                self._image_condition.notify_all()

    def _snapshot_images(self) -> dict[str, np.ndarray]:
        if self._image_decoder_threads:
            with self._lock:
                return dict(self._images)

        pending: list[tuple[str, float, bytes]] = []
        with self._lock:
            for camera_name in self.config.camera_names:
                stamp = self._compressed_image_stamps.get(camera_name)
                data = self._compressed_images.get(camera_name)
                decoded_stamp = self._image_decode_stamps.get(camera_name)
                if stamp is not None and data is not None and stamp != decoded_stamp:
                    pending.append((camera_name, stamp, data))

        for camera_name, stamp, data in pending:
            image = self._decode_compressed_image(camera_name, data)
            if image is None:
                continue
            with self._lock:
                self._images[camera_name] = image
                self._image_decode_stamps[camera_name] = stamp

        with self._lock:
            return dict(self._images)

    def snapshot(self, include_images: bool = True) -> dict[str, Any]:
        images = self._snapshot_images() if include_images else {}
        with self._lock:
            now = now_monotonic()
            return {
                "joint_pos": dict(self._joint_pos),
                "joint_vel": dict(self._joint_vel),
                "joint_effort": dict(self._joint_effort),
                "base": dict(self._base),
                "battery": dict(self._battery),
                "front_bumper": self._bumper_pressed,
                "arm_diagnostics": dict(self._arm_diagnostics),
                "left_gripper_state": self._left_gripper_state,
                "right_gripper_state": self._right_gripper_state,
                "left_gripper_pose": list(self._left_gripper_pose) if self._left_gripper_pose else None,
                "right_gripper_pose": list(self._right_gripper_pose) if self._right_gripper_pose else None,
                "arm_effort": dict(self._arm_effort),
                "images": images,
                "image_decode_counts": dict(self._image_decode_counts),
                "stamps": dict(self._stamps),
                "time": now,
            }

    def _required_warmup_keys(self) -> tuple[str, ...]:
        keys: list[str] = []
        if self.config.use_left_arm:
            keys.append("left_arm" if self.config.left_arm_state_topic else "joint_states")
        if self.config.use_right_arm:
            keys.append("right_arm" if self.config.right_arm_state_topic else "joint_states")
        if self.config.use_head:
            keys.append("head" if self.config.head_state_topic else "joint_states")
        if self.config.use_lift:
            keys.append("lift")
        if self.config.use_base_observation:
            keys.append("odom")
        if self.config.use_bumper_observation:
            keys.append("front_bumper")
        return tuple(dict.fromkeys(keys))

    def wait_for_first_observation(self, timeout_s: float | None = None) -> bool:
        timeout_s = self.config.connect_timeout_s if timeout_s is None else timeout_s
        deadline = time.monotonic() + timeout_s
        required_keys = self._required_warmup_keys()
        while time.monotonic() < deadline:
            snap = self.snapshot()
            warmup_frames = max(1, int(self.config.camera_warmup_frames))
            cameras_ready = (
                not self.config.use_cameras
                or all(
                    camera_name in snap["images"]
                    and snap["image_decode_counts"].get(camera_name, 0) >= warmup_frames
                    for camera_name in self.config.camera_names
                )
            )
            if required_keys and cameras_ready and all(key in snap["stamps"] for key in required_keys):
                return True
            if not required_keys and cameras_ready and (snap["joint_pos"] or snap["base"] or snap["images"]):
                return True
            time.sleep(0.05)
        return False

    def _new_joint_state_msg(self, names: list[str], positions: list[float]) -> Any:
        assert self.ros is not None and self.node is not None
        msg = self.ros.JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = names
        msg.position = [float(v) for v in positions]
        return msg

    def publish_arm_movej(self, side: str, positions: list[float]) -> None:
        publisher = self._pub_left_arm if side == "left" else self._pub_right_arm
        if publisher is None or self.ros is None:
            return
        payload: dict[str, Any] = {"joints": [float(v) for v in positions]}
        if self.config.arm_movej_speed_scale is not None:
            payload["speed_scale"] = float(self.config.arm_movej_speed_scale)
        msg = self.ros.String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        publisher.publish(msg)

    def publish_record_data(
        self,
        left_positions: list[float],
        right_positions: list[float],
        left_velocities: list[float] | None = None,
        right_velocities: list[float] | None = None,
    ) -> None:
        if self._pub_record_data is None or self.ros is None:
            return

        left_velocities = [0.0] * len(left_positions) if left_velocities is None else left_velocities
        right_velocities = [0.0] * len(right_positions) if right_velocities is None else right_velocities
        if len(left_velocities) != len(left_positions):
            raise ValueError("left_velocities length must match left_positions length")
        if len(right_velocities) != len(right_positions):
            raise ValueError("right_velocities length must match right_positions length")

        msg = self.ros.Float64MultiArray()
        msg.data = (
            [float(v) for v in left_positions]
            + [float(v) for v in left_velocities]
            + [float(v) for v in right_positions]
            + [float(v) for v in right_velocities]
        )
        self._pub_record_data.publish(msg)

    def publish_lift(self, height_m: float) -> None:
        if self._pub_lift is None:
            return
        msg = self._new_joint_state_msg([self.config.lift_joint_name], [height_m])
        if self.config.lift_command_velocity_rpm is not None:
            msg.velocity = [float(self.config.lift_command_velocity_rpm)]
        self._pub_lift.publish(msg)

    def publish_head(self, pitch: float | None = None, yaw: float | None = None) -> None:
        if self._pub_head is None:
            return
        names: list[str] = []
        positions: list[float] = []
        if pitch is not None:
            names.append(self.config.head_pitch_joint_name)
            positions.append(float(pitch))
        if yaw is not None:
            names.append(self.config.head_yaw_joint_name)
            positions.append(float(yaw))
        if names:
            self._pub_head.publish(self._new_joint_state_msg(names, positions))

    def publish_base_velocity(self, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0) -> None:
        if self._pub_base is None or self.ros is None:
            return
        msg = self.ros.Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self._pub_base.publish(msg)

    def _warn_stale(self, key: str) -> None:
        """Warn if data for a key hasn't been received recently (debounced)."""
        now = now_monotonic()
        last = self._stamps.get(key)
        if last is not None and now - last < self.config.stale_observation_s:
            return
        last_warn = self._last_warn_time.get(key, 0.0)
        if now - last_warn < self._warn_interval_s:
            return
        self._last_warn_time[key] = now
        if last is None:
            _logger.warning("No data received for '%s'", key)
        else:
            _logger.warning("Stale data for '%s' (%.1fs since last update)", key, now - last)

    def _start_fps_monitor(self) -> None:
        if not self.config.use_cameras:
            return
        self._fps_running = True
        self._fps_thread = threading.Thread(
            target=self._fps_monitor_thread, name="onero_h1_camera_fps", daemon=True
        )
        self._fps_thread.start()
        _logger.info("Camera FPS monitor started (cameras: %s)", ", ".join(self.config.camera_names))

    def _fps_monitor_thread(self) -> None:
        """Periodically log camera FPS (every 5 seconds)."""
        last_time = time.perf_counter()
        while self._fps_running:
            time.sleep(5.0)
            current_time = time.perf_counter()
            elapsed = current_time - last_time
            fps_info: list[str] = []
            for cam_name in sorted(self.config.camera_names):
                count = self._camera_frame_counts.get(cam_name, 0)
                self._camera_frame_counts[cam_name] = 0
                real_fps = count / elapsed if elapsed > 0 else 0.0
                self._camera_fps_cache[cam_name] = round(real_fps, 1)
                fps_info.append(f"{cam_name}: {real_fps:.1f} fps")
            if fps_info:
                ts = datetime.now().strftime("%H:%M:%S")
                _logger.info("[%s] Camera FPS: %s", ts, " | ".join(fps_info))
            last_time = current_time

    def _stop_fps_monitor(self) -> None:
        self._fps_running = False
        if self._fps_thread is not None:
            self._fps_thread.join(timeout=2.0)

    def get_camera_fps(self) -> dict[str, float]:
        """Get latest camera FPS values."""
        return dict(self._camera_fps_cache)

    def disconnect(self) -> None:
        if not self._connected:
            return

        _logger.info("H1RosClient disconnecting...")

        if self.config.stop_base_on_disconnect:
            self.publish_base_velocity(0.0, 0.0, 0.0)

        self._stop_image_decoder_threads()
        self._stop_fps_monitor()

        if self.executor is not None and self.node is not None:
            self.executor.remove_node(self.node)

        self._connected = False
        if self.executor is not None:
            self.executor.shutdown(timeout_sec=2.0)
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)
        self._drain_executor_futures()

        if self.node is not None:
            self.node.destroy_node()

        if (
            self.config.shutdown_rclpy_on_disconnect
            and self._owns_rclpy
            and self.ros is not None
            and self.ros.rclpy.ok()
        ):
            self.ros.rclpy.shutdown()

        self.node = None
        self.executor = None
        self._spin_thread = None
        self._subscriptions.clear()

    def _drain_executor_futures(self) -> None:
        if self.executor is None:
            return
        worker = getattr(self.executor, "_executor", None)
        if worker is not None:
            worker.shutdown(wait=True)
        futures = getattr(self.executor, "_futures", None)
        if futures is None:
            return
        for future in list(futures):
            if not future.done():
                continue
            try:
                future.result()
            except Exception:
                pass
            finally:
                if future in futures:
                    futures.remove(future)
