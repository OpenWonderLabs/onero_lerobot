"""ROS2 client for the Onero H1 SDK topics documented in this repository."""

from __future__ import annotations

import json
import threading
import time
from types import SimpleNamespace
from typing import Any

import numpy as np

from .config import OneroH1Config
from .utils import now_monotonic, quaternion_to_yaw


class RosImportError(RuntimeError):
    pass


def _import_ros() -> SimpleNamespace:
    """Import ROS2 modules lazily so the package remains importable off-robot."""

    try:
        import rclpy
        from diagnostic_msgs.msg import DiagnosticStatus
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.executors import MultiThreadedExecutor
        from sensor_msgs.msg import BatteryState, CompressedImage, JointState
        from std_msgs.msg import Bool, String
    except Exception as exc:  # pragma: no cover - depends on ROS2 installation
        raise RosImportError(
            "ROS2 Python packages are not available. Source your ROS2 Jazzy environment "
            "before using OneroH1Robot, for example: `source /opt/ros/jazzy/setup.bash`."
        ) from exc

    return SimpleNamespace(
        rclpy=rclpy,
        MultiThreadedExecutor=MultiThreadedExecutor,
        JointState=JointState,
        CompressedImage=CompressedImage,
        Odometry=Odometry,
        BatteryState=BatteryState,
        DiagnosticStatus=DiagnosticStatus,
        Twist=Twist,
        Bool=Bool,
        String=String,
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
        self._images: dict[str, np.ndarray] = {}
        self._stamps: dict[str, float] = {}

        self._pub_left_arm = None
        self._pub_right_arm = None
        self._pub_lift = None
        self._pub_head = None
        self._pub_base = None
        self._subscriptions: list[Any] = []

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

        self._spin_thread = threading.Thread(target=self._spin, name="onero_h1_ros_spin", daemon=True)
        self._spin_thread.start()
        self._connected = True

    def _spin(self) -> None:
        assert self.executor is not None
        try:
            self.executor.spin()
        except Exception:
            if self._connected:
                raise

    def _create_publishers(self) -> None:
        assert self.ros is not None and self.node is not None
        if self.config.use_left_arm:
            self._pub_left_arm = self.node.create_publisher(
                self.ros.String, self.config.left_arm_movej_topic, 10
            )
        if self.config.use_right_arm:
            self._pub_right_arm = self.node.create_publisher(
                self.ros.String, self.config.right_arm_movej_topic, 10
            )
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

        if self.config.use_cameras:
            for camera_name in self.config.camera_names:
                topic = self.config.camera_topics.get(camera_name)
                if not topic:
                    continue
                self._subscriptions.append(
                    self.node.create_subscription(
                        self.ros.CompressedImage,
                        topic,
                        lambda msg, name=camera_name: self._on_compressed_image(name, msg),
                        2,
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

    def _on_compressed_image(self, camera_name: str, msg: Any) -> None:
        try:
            import cv2
        except Exception as exc:  # pragma: no cover - depends on optional OpenCV runtime
            raise RuntimeError("OpenCV is required for decoding ROS CompressedImage camera topics") from exc

        data = np.frombuffer(msg.data, dtype=np.uint8)
        image_bgr = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if image_bgr is None:
            return
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

        if self.config.resize_camera_images:
            shape = self.config.camera_shapes.get(camera_name)
            if shape is not None:
                height, width, _channels = shape
                if image_rgb.shape[:2] != (height, width):
                    image_rgb = cv2.resize(image_rgb, (width, height), interpolation=cv2.INTER_AREA)

        with self._lock:
            self._images[camera_name] = image_rgb
            self._stamp(f"camera.{camera_name}")

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "joint_pos": dict(self._joint_pos),
                "joint_vel": dict(self._joint_vel),
                "joint_effort": dict(self._joint_effort),
                "base": dict(self._base),
                "battery": dict(self._battery),
                "front_bumper": self._bumper_pressed,
                "arm_diagnostics": dict(self._arm_diagnostics),
                "images": {key: value.copy() for key, value in self._images.items()},
                "stamps": dict(self._stamps),
                "time": now_monotonic(),
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
            if required_keys and all(key in snap["stamps"] for key in required_keys):
                return True
            if not required_keys and (snap["joint_pos"] or snap["base"] or snap["images"]):
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

    def disconnect(self) -> None:
        if not self._connected:
            return

        if self.config.stop_base_on_disconnect:
            self.publish_base_velocity(0.0, 0.0, 0.0)

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
