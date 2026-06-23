"""ROS2 JointState teleoperator for Onero H1 LeRobot workflows."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from types import SimpleNamespace
from typing import Any

from .compat import RobotAction, Teleoperator, TeleoperatorConfig
from .config import DEFAULT_LEFT_ARM_JOINTS, DEFAULT_RIGHT_ARM_JOINTS
from .ros_client import _import_ros
from .utils import now_monotonic


@dataclass(kw_only=True)
class OneroH1RosJointTeleopConfig(TeleoperatorConfig):
    """Read LeRobot actions from ROS2 teleoperation topics.

    The expected teleop inputs are ``sensor_msgs/msg/JointState`` for each arm,
    optional ``JointState`` for lift/head, and optional ``geometry_msgs/msg/Twist``
    for base velocity. If a JointState message names the configured joints, values
    are matched by name; otherwise positions are matched by order.
    """

    ros_node_name: str = "onero_h1_ros_joint_teleop"
    ros_namespace: str = ""
    auto_init_rclpy: bool = True
    shutdown_rclpy_on_disconnect: bool = False
    executor_threads: int = 2
    connect_timeout_s: float = 5.0

    use_left_arm: bool = True
    use_right_arm: bool = True
    use_lift: bool = True
    use_head: bool = True
    use_base_velocity_action: bool = False

    left_arm_topic: str = "/teleop/left_arm/joint_states"
    right_arm_topic: str = "/teleop/right_arm/joint_states"
    lift_topic: str = "/teleop/lift/joint_states"
    head_topic: str = "/teleop/head/joint_states"
    base_velocity_topic: str = "/teleop/cmd_vel"

    left_arm_joint_names: tuple[str, ...] = DEFAULT_LEFT_ARM_JOINTS
    right_arm_joint_names: tuple[str, ...] = DEFAULT_RIGHT_ARM_JOINTS
    lift_joint_name: str = "lift_joint"
    head_pitch_joint_name: str = "head_pitch_joint"
    head_yaw_joint_name: str = "head_yaw_joint"

    stale_action_s: float = 1.0
    require_fresh_action: bool = True
    allow_missing_keys: bool = False


if hasattr(TeleoperatorConfig, "register_subclass"):
    OneroH1RosJointTeleopConfig = TeleoperatorConfig.register_subclass("onero_h1_ros_joint")(
        OneroH1RosJointTeleopConfig
    )


class OneroH1RosJointTeleop(Teleoperator):
    """Teleoperator that converts ROS2 teleop topics to LeRobot action dicts."""

    config_class = OneroH1RosJointTeleopConfig
    name = "onero_h1_ros_joint"

    def __init__(self, config: OneroH1RosJointTeleopConfig):
        super().__init__(config)
        self.config = config
        self.ros: SimpleNamespace | None = None
        self.node = None
        self.executor = None
        self._spin_thread: threading.Thread | None = None
        self._owns_rclpy = False
        self._connected = False
        self._lock = threading.RLock()
        self._action: dict[str, float] = {}
        self._stamps: dict[str, float] = {}
        self._subscriptions: list[Any] = []

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return True

    @property
    def action_feature_names(self) -> tuple[str, ...]:
        names: list[str] = []
        if self.config.use_left_arm:
            names.extend(f"left_arm.{joint}.pos" for joint in self.config.left_arm_joint_names)
        if self.config.use_right_arm:
            names.extend(f"right_arm.{joint}.pos" for joint in self.config.right_arm_joint_names)
        if self.config.use_lift:
            names.append("lift.pos")
        if self.config.use_head:
            names.extend(["head.pitch.pos", "head.yaw.pos"])
        if self.config.use_base_velocity_action:
            names.extend(["base.vx", "base.vy", "base.wz"])
        return tuple(names)

    @property
    def action_features(self) -> dict:
        return {name: float for name in self.action_feature_names}

    @property
    def feedback_features(self) -> dict:
        return {}

    def calibrate(self) -> None:
        return None

    def configure(self) -> None:
        return None

    def connect(self, calibrate: bool = True) -> None:
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
        self._create_subscribers()
        self._spin_thread = threading.Thread(target=self._spin, name="onero_h1_teleop_ros_spin", daemon=True)
        self._spin_thread.start()
        self._connected = True
        self._wait_for_first_action(self.config.connect_timeout_s)

    def _spin(self) -> None:
        assert self.executor is not None
        try:
            self.executor.spin()
        except Exception:
            if self._connected:
                raise

    def _create_subscribers(self) -> None:
        assert self.ros is not None and self.node is not None
        if self.config.use_left_arm:
            self._subscriptions.append(
                self.node.create_subscription(
                    self.ros.JointState,
                    self.config.left_arm_topic,
                    lambda msg: self._on_arm("left", self.config.left_arm_joint_names, msg),
                    10,
                )
            )
        if self.config.use_right_arm:
            self._subscriptions.append(
                self.node.create_subscription(
                    self.ros.JointState,
                    self.config.right_arm_topic,
                    lambda msg: self._on_arm("right", self.config.right_arm_joint_names, msg),
                    10,
                )
            )
        if self.config.use_lift:
            self._subscriptions.append(
                self.node.create_subscription(self.ros.JointState, self.config.lift_topic, self._on_lift, 10)
            )
        if self.config.use_head:
            self._subscriptions.append(
                self.node.create_subscription(self.ros.JointState, self.config.head_topic, self._on_head, 10)
            )
        if self.config.use_base_velocity_action:
            self._subscriptions.append(
                self.node.create_subscription(
                    self.ros.Twist, self.config.base_velocity_topic, self._on_base_velocity, 10
                )
            )

    def _stamp(self, key: str) -> None:
        self._stamps[key] = now_monotonic()

    @staticmethod
    def _positions_by_joint(msg: Any, joint_names: tuple[str, ...]) -> dict[str, float]:
        by_name = {
            name: float(msg.position[i])
            for i, name in enumerate(msg.name)
            if i < len(msg.position) and name
        }
        if all(name in by_name for name in joint_names):
            return {name: by_name[name] for name in joint_names}
        if len(msg.position) >= len(joint_names):
            return {name: float(msg.position[i]) for i, name in enumerate(joint_names)}
        return {}

    def _on_arm(self, side: str, joint_names: tuple[str, ...], msg: Any) -> None:
        positions = self._positions_by_joint(msg, joint_names)
        if not positions:
            return
        with self._lock:
            for joint, value in positions.items():
                self._action[f"{side}_arm.{joint}.pos"] = value
            self._stamp(f"{side}_arm")

    def _on_lift(self, msg: Any) -> None:
        positions = self._positions_by_joint(msg, (self.config.lift_joint_name,))
        if not positions:
            return
        with self._lock:
            self._action["lift.pos"] = positions[self.config.lift_joint_name]
            self._stamp("lift")

    def _on_head(self, msg: Any) -> None:
        positions = self._positions_by_joint(
            msg, (self.config.head_pitch_joint_name, self.config.head_yaw_joint_name)
        )
        if not positions:
            return
        with self._lock:
            self._action["head.pitch.pos"] = positions[self.config.head_pitch_joint_name]
            self._action["head.yaw.pos"] = positions[self.config.head_yaw_joint_name]
            self._stamp("head")

    def _on_base_velocity(self, msg: Any) -> None:
        with self._lock:
            self._action["base.vx"] = float(msg.linear.x)
            self._action["base.vy"] = float(msg.linear.y)
            self._action["base.wz"] = float(msg.angular.z)
            self._stamp("base")

    def _required_stamp_keys(self) -> tuple[str, ...]:
        keys: list[str] = []
        if self.config.use_left_arm:
            keys.append("left_arm")
        if self.config.use_right_arm:
            keys.append("right_arm")
        if self.config.use_lift:
            keys.append("lift")
        if self.config.use_head:
            keys.append("head")
        if self.config.use_base_velocity_action:
            keys.append("base")
        return tuple(keys)

    def _wait_for_first_action(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self._lock:
                if all(key in self._stamps for key in self._required_stamp_keys()):
                    return True
            time.sleep(0.05)
        return False

    def get_action(self) -> RobotAction:
        if not self.is_connected:
            raise ConnectionError(f"{self} is not connected")

        now = now_monotonic()
        with self._lock:
            action = dict(self._action)
            stamps = dict(self._stamps)

        missing = [key for key in self.action_feature_names if key not in action]
        if missing and not self.config.allow_missing_keys:
            raise RuntimeError(
                "Teleop action is missing keys. Check that all configured ROS teleop topics are publishing: "
                + ", ".join(missing)
            )
        for key in missing:
            action[key] = 0.0

        if self.config.require_fresh_action:
            stale_sources = [
                key
                for key in self._required_stamp_keys()
                if key not in stamps or now - stamps[key] > self.config.stale_action_s
            ]
            if stale_sources:
                raise RuntimeError("Teleop action is stale or unavailable: " + ", ".join(stale_sources))

        return {key: float(action[key]) for key in self.action_feature_names}

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        return None

    def disconnect(self) -> None:
        if not self._connected:
            return
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
