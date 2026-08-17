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

    Teleop type determines default topics and gripper handling:
    - homogeneous: /left/joint_states, /right/joint_states, /joystick_info (Int32)
    - heterogeneous: /teleop/left/joint_states, /teleop/right/joint_states, /joystick_info (Int32)
    - vr: /left_joint_states, /right_joint_states, separate gripper topics (Float32)
    """

    # Teleop type: "homogeneous" | "heterogeneous" | "vr"
    teleop_type: str = "homogeneous"

    ros_node_name: str = "onero_h1_ros_joint_teleop"
    ros_namespace: str = ""
    auto_init_rclpy: bool = True
    shutdown_rclpy_on_disconnect: bool = False
    executor_threads: int = 2
    connect_timeout_s: float = 5.0

    use_left_arm: bool = True
    use_right_arm: bool = True
    use_arm_velocity_action: bool = True
    use_lift: bool = True
    use_head: bool = False
    use_base_velocity_action: bool = False

    left_arm_topic: str = ""
    right_arm_topic: str = ""
    lift_topic: str = "/lift/joint_states"
    head_topic: str = "/head/joint_states"
    base_velocity_topic: str = "/teleop/cmd_vel"

    # Gripper action
    use_gripper: bool = True
    gripper_topic: str = ""
    gripper_type: str = ""  # set by teleop_type
    # VR 遥操夹爪话题（gripper_type=float32 时使用）
    left_gripper_topic: str = ""
    right_gripper_topic: str = ""

    # Action diff — frame-to-frame action position delta (computed locally)
    use_action_diff: bool = True

    left_arm_joint_names: tuple[str, ...] = DEFAULT_LEFT_ARM_JOINTS
    right_arm_joint_names: tuple[str, ...] = DEFAULT_RIGHT_ARM_JOINTS
    lift_joint_name: str = "lift_joint"
    head_pitch_joint_name: str = "head_pitch_joint"
    head_yaw_joint_name: str = "head_yaw_joint"

    stale_action_s: float = 1.0
    max_estimated_arm_velocity_radps: float = 6.0
    smooth_arm_actions: bool = True
    arm_position_alpha: float = 0.35
    arm_velocity_alpha: float = 0.25
    arm_position_deadband_rad: float = 0.0
    arm_velocity_deadband_radps: float = 0.02
    arm_filter_reset_after_s: float = 0.25
    require_fresh_action: bool = True
    allow_missing_keys: bool = False

    def __post_init__(self) -> None:
        """Apply teleop-type strategy defaults — only fills empty values."""
        if self.teleop_type == "homogeneous":
            self._apply_defaults(
                left_arm="/left/joint_states",
                right_arm="/right/joint_states",
                gripper="/joystick_info",
                gripper_type="int32",
            )
        elif self.teleop_type == "heterogeneous":
            self._apply_defaults(
                left_arm="/teleop/left/joint_states",
                right_arm="/teleop/right/joint_states",
                gripper="/joystick_info",
                gripper_type="int32",
            )
        elif self.teleop_type == "vr":
            self._apply_defaults(
                left_arm="/left_joint_states",
                right_arm="/right_joint_states",
                gripper_type="float32",
                left_gripper="/vr/left_gripper/open_ratio",
                right_gripper="/vr/right_gripper/open_ratio",
            )

    def _apply_defaults(
        self,
        left_arm: str = "",
        right_arm: str = "",
        gripper: str = "",
        gripper_type: str = "",
        left_gripper: str = "",
        right_gripper: str = "",
    ) -> None:
        if not self.left_arm_topic:
            self.left_arm_topic = left_arm
        if not self.right_arm_topic:
            self.right_arm_topic = right_arm
        if not self.gripper_topic:
            self.gripper_topic = gripper
        if not self.gripper_type:
            self.gripper_type = gripper_type
        if not self.left_gripper_topic:
            self.left_gripper_topic = left_gripper
        if not self.right_gripper_topic:
            self.right_gripper_topic = right_gripper


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
        self._last_arm_positions: dict[str, dict[str, float]] = {}
        self._filtered_arm_positions: dict[str, dict[str, float]] = {}
        self._last_arm_position_time: dict[str, float] = {}
        self._last_arm_velocities: dict[str, dict[str, float]] = {}
        self._subscriptions: list[Any] = []

        # Cached action positions for diff computation
        self._cached_action_positions: dict[str, float] = {}
        self._cached_action_time: float = 0.0

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
            if self.config.use_arm_velocity_action:
                names.extend(f"left_arm.{joint}.vel" for joint in self.config.left_arm_joint_names)
        if self.config.use_right_arm:
            names.extend(f"right_arm.{joint}.pos" for joint in self.config.right_arm_joint_names)
            if self.config.use_arm_velocity_action:
                names.extend(f"right_arm.{joint}.vel" for joint in self.config.right_arm_joint_names)
        if self.config.use_gripper:
            names.extend(["left_gripper.pos", "right_gripper.pos"])
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
        # 夹爪回调只设置一侧，另一侧需预置默认值，避免 get_action() 报 missing keys
        if self.config.use_gripper and self.config.gripper_type != "float32":
            with self._lock:
                self._action.setdefault("left_gripper.pos", 0.0)
                self._action.setdefault("right_gripper.pos", 0.0)
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
        if self.config.use_gripper:
            if self.config.gripper_type == "float32":
                self._subscriptions.append(
                    self.node.create_subscription(
                        self.ros.Float32, self.config.left_gripper_topic, self._on_left_gripper_float32, 10
                    )
                )
                self._subscriptions.append(
                    self.node.create_subscription(
                        self.ros.Float32, self.config.right_gripper_topic, self._on_right_gripper_float32, 10
                    )
                )
            else:
                self._subscriptions.append(
                    self.node.create_subscription(
                        self.ros.Int32, self.config.gripper_topic, self._on_gripper, 10
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

    @staticmethod
    def _velocities_by_joint(msg: Any, joint_names: tuple[str, ...]) -> dict[str, float]:
        by_name = {
            name: float(msg.velocity[i])
            for i, name in enumerate(msg.name)
            if i < len(msg.velocity) and name
        }
        if all(name in by_name for name in joint_names):
            return {name: by_name[name] for name in joint_names}
        if len(msg.velocity) >= len(joint_names):
            return {name: float(msg.velocity[i]) for i, name in enumerate(joint_names)}
        return {}

    def _estimated_velocities(
        self,
        side: str,
        positions: dict[str, float],
        now: float,
    ) -> dict[str, float]:
        last_positions = self._last_arm_positions.get(side)
        last_time = self._last_arm_position_time.get(side)
        last_velocities = self._last_arm_velocities.get(side, {})

        self._last_arm_positions[side] = dict(positions)
        self._last_arm_position_time[side] = now

        if last_positions is None or last_time is None:
            velocities = {joint: 0.0 for joint in positions}
            self._last_arm_velocities[side] = velocities
            return velocities

        dt = max(now - last_time, 1e-3)
        max_vel = max(0.0, float(self.config.max_estimated_arm_velocity_radps))
        alpha = min(max(float(self.config.arm_velocity_alpha), 0.0), 1.0)
        velocities: dict[str, float] = {}
        for joint, position in positions.items():
            if joint not in last_positions:
                velocity = 0.0
            else:
                velocity = (float(position) - float(last_positions[joint])) / dt
            if max_vel > 0.0:
                velocity = min(max(velocity, -max_vel), max_vel)
            if joint in last_velocities:
                velocity = alpha * velocity + (1.0 - alpha) * float(last_velocities[joint])
            velocities[joint] = velocity

        self._last_arm_velocities[side] = velocities
        return velocities

    def _filter_arm_action(
        self,
        side: str,
        positions: dict[str, float],
        velocities: dict[str, float],
        now: float,
    ) -> tuple[dict[str, float], dict[str, float]]:
        if not self.config.smooth_arm_actions:
            if self.config.use_arm_velocity_action and not velocities:
                velocities = self._estimated_velocities(side, positions, now)
            else:
                self._last_arm_positions[side] = dict(positions)
                self._filtered_arm_positions[side] = dict(positions)
                self._last_arm_position_time[side] = now
                if velocities:
                    self._last_arm_velocities[side] = dict(velocities)
            return positions, velocities

        last_positions = self._last_arm_positions.get(side)
        last_filtered_positions = self._filtered_arm_positions.get(side)
        last_time = self._last_arm_position_time.get(side)
        last_velocities = self._last_arm_velocities.get(side, {})

        self._last_arm_positions[side] = dict(positions)
        self._last_arm_position_time[side] = now

        reset_after_s = max(0.0, float(self.config.arm_filter_reset_after_s))
        if (
            last_positions is None
            or last_filtered_positions is None
            or last_time is None
            or now <= last_time
            or (reset_after_s > 0.0 and now - last_time > reset_after_s)
        ):
            filtered_positions = dict(positions)
            filtered_velocities = {
                joint: float(velocities.get(joint, 0.0))
                for joint in positions
            }
            self._filtered_arm_positions[side] = filtered_positions
            self._last_arm_velocities[side] = filtered_velocities
            return filtered_positions, filtered_velocities

        dt = max(now - last_time, 1e-3)
        position_alpha = min(max(float(self.config.arm_position_alpha), 0.0), 1.0)
        velocity_alpha = min(max(float(self.config.arm_velocity_alpha), 0.0), 1.0)
        position_deadband = max(0.0, float(self.config.arm_position_deadband_rad))
        velocity_deadband = max(0.0, float(self.config.arm_velocity_deadband_radps))
        max_vel = max(0.0, float(self.config.max_estimated_arm_velocity_radps))

        filtered_positions: dict[str, float] = {}
        filtered_velocities: dict[str, float] = {}
        for joint, raw_position in positions.items():
            previous_filtered_position = float(last_filtered_positions.get(joint, raw_position))
            position_error = float(raw_position) - previous_filtered_position
            if abs(position_error) <= position_deadband:
                filtered_position = previous_filtered_position
            else:
                filtered_position = previous_filtered_position + position_alpha * position_error
            filtered_positions[joint] = filtered_position

            if joint in velocities:
                raw_velocity = float(velocities[joint])
            elif joint in last_positions:
                raw_velocity = (float(raw_position) - float(last_positions[joint])) / dt
            else:
                raw_velocity = 0.0

            if max_vel > 0.0:
                raw_velocity = min(max(raw_velocity, -max_vel), max_vel)
            if abs(raw_velocity) <= velocity_deadband:
                raw_velocity = 0.0

            previous_velocity = float(last_velocities.get(joint, raw_velocity))
            filtered_velocities[joint] = (
                velocity_alpha * raw_velocity + (1.0 - velocity_alpha) * previous_velocity
            )

        self._filtered_arm_positions[side] = filtered_positions
        self._last_arm_velocities[side] = filtered_velocities
        return filtered_positions, filtered_velocities

    def _on_arm(self, side: str, joint_names: tuple[str, ...], msg: Any) -> None:
        positions = self._positions_by_joint(msg, joint_names)
        if not positions:
            return
        now = now_monotonic()
        velocities = self._velocities_by_joint(msg, joint_names)
        positions, velocities = self._filter_arm_action(side, positions, velocities, now)
        with self._lock:
            for joint, value in positions.items():
                self._action[f"{side}_arm.{joint}.pos"] = value
            if self.config.use_arm_velocity_action:
                for joint, value in velocities.items():
                    self._action[f"{side}_arm.{joint}.vel"] = value
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

    def _on_gripper(self, msg: Any) -> None:
        """Handle gripper action from joystick_info (Int32)."""
        command = msg.data
        with self._lock:
            if 100 <= command < 200:
                self._action["left_gripper.pos"] = (command - 100) / 100.0
            elif 200 <= command < 300:
                self._action["right_gripper.pos"] = (command - 200) / 100.0
            self._stamp("gripper")

    def _on_left_gripper_float32(self, msg: Any) -> None:
        """Handle left gripper action from VR topic (Float32, 0.0-1.0)."""
        value = float(msg.data)
        with self._lock:
            self._action["left_gripper.pos"] = max(0.0, min(1.0, value))
            self._stamp("gripper")

    def _on_right_gripper_float32(self, msg: Any) -> None:
        """Handle right gripper action from VR topic (Float32, 0.0-1.0)."""
        value = float(msg.data)
        with self._lock:
            self._action["right_gripper.pos"] = max(0.0, min(1.0, value))
            self._stamp("gripper")

    def _required_stamp_keys(self) -> tuple[str, ...]:
        keys: list[str] = []
        if self.config.use_left_arm:
            keys.append("left_arm")
        if self.config.use_right_arm:
            keys.append("right_arm")
        if self.config.use_gripper and self.config.gripper_type == "float32":
            keys.append("gripper")
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

    def _action_snapshot(self) -> tuple[dict[str, float], dict[str, float]]:
        with self._lock:
            return dict(self._action), dict(self._stamps)

    def get_action(self) -> RobotAction:
        if not self.is_connected:
            raise ConnectionError(f"{self} is not connected")

        now = now_monotonic()
        action, stamps = self._action_snapshot()

        missing = [key for key in self.action_feature_names if key not in action]
        stale_sources: list[str] = []
        if self.config.require_fresh_action:
            stale_sources = [
                key
                for key in self._required_stamp_keys()
                if key not in stamps or now - stamps[key] > self.config.stale_action_s
            ]

        if missing or stale_sources:
            self._wait_for_first_action(self.config.connect_timeout_s)
            now = now_monotonic()
            action, stamps = self._action_snapshot()
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

        result = {key: float(action[key]) for key in self.action_feature_names}

        # Compute action diff (frame-to-frame delta)
        if self.config.use_action_diff:
            now_diff = now_monotonic()
            dt = max(now_diff - self._cached_action_time, 1e-6)
            for key in list(result.keys()):
                if key.endswith(".pos"):
                    motor_name = key.removesuffix(".pos")
                    diff_key = f"{motor_name}.diff"
                    cur = result[key]
                    prev = self._cached_action_positions.get(key, cur)
                    result[diff_key] = (cur - prev) / dt
            # Update cache
            for key, value in result.items():
                if key.endswith(".pos"):
                    self._cached_action_positions[key] = value
            self._cached_action_time = now_diff

        return result

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
