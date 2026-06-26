"""LeRobot Robot implementation for Onero H1 / SwitchBot H1."""

from __future__ import annotations

from typing import Any

import numpy as np

from .compat import Robot, RobotAction, RobotObservation
from .config import OneroH1Config
from .ros_client import H1RosClient
from .safety import ActionLimiter
from .utils import normalize_action_dict, now_monotonic


class OneroH1Robot(Robot):
    """LeRobot-compatible wrapper around the Onero H1 ROS2 SDK.

    This class exposes the documented H1 ROS2 topics as LeRobot's standard
    ``connect/get_observation/send_action/disconnect`` interface.
    """

    config_class = OneroH1Config
    name = "onero_h1"

    def __init__(self, config: OneroH1Config):
        super().__init__(config)
        self.config = config
        self.client: H1RosClient | None = None
        self._limiter = ActionLimiter(config)
        self.cameras = {name: None for name in config.camera_names} if config.use_cameras else {}
        self._last_arm_movej_positions: dict[str, list[float]] = {}
        self._last_arm_movej_time: dict[str, float] = {}
        self._last_record_data_positions: dict[str, list[float]] = {}
        self._last_record_data_time: dict[str, float] = {}
        self._last_record_data_velocities: dict[str, list[float]] = {}

    @property
    def is_connected(self) -> bool:
        return self.client is not None and self.client.is_connected

    @property
    def is_calibrated(self) -> bool:
        # H1 calibration is handled by ROS-side drivers/controllers. This adapter
        # does not manage motor zeroing or intrinsic calibration.
        return True

    def calibrate(self) -> None:
        return None

    def configure(self) -> None:
        return None

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
        if self.config.use_lift:
            names.append("lift.pos")
        if self.config.use_head:
            names.extend(["head.pitch.pos", "head.yaw.pos"])
        if self.config.use_base_velocity_action:
            names.extend(["base.vx", "base.vy", "base.wz"])
        return tuple(names)

    @property
    def observation_features(self) -> dict:
        features: dict[str, type | tuple[int, int, int]] = {}

        if self.config.use_left_arm:
            for joint in self.config.left_arm_joint_names:
                features[f"left_arm.{joint}.pos"] = float
        if self.config.use_right_arm:
            for joint in self.config.right_arm_joint_names:
                features[f"right_arm.{joint}.pos"] = float
        if self.config.use_lift:
            features["lift.pos"] = float
        if self.config.use_head:
            features["head.pitch.pos"] = float
            features["head.yaw.pos"] = float
        if self.config.use_base_observation:
            for key in ["base.x", "base.y", "base.yaw", "base.vx", "base.vy", "base.wz"]:
                features[key] = float
        if self.config.use_battery_observation:
            for key in ["battery.voltage", "battery.current", "battery.percentage"]:
                features[key] = float
        if self.config.use_bumper_observation:
            features["front_bumper.pressed"] = float

        if self.config.include_staleness_flags:
            for key in ["joint_states", "lift", "odom", "battery", "front_bumper"]:
                features[f"status.{key}.stale"] = float
            if self.config.use_cameras:
                for camera_name in self.config.camera_names:
                    features[f"status.camera.{camera_name}.stale"] = float

        if self.config.use_cameras:
            for camera_name in self.config.camera_names:
                shape = self.config.camera_shapes.get(camera_name)
                if shape is not None:
                    features[camera_name] = shape

        return features

    @property
    def action_features(self) -> dict:
        return {name: float for name in self.action_feature_names}

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            return
        self.client = H1RosClient(self.config)
        self.client.connect()
        # Scalar-only teleop can connect best-effort, but camera recording must
        # not start until each camera has produced fresh decoded frames.
        ready = self.client.wait_for_first_observation(self.config.connect_timeout_s)
        if self.config.use_cameras and not ready:
            self.client.disconnect()
            self.client = None
            raise TimeoutError(
                "Timed out waiting for Onero H1 camera warm-up. "
                "Each configured camera must decode multiple fresh frames before recording starts."
            )
        if calibrate and not self.is_calibrated:
            self.calibrate()

    def _require_connected(self) -> H1RosClient:
        if not self.is_connected or self.client is None:
            raise ConnectionError(f"{self} is not connected")
        return self.client

    def _is_stale(self, stamps: dict[str, float], key: str, now: float) -> float:
        stamp = stamps.get(key)
        if stamp is None:
            return 1.0
        return 1.0 if now - stamp > self.config.stale_observation_s else 0.0

    def _scalar_observation_from_snapshot(self, snap: dict[str, Any]) -> dict[str, float]:
        joint_pos: dict[str, float] = snap["joint_pos"]
        base: dict[str, float] = snap["base"]
        battery: dict[str, float] = snap["battery"]
        now = float(snap.get("time", now_monotonic()))
        stamps: dict[str, float] = snap["stamps"]

        obs: dict[str, float] = {}
        if self.config.use_left_arm:
            for joint in self.config.left_arm_joint_names:
                obs[f"left_arm.{joint}.pos"] = float(joint_pos.get(joint, 0.0))
        if self.config.use_right_arm:
            for joint in self.config.right_arm_joint_names:
                obs[f"right_arm.{joint}.pos"] = float(joint_pos.get(joint, 0.0))
        if self.config.use_lift:
            obs["lift.pos"] = float(joint_pos.get(self.config.lift_joint_name, 0.0))
        if self.config.use_head:
            obs["head.pitch.pos"] = float(joint_pos.get(self.config.head_pitch_joint_name, 0.0))
            obs["head.yaw.pos"] = float(joint_pos.get(self.config.head_yaw_joint_name, 0.0))
        if self.config.use_base_observation:
            for key in ["base.x", "base.y", "base.yaw", "base.vx", "base.vy", "base.wz"]:
                obs[key] = float(base.get(key, 0.0))
        if self.config.use_battery_observation:
            for key in ["battery.voltage", "battery.current", "battery.percentage"]:
                obs[key] = float(battery.get(key, 0.0))
        if self.config.use_bumper_observation:
            obs["front_bumper.pressed"] = 1.0 if snap.get("front_bumper") else 0.0

        if self.config.include_staleness_flags:
            for key in ["joint_states", "lift", "odom", "battery", "front_bumper"]:
                obs[f"status.{key}.stale"] = self._is_stale(stamps, key, now)
            if self.config.use_cameras:
                for camera_name in self.config.camera_names:
                    obs[f"status.camera.{camera_name}.stale"] = self._is_stale(
                        stamps, f"camera.{camera_name}", now
                    )

        return obs

    def get_observation(self) -> RobotObservation:
        client = self._require_connected()
        snap = client.snapshot()
        obs: dict[str, Any] = self._scalar_observation_from_snapshot(snap)

        if self.config.use_cameras:
            images: dict[str, np.ndarray] = snap["images"]
            for camera_name in self.config.camera_names:
                if camera_name in images:
                    obs[camera_name] = images[camera_name]
                    continue
                if self.config.fill_missing_images:
                    shape = self.config.camera_shapes[camera_name]
                    obs[camera_name] = np.zeros(shape, dtype=np.uint8)

        return obs

    def _arm_positions_from_action(
        self,
        action: dict[str, float],
        observation: dict[str, float],
        side: str,
        joint_names: tuple[str, ...],
    ) -> list[float] | None:
        prefix = f"{side}_arm"
        values: list[float] = []
        missing: list[str] = []
        for joint in joint_names:
            key = f"{prefix}.{joint}.pos"
            if key in action:
                values.append(float(action[key]))
            elif not self.config.require_complete_arm_action and key in observation:
                values.append(float(observation[key]))
            else:
                missing.append(key)

        if missing:
            return None
        return values

    def _arm_velocities_from_action(
        self,
        action: dict[str, float],
        side: str,
        joint_names: tuple[str, ...],
    ) -> list[float] | None:
        if not self.config.use_arm_velocity_action:
            return None

        prefix = f"{side}_arm"
        values: list[float] = []
        for joint in joint_names:
            key = f"{prefix}.{joint}.vel"
            if key not in action:
                return None
            values.append(float(action[key]))
        return values

    def _should_publish_arm_movej(self, side: str, positions: list[float]) -> bool:
        now = now_monotonic()
        last_positions = self._last_arm_movej_positions.get(side)
        last_time = self._last_arm_movej_time.get(side)

        if last_positions is None or last_time is None:
            self._last_arm_movej_positions[side] = list(positions)
            self._last_arm_movej_time[side] = now
            return True

        min_delta = max(0.0, float(self.config.arm_movej_min_delta_rad))
        max_delta = max(abs(float(new) - float(old)) for new, old in zip(positions, last_positions))
        if max_delta < min_delta:
            return False

        publish_hz = float(self.config.arm_movej_publish_hz)
        if publish_hz > 0.0 and now - last_time < 1.0 / publish_hz:
            return False

        self._last_arm_movej_positions[side] = list(positions)
        self._last_arm_movej_time[side] = now
        return True

    def _estimated_arm_velocities(self, side: str, positions: list[float]) -> list[float]:
        now = now_monotonic()
        last_positions = self._last_record_data_positions.get(side)
        last_time = self._last_record_data_time.get(side)
        last_velocities = self._last_record_data_velocities.get(side)

        self._last_record_data_positions[side] = list(positions)
        self._last_record_data_time[side] = now

        if last_positions is None or last_time is None or now - last_time > self.config.stale_observation_s:
            velocities = [0.0] * len(positions)
            self._last_record_data_velocities[side] = velocities
            return velocities

        dt = max(now - last_time, 1e-3)
        max_vel = max(0.0, float(self.config.record_data_max_velocity_radps))
        velocities = [(float(new) - float(old)) / dt for new, old in zip(positions, last_positions)]
        if max_vel > 0.0:
            velocities = [min(max(v, -max_vel), max_vel) for v in velocities]

        alpha = min(max(float(self.config.record_data_velocity_alpha), 0.0), 1.0)
        if last_velocities is not None and len(last_velocities) == len(velocities):
            velocities = [
                alpha * float(new) + (1.0 - alpha) * float(old)
                for new, old in zip(velocities, last_velocities)
            ]

        self._last_record_data_velocities[side] = velocities
        return velocities

    def _send_arm_action(
        self,
        client: H1RosClient,
        left_positions: list[float] | None,
        right_positions: list[float] | None,
        left_velocities: list[float] | None = None,
        right_velocities: list[float] | None = None,
    ) -> None:
        arm_command_mode = self.config.arm_command_mode.lower()
        if arm_command_mode == "record_data":
            if left_positions is None or right_positions is None:
                return
            left_velocities = left_velocities or self._estimated_arm_velocities("left", left_positions)
            right_velocities = right_velocities or self._estimated_arm_velocities("right", right_positions)
            max_vel = max(0.0, float(self.config.record_data_max_velocity_radps))
            if max_vel > 0.0:
                left_velocities = [min(max(float(v), -max_vel), max_vel) for v in left_velocities]
                right_velocities = [min(max(float(v), -max_vel), max_vel) for v in right_velocities]
            if self.config.record_data_use_fixed_joint7_velocity:
                if len(left_velocities) >= 7:
                    left_velocities[6] = float(self.config.record_data_left_joint7_velocity)
                if len(right_velocities) >= 7:
                    right_velocities[6] = float(self.config.record_data_right_joint7_velocity)
            client.publish_record_data(
                left_positions,
                right_positions,
                left_velocities,
                right_velocities,
            )
            return

        if arm_command_mode != "movej":
            raise ValueError(f"Unsupported arm_command_mode: {self.config.arm_command_mode}")

        if left_positions is not None and self._should_publish_arm_movej("left", left_positions):
            client.publish_arm_movej("left", left_positions)
        if right_positions is not None and self._should_publish_arm_movej("right", right_positions):
            client.publish_arm_movej("right", right_positions)

    def send_action(self, action: RobotAction) -> RobotAction:
        client = self._require_connected()
        normalized = normalize_action_dict(dict(action), self.action_feature_names)
        snap = client.snapshot(include_images=False)
        observation = self._scalar_observation_from_snapshot(snap)

        arm_command_mode = self.config.arm_command_mode.lower()
        if arm_command_mode == "record_data" and not self.config.record_data_apply_delta_limit:
            arm_action = {
                key: value
                for key, value in normalized.items()
                if key.startswith("left_arm.") or key.startswith("right_arm.")
            }
            other_action = {key: value for key, value in normalized.items() if key not in arm_action}
            clipped = {}
            if arm_action:
                clipped.update(self._limiter.clip_action(arm_action, observation, apply_delta=False))
            if other_action:
                clipped.update(self._limiter.clip_action(other_action, observation, apply_delta=True))
        else:
            clipped = self._limiter.clip_action(normalized, observation)

        left_positions: list[float] | None = None
        left_velocities: list[float] | None = None
        if self.config.use_left_arm:
            left_positions = self._arm_positions_from_action(
                clipped, observation, "left", self.config.left_arm_joint_names
            )
            left_velocities = self._arm_velocities_from_action(
                clipped, "left", self.config.left_arm_joint_names
            )

        right_positions: list[float] | None = None
        right_velocities: list[float] | None = None
        if self.config.use_right_arm:
            right_positions = self._arm_positions_from_action(
                clipped, observation, "right", self.config.right_arm_joint_names
            )
            right_velocities = self._arm_velocities_from_action(
                clipped, "right", self.config.right_arm_joint_names
            )

        if self.config.use_left_arm or self.config.use_right_arm:
            self._send_arm_action(client, left_positions, right_positions, left_velocities, right_velocities)

        if self.config.use_lift and "lift.pos" in clipped:
            client.publish_lift(clipped["lift.pos"])

        if self.config.use_head:
            pitch = clipped.get("head.pitch.pos")
            yaw = clipped.get("head.yaw.pos")
            if pitch is not None or yaw is not None:
                client.publish_head(pitch=pitch, yaw=yaw)

        if self.config.use_base_velocity_action:
            client.publish_base_velocity(
                clipped.get("base.vx", 0.0),
                clipped.get("base.vy", 0.0),
                clipped.get("base.wz", 0.0),
            )

        return clipped

    def disconnect(self) -> None:
        if self.client is not None:
            self.client.disconnect()
        self.client = None
        self._limiter.reset()
