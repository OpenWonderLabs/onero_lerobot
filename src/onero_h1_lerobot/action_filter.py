"""Optional position/velocity Kalman filtering at the robot output boundary."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from .config import OneroH1Config


class ActionFilter:
    """Filter only commanded arm/gripper positions; never aggregate action chunks.

    Joint 1 on each arm uses shortest angular differences. Internal angles stay
    continuous; the existing action limiter remains responsible for joint limits.
    """

    def __init__(self, config: OneroH1Config):
        if config.action_filter not in {"none", "kalman"}:
            raise ValueError("action_filter must be 'none' or 'kalman'")
        self.enabled = config.action_filter == "kalman"
        for name in (
            "action_filter_fps",
            "kalman_process_variance",
            "kalman_threshold",
            "kalman_jump_variance",
        ):
            value = getattr(config, name)
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        self._transition = np.array([[1.0, 1.0 / config.action_filter_fps], [0.0, 1.0]])
        self._process = config.kalman_process_variance * np.eye(2)
        self._threshold = config.kalman_threshold
        self._jump_variance = config.kalman_jump_variance
        self._keys = {
            *(f"left_arm.{joint}.pos" for joint in config.left_arm_joint_names),
            *(f"right_arm.{joint}.pos" for joint in config.right_arm_joint_names),
            "left_gripper.pos",
            "right_gripper.pos",
        }
        self._circular = {
            f"{side}_arm.{joints[0]}.pos"
            for side, joints in (
                ("left", config.left_arm_joint_names),
                ("right", config.right_arm_joint_names),
            )
            if joints
        }
        self._states: dict[str, tuple[np.ndarray, np.ndarray, float]] = {}

    def reset(self) -> None:
        self._states.clear()

    @staticmethod
    def validate(action: dict[str, float]) -> None:
        if not all(math.isfinite(value) for value in action.values()):
            raise ValueError("action contains non-finite values")

    def apply(self, action: dict[str, float], observation: dict[str, object]) -> dict[str, float]:
        # Validate the complete command before changing any filter state.
        self.validate(action)
        if not self.enabled:
            return dict(action)
        for key in action.keys() & self._keys:
            if key not in observation or not math.isfinite(float(observation[key])):
                raise ValueError(f"Kalman initialization requires a finite observation for {key}")
        result = dict(action)
        for key in action.keys() & self._keys:
            measurement = action[key]
            if key not in self._states:
                self._states[key] = (
                    np.array([float(observation[key]), 0.0]),
                    0.1 * np.eye(2),
                    float(observation[key]),
                )
            state, covariance, previous = self._states[key]
            if key in self._circular:
                delta = (measurement - previous + math.pi) % (2 * math.pi) - math.pi
                measurement = previous + delta
            noise = self._jump_variance if abs(measurement - previous) > self._threshold else 7.5
            predicted = self._transition @ state
            covariance = self._transition @ covariance @ self._transition.T + self._process
            gain = covariance[:, 0] / (covariance[0, 0] + noise)
            state = predicted + gain * (measurement - predicted[0])
            covariance = covariance - np.outer(gain, covariance[0])
            self._states[key] = state, covariance, measurement
            result[key] = float(state[0])
        return result

    def accept_limited(self, action: dict[str, float]) -> None:
        """Avoid accumulating hidden velocity against a clipped joint limit."""
        for key, value in action.items():
            if key not in self._states:
                continue
            state, covariance, previous = self._states[key]
            if float(state[0]) != value:
                state = np.array([value, 0.0])
                self._states[key] = state, covariance, previous
