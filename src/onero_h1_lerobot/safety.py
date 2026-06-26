"""Safety clipping for Onero H1 LeRobot actions."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .config import OneroH1Config

DEG = math.pi / 180.0


@dataclass(frozen=True)
class ScalarLimit:
    min_value: float
    max_value: float
    max_delta: float | None = None

    def clip(self, value: float, reference: float | None = None, apply_delta: bool = True) -> float:
        clipped = min(max(value, self.min_value), self.max_value)
        if apply_delta and self.max_delta is not None and reference is not None:
            clipped = min(max(clipped, reference - self.max_delta), reference + self.max_delta)
        return clipped


class ActionLimiter:
    """Clip actions to documented joint ranges and per-step deltas.

    The limiter is deliberately conservative. It does not replace collision
    checking, MoveIt planning, emergency stop handling, or human supervision.
    """

    def __init__(self, config: OneroH1Config):
        self.config = config
        self._last_action: dict[str, float] = {}
        self._limits = self._build_limits(config)

    @staticmethod
    def _arm_joint_limits(
        ranges: tuple[tuple[float, float], ...], max_delta: float | None
    ) -> list[ScalarLimit]:
        return [
            ScalarLimit(float(lo), float(hi), max_delta)
            for lo, hi in ranges
        ]

    @staticmethod
    def _legacy_doc_arm_joint_limits(config: OneroH1Config) -> list[ScalarLimit]:
        # Fallback for custom configurations without side-specific H1 limits.
        ranges_deg = [
            (-60, 180),
            (-15, 180),
            (-158, 158),
            (-110, 110),
            (-158, 158),
            (-90, 90),
            (-158, 158),
        ]
        return [
            ScalarLimit(lo * DEG, hi * DEG, config.max_arm_delta_rad)
            for lo, hi in ranges_deg
        ]

    @classmethod
    def _build_limits(cls, config: OneroH1Config) -> dict[str, ScalarLimit]:
        limits: dict[str, ScalarLimit] = {}

        for prefix, joint_names, configured_limits in (
            ("left_arm", config.left_arm_joint_names, config.left_arm_position_limits),
            ("right_arm", config.right_arm_joint_names, config.right_arm_position_limits),
        ):
            arm_limits = (
                cls._arm_joint_limits(configured_limits, config.max_arm_delta_rad)
                if configured_limits
                else cls._legacy_doc_arm_joint_limits(config)
            )
            for i, joint_name in enumerate(joint_names):
                base_limit = arm_limits[min(i, len(arm_limits) - 1)]
                limits[f"{prefix}.{joint_name}.pos"] = base_limit

        limits["lift.pos"] = ScalarLimit(0.40, 1.30, config.max_lift_delta_m)
        limits["head.pitch.pos"] = ScalarLimit(-28 * DEG, 28 * DEG, config.max_head_delta_rad)
        limits["head.yaw.pos"] = ScalarLimit(-90 * DEG, 90 * DEG, config.max_head_delta_rad)

        limits["base.vx"] = ScalarLimit(-config.max_base_vx_mps, config.max_base_vx_mps)
        limits["base.vy"] = ScalarLimit(-config.max_base_vy_mps, config.max_base_vy_mps)
        limits["base.wz"] = ScalarLimit(-config.max_base_wz_radps, config.max_base_wz_radps)
        return limits

    def clip_action(
        self,
        action: dict[str, float],
        observation: dict[str, object] | None = None,
        apply_delta: bool = True,
    ) -> dict[str, float]:
        if not self.config.enable_safety:
            self._last_action.update(action)
            return dict(action)

        clipped: dict[str, float] = {}
        observation = observation or {}
        for key, value in action.items():
            limit = self._limits.get(key)
            if limit is None:
                clipped[key] = value
                continue

            reference = self._last_action.get(key)
            if reference is None and key in observation:
                try:
                    reference = float(observation[key])
                except (TypeError, ValueError):
                    reference = None
            clipped[key] = limit.clip(value, reference, apply_delta=apply_delta)

        self._last_action.update(clipped)
        return clipped

    def reset(self) -> None:
        self._last_action.clear()
