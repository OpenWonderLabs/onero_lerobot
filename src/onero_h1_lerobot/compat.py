"""Compatibility layer for using this package with or without LeRobot installed.

The real target runtime is LeRobot. The small fallback classes below only make the
package importable in ROS-only development environments and in unit tests.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, TypeAlias

try:  # pragma: no cover - depends on the user's LeRobot installation
    from lerobot.robots.config import RobotConfig
    from lerobot.robots.robot import Robot
    from lerobot.teleoperators.config import TeleoperatorConfig
    from lerobot.teleoperators.teleoperator import Teleoperator
    from lerobot.types import RobotAction, RobotObservation

    LEROBOT_AVAILABLE = True
except Exception:  # pragma: no cover - exercised when LeRobot is not installed
    LEROBOT_AVAILABLE = False

    RobotAction: TypeAlias = dict[str, Any]
    RobotObservation: TypeAlias = dict[str, Any]

    @dataclass(kw_only=True)
    class RobotConfig:
        """Minimal fallback matching the fields used by LeRobot's RobotConfig."""

        id: str | None = None
        calibration_dir: Path | None = None

        @property
        def type(self) -> str:
            return self.__class__.__name__

    @dataclass(kw_only=True)
    class TeleoperatorConfig:
        """Minimal fallback matching the fields used by LeRobot's TeleoperatorConfig."""

        id: str | None = None
        calibration_dir: Path | None = None

        @property
        def type(self) -> str:
            return self.__class__.__name__

    class Robot:
        """Minimal fallback base class.

        This is intentionally small. It is not a replacement for LeRobot; install
        LeRobot for recording, training, and policy evaluation.
        """

        config_class: type[RobotConfig]
        name: str

        def __init__(self, config: RobotConfig):
            self.robot_type = self.name
            self.id = config.id
            self.calibration_dir = config.calibration_dir
            self.calibration = {}

        def __enter__(self):
            self.connect()
            return self

        def __exit__(self, exc_type, exc_value, traceback) -> None:
            self.disconnect()

    class Teleoperator:
        """Minimal fallback base class for import-time tests without LeRobot."""

        config_class: type[TeleoperatorConfig]
        name: str

        def __init__(self, config: TeleoperatorConfig):
            self.id = config.id
            self.calibration_dir = config.calibration_dir
            self.calibration = {}

        def __enter__(self):
            self.connect()
            return self

        def __exit__(self, exc_type, exc_value, traceback) -> None:
            self.disconnect()
