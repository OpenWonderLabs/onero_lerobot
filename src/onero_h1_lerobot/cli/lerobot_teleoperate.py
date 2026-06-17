"""Wrapper that registers Onero H1 before delegating to LeRobot teleoperate."""

from __future__ import annotations

# Importing the package registers OneroH1Config and OneroH1RosJointTeleopConfig
# with LeRobot's draccus registries when LeRobot is installed.
import onero_h1_lerobot  # noqa: F401


def main() -> None:
    from lerobot.scripts.lerobot_teleoperate import main as lerobot_teleoperate_main

    lerobot_teleoperate_main()


if __name__ == "__main__":
    main()
