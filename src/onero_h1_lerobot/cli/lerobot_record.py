"""Wrapper that registers Onero H1 before delegating to LeRobot's recorder."""

from __future__ import annotations

# Importing the package registers OneroH1Config and OneroH1RosJointTeleopConfig
# with LeRobot's draccus registries when LeRobot is installed.
import onero_h1_lerobot  # noqa: F401


def main() -> None:
    from lerobot.scripts.lerobot_record import main as lerobot_record_main

    lerobot_record_main()


if __name__ == "__main__":
    main()
