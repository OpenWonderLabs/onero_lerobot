"""Wrapper that registers Onero H1 before delegating to LeRobot's recorder."""

from __future__ import annotations

import sys

# Importing the package registers OneroH1Config and OneroH1RosJointTeleopConfig
# with LeRobot's draccus registries when LeRobot is installed.
import onero_h1_lerobot  # noqa: F401
from onero_h1_lerobot.native_cli import configure_for_command


def main() -> None:
    try:
        from lerobot.scripts.lerobot_record import main as lerobot_record_main
    except Exception:
        raise SystemExit(
            "ERROR: LeRobot is not available. Install LeRobot support with: "
            "python3 -m pip install -e '.[lerobot]'"
        ) from None

    configure_for_command(sys.argv)
    try:
        lerobot_record_main()
    except KeyboardInterrupt:
        # LeRobot's recorder saves the in-progress episode (via record_guard),
        # finalizes the dataset, and calls robot.disconnect() in its own finally
        # block, which sends the 21 handshake. Catching here only avoids a
        # traceback.
        pass


if __name__ == "__main__":
    main()
