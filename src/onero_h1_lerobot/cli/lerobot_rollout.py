"""Register the Onero H1 plugins, then delegate to LeRobot's rollout CLI."""

from __future__ import annotations

import sys

# Registration must happen before draccus parses --robot.type=onero_h1.
import onero_h1_lerobot  # noqa: F401
from onero_h1_lerobot.native_cli import configure_for_command


def main() -> None:
    try:
        from lerobot.scripts.lerobot_rollout import main as lerobot_rollout_main
    except Exception:
        raise SystemExit(
            "ERROR: LeRobot is not available. Install LeRobot support with: "
            "uv sync --extra lerobot"
        ) from None

    configure_for_command(sys.argv)
    try:
        lerobot_rollout_main()
    except KeyboardInterrupt:
        # LeRobot's rollout teardown disconnects the robot, which sends the 21
        # handshake and returns the arms home. Catching here avoids a traceback.
        pass


if __name__ == "__main__":
    main()
