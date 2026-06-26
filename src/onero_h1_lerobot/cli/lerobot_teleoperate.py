"""Wrapper that registers Onero H1 before delegating to LeRobot teleoperate."""

from __future__ import annotations

import sys

# Importing the package registers OneroH1Config and OneroH1RosJointTeleopConfig
# with LeRobot's draccus registries when LeRobot is installed.
import onero_h1_lerobot  # noqa: F401


def _ensure_default_fps(argv: list[str]) -> None:
    if any(arg == "--fps" or arg.startswith("--fps=") for arg in argv[1:]):
        return
    argv.append("--fps=100")


def main() -> None:
    try:
        from lerobot.scripts.lerobot_teleoperate import main as lerobot_teleoperate_main
    except Exception:
        raise SystemExit(
            "ERROR: LeRobot is not available. Install LeRobot support with: "
            "python3 -m pip install -e '.[lerobot]'"
        ) from None

    _ensure_default_fps(sys.argv)
    lerobot_teleoperate_main()


if __name__ == "__main__":
    main()
