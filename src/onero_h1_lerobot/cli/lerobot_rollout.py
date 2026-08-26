"""Register the Onero H1 plugins, then delegate to LeRobot's rollout CLI."""

from __future__ import annotations

# Registration must happen before draccus parses --robot.type=onero_h1.
import onero_h1_lerobot  # noqa: F401


def main() -> None:
    try:
        from lerobot.scripts.lerobot_rollout import main as lerobot_rollout_main
    except Exception:
        raise SystemExit(
            "ERROR: LeRobot is not available. Install LeRobot support with: "
            "uv sync --extra lerobot"
        ) from None

    lerobot_rollout_main()


if __name__ == "__main__":
    main()
