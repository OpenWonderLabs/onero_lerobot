"""Shared runtime defaults for native LeRobot CLIs and ``onero-h1-*`` wrappers.

LeRobot imports the ``lerobot_robot_onero_h1`` plugin at CLI startup, before it
parses arguments. That lets native commands pick up exactly the same robot-side
defaults as the wrapper commands:

- teleoperate: leader arm in teleop mode (remapped), 20/21 handshake, active control
- record: active control by default, passive only with an explicit
  ``--robot.send_action=false``
- rollout: leader arm remapped, 20/21 handshake, policy actions sent to the robot
- train: no robot involved, so nothing to configure
"""

from __future__ import annotations

import os
import sys

_TELEOP_COMMANDS = frozenset(
    {"lerobot-teleoperate", "onero-h1-lerobot-teleoperate", "lerobot_teleoperate.py"}
)
_RECORD_COMMANDS = frozenset({"lerobot-record", "onero-h1-lerobot-record", "lerobot_record.py"})
_ROLLOUT_COMMANDS = frozenset({"lerobot-rollout", "onero-h1-lerobot-rollout", "lerobot_rollout.py"})


def _arg_value(argv: list[str], name: str) -> str | None:
    for index, arg in enumerate(argv):
        if arg.startswith(f"{name}="):
            return arg.split("=", 1)[1]
        if arg == name and index + 1 < len(argv):
            return argv[index + 1]
    return None


def _setdefault(key: str, value: str) -> None:
    os.environ.setdefault(key, value)


def configure_for_command(argv: list[str] | None = None) -> str:
    """Apply the wrapper's robot environment defaults to the running command.

    Idempotent. Values already set by the user or by a wrapper are preserved.
    Returns the detected command name (empty when unknown).
    """
    args = sys.argv if argv is None else argv
    command = os.path.basename(args[0]) if args else ""

    if command in _TELEOP_COMMANDS:
        _setdefault("ONERO_H1_ARM_NODE_MODE", "teleop")
        _setdefault("ONERO_H1_AUTO_TELEOP", "1")
        _setdefault("ONERO_H1_SEND_ACTION", "1")
        if not any(arg == "--fps" or arg.startswith("--fps=") for arg in args[1:]):
            args.append("--fps=100")
    elif command in _RECORD_COMMANDS:
        send_action = (_arg_value(args, "--robot.send_action") or "").lower()
        # LeRobot drives the follower by default; only an explicit
        # send_action=false is treated as passive recording.
        mode = "record" if send_action in ("0", "false", "no") else "teleop"
        _setdefault("ONERO_H1_ARM_NODE_MODE", mode)
        _setdefault("ONERO_H1_AUTO_TELEOP", "1")
        _setdefault("ONERO_H1_SEND_ACTION", "1" if mode == "teleop" else "0")
    elif command in _ROLLOUT_COMMANDS:
        _setdefault("ONERO_H1_ARM_NODE_MODE", "teleop")
        _setdefault("ONERO_H1_AUTO_TELEOP", "1")
        _setdefault("ONERO_H1_SEND_ACTION", "1")

    return command
