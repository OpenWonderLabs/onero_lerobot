"""LeRobot third-party plugin entrypoint for Onero H1.

LeRobot's plugin loader imports installed distributions whose names start with
``lerobot_robot_``. This module forwards that import to the actual package.
"""

from onero_h1_lerobot import *  # noqa: F403

from onero_h1_lerobot.native_cli import configure_for_command
from onero_h1_lerobot.record_guard import install_native_record_guard

# LeRobot imports this module at CLI startup, before it parses arguments or
# creates the dataset. This makes native commands behave like the wrappers and
# lets lerobot-record save the in-progress episode on Ctrl+C.
configure_for_command()
install_native_record_guard()
