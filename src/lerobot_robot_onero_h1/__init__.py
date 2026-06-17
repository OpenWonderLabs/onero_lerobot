"""LeRobot third-party plugin entrypoint for Onero H1.

LeRobot's plugin loader imports installed distributions whose names start with
``lerobot_robot_``. This module forwards that import to the actual package.
"""

from onero_h1_lerobot import *  # noqa: F403
