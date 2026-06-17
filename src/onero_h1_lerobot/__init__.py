"""LeRobot adapter for Onero H1 / SwitchBot H1."""

from .compat import LEROBOT_AVAILABLE
from .config import OneroH1Config
from .robot import OneroH1Robot
from .teleoperator import OneroH1RosJointTeleop, OneroH1RosJointTeleopConfig

# LeRobot's dynamic third-party factory derives the device class name by
# removing "Config" from the config class. OneroH1Config therefore maps to
# a class named OneroH1.
OneroH1 = OneroH1Robot

__all__ = [
    "LEROBOT_AVAILABLE",
    "OneroH1",
    "OneroH1Config",
    "OneroH1Robot",
    "OneroH1RosJointTeleop",
    "OneroH1RosJointTeleopConfig",
]
