"""Configuration for the Onero H1 LeRobot adapter."""

from __future__ import annotations

from dataclasses import dataclass, field

from .compat import RobotConfig

DEFAULT_LEFT_ARM_JOINTS = tuple(f"left_joint{i}" for i in range(1, 8))
DEFAULT_RIGHT_ARM_JOINTS = tuple(f"right_joint{i}" for i in range(1, 8))
DEFAULT_CAMERA_TOPICS = {
    "head": "/head/camera/rgb",
    "head_depth_rgb": "/head2/camera/rgb",
    "left": "/left/camera/rgb",
    "right": "/right/camera/rgb",
}
DEFAULT_CAMERA_SHAPES = {
    "head": (480, 640, 3),
    "head_depth_rgb": (480, 640, 3),
    "left": (480, 640, 3),
    "right": (480, 640, 3),
}


@dataclass(kw_only=True)
class OneroH1Config(RobotConfig):
    """Runtime configuration for :class:`OneroH1Robot`.

    The defaults follow the API topics documented in this repository. All joint
    names and topic names are configurable because production robots often ship
    with URDF or launch-file naming differences.
    """

    # ROS node/runtime
    ros_node_name: str = "onero_h1_lerobot"
    ros_namespace: str = ""
    auto_init_rclpy: bool = True
    shutdown_rclpy_on_disconnect: bool = False
    executor_threads: int = 2
    connect_timeout_s: float = 5.0

    # Robot modules included in observations/actions
    use_left_arm: bool = True
    use_right_arm: bool = True
    use_lift: bool = True
    use_head: bool = True
    use_base_observation: bool = True
    use_base_velocity_action: bool = False
    use_battery_observation: bool = True
    use_bumper_observation: bool = True

    # Camera configuration. Keys become LeRobot image feature names.
    use_cameras: bool = True
    camera_names: tuple[str, ...] = ("head", "left", "right")
    camera_topics: dict[str, str] = field(default_factory=lambda: dict(DEFAULT_CAMERA_TOPICS))
    camera_shapes: dict[str, tuple[int, int, int]] = field(
        default_factory=lambda: dict(DEFAULT_CAMERA_SHAPES)
    )
    resize_camera_images: bool = True
    fill_missing_images: bool = True

    # ROS topics from the current SDK documentation
    joint_states_topic: str = "/joint_states"
    odom_topic: str = "/odom"
    front_bumper_topic: str = "/front_bumper"
    battery_topic: str = "/battery/state"

    left_arm_movej_topic: str = "/left_arm/movej"
    right_arm_movej_topic: str = "/right_arm/movej"
    left_arm_joint_names: tuple[str, ...] = DEFAULT_LEFT_ARM_JOINTS
    right_arm_joint_names: tuple[str, ...] = DEFAULT_RIGHT_ARM_JOINTS
    require_complete_arm_action: bool = True

    lift_state_topic: str = "/lift/joint_states"
    lift_command_topic: str = "/lift/joint_states/update"
    lift_joint_name: str = "lift_joint"
    lift_command_velocity_rpm: float | None = 300.0

    head_command_topic: str = "/head/joint_states/update"
    head_pitch_joint_name: str = "head_pitch_joint"
    head_yaw_joint_name: str = "head_yaw_joint"

    base_velocity_topic: str = "/cmd_vel"
    stop_base_on_disconnect: bool = True

    # Safety defaults from the H1 SDK docs. Values are in SI units/radians.
    enable_safety: bool = True
    max_arm_delta_rad: float = 0.20
    max_lift_delta_m: float = 0.05
    max_head_delta_rad: float = 0.10
    max_base_vx_mps: float = 0.30
    max_base_vy_mps: float = 0.0
    max_base_wz_radps: float = 0.50

    # Observation behavior
    stale_observation_s: float = 2.0
    include_staleness_flags: bool = True

    @property
    def left_action_keys(self) -> tuple[str, ...]:
        return tuple(f"left_arm.{name}.pos" for name in self.left_arm_joint_names)

    @property
    def right_action_keys(self) -> tuple[str, ...]:
        return tuple(f"right_arm.{name}.pos" for name in self.right_arm_joint_names)


if hasattr(RobotConfig, "register_subclass"):
    OneroH1Config = RobotConfig.register_subclass("onero_h1")(OneroH1Config)
