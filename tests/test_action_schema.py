import pytest

from onero_h1_lerobot import (
    OneroH1Config,
    OneroH1Robot,
    OneroH1RosJointTeleop,
    OneroH1RosJointTeleopConfig,
)


def test_default_robot_and_homogeneous_teleop_share_policy_schema():
    robot = OneroH1Robot(OneroH1Config(id="schema-test"))
    teleop = OneroH1RosJointTeleop(OneroH1RosJointTeleopConfig(id="schema-test"))

    expected = (
        *(f"left_joint_{i}.pos" for i in range(1, 8)),
        "left_gripper.pos",
        *(f"right_joint_{i}.pos" for i in range(1, 8)),
        "right_gripper.pos",
    )

    assert robot.action_feature_names == expected
    assert teleop.action_feature_names == expected
    assert robot.action_features == teleop.action_features

    scalar_observations = [key for key, value in robot.observation_features.items() if value is float]
    assert scalar_observations == list(expected)


def test_extra_scalar_channels_are_rejected():
    with pytest.raises(ValueError, match="policy layout"):
        OneroH1Robot(OneroH1Config(id="schema-lift", use_lift=True))
    with pytest.raises(ValueError, match="policy layout"):
        OneroH1Robot(OneroH1Config(id="schema-velocity", use_arm_velocity_action=True))
