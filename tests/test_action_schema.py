from onero_h1_lerobot import (
    OneroH1Config,
    OneroH1Robot,
    OneroH1RosJointTeleop,
    OneroH1RosJointTeleopConfig,
)


def test_default_robot_and_homogeneous_teleop_share_canonical_action_schema():
    robot = OneroH1Robot(OneroH1Config(id="schema-test"))
    teleop = OneroH1RosJointTeleop(OneroH1RosJointTeleopConfig(id="schema-test"))

    expected = (
        *(f"left_arm.joint{i}-l.pos" for i in range(1, 8)),
        *(f"right_arm.joint{i}-r.pos" for i in range(1, 8)),
        "left_gripper.pos",
        "right_gripper.pos",
    )

    assert robot.action_feature_names == expected
    assert teleop.action_feature_names == expected
    assert robot.action_features == teleop.action_features
    assert len(expected) == 16

    scalar_observations = [key for key, value in robot.observation_features.items() if value is float]
    assert len(scalar_observations) == 17
    assert scalar_observations[-1] == "lift.pos"


def test_velocity_and_lift_actions_are_explicit_opt_in():
    robot = OneroH1Robot(
        OneroH1Config(
            id="schema-opt-in-test",
            use_arm_velocity_action=True,
            use_lift_action=True,
        )
    )

    assert "left_arm.joint1-l.vel" in robot.action_feature_names
    assert "right_arm.joint7-r.vel" in robot.action_feature_names
    assert "lift.pos" in robot.action_feature_names
