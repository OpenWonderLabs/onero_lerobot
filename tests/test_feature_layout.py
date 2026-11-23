from types import SimpleNamespace

import numpy as np
import pytest

from onero_h1_lerobot import (
    OneroH1Config,
    OneroH1Robot,
    OneroH1RosJointTeleop,
    OneroH1RosJointTeleopConfig,
)
from onero_h1_lerobot.ros_client import H1RosClient

MODEL_ORDER = (
    *(f"left_joint_{i}.pos" for i in range(1, 8)),
    "left_gripper.pos",
    *(f"right_joint_{i}.pos" for i in range(1, 8)),
    "right_gripper.pos",
)
LEFT = [-0.1, 0.1, 0.2, -1.3, 0.3, -0.2, 0.03]
RIGHT = [0.15, -0.2, 0.1, 1.3, 0.2, 0.3, -0.02]
MODEL_VALUES = [*LEFT, 0.37, *RIGHT, 0.81]


class Message:
    def __init__(self):
        self.data = None


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message.data)


def make_robot(*, send_action=False, **kwargs):
    config = OneroH1Config(
        policy_feature_layout="interleaved",
        use_lift=False,
        use_cameras=False,
        send_action=send_action,
        **kwargs,
    )
    robot = OneroH1Robot(config)
    client = H1RosClient(config)
    client._connected = True
    client.ros = SimpleNamespace(Float64MultiArray=Message, Int32=Message)
    client._pub_record_data = Publisher()
    client._pub_gripper = Publisher()
    client._joint_pos = {
        **dict(zip(config.left_arm_joint_names, LEFT, strict=True)),
        **dict(zip(config.right_arm_joint_names, RIGHT, strict=True)),
    }
    client._left_gripper_state = 0.37
    client._right_gripper_state = 0.81
    robot.client = client
    return robot, client


def test_model_state_action_and_teleop_have_identical_names_and_order():
    robot, _ = make_robot()
    teleop = OneroH1RosJointTeleop(
        OneroH1RosJointTeleopConfig(policy_feature_layout="interleaved")
    )
    assert robot.action_feature_names == MODEL_ORDER
    assert teleop.action_feature_names == MODEL_ORDER
    assert tuple(robot.observation_features) == MODEL_ORDER
    assert tuple(robot.action_features) == MODEL_ORDER
    observation = robot.get_observation()
    assert tuple(observation) == MODEL_ORDER
    assert list(observation.values()) == pytest.approx(MODEL_VALUES)


@pytest.mark.parametrize("vector", [False, True])
def test_model_outputs_route_to_the_correct_28_value_packet_and_grippers(vector):
    robot, client = make_robot(send_action=True)
    action = {"action": np.asarray(MODEL_VALUES)} if vector else dict(zip(MODEL_ORDER, MODEL_VALUES, strict=True))
    sent = robot.send_action(action)
    assert tuple(sent) == MODEL_ORDER
    assert list(sent.values()) == pytest.approx(MODEL_VALUES)
    packet, = client._pub_record_data.messages
    assert len(packet) == 28
    assert packet[:7] == pytest.approx(LEFT)
    assert packet[14:21] == pytest.approx(RIGHT)
    assert client._pub_gripper.messages == [137, 281]


def test_receive_only_mode_never_publishes_mapped_actions():
    robot, client = make_robot(send_action=False)
    robot.send_action({"action": MODEL_VALUES})
    assert client._pub_record_data.messages == []
    assert client._pub_gripper.messages == []


def test_unknown_action_names_are_rejected_before_publication():
    robot, client = make_robot(send_action=True)
    with pytest.raises(ValueError, match="Unknown interleaved action"):
        robot.send_action({"left_arm.joint1-l.pos": 0.1})
    assert client._pub_record_data.messages == []
    assert client._pub_gripper.messages == []


def test_teleop_converts_real_callback_values_to_the_model_order():
    cfg = OneroH1RosJointTeleopConfig(
        policy_feature_layout="interleaved", smooth_arm_actions=False, use_action_diff=False
    )
    teleop = OneroH1RosJointTeleop(cfg)
    teleop._connected = True
    for side, positions in [("left", LEFT), ("right", RIGHT)]:
        names = getattr(cfg, f"{side}_arm_joint_names")
        teleop._on_arm(side, names, SimpleNamespace(name=names, position=positions, velocity=[]))
    teleop._on_gripper(SimpleNamespace(data=137))
    teleop._on_gripper(SimpleNamespace(data=281))
    action = teleop.get_action()
    assert tuple(action) == MODEL_ORDER
    assert list(action.values()) == pytest.approx(MODEL_VALUES)


@pytest.mark.parametrize(
    "settings",
    [
        {"policy_feature_layout": "typo"},
        {"use_left_arm": False},
        {"use_right_arm": False},
        {"use_gripper": False},
        {"use_lift": True},
        {"use_head": True},
        {"use_arm_velocity_action": True},
        {"use_base_velocity_action": True},
        {"left_arm_joint_names": ("same",) * 7},
        {"right_arm_joint_names": ("one",)},
    ],
)
@pytest.mark.parametrize(
    "config_type,device_type", [(OneroH1Config, OneroH1Robot), (OneroH1RosJointTeleopConfig, OneroH1RosJointTeleop)]
)
def test_invalid_layout_is_rejected_before_connecting(config_type, device_type, settings):
    options = {"policy_feature_layout": "interleaved", "use_lift": False, **settings}
    with pytest.raises(ValueError):
        device_type(config_type(**options))


def test_custom_ros_joint_names_keep_policy_names_and_wire_order():
    robot, client = make_robot(
        send_action=True,
        left_arm_joint_names=tuple(f"custom_left_{i}" for i in range(7)),
        right_arm_joint_names=tuple(f"custom_right_{i}" for i in range(7)),
    )
    assert list(robot.get_observation().values()) == pytest.approx(MODEL_VALUES)
    robot.send_action({"action": MODEL_VALUES})
    assert client._pub_record_data.messages[0][:7] == pytest.approx(LEFT)
    assert client._pub_record_data.messages[0][14:21] == pytest.approx(RIGHT)


@pytest.mark.parametrize("flag", [
    "use_base_observation", "use_battery_observation", "use_bumper_observation",
    "use_arm_effort", "use_observation_diff", "use_gripper_pose", "include_staleness_flags",
])
def test_interleaved_state_cannot_silently_gain_extra_scalar_channels(flag):
    with pytest.raises(ValueError, match="extra scalar observations"):
        make_robot(**{flag: True})


def test_native_lerobot_flattens_observations_and_labels_actions_in_model_order():
    pytest.importorskip("lerobot")
    import torch
    from lerobot.policies.utils import make_robot_action
    from lerobot.utils import feature_utils
    robot, client = make_robot(send_action=True)
    obs_features = feature_utils.hw_to_dataset_features(robot.observation_features, "observation")
    frame = feature_utils.build_dataset_frame(obs_features, robot.get_observation(), prefix="observation")
    np.testing.assert_allclose(frame["observation.state"], MODEL_VALUES)
    features = feature_utils.hw_to_dataset_features(robot.action_features, "action")
    action = make_robot_action(torch.tensor(MODEL_VALUES), features)
    robot.send_action(action)
    assert client._pub_record_data.messages[0][:7] == pytest.approx(LEFT)
    assert client._pub_record_data.messages[0][14:21] == pytest.approx(RIGHT)
