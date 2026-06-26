import json
from types import SimpleNamespace

import pytest

from onero_h1_lerobot.cli.lerobot_teleoperate import _ensure_default_fps
from onero_h1_lerobot.config import OneroH1Config
from onero_h1_lerobot.ros_client import H1RosClient
from onero_h1_lerobot.safety import ActionLimiter
from onero_h1_lerobot.teleoperator import OneroH1RosJointTeleop, OneroH1RosJointTeleopConfig
from onero_h1_lerobot.utils import normalize_action_dict, quaternion_to_yaw


def test_quaternion_to_yaw_identity():
    assert quaternion_to_yaw(0.0, 0.0, 0.0, 1.0) == 0.0


def test_action_vector_expansion():
    names = ("a", "b")
    assert normalize_action_dict({"action": [1, 2]}, names) == {"a": 1.0, "b": 2.0}


def test_lift_clipping_and_delta():
    limiter = ActionLimiter(OneroH1Config())
    clipped = limiter.clip_action({"lift.pos": 2.0}, {"lift.pos": 1.0})
    assert clipped["lift.pos"] == 1.05


def test_head_range_clipping():
    cfg = OneroH1Config(max_head_delta_rad=100.0)
    limiter = ActionLimiter(cfg)
    clipped = limiter.clip_action({"head.yaw.pos": 10.0}, {})
    assert clipped["head.yaw.pos"] < 1.58


def test_arm_movej_publishes_h1_string_json_protocol():
    class FakeString:
        data = ""

    class FakePublisher:
        def __init__(self):
            self.messages = []

        def publish(self, msg):
            self.messages.append(msg)

    publisher = FakePublisher()
    client = H1RosClient(OneroH1Config(arm_movej_speed_scale=0.5))
    client.ros = SimpleNamespace(String=FakeString)
    client._pub_left_arm = publisher

    client.publish_arm_movej("left", [0.1, -0.2])

    assert len(publisher.messages) == 1
    assert json.loads(publisher.messages[0].data) == {
        "joints": [0.1, -0.2],
        "speed_scale": 0.5,
    }


def test_record_data_publishes_legacy_low_latency_protocol():
    class FakeFloat64MultiArray:
        def __init__(self):
            self.data = []

    class FakePublisher:
        def __init__(self):
            self.messages = []

        def publish(self, msg):
            self.messages.append(msg)

    publisher = FakePublisher()
    client = H1RosClient(OneroH1Config())
    client.ros = SimpleNamespace(Float64MultiArray=FakeFloat64MultiArray)
    client._pub_record_data = publisher

    client.publish_record_data(
        [0.1] * 7,
        [-0.1] * 7,
        [0.2] * 7,
        [-0.2] * 7,
    )

    assert len(publisher.messages) == 1
    assert publisher.messages[0].data == [0.1] * 7 + [0.2] * 7 + [-0.1] * 7 + [-0.2] * 7


def test_lerobot_teleoperate_defaults_to_100fps_without_overriding_user_value():
    argv = ["onero-h1-lerobot-teleoperate", "--robot.type=onero_h1"]
    _ensure_default_fps(argv)
    assert argv[-1] == "--fps=100"

    argv = ["onero-h1-lerobot-teleoperate", "--fps=60"]
    _ensure_default_fps(argv)
    assert argv == ["onero-h1-lerobot-teleoperate", "--fps=60"]

    argv = ["onero-h1-lerobot-teleoperate", "--fps", "120"]
    _ensure_default_fps(argv)
    assert argv == ["onero-h1-lerobot-teleoperate", "--fps", "120"]


def test_teleop_filters_arm_positions_and_estimates_smooth_velocities():
    cfg = OneroH1RosJointTeleopConfig(
        arm_position_alpha=0.5,
        arm_velocity_alpha=0.5,
        arm_velocity_deadband_radps=0.0,
        max_estimated_arm_velocity_radps=4.0,
    )
    teleop = OneroH1RosJointTeleop(cfg)
    joints = cfg.left_arm_joint_names

    positions0 = {joint: 0.0 for joint in joints}
    filtered0, velocities0 = teleop._filter_arm_action("left", positions0, {}, now=1.0)
    assert filtered0 == positions0
    assert all(value == 0.0 for value in velocities0.values())

    positions1 = {joint: 1.0 for joint in joints}
    filtered1, velocities1 = teleop._filter_arm_action("left", positions1, {}, now=1.01)
    assert all(value == pytest.approx(0.5) for value in filtered1.values())
    assert all(value == pytest.approx(2.0) for value in velocities1.values())

    filtered2, velocities2 = teleop._filter_arm_action("left", positions1, {}, now=1.02)
    assert all(value == pytest.approx(0.75) for value in filtered2.values())
    assert all(value == pytest.approx(1.0) for value in velocities2.values())
