import json
from datetime import datetime
from types import SimpleNamespace

import pytest

from onero_h1_lerobot.cli.record_episode import resolve_dataset_target
from onero_h1_lerobot.config import OneroH1Config
from onero_h1_lerobot.native_cli import configure_for_command
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


def test_lerobot_teleoperate_defaults_to_100fps_without_overriding_user_value(monkeypatch):
    # configure_for_command sets process-wide defaults; isolate the test so
    # later tests constructing OneroH1Config see the real defaults.
    for key in (
        "ONERO_H1_ARM_NODE_MODE",
        "ONERO_H1_AUTO_TELEOP",
        "ONERO_H1_SEND_ACTION",
    ):
        monkeypatch.delenv(key, raising=False)

    argv = ["onero-h1-lerobot-teleoperate", "--robot.type=onero_h1"]
    configure_for_command(argv)
    assert argv[-1] == "--fps=100"

    argv = ["onero-h1-lerobot-teleoperate", "--fps=60"]
    configure_for_command(argv)
    assert argv == ["onero-h1-lerobot-teleoperate", "--fps=60"]

    argv = ["onero-h1-lerobot-teleoperate", "--fps", "120"]
    configure_for_command(argv)
    assert argv == ["onero-h1-lerobot-teleoperate", "--fps", "120"]


def test_new_recording_session_gets_timestamped_repo_id(tmp_path):
    repo_id, root = resolve_dataset_target(
        "team/task", str(tmp_path), now=datetime(2026, 11, 19, 15, 30, 0)
    )
    assert repo_id == "team/task_20261119_153000"
    assert root == str(tmp_path / "team" / "task_20261119_153000")


def test_existing_repo_id_stamps_by_default_and_never_overwrites(tmp_path):
    (tmp_path / "team" / "task").mkdir(parents=True)
    with pytest.raises(SystemExit):
        resolve_dataset_target("team/task", str(tmp_path), no_stamp=True)
    repo_id, _ = resolve_dataset_target(
        "team/task", str(tmp_path), now=datetime(2026, 11, 19, 15, 30, 0)
    )
    assert repo_id == "team/task_20261119_153000"


def test_resume_requires_existing_dataset(tmp_path):
    with pytest.raises(SystemExit):
        resolve_dataset_target("team/task", str(tmp_path), resume=True)
    (tmp_path / "team" / "task").mkdir(parents=True)
    repo_id, root = resolve_dataset_target("team/task", str(tmp_path), resume=True)
    assert repo_id == "team/task"
    assert root == str(tmp_path / "team" / "task")


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
