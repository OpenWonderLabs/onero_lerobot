import json
from types import SimpleNamespace

from onero_h1_lerobot.config import OneroH1Config
from onero_h1_lerobot.ros_client import H1RosClient
from onero_h1_lerobot.safety import ActionLimiter
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
