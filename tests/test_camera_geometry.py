from types import SimpleNamespace

import cv2
import numpy as np
import pytest
import torch
from lerobot.policies.pi05.modeling_pi05 import PI05Policy
from lerobot.policies.utils import prepare_observation_for_inference
from lerobot.utils.feature_utils import build_dataset_frame, hw_to_dataset_features

from onero_h1_lerobot import OneroH1Config, OneroH1Robot
from onero_h1_lerobot.ros_client import H1RosClient


def jpeg(height=240, width=320):
    bgr = np.zeros((height, width, 3), dtype=np.uint8)
    bgr[:, : width // 2, 0] = 255
    bgr[:, width // 2 :, 2] = 255
    ok, data = cv2.imencode(".jpg", bgr)
    assert ok
    return data.tobytes()


def test_native_rollout_keeps_rgb_rectangular_geometry_until_pi05_padding():
    cfg = OneroH1Config(
        policy_feature_layout="interleaved",
        use_lift=False,
        fill_missing_images=False,
        require_ready_observations=True,
        camera_shapes=dict.fromkeys(("head", "left", "right"), (168, 224, 3)),
    )
    robot = OneroH1Robot(cfg)
    client = H1RosClient(cfg)
    decoded = client._decode_compressed_image("head", jpeg())
    assert decoded.shape == (168, 224, 3) and decoded.dtype == np.uint8
    assert decoded[80, 40, 2] > 240 and decoded[80, 40, 0] < 5
    assert decoded[80, 180, 0] > 240 and decoded[80, 180, 2] < 5
    observation = dict.fromkeys(robot.action_feature_names, 0.0)
    observation.update(dict.fromkeys(cfg.camera_names, decoded))
    features = hw_to_dataset_features(robot.observation_features, "observation")
    frame = build_dataset_frame(features, observation, prefix="observation")
    batch = prepare_observation_for_inference(frame, torch.device("cpu"), "fixture", "onero_h1")
    assert batch["observation.images.head"].shape == (1, 3, 168, 224)
    probe = SimpleNamespace(
        config=SimpleNamespace(
            image_resolution=(224, 224),
            image_features={f"observation.images.{name}": None for name in cfg.camera_names},
        ),
        parameters=lambda: iter([torch.zeros(1)]),
    )
    images, _ = PI05Policy._preprocess_images(probe, batch)
    for image in images:
        assert image.shape == (1, 3, 224, 224)
        assert torch.all(image[:, :, :28] == -1)
        assert torch.all(image[:, :, -28:] == -1)
        torch.testing.assert_close(
            image[:, :, 28:-28],
            batch["observation.images.head"] * 2 - 1,
            rtol=0,
            atol=0,
        )


@pytest.mark.parametrize("height,width", [(240, 480), (120, 160)])
def test_strict_geometry_rejects_stretching_and_upscaling(height, width):
    cfg = OneroH1Config(
        require_ready_observations=True,
        fill_missing_images=False,
        camera_shapes={"head": (168, 224, 3)},
    )
    assert H1RosClient(cfg)._decode_compressed_image("head", jpeg(height, width)) is None
