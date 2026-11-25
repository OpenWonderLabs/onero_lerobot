from pathlib import Path
from types import SimpleNamespace

import pytest
import torch
from safetensors.torch import save_file

from onero_h1_lerobot.config import OneroH1Config
from onero_h1_lerobot.feature_layout import make_policy_feature_map
from onero_h1_lerobot.pi05_inference import prepare_pi05_rollout, verify_loaded_pi05_weights


def make_config(tmp_path: Path, *, compile_model=False):
    robot = OneroH1Config(
        policy_feature_layout="interleaved",
        use_lift=False,
        fill_missing_images=False,
        camera_shapes=dict.fromkeys(("head", "left", "right"), (168, 224, 3)),
    )
    for name in ("config.json", "policy_preprocessor.json", "policy_postprocessor.json"):
        (tmp_path / name).write_text("{}")
    save_file({"weight": torch.zeros(1)}, tmp_path / "model.safetensors")
    images = {
        f"observation.images.{name}": SimpleNamespace(shape=(3, 224, 224))
        for name in robot.camera_names
    }
    policy = SimpleNamespace(
        type="pi05",
        use_peft=False,
        use_relative_actions=False,
        action_feature_names=list(make_policy_feature_map(robot)),
        input_features={"observation.state": SimpleNamespace(shape=(16,))},
        output_features={"action": SimpleNamespace(shape=(16,))},
        image_features=images,
        pretrained_path=tmp_path,
        compile_mode="max-autotune",
    )
    return SimpleNamespace(
        robot=robot,
        policy=policy,
        rename_map={},
        fps=30,
        use_torch_compile=compile_model,
        interpolation_multiplier=1,
        inference=SimpleNamespace(rtc=SimpleNamespace(enabled=False)),
    )


def test_prepare_preserves_checkpoint_bytes_and_native_rtc_choice(tmp_path):
    cfg = make_config(tmp_path, compile_model=True)
    before = {path.name: path.read_bytes() for path in tmp_path.iterdir()}
    inference = cfg.inference
    assert prepare_pi05_rollout(cfg) == tmp_path
    assert cfg.robot.require_ready_observations
    assert cfg.robot.action_filter == "none"
    assert cfg.policy.compile_mode == "max-autotune-no-cudagraphs"
    assert cfg.inference is inference and not cfg.inference.rtc.enabled
    assert {path.name: path.read_bytes() for path in tmp_path.iterdir()} == before


def test_compile_is_not_implicitly_enabled_and_filter_uses_command_rate(tmp_path):
    cfg = make_config(tmp_path)
    cfg.interpolation_multiplier = 3
    prepare_pi05_rollout(cfg)
    assert not cfg.use_torch_compile
    assert cfg.policy.compile_mode == "max-autotune"
    assert cfg.robot.action_filter_fps == 90


@pytest.mark.parametrize("fault", ["order", "state", "camera", "missing", "relative", "peft"])
def test_bad_contract_fails_before_connecting(tmp_path, fault):
    cfg = make_config(tmp_path)
    if fault == "order":
        cfg.policy.action_feature_names.reverse()
    elif fault == "state":
        cfg.policy.input_features["observation.state"].shape = (14,)
    elif fault == "camera":
        cfg.rename_map["observation.images.head"] = "observation.images.left"
    elif fault == "missing":
        (tmp_path / "policy_postprocessor.json").unlink()
    elif fault == "relative":
        cfg.policy.use_relative_actions = True
    else:
        cfg.policy.use_peft = True
    with pytest.raises(ValueError):
        prepare_pi05_rollout(cfg)


class SmallPolicy(torch.nn.Module):
    def __init__(self):
        super().__init__()
        self.model = torch.nn.Linear(2, 1)
        self.config = SimpleNamespace()

    def _fix_pytorch_state_dict_keys(self, state, _config):
        # Exercise the real upstream key-fixup extension without allocating PI05.
        from lerobot.policies.pi05.modeling_pi05 import PI05Policy

        return PI05Policy._fix_pytorch_state_dict_keys(self, state, self.config)


def saved_policy(tmp_path):
    policy = SmallPolicy()
    tensors = {name: value.clone() for name, value in policy.model.state_dict().items()}
    save_file(tensors, tmp_path / "model.safetensors")
    return policy, tensors


def test_verifies_actual_loaded_parameters_with_native_key_mapping(tmp_path):
    policy, _ = saved_policy(tmp_path)
    assert verify_loaded_pi05_weights(policy, tmp_path) == {
        "source_tensors": 2,
        "verified_policy_tensors": 2,
    }


def test_dtype_conversion_matches_native_load_state_dict(tmp_path):
    policy, _ = saved_policy(tmp_path)
    policy.to(dtype=torch.bfloat16)
    assert verify_loaded_pi05_weights(policy, tmp_path)["source_tensors"] == 2


def test_catches_random_or_partially_loaded_model_despite_matching_keys(tmp_path):
    policy, _ = saved_policy(tmp_path)
    with torch.no_grad():
        policy.model.weight.add_(1)
    with pytest.raises(ValueError, match="does not match checkpoint"):
        verify_loaded_pi05_weights(policy, tmp_path)


@pytest.mark.parametrize("fault", ["missing", "extra", "nan", "empty"])
def test_corrupt_checkpoint_is_rejected(tmp_path, fault):
    policy, tensors = saved_policy(tmp_path)
    if fault == "missing":
        tensors.pop("bias")
    elif fault == "extra":
        tensors["unexpected"] = torch.zeros(1)
    elif fault == "nan":
        tensors["weight"][0, 0] = float("nan")
    else:
        tensors.clear()
    save_file(tensors, tmp_path / "model.safetensors")
    with pytest.raises(ValueError):
        verify_loaded_pi05_weights(policy, tmp_path)
