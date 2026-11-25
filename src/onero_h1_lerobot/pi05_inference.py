"""Guards for upstream PI05 rollout; no policy replacement or runtime patching."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

from .config import OneroH1Config
from .feature_layout import make_policy_feature_map


def prepare_pi05_rollout(cfg: Any) -> Path:
    """Validate the robot/model contract before any robot connection."""
    if not isinstance(cfg.robot, OneroH1Config) or cfg.robot.policy_feature_layout != "interleaved":
        raise ValueError("Guarded PI05 rollout requires onero_h1 with interleaved feature layout")
    names = list(make_policy_feature_map(cfg.robot))
    if cfg.policy.type != "pi05" or cfg.policy.use_peft:
        raise ValueError(
            "Guarded PI05 rollout requires a full native PI05 checkpoint, without PEFT"
        )
    if cfg.policy.use_relative_actions:
        raise ValueError("Guarded H1 PI05 rollout requires absolute joint actions")
    if cfg.policy.action_feature_names != names:
        raise ValueError("Checkpoint action names/order do not match the H1 interleaved layout")
    if tuple(cfg.policy.input_features["observation.state"].shape) != (16,):
        raise ValueError("Checkpoint observation.state must have 16 channels")
    if tuple(cfg.policy.output_features["action"].shape) != (16,):
        raise ValueError("Checkpoint action must have 16 channels")
    if not cfg.robot.use_cameras or tuple(cfg.robot.camera_names) != ("head", "left", "right"):
        raise ValueError("Guarded PI05 rollout requires head, left and right cameras")
    mapped_cameras = {
        cfg.rename_map.get(f"observation.images.{name}", f"observation.images.{name}")
        for name in cfg.robot.camera_names
    }
    if len(mapped_cameras) != 3 or mapped_cameras != set(cfg.policy.image_features):
        raise ValueError("Camera rename_map does not match checkpoint image features")
    for name in cfg.robot.camera_names:
        shape = cfg.robot.camera_shapes.get(name, ())
        if (
            len(shape) != 3
            or shape[2] != 3
            or any(type(value) is not int or value <= 0 for value in shape)
        ):
            raise ValueError(f"Camera {name} must declare a positive HWC RGB shape")
    if cfg.robot.fill_missing_images:
        raise ValueError("Guarded PI05 rollout requires robot.fill_missing_images=false")
    if isinstance(cfg.fps, bool) or not math.isfinite(cfg.fps) or cfg.fps <= 0:
        raise ValueError("Rollout FPS must be finite and positive")
    checkpoint = Path(cfg.policy.pretrained_path).expanduser().resolve(strict=True)
    if not checkpoint.is_dir():
        raise ValueError("Guarded PI05 rollout requires a local checkpoint directory")
    for name in (
        "config.json",
        "model.safetensors",
        "policy_preprocessor.json",
        "policy_postprocessor.json",
    ):
        if not (checkpoint / name).is_file():
            raise ValueError(f"Checkpoint is missing {name}")
    # Native rollout already sets compile_model from use_torch_compile. Keep
    # that opt-in; when enabled, avoid CUDA graphs across inference threads.
    if cfg.use_torch_compile:
        cfg.policy.compile_mode = "max-autotune-no-cudagraphs"
    multiplier = cfg.interpolation_multiplier
    if type(multiplier) is not int or multiplier <= 0:
        raise ValueError("interpolation_multiplier must be a positive integer")
    cfg.robot.action_filter_fps = float(cfg.fps * multiplier)
    cfg.robot.require_ready_observations = True
    # The upstream adapter starts a leader control node by default. Guarded
    # policy rollout must not start that independent publisher or a handshake.
    cfg.robot.auto_start_arm_node = False
    cfg.robot.auto_enter_teleop = False
    return checkpoint


def verify_loaded_pi05_weights(policy: Any, checkpoint: Path) -> dict[str, int]:
    """Require the *actual loaded policy* to match every saved tensor.

    Upstream PI05.from_pretrained can catch load errors and return its freshly
    initialized model. Checking only the file or the returned key count cannot
    detect that case. Stream saved tensors through the native key-fixup hook,
    then compare their values after the same dtype conversion as load_state_dict.
    """
    import torch
    from safetensors import safe_open

    path = checkpoint / "model.safetensors"
    before = path.stat()
    actual = policy.state_dict()
    seen: set[str] = set()
    source_count = 0
    with safe_open(path, framework="pt", device="cpu") as archive:
        for source_name in archive.keys():
            source_count += 1
            tensor = archive.get_tensor(source_name)
            if not torch.isfinite(tensor).all().item():
                raise ValueError(f"Checkpoint tensor is non-finite: {source_name}")
            fixed = policy._fix_pytorch_state_dict_keys({source_name: tensor}, policy.config)
            if not fixed:
                raise ValueError(f"Native PI05 key mapping discarded {source_name}")
            for name, expected in fixed.items():
                candidates = [key for key in (name, f"model.{name}") if key in actual]
                if len(candidates) != 1:
                    raise ValueError(f"Checkpoint key does not map to one policy tensor: {name}")
                key = candidates[0]
                loaded = actual[key].detach().cpu()
                if (
                    not torch.isfinite(loaded).all().item()
                    or loaded.shape != expected.shape
                    or not torch.equal(loaded, expected.to(dtype=loaded.dtype))
                ):
                    raise ValueError(f"Loaded PI05 tensor does not match checkpoint: {key}")
                seen.add(key)
    after = path.stat()
    if (before.st_ino, before.st_size, before.st_mtime_ns) != (
        after.st_ino,
        after.st_size,
        after.st_mtime_ns,
    ):
        raise ValueError("Checkpoint changed during weight verification")
    if not source_count or seen != set(actual):
        raise ValueError(
            f"Checkpoint does not cover the loaded policy: missing={len(set(actual) - seen)}"
        )
    return {"source_tensors": source_count, "verified_policy_tensors": len(seen)}
