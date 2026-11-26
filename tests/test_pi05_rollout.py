from threading import Event
from types import SimpleNamespace

import pytest
import torch
from test_pi05_inference import make_config, saved_policy

from onero_h1_lerobot.cli.pi05_rollout import run_verified_rollout


def test_weight_guard_precedes_strategy_start_and_never_returns_pose_on_failure(
    tmp_path, monkeypatch
):
    cfg = make_config(tmp_path)
    cfg.display_data = False
    cfg.strategy = SimpleNamespace(type="base")
    cfg.return_to_initial_position = True
    policy, _ = saved_policy(tmp_path)
    with torch.no_grad():
        policy.model.weight.add_(1)
    events = []
    context = SimpleNamespace(
        policy=SimpleNamespace(
            policy=policy, inference=SimpleNamespace(stop=lambda: events.append("stop"))
        ),
        hardware=SimpleNamespace(
            teleop=None,
            robot_wrapper=SimpleNamespace(
                inner=SimpleNamespace(disconnect=lambda: events.append("disconnect")),
            ),
        ),
    )
    monkeypatch.setattr("lerobot.rollout.build_rollout_context", lambda *_args: context)
    monkeypatch.setattr(
        "lerobot.rollout.create_strategy", lambda *_args: pytest.fail("strategy started")
    )
    monkeypatch.setattr(
        "lerobot.utils.process.ProcessSignalHandler",
        lambda **_kwargs: SimpleNamespace(shutdown_event=Event()),
    )
    with pytest.raises(ValueError, match="does not match checkpoint"):
        run_verified_rollout(cfg)
    assert events == ["stop", "disconnect"]


def test_verified_policy_uses_the_native_strategy_lifecycle(tmp_path, monkeypatch):
    cfg = make_config(tmp_path)
    cfg.display_data = False
    cfg.strategy = SimpleNamespace(type="base")
    policy, _ = saved_policy(tmp_path)
    context = SimpleNamespace(policy=SimpleNamespace(policy=policy))
    events = []
    strategy = SimpleNamespace(
        **{
            key: (lambda _ctx, event=key: events.append(event))
            for key in ("setup", "run", "teardown")
        }
    )
    monkeypatch.setattr("lerobot.rollout.build_rollout_context", lambda *_args: context)
    monkeypatch.setattr("lerobot.rollout.create_strategy", lambda *_args: strategy)
    monkeypatch.setattr(
        "lerobot.utils.process.ProcessSignalHandler",
        lambda **_kwargs: SimpleNamespace(shutdown_event=Event()),
    )
    run_verified_rollout(cfg)
    assert events == ["setup", "run", "teardown"]


def test_guarded_context_never_starts_leader_or_sends_teleop_handshake(tmp_path, monkeypatch):
    cfg = make_config(tmp_path)
    cfg.display_data = False
    cfg.robot.auto_start_arm_node = True
    cfg.robot.auto_enter_teleop = True

    def build(config, _event):
        assert not config.robot.auto_start_arm_node
        assert not config.robot.auto_enter_teleop
        raise RuntimeError("stopped before robot connection")

    monkeypatch.setattr("lerobot.rollout.build_rollout_context", build)
    monkeypatch.setattr(
        "lerobot.utils.process.ProcessSignalHandler",
        lambda **_kwargs: SimpleNamespace(shutdown_event=Event()),
    )
    with pytest.raises(RuntimeError, match="before robot connection"):
        run_verified_rollout(cfg)
