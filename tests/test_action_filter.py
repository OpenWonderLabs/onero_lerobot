import math

import numpy as np
import pytest

from onero_h1_lerobot.action_filter import ActionFilter
from onero_h1_lerobot.config import OneroH1Config

KEY = "left_arm.joint1-l.pos"


def test_default_filter_preserves_actions_exactly():
    action = {KEY: 0.4, "left_gripper.pos": 0.75}
    filtered = ActionFilter(OneroH1Config()).apply(action, {})
    assert filtered == action
    assert filtered is not action


def test_kalman_starts_from_feedback_and_reduces_alternating_noise():
    smoother = ActionFilter(OneroH1Config(action_filter="kalman"))
    samples = [(-1) ** index * 0.02 for index in range(120)]
    output = [smoother.apply({KEY: value}, {KEY: 0.0})[KEY] for value in samples]
    assert 0 < output[0] < samples[0]
    assert np.std(output[20:]) < np.std(samples[20:]) / 2


def test_circular_joint_does_not_interpolate_across_zero():
    smoother = ActionFilter(OneroH1Config(action_filter="kalman"))
    values = [smoother.apply({KEY: value}, {KEY: 3.13})[KEY] for value in (-3.13, -3.12)]
    assert all(abs(value - 3.13) < 0.03 for value in values)


def test_filter_reset_uses_new_episode_feedback():
    smoother = ActionFilter(OneroH1Config(action_filter="kalman"))
    smoother.apply({KEY: 1.0}, {KEY: 0.0})
    smoother.reset()
    assert smoother.apply({KEY: -1.0}, {KEY: -1.0})[KEY] == -1.0


def test_clipping_does_not_leave_hidden_velocity_beyond_joint_limit():
    smoother = ActionFilter(OneroH1Config(action_filter="kalman"))
    smoother.apply({KEY: 2.0}, {KEY: 0.0})
    smoother.accept_limited({KEY: 0.0})
    state = smoother._states[KEY][0]
    np.testing.assert_array_equal(state, [0.0, 0.0])


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_invalid_command_is_atomic(value):
    smoother = ActionFilter(OneroH1Config(action_filter="kalman"))
    with pytest.raises(ValueError, match="non-finite"):
        smoother.apply({KEY: 0.3, "left_gripper.pos": value}, {KEY: 0.0})
    assert smoother._states == {}


def test_feedback_is_required_instead_of_seeding_from_a_large_requested_jump():
    with pytest.raises(ValueError, match="finite observation"):
        ActionFilter(OneroH1Config(action_filter="kalman")).apply({KEY: 1.0}, {})


@pytest.mark.parametrize(
    "setting",
    [
        {"action_filter": "typo"},
        {"action_filter_fps": 0},
        {"kalman_process_variance": -1},
        {"kalman_threshold": math.nan},
        {"kalman_jump_variance": math.inf},
    ],
)
def test_invalid_filter_configuration_is_rejected(setting):
    with pytest.raises(ValueError):
        ActionFilter(OneroH1Config(**setting))
