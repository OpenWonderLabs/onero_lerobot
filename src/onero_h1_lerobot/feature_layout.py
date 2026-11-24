"""Fixed policy feature names and ordering (pi0.5 / one_vla compatible).

Motor features are exposed to LeRobot as:

    left_joint_1..7, left_gripper, right_joint_1..7, right_gripper

while the ROS control protocol keeps its own hardware names
(``left_arm.joint1-l.pos`` etc.). These helpers translate between the two.
"""

from __future__ import annotations

from typing import Any, TypeVar

T = TypeVar("T")


def make_policy_feature_map(config: Any) -> dict[str, str]:
    """Return ordered policy-name -> hardware-name pairs."""
    required = ("use_left_arm", "use_right_arm", "use_gripper")
    unsupported = (
        "use_arm_velocity_action", "use_lift", "use_head", "use_base_velocity_action",
        "use_base_observation", "use_battery_observation", "use_bumper_observation",
        "use_arm_effort", "use_observation_diff", "use_gripper_pose", "include_staleness_flags",
    )
    if not all(getattr(config, key) for key in required) or any(
        getattr(config, key, False) for key in unsupported
    ):
        raise ValueError(
            "policy layout requires both 7-joint arms and grippers, "
            "with use_lift=false, use_head=false, use_arm_velocity_action=false "
            "and use_base_velocity_action=false; extra scalar observations must be disabled"
        )

    mapping: dict[str, str] = {}
    for side in ("left", "right"):
        joints = getattr(config, f"{side}_arm_joint_names")
        if len(joints) != 7 or len(set(joints)) != 7:
            raise ValueError(f"policy layout requires 7 unique {side} arm joint names")
        mapping.update(
            (f"{side}_joint_{i}.pos", f"{side}_arm.{joint}.pos")
            for i, joint in enumerate(joints, start=1)
        )
        mapping[f"{side}_gripper.pos"] = f"{side}_gripper.pos"
    return mapping


def export_policy_features(values: dict[str, T], mapping: dict[str, str]) -> dict[str, T]:
    """Export motor features in policy order and retain other observations."""
    hardware_keys = set(mapping.values())
    result = {policy: values[hardware] for policy, hardware in mapping.items() if hardware in values}
    result.update((key, value) for key, value in values.items() if key not in hardware_keys)
    return result


def policy_action_to_hardware(
    action: dict[str, float], mapping: dict[str, str]
) -> dict[str, float]:
    unknown = set(action) - mapping.keys()
    if unknown:
        raise ValueError(f"Unknown policy action features: {sorted(unknown)}")
    return {mapping[key]: value for key, value in action.items()}
