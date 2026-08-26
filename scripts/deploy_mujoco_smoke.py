#!/usr/bin/env python3
"""Run a conservative scripted deploy smoke test against the H1 ROS interface.

This validates the complete OneroH1Robot observation/action path against the
MuJoCo-backed simulator. It is intentionally not a learned-policy rollout.
"""

from __future__ import annotations

import argparse
import math
import time

from onero_h1_lerobot import OneroH1Config, OneroH1Robot


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Deploy smoke test for the MuJoCo H1 simulator")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--hz", type=float, default=30.0)
    parser.add_argument("--amplitude", type=float, default=0.12)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = OneroH1Config(
        id="mujoco_h1",
        use_cameras=False,
        send_action=True,
        send_gripper_action=True,
        record_data_apply_delta_limit=True,
        connect_timeout_s=5.0,
    )
    robot = OneroH1Robot(config)
    period = 1.0 / max(args.hz, 1.0)

    with robot:
        initial = robot.get_observation()
        left0 = [initial[key] for key in config.left_action_keys]
        right0 = [initial[key] for key in config.right_action_keys]
        started = time.monotonic()
        frames = 0
        while time.monotonic() - started < args.duration:
            frame_start = time.monotonic()
            t = frame_start - started
            wave = args.amplitude * math.sin(0.65 * t)
            action: dict[str, float] = {}
            for i, key in enumerate(config.left_action_keys):
                action[key] = left0[i] + (wave if i < 3 else 0.0) * (1.0 - 0.2 * i)
            for i, key in enumerate(config.right_action_keys):
                action[key] = right0[i] - (wave if i < 3 else 0.0) * (1.0 - 0.2 * i)
            action["left_gripper.pos"] = 0.5 + 0.35 * math.sin(0.4 * t)
            action["right_gripper.pos"] = 0.5 + 0.35 * math.cos(0.4 * t)
            robot.send_action(action)
            frames += 1
            if frames % max(1, round(args.hz * 5)) == 0:
                observation = robot.get_observation()
                print(
                    f"deploy t={t:6.1f}s "
                    f"left_j1={observation['left_arm.joint1-l.pos']:+.3f} "
                    f"right_j1={observation['right_arm.joint1-r.pos']:+.3f}",
                    flush=True,
                )
            sleep_s = period - (time.monotonic() - frame_start)
            if sleep_s > 0:
                time.sleep(sleep_s)

    print(f"deploy smoke complete: {frames} actions")


if __name__ == "__main__":
    main()
