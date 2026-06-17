"""CLI: send one conservative test action through the LeRobot adapter."""

from __future__ import annotations

import argparse
import time

from onero_h1_lerobot import OneroH1Config, OneroH1Robot


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send a small, clipped test action to Onero H1.")
    parser.add_argument("--id", default="onero_h1")
    parser.add_argument("--dry-run", action="store_true", help="Print the action without connecting or publishing")
    parser.add_argument("--include-arms", action="store_true", help="Also send current arm positions back as MoveJ")
    parser.add_argument("--head-yaw", type=float, default=0.05, help="Small head yaw target in radians")
    parser.add_argument("--head-pitch", type=float, default=0.0, help="Small head pitch target in radians")
    parser.add_argument("--lift", type=float, default=None, help="Optional lift target height in meters")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = OneroH1Config(
        id=args.id,
        use_cameras=False,
        use_left_arm=args.include_arms,
        use_right_arm=args.include_arms,
        require_complete_arm_action=False,
    )
    robot = OneroH1Robot(config)

    action = {
        "head.pitch.pos": args.head_pitch,
        "head.yaw.pos": args.head_yaw,
    }
    if args.lift is not None:
        action["lift.pos"] = args.lift

    if args.dry_run:
        print("Dry-run action:", action)
        print("Action features:", robot.action_features)
        return

    print("WARNING: robot will receive a small head/lift command.")
    print("Ensure the robot is clear, enabled, and supervised. Press Ctrl+C to abort.")
    time.sleep(3)

    with robot:
        if args.include_arms:
            obs = robot.get_observation()
            for key in robot.action_feature_names:
                if key.startswith("left_arm.") or key.startswith("right_arm."):
                    action[key] = float(obs.get(key, 0.0))
        sent = robot.send_action(action)
        print("Sent clipped action:", sent)
        time.sleep(1.0)
        # Return head near zero after the test.
        robot.send_action({"head.pitch.pos": 0.0, "head.yaw.pos": 0.0})


if __name__ == "__main__":
    main()
