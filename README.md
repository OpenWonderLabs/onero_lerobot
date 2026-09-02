# lerobot_robot_onero_h1

LeRobot adapter for the Onero H1 / SwitchBot H1 ROS2 SDK.

This package wraps the ROS2 topics documented in the Onero H1 SDK docs as a standard LeRobot `Robot`:

```python
from onero_h1_lerobot import OneroH1Config, OneroH1Robot

robot = OneroH1Robot(OneroH1Config(id="h1"))
robot.connect()
obs = robot.get_observation()
sent = robot.send_action({"head.yaw.pos": 0.05, "head.pitch.pos": 0.0})
robot.disconnect()
```

## Scripts Overview

| Script | CLI Command | Purpose | Requires Robot |
|--------|-------------|---------|:---:|
| [record.sh](scripts/record.sh) | `onero-h1-record-episode` | Record teleoperation data | ✅ |
| [replay.sh](scripts/replay.sh) | `onero-h1-replay-episode` | Replay data to robot | ✅ |
| [viz.sh](scripts/viz.sh) | `lerobot-dataset-viz` | Offline dataset visualization | ❌ |

```bash
# Record
bash scripts/record.sh --repo-id my/test --task "pick cup" --duration 60

# Replay
bash scripts/replay.sh --repo-id my/test --episode 0

# Visualize (requires viz deps first)
bash scripts/viz.sh --repo-id my/test --episode 0
```

> See [docs/usage_passive_recording.md](docs/usage_passive_recording.md) for detailed usage.

## Scope

Implemented against the current SDK documentation:

- Observations
  - `/joint_states` for aggregated joint positions when available
  - `/left_joint_states`, `/right_joint_states`, and `/head/joint_states` for arm/head joint positions
  - `/lift/joint_states` for lift height
  - `/odom` for base pose and velocity
  - `/battery/state` and `/front_bumper`
  - `/head/camera/rgb`, `/left/camera/rgb`, `/right/camera/rgb` compressed RGB streams (`sensor_msgs/msg/CompressedImage`, subscribed with BEST_EFFORT QoS to match the publishers' `SensorDataQoS`)
- Actions
  - Dual-arm joint command, selectable via `arm_command_mode`:
    - `record_data` (default): a single `/record_data` (`std_msgs/msg/Float64MultiArray`) carrying `[left positions, left velocities, right positions, right velocities]`
    - `movej`: `/left_arm/movej` and `/right_arm/movej` (`std_msgs/msg/String` JSON with `joints` and optional `speed_scale`)
  - Gripper command via `/joystick_info` (`std_msgs/msg/Int32`, encoding: left=`int(pos*100+100)`, right=`int(pos*200+200)`)
  - `/lift/joint_states/update` (`sensor_msgs/msg/JointState`)
  - `/head/joint_states/update` (`sensor_msgs/msg/JointState`)
  - Optional `/cmd_vel` base velocity action, disabled by default

Not included in the first version: MoveP/MoveL, MoveIt2 action clients, Nav2 goals, SLAM map APIs, voice/face APIs, and autonomous docking.

## Install

This adapter is pinned to **LeRobot 0.6.1**, the latest stable non-prerelease release verified on PyPI and GitHub. It requires Python 3.12 or newer.

On the robot or ROS2 workstation:

```bash
source /opt/ros/jazzy/setup.bash
cd packages/onero_h1_lerobot
python3 -m pip install -e .
# Optional: if LeRobot is not already installed
python3 -m pip install -e '.[lerobot]'
```

`rclpy` and ROS message packages are provided by ROS2, not by PyPI. Make sure your H1 ROS workspace is sourced before running the adapter.

Camera decoding needs `cv2`, but the base install does not force a PyPI OpenCV download. Check the current environment first:

```bash
python3 -c "import cv2; print(cv2.__version__)" || python3 -m pip install -e '.[camera]'
```

## Quick checks

Print observations without cameras:

```bash
onero-h1-print-observation --no-cameras --count 5
```

Print observations with the default cameras:

```bash
onero-h1-print-observation --cameras head,left,right --rate 2
```

Send a conservative head motion test:

```bash
onero-h1-safe-test-action --head-yaw 0.05
```

Dry-run the test action without connecting:

```bash
onero-h1-safe-test-action --dry-run
```

## Recording a LeRobot dataset

Minimal package-local recorder:

```bash
onero-h1-record-episode \
  --repo-id your_name/onero_h1_test \
  --task "inspect the table" \
  --duration 20 \
  --fps 10 \
  --cameras head,left,right \
  --finalize
```

By default, recording stores a hold-position action and does **not** publish actions. Add `--send-hold-action` to publish control commands to the robot.

**Graceful stop (recommended):** when recording without `--duration`, publish to the stop topic to exit cleanly between frames:

```bash
# Terminal 1: start recording
onero-h1-record-episode --repo-id my/test --task "teleop" --teleop-type homogeneous

# Terminal 2: stop gracefully
ros2 topic pub /stop_recording std_msgs/msg/Bool "data: true" -1
```

This avoids the truncated image / partial frame issues that Ctrl+C can cause. Ctrl+C is disabled; use the stop topic instead. See [docs/usage_passive_recording.md](docs/usage_passive_recording.md) for details.

**Teleop types** (set via `--teleop-type`):

| Type | Arm action input | Gripper action input | send_action output |
|------|-----------------|---------------------|--------------------|
| `homogeneous` (default) | `/left/joint_states`, `/right/joint_states` | `/joystick_info` (Int32) | `/record_data` only (no gripper) |
| `heterogeneous` | `/teleop/left/joint_states`, `/teleop/right/joint_states` | `/joystick_info` (Int32) | TBD |
| `vr` | `/left_joint_states`, `/right_joint_states` | `/vr/left_gripper/open_ratio`, `/vr/right_gripper/open_ratio` (Float32) | TBD |

**Note on homogeneous mode with `--send-hold-action`:** the leader arm program directly publishes to `/record_data` and `/joystick_info` to control the follower. To let oneroh1lerobot take over, remap the leader's topics:

```bash
# Leader arm side
your_leader_program \
    --ros-args -r /record_data:=/leader/arm_data \
               -r /joystick_info:=/leader/joystick

# Recording side
onero-h1-record-episode \
    --repo-id my/test --task "teleop_test" \
    --duration 180 --teleop-type homogeneous \
    --action-gripper-topic /leader/joystick \
    --send-hold-action
```

In homogeneous mode, only arm joint commands are published via `/record_data`; gripper commands are **not** published, since the leader arm handles gripper directly.

## Replaying a recorded episode

Replay a recorded episode on the robot to verify data quality:

```bash
# Replay episode 0 (sends gripper commands by default)
onero-h1-replay-episode --repo-id my/test --episode 0

# Replay without gripper commands
onero-h1-replay-episode --repo-id my/test --episode 0 --no-gripper

# Replay with custom arm command mode
onero-h1-replay-episode --repo-id my/test --episode 0 --arm-command-mode movej

# Using the script
bash scripts/replay.sh --repo-id my/test --episode 0
```

The replay reads actions from the dataset and sends them to the robot via `send_action()`, using the same arm command mode and gripper settings as the recording pipeline.

## Visualizing a recorded episode

Inspect the dataset frames offline with LeRobot's built-in viewer:

```bash
# Uses the pinned release: pip install 'lerobot[dataset-viz]==0.6.1' --break-system-packages
```

**Option 1: Local (Recommended)** — Run directly on the robot's computer with a desktop:

```bash
bash scripts/viz.sh --repo-id my/test --episode 0
```

**Option 2: SSH remote** — Start on the robot, connect from your local machine:

```bash
# On the robot
bash scripts/viz.sh --repo-id my/test --episode 0 --mode distant

# On your local machine
pip install rerun-sdk
rerun --connect rerun+http://<robot-ip>:9876/proxy
```

**Option 3: Save to file** — Save as rrd, transfer to any machine with a display:

```bash
# On the robot
bash scripts/viz.sh --repo-id my/test --episode 0 --save 1 --output-dir ./output

# Transfer and view locally
scp -r wlab@<robot-ip>:~/oneroh1lerobot/output ./output
pip install rerun-sdk
rerun output/*.rrd
```

This opens a rerun viewer showing camera images, joint positions, and actions frame by frame.

## LeRobot CLI usage

This package is also installed as a LeRobot third-party plugin. With recent LeRobot versions, native commands should discover it after installation:

```bash
lerobot-teleoperate \
  --robot.type=onero_h1 \
  --robot.id=h1 \
  --robot.use_cameras=true \
  --robot.camera_names='["head", "left", "right"]' \
  --teleop.type=onero_h1_ros_joint \
  --teleop.id=exo \
  --teleop.left_arm_topic=/teleop/left_arm/joint_states \
  --teleop.right_arm_topic=/teleop/right_arm/joint_states \
  --teleop.lift_topic=/teleop/lift/joint_states \
  --teleop.head_topic=/teleop/head/joint_states \
  --display_data=true
```

```bash
lerobot-record \
  --robot.type=onero_h1 \
  --robot.id=h1 \
  --robot.use_cameras=true \
  --robot.camera_names='["head", "left", "right"]' \
  --teleop.type=onero_h1_ros_joint \
  --teleop.id=exo \
  --teleop.left_arm_topic=/teleop/left_arm/joint_states \
  --teleop.right_arm_topic=/teleop/right_arm/joint_states \
  --teleop.lift_topic=/teleop/lift/joint_states \
  --teleop.head_topic=/teleop/head/joint_states \
  --dataset.repo_id=your_name/onero_h1_test \
  --dataset.single_task="Test" \
  --dataset.num_episodes=2 \
  --dataset.episode_time_s=5 \
  --dataset.reset_time_s=5 \
  --dataset.push_to_hub=true \
  --dataset.streaming_encoding=true \
  --dataset.encoder_threads=2
```

If your LeRobot version does not auto-load third-party plugins, use the wrapper commands, which import this package before delegating to LeRobot:

```bash
onero-h1-lerobot-teleoperate --robot.type=onero_h1 --teleop.type=onero_h1_ros_joint ...
onero-h1-lerobot-record --robot.type=onero_h1 --teleop.type=onero_h1_ros_joint ...
onero-h1-lerobot-rollout --strategy.type=base --policy.path=/path/to/checkpoint --robot.type=onero_h1 ...
```

The `onero_h1_ros_joint` teleoperator reads ROS2 `JointState` topics and outputs actions matching `OneroH1Robot.action_features`.

## Feature names

The default canonical policy action has exactly 16 values:

```text
left_arm.joint1-l.pos ... left_arm.joint7-l.pos
right_arm.joint1-r.pos ... right_arm.joint7-r.pos
left_gripper.pos, right_gripper.pos
```

Robot and homogeneous teleoperator expose this exact order. Arm velocities and lift are explicit opt-in action channels (`use_arm_velocity_action=True`, `use_lift_action=True`) and should remain disabled for the first dataset. The default policy state has 17 scalars (14 arm positions + 2 grippers + lift), matching LeRobot rollout's hardware feature filter. Effort, diff, gripper-pose, and base observations are opt-in and must be configured identically during training and deployment.

### Policy rollout

Install the PyTorch build appropriate for the target platform by following the official PyTorch or hardware-vendor instructions, then install this package with its pinned LeRobot dependency. The repository does not bundle platform-specific CUDA wheels or modify LeRobot sources.

For Ubuntu 24.04, ROS2 Jazzy, and an RTX 50-series GPU, use the native installer after ROS2 Jazzy is installed:

```bash
bash scripts/setup_ubuntu24_native.sh
```

Run the complete MuJoCo recording, ACT GPU training, and checkpoint rollout validation:

```bash
bash scripts/run_sim_train_rollout.sh
```

Outputs are written to `~/onero_h1_runs/<timestamp>/` by default. Override settings with environment variables such as `H1_VALIDATION_ROOT`, `H1_TRAIN_STEPS`, and `H1_FPS`.

Source the local ROS2 environment and run the policy through the native LeRobot rollout path:

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash

onero-h1-lerobot-rollout \
  --strategy.type=base \
  --policy.path=/path/to/checkpoint \
  --robot.type=onero_h1 \
  --robot.id=h1 \
  --device=<cpu-or-cuda>
```

The adapter calls `rclpy` directly in the same Python environment. Training and deployment must use matching observation and action schemas.

When `--send-hold-action` is enabled, control commands are published to:

| Module | Topic | Type | Description |
|--------|-------|------|-------------|
| Arm | `/record_data` | `Float64MultiArray` | 28 floats (left7 pos + left7 vel + right7 pos + right7 vel) |
| Gripper | `/joystick_info` | `Int32` | left=`int(pos*100+100)`, right=`int(pos*100+200)` (heterogeneous/vr only) |
| Lift | `/lift/joint_states/update` | `JointState` | when enabled |
| Head | `/head/joint_states/update` | `JointState` | when enabled |
| Base | `/cmd_vel` | `Twist` | when enabled |

If the robot URDF or driver uses different joint names, override them in `OneroH1Config`:

```python
config = OneroH1Config(
    left_arm_joint_names=("left_joint_1", "left_joint_2", "left_joint_3", "left_joint_4", "left_joint_5", "left_joint_6", "left_joint_7"),
    right_arm_joint_names=("right_joint_1", "right_joint_2", "right_joint_3", "right_joint_4", "right_joint_5", "right_joint_6", "right_joint_7"),
)
```

## Safety

The adapter clips actions using conservative limits from the SDK docs:

- arm joints: documented J1-J7 ranges, max delta `0.20 rad` per call
- lift: `0.40 m` to `1.30 m`, max delta `0.05 m` per call
- head pitch: `±28°`, yaw: `±90°`, max delta `0.10 rad` per call
- base velocity action disabled by default

This does not replace collision checking, MoveIt planning, hardware emergency stop, or human supervision.

## Other Tools

| Command | Purpose |
|---------|---------|
| `onero-h1-print-observation` | Print current observations for debugging |
| `onero-h1-safe-test-action` | Send a safe test action (head motion), supports `--dry-run` |

## Upstreaming into LeRobot

For direct integration into LeRobot, the package can be moved under:

```text
src/lerobot/robots/onero_h1/
  __init__.py
  config_onero_h1.py
  onero_h1.py
```

and `OneroH1Robot` / `OneroH1Config` can be registered alongside other robot implementations. The teleoperator can likewise be moved under `src/lerobot/teleoperators/onero_h1/`. The runtime code already follows LeRobot's `Robot` and `Teleoperator` interfaces.
