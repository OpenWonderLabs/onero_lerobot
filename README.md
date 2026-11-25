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

# Replay (use the printed dataset id; recording appends a timestamp by default)
bash scripts/replay.sh --repo-id my/test_20261119_153000 --episode 0

# Visualize (requires viz deps first)
bash scripts/viz.sh --repo-id my/test_20261119_153000 --episode 0
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

This adapter is pinned to **LeRobot 0.6.1** and requires Python 3.12 or newer.

Choose the installer for the machine where the adapter will run. Do not mix the Jetson and workstation installers.

On NVIDIA Jetson Orin:

```bash
bash scripts/setup_orin_native.sh
```

On a standard Ubuntu 24.04 computer, install the data-collection, video, and dataset-visualization environment first:

```bash
bash scripts/setup.sh
```

If the computer will also run MuJoCo, training, or CUDA rollout, run the second stage afterward:

```bash
bash scripts/setup_ubuntu24_native.sh
```

All installers use `~/.venvs/h1-lerobot`. Do not run an entire installer with `sudo` and do not install these Python dependencies globally. In each new terminal, load the environment with:

```bash
source /opt/ros/jazzy/setup.bash
source ~/.venvs/h1-lerobot/bin/activate
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

Active control is the default. Add `--passive` for passive recording (record only, no control commands).

A new recording session appends a timestamp to `--repo-id` (e.g. `your_name/onero_h1_test_20261119_153000`), matching `lerobot-record`. Use the printed `Dataset:` id for replay/visualization. Pass `--resume` (with the full existing id) to append episodes to an existing dataset, or `--no-stamp` to keep the exact id (`--no-stamp` fails if the directory already exists; nothing is overwritten). Duplicate task names are fine.

**Graceful stop (recommended):** when recording without `--duration`, publish to the stop topic to exit cleanly between frames:

```bash
# Terminal 1: start recording
onero-h1-record-episode --repo-id my/test --task "teleop"

# Terminal 2: stop gracefully
ros2 topic pub /stop_recording std_msgs/msg/Bool "data: true" -1
```

The first Ctrl+C is also handled gracefully: the current frame finishes, the episode is saved, the exit handshake (`21`) is sent, and the robot returns home. A second Ctrl+C forces an immediate exit. The stop topic remains the recommended way to stop from another terminal. See [docs/usage_passive_recording.md](docs/usage_passive_recording.md) for details.

**Homogeneous leader/follower (onero_leader) and VR teleoperation are supported**; heterogeneous teleoperation is not. Select the mode with `--teleop-type homogeneous` (default) or `--teleop-type vr`:

| Teleop type | Arm action input | Gripper action input |
|-------------|-----------------|---------------------|
| `homogeneous` | `/left/joint_states`, `/right/joint_states` | `/joystick_info` (Int32) |
| `vr` | `/left_joint_states`, `/right_joint_states` | `/vr/left_gripper/open_ratio`, `/vr/right_gripper/open_ratio` (Float32) |

When active control starts, LeRobot automatically remaps the leader's `/record_data` to
`/leader/arm_data`; no manual remap is needed. Gripper commands are published to
`/joystick_info` by default; use `--no-gripper-action` to disable them.

## Replaying a recorded episode

Replay a recorded episode on the robot to verify data quality. Use the actual dataset id
printed at recording start/end (it carries a timestamp by default):

```bash
# Replay episode 0 (sends gripper commands by default)
onero-h1-replay-episode --repo-id my/test_20261119_153000 --episode 0

# Replay without gripper commands
onero-h1-replay-episode --repo-id my/test_20261119_153000 --episode 0 --no-gripper

# Replay with custom arm command mode
onero-h1-replay-episode --repo-id my/test_20261119_153000 --episode 0 --arm-command-mode movej

# Using the script
bash scripts/replay.sh --repo-id my/test_20261119_153000 --episode 0
```

The replay reads actions from the dataset and sends them to the robot via `send_action()`, using the same arm command mode and gripper settings as the recording pipeline.

## Visualizing a recorded episode

Inspect the dataset frames offline with LeRobot's built-in viewer. The recommended
installers already provide the required visualization dependency in the project
virtual environment.

**Option 1: Local (Recommended)** — Run directly on the robot's computer with a desktop:

```bash
bash scripts/viz.sh --repo-id my/test_20261119_153000 --episode 0
```

**Option 2: SSH remote** — Start on the robot, connect from your local machine:

```bash
# On the robot
bash scripts/viz.sh --repo-id my/test_20261119_153000 --episode 0 --mode distant

# On your local machine
pip install rerun-sdk
rerun --connect rerun+http://<robot-ip>:9876/proxy
```

**Option 3: Save to file** — Save as rrd, transfer to any machine with a display:

```bash
# On the robot
bash scripts/viz.sh --repo-id my/test_20261119_153000 --episode 0 --save 1 --output-dir ./output

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

> Native `lerobot-record` also handles Ctrl+C gracefully: the plugin saves the in-progress
> episode first, then LeRobot finalizes the dataset and calls `robot.disconnect()` (which sends
> the `21` handshake to return the arms home).

Native commands and wrapper commands apply the same robot defaults (leader arm mode, 20/21 handshake, active control, rollout action sending, and graceful Ctrl+C saving). If your LeRobot version does not auto-load third-party plugins, use the wrapper commands, which import this package before delegating to LeRobot:

```bash
onero-h1-lerobot-teleoperate --robot.type=onero_h1 --teleop.type=onero_h1_ros_joint ...
onero-h1-lerobot-record --robot.type=onero_h1 --teleop.type=onero_h1_ros_joint ...
onero-h1-lerobot-rollout --strategy.type=base --policy.path=/path/to/checkpoint --robot.type=onero_h1 ...
```

The `onero_h1_ros_joint` teleoperator reads ROS2 `JointState` topics and outputs actions matching `OneroH1Robot.action_features`.

## Feature names

The canonical policy state and action each contain 16 values in this order:

```text
left_joint_1.pos ... left_joint_7.pos, left_gripper.pos,
right_joint_1.pos ... right_joint_7.pos, right_gripper.pos
```

Robot and teleoperator expose the same schema. Lift, head, velocity channels and
extra scalar observations must remain disabled for this fixed policy contract.

### Checkpoint feature layouts

The default and only supported layout is `policy_feature_layout=interleaved`.
For SOP compatibility it can also be selected explicitly with
`--robot.policy_feature_layout=interleaved --robot.use_lift=false`:

```text
left_joint_1.pos ... left_joint_7.pos, left_gripper.pos,
right_joint_1.pos ... right_joint_7.pos, right_gripper.pos
```

The adapter exports observations in that order and maps policy actions back to
the corresponding ROS joints and grippers. ROS topic names, joint units, control
messages and existing limit settings are unchanged. This is a fixed position-only
layout: both 7-joint arms and grippers must be enabled, with lift, head, velocity
channels and extra scalar observations disabled.

Use the same layout on both devices when collecting training data:

```bash
lerobot-record \
  --robot.type=onero_h1 \
  --robot.policy_feature_layout=interleaved \
  --robot.use_lift=false \
  --robot.send_action=false \
  --teleop.type=onero_h1_ros_joint \
  --teleop.policy_feature_layout=interleaved \
  --dataset.repo_id=your_name/a1_cloth \
  --dataset.single_task="Fold the cloth" \
  --dataset.push_to_hub=false
```

Start with `lerobot-rollout --robot.policy_feature_layout=interleaved
--robot.use_lift=false --robot.send_action=false ...` and check the complete
checkpoint feature contract before enabling action publishing. The layout is
implemented in this adapter; upstream LeRobot 0.6.1 is not patched.

### Policy rollout

Install the PyTorch build appropriate for the target platform by following the official PyTorch or hardware-vendor instructions, then install this package with its pinned LeRobot dependency. The repository does not bundle platform-specific CUDA wheels or modify LeRobot sources.

For Ubuntu 24.04, ROS2 Jazzy, and an RTX 50-series GPU, install the base environment first and then add the GPU, MuJoCo, and training dependencies:

```bash
bash scripts/setup.sh
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

When active control is enabled (default), control commands are published to:

| Module | Topic | Type | Description |
|--------|-------|------|-------------|
| Arm | `/record_data` | `Float64MultiArray` | 28 floats (left7 pos + left7 vel + right7 pos + right7 vel) |
| Gripper | `/joystick_info` | `Int32` | left=`int(pos*100+100)`, right=`int(pos*100+200)`; `--no-gripper-action` disables |
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
