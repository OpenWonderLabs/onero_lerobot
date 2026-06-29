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
  - `/lift/joint_states/update` (`sensor_msgs/msg/JointState`)
  - `/head/joint_states/update` (`sensor_msgs/msg/JointState`)
  - Optional `/cmd_vel` base velocity action, disabled by default

Not included in the first version: MoveP/MoveL, MoveIt2 action clients, Nav2 goals, SLAM map APIs, voice/face APIs, and autonomous docking.

## Install

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

By default, recording stores a hold-position action and does **not** publish actions. Add `--send-hold-action` only if you intentionally want to publish the hold action at each frame.

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
```

The `onero_h1_ros_joint` teleoperator reads ROS2 `JointState` topics and outputs actions matching `OneroH1Robot.action_features`.

## Feature names

Default action feature order:

```text
left_arm.joint1-l.pos ... left_arm.joint7-l.pos
right_arm.joint1-r.pos ... right_arm.joint7-r.pos
lift.pos
head.pitch.pos
head.yaw.pos
```

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

## Upstreaming into LeRobot

For direct integration into LeRobot, the package can be moved under:

```text
src/lerobot/robots/onero_h1/
  __init__.py
  config_onero_h1.py
  onero_h1.py
```

and `OneroH1Robot` / `OneroH1Config` can be registered alongside other robot implementations. The teleoperator can likewise be moved under `src/lerobot/teleoperators/onero_h1/`. The runtime code already follows LeRobot's `Robot` and `Teleoperator` interfaces.
