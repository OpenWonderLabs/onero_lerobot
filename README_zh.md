# lerobot_robot_onero_h1

适用于 Onero H1 / SwitchBot H1 ROS2 SDK 的 LeRobot 适配器。

本包将 Onero H1 SDK 文档中描述的 ROS2 话题封装为标准的 LeRobot `Robot`：

```python
from onero_h1_lerobot import OneroH1Config, OneroH1Robot

robot = OneroH1Robot(OneroH1Config(id="h1"))
robot.connect()
obs = robot.get_observation()
sent = robot.send_action({"head.yaw.pos": 0.05, "head.pitch.pos": 0.0})
robot.disconnect()
```

## 支持范围

基于当前 SDK 文档实现：

- 观测（Observations）
  - `/joint_states`：在可用时提供聚合的关节位置
  - `/left_joint_states`、`/right_joint_states`、`/head/joint_states`：手臂/头部关节位置
  - `/lift/joint_states`：升降高度
  - `/odom`：底盘位姿与速度
  - `/battery/state` 和 `/front_bumper`
  - `/head/camera/rgb`、`/left/camera/rgb`、`/right/camera/rgb` 压缩 RGB 图像流（`sensor_msgs/msg/CompressedImage`，订阅端使用 BEST_EFFORT QoS，以匹配发布端的 `SensorDataQoS`）
- 动作（Actions）
  - 双臂关节指令，可通过 `arm_command_mode` 选择：
    - `record_data`（默认）：单条 `/record_data`（`std_msgs/msg/Float64MultiArray`），数据布局为 `[左臂位置, 左臂速度, 右臂位置, 右臂速度]`
    - `movej`：`/left_arm/movej` 和 `/right_arm/movej`（`std_msgs/msg/String` JSON 格式，包含 `joints` 和可选的 `speed_scale`）
  - `/lift/joint_states/update`（`sensor_msgs/msg/JointState`）
  - `/head/joint_states/update`（`sensor_msgs/msg/JointState`）
  - 可选的 `/cmd_vel` 底盘速度动作，默认关闭

第一版未包含的内容：MoveP/MoveL、MoveIt2 action 客户端、Nav2 目标点、SLAM 地图 API、语音/人脸 API 以及自主回充。

## 安装

在机器人或 ROS2 工作站上：

```bash
source /opt/ros/jazzy/setup.bash
cd packages/onero_h1_lerobot
python3 -m pip install -e .
# 可选：如果尚未安装 LeRobot
python3 -m pip install -e '.[lerobot]'
```

`rclpy` 和 ROS 消息包由 ROS2 提供，不通过 PyPI 安装。运行适配器之前，请确保已 source H1 的 ROS 工作空间。

相机解码需要 `cv2`，但基础安装不会强制从 PyPI 下载 OpenCV。请先检查当前环境：

```bash
python3 -c "import cv2; print(cv2.__version__)" || python3 -m pip install -e '.[camera]'
```

## 快速检查

不带相机打印观测：

```bash
onero-h1-print-observation --no-cameras --count 5
```

使用默认相机打印观测：

```bash
onero-h1-print-observation --cameras head,left,right --rate 2
```

发送一个保守的头部运动测试：

```bash
onero-h1-safe-test-action --head-yaw 0.05
```

不连接机器人，仅做测试动作的 dry-run：

```bash
onero-h1-safe-test-action --dry-run
```

## 录制 LeRobot 数据集

包内置的最小化录制器：

```bash
onero-h1-record-episode \
  --repo-id your_name/onero_h1_test \
  --task "inspect the table" \
  --duration 20 \
  --fps 10 \
  --cameras head,left,right \
  --finalize
```

默认情况下，录制会存储保持当前位置的动作，并且**不会**发布动作。只有在明确希望每帧发布保持动作时，才添加 `--send-hold-action`。

## LeRobot CLI 用法

本包也作为 LeRobot 的第三方插件安装。在较新的 LeRobot 版本中，安装后原生命令应能自动发现：

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

如果你使用的 LeRobot 版本不支持自动加载第三方插件，可以使用包内的包装命令，它们会在转发到 LeRobot 之前先导入本包：

```bash
onero-h1-lerobot-teleoperate --robot.type=onero_h1 --teleop.type=onero_h1_ros_joint ...
onero-h1-lerobot-record --robot.type=onero_h1 --teleop.type=onero_h1_ros_joint ...
```

`onero_h1_ros_joint` 遥操作器读取 ROS2 `JointState` 话题，并输出与 `OneroH1Robot.action_features` 对应的动作。

## 特征名称

默认动作特征顺序：

```text
left_arm.joint1-l.pos ... left_arm.joint7-l.pos
right_arm.joint1-r.pos ... right_arm.joint7-r.pos
lift.pos
head.pitch.pos
head.yaw.pos
```

如果机器人 URDF 或驱动使用不同的关节名称，可以在 `OneroH1Config` 中覆盖它们：

```python
config = OneroH1Config(
    left_arm_joint_names=("left_joint_1", "left_joint_2", "left_joint_3", "left_joint_4", "left_joint_5", "left_joint_6", "left_joint_7"),
    right_arm_joint_names=("right_joint_1", "right_joint_2", "right_joint_3", "right_joint_4", "right_joint_5", "right_joint_6", "right_joint_7"),
)
```

## 安全

适配器会基于 SDK 文档中的保守限制对动作进行裁剪：

- 手臂关节：参照文档中 J1-J7 的范围，每次调用最大增量为 `0.20 rad`
- 升降：`0.40 m` 到 `1.30 m`，每次调用最大增量为 `0.05 m`
- 头部俯仰：`±28°`，偏航：`±90°`，每次调用最大增量为 `0.10 rad`
- 底盘速度动作默认关闭

上述限制不能替代碰撞检测、MoveIt 规划、硬件急停或人工监督。

## 上游合并到 LeRobot

如需直接集成进 LeRobot，本包可以移动到：

```text
src/lerobot/robots/onero_h1/
  __init__.py
  config_onero_h1.py
  onero_h1.py
```

并将 `OneroH1Robot` / `OneroH1Config` 与其他机器人实现一起注册。遥操作器同样可以移动到 `src/lerobot/teleoperators/onero_h1/` 下。运行时代码已经遵循 LeRobot 的 `Robot` 和 `Teleoperator` 接口。
