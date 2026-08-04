# lerobot_robot_onero_h1

适用于 Onero H1 / SwitchBot H1 ROS2 SDK 的 LeRobot 适配器。

本包将 H1 的 ROS2 话题封装为标准的 LeRobot `Robot` / `Teleoperator`，用于遥操作数据的**被动采集**——默认只订阅不发布，不参与遥操控制链路。

## 支持范围

### 观测（Observations）

| 类别 | 话题来源 | 特征 |
|------|---------|------|
| 关节位置 `.pos` | `/joint_states`, `/left_joint_states`, `/right_joint_states` | 双臂 14 关节 + 夹爪 + 升降 |
| 关节力矩 `.effort` | `/left_joint_states`, `/right_joint_states` 的 `effort` 字段 | 双臂 14 关节 |
| 帧间差 `.diff` | 本地计算（当前帧 - 上一帧位置差 / dt） | 双臂 14 关节 + 夹爪 + 升降 |
| 夹爪位姿差 `.diff_pos` | `/left_pose`, `/right_pose` → 本地计算 | 左右各 9 维（x/y/z/速度/姿态） |
| 底盘 | `/agv/odom` | base.x, base.y, base.yaw, base.vx, base.vy, base.wz |
| 相机 | `/head/camera/rgb`, `/left/camera/rgb`, `/right/camera/rgb` | CompressedImage → RGB |

### 动作（Actions）— 由 Teleoperator 订阅

| 类别 | 话题来源 | 特征 |
|------|---------|------|
| 关节位置 `.pos` | 可配置（默认 `/left/joint_states`, `/right/joint_states`） | 双臂 14 关节 + 夹爪 + 升降 |
| 关节速度 `.vel` | 同上 | 双臂 14 关节 |
| 帧间差 `.diff` | 本地计算 | 双臂 14 关节 + 夹爪 + 升降 |
| 夹爪 | `/joystick_info`（Int32） | left_gripper.pos, right_gripper.pos |

### 控制指令 — 默认关闭

通过 `send_action=True` 或 `--send-action` 显式开启，支持 `record_data`（默认）和 `movej` 两种模式。

## 安装

### 前置条件

- Ubuntu 24.04 + ROS2 Jazzy
- Python 3.10+
- H1 机器人 SDK 已安装并 source

### 下载并解压

```bash
# 下载压缩包（链接请替换为实际地址）
wget <下载链接> -O oneroh1lerobot.tar.gz
tar -xzf oneroh1lerobot.tar.gz -C ~/
cd ~/oneroh1lerobot
```

### 自动安装（推荐）

```bash
cd /home/denglanjin/vr/oneroh1lerobot
sudo bash scripts/setup.sh
```

### 手动安装

```bash
# 1. 加载 ROS2 环境
source /opt/ros/jazzy/setup.bash

# 2. 安装 LeRobot（GitHub 源）
python3 -m pip install "git+https://github.com/huggingface/lerobot.git" --break-system-packages

# 3. 安装 oneroh1lerobot + OpenCV
cd ~/oneroh1lerobot
python3 -m pip install -e '.[camera]' --break-system-packages
```

### 验证安装

```bash
# 检查包是否可导入
python3 -c "from onero_h1_lerobot import OneroH1Config, OneroH1Robot; print('OK')"

# 不带相机打印观测（需 ROS2 环境和机器人运行中）
onero-h1-print-observation --no-cameras --count 5

# 发送安全测试动作
onero-h1-safe-test-action --head-yaw 0.05 --dry-run
```

## 录制 LeRobot 数据集

详细使用说明见 [docs/usage_passive_recording.md](docs/usage_passive_recording.md)。

### 使用脚本启动（推荐）

```bash
# 定时录制 60 秒
bash scripts/record.sh --repo-id my/test --task "pick cup" --duration 60

# 持续录制，Ctrl+C 手动停止
bash scripts/record.sh --repo-id my/test --task "teleoperation"

# VR 遥操录制（指定 action topic）
bash scripts/record.sh --repo-id my/test --task "VR teleop" \
  --action-left-arm-topic /left_joint_states \
  --action-right-arm-topic /right_joint_states
```

### 直接使用 CLI

```bash
# 定时录制
onero-h1-record-episode \
  --repo-id my_name/h1_dataset \
  --task "task description" \
  --duration 60 --fps 10 \
  --cameras head,left,right \
  --finalize

# 持续录制（不传 --duration）
onero-h1-record-episode \
  --repo-id my_name/h1_dataset \
  --task "task description" \
  --fps 10 \
  --cameras head,left,right \
  --finalize
```

## LeRobot CLI 用法

本包作为 LeRobot 第三方插件安装，安装后原生命令可自动发现。

### 遥操作预览

```bash
lerobot-teleoperate \
  --robot.type=onero_h1 \
  --robot.id=h1 \
  --robot.send_action=false \
  --robot.use_cameras=true \
  --robot.camera_names='["head", "left", "right"]' \
  --teleop.type=onero_h1_ros_joint \
  --teleop.id=exo \
  --teleop.left_arm_topic=/teleop/left_arm/joint_states \
  --teleop.right_arm_topic=/teleop/right_arm/joint_states \
  --display_data=true
```

### 录制数据集

```bash
lerobot-record \
  --robot.type=onero_h1 \
  --robot.id=h1 \
  --robot.send_action=false \
  --robot.use_cameras=true \
  --robot.camera_names='["head", "left", "right"]' \
  --teleop.type=onero_h1_ros_joint \
  --teleop.id=exo \
  --teleop.left_arm_topic=/teleop/left_arm/joint_states \
  --teleop.right_arm_topic=/teleop/right_arm/joint_states \
  --dataset.repo_id=your_name/onero_h1_test \
  --dataset.single_task="Test" \
  --dataset.num_episodes=2 \
  --dataset.episode_time_s=5 \
  --dataset.reset_time_s=5 \
  --dataset.push_to_hub=true \
  --dataset.streaming_encoding=true \
  --dataset.encoder_threads=2
```

如果 LeRobot 版本不支持自动加载第三方插件，使用包装命令：

```bash
onero-h1-lerobot-teleoperate --robot.type=onero_h1 --teleop.type=onero_h1_ros_joint ...
onero-h1-lerobot-record --robot.type=onero_h1 --teleop.type=onero_h1_ros_joint ...
```

## 特征名称

默认动作特征顺序：

```text
left_arm.joint1-l.pos ... left_arm.joint7-l.pos
left_arm.joint1-l.vel ... left_arm.joint7-l.vel
right_arm.joint1-r.pos ... right_arm.joint7-r.pos
right_arm.joint1-r.vel ... right_arm.joint7-r.vel
left_gripper.pos, right_gripper.pos
lift.pos
```

观测特征额外包含 `.effort`、`.diff`、`.diff_pos` 等。

## 安全

适配器会基于 SDK 文档中的保守限制对动作进行裁剪。上述限制不能替代碰撞检测、硬件急停或人工监督。
