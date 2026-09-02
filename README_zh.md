# lerobot_robot_onero_h1

适用于 Onero H1 / SwitchBot H1 ROS2 SDK 的 LeRobot 适配器。

本包将 H1 的 ROS2 话题封装为标准的 LeRobot `Robot` / `Teleoperator`，用于遥操作数据的**被动采集**——默认只订阅不发布，不参与遥操控制链路。

## 脚本总览

| 脚本 | CLI 命令 | 作用 | 需要机器人 |
|------|---------|------|:---:|
| [record.sh](scripts/record.sh) | `onero-h1-record-episode` | 录制遥操作数据 | ✅ |
| [replay.sh](scripts/replay.sh) | `onero-h1-replay-episode` | 回放数据到机器人 | ✅ |
| [viz.sh](scripts/viz.sh) | `lerobot-dataset-viz` | 离线可视化查看数据 | ❌ |

```bash
# 录制
bash scripts/record.sh --repo-id my/test --task "pick cup" --duration 60

# 回放
bash scripts/replay.sh --repo-id my/test --episode 0

# 可视化（需要先安装 viz 依赖）
bash scripts/viz.sh --repo-id my/test --episode 0
```

> 详细使用说明见 [docs/usage_passive_recording.md](docs/usage_passive_recording.md)。

## 支持范围

### 观测（Observations）

| 类别 | 话题来源 | 特征 |
|------|---------|------|
| 关节位置 `.pos` | `/joint_states`, `/left_joint_states`, `/right_joint_states` | 双臂 14 关节 + 夹爪 + 升降 |
| 关节力矩 `.effort` | `/left_joint_states`, `/right_joint_states` 的 `effort` 字段 | 双臂 14 关节 |
| 帧间差 `.diff` | 本地计算（当前帧 - 上一帧位置差 / dt） | 双臂 14 关节 + 夹爪 + 升降 |
| 夹爪位姿差 `.diff_pos` | `/left_pose`, `/right_pose` → 本地计算 | 左右各 9 维（x/y/z=原始位置，pitch/roll/yaw=速度） |
| 底盘 | `/agv/odom` | base.x, base.y, base.yaw, base.vx, base.vy, base.wz |
| 相机 | `/head/camera/rgb`, `/left/camera/rgb`, `/right/camera/rgb` | CompressedImage → RGB |

### 动作（Actions）— 由 Teleoperator 订阅

| 类别 | 话题来源 | 特征 |
|------|---------|------|
| 关节位置 `.pos` | 可配置，默认见 `--teleop-type`（homogeneous=`/left/joint_states` 等） | 双臂 14 关节；升降可选 |
| 关节速度 `.vel` | 同上（从 JointState.velocity 字段读取或本地估算） | 双臂 14 关节，可选，默认关闭 |
| 帧间差 `.diff` | 本地计算 | 双臂 14 关节 + 夹爪 + 升降 |
| 夹爪 | `/joystick_info`（Int32）或 VR 独立 Float32 话题 | left_gripper.pos, right_gripper.pos |

### 控制指令 — 默认关闭

通过 `--send-hold-action` 显式开启，支持 `record_data`（默认）和 `movej` 两种模式。

> **注意：** 不同遥操类型的控制指令行为有所区别。
> - `homogeneous`：已支持，详见下方说明
> - `heterogeneous`：待定
> - `vr`：待定

**同构遥操（homogeneous）发布话题：**

| 话题 | 消息类型 | 内容 | 说明 |
|------|----------|------|------|
| `/record_data` | `Float64MultiArray` | 28 floats: 左7位置+左7速度+右7位置+右7速度 | 手臂关节控制 |
| `/joystick_info` | ❌ 不发布 | — | 同构模式不发布夹爪命令 |

同构模式下启用 `--send-hold-action` 时，需要将主臂程序的 `/record_data` 和 `/joystick_info` 话题 remap 掉，避免与 oneroh1lerobot 的输出冲突：

```bash
# 主臂端 remap
your_leader_program \
    --ros-args -r /record_data:=/leader/arm_data \
               -r /joystick_info:=/leader/joystick

# 录制端指定 remap 后的夹爪话题
onero-h1-record-episode \
    --repo-id my/test --task "teleop_test" \
    --duration 180 --teleop-type homogeneous \
    --action-gripper-topic /leader/joystick \
    --send-hold-action
```

## 安装

### 前置条件

- Ubuntu 24.04 + ROS2 Jazzy
- Python 3.12+（LeRobot 0.6.1 要求）
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
cd ~/oneroh1lerobot
sudo bash scripts/setup.sh
```

### 手动安装

```bash
# 1. 加载 ROS2 环境
source /opt/ros/jazzy/setup.bash

# 2. 安装固定的 LeRobot 稳定版本（训练、硬件、数据集可视化依赖）
python3 -m pip install \
  'lerobot[training,hardware,dataset-viz]==0.6.1' \
  --break-system-packages

# 3. 安装 oneroh1lerobot
cd ~/oneroh1lerobot
python3 -m pip install -e . --break-system-packages
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

# 持续录制，通过 /stop_recording 话题停止（Ctrl+C 已禁用）
bash scripts/record.sh --repo-id my/test --task "teleoperation"

# 持续录制，通过 /stop_recording 优雅停止（推荐）
bash scripts/record.sh --repo-id my/test --task "teleoperation"
# 另一终端：ros2 topic pub /stop_recording std_msgs/msg/Bool "data: true" -1

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

## 回放录制的 Episode

录制完成后，可以将数据集回放到机器人上，用于验证数据质量：

```bash
# 回放 episode 0（默认发送夹爪）
onero-h1-replay-episode --repo-id my/test --episode 0

# 回放不发送夹爪
onero-h1-replay-episode --repo-id my/test --episode 0 --no-gripper

# 使用 movej 模式回放
onero-h1-replay-episode --repo-id my/test --episode 0 --arm-command-mode movej

# 使用脚本
bash scripts/replay.sh --repo-id my/test --episode 0
```

回放从数据集中读取每帧的 action，通过 `send_action()` 发送到机器人，使用与录制时相同的手臂指令模式和夹爪设置。

## 可视化录制的 Episode

离线查看数据集中的每一帧数据，用于人工检查数据质量：

```bash
# 使用项目固定版本：pip install 'lerobot[dataset-viz]==0.6.1' --break-system-packages
```

**方式一：本地直接使用（推荐）** — 在机器人自带电脑上（有桌面环境）直接运行：

```bash
bash scripts/viz.sh --repo-id my/test --episode 0
```

**方式二：SSH 远程连接** — 先在机器人上启动，再从本地电脑连接：

```bash
# 机器人上
bash scripts/viz.sh --repo-id my/test --episode 0 --mode distant

# 本地电脑上
pip install rerun-sdk
rerun --connect rerun+http://<机器人IP>:9876/proxy
```

**方式三：保存为文件** — 保存为 rrd 文件，传输到任意有显示器的电脑查看：

```bash
# 机器人上
bash scripts/viz.sh --repo-id my/test --episode 0 --save 1 --output-dir ./output

# 传到本地电脑
scp -r wlab@<机器人IP>:~/oneroh1lerobot/output ./output
pip install rerun-sdk
rerun output/*.rrd
```

这会打开 rerun 可视化界面，可以逐帧查看相机图像、关节位置和动作数据。

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
onero-h1-lerobot-rollout --strategy.type=base --policy.path=/path/to/checkpoint --robot.type=onero_h1 ...
```

## 特征名称

默认 canonical policy action 固定为 16 维：

```text
left_arm.joint1-l.pos ... left_arm.joint7-l.pos
right_arm.joint1-r.pos ... right_arm.joint7-r.pos
left_gripper.pos, right_gripper.pos
```

Robot 与同构 Teleoperator 使用完全相同的顺序。速度动作和升降动作必须分别通过
`use_arm_velocity_action=True`、`use_lift_action=True` 显式开启；第一版训练数据不要开启。
默认 policy state 为 17 维（双臂位置 14 + 夹爪 2 + 升降 1），与 LeRobot rollout 的硬件特征筛选一致。`.effort`、`.diff`、`.diff_pos` 和底盘观测均可显式开启，但开启后训练与部署必须使用相同配置。

### 策略部署

请按照 PyTorch 官方文档或目标硬件厂商文档安装适合当前平台的 PyTorch，再安装本项目固定的 LeRobot 依赖。本仓库不捆绑特定平台的 CUDA wheel，也不修改 LeRobot 源码。

Ubuntu 24.04 + ROS2 Jazzy + RTX 50 系列可使用原生安装脚本（需要先安装 ROS2 Jazzy）：

```bash
bash scripts/setup_ubuntu24_native.sh
```

完整验证 MuJoCo 仿真录制、ACT GPU 训练和 checkpoint rollout：

```bash
bash scripts/run_sim_train_rollout.sh
```

产物默认写入 `~/onero_h1_runs/<timestamp>/`，可通过 `H1_VALIDATION_ROOT`、`H1_TRAIN_STEPS`、`H1_FPS` 等环境变量覆盖。

加载本机 ROS2 环境后，通过 LeRobot 原生 rollout 路径运行策略：

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash

onero-h1-lerobot-rollout \
  --strategy.type=base \
  --policy.path=/path/to/checkpoint \
  --robot.type=onero_h1 \
  --robot.id=h1 \
  --device=<cpu-or-cuda>
```

适配器在同一 Python 环境中直接调用 `rclpy`。训练和部署必须使用一致的 observation 与 action schema。

当 `--send-hold-action` 开启时，控制指令通过以下话题发布（homogeneous 模式不发布夹爪）：

| 模块 | 话题 | 消息类型 | 说明 |
|------|------|----------|------|
| 手臂 | `/record_data` | `Float64MultiArray` | 28 维（左7位置+左7速度+右7位置+右7速度） |
| 夹爪 | `/joystick_info` | `Int32` | 编码：左=pos×100+100，右=pos×100+200（仅 heterogeneous/vr） |
| 升降 | `/lift/joint_states/update` | `JointState` | 如启用 |
| 头部 | `/head/joint_states/update` | `JointState` | 如启用 |
| 底盘 | `/cmd_vel` | `Twist` | 如启用 |

## 安全

适配器会基于 SDK 文档中的保守限制对动作进行裁剪。上述限制不能替代碰撞检测、硬件急停或人工监督。

## 其他工具

| 命令 | 作用 |
|------|------|
| `onero-h1-print-observation` | 打印当前观测数据，调试用 |
| `onero-h1-safe-test-action` | 发送安全测试动作（头部运动），可 `--dry-run` 干跑 |
