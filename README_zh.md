# lerobot_robot_onero_h1

适用于 Onero H1 / SwitchBot H1 ROS2 SDK 的 LeRobot 适配器。

本包将 H1 的 ROS2 话题封装为标准的 LeRobot `Robot` / `Teleoperator`，用于主从同构遥操数据采集。录制默认主动控制；加 `--passive` 为被动采集。

## 脚本总览

| 脚本 | CLI 命令 | 作用 | 需要机器人 |
|------|---------|------|:---:|
| [record.sh](scripts/record.sh) | `onero-h1-record-episode` | 录制遥操作数据 | ✅ |
| [replay.sh](scripts/replay.sh) | `onero-h1-replay-episode` | 回放数据到机器人 | ✅ |
| [viz.sh](scripts/viz.sh) | `lerobot-dataset-viz` | 离线可视化查看数据 | ❌ |

```bash
# 录制（实际 ID 会追加时间戳，以启动日志为准）
bash scripts/record.sh --repo-id my/test --task "pick cup" --duration 60

# 回放：--repo-id 使用实际 ID，如 my/test_20261119_153000
bash scripts/replay.sh --repo-id my/test_20261119_153000 --episode 0

# 可视化（使用同一个实际 ID；需要先安装 viz 依赖）
bash scripts/viz.sh --repo-id my/test_20261119_153000 --episode 0
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
| 关节位置 `.pos` | homogeneous: `/left/joint_states`、`/right/joint_states`；VR: `/left_joint_states`、`/right_joint_states` | 双臂 14 关节；升降可选 |
| 关节速度 `.vel` | 同上（从 JointState.velocity 字段读取或本地估算） | 双臂 14 关节，可选，默认关闭 |
| 帧间差 `.diff` | 本地计算 | 双臂 14 关节 + 夹爪 + 升降 |
| 夹爪 | homogeneous: `/joystick_info`（Int32）；VR: `/vr/*/gripper/open_ratio`（Float32） | left_gripper.pos, right_gripper.pos |

### 控制指令 — 默认开启

录制默认执行主动控制；被动录制需显式加 `--passive`。

支持主从同构（onero_leader）和 VR，不支持异构：
- homogeneous：`/left/joint_states`、`/right/joint_states`、`/joystick_info`（Int32）
- vr：`/left_joint_states`、`/right_joint_states`、VR 夹爪 Float32 话题

发布话题：

| 话题 | 消息类型 | 内容 | 说明 |
|------|----------|------|------|
| `/record_data` | `Float64MultiArray` | 28 floats: 左7位置+左7速度+右7位置+右7速度 | 手臂关节控制 |
| `/joystick_info` | `Int32` | 夹爪命令，左=`pos*100+100`，右=`pos*100+200` | homogeneous 默认发布；`--no-gripper-action` 关闭 |

主动录制时，LeRobot 会自动启动主臂并把 `/record_data` remap 到 `/leader/arm_data`，
无需手动 remap。

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

根据运行设备选择安装流程，不要混用脚本。

#### NVIDIA Jetson Orin

```bash
cd ~/oneroh1lerobot
bash scripts/setup_orin_native.sh
```

#### 普通 Ubuntu 24.04 电脑

先安装数采、视频处理和数据集可视化基础环境：

```bash
bash scripts/setup.sh
```

如果还需要 MuJoCo、模型训练或 CUDA rollout，再执行第二阶段：

```bash
bash scripts/setup_ubuntu24_native.sh
```

两个脚本复用 `~/.venvs/h1-lerobot`，不要使用 `sudo` 启动整个安装脚本，也不要向系统 Python
执行全局 pip 安装。

### 加载环境

每次新开终端后执行：

```bash
source /opt/ros/jazzy/setup.bash
source ~/.venvs/h1-lerobot/bin/activate
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

### 主臂自动启动与遥操握手

- 录制/遥操启动时，LeRobot 会自动启动主臂程序 `arm_record_node`。
- **被动录制**（`--passive`）：主臂正常启动，由主臂直接控制从臂。
- **主动控制**（默认）：remap 主臂的 `/record_data`，避免与 LeRobot 的控制指令冲突。
- 主臂启动后自动发送 `/joystick_info` 的 `20` 进入遥操状态，终端暂停并提示
  “请确认机械臂已进入遥操模式，按 Enter 开始数采或遥操...”。
- 录制结束或退出遥操时发送 `21` 让两臂回到初始姿态，等待 5 秒后退出。
- 自动化/无终端场景可加 `--no-confirm`，或设置
  `ONERO_H1_TELEOP_CONFIRM=0` 跳过 Enter 提示。

### 使用脚本启动（推荐）

```bash
# 定时录制 60 秒
bash scripts/record.sh --repo-id my/test --task "pick cup" --duration 60

# 持续录制，通过 /stop_recording 话题停止（也支持 Ctrl+C 优雅停止）
bash scripts/record.sh --repo-id my/test --task "teleoperation"

# 持续录制，通过 /stop_recording 优雅停止（推荐）
bash scripts/record.sh --repo-id my/test --task "teleoperation"
# 另一终端：ros2 topic pub /stop_recording std_msgs/msg/Bool "data: true" -1

# 被动录制（只记录，不下发控制指令）
bash scripts/record.sh --repo-id my/test --task "passive" --passive

# VR 遥操录制（Float32 夹爪默认话题）
bash scripts/record.sh --repo-id my/test --task "vr teleop" --teleop-type vr
```

> 默认会给 `--repo-id` 追加时间戳（如 `my/test_20261119_153000`），实际 ID 以启动日志和结尾
> `Dataset:` 输出为准；回放、可视化使用该 ID。继续录制已有数据集用 `--resume`，
> 保持原始 ID 用 `--no-stamp`（目录已存在会报错，不会覆盖）。

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

使用录制时打印的实际数据集 ID（默认带时间戳）：

```bash
# 回放 episode 0（默认发送夹爪）
onero-h1-replay-episode --repo-id my/test_20261119_153000 --episode 0

# 回放不发送夹爪
onero-h1-replay-episode --repo-id my/test_20261119_153000 --episode 0 --no-gripper

# 使用 movej 模式回放
onero-h1-replay-episode --repo-id my/test_20261119_153000 --episode 0 --arm-command-mode movej

# 使用脚本
bash scripts/replay.sh --repo-id my/test_20261119_153000 --episode 0
```

回放从数据集中读取每帧的 action，通过 `send_action()` 发送到机器人，使用与录制时相同的手臂指令模式和夹爪设置。

## 可视化录制的 Episode

离线查看数据集中的每一帧数据，用于人工检查数据质量。推荐安装脚本已经把可视化依赖安装到
项目虚拟环境中，无需向系统 Python 单独安装。

**方式一：本地直接使用（推荐）** — 在机器人自带电脑上（有桌面环境）直接运行：

```bash
bash scripts/viz.sh --repo-id my/test_20261119_153000 --episode 0
```

**方式二：SSH 远程连接** — 先在机器人上启动，再从本地电脑连接：

```bash
# 机器人上
bash scripts/viz.sh --repo-id my/test_20261119_153000 --episode 0 --mode distant

# 本地电脑上
pip install rerun-sdk
rerun --connect rerun+http://<机器人IP>:9876/proxy
```

**方式三：保存为文件** — 保存为 rrd 文件，传输到任意有显示器的电脑查看：

```bash
# 机器人上
bash scripts/viz.sh --repo-id my/test_20261119_153000 --episode 0 --save 1 --output-dir ./output

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

> 原生 `lerobot-record` 也支持 Ctrl+C 优雅结束：插件会先保存当前正在录制的 episode，
> 再由 LeRobot 执行 finalize 和 `robot.disconnect()`（发送 `21` 回初始姿态）。

原生命令和包装命令使用同一套默认配置（主臂模式、20/21 握手、主动控制、rollout 发送动作和
Ctrl+C 优雅保存），行为一致。如果 LeRobot 版本不支持自动加载第三方插件，改用包装命令：

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

Ubuntu 24.04 + ROS2 Jazzy + RTX 50 系列需要先安装基础环境，再补充 GPU、MuJoCo 和训练依赖：

```bash
bash scripts/setup.sh
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

主动控制（默认开启，`--passive` 关闭）时，控制指令通过以下话题发布：

| 模块 | 话题 | 消息类型 | 说明 |
|------|------|----------|------|
| 手臂 | `/record_data` | `Float64MultiArray` | 28 维（左7位置+左7速度+右7位置+右7速度） |
| 夹爪 | `/joystick_info` | `Int32` | 左=pos×100+100，右=pos×100+200；`--no-gripper-action` 关闭 |
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
