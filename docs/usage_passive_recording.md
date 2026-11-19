# 数据采集、MuJoCo 与 Rollout 使用说明

本文面向项目使用者，说明 Orin 和普通 GPU 电脑的环境部署方式，以及 H1 遥操数据录制、校验、
回放、MuJoCo 仿真和模型 rollout 的操作流程。

> 录制默认是主动控制的，会向机器人发布控制指令；加 `--passive` 才是被动录制。
> 回放和主动控制（默认）会真实下发动作，只能在清场、急停可用且有人监护时执行。

## 脚本总览

| 脚本或命令 | 作用 | 运行位置 | 是否控制真机 |
|---|---|---|:---:|
| `setup_orin_native.sh` | 一键部署 Orin 原生依赖、虚拟环境、Jetson PyTorch、LeRobot 和本项目 | Orin | 否 |
| `setup.sh` | 部署普通电脑的数采、视频处理和数据集可视化基础环境 | 普通电脑 | 否 |
| `record.sh` | 录制遥操数据集 | Orin/机器人计算机 | 是（默认）；`--passive` 否 |
| `replay.sh` | 将数据集 action 回放到机器人 | Orin/机器人计算机 | 是 |
| `viz.sh` | 离线可视化数据集 | 任意已部署环境 | 否 |
| `start_mujoco_viewer.sh` | 启动带界面的 H1 MuJoCo ROS 仿真器 | 图形工作站 | 仅控制仿真 |
| `deploy_mujoco_smoke.py` | 向 MuJoCo 仿真器发送保守的周期动作 | 开发工作站 | 仅控制仿真 |
| `setup_ubuntu24_native.sh` | 部署工作站训练和完整仿真环境 | Ubuntu 24.04 + RTX 50 系列工作站 | 否 |
| `run_sim_train_rollout.sh` | 自动完成仿真、数采、ACT 短训练和仿真 rollout | NVIDIA GPU 工作站 | 仅控制仿真 |
| `onero-h1-lerobot-rollout` | 加载 checkpoint 执行策略推理 | Orin 或工作站 | 由 `robot.send_action` 决定 |

所有命令均从项目根目录执行。

## 环境部署

部署分为两类设备：

| 设备 | 使用场景 | 执行顺序 |
|---|---|---|
| NVIDIA Jetson Orin | 数采、数据校验和模型 rollout | 只运行 `setup_orin_native.sh` |
| Ubuntu 24.04 + RTX 50 系列普通电脑 | 数采、数据处理、MuJoCo、模型训练和 rollout | 先运行 `setup.sh`；需要 GPU 功能时再运行 `setup_ubuntu24_native.sh` |

三个脚本都使用 `~/.venvs/h1-lerobot`。部署完成后，两类设备使用相同的环境加载方式和操作命令。

### 部署方式一：NVIDIA Jetson Orin

Orin 使用专用脚本，不能运行普通电脑的 CUDA 安装脚本。进入项目根目录后执行：

```bash
bash scripts/setup_orin_native.sh
```

该脚本可以重复执行，默认创建 `~/.venvs/h1-lerobot`，不使用全局 pip，也不默认安装完整
JetPack/TensorRT。它会完成：

1. 检查 Orin、系统、Python 和 ROS2 版本；
2. 安装 Python venv、pip 和 FFmpeg 等最小系统依赖；
3. 创建隔离的 Python 虚拟环境；
4. 安装并验证适用于 Jetson 的 GPU PyTorch；
5. 安装 LeRobot 0.6.1、本项目及数据集依赖；
6. 检查 CUDA、17/16 维 schema，并运行项目测试。

#### 常用安装选项

```bash
# 同时安装数据集可视化依赖
H1_INSTALL_VIZ=1 bash scripts/setup_orin_native.sh

# 临时跳过单元测试
H1_RUN_TESTS=0 bash scripts/setup_orin_native.sh

# 确实需要完整 JetPack runtime 时才启用
H1_INSTALL_JETPACK=1 bash scripts/setup_orin_native.sh
```

一般数采和 PyTorch rollout 不需要设置 `H1_INSTALL_JETPACK=1`。当前兼容的 PyTorch wheel 即使没有
在架构列表中原生显示 `sm_87`，也会由安装脚本执行实际 GPU rollout 算子测试；不要随意开启严格
模式或更换 PyTorch 版本和下载源。虚拟环境位置可通过 `H1_LEROBOT_VENV` 修改。

#### 安装中断后的处理

- 网络中断：恢复网络后重新运行相同安装命令，pip 会重新检查并补齐依赖。
- 脚本提示需要重启：执行 `sudo reboot`，重新登录后再次运行安装脚本。
- 安装失败：查看终端最后的“安装日志”位置，先处理首个 `ERROR`，再重试。
- 已存在虚拟环境：脚本会复用并重新校验，不需要手动删除。

脚本会使用 `sudo apt` 安装少量系统包，但 Python 包仅安装到专用虚拟环境；它不会卸载已有系统包，
也不会使用系统级 pip。

### 部署方式二：普通电脑

普通电脑不要运行 Orin 脚本。环境按功能分两步部署。

#### 第一步：数采基础环境

所有普通电脑先执行：

```bash
bash scripts/setup.sh
```

该脚本在 `~/.venvs/h1-lerobot` 中安装 CPU PyTorch、LeRobot、项目适配器、FFmpeg 和数据集可视化
依赖，可用于遥操数采、数据校验、回放和可视化。如果普通电脑只使用这些功能，到此即可。

#### 第二步：MuJoCo、训练和 Rollout 环境

需要 MuJoCo、ACT 训练或 CUDA rollout 时，在第一步完成后继续执行：

```bash
bash scripts/setup_ubuntu24_native.sh
```

该脚本复用同一个虚拟环境，将 CPU PyTorch 更新为 CUDA PyTorch，并补充 MuJoCo、训练和 GPU
rollout 依赖。第一步已经安装的视频处理和可视化依赖会继续保留。

普通电脑的完整部署顺序如下：

```bash
bash scripts/setup.sh
bash scripts/setup_ubuntu24_native.sh
```

完整仿真和训练流程应在普通 GPU 电脑运行，不要放在只负责真机数采和推理的 Orin 上。

## 使用

### 1. 加载运行环境

```bash
source /opt/ros/jazzy/setup.bash
source ~/.venvs/h1-lerobot/bin/activate
```

机器人通信环境应在使用本适配器前由现场系统完成。加载上述环境后，确认能看到机器人话题：

```bash
ros2 topic list | sort
onero-h1-print-observation --no-cameras --count 5
```

有相机录制前，至少应看到以下话题持续发布：

```text
/left_joint_states       /right_joint_states
/left_gripper_state      /right_gripper_state
/head/camera/rgb         /left/camera/rgb         /right/camera/rgb
```

同构遥操还需确认主臂 action 输入存在：

```bash
ros2 topic hz /left/joint_states
ros2 topic hz /right/joint_states
ros2 topic echo /joystick_info --once
```

### 2. 录制一个短 Episode

先用 20 FPS、60 秒完成一次同构遥操短测：

```bash
bash scripts/record.sh \
  --repo-id validation/h1_homogeneous \
  --task "pick cup" \
  --passive \
  --duration 60 \
  --fps 20 \
  --cameras head,left,right \
  --root "$HOME/lerobot_datasets"
```

录制期间正常操作主臂即可。脚本默认主动控制，本文的被动录制示例都显式加了 `--passive`。
成功结束应出现：

```text
Episode saved and finalized successfully
Done.
```

默认会在 `--repo-id` 后追加时间戳，上例的实际数据集 ID 形如
`validation/h1_homogeneous_20261119_153000`。启动日志和结尾的 `Dataset:` 会打印实际 ID，
后续校验、回放和可视化必须使用它。

- 继续录制已有数据集：`--resume`，`--repo-id` 必须是完整已有 ID；
- 保持原始 ID：`--no-stamp`，目录已存在时会报错，不会覆盖；
- task 名称重复不会冲突，同名 task 会复用已有 task_index。

无相机链路测试可添加 `--no-cameras`：

```bash
bash scripts/record.sh \
  --repo-id validation/h1_state \
  --task "state smoke" \
  --passive \
  --duration 30 \
  --fps 20 \
  --no-cameras
```

### 3. 校验数据集

以下命令沿用上面的示例：

```bash
export HF_LEROBOT_HOME="$HOME/lerobot_datasets"
# 替换为录制结束时打印的实际 ID（默认带时间戳）
export DATASET_REPO_ID="validation/h1_homogeneous_20261119_153000"
export DATASET_DIR="$HF_LEROBOT_HOME/$DATASET_REPO_ID"

find "$DATASET_DIR" -maxdepth 4 -type f | sort
```

用 LeRobot 检查元数据、Episode 数、帧数和特征：

```bash
python - "$DATASET_REPO_ID" <<'PY'
import sys
from lerobot.datasets.lerobot_dataset import LeRobotDataset

dataset = LeRobotDataset(sys.argv[1])
print("episodes:", dataset.num_episodes)
print("frames:", dataset.num_frames)
print("fps:", dataset.fps)
print("features:")
for name, feature in dataset.features.items():
    print(" ", name, feature)
assert dataset.num_episodes >= 1
assert dataset.num_frames > 0
PY
```

三路视频应分别存在。使用 PyAV 完整解码，避免只检查文件头而漏掉损坏帧：

```bash
python - "$DATASET_DIR" <<'PY'
import pathlib
import sys
import av

root = pathlib.Path(sys.argv[1])
videos = sorted(root.glob("videos/**/*.mp4"))
assert videos, f"no videos found under {root}"
for path in videos:
    with av.open(str(path)) as container:
        frames = sum(1 for _ in container.decode(video=0))
    print(f"OK  {frames:5d} frames  {path}")
    assert frames > 0, path
PY
```

最后人工检查三路画面时间顺序、视角、动作与夹爪是否同步。仅看到文件存在不代表数据可用于训练。

## 参数说明

常用录制参数如下，完整列表以 `bash scripts/record.sh --help` 为准。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--repo-id` | 必填 | 数据集 ID 前缀，例如 `team/task`；默认追加时间戳作为实际 ID |
| `--task` | 必填 | 任务描述，写入每帧的 task 元数据；重复名称不会冲突 |
| `--resume` | 关闭 | 继续录制已有数据集；`--repo-id` 必须是完整已有 ID |
| `--no-stamp` | 关闭 | 不追加时间戳；目录已存在时报错，不会覆盖 |
| `--duration` | 无 | 秒数；不传时持续录制，使用停止话题结束 |
| `--fps` | `30` | 数据集目标帧率；三相机建议先以 10～20 FPS 验证 |
| `--cameras` | `head,left,right` | 逗号分隔的相机名 |
| `--no-cameras` | 关闭 | 仅录制状态和 action |
| `--teleop-type` | `homogeneous` | 遥操类型：`homogeneous`（主从同构）或 `vr` |
| `--passive` | 关闭 | 被动录制，不下发控制指令 |
| `--no-gripper` | 关闭 | 不订阅夹爪 action |
| `--no-gripper-action` | 关闭 | 不向 `/joystick_info` 发布夹爪指令 |
| `--root` | `~/lerobot_datasets` | 数据集存储根目录 |

### Action 输入话题

支持主从同构（onero_leader）和 VR，不支持异构：

| 遥操类型 | 左臂 | 右臂 | 夹爪 |
|---|---|---|---|
| `homogeneous`（默认） | `/left/joint_states` | `/right/joint_states` | `/joystick_info`（`Int32`） |
| `vr` | `/left_joint_states` | `/right_joint_states` | `/vr/*/gripper/open_ratio`（`Float32`） |

需要覆盖默认值时使用：

```text
--action-left-arm-topic
--action-right-arm-topic
--action-gripper-topic
--action-left-gripper-topic    # 仅 VR
--action-right-gripper-topic   # 仅 VR
```

## 优雅停止录制

不传 `--duration` 时，在第二个已经加载机器人通信环境的终端发布一次停止信号：

```bash
ros2 topic pub /stop_recording std_msgs/msg/Bool "data: true" -1
```

程序会在当前帧完成后保存并 finalize。第一次 `Ctrl+C` 与停止话题等价：结束当前帧、保存数据集，
然后发送 `21` 让机械臂回到初始姿态；第二次 `Ctrl+C` 才强制退出。定时录制则会在达到
`duration × fps` 帧后自动保存。

## 主臂自动启动与遥操握手

录制/遥操启动时，LeRobot 会自动启动主臂程序 `arm_record_node`：

- **被动录制**（`--passive`）：主臂正常启动，由主臂直接控制从臂；
- **主动控制**（默认）：把 `/record_data` remap 到 `/leader/arm_data`，
  避免与 LeRobot 的控制循环同时控制从臂。

启动后自动发送 `/joystick_info` 的 `20` 进入遥操状态，终端暂停并提示：

```text
请确认机械臂已进入遥操模式，按 Enter 开始数采或遥操...
```

录制结束或退出遥操时发送 `21` 让两臂回到初始姿态，等待 5 秒后退出。自动化场景可加
`--no-confirm` 跳过 Enter 提示。

`/joystick_info` 不会被 remap：它同时承载 20/21 模式握手和原有夹爪控制链路。

> 如果主臂程序已在运行，自动启动会跳过，本次 remap 不会生效；请先停止旧实例。
> VR 遥操（`--teleop-type vr`）直接订阅 VR 话题，不自动启动主臂程序。

## 回放录制的 Episode

回放会真实控制机器人。退出遥操控制程序、清空工作区并确认急停后再执行：

```bash
bash scripts/replay.sh \
  --repo-id "$DATASET_REPO_ID" \
  --episode 0 \
  --root "$HF_LEROBOT_HOME"
```

默认通过 `/record_data` 回放手臂并发送夹爪命令。可选参数：

```bash
# 不发送夹爪
bash scripts/replay.sh --repo-id "$DATASET_REPO_ID" --episode 0 --no-gripper

# 指定较低回放频率
bash scripts/replay.sh --repo-id "$DATASET_REPO_ID" --episode 0 --fps 10
```

首次真机回放建议使用低 FPS、小动作数据，并全程保留人工急停能力。

## 可视化录制的 Episode

可视化是离线操作，不需要连接机器人。

```bash
# Orin 本地桌面打开
bash scripts/viz.sh \
  --repo-id "$DATASET_REPO_ID" \
  --episode 0 \
  --root "$HF_LEROBOT_HOME"

# 保存为 RRD 文件，带到有桌面的电脑查看
bash scripts/viz.sh \
  --repo-id "$DATASET_REPO_ID" \
  --episode 0 \
  --root "$HF_LEROBOT_HOME" \
  --save 1 \
  --output-dir ./output
```

远程流式模式：

```bash
# Orin
bash scripts/viz.sh --repo-id "$DATASET_REPO_ID" --episode 0 \
  --root "$HF_LEROBOT_HOME" --mode distant

# 查看端
rerun --connect rerun+http://<ORIN_IP>:9876/proxy
```

## MuJoCo 仿真验证

MuJoCo 用于在不连接真机的情况下验证 ROS 话题、观测、动作、数采和 rollout 链路。它不是 Orin
数采的必需依赖；模型训练和完整仿真流水线建议放在独立 NVIDIA GPU 工作站上。

### 快速验证仿真控制链路

普通电脑完成 `setup_ubuntu24_native.sh` 部署后，终端 1 启动带界面的仿真器：

```bash
H1_SIM_PYTHON="$HOME/.venvs/h1-lerobot/bin/python" \
  bash scripts/start_mujoco_viewer.sh
```

终端 2 加载项目环境，并发送一段小幅周期动作：

```bash
source /opt/ros/jazzy/setup.bash
source ~/.venvs/h1-lerobot/bin/activate
python scripts/deploy_mujoco_smoke.py \
  --duration 60 \
  --hz 30 \
  --amplitude 0.12
```

验收时应同时确认：仿真窗口中的双臂与夹爪运动连续、终端持续打印关节状态，并最终显示
`deploy smoke complete`。该脚本会向仿真控制话题发送动作，不应在连接真机的 ROS 环境中运行。

### 完整仿真、训练和 Rollout 流水线

普通电脑完成环境部署后，执行端到端验证：

```bash
H1_EPISODES=2 \
H1_EPISODE_SECONDS=4 \
H1_TRAIN_STEPS=10 \
H1_ROLLOUT_SECONDS=5 \
bash scripts/run_sim_train_rollout.sh
```

该脚本依次完成：

1. 启动 H1 MuJoCo ROS 仿真器；
2. 录制合成 LeRobot 数据集；
3. 使用 GPU 短训练 ACT；
4. 加载生成的 checkpoint 在仿真中 rollout。

默认产物位于 `~/onero_h1_runs/<时间戳>/`，其中包括 `dataset/`、`train/`、`logs/` 和
`SUCCESS.txt`。这是一条开发验证流水线，不应在只负责数采和推理的 Orin 上执行训练。

## 模型 Rollout

Rollout 不需要启动遥操程序：策略模型取代 Teleoperator 生成 action。真机运行时必须停止其他会向
`/record_data` 或 `/joystick_info` 发布控制命令的节点，保证每个控制话题只有一个预期发布者。

### 1. 准备兼容的 checkpoint

将训练机产生的整个 `pretrained_model` 目录复制到 Orin，例如：

```bash
scp -r /path/to/pretrained_model \
  nvidia@<ORIN_IP>:~/oneroh1lerobot/checkpoints/my_policy/
```

在 Orin 上设置路径并确认文件完整：

```bash
export H1_CKPT="$HOME/oneroh1lerobot/checkpoints/my_policy"
test -f "$H1_CKPT/config.json"
find "$H1_CKPT" -maxdepth 1 -type f -printf '%f\n' | sort
```

checkpoint 必须与部署配置使用相同特征：当前默认 action 为 16 维（双臂 14 关节位置和双夹爪），
默认 state 为 17 维（action 对应状态加升降），图像模型还必须使用一致的相机名称、数量、分辨率和
预处理。禁止通过截断或补零强行运行 schema 不一致的旧模型。

### 2. 无动作 Rollout

先连接真实观测和三路相机，但禁止创建控制 publisher：

```bash
onero-h1-lerobot-rollout \
  --strategy.type=base \
  --policy.path="$H1_CKPT" \
  --robot.type=onero_h1 \
  --robot.id=h1_dry_rollout \
  --robot.use_cameras=true \
  --robot.camera_names='["head","left","right"]' \
  --robot.send_action=false \
  --device=cuda \
  --fps=20 \
  --duration=10 \
  --task="rollout pipeline smoke" \
  --return_to_initial_position=false
```

通过标准是 checkpoint 和 CUDA 加载成功、连续推理完成并显示 `Rollout finished`，且没有 schema、
NaN、OOM 或 CUDA kernel 错误。`robot.send_action=false` 只能证明推理链路，不能证明机器人动作效果。

### 3. 短时真机 Rollout

只有在 checkpoint schema、无动作 rollout、机器人使能状态和急停均检查通过后，才允许执行。
第一次测试使用低帧率、短时长和较小的逐帧关节限幅：

```bash
ros2 topic info /record_data --verbose
ros2 topic info /joystick_info --verbose

onero-h1-lerobot-rollout \
  --strategy.type=base \
  --policy.path="$H1_CKPT" \
  --robot.type=onero_h1 \
  --robot.id=h1_real_rollout \
  --robot.use_cameras=true \
  --robot.camera_names='["head","left","right"]' \
  --robot.send_action=true \
  --robot.record_data_apply_delta_limit=true \
  --robot.max_arm_delta_rad=0.003 \
  --device=cuda \
  --fps=20 \
  --duration=10 \
  --task="real robot rollout" \
  --return_to_initial_position=false
```

运行期间应由一人监控日志和机器人，另一人负责急停。确认动作方向、幅度和延迟正常后，再逐步增加
`--duration`；不要在首次验证时直接取消关节变化限幅。日志中的实际循环 FPS 低于目标值表示实时性
不足，但若最终正常出现 `Rollout finished`，模型加载与控制流程仍已完整执行。

## 常见问题

### 视频播放起来像快进

数据集按目标 FPS 编码，但录制循环是按 `duration × fps` 收集固定帧数。如果相机解码、写盘或 CPU
负载使实际循环低于目标 FPS，采集的真实时间会超过 `--duration`，播放时便像快进。降低 `--fps`
后重新采集，并保证采集、训练与 rollout 使用一致频率。

### 新数据集出现 `FileExistsError` 或特征不兼容

先核对脚本打印的实际数据集 ID 和路径。已有数据集只有在特征完全一致时才能追加；任务、相机或
schema 改变时应使用新的 `--repo-id`，不要直接删除原数据。自定义存储位置优先使用
`record.sh --root ...`。

### 启动时出现 TorchCodec 或 `libnppicc.so.13` 警告

这是 TorchCodec 与当前 Jetson CUDA/FFmpeg 运行库不匹配。只要录制最终保存成功，LeRobot 可回退到
其他编码路径；仍应使用上面的 PyAV 命令完整解码所有视频确认数据有效。

### 保存成功后出现 `Aborted (core dumped)`

如果已出现 `Episode saved and finalized successfully`，数据通常已经落盘，但进程退出仍不正常。
保留日志，并完成元数据加载、全视频解码和人工查看后才能判定本次数据有效。

## 其他调试工具

```bash
# 只读检查状态链路
onero-h1-print-observation --no-cameras --count 5

# 仅打印安全测试 action，不连接或控制机器人
onero-h1-safe-test-action --head-yaw 0.05 --dry-run
```
