# H1 使用指南：遥操、数据采集、训练与 Rollout

本文说明 H1 遥操、数据录制、校验、回放、训练、MuJoCo 仿真和策略 rollout；环境部署统一见 [setup](setup.md)。

> 录制默认主动控制。自研录制器用 `--passive`，原生录制用 `--robot.send_action=false`；被动采集时主臂仍可能直接驱动从臂。
> 回放和主动控制（默认）会真实下发动作，只能在清场、急停可用且有人监护时执行。

## 脚本总览

| 脚本或命令 | 作用 | 运行位置 | 是否控制真机 |
|---|---|---|:---:|
| `setup_orin_native.sh` | 一键部署 Orin 原生依赖、虚拟环境、Jetson PyTorch、LeRobot 和本项目 | Orin | 否 |
| `setup.sh` | 部署普通电脑的数采、视频处理和数据集可视化基础环境 | 普通电脑 | 否 |
| `record.sh` | 录制遥操数据集 | Orin/机器人计算机 | 是（默认）；被动模式不由适配器下发，但主臂仍可控制 |
| `replay.sh` | 将数据集 action 回放到机器人 | Orin/机器人计算机 | 是 |
| `viz.sh` | 离线可视化数据集 | 任意已部署环境 | 否 |
| `start_mujoco_viewer.sh` | 启动带界面的 H1 MuJoCo ROS 仿真器 | 图形工作站 | 仅控制仿真 |
| `deploy_mujoco_smoke.py` | 向 MuJoCo 仿真器发送保守的周期动作 | 开发工作站 | 仅控制仿真 |
| `setup_ubuntu24_native.sh` | 部署工作站训练和完整仿真环境 | Ubuntu 24.04 + RTX 50 系列工作站 | 否 |
| `run_sim_train_rollout.sh` | 自动完成仿真、数采、ACT 短训练和仿真 rollout | NVIDIA GPU 工作站 | 仅控制仿真 |
| `onero-h1-lerobot-rollout` | 加载 checkpoint 执行策略推理 | Orin 或工作站 | 动作下发由 `send_action` 控制；另有启动/握手副作用 |
| `onero-h1-pi05-rollout` | 原生 PI05 推理、实际权重核验与可选 Kalman 滤波 | Orin 或工作站 | `send_action=true` 时；禁用自动主臂启动和遥操握手 |

所有命令均从项目根目录执行。

## 环境部署

先完成 [部署指南](setup.md)。整机安装器与本仓库的 Python 环境脚本职责不同，请按指南选择。

## 安全与数据约定

遥操、主动录制、回放和真机 rollout 会控制机器人。清空工作区，保持急停可用；停止其他控制发布者。
退出遥操时机械臂保持当前姿态，启动新任务可能发生姿态调整。

策略 state/action 固定为 16 维 interleaved：

```text
left_joint_1..7.pos, left_gripper.pos, right_joint_1..7.pos, right_gripper.pos
```

不支持通过 `policy_feature_layout=native` 切换；不含升降、头部、底盘或速度通道。
为兼容已有 SOP，机器人和遥操配置接受显式 `policy_feature_layout=interleaved`，不提供其他布局选择。
相机是单独的图像特征。

### 底层接口说明

模型只需输出固定 16 维动作；适配器会转换为机器人控制消息，无需按 ROS 消息长度配置模型。
默认手臂控制接口 `/record_data` 包含左臂 7 个位置、7 个速度、右臂 7 个位置、7 个速度，共 28 项；夹爪命令另走 `/joystick_info`。
这些是底层通信格式，不应据此修改数据集或模型的 16 维动作约定。

## 原生 LeRobot CLI（推荐入口）

先按 [setup](setup.md) 激活环境。以下示例会真实控制机器人，不属于安装验收。

### 遥操

```bash
lerobot-teleoperate \
  --robot.type=onero_h1 --robot.id=h1 \
  --robot.use_cameras=true \
  --robot.camera_names='["head","left","right"]' \
  --teleop.type=onero_h1_ros_joint --teleop.id=exo \
  --display_data=false
```

### 录制

```bash
lerobot-record \
  --robot.type=onero_h1 --robot.id=h1 \
  --robot.use_cameras=true \
  --teleop.type=onero_h1_ros_joint --teleop.id=exo \
  --dataset.repo_id=my/test \
  --dataset.single_task="pick cup" \
  --dataset.num_episodes=1 --dataset.episode_time_s=30 \
  --dataset.reset_time_s=10 --dataset.fps=30 \
  --dataset.push_to_hub=false
```

默认主动录制；显式加 `--robot.send_action=false` 切为被动采集，此时主臂仍可能直接带动从臂。
原生命令的数据集 ID/目录以运行日志为准；不要假定与自研录制器有相同的时间戳行为。续录使用完整已有 ID 和 `--resume=true`。
原生录制快捷键受 LeRobot 0.6.1 与终端环境影响；保存/编码时不要用 Ctrl+C 打断。
适配器会在 Ctrl+C 时尝试保存未完成 episode，但帧写入中断或保存阶段的再次中断仍可能造成丢失，不能保证恢复。
自研录制器的停止方式见后文，不应与原生快捷键混用。

### 原生回放与离线可视化

```bash
DATASET_REPO_ID=my/test_20261119_153000  # 替换成实际 ID
lerobot-dataset-viz --repo-id "$DATASET_REPO_ID" \
  --root "$HF_LEROBOT_HOME/$DATASET_REPO_ID" --episode-index 0 --mode local --save 0
```

`lerobot-replay` 不自动完成遥操握手。清空工作区并确认控制权后，用手柄 start
或现场批准的握手流程进入遥操，再执行（会控制真机）：

```bash
lerobot-replay --robot.type=onero_h1 --robot.id=h1 \
  --dataset.repo_id="$DATASET_REPO_ID" \
  --dataset.root="$HF_LEROBOT_HOME/$DATASET_REPO_ID" \
  --dataset.episode=0 --robot.send_action=true
```

自研 `onero-h1-replay-episode` 自动进入遥操，具体差异见后文。

## 训练与权重兼容

```bash
DATASET_REPO_ID=my/test_20261119_153000  # 替换成实际 ID
lerobot-train \
  --dataset.repo_id="$DATASET_REPO_ID" \
  --dataset.root="$HF_LEROBOT_HOME/$DATASET_REPO_ID" \
  --policy.type=act --policy.device=cuda --policy.push_to_hub=false \
  --steps=100000 --save_freq=20000 --batch_size=8 \
  --output_dir=outputs/train/act_h1
```

仅验证流程时可将 steps/save_freq 同时降低；短训练通过不代表策略可安全用于真机。
输出目录使用新目录；续训需按 LeRobot 的 resume 配置恢复，不能与新微调混淆。
用 `--policy.path=<pretrained_model目录>` 初始化新训练不等于恢复优化器和训练进度。

pi0.5 使用 `--policy.type=pi05`，需要 `[pi]` 依赖和预训练资源，首次运行可能下载 Hugging Face 文件。
飞书 SOP 的单卡 32 GB 配置建议冻结 VLM：
`--policy.train_expert_only=true --policy.dtype=bfloat16 --policy.gradient_checkpointing=true --batch_size=1`。
这是部署经验而非所有硬件的显存保证；仍需短运行核实显存及版本参数。ACT 不使用 `policy.dtype`。

已有权重必须匹配 16 维特征顺序、相机名/分辨率、预处理、归一化统计和控制频率。
相机名不一致时核对 checkpoint 后再使用 `--rename_map`，不要截断或补零绕过不兼容。
pi0.5 rollout 的 `--task` 使用训练数据集的实际任务文本；ACT 通常不依赖该文本。
完整 rollout 与安全分阶段验证见后文。

## 自研命令与脚本

原生命令是通用入口；自研录制器将控制线程与写盘频率分离，适合数采调试。
以下保留详细参数、回放、可视化、仿真及故障排查。

## 使用

### 1. 加载运行环境

整机部署使用 `source ~/h1_env.sh`；源码安装需手动加载 ROS、通信和虚拟环境，见 [setup](setup.md)。

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
export HF_LEROBOT_HOME="${HF_LEROBOT_HOME:-$HOME/lerobot_datasets}"
# 替换为录制结束时打印的实际 ID（默认带时间戳）
export DATASET_REPO_ID="validation/h1_homogeneous_20261119_153000"
export DATASET_DIR="$HF_LEROBOT_HOME/$DATASET_REPO_ID"

find "$DATASET_DIR" -maxdepth 4 -type f | sort
```

用 LeRobot 检查元数据、Episode 数、帧数和特征：

```bash
python - "$DATASET_REPO_ID" "$DATASET_DIR" <<'PY'
import sys
from lerobot.datasets.lerobot_dataset import LeRobotDataset

dataset = LeRobotDataset(sys.argv[1], root=sys.argv[2])
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
| `--no-gripper-action` | 关闭 | 不发布夹爪指令，仍保留 16 维特征 |
| `--root` | `~/lerobot_datasets`（Shell 脚本） | 数据集存储根目录，脚本会覆盖 `HF_LEROBOT_HOME` |

`record.sh --root` 是存储根目录；直接调用 `onero-h1-record-episode --root` 也是根目录。
原生 `lerobot-* --dataset.root` 和直接调用 `onero-h1-replay-episode --root` 则是完整数据集目录。
Shell 的 record/replay/viz 脚本默认使用 `~/lerobot_datasets`，自定义位置请显式传 `--root`。
固定 16 维 schema 要求保留夹爪特征；当前不要使用录制器的 `--no-gripper`，该遗留参数会导致特征校验失败。

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

采集循环中的第一次 `Ctrl+C` 与停止话题等价：结束采集并保存、finalize；循环内再次中断会强制退出。
进入保存阶段后 Ctrl+C 已恢复默认行为，不要再按，否则可能中断写盘。退出不发送 `21`、不自动回位。
定时录制按 `max(1, int(duration × fps))` 帧结束；若实际采集速度不足，真实耗时可能更长。

## 主臂自动启动与遥操握手

录制/遥操启动时，LeRobot 会自动启动主臂程序 `arm_record_node`：

- **被动录制**（`--passive`）：主臂正常启动，由主臂直接控制从臂；
- **主动控制**（默认）：把 `/record_data` remap 到 `/leader/arm_data`，
  避免与 LeRobot 的控制循环同时控制从臂。

启动后发送 `/joystick_info` 的 `20` 进入遥操，按配置等待短暂稳定时间后自动开始，无需 Enter 确认。
退出时保持当前姿态，不发送 `21`、不等待回位；当前没有 `--no-confirm` 参数。
`/joystick_info` 不会被 remap，它同时承载遥操握手和夹爪命令。

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

首次真机回放选择已核验的小动作数据，并全程保留人工急停能力。降低 FPS 只会放慢发送，
不会减少相邻帧的目标差，也不构成限幅或安全保证；正常运行应与录制/训练频率保持一致。
自研回放不启动主臂，会自动发 `20` 进入遥操，退出保持当前姿态。

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

默认使用 `urdf/urdf/H1_mjcf_with_grippers.urdf`，模型来源见 [模型说明](../urdf/README.md)。
仿真发布的话题与真机同名。请使用与真机隔离的 ROS/Zenoh 通信环境，不要在连通真机控制网络的终端直接运行仿真或验证流水线。

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

原生 PI0.5 部署可使用新增的 `onero-h1-pi05-rollout`，流程见
[PI0.5 推理指南](native_pi05_inference.md)。该入口复用官方 LeRobot 0.6.1，
在策略启动前核验实际加载权重，并严格检查 16 维契约、相机几何和反馈新鲜度；
自动关闭主臂节点启动与遥操握手，输出 Kalman 需显式开启。
指南提供独立的真实观测无动作检查；原有 rollout 命令仍保留原生加载行为。

Rollout 不需要启动遥操程序：策略模型取代 Teleoperator 生成 action。真机运行时必须停止其他会向
`/record_data` 或 `/joystick_info` 发布控制命令的节点，保证每个控制话题只有一个预期发布者。

### 1. 准备兼容的 checkpoint

将训练机产生的整个 `pretrained_model` 目录复制到 Orin，例如：

```bash
ssh nvidia@<ORIN_IP> 'mkdir -p ~/models/my_policy'
scp -r /path/to/pretrained_model \
  nvidia@<ORIN_IP>:~/models/my_policy/
```

在 Orin 上设置路径并确认文件完整：

```bash
export H1_CKPT="$HOME/models/my_policy/pretrained_model"
test -f "$H1_CKPT/config.json"
find "$H1_CKPT" -maxdepth 1 -type f -printf '%f\n' | sort
```

checkpoint 必须与部署配置使用相同特征：当前默认 action 为 16 维（双臂 14 关节位置和双夹爪），
默认 state 为 16 维（左臂 7 关节、左夹爪、右臂 7 关节、右夹爪，不含升降），图像模型还必须使用一致的相机名称、数量、分辨率和
预处理。禁止通过截断或补零强行运行 schema 不一致的旧模型。

### 2. 无动作 Rollout

先连接真实观测和三路相机，关闭动作下发、主臂自动启动和遥操握手。仅设置 `send_action=false` 不代表启动过程完全无控制副作用：

```bash
onero-h1-lerobot-rollout \
  --strategy.type=base \
  --policy.path="$H1_CKPT" \
  --robot.type=onero_h1 \
  --robot.id=h1_dry_rollout \
  --robot.auto_start_arm_node=false \
  --robot.auto_enter_teleop=false \
  --robot.use_cameras=true \
  --robot.camera_names='["head","left","right"]' \
  --robot.send_action=false \
  --device=cuda \
  --fps=20 \
  --duration=10 \
  --task="<训练数据集的实际 single_task>" \
  --return_to_initial_position=false
```

通过标准是 checkpoint 和 CUDA 加载成功、连续推理完成并显示 `Rollout finished`，且没有 schema、
NaN、OOM 或 CUDA kernel 错误。`robot.send_action=false` 只能证明推理链路，不能证明机器人动作效果。

### 3. 短时真机 Rollout

只有在 checkpoint schema、无动作 rollout、机器人使能状态和急停均检查通过后，才允许执行。
第一次测试使用短时长和较小的逐帧关节限幅；示例 20 Hz 仅适用于对应训练频率为 20 Hz 的模型，
否则应替换为训练/部署配置的实际频率。限幅不替代工作区安全与急停：

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
  --task="<训练数据集的实际 single_task>" \
  --return_to_initial_position=false
```

运行期间应由一人监控日志和机器人，另一人负责急停。确认动作方向、幅度和延迟正常后，再逐步增加
`--duration`；不要在首次验证时直接取消关节变化限幅。日志中的实际循环 FPS 低于目标值表示实时性
不足，不能仅凭 `Rollout finished` 判定实时控制效果合格；需同时核对动作、延迟和异常日志。

## 常见问题

### 视频播放起来像快进

数据集按目标 FPS 编码，但录制循环是按 `duration × fps` 收集固定帧数。如果相机解码、写盘或 CPU
负载使实际循环低于目标 FPS，采集的真实时间会超过 `--duration`，播放时便像快进。降低 `--fps`
后重新采集，并保证采集、训练与 rollout 使用一致频率。

### 新数据集出现 `FileExistsError` 或特征不兼容

先核对脚本打印的实际数据集 ID 和路径。已有数据集只有在特征完全一致时才能追加；任务、相机或
schema 改变时应使用新的 `--repo-id`，不要直接删除原数据。自定义存储位置优先使用
`record.sh --root ...`。

`onero-h1-safe-test-action` 是遗留头部/升降测试入口，与当前固定 16 维配置不兼容，
不要作为安装或动作验收命令使用（包括 `--dry-run`）。

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

# 查看当前固定 schema（不连接机器人）
python - <<'PY'
from onero_h1_lerobot import OneroH1Config, OneroH1Robot
robot = OneroH1Robot(OneroH1Config(use_cameras=False, auto_start_arm_node=False))
print(robot.action_features)
PY
```

## 参考

[现场部署与使用 SOP](https://ihqz5dyhwa.feishu.cn/wiki/EC81wA9H0iuzOZkbM4bcu6b0nHd)（可能需要飞书权限）。
本文命令针对本仓库与 LeRobot 0.6.1；现场部署步骤见 setup，平台限制以对应脚本为准。
