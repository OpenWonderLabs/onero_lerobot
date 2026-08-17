# 遥操数据录制 — 使用说明

---

## 脚本总览

| 脚本 | CLI 命令 | 作用 | 需要机器人 |
|------|---------|------|:---:|
| [record.sh](file:///home/denglanjin/vr/oneroh1lerobot/scripts/record.sh) | `onero-h1-record-episode` | 录制遥操作数据 | ✅ |
| [replay.sh](file:///home/denglanjin/vr/oneroh1lerobot/scripts/replay.sh) | `onero-h1-replay-episode` | 回放数据到机器人 | ✅ |
| [viz.sh](file:///home/denglanjin/vr/oneroh1lerobot/scripts/viz.sh) | `lerobot-dataset-viz` | 离线可视化查看数据 | ❌ |

```bash
# 录制
bash scripts/record.sh --repo-id my/test --task "pick cup" --duration 60

# 回放
bash scripts/replay.sh --repo-id my/test --episode 0

# 可视化
bash scripts/viz.sh --repo-id my/test --episode 0
```

> 三个脚本的详细用法见下方对应章节。

---

## 设计思路

`oneroh1lerobot` 定位为**遥操作数据的被动采集工具**，核心原则：

1. **action 话题通过启动脚本传入**，observation 和相机话题从 H1 从机订阅，话题固定不变
2. **默认不参与遥操**，只订阅不发布。通过 `--send-hold-action` 显式开启控制输出
3. **输出统一的 lerobot 标准关节级格式**，录制内容与 `onero-local-backend` 对齐（含 effort、diff、diff_pos、gripper）

整体架构：

```
┌──────────────┐         ┌──────────────┐
│ Teleoperator │         │    Robot     │
│ 订阅 action   │         │ 订阅 obs      │
│ topic (可配)  │         │ + camera     │
│ /joystick_info│         │ (H1从机固定)  │
└──────┬───────┘         └──────┬───────┘
       │                        │
       │  get_action()          │  get_observation()
       │  .pos + .vel + .diff   │  .pos + .effort + .diff + .diff_pos
       ▼                        ▼
┌──────────────────────────────────────────┐
│  录制循环 (固定频率，fps 控制)             │
│  send_action=False → 不发布任何控制指令   │
│  send_action=True  → 发布手臂/夹爪/升降等  │
└──────────────────────────────────────────┘
```

录制触发方式：**固定频率循环**（`duration × fps` 帧，`time.sleep` 凑帧率）。

---

## 安装

### 前置条件

- Ubuntu 24.04 + ROS2 Jazzy
- Python 3.10+
- H1 机器人 SDK 已安装并 source

### 下载并解压

```bash
wget <下载链接> -O oneroh1lerobot.tar.gz
tar -xzf oneroh1lerobot.tar.gz -C ~/
cd ~/oneroh1lerobot
```

### 一键安装

```bash
sudo bash scripts/setup.sh
```

### 验证

```bash
python3 -c "from onero_h1_lerobot import OneroH1Config, OneroH1Robot; print('OK')"
```

---

## 使用

### 使用脚本（推荐）

```bash
# 主从同构遥操，定时 60 秒
bash scripts/record.sh --repo-id my/test --task "pick cup" --teleop-type homogeneous --duration 60

# 主从异构遥操，持续录制
bash scripts/record.sh --repo-id my/test --task "exo teleop" --teleop-type heterogeneous

# VR 遥操，持续录制
bash scripts/record.sh --repo-id my/test --task "VR teleop" --teleop-type vr

# 自定义 action topic
bash scripts/record.sh --repo-id my/test --task "custom" --teleop-type homogeneous \
  --action-left-arm-topic /custom/left --action-right-arm-topic /custom/right
```

`--teleop-type` 会根据类型自动设置默认值：

| `--teleop-type` | action 左臂 topic | action 右臂 topic | 夹爪格式 |
|:---:|---|---|:---:|
| `homogeneous`（默认） | `/left/joint_states` | `/right/joint_states` | Int32 |
| `heterogeneous` | `/teleop/left/joint_states` | `/teleop/right/joint_states` | Int32 |
| `vr` | `/left_joint_states` | `/right_joint_states` | Float32 |

不传 `--duration` 则持续录制，通过 `--stop-topic` 话题优雅停止后自动保存。

### 直接使用 CLI

```bash
onero-h1-record-episode \
  --repo-id my_name/h1_dataset \
  --task "task description" \
  --duration 60 --fps 10 \
  --action-left-arm-topic <action 左臂 topic> \
  --action-right-arm-topic <action 右臂 topic> \
  --cameras head,left,right \
  --finalize
```

---

## 参数说明

### 录制控制

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--repo-id` | str | 必填 | 数据集 ID，格式 `用户名/数据集名`，如 `my_name/h1_test` |
| `--task` | str | 必填 | 任务描述，存入每帧数据中 |
| `--teleop-type` | str | `homogeneous` | 遥操方式：`homogeneous` / `heterogeneous` / `vr`。脚本自动设置对应的 action topic 和夹爪格式 |
| `--duration` | float | 无 | 录制时长（秒）。不传则持续录制，通过 `--stop-topic` 话题停止 |
| `--fps` | int | `30` | 录制帧率 |
| `--cameras` | str | `head,left,right` | 相机列表，逗号分隔 |
| `--no-cameras` | flag | - | 禁用相机 |
| `--root` | str | `~/lerobot_datasets` | 数据集本地存储根目录。数据集以 `<repo_id>_<task>` 为子目录名保存 |
| `--id` | str | `onero_h1` | 机器人 ID |
| `--send-hold-action` | flag | - | 向机器人发布控制指令。开启后将创建所有 ROS publisher 并下发控制命令（详见下方"控制指令"章节） |
| `--finalize` | flag | - | 录制完成后锁定数据集 |
| `--stop-topic` | str | `/stop_recording` | 柔和停止录制的 ROS2 Bool 话题（发布 `true` 在当前帧完成后退出，避免截断图片） |

### Action 输入话题（Teleoperator 订阅）

| `--teleop-type` | action 左臂 topic | action 右臂 topic | 夹爪 topic | 夹爪格式 |
|:---:|---|---|---|:---:|
| `homogeneous`（默认） | `/left/joint_states` | `/right/joint_states` | `/joystick_info` | int32 |
| `heterogeneous` | `/teleop/left/joint_states` | `/teleop/right/joint_states` | `/joystick_info` | int32 |
| `vr` | `/left_joint_states` | `/right_joint_states` | 左: `/vr/left_gripper/open_ratio`<br>右: `/vr/right_gripper/open_ratio` | float32 |

可通过 `--action-left-arm-topic` / `--action-right-arm-topic` / `--action-gripper-topic`（int32 模式）/ `--action-left-gripper-topic` / `--action-right-gripper-topic`（float32 模式）手动覆盖默认值。

---

## 控制指令（`--send-hold-action`）

> **注意：** 不同遥操类型的控制指令行为有所区别，请仔细阅读对应章节。

### 同构遥操（homogeneous）— 已支持

同构遥操模式下，主臂程序会直接发布 `/record_data`（Float64MultiArray）和 `/joystick_info`（Int32）控制从臂。启用 `--send-hold-action` 后，需要将主臂的这两个话题 remap 掉，改由 oneroh1lerobot 接管控制。

**发布话题：**

| 话题 | 消息类型 | 内容 | 说明 |
|------|----------|------|------|
| `/record_data` | `Float64MultiArray` | 28 floats: 左7位置+左7速度+右7位置+右7速度 | 手臂关节控制 |
| `/joystick_info` | ❌ 不发布 | — | 同构模式不发布夹爪命令，避免与主臂夹爪链路冲突 |

**完整启动步骤：**

```bash
# 1. 主臂程序 remap 掉直接控制从臂的话题
your_leader_program \
    --ros-args -r /record_data:=/leader/arm_data \
               -r /joystick_info:=/leader/joystick

# 2. 启动录制，指定夹爪 action 来源为 remap 后的新话题
onero-h1-record-episode \
    --repo-id my/test --task "teleop_test" \
    --duration 180 --teleop-type homogeneous \
    --action-gripper-topic /leader/joystick \
    --send-hold-action
```

**数据流：**

```
主臂
  ├─→ /left/joint_states──→ teleoperator（arm action）
  ├─→ /right/joint_states ──→ teleoperator（arm action）
  └─→ /leader/joystick    ──→ teleoperator（夹爪，--action-gripper-topic 指定）
                                │
                       record + send_action
                                │
                         /record_data
                                │
                           ──→ 从臂
```

**注意事项：**
- 主臂的 `/record_data` 必须 remap 掉（如 `/leader/arm_data`），否则 oneroh1lerobot 输出到 `/record_data` 会与主臂冲突
- 主臂的 `/joystick_info` 必须 remap 掉（如 `/leader/joystick`），否则两个节点同时发布到同一话题
- teleoperator 夹爪输入需通过 `--action-gripper-topic` 指向 remap 后的新话题
- 同构模式只录制和发布手臂关节控制，**夹爪控制由主臂自身直连从臂完成**

### 异构遥操（heterogeneous）— 待定

行为待确认。

### VR 遥操（vr）— 待定

行为待确认。

---

## 回放录制的 Episode

录制完成后，可将数据集回放到机器人上，验证数据完整性和动作质量。

### 使用脚本（推荐）

```bash
# 回放 episode 0（默认发送夹爪）
bash scripts/replay.sh --repo-id my/test --episode 0

# 回放不发送夹爪
bash scripts/replay.sh --repo-id my/test --episode 0 --no-gripper

# 使用 movej 模式回放
bash scripts/replay.sh --repo-id my/test --episode 0 --arm-command-mode movej
```

### 直接使用 CLI

```bash
onero-h1-replay-episode --repo-id my/test --episode 0
```

### 回放参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--repo-id` | str | 必填 | 数据集 ID |
| `--episode` | int | `0` | 回放的 episode 序号 |
| `--fps` | int | 数据集原始帧率 | 回放帧率 |
| `--root` | str | `~/lerobot_datasets` | 数据集本地存储根目录 |
| `--id` | str | `onero_h1` | 机器人 ID |
| `--arm-command-mode` | str | `record_data` | 手臂指令模式：`record_data` / `movej` |
| `--send-gripper` | flag | 开启 | 回放时发送夹爪指令（默认开启）。`--no-gripper` 关闭 |
| `--no-cameras` | flag | - | 禁用相机 |
| `--cameras` | str | `head,left,right` | 相机列表 |

### 回放原理

1. 加载 LeRobot 数据集，读取指定 episode 的 action 数据
2. 连接机器人，逐帧将 action 通过 `send_action()` 发送到 `/record_data`（手臂）和 `/joystick_info`（夹爪）
3. 按数据集原始帧率（或指定 `--fps`）控制回放速度
4. 回放完成后自动断开连接

**注意：** 回放使用的是与录制时完全相同的 `send_action()` 链路，包括安全裁剪、速度限制等。回放前请确保机器人处于安全状态。

---

## 可视化录制的 Episode

离线查看数据集内容，无需连接机器人，用于人工检查数据质量。

### 安装依赖

```bash
pip install 'lerobot[dataset_viz]' --break-system-packages
```

### 方式一：本地直接使用（推荐）

在机器人自带的电脑上（有桌面环境）直接运行，Rerun 会弹出 GUI 窗口：

```bash
bash scripts/viz.sh --repo-id my/test --episode 0
```

### 方式二：SSH 远程连接

通过 SSH 从另一台电脑连接查看。先在机器人上启动 distant 模式：

```bash
bash scripts/viz.sh --repo-id my/test --episode 0 --mode distant
```

然后在本地电脑上安装 rerun 并连接（替换 `10.8.69.50` 为机器人实际 IP）：

```bash
pip install rerun-sdk
rerun --connect rerun+http://10.8.69.50:9876/proxy
```

### 方式三：保存为文件

保存为 rrd 文件，传输到任意有显示器的电脑上离线查看：

```bash
# 机器人上保存
bash scripts/viz.sh --repo-id my/test --episode 0 --save 1 --output-dir ./output

# 传到本地电脑
scp -r wlab@10.8.69.50:~/oneroh1lerobot/output ./output

# 本地电脑上查看
pip install rerun-sdk
rerun output/*.rrd
```

### 可视化参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--repo-id` | str | 必填 | 数据集 ID |
| `--episode` | int | `0` | 可视化的 episode 序号 |
| `--root` | str | `~/lerobot_datasets` | 数据集本地存储根目录 |
| `--mode` | str | `local` | `local`（本地直接打开）\| `distant`（远程流式）\| `foxglove` |
| `--save` | 0/1 | `0` | `1`=保存为 rrd 文件，`0`=直接显示 |
| `--output-dir` | str | - | 保存 rrd 文件的输出目录 |
| `--grpc-port` | int | `9876` | distant 模式的 GRPC 端口 |

### 可查看的数据

- 相机图像（head / left / right）
- 关节位置（observation.state 和 action）
- 夹爪状态
- 帧间差分数据

> **注意：** 可视化是纯离线操作，不需要 ROS2 环境或机器人连接。`local` 和 `distant` 模式需要桌面环境（GUI），SSH 远程连接请使用 `distant` 或保存文件方式。

---

## 夹爪话题

### 观测侧（H1 从机固定，无需修改）

| 话题 | 默认值 | 消息类型 | 说明 |
|------|--------|----------|------|
| 左夹爪状态 | `/left_gripper_state` | `UInt8` (0-255) | 映射到 0.0-1.0 |
| 右夹爪状态 | `/right_gripper_state` | `UInt8` (0-255) | 映射到 0.0-1.0 |
| 左夹爪位姿 | `/left_pose` | `PoseWithCovarianceStamped` | 用于计算 diff_pos（x/y/z=原始位置，pitch/roll/yaw=速度） |
| 右夹爪位姿 | `/right_pose` | `PoseWithCovarianceStamped` | 用于计算 diff_pos（x/y/z=原始位置，pitch/roll/yaw=速度） |

### 夹爪命令编码（`/joystick_info`，Int32）

| 编码范围 | 含义 |
|----------|------|
| 100-199 | 左手夹爪：`pos = (value - 100) / 100`，范围 [0, 0.99] |
| 200-299 | 右手夹爪：`pos = (value - 200) / 100`，范围 [0, 0.99] |

---

## 优雅停止录制

持续录制（不传 `--duration`）时，**必须**通过 ROS2 话题停止：

```bash
# 终端1：启动录制
onero-h1-record-episode --repo-id my/test --task "teleop" --teleop-type homogeneous

# 终端2：停止录制
ros2 topic pub /stop_recording std_msgs/msg/Bool "data: true" -1
```

**原理：** 每帧循环开始时检查 `/stop_recording`，收到 `true` 后当前帧完整执行完毕才退出，不会打断 `add_frame()` 或相机写入，从根源上避免了截断图片和 buffer 不对齐的问题。

| 停止方式 | 会截断数据吗 | 推荐 |
|---------|:---:|:---:|
| `--stop-topic` 发布 `true` | ❌ 不会 | ✅ 推荐 |
| `--duration` 定时结束 | ❌ 不会 | ✅ 推荐 |

> **注意：** `--stop-topic` 依赖 ROS2 环境，录制前确保 `rclpy` 已初始化（项目中 `robot.connect()` 时会自动初始化）。Ctrl+C 已禁用，不可用于停止录制。

---

## 常见问题

**Q: 录制时 lerobot 会向机器人发指令吗？**

不会。`send_action=False`（默认）时 lerobot 不创建任何 ROS publisher，不发布任何消息。只有显式加 `--send-hold-action` 才会发布。

**Q: `--send-hold-action` 开启后发布哪些话题？**

取决于遥操类型：
- `homogeneous`：只发布 `/record_data`（手臂关节），不发布夹爪
- `heterogeneous`：待定
- `vr`：待定

**Q: 同构模式下使用 `--send-hold-action` 需要做什么额外配置？**

需要将主臂程序的 `/record_data` 和 `/joystick_info` 两个话题 remap 掉，并通过 `--action-gripper-topic` 指定 teleoperator 订阅 remap 后的夹爪话题。详见[同构遥操](#同构遥操homogeneous-已支持)章节。

**Q: 如何优雅停止持续录制而不截断数据？**

```bash
# 在另一个终端发布停止信号
ros2 topic pub /stop_recording std_msgs/msg/Bool "data: true" -1
```

详见[优雅停止录制](#优雅停止录制)章节。

**Q: action 和 observation 的 topic 可以相同吗？**

可以。两者可以来自同一个发布端，甚至同一个 topic，完全合法。

**Q: 主从同构和主从异构在 lerobot 中有区别吗？**

没有。lerobot 只关心配置了哪些 topic，不关心主端是什么设备。

**Q: 录制时遥操作会卡顿吗？**

不会。lerobot 只是额外订阅 ROS topic，不参与控制链路，仅增加网络带宽消耗。

**Q: 如何验证录制数据？**

```bash
python -c "
from lerobot.datasets.lerobot_dataset import LeRobotDataset
ds = LeRobotDataset('my_name/h1_vr_recording')
print(ds[0])
print(ds.features)
"
```

**Q: 如何回放录制的数据集验证数据质量？**

```bash
onero-h1-replay-episode --repo-id my/test --episode 0
```

回放会逐帧将 action 发送到机器人，复现录制时的动作轨迹。详见[回放录制的 Episode](#回放录制的-episode)章节。

**Q: 如何离线查看数据集而不用连接机器人？**

```bash
pip install 'lerobot[dataset_viz]' --break-system-packages
bash scripts/viz.sh --repo-id my/test --episode 0
```

这会打开 rerun 可视化界面，逐帧显示相机图像和关节数据。详见[可视化录制的 Episode](#可视化录制的-episode)章节。

---

## 其他调试工具

| 命令 | 作用 | 示例 |
|------|------|------|
| `onero-h1-print-observation` | 打印当前观测数据 | `onero-h1-print-observation --no-cameras --count 5` |
| `onero-h1-safe-test-action` | 发送安全测试动作 | `onero-h1-safe-test-action --head-yaw 0.05 --dry-run` |