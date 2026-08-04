# 遥操数据录制 — 使用说明

---

## 设计思路

`oneroh1lerobot` 定位为**遥操作数据的被动采集工具**，核心原则：

1. **action 话题通过启动脚本传入**，observation 和相机话题从 H1 从机订阅，话题固定不变
2. **默认不参与遥操**，只订阅不发布。通过 `--send-action` 显式开启控制输出
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
| `homogeneous`（默认） | `/teleop/left_arm/joint_states` | `/teleop/right_arm/joint_states` | Int32 |
| `heterogeneous` | `/teleop/left/joint_states` | `/teleop/right/joint_states` | Int32 |
| `vr` | `/left_joint_states` | `/right_joint_states` | Float32 |

不传 `--duration` 则持续录制，`Ctrl+C` 手动停止后自动保存。

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
| `--duration` | float | 无 | 录制时长（秒）。不传则持续录制，Ctrl+C 停止 |
| `--fps` | int | `30` | 录制帧率 |
| `--cameras` | str | `head,left,right` | 相机列表，逗号分隔 |
| `--no-cameras` | flag | - | 禁用相机 |
| `--root` | str | `~/lerobot_datasets` | 数据集本地存储根目录。数据集以 `<repo_id>_<task>` 为子目录名保存 |
| `--id` | str | `onero_h1` | 机器人 ID |
| `--send-hold-action` | flag | - | 向机器人发布 hold-action（默认不发布） |
| `--finalize` | flag | - | 录制完成后锁定数据集 |

### Action 话题（通过 `--teleop-type` 或手动指定）

| `--teleop-type` | action 左臂 topic | action 右臂 topic | 夹爪 topic | 夹爪格式 |
|:---:|---|---|---|:---:|
| `homogeneous`（默认） | `/left_joint_states` | `/right_joint_states` | `/joystick_info` | int32 |
| `heterogeneous` | `/teleop/left/joint_states` | `/teleop/right/joint_states` | `/joystick_info` | int32 |
| `vr` | `/left_joint_states` | `/right_joint_states` | 左: `/vr/left_gripper/open_ratio`<br>右: `/vr/right_gripper/open_ratio` | float32 |

可通过 `--action-left-arm-topic` / `--action-right-arm-topic` / `--action-gripper-topic`（int32 模式）/ `--action-left-gripper-topic` / `--action-right-gripper-topic`（float32 模式）手动覆盖默认值。

### 夹爪话题

| 参数 | 默认值 | 类型 | 说明 |
|------|--------|------|------|
| `--gripper-type` | int32 | str | 夹爪消息格式：`int32`（主从遥操）或 `float32`（VR 遥操）。脚本根据 `--teleop-type` 自动设置 |

**Observation 侧（H1 从机固定，无需修改）：**

| 话题 | 默认值 | 消息类型 | 说明 |
|------|--------|----------|------|
| 左夹爪状态 | `/left_gripper_state` | `UInt8` (0-255) | 映射到 0.0-1.0 |
| 右夹爪状态 | `/right_gripper_state` | `UInt8` (0-255) | 映射到 0.0-1.0 |
| 左夹爪位姿 | `/left_pose` | `PoseWithCovarianceStamped` | 用于计算 diff_pos |
| 右夹爪位姿 | `/right_pose` | `PoseWithCovarianceStamped` | 用于计算 diff_pos |

---

## 常见问题

**Q: 录制时 lerobot 会向机器人发指令吗？**

不会。`send_action=False`（默认）时 lerobot 不创建任何 ROS publisher，不发布任何消息。只有显式加 `--send-action` 才会发布。

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