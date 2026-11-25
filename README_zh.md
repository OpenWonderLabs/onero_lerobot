# Onero H1 LeRobot 适配器

[English](README.md) | 中文

将 Onero H1 / SwitchBot H1 的 ROS 2 接口封装为 LeRobot `Robot` / `Teleoperator`，支持遥操、数据采集、回放、ACT / pi0.5 训练与策略 rollout，并提供 MuJoCo 仿真验证工具。

> **安全：遥操、主动录制、回放和 rollout 会控制机器人。** 清空工作区、保持急停可用，并停止其他控制发布者。安装验收应先完成只读观测，不要直接运行控制示例。

## 从这里开始

| 目标 | 文档 |
|---|---|
| 新机器部署、接线、网络、环境激活和只读验收 | [部署指南](docs/setup.md) |
| 遥操、数采、回放、可视化、训练与推理 | [使用指南](docs/usage.md) |
| 原生 PI0.5 推理、实际权重核验与可选 Kalman 滤波 | [PI0.5 推理指南](docs/native_pi05_inference.md) |
| MuJoCo 模型来源、H1 文件名与上游兼容性 | [模型说明](urdf/README.md) |
| 现场操作参考 | [飞书 SOP](https://ihqz5dyhwa.feishu.cn/wiki/EC81wA9H0iuzOZkbM4bcu6b0nHd)（需权限） |

## 部署入口

推荐基线：**Ubuntu 24.04 + ROS 2 Jazzy + Python 3.12+，LeRobot 固定为 0.6.1**。

- **操作 PC（x86_64），从零配置通信与主臂**：使用 [独立整机安装器](https://github.com/OpenWonderLabs/Onero_Lerobot_setup)，步骤见部署指南。它会配置系统服务和设备规则，不只是安装 Python 包。
- **已有 SDK/通信环境，源码开发**：基础数采运行 `bash scripts/setup.sh`；GPU 工作站再运行 `bash scripts/setup_ubuntu24_native.sh`。
- **Jetson Orin**：使用 `bash scripts/setup_orin_native.sh`，不要混用工作站 CUDA 脚本。

整机部署后使用 `source ~/h1_env.sh`；本仓库 setup 脚本不生成该文件，需手动激活 ROS、通信环境和 venv，见 [setup](docs/setup.md)。

激活正确环境后先做只读检查：

```bash
onero-h1-print-observation --no-cameras --count 3
onero-h1-print-observation --count 3
```

## 固定 16 维数据约定

策略 state / action 顺序统一为：

```text
left_joint_1..7.pos, left_gripper.pos, right_joint_1..7.pos, right_gripper.pos
```

即左臂 7 关节、左夹爪、右臂 7 关节、右夹爪。不含升降、头部、底盘、速度等额外标量通道；不提供可选 native/interleaved 布局。
为兼容已有 SOP 命令，接受显式参数 `policy_feature_layout=interleaved`；其他布局值会被拒绝。
相机默认 `head`、`left`、`right`，作为单独图像特征。底层 ROS 硬件命名由适配器转换。

训练、数据集与 checkpoint 必须匹配维度、顺序、相机和预处理，禁止通过截断/补零强行使用不兼容权重。

## 命令入口

推荐从原生 `lerobot-teleoperate`、`lerobot-record`、`lerobot-replay`、`lerobot-dataset-viz`、`lerobot-train`、`lerobot-rollout` 开始，完整示例见 [使用指南](docs/usage.md)。

自研工具用于数采和调试：

| 命令 / 脚本 | 用途 |
|---|---|
| `onero-h1-print-observation` | 只读观测，不自动启动主臂 |
| `onero-h1-pi05-rollout` | 原生 PI05 推理，严格特征/反馈检查、实际加载权重核验与可选输出 Kalman 滤波 |
| `onero-h1-record-episode` / `scripts/record.sh` | 独立控制线程与录制频率；支持主动/被动录制 |
| `onero-h1-replay-episode` / `scripts/replay.sh` | 真机回放，自动遥操握手，与原生回放存在差异 |
| `scripts/viz.sh` | 离线数据集可视化 |
| `scripts/run_sim_train_rollout.sh` | MuJoCo → 录制 → ACT 短训练 → 仿真 rollout |

注意：`send_action=false` 仅关闭适配器动作下发，不保证其他主臂程序或启动握手无副作用。各命令的控制权、退出保存和回放差异见使用指南。

## 仿真与开发

默认仿真模型为 `urdf/urdf/H1_mjcf_with_grippers.urdf`，依赖 `urdf/meshes/`。真机 ROS 适配器不直接加载 URDF。运行仓库级仿真需保留完整源码，Python 包安装不能替代模型资源。

代码以 Gitee 的 `master` 为基准，GitHub 用于同步发布；独立整机安装器默认从 GitHub 安装适配器。开发安装前确认代码版本、安装来源和对应虚拟环境。
