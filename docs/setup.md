# H1 部署指南

目标：完成环境安装、通信检查和只读观测。遥操、录制、回放、训练与推理见 [使用指南](usage.md)。

## 1. 选择部署路径

| 场景 | 路径 | 边界 |
|---|---|---|
| Ubuntu 24.04 / x86_64 操作 PC，尚未配置主臂和通信 | 独立整机安装器，见第 3 节 | 配置 ROS、Zenoh、USB-CAN、主臂运行时和 Python 环境 |
| 已有机器人 SDK/通信环境的源码开发机 | 本仓库 setup 脚本，见第 4 节 | 主要安装 Python 环境，不替代整机部署 |
| NVIDIA Jetson Orin | 本仓库 `setup_orin_native.sh` | 使用 Jetson 专用 PyTorch，不混用工作站 CUDA 脚本 |

推荐基线为 Ubuntu 24.04、ROS 2 Jazzy、Python 3.12+、LeRobot 0.6.1。
部分脚本识别 Humble/其他架构，不代表整机安装、预编译主臂和 Python 依赖已在所有组合验证。
整机安装器无可用 GPU 时选择 CPU；本仓库 CUDA 第二阶段则要求 CUDA 可用。

> 安装器会使用 sudo 修改系统包、服务、udev 和 shell 环境。不要用 sudo 启动整个脚本；先审阅脚本，再在目标机器执行。
> 本文命令是操作说明，不表示已经在当前机器执行部署。

## 2. 接线与网络

1. PC 与机器人网线直连，准备左右主臂 USB-CAN 线。
2. 现场 SOP 使用 Ubuntu「设置 → 网络 → 有线 → IPv4 → 与其他计算机共享」。该操作会改变网卡配置，远程操作时注意断连风险。
3. 默认机器人地址为 `10.42.0.2`，Zenoh 端口为 `7447`。不同现场网络应先确认地址，不要盲目覆盖现有配置。
4. 按安装器提示先拔掉左右 USB-CAN，再分别插入识别左右臂。生成的规则按 USB 接口拓扑匹配，换插口后可能需要重新配置。

主臂程序、握手和控制动作可能使机械臂运动。清空工作区，保持急停可用；安装验收不要用回放或 rollout 代替只读测试。

## 3. 操作 PC：一键整机部署

独立安装仓库：[Onero_Lerobot_setup](https://github.com/OpenWonderLabs/Onero_Lerobot_setup)。
以下命令将下载并执行远端脚本；`main` 会变化，生产部署建议审阅并固定已验证的版本。

```bash
curl -fsSL https://raw.githubusercontent.com/OpenWonderLabs/Onero_Lerobot_setup/main/install.sh | bash
```

按终端提示完成。安装器源码包含：

- ROS 安装、系统依赖和 Zenoh Router 级联部署；
- Python venv、PyTorch、LeRobot、适配器和 MuJoCo；
- 左右 USB-CAN udev 规则；
- `~/robot-h1/install` 主臂运行时及 `~/.local/bin/arm_record_node` 入口；
- `~/h1_env.sh` 环境脚本与完成后的 `OK / WARN / FAIL` 自检。

默认适配器从 GitHub `OpenWonderLabs/onero_lerobot@master` 安装；开发代码以 Gitee 为基准，同步发布到 GitHub 后才会被此路径获取。
需访问 GitHub、Python/ROS 软件源；pi0.5 还可能访问 Hugging Face。

安装完成后新开终端：

```bash
source ~/h1_env.sh
export HF_LEROBOT_HOME="${HF_LEROBOT_HOME:-$HOME/lerobot_datasets}"
```

该路径无需再机械重复运行本仓库的两个工作站 setup 脚本。已有部署机更新适配器时先确认安装来源和虚拟环境，避免为了更新 Python 包重新覆盖 udev 或主臂运行时。

## 4. 已有通信环境：源码安装

先准备 ROS 2、机器人 SDK/通信及主臂运行时；以下脚本不负责完整配置这些组件。
选择一个长期保留的目录克隆源码，editable 安装后不要移动或删除源码目录：

```bash
git clone https://gitee.com/switchbot/oneroh1lerobot.git
cd oneroh1lerobot
git switch master
```

### 普通 PC：基础数采

```bash
bash scripts/setup.sh
```

安装 CPU PyTorch、LeRobot 0.6.1、适配器和视频/可视化依赖。
只做遥操、采集和离线查看时可到此结束。

### Ubuntu 24.04 + NVIDIA GPU：追加训练和仿真

基础环境完成后：

```bash
bash scripts/setup_ubuntu24_native.sh
```

此脚本安装固定版本 CUDA PyTorch 和 MuJoCo，并实际执行 GPU 检查；它不是无 GPU 的通用安装入口，也不会自动安装 ROS。
pi0.5 还需补齐对应依赖：

```bash
source /opt/ros/jazzy/setup.bash
source ~/.venvs/h1-lerobot/bin/activate
python -m pip install 'lerobot[pi]==0.6.1'
```

### Jetson Orin

```bash
bash scripts/setup_orin_native.sh
# 可选：需要桌面数据集可视化
# H1_INSTALL_VIZ=1 bash scripts/setup_orin_native.sh
```

不要运行工作站 CUDA 脚本。JetPack、驱动和 wheel 组合受设备环境约束，以脚本检查与 GPU 算子验证为准；不默认安装完整 JetPack runtime。
安装日志默认位于 `~/.cache/onero_h1_setup/`。

### 每个新终端激活

本仓库脚本默认使用 `~/.venvs/h1-lerobot`，可用 `H1_LEROBOT_VENV` 覆盖，但不生成 `~/h1_env.sh`。

```bash
source /opt/ros/jazzy/setup.bash
# 使用现场已配置的通信环境；Zenoh 部署生成该文件时才加载
if [ -f /etc/zenoh/ros.env ]; then
  set -a
  source /etc/zenoh/ros.env
  set +a
fi
source ~/.venvs/h1-lerobot/bin/activate
export HF_LEROBOT_HOME="${HF_LEROBOT_HOME:-$HOME/lerobot_datasets}"
```

自定义 venv 时替换 activate 路径。仅激活 ROS/venv 不代表已连通机器人。

## 5. 分层验收（不下发动作）

### Python 与 schema

```bash
python - <<'PY'
from importlib.metadata import version
from onero_h1_lerobot import OneroH1Config, OneroH1Robot
print('LeRobot:', version('lerobot'))
print('Adapter:', version('lerobot_robot_onero_h1'))
robot = OneroH1Robot(OneroH1Config(use_cameras=False, auto_start_arm_node=False))
assert len(robot.action_features) == 16
print('IMPORT_OK; 16-D action schema OK')
PY
```

这里不调用 connect，也不启动主臂。固定 state/action 顺序为左 7、左夹爪、右 7、右夹爪；不包含升降等额外标量通道。

### 通信与设备

Zenoh 整机部署路径可检查（其他现场拓扑不必具备同名服务）：

```bash
systemctl status zenoh-router --no-pager
ss -lnt | grep ':7447'
printf 'RMW=%s\n' "${RMW_IMPLEMENTATION:-unset}"
ls -l /dev/ttyMasterCANL /dev/ttyMasterCANR
ls -l ~/.local/bin/arm_record_node
ros2 topic list | sort
```

本机端口监听不代表机器人 `10.42.0.2:7447` 已可达；结合安装器连通性自检及真实话题数据判断。
不要为了验证入口存在而直接运行 `arm_record_node`。

### 只读观测

```bash
onero-h1-print-observation --no-cameras --count 3
onero-h1-print-observation --count 3
```

该 CLI 显式关闭主臂自动启动。检查双臂关节值是否更新、三路图像 shape 是否符合预期；能列出话题但没有数据不算验收通过。
GPU 训练/推理机器另检查：

```bash
python -c 'import torch; print(torch.__version__, torch.version.cuda, torch.cuda.is_available())'
```

CPU 数采时 CUDA 为 False 可以接受；GPU 训练/rollout 时不可接受。全部通过后再转到 [使用指南](usage.md)。

## 6. 目录与模型

| 路径 | 用途 |
|---|---|
| `~/.venvs/h1-lerobot` | 默认 Python 环境 |
| `~/h1_env.sh` | 独立整机安装器生成的环境入口 |
| `~/lerobot_datasets` | 默认数据集根目录，可用 `HF_LEROBOT_HOME` 覆盖 |
| `~/robot-h1/install` | 整机安装器部署的主臂运行时 |
| `~/.local/bin/arm_record_node` | 主臂程序入口 |
| `/etc/udev/rules.d/usbcan.rules` | 左右 USB-CAN 规则 |
| `urdf/urdf/H1_mjcf_with_grippers.urdf` | 源码目录中的默认 MuJoCo 模型 |

URDF/STL 用于源码仿真，不是真机 ROS 适配器的直接运行依赖。仅安装 Python 包不保证包含仓库级仿真脚本与模型；仿真请保留完整源码。模型来源及兼容标识见 [模型说明](../urdf/README.md)。

## 7. 常见问题

- `h1_env.sh` 不存在：确认是否使用整机安装器；源码路径按第 4 节手动激活。
- `import rclpy` 失败：检查 ROS 是否加载、venv 的 Python 与 ROS 是否匹配，以及是否启用 system-site-packages。不要用全局 pip 覆盖系统 ROS。
- 看不到机器人数据：先检查直连网络和机器人地址，再查 Zenoh/RMW 环境；不要通过反复发控制命令排障。
- USB-CAN 端口缺失/左右错误：检查接线和 USB 端口，重新按左右顺序配置规则；不要仅按 ttyACM 数字猜测。
- 无 CUDA：基础数采可以继续；GPU 任务先解决驱动/wheel/设备兼容性，不混装 Jetson 与 x86 wheel。
- pi0.5 缺 transformers：在目标虚拟环境安装 `lerobot[pi]==0.6.1`，并确认预训练资源可下载或已缓存。

## 参考与维护边界

- [现场 SOP](https://ihqz5dyhwa.feishu.cn/wiki/EC81wA9H0iuzOZkbM4bcu6b0nHd)（需相应权限）。
- 本指南结合 SOP 与安装器源码整理；脚本静态审阅不等于在全新机器完成实测。
- 外部安装器跟随其独立版本更新；本仓库仅维护适配器、源码环境与使用说明。
