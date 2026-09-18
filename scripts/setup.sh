#!/usr/bin/env bash
# Isolated base setup for data collection on a standard ROS2 computer.
#
# Usage:
#   bash scripts/setup.sh
#
# Optional override:
#   H1_LEROBOT_VENV=$HOME/.venvs/h1-lerobot

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
VENV="${H1_LEROBOT_VENV:-$HOME/.venvs/h1-lerobot}"
LEROBOT_VERSION="0.6.1"
PIP_NETWORK_OPTS=(--timeout 120 --retries 3)

usage() {
    cat <<'EOF'
用法: bash scripts/setup.sh

为普通 ROS2 电脑创建独立虚拟环境，并安装：
  - CPU PyTorch
  - LeRobot 0.6.1（训练、硬件和数据集可视化依赖）
  - Onero H1 LeRobot 适配器

可选环境变量：
  H1_LEROBOT_VENV   虚拟环境位置（默认 ~/.venvs/h1-lerobot）

NVIDIA Jetson Orin 请改用 scripts/setup_orin_native.sh。
普通电脑如需 MuJoCo、训练或 CUDA rollout，请在本脚本完成后继续运行：
  bash scripts/setup_ubuntu24_native.sh
EOF
}

die() {
    printf 'ERROR: %s\n' "$1" >&2
    exit 1
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
fi

[[ $# -eq 0 ]] || die "不支持参数：$*. 使用 --help 查看说明。"

if [[ "${EUID}" -eq 0 ]]; then
    die "不要使用 sudo 启动整个脚本。请运行：bash scripts/setup.sh"
fi

if [[ -r /etc/nv_tegra_release ]]; then
    die "检测到 NVIDIA Jetson；请使用 bash scripts/setup_orin_native.sh"
fi

printf '============================================\n'
printf '  Onero H1 LeRobot 隔离环境安装\n'
printf '  LeRobot %s\n' "$LEROBOT_VERSION"
printf '============================================\n\n'

printf '[1/5] 检查 ROS2 和 Python\n'

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    ROS_SETUP=/opt/ros/jazzy/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then
    ROS_SETUP=/opt/ros/humble/setup.bash
else
    die "未找到 ROS2 Jazzy 或 Humble。"
fi

python3 - <<'PY' || die "LeRobot 0.6.1 需要 Python >= 3.12。"
import sys
raise SystemExit(0 if sys.version_info >= (3, 12) else 1)
PY

printf '  ROS setup: %s\n' "$ROS_SETUP"
printf '  Python: %s\n' "$(python3 --version 2>&1)"

printf '\n[2/5] 安装最小系统依赖\n'
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
    python3-venv python3-pip \
    ffmpeg libavcodec-dev libavformat-dev libavutil-dev libavdevice-dev

printf '\n[3/5] 创建 Python 虚拟环境\n'
if [[ ! -x "$VENV/bin/python" ]]; then
    python3 -m venv --system-site-packages "$VENV"
else
    printf '  复用已有虚拟环境：%s\n' "$VENV"
fi

# ROS setup scripts may reference optional variables before defining them.
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
set -u
# shellcheck disable=SC1090
source "$VENV/bin/activate"

python -m pip install "${PIP_NETWORK_OPTS[@]}" --upgrade \
    pip 'setuptools>=71,<80' wheel 'packaging>=24.2,<26'

python -c 'import rclpy; print("  rclpy:", rclpy.__file__)' \
    || die "虚拟环境无法导入 ROS2 rclpy。"

printf '\n[4/5] 安装 Python 依赖\n'

# Do not uninstall packages from the user or system Python environment. If this
# venv already contains a compatible torch build, pip keeps it; otherwise the
# standard setup installs the CPU wheel.
python -m pip install "${PIP_NETWORK_OPTS[@]}" \
    --index-url https://download.pytorch.org/whl/cpu \
    'torch>=2.7,<2.12' 'torchvision>=0.22,<0.27'

python -m pip install "${PIP_NETWORK_OPTS[@]}" \
    "lerobot[training,hardware,dataset-viz]==${LEROBOT_VERSION}"

python -m pip install "${PIP_NETWORK_OPTS[@]}" -e "$PROJECT_DIR"

printf '\n[5/5] 验证安装\n'
python - <<'PY'
import importlib.metadata as metadata

import cv2
import rclpy
import torch
from lerobot.datasets.lerobot_dataset import LeRobotDataset

from onero_h1_lerobot import OneroH1Config, OneroH1Robot

robot = OneroH1Robot(OneroH1Config(id="setup-verification", use_cameras=False))
assert metadata.version("lerobot") == "0.6.1"
assert len(robot.action_features) == 16

print("  LeRobot:", metadata.version("lerobot"))
print("  Adapter:", metadata.version("lerobot_robot_onero_h1"))
print("  PyTorch:", torch.__version__)
print("  CUDA available:", torch.cuda.is_available())
print("  OpenCV:", cv2.__version__)
print("  rclpy:", rclpy.__file__)
print("  LeRobotDataset:", LeRobotDataset.__module__)
print("  Action schema: 16 dimensions")
PY

python -c 'import rerun_sdk; print("  Rerun: OK")' \
    || die "Rerun 数据集可视化依赖无法导入。"

printf '\n============================================\n'
printf '  安装完成\n'
printf '============================================\n'
printf '虚拟环境：%s\n' "$VENV"
printf '\n每次使用前执行：\n'
printf '  source %q\n' "$ROS_SETUP"
printf '  source %q/bin/activate\n' "$VENV"
printf '\n只读验证：\n'
printf '  onero-h1-print-observation --no-cameras --count 5\n'
