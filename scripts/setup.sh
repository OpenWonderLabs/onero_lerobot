#!/bin/bash
# oneroh1lerobot 环境搭建脚本
# 用法: sudo bash scripts/setup.sh
#   sudo 仅用于 apt-get 安装系统依赖，pip 安装以原用户身份执行

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

if [[ "${1:-}" == "-h" ]] || [[ "${1:-}" == "--help" ]]; then
    echo "用法: sudo bash scripts/setup.sh"
    echo ""
    echo "一键安装 oneroh1lerobot 及其所有依赖（LeRobot + OpenCV）"
    exit 0
fi

# 检测是否以 sudo 运行，pip 需要以原用户身份执行
if [ -n "${SUDO_USER:-}" ]; then
    REAL_USER="$SUDO_USER"
    REAL_HOME=$(eval echo "~$REAL_USER")
    PIP_CMD="sudo -u $REAL_USER python3 -m pip"
else
    REAL_USER="$(whoami)"
    REAL_HOME="$HOME"
    PIP_CMD="python3 -m pip"
fi

echo "============================================"
echo "  oneroh1lerobot 环境搭建"
echo "============================================"
echo ""

# --- 1. 检查 ROS2 环境 ---
echo "[1/3] 检查 ROS2 环境..."

if [ -z "${ROS_DISTRO:-}" ]; then
    if [ -f /opt/ros/jazzy/setup.bash ]; then
        echo "  加载 ROS2 Jazzy..."
        source /opt/ros/jazzy/setup.bash
    elif [ -f /opt/ros/humble/setup.bash ]; then
        echo "  加载 ROS2 Humble..."
        source /opt/ros/humble/setup.bash
    else
        echo "  [错误] 未找到 ROS2 安装，请先安装 ROS2 Jazzy 或 Humble"
        exit 1
    fi
fi

echo "  ROS_DISTRO=${ROS_DISTRO:-unknown}"
echo "  ROS_VERSION=${ROS_VERSION:-unknown}"

# --- 2. 检查 Python ---
echo ""
echo "[2/3] 检查 Python..."

PYTHON_VERSION=$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")' 2>/dev/null || echo "unknown")
echo "  Python ${PYTHON_VERSION}"

# --- 3. 安装系统依赖 ---
echo ""
echo "[3/4] 安装系统依赖..."

# 修复 ubuntu-ports 误配问题（amd64 机器配了 ARM 镜像源会导致 404）
ARCH=$(dpkg --print-architecture 2>/dev/null || echo "unknown")
if [ "$ARCH" = "amd64" ]; then
    for f in /etc/apt/sources.list /etc/apt/sources.list.d/*.sources /etc/apt/sources.list.d/*.list; do
        if [ -f "$f" ] && grep -q "ubuntu-ports" "$f" 2>/dev/null; then
            echo "  检测到 amd64 机器使用了 ubuntu-ports 源，自动修复: $f"
            sed -i 's|ubuntu-ports|ubuntu|g' "$f"
        fi
    done
fi

echo "  安装 FFmpeg..."
apt-get update -qq
apt-get install -y -qq ffmpeg libavcodec-dev libavformat-dev libavutil-dev libavdevice-dev
echo "  FFmpeg 安装完成（torchcodec 硬解码）"

# --- 4. 安装 Python 依赖（以原用户身份） ---
echo ""
echo "[4/4] 安装 Python 依赖..."

cd "$PROJECT_DIR"

PIP_OPTS="--timeout 120 --retries 3 --break-system-packages"

# 清理可能冲突的旧版本
echo "  清理旧版本依赖..."
$PIP_CMD uninstall -y torch torchvision numpy opencv-python-headless cmake packaging setuptools draccus einops gymnasium huggingface-hub requests lerobot 2>/dev/null || true

# 安装 CPU-only torch
echo "  安装 PyTorch（CPU-only）..."
$PIP_CMD install "torch<2.12,>=2.7" "torchvision<0.27,>=0.22" --index-url https://download.pytorch.org/whl/cpu $PIP_OPTS

# LeRobot（--no-deps 防止覆盖 CPU torch）
echo "  安装 LeRobot..."
$PIP_CMD install "git+https://github.com/huggingface/lerobot.git" --no-deps $PIP_OPTS

# LeRobot 其余依赖
echo "  安装 LeRobot 其余依赖..."
$PIP_CMD install \
    "numpy<2.3,>=2.0" \
    "opencv-python-headless<4.14,>=4.9" \
    "cmake<4.2,>=3.29" \
    "packaging<26,>=24.2" \
    "setuptools<82,>=71" \
    "draccus<0.12,>=0.11.6" \
    "einops<0.9,>=0.8" \
    "gymnasium<2.0,>=1.1" \
    "huggingface-hub<2.0,>=1.6" \
    "requests<3.0,>=2.32" \
    Pillow safetensors termcolor tqdm \
    $PIP_OPTS

# oneroh1lerobot + OpenCV
echo "  安装 oneroh1lerobot + OpenCV..."
$PIP_CMD install -e '.[camera]' $PIP_OPTS
echo "  全部依赖安装完成"

# --- 验证 ---
echo ""
echo "============================================"
echo "  验证安装"
echo "============================================"

if $PIP_CMD show lerobot &>/dev/null; then
    echo "  LeRobot: OK"
else
    echo "  [错误] LeRobot 安装失败"
    exit 1
fi

if $PIP_CMD show lerobot_robot_onero_h1 &>/dev/null; then
    echo "  oneroh1lerobot: OK"
else
    echo "  [错误] oneroh1lerobot 安装失败"
    exit 1
fi

if python3 -c "import cv2; print(f'  OpenCV {cv2.__version__}: OK')" 2>/dev/null; then
    :
else
    echo "  [错误] OpenCV 导入失败"
    exit 1
fi

echo ""
echo "============================================"
echo "  安装完成！"
echo "============================================"
echo ""
echo "快速测试（需 ROS2 环境和机器人运行中）："
echo "  onero-h1-print-observation --no-cameras --count 5"
echo ""
echo "录制数据："
echo "  bash scripts/record.sh --repo-id my_name/test --task 'test' --teleop-type homogeneous"
echo "  详细说明见 docs/usage_passive_recording.md"