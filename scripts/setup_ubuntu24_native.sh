#!/usr/bin/env bash
# Native Ubuntu 24.04 / ROS2 Jazzy / RTX 50-series LeRobot setup.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
VENV="${H1_LEROBOT_VENV:-$HOME/.venvs/h1-lerobot}"
UV_BIN="${UV_BIN:-$HOME/.local/bin/uv}"
TORCH_INDEX="https://download.pytorch.org/whl/cu128"

if [ ! -f /etc/os-release ] || ! grep -q '^VERSION_ID="24.04"' /etc/os-release; then
    echo "ERROR: Ubuntu 24.04 is required" >&2
    exit 1
fi
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    echo "ERROR: ROS2 Jazzy is required" >&2
    exit 1
fi
source /opt/ros/jazzy/setup.bash

if [ ! -x "$UV_BIN" ]; then
    curl -LsSf https://astral.sh/uv/install.sh | sh
fi
if [ ! -x "$VENV/bin/python" ]; then
    "$UV_BIN" venv --system-site-packages --python /usr/bin/python3 "$VENV"
fi

"$UV_BIN" pip install --python "$VENV/bin/python" \
    --index "$TORCH_INDEX" --index-strategy unsafe-best-match \
    "torch==2.8.0" "torchvision==0.23.0"
"$UV_BIN" pip install --python "$VENV/bin/python" \
    "lerobot[training,hardware]==0.6.1" "mujoco==3.12.0" "pytest>=8,<10"
"$UV_BIN" pip install --python "$VENV/bin/python" -e "$PROJECT_DIR"

"$VENV/bin/python" - <<'PY'
import cv2
import lerobot
import mujoco
import rclpy
import torch
import torchvision

assert torch.cuda.is_available(), "CUDA is not available"
x = torch.randn(512, 512, device="cuda")
y = x @ x
torch.cuda.synchronize()
print(f"LeRobot {lerobot.__version__}")
print(f"PyTorch {torch.__version__}, torchvision {torchvision.__version__}")
print(f"CUDA {torch.version.cuda}, device={torch.cuda.get_device_name(0)}")
print(f"MuJoCo {mujoco.__version__}, OpenCV {cv2.__version__}")
print(f"rclpy {rclpy.__file__}, GPU smoke={float(y.abs().mean()):.4f}")
PY
