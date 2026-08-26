#!/usr/bin/env bash
# Create the ROS Humble-compatible MuJoCo simulator environment with uv.
set -eo pipefail

VENV="${H1_MUJOCO_VENV:-$HOME/.venvs/h1-mujoco}"
UV_BIN="${UV_BIN:-$HOME/.local/bin/uv}"

if [ -z "${ROS_DISTRO:-}" ]; then
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    elif [ -f /opt/ros/jazzy/setup.bash ]; then
        source /opt/ros/jazzy/setup.bash
    fi
fi

if [ ! -x "$UV_BIN" ]; then
    python3 -m pip install --user --upgrade uv
fi

if [ ! -x "$VENV/bin/python" ]; then
    "$UV_BIN" venv --system-site-packages --python /usr/bin/python3 "$VENV"
fi

# ROS-distributed OpenCV builds commonly target the NumPy 1.x ABI. Keep NumPy
# below 2 while allowing uv to select a version supported by the active Python.
"$UV_BIN" pip install --python "$VENV/bin/python" \
    "numpy>=1.23.5,<2" \
    "mujoco==3.12.0" \
    "setuptools>=71,<80"

"$VENV/bin/python" - <<'PY'
import cv2
import mujoco
import numpy
import rclpy

print(f"MuJoCo {mujoco.__version__}")
print(f"NumPy {numpy.__version__}")
print(f"OpenCV {cv2.__version__}")
print(f"rclpy {rclpy.__file__}")
PY
