#!/usr/bin/env bash
# Start the lightweight ROS2 topic-level Onero H1 simulator.
# ROS setup scripts reference optional unset variables, so enable nounset only
# after the environment has been sourced.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -z "${ROS_DISTRO:-}" ]; then
    if [ -f /opt/ros/jazzy/setup.bash ]; then
        source /opt/ros/jazzy/setup.bash
    elif [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    else
        echo "ERROR: ROS2 Jazzy or Humble was not found under /opt/ros" >&2
        exit 1
    fi
fi

set -u
PYTHON_BIN="${H1_SIM_PYTHON:-$HOME/.venvs/h1-mujoco/bin/python}"
if [ ! -x "$PYTHON_BIN" ]; then
    PYTHON_BIN=python3
fi
exec "$PYTHON_BIN" "$SCRIPT_DIR/h1_topic_sim.py" "$@"
