#!/usr/bin/env bash
# Restart the H1 MuJoCo ROS simulator with an interactive viewer on this display.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
RUN_DIR="$(dirname "$PROJECT_DIR")/run"
MODEL="$PROJECT_DIR/urdf/urdf/X1_mjcf_with_grippers.urdf"
mkdir -p "$RUN_DIR"

if [ -f "$RUN_DIR/h1-sim.pid" ]; then
    OLD_PID="$(cat "$RUN_DIR/h1-sim.pid")"
    if kill -0 "$OLD_PID" 2>/dev/null; then
        kill "$OLD_PID"
        for _ in 1 2 3 4 5; do
            kill -0 "$OLD_PID" 2>/dev/null || break
            sleep 1
        done
    fi
fi

echo $$ > "$RUN_DIR/h1-sim.pid"
echo "Starting Onero H1 MuJoCo viewer on DISPLAY=${DISPLAY:-unset}"
echo "Model: $MODEL"
exec bash "$SCRIPT_DIR/sim_h1.sh" \
    --mujoco-model "$MODEL" \
    --viewer \
    --fps 30 \
    --camera-fps 10
