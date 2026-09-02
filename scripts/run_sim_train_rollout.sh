#!/usr/bin/env bash
# End-to-end MuJoCo simulation -> LeRobot record -> ACT train -> policy rollout.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
VENV="${H1_LEROBOT_VENV:-$HOME/.venvs/h1-lerobot}"
PYTHON="$VENV/bin/python"
BIN_DIR="$VENV/bin"
RUN_ROOT="${H1_VALIDATION_ROOT:-$HOME/onero_h1_runs}"
RUN_ID="${H1_RUN_ID:-$(date +%Y%m%d-%H%M%S)}"
RUN_DIR="$RUN_ROOT/$RUN_ID"
DATASET_DIR="$RUN_DIR/dataset"
TRAIN_DIR="$RUN_DIR/train"
LOG_DIR="$RUN_DIR/logs"
MODEL="$PROJECT_DIR/urdf/urdf/X1_mjcf_with_grippers.urdf"
FPS="${H1_FPS:-5}"
EPISODES="${H1_EPISODES:-2}"
EPISODE_SECONDS="${H1_EPISODE_SECONDS:-4}"
TRAIN_STEPS="${H1_TRAIN_STEPS:-10}"
ROLLOUT_SECONDS="${H1_ROLLOUT_SECONDS:-5}"
REPO_ID="local/h1_sim_$RUN_ID"

if [ ! -x "$PYTHON" ]; then
    echo "ERROR: environment missing: $VENV (run setup_ubuntu24_native.sh first)" >&2
    exit 1
fi
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "ERROR: ROS2 Jazzy or Humble is required" >&2
    exit 1
fi
export PATH="$BIN_DIR:$PATH"
export PYTHONUNBUFFERED=1
mkdir -p "$LOG_DIR"

SIM_PID=""
cleanup() {
    if [ -n "$SIM_PID" ] && kill -0 "$SIM_PID" 2>/dev/null; then
        kill "$SIM_PID" 2>/dev/null || true
        wait "$SIM_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

"$PYTHON" "$SCRIPT_DIR/h1_topic_sim.py" \
    --mujoco-model "$MODEL" --leader --fps 30 --camera-fps "$FPS" \
    >"$LOG_DIR/simulator.log" 2>&1 &
SIM_PID=$!

for _ in $(seq 1 30); do
    kill -0 "$SIM_PID" 2>/dev/null || { cat "$LOG_DIR/simulator.log"; exit 1; }
    if "$PYTHON" - <<'PY' >/dev/null 2>&1
import rclpy
from rclpy.node import Node
rclpy.init()
n = Node("h1_wait_for_sim")
names = {name for name, _ in n.get_topic_names_and_types()}
n.destroy_node(); rclpy.shutdown()
raise SystemExit(0 if "/joint_states" in names and "/head/camera/rgb" in names else 1)
PY
    then break; fi
    sleep 1
done

echo "[$RUN_ID] recording synthetic MuJoCo dataset"
onero-h1-lerobot-record \
    --play_sounds=false \
    --robot.type=onero_h1 --robot.id=h1_sim \
    --robot.use_cameras=true --robot.send_action=true \
    --teleop.type=onero_h1_ros_joint --teleop.id=synthetic_leader \
    --dataset.repo_id="$REPO_ID" --dataset.root="$DATASET_DIR" \
    --dataset.single_task="Move both arms and grippers with a smooth synthetic leader trajectory" \
    --dataset.fps="$FPS" --dataset.episode_time_s="$EPISODE_SECONDS" \
    --dataset.reset_time_s=1 --dataset.num_episodes="$EPISODES" \
    --dataset.video=false --dataset.push_to_hub=false \
    2>&1 | tee "$LOG_DIR/record.log"

echo "[$RUN_ID] training ACT for $TRAIN_STEPS steps on RTX GPU"
lerobot-train \
    --dataset.repo_id="$REPO_ID" --dataset.root="$DATASET_DIR" \
    --policy.type=act --policy.device=cuda --policy.push_to_hub=false \
    --steps="$TRAIN_STEPS" --save_freq="$TRAIN_STEPS" \
    --batch_size=1 --num_workers=0 \
    --output_dir="$TRAIN_DIR" \
    2>&1 | tee "$LOG_DIR/train.log"

CHECKPOINT="$TRAIN_DIR/checkpoints/$(printf '%06d' "$TRAIN_STEPS")/pretrained_model"
if [ ! -f "$CHECKPOINT/config.json" ]; then
    echo "ERROR: checkpoint missing: $CHECKPOINT" >&2
    exit 1
fi

echo "[$RUN_ID] rolling out trained policy in MuJoCo"
onero-h1-lerobot-rollout \
    --strategy.type=base --policy.path="$CHECKPOINT" \
    --robot.type=onero_h1 --robot.id=h1_sim \
    --robot.use_cameras=true --robot.send_action=true \
    --device=cuda --fps="$FPS" --duration="$ROLLOUT_SECONDS" \
    --task="Move both arms and grippers with a smooth synthetic leader trajectory" \
    2>&1 | tee "$LOG_DIR/rollout.log"

cat >"$RUN_DIR/SUCCESS.txt" <<EOF
run_id=$RUN_ID
repo_id=$REPO_ID
dataset=$DATASET_DIR
checkpoint=$CHECKPOINT
train_steps=$TRAIN_STEPS
rollout_seconds=$ROLLOUT_SECONDS
EOF

echo "SUCCESS: $RUN_DIR"
