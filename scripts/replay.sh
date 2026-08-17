#!/bin/bash
# oneroh1lerobot 回放启动脚本
# 用法: bash scripts/replay.sh [选项]

set -eo pipefail

usage() {
    echo "用法: bash scripts/replay.sh [选项]"
    echo ""
    echo "必填参数:"
    echo "  --repo-id REPO_ID       数据集 ID，如 my_name/h1_test"
    echo ""
    echo "可选参数:"
    echo "  --episode N              回放的 episode 序号（默认 0）"
    echo "  --fps FPS                 回放帧率（默认使用数据集原始帧率）"
    echo "  --root PATH               数据集本地存储路径（默认 ~/lerobot_datasets）"
    echo "  --id ID                   机器人 ID（默认 onero_h1）"
    echo "  --arm-command-mode MODE   手臂指令模式: record_data | movej（默认 record_data）"
    echo "  --no-gripper              回放时不发送夹爪指令（默认发送）"
    echo "  --no-cameras              禁用相机"
    echo "  --cameras CAM1,CAM2       相机列表（默认 head,left,right）"
    echo ""
    echo "示例:"
    echo "  # 回放指定 episode（默认发送夹爪）"
    echo "  bash scripts/replay.sh --repo-id my/test --episode 0"
    echo ""
    echo "  # 回放不发送夹爪"
    echo "  bash scripts/replay.sh --repo-id my/test --episode 0 --no-gripper"
    echo ""
    echo "  # 回放指定数据集路径"
    echo "  bash scripts/replay.sh --repo-id my/test --episode 0 --root /path/to/datasets"
    echo ""
    exit 0
}

# --- 默认值 ---
REPO_ID=""
EPISODE=0
FPS=""
ROOT=""
ROBOT_ID=""
ARM_COMMAND_MODE="record_data"
NO_GRIPPER=""
NO_CAMERAS=""
CAMERAS="head,left,right"
DATASET_ROOT="$HOME/lerobot_datasets"

# --- 解析参数 ---
while [[ $# -gt 0 ]]; do
    case "$1" in
        --repo-id)           REPO_ID="$2"; shift 2 ;;
        --episode)           EPISODE="$2"; shift 2 ;;
        --fps)               FPS="$2"; shift 2 ;;
        --root)              ROOT="$2"; shift 2 ;;
        --id)                ROBOT_ID="$2"; shift 2 ;;
        --arm-command-mode)  ARM_COMMAND_MODE="$2"; shift 2 ;;
        --no-gripper)        NO_GRIPPER="--no-gripper"; shift ;;
        --no-cameras)        NO_CAMERAS="--no-cameras"; shift ;;
        --cameras)           CAMERAS="$2"; shift 2 ;;
        -h|--help)           usage ;;
        *) echo "未知参数: $1"; usage ;;
    esac
done

# --- 检查必填参数 ---
if [ -z "$REPO_ID" ]; then
    echo "[错误] --repo-id 为必填参数"
    usage
fi

# --- 加载 ROS2 环境 ---
if [ -z "${ROS_DISTRO:-}" ]; then
    if [ -f /opt/ros/jazzy/setup.bash ]; then
        source /opt/ros/jazzy/setup.bash
    elif [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    else
        echo "[错误] 未找到 ROS2 环境"
        exit 1
    fi
fi

# --- 构建命令 ---
CMD="onero-h1-replay-episode"
CMD="$CMD --repo-id $REPO_ID"
CMD="$CMD --episode $EPISODE"
CMD="$CMD --arm-command-mode $ARM_COMMAND_MODE"
CMD="$CMD --cameras $CAMERAS"
[ -n "$FPS" ] && CMD="$CMD --fps $FPS"
[ -n "$ROBOT_ID" ] && CMD="$CMD --id $ROBOT_ID"
[ -n "$NO_GRIPPER" ] && CMD="$CMD $NO_GRIPPER"
[ -n "$NO_CAMERAS" ] && CMD="$CMD $NO_CAMERAS"

# 数据集根目录
ACTUAL_ROOT="${ROOT:-$DATASET_ROOT}"

echo "数据集: $REPO_ID"
echo "Episode: $EPISODE"
echo "手臂指令模式: $ARM_COMMAND_MODE"
echo "发送夹爪: ${NO_GRIPPER:+否}${NO_GRIPPER:-是}"
echo "存储路径: $ACTUAL_ROOT"
echo ""

# --- 执行回放 ---
echo "执行: $CMD"
echo ""

export HF_LEROBOT_HOME="$ACTUAL_ROOT"
eval "$CMD"