#!/bin/bash
# oneroh1lerobot 录制启动脚本（支持主从同构与 VR）
# 用法: bash scripts/record.sh [选项]

set -eo pipefail

usage() {
    echo "用法: bash scripts/record.sh [选项]"
    echo ""
    echo "必填参数:"
    echo "  --repo-id REPO_ID       数据集 ID，如 my_name/h1_test（默认追加时间戳）"
    echo "  --task TASK             任务描述"
    echo ""
    echo "可选参数（与 onero-h1-record-episode 一致）:"
    echo "  --resume                  继续录制已有数据集（repo-id 必须是完整已有 ID）"
    echo "  --no-stamp                保持 repo-id 原样；目录已存在会报错，不会覆盖"
    echo "  --duration SECONDS        录制时长（秒）。不传则持续录制，通过 /stop_recording 或 Ctrl+C 停止"
    echo "  --fps FPS                 录制帧率（默认 30）"
    echo "  --cameras CAM1,CAM2       相机列表（默认 head,left,right）"
    echo "  --no-cameras              禁用相机"
    echo "  --passive                 被动录制，不下发控制指令（默认主动控制）"
    echo "  --teleop-type TYPE        遥操类型：homogeneous（默认，主臂）或 vr"
    echo "  --no-gripper              禁用夹爪 action 订阅（无夹爪话题时使用）"
    echo "  --no-gripper-action       不向 /joystick_info 发布夹爪指令"
    echo "  --root PATH               数据集本地存储路径（默认 ~/lerobot_datasets）"
    echo "  --id ID                   机器人 ID（默认 onero_h1）"
    echo "  --action-left-arm-topic    左臂 action topic（默认 /left/joint_states；VR 为 /left_joint_states）"
    echo "  --action-right-arm-topic   右臂 action topic（默认 /right/joint_states；VR 为 /right_joint_states）"
    echo "  --action-gripper-topic     夹爪 action topic（默认 /joystick_info）"
    echo "  --action-left-gripper-topic   VR 左夹爪 action topic（Float32）"
    echo "  --action-right-gripper-topic  VR 右夹爪 action topic（Float32）"
    echo "  --finalize                兼容参数；LeRobot 0.6.1 会始终 finalize 数据集"
    echo ""
    echo "示例:"
    echo "  # 主从同构遥操，主动控制并录制 60 秒"
    echo "  bash scripts/record.sh --repo-id my/test --task 'pick cup' --duration 60"
    echo ""
    echo "  # 被动录制，只记录不下发"
    echo "  bash scripts/record.sh --repo-id my/test --task 'passive' --passive"
    echo ""
    echo "  # 继续录制已有的带时间戳数据集"
    echo "  bash scripts/record.sh --repo-id my/test_20261119_153000 --task 'pick cup' --resume"
    echo ""
    exit 0
}

# --- 默认值 ---
REPO_ID=""
TASK=""
DURATION=""
FPS=30
TELEOP_TYPE="homogeneous"
RESUME=""
NO_STAMP=""
CAMERAS="head,left,right"
NO_CAMERAS=""
PASSIVE=""
ACTION_LEFT_TOPIC=""
ACTION_RIGHT_TOPIC=""
ACTION_GRIPPER_TOPIC=""
ACTION_LEFT_GRIPPER_TOPIC=""
ACTION_RIGHT_GRIPPER_TOPIC=""
NO_GRIPPER=""
NO_GRIPPER_ACTION=""
FINALIZE=""
ROOT=""
ROBOT_ID=""
DATASET_ROOT="$HOME/lerobot_datasets"

# --- 解析参数 ---
while [[ $# -gt 0 ]]; do
    case "$1" in
        --repo-id)                REPO_ID="$2"; shift 2 ;;
        --task)                   TASK="$2"; shift 2 ;;
        --resume)                 RESUME="--resume"; shift ;;
        --no-stamp)               NO_STAMP="--no-stamp"; shift ;;
        --duration)               DURATION="$2"; shift 2 ;;
        --fps)                    FPS="$2"; shift 2 ;;
        --cameras)                CAMERAS="$2"; shift 2 ;;
        --no-cameras)             NO_CAMERAS="--no-cameras"; shift ;;
        --passive)                PASSIVE="--passive"; shift ;;
        --teleop-type)
            [[ "$2" == "homogeneous" || "$2" == "vr" ]] || { echo "[错误] 仅支持 --teleop-type homogeneous|vr"; exit 1; }
            TELEOP_TYPE="$2"; shift 2 ;;
        --no-gripper)             NO_GRIPPER="--no-gripper"; shift ;;
        --no-gripper-action)      NO_GRIPPER_ACTION="--no-gripper-action"; shift ;;
        --root)                   ROOT="$2"; shift 2 ;;
        --id)                     ROBOT_ID="$2"; shift 2 ;;
        --action-left-arm-topic)  ACTION_LEFT_TOPIC="$2"; shift 2 ;;
        --action-right-arm-topic) ACTION_RIGHT_TOPIC="$2"; shift 2 ;;
        --action-gripper-topic)   ACTION_GRIPPER_TOPIC="$2"; shift 2 ;;
        --action-left-gripper-topic)  ACTION_LEFT_GRIPPER_TOPIC="$2"; shift 2 ;;
        --action-right-gripper-topic) ACTION_RIGHT_GRIPPER_TOPIC="$2"; shift 2 ;;
        # 兼容旧参数：夹爪类型已移除，由 teleop-type 决定
        --gripper-type)           shift 2 ;;
        --send-hold-action)       shift ;;
        --finalize)               FINALIZE="--finalize"; shift ;;
        -h|--help)                usage ;;
        *) echo "未知参数: $1"; usage ;;
    esac
done

# --- 检查必填参数 ---
if [ -z "$REPO_ID" ] || [ -z "$TASK" ]; then
    echo "[错误] --repo-id 和 --task 为必填参数"
    usage
fi

# --- 直接使用传入的 repo-id，不再追加 task 后缀 ---

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
CMD="onero-h1-record-episode"
CMD="$CMD --repo-id $REPO_ID"
CMD="$CMD --task '$TASK'"
CMD="$CMD --fps $FPS"
CMD="$CMD --teleop-type $TELEOP_TYPE"
[ -n "$RESUME" ] && CMD="$CMD $RESUME"
[ -n "$NO_STAMP" ] && CMD="$CMD $NO_STAMP"
[ -n "$ACTION_LEFT_TOPIC" ] && CMD="$CMD --action-left-arm-topic $ACTION_LEFT_TOPIC"
[ -n "$ACTION_RIGHT_TOPIC" ] && CMD="$CMD --action-right-arm-topic $ACTION_RIGHT_TOPIC"
[ -n "$ACTION_GRIPPER_TOPIC" ] && CMD="$CMD --action-gripper-topic $ACTION_GRIPPER_TOPIC"
[ -n "$ACTION_LEFT_GRIPPER_TOPIC" ] && CMD="$CMD --action-left-gripper-topic $ACTION_LEFT_GRIPPER_TOPIC"
[ -n "$ACTION_RIGHT_GRIPPER_TOPIC" ] && CMD="$CMD --action-right-gripper-topic $ACTION_RIGHT_GRIPPER_TOPIC"

if [ -n "$DURATION" ]; then
    CMD="$CMD --duration $DURATION"
    echo "录制模式: 定时（${DURATION}s）"
else
    echo "录制模式: 持续（/stop_recording 话题或 Ctrl+C 停止）"
fi

CMD="$CMD --cameras $CAMERAS"
[ -n "$NO_CAMERAS" ] && CMD="$CMD $NO_CAMERAS"
[ -n "$PASSIVE" ] && CMD="$CMD $PASSIVE"
[ -n "$NO_GRIPPER" ] && CMD="$CMD $NO_GRIPPER"
[ -n "$NO_GRIPPER_ACTION" ] && CMD="$CMD $NO_GRIPPER_ACTION"
[ -n "$ROBOT_ID" ] && CMD="$CMD --id $ROBOT_ID"
[ -n "$FINALIZE" ] && CMD="$CMD $FINALIZE"

if [ -n "$RESUME" ]; then
    echo "数据集: $REPO_ID（继续录制）"
elif [ -n "$NO_STAMP" ]; then
    echo "数据集: $REPO_ID（固定 ID）"
else
    echo "数据集: $REPO_ID（实际 ID 会追加时间戳，以启动日志为准）"
fi
echo "任务: $TASK"
echo "帧率: ${FPS} fps"
echo "遥操类型: $TELEOP_TYPE"
echo "相机: $CAMERAS"
echo "模式: $([ -n "$PASSIVE" ] && echo '被动录制' || echo '主动控制 + 录制')"
echo ""
# 如果用户指定了 --root，使用它作为根目录；否则用默认值
ACTUAL_ROOT="${ROOT:-$DATASET_ROOT}"
if [ -n "$RESUME" ] || [ -n "$NO_STAMP" ]; then
    echo "存储路径: ${ACTUAL_ROOT}/${REPO_ID}"
else
    echo "存储路径: ${ACTUAL_ROOT}/${REPO_ID}_<时间戳>"
fi

# --- 执行录制 ---
echo "执行: $CMD"
echo ""

# 设置 LeRobot 数据集根目录，LeRobot 自动创建 <repo_id> 子目录
export HF_LEROBOT_HOME="$ACTUAL_ROOT"
eval "$CMD"
