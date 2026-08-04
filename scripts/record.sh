#!/bin/bash
# oneroh1lerobot 录制启动脚本
# 用法: bash scripts/record.sh [选项]

set -eo pipefail

usage() {
    echo "用法: bash scripts/record.sh [选项]"
    echo ""
    echo "必填参数:"
    echo "  --repo-id REPO_ID       数据集 ID，如 my_name/h1_test"
    echo "  --task TASK             任务描述"
    echo ""
    echo "遥操类型:"
    echo "  --teleop-type TYPE      遥操方式: homogeneous | heterogeneous | vr"
    echo "                          默认 homogeneous。会根据类型自动设置 action topic 和夹爪格式"
    echo ""
    echo "可选参数（与 onero-h1-record-episode 一致）:"
    echo "  --duration SECONDS        录制时长（秒）。不传则持续录制，Ctrl+C 手动停止"
    echo "  --fps FPS                 录制帧率（默认 30）"
    echo "  --cameras CAM1,CAM2       相机列表（默认 head,left,right）"
    echo "  --no-cameras              禁用相机"
    echo "  --root PATH               数据集本地存储路径（默认 ~/lerobot_datasets）"
    echo "  --id ID                   机器人 ID（默认 onero_h1）"
    echo "  --action-left-arm-topic    左臂 action topic（覆盖遥操类型默认值）"
    echo "  --action-right-arm-topic   右臂 action topic（覆盖遥操类型默认值）"
    echo "  --action-gripper-topic     夹爪 action topic（覆盖默认值，int32 模式）"
    echo "  --action-left-gripper-topic  左夹爪 topic（覆盖默认值，float32/VR 模式）"
    echo "  --action-right-gripper-topic 右夹爪 topic（覆盖默认值，float32/VR 模式）"
    echo "  --gripper-type             夹爪消息格式: int32 | float32（脚本自动设置）"
    echo "  --send-hold-action        向机器人发布控制指令（默认不发布）"
    echo "  --finalize                录制完成后锁定数据集"
    echo ""
    echo "示例:"
    echo "  # 主从同构遥操，录制 60 秒"
    echo "  bash scripts/record.sh --repo-id my/test --task 'pick cup' --teleop-type homogeneous --duration 60"
    echo ""
    echo "  # 主从异构遥操"
    echo "  bash scripts/record.sh --repo-id my/test --task 'exo teleop' --teleop-type heterogeneous"
    echo ""
    echo "  # VR 遥操，持续录制"
    echo "  bash scripts/record.sh --repo-id my/test --task 'VR teleop' --teleop-type vr"
    echo ""
    echo "  # 自定义 action topic"
    echo "  bash scripts/record.sh --repo-id my/test --task 'custom' --teleop-type homogeneous \\"
    echo "    --action-left-arm-topic /custom/left --action-right-arm-topic /custom/right"
    echo ""
    exit 0
}

# --- 默认值 ---
REPO_ID=""
TASK=""
TELEOP_TYPE="homogeneous"
DURATION=""
FPS=30
CAMERAS="head,left,right"
NO_CAMERAS=""
ACTION_LEFT_TOPIC=""
ACTION_RIGHT_TOPIC=""
ACTION_GRIPPER_TOPIC=""
ACTION_LEFT_GRIPPER_TOPIC=""
ACTION_RIGHT_GRIPPER_TOPIC=""
SEND_HOLD_ACTION=""
FINALIZE=""
ROOT=""
ROBOT_ID=""
DATASET_ROOT="$HOME/lerobot_datasets"

# --- 解析参数 ---
while [[ $# -gt 0 ]]; do
    case "$1" in
        --repo-id)                REPO_ID="$2"; shift 2 ;;
        --task)                   TASK="$2"; shift 2 ;;
        --teleop-type)            TELEOP_TYPE="$2"; shift 2 ;;
        --duration)               DURATION="$2"; shift 2 ;;
        --fps)                    FPS="$2"; shift 2 ;;
        --cameras)                CAMERAS="$2"; shift 2 ;;
        --no-cameras)             NO_CAMERAS="--no-cameras"; shift ;;
        --root)                   ROOT="$2"; shift 2 ;;
        --id)                     ROBOT_ID="$2"; shift 2 ;;
        --action-left-arm-topic)  ACTION_LEFT_TOPIC="$2"; shift 2 ;;
        --action-right-arm-topic) ACTION_RIGHT_TOPIC="$2"; shift 2 ;;
        --action-gripper-topic)     ACTION_GRIPPER_TOPIC="$2"; shift 2 ;;
        --action-left-gripper-topic)  ACTION_LEFT_GRIPPER_TOPIC="$2"; shift 2 ;;
        --action-right-gripper-topic) ACTION_RIGHT_GRIPPER_TOPIC="$2"; shift 2 ;;
        --send-hold-action)       SEND_HOLD_ACTION="--send-hold-action"; shift ;;
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

# 将 task 拼接到 repo_id 中，作为数据集子目录名（空格和非字母数字替换为下划线）
TASK_SLUG=$(echo "$TASK" | tr '[:upper:]' '[:lower:]' | sed 's/[^a-z0-9]/-/g' | sed 's/--*/-/g' | sed 's/^-//;s/-$//')
REPO_ID="${REPO_ID}_${TASK_SLUG}"

# --- 根据遥操类型设置默认 action topic ---
case "$TELEOP_TYPE" in
    homogeneous)
        if [ -z "$ACTION_LEFT_TOPIC" ]; then
            ACTION_LEFT_TOPIC="/left_joint_states"
        fi
        if [ -z "$ACTION_RIGHT_TOPIC" ]; then
            ACTION_RIGHT_TOPIC="/right_joint_states"
        fi
        if [ -z "$ACTION_GRIPPER_TOPIC" ]; then
            ACTION_GRIPPER_TOPIC="/joystick_info"
        fi
        GRIPPER_TYPE="int32"
        echo "遥操类型: 主从同构"
        ;;
    heterogeneous)
        if [ -z "$ACTION_LEFT_TOPIC" ]; then
            ACTION_LEFT_TOPIC="/teleop/left/joint_states"
        fi
        if [ -z "$ACTION_RIGHT_TOPIC" ]; then
            ACTION_RIGHT_TOPIC="/teleop/right/joint_states"
        fi
        if [ -z "$ACTION_GRIPPER_TOPIC" ]; then
            ACTION_GRIPPER_TOPIC="/joystick_info"
        fi
        GRIPPER_TYPE="int32"
        echo "遥操类型: 主从异构"
        ;;
    vr)
        if [ -z "$ACTION_LEFT_TOPIC" ]; then
            ACTION_LEFT_TOPIC="/left_joint_states"
        fi
        if [ -z "$ACTION_RIGHT_TOPIC" ]; then
            ACTION_RIGHT_TOPIC="/right_joint_states"
        fi
        if [ -z "$ACTION_LEFT_GRIPPER_TOPIC" ]; then
            ACTION_LEFT_GRIPPER_TOPIC="/vr/left_gripper/open_ratio"
        fi
        if [ -z "$ACTION_RIGHT_GRIPPER_TOPIC" ]; then
            ACTION_RIGHT_GRIPPER_TOPIC="/vr/right_gripper/open_ratio"
        fi
        GRIPPER_TYPE="float32"
        echo "遥操类型: VR 遥操"
        ;;
    *)
        echo "[错误] 未知遥操类型: $TELEOP_TYPE (可选: homogeneous, heterogeneous, vr)"
        exit 1
        ;;
esac

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
CMD="$CMD --action-left-arm-topic $ACTION_LEFT_TOPIC"
CMD="$CMD --action-right-arm-topic $ACTION_RIGHT_TOPIC"
if [ "$GRIPPER_TYPE" = "float32" ]; then
    CMD="$CMD --action-left-gripper-topic $ACTION_LEFT_GRIPPER_TOPIC"
    CMD="$CMD --action-right-gripper-topic $ACTION_RIGHT_GRIPPER_TOPIC"
else
    CMD="$CMD --action-gripper-topic $ACTION_GRIPPER_TOPIC"
fi
CMD="$CMD --gripper-type $GRIPPER_TYPE"

if [ -n "$DURATION" ]; then
    CMD="$CMD --duration $DURATION"
    echo "录制模式: 定时（${DURATION}s）"
else
    echo "录制模式: 持续（Ctrl+C 停止）"
fi

CMD="$CMD --cameras $CAMERAS"
[ -n "$NO_CAMERAS" ] && CMD="$CMD $NO_CAMERAS"
[ -n "$ROBOT_ID" ] && CMD="$CMD --id $ROBOT_ID"
[ -n "$SEND_HOLD_ACTION" ] && CMD="$CMD $SEND_HOLD_ACTION"
[ -n "$FINALIZE" ] && CMD="$CMD $FINALIZE"

echo "数据集: $REPO_ID"
echo "任务: $TASK"
echo "帧率: ${FPS} fps"
echo "相机: $CAMERAS"
echo ""
echo "--- Action 话题 ---"
echo "左臂:   $ACTION_LEFT_TOPIC"
echo "右臂:   $ACTION_RIGHT_TOPIC"
if [ "$GRIPPER_TYPE" = "float32" ]; then
    echo "左夹爪: $ACTION_LEFT_GRIPPER_TOPIC ($GRIPPER_TYPE)"
    echo "右夹爪: $ACTION_RIGHT_GRIPPER_TOPIC ($GRIPPER_TYPE)"
else
    echo "夹爪:   $ACTION_GRIPPER_TOPIC ($GRIPPER_TYPE)"
fi
echo "--------------------"
# 如果用户指定了 --root，使用它作为根目录；否则用默认值
ACTUAL_ROOT="${ROOT:-$DATASET_ROOT}"
echo "存储路径: ${ACTUAL_ROOT}/${REPO_ID}"

# --- 执行录制 ---
echo "执行: $CMD"
echo ""

# 设置 LeRobot 数据集根目录，LeRobot 自动创建 <repo_id> 子目录
export HF_LEROBOT_HOME="$ACTUAL_ROOT"
eval "$CMD"