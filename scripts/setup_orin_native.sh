#!/usr/bin/env bash
# Native setup for Jetson Orin running Ubuntu 24.04 / ROS2 Jazzy.
#
# Usage:
#   bash scripts/setup_orin_native.sh
#
# Optional overrides:
#   H1_LEROBOT_VENV=$HOME/.venvs/h1-lerobot
#   H1_TORCH_INDEX=https://pypi.jetson-ai-lab.io/sbsa/cu132
#   H1_TORCH_SPEC='torch>=2.7,<2.12'
#   H1_TORCHVISION_SPEC='torchvision>=0.22,<0.27'
#   H1_REQUIRE_NATIVE_SM87=0
#   H1_JETPACK_PACKAGE=nvidia-jetpack-runtime
#   H1_INSTALL_JETPACK=0
#   H1_INSTALL_VIZ=0
#   H1_RUN_TESTS=1

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
VENV="${H1_LEROBOT_VENV:-$HOME/.venvs/h1-lerobot}"
TORCH_INDEX="${H1_TORCH_INDEX:-https://pypi.jetson-ai-lab.io/sbsa/cu132}"
TORCH_SPEC="${H1_TORCH_SPEC:-torch>=2.7,<2.12}"
TORCHVISION_SPEC="${H1_TORCHVISION_SPEC:-torchvision>=0.22,<0.27}"
REQUIRE_NATIVE_SM87="${H1_REQUIRE_NATIVE_SM87:-0}"
JETPACK_PACKAGE="${H1_JETPACK_PACKAGE:-nvidia-jetpack-runtime}"
INSTALL_JETPACK="${H1_INSTALL_JETPACK:-0}"
INSTALL_VIZ="${H1_INSTALL_VIZ:-0}"
RUN_TESTS="${H1_RUN_TESTS:-1}"
LOG_DIR="${H1_SETUP_LOG_DIR:-$HOME/.cache/onero_h1_setup}"
RUN_TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="$LOG_DIR/setup_orin_${RUN_TIMESTAMP}.log"
PIP_NETWORK_OPTS=(--timeout 120 --retries 10)

mkdir -p "$LOG_DIR"
exec > >(tee -a "$LOG_FILE") 2>&1

# Keep an audit trail of the original system state. The script never uninstalls
# system packages and never uses pip outside the dedicated virtual environment.
dpkg-query -W -f='${binary:Package}\t${Version}\n' >"$LOG_DIR/packages_before_${RUN_TIMESTAMP}.tsv" 2>/dev/null || true

step() {
    printf '\n============================================\n'
    printf '[%s] %s\n' "$(date '+%H:%M:%S')" "$1"
    printf '============================================\n'
}

die() {
    printf '\nERROR: %s\n' "$1" >&2
    printf '安装日志：%s\n' "$LOG_FILE" >&2
    exit 1
}

version_ge_312() {
    python3 - <<'PY'
import sys
raise SystemExit(0 if sys.version_info >= (3, 12) else 1)
PY
}

is_gpu_torch_compatible() {
    "$VENV/bin/python" - <<'PY'
from importlib.metadata import PackageNotFoundError, version

try:
    from packaging.version import Version
    import torch
except (ImportError, PackageNotFoundError):
    raise SystemExit(1)

torch_version = Version(version("torch").split("+")[0])
if not (Version("2.7") <= torch_version < Version("2.12")):
    raise SystemExit(2)
if not torch.cuda.is_available():
    print("PyTorch 校验失败：torch.cuda.is_available() 为 False。")
    raise SystemExit(3)
if "orin" not in torch.cuda.get_device_name(0).lower():
    print(f"PyTorch 校验失败：GPU 不是 Orin：{torch.cuda.get_device_name(0)}")
    raise SystemExit(4)

major, minor = torch.cuda.get_device_capability(0)
required_arch = f"sm_{major}{minor}"
built_arches = torch.cuda.get_arch_list()
if required_arch not in built_arches:
    print(f"PyTorch wheel 未原生列出 {required_arch}；开始验证实际 rollout 算子。")

# JetPack 7.2's available LeRobot-compatible wheel can use PTX/runtime fallback
# on Orin even though get_arch_list() doesn't list sm_87. Exercise more than
# cuBLAS matmul so a wheel that cannot execute policy kernels is rejected.
try:
    import torch.nn.functional as F

    with torch.inference_mode():
        image = torch.randn(2, 3, 64, 64, device="cuda")
        conv = torch.nn.Conv2d(3, 16, 3, padding=1, device="cuda")(image)
        norm = F.layer_norm(conv, conv.shape[1:])
        resized = F.interpolate(norm, size=(32, 32), mode="bilinear", align_corners=False)
        q = torch.randn(2, 4, 32, 64, device="cuda", dtype=torch.float16)
        attention = F.scaled_dot_product_attention(q, q, q)
        logits = torch.nn.Linear(64, 16, device="cuda")(attention.float())
        loss_like = resized.square().mean() + logits.square().mean()
        torch.cuda.synchronize()
        if not bool(torch.isfinite(loss_like)):
            raise RuntimeError("GPU smoke test produced NaN/Inf")
except Exception as exc:
    print(f"PyTorch GPU rollout 算子校验失败：{type(exc).__name__}: {exc}")
    raise SystemExit(5)

print("PyTorch GPU rollout 算子校验通过。")
PY
}

require_download_dns() {
    local host
    for host in pypi.jetson-ai-lab.io files.pythonhosted.org; do
        if ! getent ahosts "$host" >/dev/null 2>&1; then
            die "无法解析下载域名 $host。请先修复开发板 DNS，再重新运行脚本。"
        fi
    done
    printf '下载域名 DNS 检查通过。\n'
}

step "检查目标平台"

[[ "$(uname -m)" == "aarch64" ]] || die "本脚本仅支持 aarch64 Jetson Orin。"
[[ -r /etc/os-release ]] || die "无法读取 /etc/os-release。"
# shellcheck disable=SC1091
source /etc/os-release
[[ "${VERSION_ID:-}" == "24.04" ]] || die "需要 Ubuntu 24.04，当前为 ${VERSION_ID:-unknown}。"
[[ -r /etc/nv_tegra_release ]] || die "未检测到 Jetson Linux (/etc/nv_tegra_release)。"
grep -q '^# R39 ' /etc/nv_tegra_release || die "需要 Jetson Linux R39.x。"
version_ge_312 || die "LeRobot 0.6.1 需要 Python >= 3.12。"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    die "未找到 ROS2 Jazzy：/opt/ros/jazzy/setup.bash"
fi

printf '硬件：%s\n' "$(tr -d '\0' </proc/device-tree/model 2>/dev/null || printf 'Jetson')"
printf '系统：Ubuntu %s, %s\n' "${VERSION_ID}" "$(head -n 1 /etc/nv_tegra_release)"
printf 'Python：%s\n' "$(python3 --version 2>&1)"
printf 'ROS：Jazzy\n'

step "安装最小系统依赖"

if [[ "$INSTALL_JETPACK" == "1" ]]; then
    if ! dpkg-query -W -f='${Status}' "$JETPACK_PACKAGE" 2>/dev/null | grep -q 'install ok installed'; then
        printf '将安装 %s。sudo 只用于 apt 系统包。\n' "$JETPACK_PACKAGE"
        sudo apt-get update
        printf '\n--- apt 变更预览 ---\n'
        sudo apt-get --simulate install --no-install-recommends "$JETPACK_PACKAGE"
        sudo apt-get install -y --no-install-recommends "$JETPACK_PACKAGE"
    else
        printf '%s 已安装，跳过。\n' "$JETPACK_PACKAGE"
    fi
else
    printf '默认跳过 JetPack/TensorRT 元包；本项目使用 PyTorch 执行 rollout。\n'
    printf '如确实需要完整 JetPack runtime，可设置 H1_INSTALL_JETPACK=1。\n'
fi

sudo apt-get update
sudo apt-get install -y --no-install-recommends \
    python3-venv python3-pip \
    ffmpeg

if [[ -f /var/run/reboot-required ]]; then
    printf '\n系统包安装后要求重启。请执行：\n'
    printf '  sudo reboot\n'
    printf '重启后再次运行同一命令：\n'
    printf '  bash scripts/setup_orin_native.sh\n'
    printf '日志：%s\n' "$LOG_FILE"
    exit 20
fi

step "创建 Python 虚拟环境"

if [[ ! -x "$VENV/bin/python" ]]; then
    python3 -m venv --system-site-packages "$VENV"
fi

# ROS setup scripts may read optional variables before defining them, which is
# incompatible with this script's global `set -u`. Disable nounset only while
# sourcing ROS, then restore strict checking for the rest of the installer.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u
# shellcheck disable=SC1090
source "$VENV/bin/activate"

python -m pip install "${PIP_NETWORK_OPTS[@]}" --upgrade \
    pip 'setuptools>=71,<80' wheel 'packaging>=24.2,<26'
python -c 'import rclpy; print("rclpy:", rclpy.__file__)' \
    || die "虚拟环境无法导入 Jazzy rclpy。"

step "安装并验证 Jetson GPU PyTorch"

if is_gpu_torch_compatible; then
    printf '检测到兼容的 Jetson GPU PyTorch，保留现有安装。\n'
else
    TORCH_CHECK_STATUS=$?
    if [[ "$TORCH_CHECK_STATUS" == "5" ]]; then
        die "现有 PyTorch 无法执行 rollout 常用 GPU 算子；重复下载同一 wheel 无法解决。"
    fi
    require_download_dns
    printf '从 JetPack 7.2 可用的 ARM64 wheel 源安装：\n'
    printf '  index=%s\n' "$TORCH_INDEX"
    printf '  %s\n' "$TORCH_SPEC"
    printf '  %s\n' "$TORCHVISION_SPEC"
    printf '\n注意：该 wheel 源由 Jetson AI Lab 提供，不是 PyPI 默认源。\n'

    python -m pip install \
        "${PIP_NETWORK_OPTS[@]}" \
        --no-cache-dir \
        --upgrade \
        --force-reinstall \
        --index-url "$TORCH_INDEX" \
        "$TORCH_SPEC" "$TORCHVISION_SPEC" \
        || die "没有找到满足 LeRobot 0.6.1 约束的 Jetson GPU PyTorch。不要改装普通 PyPI torch。"
fi

is_gpu_torch_compatible \
    || die "PyTorch 未通过版本、CUDA、Orin GPU 或 rollout 算子检查。"

if [[ "$REQUIRE_NATIVE_SM87" == "1" ]]; then
    "$VENV/bin/python" - <<'PY' \
        || die "严格模式要求 wheel 原生包含 sm_87；当前可用的 LeRobot 0.6.1 wheel 不满足。"
import torch
raise SystemExit(0 if "sm_87" in torch.cuda.get_arch_list() else 1)
PY
fi

python - <<'PY'
import torch

print("PyTorch:", torch.__version__)
print("CUDA build:", torch.version.cuda)
print("CUDA available:", torch.cuda.is_available())
print("Device:", torch.cuda.get_device_name(0))
print("Device capability:", torch.cuda.get_device_capability(0))
print("Wheel architectures:", torch.cuda.get_arch_list())
x = torch.randn(1024, 1024, device="cuda")
y = x @ x
torch.cuda.synchronize()
print("GPU smoke: OK", float(y.abs().mean()))
PY

step "安装 LeRobot 0.6.1 和 Onero H1 适配器"

LEROBOT_EXTRAS="dataset"
if [[ "$INSTALL_VIZ" == "1" ]]; then
    LEROBOT_EXTRAS="dataset_viz"
fi

# A compatible torch is already installed. pip should retain it; the mandatory
# post-install GPU check below prevents a silent replacement with a CPU wheel.
python -m pip install "${PIP_NETWORK_OPTS[@]}" "lerobot[$LEROBOT_EXTRAS]==0.6.1"

# The venv deliberately exposes ROS Jazzy through --system-site-packages. On
# Ubuntu 24.04 this also exposes the distro pandas build, which can satisfy
# pip's version constraint while being ABI-incompatible with the newer NumPy
# required by LeRobot. The distro SciPy build has the same NumPy-version
# problem, so install all three into the venv instead of resolving them from
# /usr/lib/python3/dist-packages.
python -m pip install \
    "${PIP_NETWORK_OPTS[@]}" \
    --upgrade \
    --ignore-installed \
    'numpy>=2,<2.3' 'pandas>=2,<3' 'scipy>=1.14,<2' 'cffi>=1.17,<2'

python -m pip install "${PIP_NETWORK_OPTS[@]}" -e "$PROJECT_DIR"

is_gpu_torch_compatible \
    || die "安装 LeRobot 后 Jetson GPU PyTorch 被替换或变得不可用。"

step "最终验证"

python - <<'PY'
import importlib.metadata as metadata
import rclpy
import torch
from lerobot.datasets.lerobot_dataset import LeRobotDataset

from onero_h1_lerobot import OneroH1Config, OneroH1Robot

robot = OneroH1Robot(OneroH1Config(id="setup-verification"))
scalar_observations = [
    key
    for key, feature_type in robot.observation_features.items()
    if feature_type is float
]

assert metadata.version("lerobot") == "0.6.1"
assert torch.cuda.is_available()
assert len(robot.action_features) == 16
assert len(scalar_observations) == 17

print("LeRobot:", metadata.version("lerobot"))
print("LeRobotDataset:", LeRobotDataset.__module__)
print("Adapter:", metadata.version("lerobot_robot_onero_h1"))
print("rclpy:", rclpy.__file__)
print("PyTorch:", torch.__version__)
print("GPU:", torch.cuda.get_device_name(0))
print("GPU capability:", torch.cuda.get_device_capability(0))
print("Wheel architectures:", torch.cuda.get_arch_list())
print("GPU rollout operator smoke: passed")
print("Action schema: 16 dimensions")
print("Observation state schema: 17 dimensions")
PY

if [[ "$RUN_TESTS" == "1" ]]; then
    python -m pip install "${PIP_NETWORK_OPTS[@]}" 'pytest>=8,<9'
    # ROS Jazzy exposes launch_testing globally through --system-site-packages.
    # It is unrelated to this package and can be incompatible with venv pytest.
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q "$PROJECT_DIR/tests"
fi

step "安装完成"

printf '虚拟环境：%s\n' "$VENV"
printf '安装日志：%s\n' "$LOG_FILE"
printf '\n每次使用前执行：\n'
printf '  source /opt/ros/jazzy/setup.bash\n'
printf '  source %q/bin/activate\n' "$VENV"
printf '\n下一步只读验证：\n'
printf '  onero-h1-print-observation --no-cameras --count 5\n'
