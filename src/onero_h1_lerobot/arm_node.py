from __future__ import annotations

import logging
import shutil
import subprocess
import time
from pathlib import Path

_logger = logging.getLogger("onero_h1.arm_node")

# 主臂固定端口（由 robot_ws 的 udev 规则生成）
DEVICE_PATHS = ("/dev/ttyMasterCANL", "/dev/ttyMasterCANR")
# 用于识别已运行实例的进程特征（兼容本地 install/ 与 /opt/robot-h1 两种路径）
PROCESS_MARKER = "armcontrol/lib/armcontrol/arm_record_node"


def resolve_remaps(mode: str) -> list[tuple[str, str]]:
    """Return ROS remaps for the homogeneous leader arm program.

    In ``teleop`` mode the leader must not publish ``/record_data`` while
    LeRobot's control loop is also publishing it, so that output is redirected.
    ``/joystick_info`` is intentionally not remapped: it carries the 20/21 mode
    handshake and the gripper command chain.
    """
    if mode == "teleop":
        return [("/record_data", "/leader/arm_data")]
    return []


class ArmNodeLauncher:
    """按需启动 H1 主臂桥接程序 ``arm_record_node``。

    LeRobot 连接机器人时自动调用；若程序未安装、设备不存在或已有实例在运行，
    则安静跳过，不影响其他流程。
    """

    def __init__(
        self,
        executable: str = "arm_record_node",
        remaps: list[tuple[str, str]] | None = None,
    ) -> None:
        self._executable = executable
        self._remaps = remaps or []
        self._process: subprocess.Popen | None = None
        self._started_by_us = False

    def is_running(self) -> bool:
        try:
            result = subprocess.run(
                ["pgrep", "-f", PROCESS_MARKER],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )
        except FileNotFoundError:
            return False
        return result.returncode == 0

    def start(self) -> bool:
        if self.is_running():
            if self._remaps:
                _logger.warning(
                    "主臂程序已在运行，当前实例未应用本次 remap；"
                    "如需遥操，请先停止已有实例再重新启动"
                )
            else:
                _logger.info("主臂程序已在运行，跳过自动启动")
            return False

        executable = shutil.which(self._executable)
        if executable is None:
            fallback = Path("/usr/local/bin") / self._executable
            if fallback.is_file():
                executable = str(fallback)
            else:
                _logger.info("未找到 %s，跳过自动启动主臂", self._executable)
                return False

        missing = [path for path in DEVICE_PATHS if not Path(path).exists()]
        if missing:
            _logger.warning("未检测到主臂设备 %s，跳过自动启动", ", ".join(missing))
            return False

        command = [executable]
        if self._remaps:
            command.append("--ros-args")
            for source, target in self._remaps:
                command.extend(["-r", f"{source}:={target}"])

        _logger.info("自动启动主臂程序: %s", " ".join(command))
        try:
            self._process = subprocess.Popen(command, stdin=subprocess.DEVNULL)
        except OSError as exc:
            _logger.warning("主臂程序启动失败: %s", exc)
            self._process = None
            return False

        # 快速确认进程没有立即退出
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if self._process.poll() is not None:
                _logger.warning("主臂程序启动后立即退出 (code=%s)", self._process.returncode)
                self._process = None
                return False
            time.sleep(0.1)

        self._started_by_us = True
        return True

    def stop(self) -> None:
        if self._process is None or not self._started_by_us:
            return
        _logger.info("停止自动启动的主臂程序")
        self._process.terminate()
        try:
            self._process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self._process.kill()
            self._process.wait()
        self._process = None
        self._started_by_us = False
