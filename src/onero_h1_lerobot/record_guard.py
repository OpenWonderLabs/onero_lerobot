"""Graceful Ctrl+C handling for LeRobot's native record CLI.

LeRobot's ``lerobot-record`` finalizes the dataset and disconnects the robot on
Ctrl+C, but it does not save the in-progress episode: its
``VideoEncodingManager`` cleanup discards the pending buffer. This hook saves
that episode first, so the native command behaves like the stop topic.

The hook is installed when LeRobot imports the third-party plugin
(``lerobot_robot_onero_h1``) at CLI startup.
"""

from __future__ import annotations

import logging

_logger = logging.getLogger("onero_h1.record_guard")

_INSTALLED = False


def install_native_record_guard() -> None:
    """Patch ``VideoEncodingManager.__exit__`` to save on Ctrl+C.

    Idempotent; silently does nothing when LeRobot is not installed.
    """
    global _INSTALLED
    if _INSTALLED:
        return

    try:
        from lerobot.datasets.video_utils import VideoEncodingManager
    except Exception:
        return

    original_exit = VideoEncodingManager.__exit__

    def _has_pending_frames(dataset) -> bool:
        try:
            return bool(dataset.has_pending_frames())
        except Exception:
            # The buffer may be mid-save (e.g. its "size" key already popped);
            # in that case let LeRobot's default cleanup handle it.
            return False

    def _exit(self, exc_type, exc_val, exc_tb):
        dataset = getattr(self, "dataset", None)
        interrupted = exc_type is not None and issubclass(exc_type, KeyboardInterrupt)
        has_pending = dataset is not None and hasattr(dataset, "has_pending_frames")
        if interrupted and has_pending and _has_pending_frames(dataset):
            _logger.info("检测到 Ctrl+C，正在保存当前 episode...")
            try:
                dataset.save_episode()
                _logger.info("当前 episode 已保存")
            except Exception:
                # A frame may have been interrupted mid-write; fall back to
                # LeRobot's default cleanup instead of blocking shutdown.
                _logger.exception("保存被中断的 episode 失败，将按 LeRobot 默认流程清理")
        return original_exit(self, exc_type, exc_val, exc_tb)

    VideoEncodingManager.__exit__ = _exit
    _INSTALLED = True
