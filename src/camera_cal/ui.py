"""UI drawing module for camera calibration.

This module provides functions for drawing information overlays,
chessboard corners, and other visual feedback during calibration.
"""

import cv2
import numpy as np
from enum import Enum
from typing import Optional

from .config import CalibrationConfig


class CalibrationState(Enum):
    """Calibration state machine."""

    CAPTURE = "CAPTURE"
    PREVIEW = "PREVIEW"


def draw_info_overlay(
    frame: np.ndarray,
    state: CalibrationState,
    sample_count: int,
    detected: bool,
    config: CalibrationConfig,
    saved: bool = False,
) -> None:
    """Draw information overlay on frame.

    Args:
        frame: Input frame (modified in-place)
        state: Current calibration state
        sample_count: Number of captured samples
        detected: Whether chessboard is detected
        config: Calibration configuration
        saved: Whether calibration is saved
    """
    h, w = frame.shape[:2]
    y_offset = 30
    line_height = 30

    # State indicator
    state_color = (
        config.color_capture_mode
        if state == CalibrationState.CAPTURE
        else config.color_preview_mode
    )
    cv2.putText(
        frame,
        f"Mode: {state.value}",
        (10, y_offset),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        state_color,
        2,
    )
    y_offset += line_height

    if state == CalibrationState.CAPTURE:
        _draw_capture_mode_overlay(
            frame, y_offset, line_height, sample_count, detected, config
        )
    else:
        _draw_preview_mode_overlay(frame, y_offset, line_height, sample_count, saved, config)


def _draw_capture_mode_overlay(
    frame: np.ndarray,
    y_offset: int,
    line_height: int,
    sample_count: int,
    detected: bool,
    config: CalibrationConfig,
) -> None:
    """Draw overlay for capture mode.

    Args:
        frame: Input frame (modified in-place)
        y_offset: Starting Y position for text
        line_height: Line height
        sample_count: Number of captured samples
        detected: Whether chessboard is detected
        config: Calibration configuration
    """
    # Sample count
    remaining = max(0, config.min_samples - sample_count)
    count_color = config.color_good_count if remaining == 0 else config.color_need_more
    cv2.putText(
        frame,
        f"Samples: {sample_count} / {config.min_samples}+",
        (10, y_offset),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        count_color,
        2,
    )
    y_offset += line_height

    # Chessboard detection status
    detect_color = config.color_detected if detected else config.color_not_detected
    detect_text = "Chessboard: DETECTED" if detected else "Chessboard: NOT DETECTED"
    cv2.putText(
        frame, detect_text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.7, detect_color, 2
    )
    y_offset += line_height * 2

    # Controls help
    controls = [
        "C: Capture | D: Delete last",
        "ENTER: Calibrate | Q: Quit",
    ]

    for control in controls:
        cv2.putText(
            frame,
            control,
            (10, y_offset),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            config.color_text,
            2,
        )
        y_offset += line_height


def _draw_preview_mode_overlay(
    frame: np.ndarray,
    y_offset: int,
    line_height: int,
    sample_count: int,
    saved: bool,
    config: CalibrationConfig,
) -> None:
    """Draw overlay for preview mode.

    Args:
        frame: Input frame (modified in-place)
        y_offset: Starting Y position for text
        line_height: Line height
        sample_count: Number of captured samples
        saved: Whether calibration is saved
        config: Calibration configuration
    """
    # Sample count
    cv2.putText(
        frame,
        f"Samples: {sample_count}",
        (10, y_offset),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        config.color_good_count,
        2,
    )
    y_offset += line_height

    if saved:
        cv2.putText(
            frame,
            "Saved to calibration.yaml",
            (10, y_offset),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            config.color_good_count,
            2,
        )
    y_offset += line_height * 2

    # Controls help
    controls = [
        "U: Toggle undistort | S: Save",
        "Q: Quit",
    ]

    for control in controls:
        cv2.putText(
            frame,
            control,
            (10, y_offset),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            config.color_text,
            2,
        )
        y_offset += line_height


def draw_chessboard_corners(
    frame: np.ndarray, corners: np.ndarray, pattern_size: tuple[int, int]
) -> None:
    """Draw detected chessboard corners on frame.

    Args:
        frame: Input frame (modified in-place)
        corners: Detected corner points
        pattern_size: Chessboard pattern size (width, height)
    """
    cv2.drawChessboardCorners(frame, pattern_size, corners, True)


def draw_side_by_side(
    frame: np.ndarray, undistorted: np.ndarray
) -> np.ndarray:
    """Draw side-by-side comparison of original and undistorted frames.

    Args:
        frame: Original frame
        undistorted: Undistorted frame

    Returns:
        Combined frame with both original and undistorted views
    """
    combined = np.hstack([frame, undistorted])
    cv2.putText(
        combined,
        "Original",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 0),
        2,
    )
    cv2.putText(
        combined,
        "Undistorted",
        (frame.shape[1] + 10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 0),
        2,
    )
    return combined


def draw_pattern_hint(
    frame: np.ndarray, pattern_size: tuple[int, int], config: CalibrationConfig
) -> None:
    """Draw pattern hint at bottom of frame.

    Args:
        frame: Input frame (modified in-place)
        pattern_size: Chessboard pattern size
        config: Calibration configuration
    """
    hint_text = f"Pattern: {pattern_size[0]}x{pattern_size[1]} corners"
    cv2.putText(
        frame,
        hint_text,
        (10, frame.shape[0] - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        config.color_hint,
        2,
    )
