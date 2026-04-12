"""Configuration module for camera calibration.

This module contains all configuration parameters for the camera calibration
tool, including chessboard pattern settings, calibration parameters, and UI
settings.
"""

from dataclasses import dataclass
import cv2


@dataclass
class CalibrationConfig:
    """Configuration parameters for camera calibration."""

    # Chessboard pattern configuration
    pattern_size: tuple[int, int] = (9, 6)  # Inner corners (width x height)
    square_size: float = 19.0  # Square size in millimeters
    min_samples: int = 12  # Minimum number of images for calibration

    # Calibration parameters
    calib_flags: int = cv2.CALIB_FIX_K3
    alpha: float = 1.0  # Free scaling parameter for undistortion

    # UI configuration
    window_name: str = "Interactive Calibration"
    window_width: int = 1280
    window_height: int = 720
    capture_cooldown: float = 0.6  # Minimum seconds between captures

    # Display colors
    color_capture_mode: tuple[int, int, int] = (0, 255, 0)
    color_preview_mode: tuple[int, int, int] = (255, 0, 255)
    color_detected: tuple[int, int, int] = (0, 255, 0)
    color_not_detected: tuple[int, int, int] = (0, 0, 255)
    color_good_count: tuple[int, int, int] = (0, 255, 0)
    color_need_more: tuple[int, int, int] = (0, 165, 255)
    color_text: tuple[int, int, int] = (255, 255, 255)
    color_hint: tuple[int, int, int] = (0, 255, 255)

    # Camera settings
    usb_width: int = 1280
    usb_height: int = 720

    # Quality thresholds for reprojection error (in pixels)
    quality_excellent: float = 0.3
    quality_good: float = 0.5
    quality_acceptable: float = 1.0

    # Performance optimization
    preview_scale: float = 0.5  # Downscale factor for preview detection
    detect_every_n_frames: int = 3  # Skip frames (0 = every frame)
    enable_hardware_decode: bool = False  # VA-API HW decode for RTSP


# Default configuration instance
default_config = CalibrationConfig()
