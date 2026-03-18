"""OpenCV USB Camera Calibration Package."""

__version__ = "0.1.0"

# Main entry point
from .interactive_calibrate import main

# Configuration
from .config import CalibrationConfig, default_config

# Camera handling
from .camera import CameraStream

# Calibration logic
from .calibrator import CameraCalibrator, CalibrationResult

# UI components
from .ui import CalibrationState

__all__ = [
    "main",
    "CalibrationConfig",
    "default_config",
    "CameraStream",
    "CameraCalibrator",
    "CalibrationResult",
    "CalibrationState",
]
