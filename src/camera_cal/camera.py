"""Camera stream management module.

This module provides a unified interface for camera input from both
USB devices and RTSP streams using GStreamer.
"""

import cv2
import numpy as np
from typing import Tuple
from .config import CalibrationConfig


class CameraStream:
    """Unified camera stream interface supporting USB and RTSP sources.

    This class encapsulates video capture functionality with support for:
    - USB cameras (via device ID)
    - RTSP streams (via GStreamer pipeline)
    - Configurable resolution
    """

    def __init__(
        self,
        camera_id: int = 0,
        rtsp_url: str | None = None,
        config: CalibrationConfig | None = None,
    ):
        """Initialize camera stream.

        Args:
            camera_id: USB camera device ID (default: 0)
            rtsp_url: RTSP stream URL (if provided, takes precedence over camera_id)
            config: Calibration configuration (uses default if None)
        """
        self.camera_id = camera_id
        self.rtsp_url = rtsp_url
        self.config = config or CalibrationConfig()
        self.cap: cv2.VideoCapture | None = None
        self._is_rtsp = rtsp_url is not None

    def _create_pipeline(self, url: str) -> str:
        """Create GStreamer pipeline for RTSP stream.

        Supports hardware acceleration if enabled in config.

        Args:
            url: RTSP stream URL

        Returns:
            GStreamer pipeline string
        """
        if self.config.enable_hardware_decode:
            # Try VA-API hardware decode
            return (
                f"rtspsrc location={url} latency=100 ! "
                "rtph264depay ! h264parse ! "
                "vah264dec ! videoconvert ! "
                "video/x-raw,format=BGR ! "
                "appsink max-buffers=1 drop=true sync=false"
            )
        else:
            # Software decode (current)
            return (
                f"rtspsrc location={url} latency=100 ! "
                "rtph264depay ! h264parse ! avdec_h264 ! "
                "videoconvert ! appsink drop=true max-buffers=1 sync=false"
            )

    def open(self) -> bool:
        """Open the camera stream.

        Returns:
            True if successfully opened, False otherwise
        """
        if self._is_rtsp:
            pipeline = self._create_pipeline(self.rtsp_url)
            print(f"Connecting to RTSP: {self.rtsp_url}")
            self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        else:
            print(f"Using USB camera: /dev/video{self.camera_id}")
            self.cap = cv2.VideoCapture(self.camera_id)
            # Set resolution for USB camera
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.usb_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.usb_height)

        if not self.cap.isOpened():
            if self._is_rtsp:
                print(f"Error: Cannot open RTSP stream: {self.rtsp_url}")
            else:
                print(f"Error: Cannot open camera {self.camera_id}")
            return False

        if self._is_rtsp:
            print(f"Connected to RTSP: {self.rtsp_url}")

        return True

    def read(self) -> Tuple[bool, np.ndarray]:
        """Read a frame from the camera.

        Returns:
            Tuple of (success, frame) where frame is numpy array if success
        """
        if self.cap is None:
            return False, np.array([])
        return self.cap.read()

    def release(self) -> None:
        """Release the camera resource."""
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def __enter__(self):
        """Context manager entry."""
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.release()
