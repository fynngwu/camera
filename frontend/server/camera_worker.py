"""Camera worker thread for background frame capture.

Implements low-latency frame grabbing using GStreamer RTSP or USB camera,
running in a QThread to avoid blocking the GUI.
"""

import time
import cv2
import numpy as np
from PySide6.QtCore import QThread, Signal, QObject
from typing import Optional

from .frame_buffer import FrameBuffer


# Default GStreamer pipeline for low-latency RTSP streaming
DEFAULT_RTSP_PIPELINE = (
    "rtspsrc location=rtsp://100.103.8.66:8554/video latency=0 ! "
    "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
    "appsink drop=true max-buffers=1 sync=false"
)


class CameraWorker(QThread):
    """Background thread for camera frame capture.

    This worker continuously grabs frames from a camera source
    and emits signals when new frames are available.

    Supports:
    - RTSP streams via GStreamer (low-latency pipeline)
    - USB cameras via OpenCV
    """

    # Signals for thread-safe communication
    frame_ready = Signal(np.ndarray)  # New frame available
    fps_updated = Signal(float)  # FPS statistics update
    error_occurred = Signal(str)  # Error message
    started_capture = Signal()  # Successfully started

    def __init__(
        self,
        source: str = "rtsp",
        rtsp_url: Optional[str] = None,
        camera_id: int = 0,
        gstreamer_pipeline: Optional[str] = None,
        parent: Optional[QObject] = None,
    ):
        """Initialize camera worker.

        Args:
            source: Camera source type - "rtsp" or "usb"
            rtsp_url: RTSP stream URL (for source="rtsp")
            camera_id: USB camera device ID (for source="usb")
            gstreamer_pipeline: Custom GStreamer pipeline (overrides rtsp_url)
            parent: Parent QObject
        """
        super().__init__(parent)

        self._source = source
        self._rtsp_url = rtsp_url
        self._camera_id = camera_id
        self._pipeline = gstreamer_pipeline
        self._cap: Optional[cv2.VideoCapture] = None
        self._buffer = FrameBuffer()
        self._stop_flag = False
        self._is_running = False

        # Frame dimensions (set after first frame)
        self._frame_width = 0
        self._frame_height = 0

    def _create_gstreamer_pipeline(self, url: str) -> str:
        """Create low-latency GStreamer pipeline for RTSP.

        Args:
            url: RTSP stream URL

        Returns:
            GStreamer pipeline string
        """
        return (
            f"rtspsrc location={url} latency=0 ! "
            "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
            "appsink drop=true max-buffers=1 sync=false"
        )

    def _open_camera(self) -> bool:
        """Open the camera source.

        Returns:
            True if successfully opened
        """
        if self._source == "rtsp":
            if self._pipeline:
                pipeline = self._pipeline
            elif self._rtsp_url:
                pipeline = self._create_gstreamer_pipeline(self._rtsp_url)
            else:
                pipeline = DEFAULT_RTSP_PIPELINE

            self._cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        else:  # USB camera
            self._cap = cv2.VideoCapture(self._camera_id)
            # Set high resolution for USB camera
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
            self._cap.set(cv2.CAP_PROP_FPS, 30)

        if not self._cap or not self._cap.isOpened():
            self.error_occurred.emit(f"Cannot open camera source: {self._source}")
            return False

        return True

    def _wait_for_first_frame(self, timeout: float = 3.0) -> Optional[np.ndarray]:
        """Wait for the first frame with timeout.

        Args:
            timeout: Maximum wait time in seconds

        Returns:
            First frame if received, None otherwise
        """
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline:
            ok, frame = self._cap.read()
            if ok and frame is not None:
                self._frame_width = frame.shape[1]
                self._frame_height = frame.shape[0]
                return frame
            time.sleep(0.01)

        return None

    def run(self) -> None:
        """Main worker loop - runs in background thread."""
        if not self._open_camera():
            return

        # Wait for first frame
        first_frame = self._wait_for_first_frame()
        if first_frame is None:
            self.error_occurred.emit("Cannot receive first frame")
            self._cap.release()
            return

        self._buffer.write(first_frame)
        self._is_running = True
        self.started_capture.emit()
        self.frame_ready.emit(first_frame)

        # Main capture loop
        while not self._stop_flag:
            ok, frame = self._cap.read()

            if not ok or frame is None:
                # Brief pause on read failure
                time.sleep(0.005)
                continue

            # Store in buffer
            frame_id = self._buffer.write(frame)

            # Emit signal for new frame
            self.frame_ready.emit(frame)

            # Periodically emit FPS
            fps = self._buffer.get_fps()
            if fps > 0:
                self.fps_updated.emit(fps)

        # Cleanup
        self._is_running = False
        if self._cap:
            self._cap.release()
            self._cap = None

    def stop(self) -> None:
        """Stop the worker thread."""
        self._stop_flag = True
        self.wait()  # Wait for thread to finish

    def get_buffer(self) -> FrameBuffer:
        """Get the frame buffer for direct access.

        Returns:
            FrameBuffer instance
        """
        return self._buffer

    def get_frame_dimensions(self) -> tuple[int, int]:
        """Get the frame dimensions.

        Returns:
            Tuple of (width, height)
        """
        return self._frame_width, self._frame_height

    def is_running(self) -> bool:
        """Check if worker is actively capturing.

        Returns:
            True if running and capturing frames
        """
        return self._is_running

    def get_latest_frame(self) -> Optional[np.ndarray]:
        """Get the latest frame from buffer.

        Returns:
            Latest frame or None if buffer empty
        """
        _, frame = self._buffer.read(0)
        return frame