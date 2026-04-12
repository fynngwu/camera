"""Camera Viewer Qt Frontend.

A high-performance camera viewer application built with PySide6,
supporting RTSP streams (via GStreamer) and USB cameras.

Modules:
    - view: UI components (MainWindow, CameraWidget)
    - server: Backend threads (CameraWorker, FrameBuffer)

Usage:
    python -m frontend.main
"""

from .view import MainWindow, CameraWidget
from .server import CameraWorker, FrameBuffer

__all__ = ["MainWindow", "CameraWidget", "CameraWorker", "FrameBuffer"]