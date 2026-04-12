"""Camera display widget with high-performance rendering.

Uses QImage for efficient frame display from numpy arrays,
with minimal memory copies and optimized refresh rates.
"""

import numpy as np
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QImage, QPixmap, QPainter
from PySide6.QtWidgets import QLabel, QWidget, QVBoxLayout, QHBoxLayout, QFrame


class CameraDisplay(QLabel):
    """High-performance camera frame display widget.

    Optimized for real-time video display using QImage/QPixmap
    with minimal memory overhead.
    """

    def __init__(self, parent: QWidget | None = None):
        """Initialize camera display widget."""
        super().__init__(parent)
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(320, 240)
        self.setStyleSheet("background-color: #1a1a1a;")
        self.setText("等待相机连接...")
        self._current_frame: np.ndarray | None = None
        self._scaled_pixmap: QPixmap | None = None

    def set_frame(self, frame: np.ndarray) -> None:
        """Display a new frame from numpy array.

        Converts BGR numpy array to QImage efficiently.

        Args:
            frame: numpy array in BGR format (from OpenCV)
        """
        if frame is None or frame.size == 0:
            return

        self._current_frame = frame

        # Create QImage from numpy array (BGR format)
        # This avoids memory copy by using the array data directly
        h, w, ch = frame.shape
        bytes_per_line = ch * w

        # Convert BGR to RGB for QImage
        rgb_frame = frame.copy()
        # cv2.cvtColor is faster than manual swap
        import cv2
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        image = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)

        # Scale to fit widget size while maintaining aspect ratio
        pixmap = QPixmap.fromImage(image)
        self._scaled_pixmap = pixmap.scaled(
            self.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation
        )
        self.setPixmap(self._scaled_pixmap)

    def clear_frame(self) -> None:
        """Clear the current frame."""
        self._current_frame = None
        self._scaled_pixmap = None
        self.clear()
        self.setText("等待相机连接...")

    def resizeEvent(self, event) -> None:
        """Handle resize - rescale current frame."""
        super().resizeEvent(event)
        if self._current_frame is not None:
            self.set_frame(self._current_frame)


class CameraWidget(QWidget):
    """Complete camera display widget with status overlay.

    Includes:
    - Camera display area
    - FPS indicator
    - Status information
    """

    fps_changed = Signal(float)
    connection_status = Signal(str)

    def __init__(self, title: str = "相机视图", parent: QWidget | None = None):
        """Initialize camera widget.

        Args:
            title: Widget title
            parent: Parent widget
        """
        super().__init__(parent)

        self._title = title
        self._fps = 0.0
        self._status = "未连接"

        self._setup_ui()

    def _setup_ui(self) -> None:
        """Setup the widget UI layout."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)

        # Header with title and status
        header = QWidget()
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(0, 0, 0, 0)

        # Title label
        self._title_label = QLabel(self._title)
        self._title_label.setStyleSheet(
            "font-size: 14px; font-weight: bold; color: #ffffff;"
        )
        header_layout.addWidget(self._title_label)

        header_layout.addStretch()

        # FPS display
        self._fps_label = QLabel("FPS: 0.0")
        self._fps_label.setStyleSheet(
            "font-size: 12px; color: #00ff00; font-family: monospace;"
        )
        header_layout.addWidget(self._fps_label)

        # Status indicator
        self._status_label = QLabel("● 未连接")
        self._status_label.setStyleSheet(
            "font-size: 12px; color: #ff6600;"
        )
        header_layout.addWidget(self._status_label)

        layout.addWidget(header)

        # Camera display area with border
        self._display = CameraDisplay()
        self._display.setStyleSheet(
            "border: 2px solid #333333; border-radius: 4px;"
        )
        layout.addWidget(self._display, 1)

    def set_frame(self, frame: np.ndarray) -> None:
        """Display a new frame.

        Args:
            frame: numpy array in BGR format
        """
        self._display.set_frame(frame)

    def clear_frame(self) -> None:
        """Clear the current frame."""
        self._display.clear_frame()

    def set_fps(self, fps: float) -> None:
        """Update FPS display.

        Args:
            fps: Current frames per second
        """
        self._fps = fps
        self._fps_label.setText(f"FPS: {fps:.1f}")
        self.fps_changed.emit(fps)

    def set_status(self, status: str, connected: bool = False) -> None:
        """Update connection status.

        Args:
            status: Status message
            connected: Whether camera is connected
        """
        self._status = status
        if connected:
            self._status_label.setText(f"● {status}")
            self._status_label.setStyleSheet(
                "font-size: 12px; color: #00ff00;"
            )
        else:
            self._status_label.setText(f"● {status}")
            self._status_label.setStyleSheet(
                "font-size: 12px; color: #ff6600;"
            )
        self.connection_status.emit(status)

    def get_display(self) -> CameraDisplay:
        """Get the camera display widget.

        Returns:
            CameraDisplay instance
        """
        return self._display