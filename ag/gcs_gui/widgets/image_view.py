"""Generic image display widget."""
from __future__ import annotations

from typing import Optional

import numpy as np

from gcs_gui.image_utils import ndarray_to_qimage
from gcs_gui.qt_compat import QColor, QPainter, QPen, QSize, Qt, QWidget


class ImageView(QWidget):
    """Simple widget that displays the latest frame with aspect-ratio preservation."""

    def __init__(self, title: str) -> None:
        super().__init__()
        self.title = title
        self._image = None
        self._overlay_text = ""
        self.setMinimumSize(QSize(320, 240))

    def set_frame(self, frame: np.ndarray, overlay_text: str = "") -> None:
        """Update displayed image from an OpenCV frame."""
        self._image = ndarray_to_qimage(frame)
        self._overlay_text = overlay_text
        self.update()

    def set_placeholder(self, text: str) -> None:
        """Show placeholder text without a frame."""
        self._image = None
        self._overlay_text = text
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802
        """Render the current frame or a placeholder background."""
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(18, 18, 18))
        if self._image is not None:
            scaled = self._image.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            x = (self.width() - scaled.width()) // 2
            y = (self.height() - scaled.height()) // 2
            painter.drawImage(x, y, scaled)
        painter.setPen(QPen(QColor(255, 255, 255), 1))
        painter.drawText(12, 24, self.title)
        if self._overlay_text:
            painter.drawText(12, self.height() - 12, self._overlay_text)
