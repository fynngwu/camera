"""Helpers for converting numpy / OpenCV frames into Qt images."""
from __future__ import annotations

from typing import Optional

import numpy as np

from gcs_gui.qt_compat import QImage


def ndarray_to_qimage(frame: Optional[np.ndarray]) -> Optional[QImage]:
    """Convert a BGR / RGB / grayscale numpy image to a detached QImage."""
    if frame is None:
        return None
    array = np.ascontiguousarray(frame)
    if array.ndim == 2:
        height, width = array.shape
        image = QImage(array.data, width, height, width, QImage.Format_Grayscale8)
        return image.copy()
    if array.ndim != 3:
        return None
    height, width, channels = array.shape
    if channels == 3:
        rgb = array[:, :, ::-1].copy()
        image = QImage(rgb.data, width, height, width * 3, QImage.Format_RGB888)
        return image.copy()
    if channels == 4:
        image = QImage(array.data, width, height, width * 4, QImage.Format_RGBA8888)
        return image.copy()
    return None
