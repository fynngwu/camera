"""Frame buffer management for low-latency camera streaming.

Provides thread-safe frame storage with minimal latency,
inspired by the LatestFrameGrabber pattern in apriltag_to_bev.
"""

import threading
import time
from typing import Optional, Tuple
import numpy as np


class FrameBuffer:
    """Thread-safe frame buffer optimized for low-latency streaming.

    This class stores the latest frame with minimal overhead,
    using a lock-free pattern where possible and tracking frame IDs
    to avoid processing the same frame multiple times.
    """

    def __init__(self, max_fps_stats_interval: float = 1.0):
        """Initialize frame buffer.

        Args:
            max_fps_stats_interval: Interval in seconds for FPS calculation
        """
        self._lock = threading.Lock()
        self._frame: Optional[np.ndarray] = None
        self._frame_id: int = 0
        self._frame_time: float = 0.0

        # FPS statistics
        self._read_count: int = 0
        self._last_fps_time: float = time.perf_counter()
        self._fps_interval: float = max_fps_stats_interval
        self._current_fps: float = 0.0

    def write(self, frame: np.ndarray) -> int:
        """Write a new frame to the buffer.

        Args:
            frame: numpy array (BGR format from OpenCV)

        Returns:
            The new frame ID
        """
        with self._lock:
            self._frame = frame
            self._frame_id += 1
            self._frame_time = time.perf_counter()

            # Update FPS
            self._read_count += 1
            now = time.perf_counter()
            elapsed = now - self._last_fps_time
            if elapsed >= self._fps_interval:
                self._current_fps = self._read_count / elapsed
                self._read_count = 0
                self._last_fps_time = now

            return self._frame_id

    def read(self, last_seen_id: int) -> Tuple[int, Optional[np.ndarray]]:
        """Read the latest frame if it's newer than last_seen_id.

        Args:
            last_seen_id: The last frame ID that was processed

        Returns:
            Tuple of (frame_id, frame) where frame may be None if no new frame
        """
        with self._lock:
            if self._frame is None or self._frame_id == last_seen_id:
                return last_seen_id, None
            return self._frame_id, self._frame.copy()

    def read_no_copy(self, last_seen_id: int) -> Tuple[int, Optional[np.ndarray]]:
        """Read the latest frame without copying (use carefully).

        WARNING: The returned frame is shared and may be overwritten.
        Only use this if you process the frame immediately.

        Args:
            last_seen_id: The last frame ID that was processed

        Returns:
            Tuple of (frame_id, frame) where frame may be None if no new frame
        """
        with self._lock:
            if self._frame is None or self._frame_id == last_seen_id:
                return last_seen_id, None
            return self._frame_id, self._frame

    def get_fps(self) -> float:
        """Get current frame rate.

        Returns:
            FPS calculated over the last stats interval
        """
        return self._current_fps

    def get_frame_count(self) -> int:
        """Get total frame count since start.

        Returns:
            Total number of frames written
        """
        with self._lock:
            return self._frame_id

    def clear(self) -> None:
        """Clear the buffer."""
        with self._lock:
            self._frame = None
            self._frame_id = 0

    def is_empty(self) -> bool:
        """Check if buffer has no frame.

        Returns:
            True if no frame is stored
        """
        with self._lock:
            return self._frame is None