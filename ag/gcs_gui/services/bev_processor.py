"""AprilTag-based BEV processor for the GUI camera pipeline."""
from __future__ import annotations

import os
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import cv2
import numpy as np

try:  # pragma: no cover
    from pupil_apriltags import Detector
except Exception:  # pragma: no cover
    Detector = None


@dataclass
class BevProcessorConfig:
    """Configuration for the BEV processor."""
    enabled: bool = True
    calibration_path: str = ""
    target_id: int = 0
    tag_size: float = 0.09
    x_min: float = -1.0
    x_max: float = 1.0
    y_min: float = -1.0
    y_max: float = 1.0
    ppm: int = 300


class BevProcessor:
    """Convert raw RTSP frames into a BEV canvas using AprilTag detection."""

    def __init__(self, config: BevProcessorConfig) -> None:
        self.config = config
        self.detector = None
        self.map1 = None
        self.map2 = None
        self.last_bev: Optional[np.ndarray] = None
        if Detector is not None and self.config.enabled:
            self.detector = Detector(
                families="tag36h11",
                nthreads=max(1, min(4, os.cpu_count() or 2)),
                quad_decimate=1.5,
                quad_sigma=0.0,
                refine_edges=1,
                decode_sharpening=0.5,
                debug=0,
            )

    @property
    def bev_size(self) -> Tuple[int, int]:
        """Return BEV image size as width, height."""
        width = int((self.config.x_max - self.config.x_min) * self.config.ppm)
        height = int((self.config.y_max - self.config.y_min) * self.config.ppm)
        return max(32, width), max(32, height)

    def process(self, frame: np.ndarray) -> Tuple[np.ndarray, np.ndarray, Dict[str, object]]:
        """Process one raw frame into annotated raw image + BEV image + metadata."""
        width, height = self.bev_size
        raw = frame.copy()
        if frame is None or frame.size == 0:
            return raw, np.zeros((height, width, 3), dtype=np.uint8), {"tag_detected": False, "mode": "empty", "detect_ms": 0.0}
        undistorted = self._undistort_if_possible(raw)
        bev = np.zeros((height, width, 3), dtype=np.uint8)
        bev[:] = (20, 20, 20)
        self._draw_grid(bev)
        if not self.config.enabled or self.detector is None:
            small = cv2.resize(undistorted, (width, height), interpolation=cv2.INTER_LINEAR)
            bev = cv2.addWeighted(bev, 0.35, small, 0.65, 0.0)
            self.last_bev = bev
            return undistorted, bev, {"tag_detected": False, "mode": "passthrough", "detect_ms": 0.0}
        gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)
        t0 = time.perf_counter()
        tags = self.detector.detect(gray)
        detect_ms = (time.perf_counter() - t0) * 1000.0
        det = None
        for item in tags:
            if int(item.tag_id) == int(self.config.target_id):
                det = item
                break
        if det is None:
            if self.last_bev is not None:
                bev = self.last_bev.copy()
            return undistorted, bev, {"tag_detected": False, "mode": "no_tag", "detect_ms": detect_ms}
        corners = det.corners.astype(np.float32)
        cv2.polylines(undistorted, [corners.astype(np.int32)], True, (0, 255, 0), 2)
        cv2.putText(
            undistorted,
            f"tag={det.tag_id} {detect_ms:.1f}ms",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
        )
        dst = np.array(
            [
                self.world_to_pixel(self.config.tag_size / 2.0, self.config.tag_size / 2.0),
                self.world_to_pixel(-self.config.tag_size / 2.0, self.config.tag_size / 2.0),
                self.world_to_pixel(-self.config.tag_size / 2.0, -self.config.tag_size / 2.0),
                self.world_to_pixel(self.config.tag_size / 2.0, -self.config.tag_size / 2.0),
            ],
            dtype=np.float32,
        )
        matrix = cv2.getPerspectiveTransform(corners, dst)
        warped = cv2.warpPerspective(undistorted, matrix, (width, height))
        bev = cv2.addWeighted(bev, 0.25, warped, 0.75, 0.0)
        self._draw_grid(bev)
        self.last_bev = bev
        return undistorted, bev, {"tag_detected": True, "mode": "apriltag", "detect_ms": detect_ms, "tag_id": int(det.tag_id)}

    def world_to_pixel(self, x: float, y: float) -> Tuple[float, float]:
        """Convert one world point to BEV pixel coordinates."""
        width, height = self.bev_size
        u = (float(x) - self.config.x_min) / max(self.config.x_max - self.config.x_min, 1e-6) * width
        v = (self.config.y_max - float(y)) / max(self.config.y_max - self.config.y_min, 1e-6) * height
        return float(u), float(v)

    def pixel_to_world(self, u: float, v: float) -> Tuple[float, float]:
        """Convert one BEV pixel coordinate to world coordinates."""
        width, height = self.bev_size
        x = self.config.x_min + (float(u) / max(width, 1)) * (self.config.x_max - self.config.x_min)
        y = self.config.y_max - (float(v) / max(height, 1)) * (self.config.y_max - self.config.y_min)
        return float(x), float(y)

    def _undistort_if_possible(self, frame: np.ndarray) -> np.ndarray:
        """Undistort one frame when calibration data is available."""
        if not self.config.calibration_path or not os.path.exists(self.config.calibration_path):
            return frame
        if self.map1 is None or self.map2 is None:
            try:
                data = np.load(self.config.calibration_path)
                camera_matrix = data["camera_matrix"].astype(np.float32)
                dist_coeffs = data["dist_coeffs"].astype(np.float32)
                h, w = frame.shape[:2]
                new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 0, (w, h))
                self.map1, self.map2 = cv2.initUndistortRectifyMap(
                    camera_matrix,
                    dist_coeffs,
                    None,
                    new_camera_matrix,
                    (w, h),
                    cv2.CV_32FC1,
                )
            except Exception:
                return frame
        return cv2.remap(frame, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)

    def _draw_grid(self, image: np.ndarray) -> None:
        """Draw a metric-looking helper grid on the BEV image."""
        width, height = self.bev_size
        step = max(20, self.config.ppm // 5)
        for x in range(0, width, step):
            cv2.line(image, (x, 0), (x, height), (50, 50, 50), 1)
        for y in range(0, height, step):
            cv2.line(image, (0, y), (width, y), (50, 50, 50), 1)
