"""Ground station image backend: RTSP ingest + AprilTag BEV conversion."""
from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, Optional, Tuple

import cv2
import numpy as np
import yaml


LogCallback = Callable[[str, str], None]
FrameCallback = Callable[[np.ndarray], None]
MetaCallback = Callable[[Dict[str, object]], None]


@dataclass
class BevConfig:
    """Configuration for ArUco based BEV processing."""

    enabled: bool = True
    calibration_path: str = "config/calibration.yaml"
    target_id: int = 0
    tag_size: float = 0.09
    aruco_dict: str = "DICT_4X4_50"
    x_min: float = -1.0
    x_max: float = 1.0
    y_min: float = -1.0
    y_max: float = 1.0
    ppm: int = 300


class AprilTagBevProcessor:
    """Convert raw image to BEV image with ArUco marker perspective anchoring."""

    def __init__(self, config: BevConfig) -> None:
        self.config = config
        self.map1 = None
        self.map2 = None
        self.last_bev: Optional[np.ndarray] = None
        self.detector = None
        if config.enabled:
            dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            parameters = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    @property
    def bev_size(self) -> Tuple[int, int]:
        width = int((self.config.x_max - self.config.x_min) * self.config.ppm)
        height = int((self.config.y_max - self.config.y_min) * self.config.ppm)
        return max(32, width), max(32, height)

    def process(self, frame: np.ndarray) -> Tuple[np.ndarray, np.ndarray, Dict[str, object]]:
        """Process one frame and return (raw_annotated, bev, meta)."""
        width, height = self.bev_size
        raw = frame.copy()
        if frame is None or frame.size == 0:
            return raw, np.zeros((height, width, 3), dtype=np.uint8), {"mode": "empty", "tag_detected": False}

        undistorted = self._undistort(frame)
        bev = np.zeros((height, width, 3), dtype=np.uint8)
        bev[:] = (24, 24, 24)
        self._draw_grid(bev)

        if not self.config.enabled or self.detector is None:
            passthrough = cv2.resize(undistorted, (width, height), interpolation=cv2.INTER_LINEAR)
            bev = cv2.addWeighted(bev, 0.35, passthrough, 0.65, 0.0)
            self.last_bev = bev
            return undistorted, bev, {"mode": "passthrough", "tag_detected": False, "detect_ms": 0.0}

        gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)
        t0 = time.perf_counter()
        corners_list, ids, _ = self.detector.detectMarkers(gray)
        detect_ms = (time.perf_counter() - t0) * 1000.0

        tag = None
        if ids is not None:
            for raw_corners, tag_id in zip(corners_list, ids.reshape(-1)):
                if int(tag_id) == int(self.config.target_id):
                    corners = np.asarray(raw_corners, dtype=np.float32).reshape(4, 2)
                    # cv2.aruco corners are TL/TR/BR/BL; reorder to BL/BR/TR/TL
                    corners = corners[[3, 2, 1, 0], :]
                    tag = corners
                    break

        if tag is None:
            if self.last_bev is not None:
                bev = self.last_bev.copy()
            return undistorted, bev, {"mode": "no_tag", "tag_detected": False, "detect_ms": detect_ms}

        cv2.polylines(undistorted, [tag.astype(np.int32)], True, (0, 255, 0), 2)
        cv2.putText(
            undistorted,
            f"aruco={self.config.target_id} {detect_ms:.1f}ms",
            (20, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
        )

        half = self.config.tag_size * 0.5
        dst = np.array(
            [
                self.world_to_pixel(+half, +half),
                self.world_to_pixel(-half, +half),
                self.world_to_pixel(-half, -half),
                self.world_to_pixel(+half, -half),
            ],
            dtype=np.float32,
        )
        matrix = cv2.getPerspectiveTransform(tag, dst)
        warped = cv2.warpPerspective(undistorted, matrix, (width, height))
        bev = cv2.addWeighted(bev, 0.25, warped, 0.75, 0.0)
        self._draw_grid(bev)
        self.last_bev = bev
        return undistorted, bev, {"mode": "aruco", "tag_detected": True, "detect_ms": detect_ms, "tag_id": int(self.config.target_id)}

    def world_to_pixel(self, x: float, y: float) -> Tuple[float, float]:
        width, height = self.bev_size
        u = (float(x) - self.config.x_min) / max(self.config.x_max - self.config.x_min, 1e-6) * width
        v = (self.config.y_max - float(y)) / max(self.config.y_max - self.config.y_min, 1e-6) * height
        return float(u), float(v)

    def _undistort(self, frame: np.ndarray) -> np.ndarray:
        path = self.config.calibration_path
        if not path or not os.path.exists(path):
            return frame
        if self.map1 is None or self.map2 is None:
            try:
                with open(path, "r", encoding="utf-8") as fp:
                    data = yaml.safe_load(fp)
                camera_matrix = np.asarray(data["camera_matrix"], dtype=np.float32).reshape(3, 3)
                dist_coeffs = np.asarray(data["dist_coeffs"], dtype=np.float32).reshape(-1)
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
        width, height = self.bev_size
        step = max(20, self.config.ppm // 5)
        for x in range(0, width, step):
            cv2.line(image, (x, 0), (x, height), (50, 50, 50), 1)
        for y in range(0, height, step):
            cv2.line(image, (0, y), (width, y), (50, 50, 50), 1)


class BevBackend:
    """RTSP reader backend (FFmpeg low-latency) with periodic BEV processing callbacks."""

    def __init__(
        self,
        rtsp_url: str,
        fps_limit: float,
        process_every_n: int,
        bev_config: BevConfig,
        on_raw_frame: Optional[FrameCallback] = None,
        on_bev_frame: Optional[FrameCallback] = None,
        on_meta: Optional[MetaCallback] = None,
        on_log: Optional[LogCallback] = None,
    ) -> None:
        self.rtsp_url = str(rtsp_url)
        self.fps_limit = float(fps_limit)
        self.process_every_n = max(1, int(process_every_n))
        self.processor = AprilTagBevProcessor(bev_config)
        self.on_raw_frame = on_raw_frame
        self.on_bev_frame = on_bev_frame
        self.on_meta = on_meta
        self.on_log = on_log

        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        """Start RTSP worker thread."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop RTSP worker thread."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.5)

    def _run(self) -> None:
        # Configure FFmpeg for low-latency RTSP over UDP
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
            "rtsp_transport;udp|max_delay;0|fflags;nobuffer|flags;low_delay"
        )
        cap = cv2.VideoCapture(
            self.rtsp_url,
            cv2.CAP_FFMPEG,
            [
                cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 3000,
                cv2.CAP_PROP_READ_TIMEOUT_MSEC, 1000,
                cv2.CAP_PROP_N_THREADS, 1,
            ],
        )
        if not cap.isOpened():
            self._log("ERROR", f"failed to open rtsp source: {self.rtsp_url}")
            return
        self._log("INFO", f"rtsp opened (ffmpeg): {self.rtsp_url}")

        # Background reader thread keeps grabbing latest frames
        latest: dict = {"frame": None}
        lock = threading.Lock()

        def _reader() -> None:
            while not self._stop_event.is_set():
                if not cap.grab():
                    continue
                ok, frame = cap.retrieve()
                if not ok:
                    continue
                with lock:
                    latest["frame"] = frame

        reader_thread = threading.Thread(target=_reader, daemon=True)
        reader_thread.start()

        frame_idx = 0
        target_period = 1.0 / max(1.0, self.fps_limit)
        try:
            while not self._stop_event.is_set():
                tick = time.perf_counter()
                with lock:
                    frame = None if latest["frame"] is None else latest["frame"].copy()
                if frame is None:
                    time.sleep(0.01)
                    continue

                raw, bev, meta = self.processor.process(frame)
                frame_idx += 1

                if self.on_raw_frame is not None:
                    self.on_raw_frame(raw)
                if frame_idx % self.process_every_n == 0:
                    if self.on_bev_frame is not None:
                        self.on_bev_frame(bev)
                    if self.on_meta is not None:
                        self.on_meta(meta)

                dt = time.perf_counter() - tick
                sleep_sec = target_period - dt
                if sleep_sec > 0.0:
                    time.sleep(sleep_sec)
        finally:
            cap.release()
            self._log("INFO", "rtsp stopped")

    def _log(self, level: str, text: str) -> None:
        if self.on_log is not None:
            self.on_log(level, text)
