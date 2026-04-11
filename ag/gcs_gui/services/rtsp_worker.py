"""Background RTSP reader worker with optional BEV processing."""
from __future__ import annotations

import threading
import time
from typing import Optional

import cv2
import numpy as np

from gcs_gui.services.bev_processor import BevProcessor
from gcs_gui.qt_compat import QObject, Signal, Slot


class RtspWorker(QObject):
    """Read RTSP frames in a background thread and emit raw / BEV frames."""

    rawFrameReady = Signal(object)
    bevFrameReady = Signal(object)
    statsUpdated = Signal(object)
    connectionChanged = Signal(bool)
    logLine = Signal(str, str)
    bevMetaUpdated = Signal(object)

    def __init__(self, url: str, pipeline: str, fps_limit: float, bev_processor: Optional[BevProcessor] = None, process_every_n: int = 2) -> None:
        super().__init__()
        self.url = url
        self.pipeline = pipeline
        self.fps_limit = float(fps_limit)
        self.bev_processor = bev_processor
        self.process_every_n = max(1, int(process_every_n))
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

    @Slot()
    def start(self) -> None:
        """Start the RTSP reader thread."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    @Slot()
    def stop(self) -> None:
        """Stop the RTSP reader thread."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def _run(self) -> None:
        """Read frames, optionally process them, and emit Qt signals."""
        source = self.pipeline.strip() or self._build_pipeline(self.url)
        cap = cv2.VideoCapture(source, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            cap = cv2.VideoCapture(self.url)
        if not cap.isOpened():
            self.connectionChanged.emit(False)
            self.logLine.emit("ERROR", f"failed to open RTSP source: {self.url}")
            return
        self.connectionChanged.emit(True)
        self.logLine.emit("INFO", f"RTSP stream opened: {self.url}")
        frame_idx = 0
        stat_t0 = time.perf_counter()
        frame_count = 0
        bev_count = 0
        target_period = 1.0 / max(self.fps_limit, 1.0)
        try:
            while not self._stop_event.is_set():
                t_loop = time.perf_counter()
                ok, frame = cap.read()
                if not ok:
                    self.logLine.emit("WARN", "RTSP frame read failed")
                    time.sleep(0.05)
                    continue
                frame_idx += 1
                frame_count += 1
                # 每一帧都经过处理（去畸变），但 BEV 画面每隔 N 帧更新
                if self.bev_processor is not None:
                    raw_vis, bev_vis, meta = self.bev_processor.process(frame)
                    self.rawFrameReady.emit(raw_vis)
                    if frame_idx % self.process_every_n == 0:
                        self.bevFrameReady.emit(bev_vis)
                        self.bevMetaUpdated.emit(meta)
                        bev_count += 1
                else:
                    self.rawFrameReady.emit(frame)
                now = time.perf_counter()
                if now - stat_t0 >= 1.0:
                    self.statsUpdated.emit({
                        "read_fps": frame_count / (now - stat_t0),
                        "bev_fps": bev_count / (now - stat_t0),
                    })
                    stat_t0 = now
                    frame_count = 0
                    bev_count = 0
                sleep_time = target_period - (time.perf_counter() - t_loop)
                if sleep_time > 0.0:
                    time.sleep(sleep_time)
        finally:
            cap.release()
            self.connectionChanged.emit(False)
            self.logLine.emit("INFO", "RTSP stream stopped")

    @staticmethod
    def _build_pipeline(url: str) -> str:
        """Build a default GStreamer pipeline for an RTSP URL."""
        return (
            f"rtspsrc location={url} latency=0 ! "
            "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
            "appsink drop=true max-buffers=1 sync=false"
        )
