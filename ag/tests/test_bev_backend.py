"""Tests for the RTSP/BEV backend hot paths."""
from __future__ import annotations

import time
import unittest
from unittest.mock import patch

import numpy as np

from ground_station.bev_backend import BevBackend, BevConfig, _limit_opencv_threads


class _FakeProcessor:
    def __init__(self) -> None:
        self.calls = 0
        self.prepare_calls = 0
        self.frame_is_undistorted_flags: list[bool] = []

    def prepare_raw_frame(self, frame: np.ndarray) -> np.ndarray:
        self.prepare_calls += 1
        return frame + 10

    def process(self, frame: np.ndarray, *, frame_is_undistorted: bool = False):
        self.calls += 1
        self.frame_is_undistorted_flags.append(frame_is_undistorted)
        return frame + 20, frame + 30, {"mode": "fake", "detect_ms": 1.0}


class _FakeCapture:
    def __init__(self, backend: BevBackend, frames_before_stop: int) -> None:
        self.backend = backend
        self.frames_before_stop = frames_before_stop
        self.read_calls = 0

    def read(self):
        self.read_calls += 1
        frame = np.full((8, 8, 3), self.read_calls, dtype=np.uint8)
        if self.read_calls >= self.frames_before_stop:
            self.backend._stop_event.set()
        return True, frame


class BevBackendTests(unittest.TestCase):
    def test_limit_opencv_threads_caps_opencv_parallelism(self) -> None:
        with (
            patch("ground_station.bev_backend.cv2.getNumThreads", return_value=16),
            patch("ground_station.bev_backend.cv2.setNumThreads") as set_num_threads,
        ):
            chosen = _limit_opencv_threads(3)

        self.assertEqual(chosen, 3)
        set_num_threads.assert_called_once_with(3)

    def test_run_capture_session_only_processes_every_nth_frame(self) -> None:
        raw_frames: list[np.ndarray] = []
        bev_frames: list[np.ndarray] = []
        metas: list[dict[str, object]] = []

        backend = BevBackend(
            rtsp_url="rtsp://example.invalid/test",
            fps_limit=120.0,
            process_every_n=3,
            bev_config=BevConfig(enabled=False),
            on_raw_frame=raw_frames.append,
            on_bev_frame=bev_frames.append,
            on_meta=metas.append,
        )
        backend.processor = _FakeProcessor()
        cap = _FakeCapture(backend, frames_before_stop=5)

        backend._run_capture_session(cap)

        self.assertEqual(cap.read_calls, 5)
        self.assertEqual(len(raw_frames), 5)
        self.assertEqual(backend.processor.calls, 1)
        self.assertEqual(len(bev_frames), 1)
        self.assertEqual(len(metas), 1)

    def test_run_capture_session_keeps_raw_view_transform_consistent(self) -> None:
        raw_frames: list[np.ndarray] = []

        backend = BevBackend(
            rtsp_url="rtsp://example.invalid/test",
            fps_limit=120.0,
            process_every_n=3,
            bev_config=BevConfig(enabled=False),
            on_raw_frame=raw_frames.append,
        )
        backend.processor = _FakeProcessor()
        cap = _FakeCapture(backend, frames_before_stop=5)

        backend._run_capture_session(cap)

        self.assertEqual(backend.processor.prepare_calls, 5)
        self.assertEqual(backend.processor.calls, 1)
        self.assertEqual(backend.processor.frame_is_undistorted_flags, [True])
        self.assertEqual(int(raw_frames[0][0, 0, 0]), 11)
        self.assertEqual(int(raw_frames[1][0, 0, 0]), 12)
        self.assertEqual(int(raw_frames[2][0, 0, 0]), 13)

    def test_run_capture_session_respects_fps_limit_sleep(self) -> None:
        backend = BevBackend(
            rtsp_url="rtsp://example.invalid/test",
            fps_limit=20.0,
            process_every_n=10,
            bev_config=BevConfig(enabled=False),
        )
        backend.processor = _FakeProcessor()
        cap = _FakeCapture(backend, frames_before_stop=3)

        started = time.perf_counter()
        backend._run_capture_session(cap)
        elapsed = time.perf_counter() - started

        self.assertGreaterEqual(elapsed, 0.10)


if __name__ == "__main__":
    unittest.main()
