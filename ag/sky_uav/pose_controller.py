"""Sky-side pose simulation, path smoothing, and cmd generation."""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from common.protocol import clamp, wrap_angle


Point2D = Tuple[float, float]
CmdCallback = Callable[[Dict[str, float], Dict[str, object]], None]


@dataclass
class ControllerConfig:
    """Controller parameters."""

    control_hz: float = 20.0
    vx_limit: float = 0.35
    vy_limit: float = 0.00
    wz_limit: float = 0.8
    lookahead_distance: float = 0.35
    goal_tolerance: float = 0.15
    slowdown_radius: float = 0.6
    target_speed: float = 0.2
    k_yaw: float = 1.8
    k_speed: float = 0.9
    resample_step: float = 0.08
    spline_samples_per_segment: int = 16
    initial_x: float = 0.0
    initial_y: float = 0.0
    initial_yaw: float = 0.0


class PoseController:
    """Maintain virtual odom pose and output cmd_vel to track a path."""

    def __init__(self, config: Dict[str, object], on_command: Optional[CmdCallback] = None) -> None:
        cfg = ControllerConfig(**{**ControllerConfig().__dict__, **config})
        self.cfg = cfg
        self.on_command = on_command

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self._pose = {"x": cfg.initial_x, "y": cfg.initial_y, "yaw": cfg.initial_yaw}
        self._cmd = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
        self._target_index = 0
        self._goal_reached = False
        self._path_seq = 0
        self._target_speed = cfg.target_speed
        self._raw_path: List[Point2D] = []
        self._smooth_path: List[Point2D] = []
        self._last_control_ts = time.monotonic()

    def start(self) -> None:
        """Start background control loop."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._last_control_ts = time.monotonic()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop background control loop."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def set_path(self, seq: int, points: Sequence[Sequence[float]], target_speed: float) -> List[Point2D]:
        """Set a new path and return the smoothed/resampled polyline."""
        normalized = [(float(p[0]), float(p[1])) for p in points]
        if len(normalized) < 2:
            raise ValueError("path requires at least 2 points")
        smooth = smooth_path(normalized, self.cfg.spline_samples_per_segment)
        sampled = resample_polyline(smooth, self.cfg.resample_step)
        with self._lock:
            self._path_seq = int(seq)
            self._target_speed = float(target_speed)
            self._raw_path = normalized
            self._smooth_path = sampled
            self._target_index = 0
            self._goal_reached = False
        return sampled

    def step_once(self, dt: float) -> Dict[str, object]:
        """Run one control step (used by tests and the background loop)."""
        dt = max(0.0, min(float(dt), 0.2))
        with self._lock:
            self._integrate_odom(dt)
            self._cmd = self._compute_cmd()
            snapshot = self._build_snapshot_locked()
        if self.on_command is not None:
            self.on_command(dict(snapshot["cmd"]), snapshot)
        return snapshot

    def get_snapshot(self) -> Dict[str, object]:
        """Return thread-safe state snapshot for telemetry."""
        with self._lock:
            return self._build_snapshot_locked()

    def _run_loop(self) -> None:
        period = 1.0 / max(self.cfg.control_hz, 1.0)
        while not self._stop_event.is_set():
            now = time.monotonic()
            dt = now - self._last_control_ts
            self._last_control_ts = now
            self.step_once(dt)
            elapsed = time.monotonic() - now
            sleep_sec = period - elapsed
            if sleep_sec > 0.0:
                time.sleep(sleep_sec)

    def _integrate_odom(self, dt: float) -> None:
        if dt <= 0.0:
            return
        yaw = float(self._pose["yaw"])
        vx = float(self._cmd["vx"])
        vy = float(self._cmd["vy"])
        wz = float(self._cmd["wz"])
        self._pose["x"] += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
        self._pose["y"] += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
        self._pose["yaw"] = wrap_angle(yaw + wz * dt)

    def _compute_cmd(self) -> Dict[str, float]:
        if len(self._smooth_path) < 2:
            self._goal_reached = False
            return {"vx": 0.0, "vy": 0.0, "wz": 0.0}

        pose_x = float(self._pose["x"])
        pose_y = float(self._pose["y"])
        pose_yaw = float(self._pose["yaw"])
        goal_x, goal_y = self._smooth_path[-1]
        goal_dist = math.hypot(goal_x - pose_x, goal_y - pose_y)

        if goal_dist <= self.cfg.goal_tolerance:
            self._goal_reached = True
            self._target_index = len(self._smooth_path) - 1
            return {"vx": 0.0, "vy": 0.0, "wz": 0.0}

        self._goal_reached = False
        closest_idx = min(
            range(len(self._smooth_path)),
            key=lambda idx: (self._smooth_path[idx][0] - pose_x) ** 2 + (self._smooth_path[idx][1] - pose_y) ** 2,
        )
        lookahead_idx = len(self._smooth_path) - 1
        for idx in range(closest_idx, len(self._smooth_path)):
            px, py = self._smooth_path[idx]
            if math.hypot(px - pose_x, py - pose_y) >= self.cfg.lookahead_distance:
                lookahead_idx = idx
                break
        self._target_index = max(self._target_index, lookahead_idx)
        tx, ty = self._smooth_path[self._target_index]

        dx = tx - pose_x
        dy = ty - pose_y
        dist = max(1e-6, math.hypot(dx, dy))
        heading = math.atan2(dy, dx)
        heading_error = wrap_angle(heading - pose_yaw)

        speed = min(self._target_speed, self.cfg.k_speed * dist)
        if goal_dist < self.cfg.slowdown_radius:
            speed *= clamp(goal_dist / max(self.cfg.slowdown_radius, 1e-6), 0.15, 1.0)

        return {
            "vx": clamp(speed, -self.cfg.vx_limit, self.cfg.vx_limit),
            "vy": clamp(0.0, -self.cfg.vy_limit, self.cfg.vy_limit),
            "wz": clamp(self.cfg.k_yaw * heading_error, -self.cfg.wz_limit, self.cfg.wz_limit),
        }

    def _build_snapshot_locked(self) -> Dict[str, object]:
        return {
            "path_seq": self._path_seq,
            "pose": {
                "x": float(self._pose["x"]),
                "y": float(self._pose["y"]),
                "yaw": float(self._pose["yaw"]),
            },
            "cmd": {
                "vx": float(self._cmd["vx"]),
                "vy": float(self._cmd["vy"]),
                "wz": float(self._cmd["wz"]),
            },
            "target_index": int(self._target_index),
            "goal_reached": bool(self._goal_reached),
            "raw_path_points": len(self._raw_path),
            "smooth_path_points": len(self._smooth_path),
        }


def smooth_path(points: Sequence[Point2D], samples_per_segment: int = 16) -> List[Point2D]:
    """Smooth polyline with lightweight cubic Catmull-Rom interpolation."""
    pts = list(points)
    if len(pts) < 3:
        return pts
    samples_per_segment = max(4, int(samples_per_segment))

    dense: List[Point2D] = []
    for i in range(len(pts) - 1):
        p0 = pts[max(0, i - 1)]
        p1 = pts[i]
        p2 = pts[i + 1]
        p3 = pts[min(len(pts) - 1, i + 2)]
        steps = samples_per_segment if i < len(pts) - 2 else samples_per_segment + 1
        for j in range(steps):
            t = j / float(samples_per_segment)
            if i > 0 and j == 0:
                continue
            dense.append(_catmull_rom(p0, p1, p2, p3, t))
    dense[0] = pts[0]
    dense[-1] = pts[-1]
    return dense


def resample_polyline(points: Sequence[Point2D], step: float) -> List[Point2D]:
    """Resample points at approximately fixed arc-length spacing."""
    pts = list(points)
    if len(pts) < 2:
        return pts
    step = max(0.02, float(step))
    dists = [0.0]
    for i in range(1, len(pts)):
        seg = math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
        dists.append(dists[-1] + seg)
    total = dists[-1]
    if total <= 1e-6:
        return [pts[0], pts[-1]]

    result: List[Point2D] = [pts[0]]
    target = step
    while target < total:
        result.append(_sample_at_distance(pts, dists, target))
        target += step
    result.append(pts[-1])
    return result


def _sample_at_distance(points: Sequence[Point2D], dists: Sequence[float], target: float) -> Point2D:
    for i in range(1, len(points)):
        if dists[i] >= target:
            d0 = dists[i - 1]
            d1 = dists[i]
            if d1 <= d0 + 1e-9:
                return points[i]
            ratio = (target - d0) / (d1 - d0)
            x = points[i - 1][0] + ratio * (points[i][0] - points[i - 1][0])
            y = points[i - 1][1] + ratio * (points[i][1] - points[i - 1][1])
            return (x, y)
    return points[-1]


def _catmull_rom(p0: Point2D, p1: Point2D, p2: Point2D, p3: Point2D, t: float) -> Point2D:
    t2 = t * t
    t3 = t2 * t
    x = 0.5 * (
        (2.0 * p1[0])
        + (-p0[0] + p2[0]) * t
        + (2.0 * p0[0] - 5.0 * p1[0] + 4.0 * p2[0] - p3[0]) * t2
        + (-p0[0] + 3.0 * p1[0] - 3.0 * p2[0] + p3[0]) * t3
    )
    y = 0.5 * (
        (2.0 * p1[1])
        + (-p0[1] + p2[1]) * t
        + (2.0 * p0[1] - 5.0 * p1[1] + 4.0 * p2[1] - p3[1]) * t2
        + (-p0[1] + 3.0 * p1[1] - 3.0 * p2[1] + p3[1]) * t3
    )
    return (x, y)
