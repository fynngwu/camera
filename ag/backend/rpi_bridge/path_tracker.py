"""Simple geometric path tracker for the Raspberry Pi bridge."""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

from backend.shared.protocol import clamp

Point2D = Tuple[float, float]


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class PathTracker:
    """Compute planar velocity commands from a path and a 2D pose."""

    def __init__(self, config: Dict[str, float]) -> None:
        self.lookahead_distance = float(config.get("lookahead_distance", 0.5))
        self.k_yaw = float(config.get("k_yaw", 1.5))
        self.goal_tolerance = float(config.get("goal_tolerance", 0.2))
        self.target_speed = float(config.get("target_speed", 0.2))
        self.slowdown_radius = float(config.get("slowdown_radius", 0.6))

    def compute_command(
        self,
        path_points: Sequence[Sequence[float]],
        pose_x: float,
        pose_y: float,
        pose_yaw: float,
        vx_limit: float,
        wz_limit: float,
        target_speed_override: Optional[float] = None,
    ) -> Dict[str, float]:
        """Compute one cmd_vel command from path and pose."""
        if len(path_points) < 2:
            return {"vx": 0.0, "vy": 0.0, "wz": 0.0, "goal_reached": True}
        points = [(float(p[0]), float(p[1])) for p in path_points]
        goal = points[-1]
        goal_dist = math.hypot(goal[0] - pose_x, goal[1] - pose_y)
        if goal_dist <= self.goal_tolerance:
            return {"vx": 0.0, "vy": 0.0, "wz": 0.0, "goal_reached": True}
        lookahead = self._select_lookahead_point(points, pose_x, pose_y)
        dx = lookahead[0] - pose_x
        dy = lookahead[1] - pose_y
        target_heading = math.atan2(dy, dx)
        heading_error = wrap_angle(target_heading - pose_yaw)
        speed = float(target_speed_override) if target_speed_override is not None else self.target_speed
        if goal_dist < self.slowdown_radius:
            speed *= clamp(goal_dist / max(self.slowdown_radius, 1e-6), 0.15, 1.0)
        return {
            "vx": clamp(speed, -vx_limit, vx_limit),
            "vy": 0.0,
            "wz": clamp(self.k_yaw * heading_error, -wz_limit, wz_limit),
            "goal_reached": False,
        }

    def _select_lookahead_point(self, points: List[Point2D], pose_x: float, pose_y: float) -> Point2D:
        """Select a geometric lookahead point from the polyline."""
        dists = [math.hypot(px - pose_x, py - pose_y) for px, py in points]
        closest_idx = min(range(len(dists)), key=dists.__getitem__)
        for idx in range(closest_idx, len(points)):
            if dists[idx] >= self.lookahead_distance:
                return points[idx]
        return points[-1]
