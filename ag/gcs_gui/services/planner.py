"""Simple A* planner for GUI obstacle drawing and path generation."""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

from gcs_gui.app_state import ObstacleRect

GridIndex = Tuple[int, int]
WorldPoint = Tuple[float, float]


@dataclass
class PlannerConfig:
    """Planner grid configuration."""
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    resolution_m: float
    robot_radius_m: float


class AStarPlanner:
    """2D grid A* planner working on axis-aligned rectangular obstacles."""

    def __init__(self, config: PlannerConfig) -> None:
        self.config = config
        self.width = max(2, int(math.ceil((config.x_max - config.x_min) / config.resolution_m)))
        self.height = max(2, int(math.ceil((config.y_max - config.y_min) / config.resolution_m)))

    def plan(self, start: WorldPoint, goals: Sequence[WorldPoint], obstacles: Sequence[ObstacleRect]) -> List[WorldPoint]:
        """Plan a polyline that visits all goals sequentially."""
        if not goals:
            return [start]
        occupancy = self._build_occupancy(obstacles)
        total_path: List[WorldPoint] = [start]
        current = start
        for goal in goals:
            segment = self._plan_segment(current, goal, occupancy)
            if len(segment) <= 1:
                total_path.append(goal)
            else:
                total_path.extend(segment[1:])
            current = goal
        return self._simplify(total_path)

    def _plan_segment(self, start: WorldPoint, goal: WorldPoint, occupancy: List[List[bool]]) -> List[WorldPoint]:
        """Plan one A* segment between two world points."""
        s = self._world_to_grid(*start)
        g = self._world_to_grid(*goal)
        if self._is_blocked(s, occupancy):
            raise ValueError("start cell is blocked")
        if self._is_blocked(g, occupancy):
            raise ValueError("goal cell is blocked")
        frontier: List[Tuple[float, GridIndex]] = []
        heapq.heappush(frontier, (0.0, s))
        came_from: Dict[GridIndex, Optional[GridIndex]] = {s: None}
        cost_so_far: Dict[GridIndex, float] = {s: 0.0}
        while frontier:
            _, current = heapq.heappop(frontier)
            if current == g:
                break
            for nxt, step_cost in self._neighbors(current, occupancy):
                new_cost = cost_so_far[current] + step_cost
                if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                    cost_so_far[nxt] = new_cost
                    priority = new_cost + self._heuristic(nxt, g)
                    heapq.heappush(frontier, (priority, nxt))
                    came_from[nxt] = current
        if g not in came_from:
            raise ValueError("planner failed to find a path")
        path_grid: List[GridIndex] = []
        cur: Optional[GridIndex] = g
        while cur is not None:
            path_grid.append(cur)
            cur = came_from[cur]
        path_grid.reverse()
        return [self._grid_to_world(ix, iy) for ix, iy in path_grid]

    def _build_occupancy(self, obstacles: Sequence[ObstacleRect]) -> List[List[bool]]:
        """Rasterize obstacles into an inflated occupancy grid."""
        grid = [[False for _ in range(self.width)] for _ in range(self.height)]
        inflate = max(0, int(math.ceil(self.config.robot_radius_m / self.config.resolution_m)))
        for obs in obstacles:
            rect = obs.normalized()
            x0, y0 = self._world_to_grid(rect.x_min, rect.y_min)
            x1, y1 = self._world_to_grid(rect.x_max, rect.y_max)
            for iy in range(max(0, min(y0, y1) - inflate), min(self.height, max(y0, y1) + inflate + 1)):
                for ix in range(max(0, min(x0, x1) - inflate), min(self.width, max(x0, x1) + inflate + 1)):
                    grid[iy][ix] = True
        return grid

    def _neighbors(self, node: GridIndex, occupancy: List[List[bool]]) -> List[Tuple[GridIndex, float]]:
        """Return 8-connected free neighbors."""
        x, y = node
        result: List[Tuple[GridIndex, float]] = []
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.width and 0 <= ny < self.height and not occupancy[ny][nx]:
                    result.append(((nx, ny), math.hypot(dx, dy)))
        return result

    def _world_to_grid(self, x: float, y: float) -> GridIndex:
        """Convert world coordinates to grid indices."""
        ix = int((x - self.config.x_min) / self.config.resolution_m)
        iy = int((y - self.config.y_min) / self.config.resolution_m)
        ix = max(0, min(self.width - 1, ix))
        iy = max(0, min(self.height - 1, iy))
        return ix, iy

    def _grid_to_world(self, ix: int, iy: int) -> WorldPoint:
        """Convert a grid cell index to its world-center point."""
        x = self.config.x_min + (ix + 0.5) * self.config.resolution_m
        y = self.config.y_min + (iy + 0.5) * self.config.resolution_m
        return x, y

    def _is_blocked(self, node: GridIndex, occupancy: List[List[bool]]) -> bool:
        """Return whether a node is occupied."""
        x, y = node
        return occupancy[y][x]

    @staticmethod
    def _heuristic(a: GridIndex, b: GridIndex) -> float:
        """Euclidean heuristic."""
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def _simplify(points: Sequence[WorldPoint], epsilon: float = 0.03) -> List[WorldPoint]:
        """Simplify a polyline with a lightweight RDP-like recursion."""
        pts = list(points)
        if len(pts) <= 2:
            return pts
        start = pts[0]
        end = pts[-1]
        max_dist = -1.0
        max_idx = -1
        for idx in range(1, len(pts) - 1):
            dist = _point_to_segment_distance(pts[idx], start, end)
            if dist > max_dist:
                max_dist = dist
                max_idx = idx
        if max_dist <= epsilon:
            return [start, end]
        left = AStarPlanner._simplify(pts[: max_idx + 1], epsilon)
        right = AStarPlanner._simplify(pts[max_idx:], epsilon)
        return left[:-1] + right


def _point_to_segment_distance(p: WorldPoint, a: WorldPoint, b: WorldPoint) -> float:
    """Return distance from a point to a line segment."""
    ax, ay = a
    bx, by = b
    px, py = p
    dx = bx - ax
    dy = by - ay
    if dx == 0.0 and dy == 0.0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    cx = ax + t * dx
    cy = ay + t * dy
    return math.hypot(px - cx, py - cy)
