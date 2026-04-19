"""Pure Python goal and obstacle editing model."""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from ground_station.grid_map import LineObstacle, RectObstacle, WorldPoint


class ToolMode(str, Enum):
    NONE = "none"
    SET_GOAL = "set_goal"
    DRAW_RECT = "draw_rect"
    DRAW_LINE = "draw_line"
    ERASE = "erase"


@dataclass
class MapEditor:
    goal: Optional[WorldPoint] = None
    rectangles: list[RectObstacle] = field(default_factory=list)
    lines: list[LineObstacle] = field(default_factory=list)
    tool: ToolMode = ToolMode.NONE
    locked: bool = False
    erase_radius: float = 0.08
    _pending_rect_start: Optional[WorldPoint] = None
    _active_line: Optional[LineObstacle] = None

    def set_tool(self, tool: ToolMode) -> None:
        self.tool = tool
        if tool != ToolMode.DRAW_RECT:
            self._pending_rect_start = None
        if tool != ToolMode.DRAW_LINE:
            self._active_line = None

    def left_click(self, point: WorldPoint) -> None:
        if self.locked:
            return
        if self.tool == ToolMode.SET_GOAL:
            self.goal = point
            return
        if self.tool == ToolMode.DRAW_RECT:
            self._left_click_rect(point)
            return
        if self.tool == ToolMode.DRAW_LINE:
            self._left_click_line(point)
            return
        if self.tool == ToolMode.ERASE:
            self._erase(point)

    def right_click(self, point: WorldPoint) -> None:
        if self.locked:
            return
        if self.tool == ToolMode.DRAW_LINE:
            self._active_line = None

    def _left_click_rect(self, point: WorldPoint) -> None:
        if self._pending_rect_start is None:
            self._pending_rect_start = point
            return
        self.rectangles.append(RectObstacle(*self._pending_rect_start, *point))
        self._pending_rect_start = None

    def _left_click_line(self, point: WorldPoint) -> None:
        if self._active_line is None:
            self._active_line = LineObstacle(points=[point])
            self.lines.append(self._active_line)
            return
        self._active_line.points.append(point)

    def _erase(self, point: WorldPoint) -> None:
        if self.goal is not None and self._point_distance(point, self.goal) <= self.erase_radius:
            self.goal = None
            return

        for idx, rect in enumerate(self.rectangles):
            norm = rect.normalized()
            if norm.x0 <= point[0] <= norm.x1 and norm.y0 <= point[1] <= norm.y1:
                del self.rectangles[idx]
                return

        for idx, line in enumerate(self.lines):
            if self._line_distance(point, line) <= self.erase_radius:
                del self.lines[idx]
                if self._active_line is line:
                    self._active_line = None
                return

    @staticmethod
    def _point_distance(a: WorldPoint, b: WorldPoint) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _line_distance(self, point: WorldPoint, line: LineObstacle) -> float:
        if len(line.points) < 2:
            return self._point_distance(point, line.points[0])
        return min(self._segment_distance(point, start, end) for start, end in zip(line.points, line.points[1:]))

    @staticmethod
    def _segment_distance(point: WorldPoint, start: WorldPoint, end: WorldPoint) -> float:
        sx, sy = start
        ex, ey = end
        px, py = point
        dx = ex - sx
        dy = ey - sy
        if dx == 0.0 and dy == 0.0:
            return math.hypot(px - sx, py - sy)
        t = ((px - sx) * dx + (py - sy) * dy) / (dx * dx + dy * dy)
        t = max(0.0, min(1.0, t))
        cx = sx + t * dx
        cy = sy + t * dy
        return math.hypot(px - cx, py - cy)
