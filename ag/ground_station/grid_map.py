"""Shared world/grid/pixel mapping and obstacle rasterization."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence


WorldPoint = tuple[float, float]


@dataclass(frozen=True)
class RectObstacle:
    x0: float
    y0: float
    x1: float
    y1: float

    def normalized(self) -> "RectObstacle":
        return RectObstacle(
            min(self.x0, self.x1),
            min(self.y0, self.y1),
            max(self.x0, self.x1),
            max(self.y0, self.y1),
        )


@dataclass
class LineObstacle:
    points: list[WorldPoint]


@dataclass(frozen=True)
class GridConfig:
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    resolution_m: float
    ppm: int
    line_obstacle_width_m: float


class GridMapper:
    def __init__(self, cfg: GridConfig) -> None:
        self.cfg = cfg
        self.width = max(1, int(math.ceil((cfg.x_max - cfg.x_min) / cfg.resolution_m)))
        self.height = max(1, int(math.ceil((cfg.y_max - cfg.y_min) / cfg.resolution_m)))
        self.width_px = max(1, int(round((cfg.x_max - cfg.x_min) * cfg.ppm)))
        self.height_px = max(1, int(round((cfg.y_max - cfg.y_min) * cfg.ppm)))

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        ix = int((float(x) - self.cfg.x_min) / self.cfg.resolution_m)
        iy = int((float(y) - self.cfg.y_min) / self.cfg.resolution_m)
        return self._clamp_cell(ix, iy)

    def grid_to_world(self, ix: int, iy: int) -> WorldPoint:
        return (
            self.cfg.x_min + (int(ix) + 0.5) * self.cfg.resolution_m,
            self.cfg.y_min + (int(iy) + 0.5) * self.cfg.resolution_m,
        )

    def world_to_pixel(self, x: float, y: float) -> tuple[float, float]:
        u = (float(x) - self.cfg.x_min) / (self.cfg.x_max - self.cfg.x_min) * self.width_px
        v = (self.cfg.y_max - float(y)) / (self.cfg.y_max - self.cfg.y_min) * self.height_px
        return float(u), float(v)

    def pixel_to_world(self, u: float, v: float) -> WorldPoint:
        return (
            self.cfg.x_min + float(u) / self.width_px * (self.cfg.x_max - self.cfg.x_min),
            self.cfg.y_max - float(v) / self.height_px * (self.cfg.y_max - self.cfg.y_min),
        )

    def build_occupancy(
        self,
        rects: Sequence[RectObstacle],
        lines: Sequence[LineObstacle],
    ) -> list[list[bool]]:
        grid = [[False for _ in range(self.width)] for _ in range(self.height)]
        for rect in rects:
            norm = rect.normalized()
            x0, y0 = self.world_to_grid(norm.x0, norm.y0)
            x1, y1 = self.world_to_grid(norm.x1, norm.y1)
            for iy in range(min(y0, y1), max(y0, y1) + 1):
                for ix in range(min(x0, x1), max(x0, x1) + 1):
                    grid[iy][ix] = True

        inflate = max(0, int(math.ceil(self.cfg.line_obstacle_width_m / self.cfg.resolution_m / 2.0)))
        for line in lines:
            for start, end in zip(line.points, line.points[1:]):
                for ix, iy in self._segment_cells(start, end):
                    for dy in range(-inflate, inflate + 1):
                        for dx in range(-inflate, inflate + 1):
                            nx, ny = self._clamp_cell(ix + dx, iy + dy)
                            grid[ny][nx] = True
        return grid

    def _segment_cells(self, start: WorldPoint, end: WorldPoint) -> Iterable[tuple[int, int]]:
        x0, y0 = self.world_to_grid(*start)
        x1, y1 = self.world_to_grid(*end)
        steps = max(abs(x1 - x0), abs(y1 - y0), 1)
        for idx in range(steps + 1):
            t = idx / steps
            yield self._clamp_cell(
                int(round(x0 + (x1 - x0) * t)),
                int(round(y0 + (y1 - y0) * t)),
            )

    def _clamp_cell(self, ix: int, iy: int) -> tuple[int, int]:
        return (
            max(0, min(self.width - 1, int(ix))),
            max(0, min(self.height - 1, int(iy))),
        )
