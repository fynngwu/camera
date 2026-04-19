"""Tests for shared grid/world/pixel mapping and obstacle rasterization."""
from __future__ import annotations

import unittest

from ground_station.grid_map import GridConfig, GridMapper, LineObstacle, RectObstacle


class GridMapTests(unittest.TestCase):
    def setUp(self) -> None:
        self.mapper = GridMapper(
            GridConfig(
                x_min=-1.0,
                x_max=1.0,
                y_min=-1.0,
                y_max=1.0,
                resolution_m=0.1,
                ppm=100,
                line_obstacle_width_m=0.2,
            )
        )

    def test_world_grid_round_trip_stays_in_same_cell(self) -> None:
        cell = self.mapper.world_to_grid(0.24, -0.31)
        x_world, y_world = self.mapper.grid_to_world(*cell)
        self.assertEqual(self.mapper.world_to_grid(x_world, y_world), cell)

    def test_world_pixel_round_trip_stays_close(self) -> None:
        pixel = self.mapper.world_to_pixel(0.35, -0.25)
        point = self.mapper.pixel_to_world(*pixel)
        self.assertAlmostEqual(point[0], 0.35, delta=0.02)
        self.assertAlmostEqual(point[1], -0.25, delta=0.02)

    def test_rect_obstacle_marks_cells(self) -> None:
        occupancy = self.mapper.build_occupancy(
            rects=[RectObstacle(0.0, 0.0, 0.2, 0.2)],
            lines=[],
        )
        self.assertTrue(any(any(row) for row in occupancy))

    def test_line_obstacle_marks_cells(self) -> None:
        occupancy = self.mapper.build_occupancy(
            rects=[],
            lines=[LineObstacle(points=[(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5)])],
        )
        self.assertTrue(any(any(row) for row in occupancy))


if __name__ == "__main__":
    unittest.main()
