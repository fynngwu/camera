"""Unit tests for ground-station A* planning behavior."""
from __future__ import annotations

import unittest

from ground_station.planning_backend import AStarPlanner, ObstacleRect, PlannerConfig


class PlanningBackendTests(unittest.TestCase):
    """Keep planning outputs intuitive and waypoint-faithful."""

    def _build_planner(self) -> AStarPlanner:
        return AStarPlanner(
            PlannerConfig(
                x_min=-1.0,
                x_max=1.0,
                y_min=-1.0,
                y_max=1.0,
                resolution_m=0.1,
                robot_radius_m=0.0,
            )
        )

    def test_no_obstacles_prefers_direct_straight_segment(self) -> None:
        planner = self._build_planner()
        start = (-0.83, -0.41)
        goal = (0.74, 0.58)

        path = planner.plan(start=start, goals=[goal], obstacles=[])

        self.assertEqual(path, [start, goal])

    def test_preserves_exact_requested_waypoints(self) -> None:
        planner = self._build_planner()
        start = (-0.92, -0.88)
        goals = [(-0.37, -0.11), (0.23, 0.49), (0.81, 0.77)]
        obstacles = [ObstacleRect(x_min=-0.10, y_min=0.00, x_max=0.35, y_max=0.45)]

        path = planner.plan(start=start, goals=goals, obstacles=obstacles)

        self.assertEqual(path[0], start)
        self.assertEqual(path[-1], goals[-1])
        cursor = 0
        for goal in goals:
            cursor = path.index(goal, cursor) + 1

    def test_rejects_goal_that_is_truly_inside_obstacle(self) -> None:
        planner = self._build_planner()
        obstacle = ObstacleRect(x_min=0.10, y_min=0.10, x_max=0.40, y_max=0.40)

        with self.assertRaisesRegex(ValueError, "goal point is inside obstacle"):
            planner.plan(start=(0.0, 0.0), goals=[(0.20, 0.20)], obstacles=[obstacle])


if __name__ == "__main__":
    unittest.main()
