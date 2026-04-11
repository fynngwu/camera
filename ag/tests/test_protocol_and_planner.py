"""Basic smoke tests for the protocol helpers and planner."""
from __future__ import annotations

import unittest

from backend.shared.protocol import decode_message, encode_message, make_message
from gcs_gui.app_state import ObstacleRect
from gcs_gui.services.planner import AStarPlanner, PlannerConfig


class ProtocolTests(unittest.TestCase):
    """Protocol encode / decode tests."""

    def test_roundtrip(self) -> None:
        """Encoding then decoding should preserve key fields."""
        msg = make_message("cmd_vel", vx=0.1, vy=0.0, wz=0.2)
        obj = decode_message(encode_message(msg))
        self.assertEqual(obj["type"], "cmd_vel")
        self.assertAlmostEqual(obj["vx"], 0.1)
        self.assertAlmostEqual(obj["wz"], 0.2)


class PlannerTests(unittest.TestCase):
    """Planner smoke tests."""

    def setUp(self) -> None:
        self.planner = AStarPlanner(
            PlannerConfig(
                x_min=-1.0,
                x_max=1.0,
                y_min=-1.0,
                y_max=1.0,
                resolution_m=0.05,
                robot_radius_m=0.05,
            )
        )

    def test_plan_without_obstacles(self) -> None:
        """Planner should produce a path to the goal."""
        path = self.planner.plan((0.0, 0.0), [(0.5, 0.5)], [])
        self.assertGreaterEqual(len(path), 2)
        self.assertAlmostEqual(path[-1][0], 0.5, delta=0.1)
        self.assertAlmostEqual(path[-1][1], 0.5, delta=0.1)

    def test_plan_with_obstacle(self) -> None:
        """Planner should still route around a central rectangle."""
        obstacle = ObstacleRect(-0.1, -0.1, 0.1, 0.1)
        path = self.planner.plan((-0.5, 0.0), [(0.5, 0.0)], [obstacle])
        self.assertGreaterEqual(len(path), 2)
        self.assertAlmostEqual(path[-1][0], 0.5, delta=0.1)


if __name__ == "__main__":
    unittest.main()
