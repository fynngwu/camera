"""Tests for the shared planner/controller backend and local simulator robot."""
from __future__ import annotations

import time
import unittest

from common.air_ground_bridge import Pose2D
from ground_robot.local_sim_client import LocalSimRobotClient
from ground_station.grid_map import GridConfig, GridMapper, RectObstacle
from ground_station.navigation_backend import AStarPlanner, GroundStationRuntime, PurePursuitController


class NavigationBackendTests(unittest.TestCase):
    def setUp(self) -> None:
        self.mapper = GridMapper(
            GridConfig(
                x_min=-1.0,
                x_max=1.0,
                y_min=-1.0,
                y_max=1.0,
                resolution_m=0.05,
                ppm=200,
                line_obstacle_width_m=0.08,
            )
        )

    def test_astar_detours_around_rectangle(self) -> None:
        planner = AStarPlanner(self.mapper)
        path = planner.plan(
            start=(0.0, 0.0),
            goal=(0.8, 0.0),
            rects=[RectObstacle(0.25, -0.1, 0.55, 0.1)],
            lines=[],
        )

        self.assertEqual(path[0], (0.0, 0.0))
        self.assertEqual(path[-1], (0.8, 0.0))
        self.assertGreater(len(path), 2)
        self.assertTrue(any(abs(point[1]) > 0.08 for point in path[1:-1]))

    def test_pure_pursuit_outputs_forward_command_on_straight_path(self) -> None:
        controller = PurePursuitController(
            {
                "lookahead_m": 0.25,
                "goal_tolerance_m": 0.08,
                "slowdown_radius_m": 0.35,
                "max_vx": 0.25,
                "max_wz": 0.8,
            }
        )
        controller.set_path([(0.0, 0.0), (1.0, 0.0)])

        command, goal_reached = controller.compute_command(Pose2D(0.0, 0.0, 0.0), timestamp=1.0)

        self.assertFalse(goal_reached)
        self.assertGreater(command.v_x, 0.0)
        self.assertAlmostEqual(command.v_y, 0.0)
        self.assertAlmostEqual(command.w_z, 0.0, delta=0.1)

    def test_pure_pursuit_stops_at_goal(self) -> None:
        controller = PurePursuitController(
            {
                "lookahead_m": 0.25,
                "goal_tolerance_m": 0.08,
                "slowdown_radius_m": 0.35,
                "max_vx": 0.25,
                "max_wz": 0.8,
            }
        )
        controller.set_path([(0.0, 0.0), (1.0, 0.0)])

        command, goal_reached = controller.compute_command(Pose2D(0.97, 0.01, 0.0), timestamp=2.0)

        self.assertTrue(goal_reached)
        self.assertAlmostEqual(command.v_x, 0.0)
        self.assertAlmostEqual(command.v_y, 0.0)
        self.assertAlmostEqual(command.w_z, 0.0)

    def test_runtime_drives_local_sim_robot_to_goal(self) -> None:
        runtime = GroundStationRuntime(
            {
                "server": {"host": "127.0.0.1", "port": 0},
                "map": {"x_min": -1.0, "x_max": 1.0, "y_min": -1.0, "y_max": 1.0, "ppm": 200},
                "grid": {"resolution_m": 0.05, "line_obstacle_width_m": 0.08},
                "control": {
                    "control_hz": 20.0,
                    "lookahead_m": 0.22,
                    "goal_tolerance_m": 0.08,
                    "slowdown_radius_m": 0.30,
                    "max_vx": 0.25,
                    "max_wz": 0.8,
                },
            }
        )
        runtime.start()
        host, port = runtime.bound_address
        robot = LocalSimRobotClient(host=host, port=port)
        robot.start()
        try:
            deadline = time.time() + 2.0
            while time.time() < deadline and not runtime.connected:
                time.sleep(0.02)
            self.assertTrue(runtime.connected)

            runtime.update_pose(robot.pose)
            path = runtime.plan(goal=(0.6, 0.0), rects=[], lines=[])
            self.assertGreaterEqual(len(path), 2)
            runtime.start_execution()

            deadline = time.time() + 6.0
            while time.time() < deadline and runtime.state.state == "executing":
                runtime.update_pose(robot.pose)
                runtime.tick()
                time.sleep(0.05)

            deadline = time.time() + 1.0
            while time.time() < deadline and robot.reach_goal_count < 1:
                time.sleep(0.02)

            self.assertEqual(runtime.state.state, "idle")
            self.assertGreater(robot.pose.x, 0.45)
            self.assertEqual(robot.reach_goal_count, 1)
        finally:
            robot.stop()
            runtime.stop()


if __name__ == "__main__":
    unittest.main()
