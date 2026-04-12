"""Unit tests for path smoothing and virtual odom controller."""
from __future__ import annotations

import math
import unittest

from sky_uav.pose_controller import PoseController, resample_polyline, smooth_path


class PathSmoothingTests(unittest.TestCase):
    """Validate cubic smoothing and arc-length resampling behavior."""

    def test_smooth_path_keeps_endpoints(self) -> None:
        raw = [(0.0, 0.0), (0.5, 0.4), (1.0, 0.0)]
        smooth = smooth_path(raw, samples_per_segment=12)
        self.assertGreater(len(smooth), len(raw))
        self.assertEqual(smooth[0], raw[0])
        self.assertEqual(smooth[-1], raw[-1])

    def test_resample_spacing_is_reasonable(self) -> None:
        sampled = resample_polyline([(0.0, 0.0), (1.0, 0.0)], step=0.2)
        self.assertEqual(sampled[0], (0.0, 0.0))
        self.assertEqual(sampled[-1], (1.0, 0.0))
        gaps = [abs(sampled[i + 1][0] - sampled[i][0]) for i in range(len(sampled) - 1)]
        for gap in gaps:
            self.assertAlmostEqual(gap, 0.2, delta=0.03)


class PoseControllerTests(unittest.TestCase):
    """Validate virtual odom integration and point-by-point tracking."""

    def _build_controller(self) -> PoseController:
        return PoseController(
            {
                "target_speed": 0.28,
                "k_speed": 1.3,
                "k_yaw": 2.0,
                "lookahead_distance": 0.2,
                "goal_tolerance": 0.12,
                "slowdown_radius": 0.35,
                "vx_limit": 0.35,
                "wz_limit": 1.0,
                "resample_step": 0.06,
                "spline_samples_per_segment": 16,
            }
        )

    def test_reaches_goal_on_straight_path(self) -> None:
        ctrl = self._build_controller()
        ctrl.set_path(seq=1, points=[(0.0, 0.0), (1.0, 0.0)], target_speed=0.25)

        snap = ctrl.get_snapshot()
        for _ in range(420):
            snap = ctrl.step_once(0.05)
            if snap["goal_reached"]:
                break

        self.assertTrue(snap["goal_reached"])
        self.assertGreater(float(snap["pose"]["x"]), 0.8)
        self.assertAlmostEqual(float(snap["cmd"]["vx"]), 0.0, delta=1e-6)

    def test_turning_path_changes_heading(self) -> None:
        ctrl = self._build_controller()
        ctrl.set_path(seq=2, points=[(0.0, 0.0), (0.7, 0.0), (0.7, 0.6)], target_speed=0.23)

        snap = ctrl.get_snapshot()
        for _ in range(520):
            snap = ctrl.step_once(0.05)
            if snap["goal_reached"]:
                break

        self.assertTrue(snap["goal_reached"])
        self.assertGreater(float(snap["pose"]["y"]), 0.45)
        self.assertGreater(abs(float(snap["pose"]["yaw"])), 0.2)
        self.assertLess(abs(float(snap["cmd"]["wz"])), 1e-5)

    def test_cmd_zero_without_path(self) -> None:
        ctrl = self._build_controller()
        snap = ctrl.step_once(0.05)
        cmd = snap["cmd"]
        self.assertAlmostEqual(float(cmd["vx"]), 0.0)
        self.assertAlmostEqual(float(cmd["vy"]), 0.0)
        self.assertAlmostEqual(float(cmd["wz"]), 0.0)


if __name__ == "__main__":
    unittest.main()
