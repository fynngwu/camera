"""Unit tests for AG JSONL protocol helpers."""
from __future__ import annotations

import unittest

from common.protocol import (
    decode_message,
    encode_message,
    make_message,
    validate_cmd_vel,
    validate_path,
    validate_telemetry,
)


class ProtocolValidationTests(unittest.TestCase):
    """Schema and validation tests for path/cmd/telemetry messages."""

    def test_roundtrip_message(self) -> None:
        msg = make_message("cmd_vel", seq=3, vx=0.2, vy=0.0, wz=-0.1)
        obj = decode_message(encode_message(msg))
        self.assertEqual(obj["type"], "cmd_vel")
        self.assertEqual(obj["seq"], 3)
        self.assertAlmostEqual(obj["vx"], 0.2)

    def test_validate_path_success(self) -> None:
        points = validate_path(
            {
                "type": "path",
                "seq": 7,
                "points": [[0.0, 0.0], [0.5, 0.2], [0.8, 0.4]],
                "target_speed": 0.25,
            }
        )
        self.assertEqual(len(points), 3)
        self.assertAlmostEqual(points[1][0], 0.5)

    def test_validate_path_rejects_short(self) -> None:
        with self.assertRaises(ValueError):
            validate_path({"type": "path", "seq": 1, "points": [[0.0, 0.0]]})

    def test_validate_cmd_vel_requires_vx_vy_wz(self) -> None:
        with self.assertRaises(ValueError):
            validate_cmd_vel({"type": "cmd_vel", "vx": 0.1, "wz": 0.2})

    def test_validate_telemetry_success(self) -> None:
        msg = {
            "type": "telemetry",
            "seq": 9,
            "pose": {"x": 0.1, "y": -0.2, "yaw": 1.2},
            "cmd": {"vx": 0.2, "vy": 0.0, "wz": 0.3},
            "target_index": 5,
            "goal_reached": False,
        }
        validate_telemetry(msg)

    def test_validate_telemetry_rejects_bad_goal_flag(self) -> None:
        with self.assertRaises(ValueError):
            validate_telemetry(
                {
                    "type": "telemetry",
                    "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
                    "cmd": {"vx": 0.0, "vy": 0.0, "wz": 0.0},
                    "target_index": 0,
                    "goal_reached": "false",
                }
            )


if __name__ == "__main__":
    unittest.main()
