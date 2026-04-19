"""Tests for the ground-station protocol and TOML config loader."""
from __future__ import annotations

import tempfile
import textwrap
import unittest
from pathlib import Path

from common.protocol import decode_message, encode_cmd_vel, encode_reach_goal
from ground_station.config import load_config


class GroundStationProtocolTests(unittest.TestCase):
    def test_encode_cmd_vel_decodes_to_expected_dict(self) -> None:
        message = decode_message(encode_cmd_vel(0.2, 0.0, -0.1, 1710000000.0))
        self.assertEqual(
            message,
            {
                "type": "cmd_vel",
                "vx": 0.2,
                "vy": 0.0,
                "wz": -0.1,
                "timestamp": 1710000000.0,
            },
        )

    def test_encode_reach_goal_decodes_to_expected_dict(self) -> None:
        message = decode_message(encode_reach_goal(1710000001.0))
        self.assertEqual(
            message,
            {
                "type": "reach_goal",
                "timestamp": 1710000001.0,
            },
        )

    def test_load_config_merges_defaults_for_missing_sections(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            config_path = Path(tmpdir) / "ground_station.toml"
            config_path.write_text(
                textwrap.dedent(
                    """
                    [server]
                    port = 12345

                    [bev]
                    enabled = false
                    """
                ).lstrip(),
                encoding="utf-8",
            )

            config = load_config(config_path)

        self.assertEqual(config["server"]["host"], "127.0.0.1")
        self.assertEqual(config["server"]["port"], 12345)
        self.assertEqual(config["rtsp"]["url"], "rtsp://10.28.215.179:8554/cam")
        self.assertEqual(config["grid"]["resolution_m"], 0.05)
        self.assertEqual(config["control"]["max_vx"], 0.25)
        self.assertFalse(config["bev"]["enabled"])
        self.assertEqual(config["bev"]["tag_size"], 0.09)


if __name__ == "__main__":
    unittest.main()
