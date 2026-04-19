"""Tests for the GCS TCP server."""
from __future__ import annotations

import socket
import unittest

from common.protocol import decode_message
from ground_station.gcs_server import GcsServer


class GcsServerTests(unittest.TestCase):
    def test_connected_client_receives_cmd_vel(self) -> None:
        server = GcsServer(host="127.0.0.1", port=0)
        server.start()
        try:
            host, port = server.bound_address
            with socket.create_connection((host, port), timeout=1.0) as sock:
                server.send_cmd_vel(vx=0.1, vy=0.0, wz=0.2, timestamp=1.0)
                line = sock.recv(1024)
            self.assertEqual(
                decode_message(line),
                {
                    "type": "cmd_vel",
                    "vx": 0.1,
                    "vy": 0.0,
                    "wz": 0.2,
                    "timestamp": 1.0,
                },
            )
        finally:
            server.stop()


if __name__ == "__main__":
    unittest.main()
