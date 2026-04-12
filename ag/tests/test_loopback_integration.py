"""Local loopback integration test: ground_station -> sky_uav -> robot."""
from __future__ import annotations

import socket
import threading
import time
import unittest
from typing import Dict, List, Optional

from common.protocol import decode_message, encode_message, make_message, validate_cmd_vel, validate_telemetry
from sky_uav.tcp_connector import SkyTcpConnector


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def _connect_with_retry(host: str, port: int, timeout_sec: float = 3.0) -> socket.socket:
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        try:
            return socket.create_connection((host, port), timeout=0.4)
        except OSError:
            time.sleep(0.05)
    raise TimeoutError(f"connect timeout: {host}:{port}")


class RobotStub:
    """Minimal robot endpoint for integration test."""

    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = port
        self.commands: List[Dict[str, float]] = []
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._sock: Optional[socket.socket] = None

    def start(self) -> None:
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        if self._thread is not None:
            self._thread.join(timeout=1.5)

    def _run(self) -> None:
        try:
            sock = _connect_with_retry(self.host, self.port, timeout_sec=4.0)
        except Exception:
            return
        self._sock = sock
        sock.settimeout(0.2)
        buffer = b""
        try:
            while not self._stop_event.is_set():
                try:
                    chunk = sock.recv(65536)
                except socket.timeout:
                    continue
                except OSError:
                    return
                if not chunk:
                    return
                buffer += chunk
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    if not line.strip():
                        continue
                    msg = decode_message(line)
                    if msg.get("type") != "cmd_vel":
                        continue
                    cmd = validate_cmd_vel(msg)
                    self.commands.append(cmd)
                    ack = make_message("ack", ack_seq=int(msg.get("seq", 0)), ok=True, msg="cmd received")
                    try:
                        sock.sendall(encode_message(ack))
                    except OSError:
                        return
        finally:
            try:
                sock.close()
            except OSError:
                pass
            self._sock = None


class LoopbackIntegrationTests(unittest.TestCase):
    """Verify end-to-end command + telemetry flow on localhost."""

    def test_path_to_cmd_and_telemetry(self) -> None:
        gcs_port = _free_port()
        robot_port = _free_port()
        connector = SkyTcpConnector(
            {
                "gcs_bind_host": "127.0.0.1",
                "gcs_port": gcs_port,
                "robot_bind_host": "127.0.0.1",
                "robot_port": robot_port,
                "telemetry_hz": 15.0,
                "controller": {
                    "control_hz": 25.0,
                    "target_speed": 0.25,
                    "goal_tolerance": 0.12,
                    "lookahead_distance": 0.2,
                    "resample_step": 0.06,
                },
            }
        )

        connector_thread = threading.Thread(target=connector.run_forever, daemon=True)
        connector_thread.start()

        robot = RobotStub("127.0.0.1", robot_port)
        robot.start()

        gcs_sock: Optional[socket.socket] = None
        try:
            gcs_sock = _connect_with_retry("127.0.0.1", gcs_port, timeout_sec=4.0)
            gcs_sock.settimeout(0.2)
            gcs_sock.sendall(
                encode_message(
                    make_message(
                        "path",
                        seq=1,
                        points=[[0.0, 0.0], [0.9, 0.0], [0.9, 0.4]],
                        target_speed=0.24,
                    )
                )
            )

            telemetries = []
            ack_ok = False
            buffer = b""
            deadline = time.time() + 5.0
            while time.time() < deadline and (len(telemetries) < 4 or len(robot.commands) < 4):
                try:
                    chunk = gcs_sock.recv(65536)
                except socket.timeout:
                    continue
                if not chunk:
                    break
                buffer += chunk
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    if not line.strip():
                        continue
                    msg = decode_message(line)
                    if msg.get("type") == "ack" and int(msg.get("ack_seq", -1)) == 1:
                        ack_ok = bool(msg.get("ok", False))
                    elif msg.get("type") == "telemetry":
                        validate_telemetry(msg)
                        telemetries.append(msg)

            self.assertTrue(ack_ok)
            self.assertGreaterEqual(len(telemetries), 3)
            self.assertGreaterEqual(len(robot.commands), 3)
            self.assertTrue(any(abs(cmd["vx"]) > 1e-4 for cmd in robot.commands))

        finally:
            if gcs_sock is not None:
                try:
                    gcs_sock.close()
                except OSError:
                    pass
            robot.stop()
            connector.stop()
            connector_thread.join(timeout=2.5)


if __name__ == "__main__":
    unittest.main()
