"""Sky UAV TCP hub: route path/cmd/telemetry between GCS and ground robot."""
from __future__ import annotations

import argparse
import json
import logging
import socket
import threading
import time
from typing import Any, Dict, Optional

from common.protocol import (
    decode_message,
    encode_message,
    make_ack,
    make_event,
    make_message,
    now_ts,
    validate_path,
)
from sky_uav.pose_controller import PoseController

LOGGER = logging.getLogger(__name__)


DEFAULT_CONFIG = {
    "gcs_bind_host": "0.0.0.0",
    "gcs_port": 46001,
    "robot_bind_host": "0.0.0.0",
    "robot_port": 47001,
    "telemetry_hz": 10.0,
    "controller": {},
}


class SkyTcpConnector:
    """Run GCS/robot sockets and route messages around PoseController."""

    def __init__(self, config: Dict[str, Any]) -> None:
        self.config = {**DEFAULT_CONFIG, **config}
        self.controller = PoseController(self.config.get("controller", {}), on_command=self._on_command)

        self._gcs_lock = threading.Lock()
        self._robot_lock = threading.Lock()
        self._stop_event = threading.Event()

        self._gcs_conn: Optional[socket.socket] = None
        self._robot_conn: Optional[socket.socket] = None

        self._cmd_seq = 1
        self._telemetry_seq = 1
        self._threads: list[threading.Thread] = []
        self._last_robot_warn_ts = 0.0

    def run_forever(self) -> None:
        """Start connector threads and block until interrupted."""
        self.controller.start()
        self._start_thread(self._serve_gcs, "sky-gcs-server")
        self._start_thread(self._serve_robot, "sky-robot-server")
        self._start_thread(self._telemetry_loop, "sky-telemetry-loop")

        LOGGER.info(
            "sky_uav connector started | gcs_port=%s robot_port=%s",
            self.config["gcs_port"],
            self.config["robot_port"],
        )
        try:
            while not self._stop_event.is_set():
                time.sleep(0.2)
        except KeyboardInterrupt:
            LOGGER.info("stopping sky_uav connector")
        finally:
            self.stop()

    def stop(self) -> None:
        """Stop connector and close sockets."""
        self._stop_event.set()
        self.controller.stop()
        self._set_gcs_conn(None)
        self._set_robot_conn(None)

    def _start_thread(self, target, name: str) -> None:
        thread = threading.Thread(target=target, name=name, daemon=True)
        thread.start()
        self._threads.append(thread)

    def _serve_gcs(self) -> None:
        host = str(self.config["gcs_bind_host"])
        port = int(self.config["gcs_port"])
        with _listening_socket(host, port) as server:
            LOGGER.info("listening for ground_station on %s:%s", host, port)
            while not self._stop_event.is_set():
                try:
                    server.settimeout(1.0)
                    conn, addr = server.accept()
                except socket.timeout:
                    continue
                except OSError:
                    return
                LOGGER.info("ground_station connected from %s", addr)
                self._set_gcs_conn(conn)
                self._send_gcs(make_event("INFO", f"sky_uav connected to gcs from {addr}", "sky_uav"))
                self._start_thread(lambda: self._gcs_reader_loop(conn), "sky-gcs-reader")

    def _serve_robot(self) -> None:
        host = str(self.config["robot_bind_host"])
        port = int(self.config["robot_port"])
        with _listening_socket(host, port) as server:
            LOGGER.info("listening for ground_robot on %s:%s", host, port)
            while not self._stop_event.is_set():
                try:
                    server.settimeout(1.0)
                    conn, addr = server.accept()
                except socket.timeout:
                    continue
                except OSError:
                    return
                LOGGER.info("ground_robot connected from %s", addr)
                self._set_robot_conn(conn)
                self._send_gcs(make_event("INFO", f"ground_robot connected: {addr}", "sky_uav"))
                self._start_thread(lambda: self._robot_reader_loop(conn), "sky-robot-reader")

    def _gcs_reader_loop(self, conn: socket.socket) -> None:
        try:
            self._socket_line_loop(conn, self._handle_gcs_message)
        finally:
            if self._is_same_conn(conn, gcs=True):
                LOGGER.warning("ground_station disconnected")
                self._set_gcs_conn(None)

    def _robot_reader_loop(self, conn: socket.socket) -> None:
        try:
            self._socket_line_loop(conn, self._handle_robot_message)
        finally:
            if self._is_same_conn(conn, gcs=False):
                LOGGER.warning("ground_robot disconnected")
                self._set_robot_conn(None)
                self._send_gcs(make_event("WARN", "ground_robot disconnected", "sky_uav"))

    def _socket_line_loop(self, conn: socket.socket, handler) -> None:
        buffer = b""
        conn.settimeout(1.0)
        while not self._stop_event.is_set():
            try:
                chunk = conn.recv(65536)
                if not chunk:
                    return
                buffer += chunk
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    if not line.strip():
                        continue
                    message = decode_message(line)
                    handler(message)
            except socket.timeout:
                continue
            except Exception as exc:
                LOGGER.warning("socket line loop stopped: %s", exc)
                return

    def _handle_gcs_message(self, message: Dict[str, Any]) -> None:
        mtype = message.get("type")
        seq = int(message.get("seq", 0))
        if mtype == "path":
            try:
                points = validate_path(message)
                target_speed = float(message.get("target_speed", self.controller.cfg.target_speed))
                smoothed = self.controller.set_path(seq=seq, points=points, target_speed=target_speed)
                self._send_gcs(make_ack(seq, True, f"path accepted, smoothed_points={len(smoothed)}"))
                self._send_gcs(make_event("INFO", f"new path accepted, {len(points)} -> {len(smoothed)} points", "sky_uav"))
            except Exception as exc:
                self._send_gcs(make_ack(seq, False, str(exc)))
        elif mtype == "request_status":
            self._publish_telemetry_once()
            self._send_gcs(make_ack(seq, True, "telemetry published"))
        else:
            self._send_gcs(make_ack(seq, False, f"unsupported type: {mtype}"))

    def _handle_robot_message(self, message: Dict[str, Any]) -> None:
        mtype = message.get("type", "unknown")
        if mtype in {"ack", "event"}:
            self._send_gcs(make_message("robot_message", ts=now_ts(), payload=message))

    def _on_command(self, cmd: Dict[str, float], snapshot: Dict[str, object]) -> None:
        seq = self._next_cmd_seq()
        message = make_message("cmd_vel", seq=seq, vx=cmd["vx"], vy=cmd["vy"], wz=cmd["wz"], ts=now_ts())
        self._send_robot(message)

    def _telemetry_loop(self) -> None:
        period = 1.0 / max(float(self.config.get("telemetry_hz", 10.0)), 1.0)
        while not self._stop_event.is_set():
            start = time.monotonic()
            self._publish_telemetry_once()
            elapsed = time.monotonic() - start
            wait_sec = period - elapsed
            if wait_sec > 0.0:
                time.sleep(wait_sec)

    def _publish_telemetry_once(self) -> None:
        snapshot = self.controller.get_snapshot()
        message = make_message(
            "telemetry",
            seq=self._next_telemetry_seq(),
            pose=snapshot["pose"],
            cmd=snapshot["cmd"],
            target_index=snapshot["target_index"],
            goal_reached=snapshot["goal_reached"],
            path_seq=snapshot["path_seq"],
            ts=now_ts(),
        )
        self._send_gcs(message)

    def _next_cmd_seq(self) -> int:
        value = self._cmd_seq
        self._cmd_seq += 1
        return value

    def _next_telemetry_seq(self) -> int:
        value = self._telemetry_seq
        self._telemetry_seq += 1
        return value

    def _send_gcs(self, message: Dict[str, Any]) -> bool:
        with self._gcs_lock:
            conn = self._gcs_conn
        if conn is None:
            return False
        try:
            conn.sendall(encode_message(message))
            return True
        except Exception:
            self._set_gcs_conn(None)
            return False

    def _send_robot(self, message: Dict[str, Any]) -> bool:
        with self._robot_lock:
            conn = self._robot_conn
        if conn is None:
            self._warn_robot_unavailable()
            return False
        try:
            conn.sendall(encode_message(message))
            return True
        except Exception:
            self._set_robot_conn(None)
            self._warn_robot_unavailable()
            return False

    def _warn_robot_unavailable(self) -> None:
        now = time.monotonic()
        if now - self._last_robot_warn_ts < 2.0:
            return
        self._last_robot_warn_ts = now
        LOGGER.warning("ground_robot is not connected; cmd_vel dropped")

    def _set_gcs_conn(self, conn: Optional[socket.socket]) -> None:
        with self._gcs_lock:
            if self._gcs_conn is conn:
                return
            _close_socket(self._gcs_conn)
            self._gcs_conn = conn

    def _set_robot_conn(self, conn: Optional[socket.socket]) -> None:
        with self._robot_lock:
            if self._robot_conn is conn:
                return
            _close_socket(self._robot_conn)
            self._robot_conn = conn

    def _is_same_conn(self, conn: socket.socket, gcs: bool) -> bool:
        lock = self._gcs_lock if gcs else self._robot_lock
        with lock:
            current = self._gcs_conn if gcs else self._robot_conn
            return current is conn


def _listening_socket(host: str, port: int) -> socket.socket:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen(5)
    return server


def _close_socket(conn: Optional[socket.socket]) -> None:
    if conn is None:
        return
    try:
        conn.close()
    except Exception:
        pass


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(description="Sky UAV TCP connector")
    parser.add_argument("--config", default="config/sky_uav.json", help="Path to sky_uav JSON config")
    parser.add_argument("--log-level", default="INFO", help="Python logging level")
    return parser.parse_args()


def load_config(path: str) -> Dict[str, Any]:
    """Load JSON config."""
    try:
        with open(path, "r", encoding="utf-8") as fp:
            return json.load(fp)
    except FileNotFoundError:
        LOGGER.warning("config not found, using defaults: %s", path)
        return {}


def main() -> None:
    """Program entry point."""
    args = parse_args()
    logging.basicConfig(
        level=getattr(logging, str(args.log_level).upper(), logging.INFO),
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    )
    connector = SkyTcpConnector(load_config(args.config))
    connector.run_forever()


if __name__ == "__main__":
    main()
