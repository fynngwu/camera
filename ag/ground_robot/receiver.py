"""Ground robot endpoint: receive cmd_vel from sky UAV and print values."""
from __future__ import annotations

import argparse
import json
import logging
import socket
import time
from typing import Any, Dict

from common.protocol import decode_message, encode_message, make_ack, make_event, validate_cmd_vel

LOGGER = logging.getLogger(__name__)


DEFAULT_CONFIG = {
    "sky_host": "127.0.0.1",
    "sky_port": 47001,
    "reconnect_sec": 1.0,
}


class GroundRobotReceiver:
    """TCP client that receives cmd_vel and prints it."""

    def __init__(self, config: Dict[str, Any]) -> None:
        self.config = {**DEFAULT_CONFIG, **config}

    def run_forever(self) -> None:
        """Keep reconnecting to sky side and process incoming commands."""
        host = str(self.config["sky_host"])
        port = int(self.config["sky_port"])
        reconnect = max(0.2, float(self.config.get("reconnect_sec", 1.0)))
        while True:
            try:
                LOGGER.info("connecting to sky_uav %s:%s", host, port)
                with socket.create_connection((host, port), timeout=3.0) as sock:
                    sock.settimeout(1.0)
                    self._send(sock, make_event("INFO", "ground robot connected", "ground_robot"))
                    LOGGER.info("connected to sky_uav %s:%s", host, port)
                    self._read_loop(sock)
            except KeyboardInterrupt:
                raise
            except Exception as exc:
                LOGGER.warning("connection failed/disconnected: %s", exc)
                time.sleep(reconnect)

    def _read_loop(self, sock: socket.socket) -> None:
        buffer = b""
        while True:
            chunk = sock.recv(65536)
            if not chunk:
                return
            buffer += chunk
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                if not line.strip():
                    continue
                self._handle_line(sock, line)

    def _handle_line(self, sock: socket.socket, line: bytes) -> None:
        try:
            message = decode_message(line)
            seq = int(message.get("seq", 0))
            if message.get("type") != "cmd_vel":
                self._send(sock, make_ack(seq, False, f"unsupported type: {message.get('type')}") )
                return
            cmd = validate_cmd_vel(message)
            print(
                f"[{time.strftime('%H:%M:%S')}] cmd_vel "
                f"vx={cmd['vx']:.3f} vy={cmd['vy']:.3f} wz={cmd['wz']:.3f}",
                flush=True,
            )
            self._send(sock, make_ack(seq, True, "cmd_vel received"))
        except Exception as exc:
            LOGGER.warning("invalid message: %s", exc)
            self._send(sock, make_event("WARN", f"invalid message: {exc}", "ground_robot"))

    @staticmethod
    def _send(sock: socket.socket, message: Dict[str, Any]) -> None:
        sock.sendall(encode_message(message))


def parse_args() -> argparse.Namespace:
    """Parse CLI args."""
    parser = argparse.ArgumentParser(description="Ground robot receiver")
    parser.add_argument("--config", default="config/ground_robot.json", help="Path to ground_robot config JSON")
    parser.add_argument("--log-level", default="INFO", help="Python logging level")
    return parser.parse_args()


def load_config(path: str) -> Dict[str, Any]:
    """Load JSON config if it exists."""
    try:
        with open(path, "r", encoding="utf-8") as fp:
            return json.load(fp)
    except FileNotFoundError:
        LOGGER.warning("config not found, using defaults: %s", path)
        return {}


def main() -> None:
    """Program entry."""
    args = parse_args()
    logging.basicConfig(
        level=getattr(logging, str(args.log_level).upper(), logging.INFO),
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    )
    GroundRobotReceiver(load_config(args.config)).run_forever()


if __name__ == "__main__":
    main()
