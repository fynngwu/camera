"""Ground robot endpoint: connect to the GCS server and print commands."""
from __future__ import annotations

import argparse
import json
import logging
import socket
import time
from typing import Any, Dict

from common.protocol import decode_message

LOGGER = logging.getLogger(__name__)


DEFAULT_CONFIG = {
    "gcs_host": "127.0.0.1",
    "gcs_port": 47001,
    "reconnect_sec": 1.0,
}


class GroundRobotReceiver:
    """TCP client that receives `cmd_vel` and `reach_goal` messages."""

    def __init__(self, config: Dict[str, Any]) -> None:
        self.config = {**DEFAULT_CONFIG, **config}

    def run_forever(self) -> None:
        """Keep reconnecting to the GCS server and process incoming commands."""
        host = str(self.config.get("gcs_host", self.config.get("sky_host", "127.0.0.1")))
        port = int(self.config.get("gcs_port", self.config.get("sky_port", 47001)))
        reconnect = max(0.2, float(self.config.get("reconnect_sec", 1.0)))
        while True:
            try:
                LOGGER.info("connecting to gcs %s:%s", host, port)
                with socket.create_connection((host, port), timeout=3.0) as sock:
                    sock.settimeout(1.0)
                    LOGGER.info("connected to gcs %s:%s", host, port)
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
                self._handle_line(line)

    def _handle_line(self, line: bytes) -> None:
        try:
            message = decode_message(line)
            if message.get("type") == "cmd_vel":
                print(
                    f"[{time.strftime('%H:%M:%S')}] cmd_vel "
                    f"vx={float(message['vx']):.3f} "
                    f"vy={float(message['vy']):.3f} "
                    f"wz={float(message['wz']):.3f}",
                    flush=True,
                )
                return
            if message.get("type") == "reach_goal":
                print(f"[{time.strftime('%H:%M:%S')}] reach_goal", flush=True)
                return
            raise ValueError(f"unsupported type: {message.get('type')}")
        except Exception as exc:
            LOGGER.warning("invalid message: %s", exc)


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
