"""Ground robot endpoint: background thread reads cmd_vel from GCS, main loop uses it."""
from __future__ import annotations

import argparse
import json
import socket
import threading
import time
import tomllib


class CommandReader:
    """Background thread that reads cmd_vel messages from GCS.

    Usage:
        reader = CommandReader(host, port)
        reader.start()
        # ... main loop ...
        #   vx, vy, wz = reader.last_cmd
        #   reader.stop()
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 47001) -> None:
        self.host = host
        self.port = port
        self.last_cmd: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.goal_reached = False
        self._sock: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._sock:
            self._sock.close()
        if self._thread:
            self._thread.join(timeout=2.0)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                print(f"connecting to {self.host}:{self.port}", flush=True)
                self._sock = socket.create_connection((self.host, self.port))
                print(f"connected to {self.host}:{self.port}", flush=True)
                self._read_loop()
                if self.goal_reached:
                    return
            except Exception:
                pass
            finally:
                if self._sock:
                    self._sock.close()
                    self._sock = None
                self._stop.wait(1.0)

    def _read_loop(self) -> None:
        buf = b""
        while not self._stop.is_set():
            chunk = self._sock.recv(65536)
            if not chunk:
                return
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                msg = json.loads(line)
                if msg["type"] == "cmd_vel":
                    self.last_cmd = (float(msg["vx"]), float(msg["vy"]), float(msg["wz"]))
                elif msg["type"] == "reach_goal":
                    self.goal_reached = True
                    return


def main() -> None:
    parser = argparse.ArgumentParser(description="Ground robot receiver")
    parser.add_argument("--config", default="config/ground_station.toml")
    parser.add_argument("--host", default=None)
    parser.add_argument("--port", type=int, default=None)
    args = parser.parse_args()

    with open(args.config, "rb") as f:
        cfg = tomllib.load(f)

    host = args.host or cfg.get("server", {}).get("host", "127.0.0.1")
    port = args.port or cfg.get("server", {}).get("port", 47001)

    reader = CommandReader(host, port)
    reader.start()

    state = "idle"
    try:
        while True:
            if state == "idle":
                vx, vy, wz = reader.last_cmd
                if vx != 0.0 or vy != 0.0 or wz != 0.0:
                    state = "running"
                    print(f"[{time.strftime('%H:%M:%S')}] >> running", flush=True)
            elif state == "running":
                vx, vy, wz = reader.last_cmd
                print(
                    f"[{time.strftime('%H:%M:%S')}] cmd_vel "
                    f"vx={vx:.3f} vy={vy:.3f} wz={wz:.3f}",
                    flush=True,
                )
                if reader.goal_reached:
                    state = "done"
                    reader.stop()
                    print(f"[{time.strftime('%H:%M:%S')}] >> done", flush=True)
            elif state == "done":
                break
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    reader.stop()


if __name__ == "__main__":
    main()
