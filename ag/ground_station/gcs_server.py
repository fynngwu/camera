"""Minimal TCP server that waits for a robot client to connect."""
from __future__ import annotations

import queue
import socket
import threading
from typing import Optional

from common.protocol import encode_cmd_vel, encode_reach_goal


class GcsServer:
    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = port
        self._server: Optional[socket.socket] = None
        self._client: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._outbox: "queue.Queue[bytes]" = queue.Queue()
        self.bound_address: tuple[str, int] = (host, port)

    @property
    def connected(self) -> bool:
        return self._client is not None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind((self.host, self.port))
        self._server.listen(1)
        self.bound_address = self._server.getsockname()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        for sock in (self._client, self._server):
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def send_cmd_vel(self, vx: float, vy: float, wz: float, timestamp: float) -> None:
        self._outbox.put(encode_cmd_vel(vx=vx, vy=vy, wz=wz, timestamp=timestamp))

    def send_reach_goal(self, timestamp: float) -> None:
        self._outbox.put(encode_reach_goal(timestamp=timestamp))

    def _run(self) -> None:
        assert self._server is not None
        self._server.settimeout(0.2)
        while not self._stop_event.is_set():
            if self._client is None:
                try:
                    self._client, _ = self._server.accept()
                    self._client.settimeout(0.2)
                except socket.timeout:
                    continue
                except OSError:
                    return
            try:
                payload = self._outbox.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                self._client.sendall(payload)
            except OSError:
                try:
                    self._client.close()
                except OSError:
                    pass
                self._client = None
