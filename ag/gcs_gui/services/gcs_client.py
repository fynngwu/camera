"""Thread-based TCP client for the Qt GUI."""
from __future__ import annotations

import json
import queue
import socket
import threading
import time
from typing import Any, Callable, Dict, Optional

from backend.shared.protocol import make_message, now_ts
from gcs_gui.qt_compat import QObject, Signal, Slot


class GcsTcpClient:
    """Pure Python TCP client for the ground-control GUI."""

    def __init__(self, host: str, port: int, heartbeat_interval: float = 1.0, reconnect_interval: float = 1.0) -> None:
        self.host = host
        self.port = int(port)
        self.heartbeat_interval = float(heartbeat_interval)
        self.reconnect_interval = float(reconnect_interval)
        self.on_message: Optional[Callable[[Dict[str, Any]], None]] = None
        self.on_log: Optional[Callable[[str, str], None]] = None
        self.on_connection_change: Optional[Callable[[bool], None]] = None
        self._outbox: "queue.Queue[Dict[str, Any]]" = queue.Queue()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._seq = 1
        self._socket: Optional[socket.socket] = None
        self._connected = False

    def start(self) -> None:
        """Start the background networking thread."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop the client and close the underlying socket."""
        self._stop_event.set()
        self._close_socket()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def reconfigure(self, host: str, port: int) -> None:
        """Update host and port used for the next reconnect."""
        self.host = host
        self.port = int(port)
        self._close_socket()

    def next_seq(self) -> int:
        """Return the next message sequence number."""
        value = self._seq
        self._seq += 1
        return value

    def send(self, message: Dict[str, Any]) -> None:
        """Queue one outgoing JSON message."""
        self._outbox.put(message)

    def send_mode(self, mode: str) -> None:
        """Send a mode message."""
        self.send(make_message("mode", seq=self.next_seq(), mode=mode))

    def send_cmd_vel(self, vx: float, vy: float, wz: float, timeout_ms: int = 400) -> None:
        """Send a manual velocity command."""
        self.send(make_message("cmd_vel", seq=self.next_seq(), t=now_ts(), vx=vx, vy=vy, wz=wz, timeout_ms=timeout_ms))

    def send_pose(self, x: float, y: float, yaw: float) -> None:
        """Send a pose2d update."""
        self.send(make_message("pose2d", seq=self.next_seq(), frame="world", x=x, y=y, yaw=yaw))

    def send_path(self, points: list[list[float]], target_speed: float = 0.2) -> None:
        """Send a planned path."""
        self.send(
            make_message(
                "path",
                seq=self.next_seq(),
                path_id=int(time.time()),
                frame="world",
                target_speed=target_speed,
                points=points,
            )
        )

    def request_status(self) -> None:
        """Request one bridge_status update."""
        self.send(make_message("request_status", seq=self.next_seq()))

    def _run(self) -> None:
        """Main socket loop with reconnect and heartbeat."""
        buffer = b""
        last_heartbeat = 0.0
        while not self._stop_event.is_set():
            if self._socket is None:
                self._connect_once()
                last_heartbeat = 0.0
                buffer = b""
                if self._socket is None:
                    time.sleep(self.reconnect_interval)
                    continue
            now = time.time()
            if now - last_heartbeat >= self.heartbeat_interval:
                self.send(make_message("heartbeat", src="gcs", seq=self.next_seq(), t=now_ts()))
                last_heartbeat = now
            self._flush_outbox()
            try:
                chunk = self._socket.recv(65536)
                if not chunk:
                    raise ConnectionError("peer closed")
                buffer += chunk
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    if not line.strip():
                        continue
                    message = json.loads(line.decode("utf-8"))
                    self._emit_message(message)
            except socket.timeout:
                continue
            except (OSError, ValueError, ConnectionError) as exc:
                self._log("WARN", f"socket disconnected: {exc}")
                self._close_socket()
                time.sleep(self.reconnect_interval)

    def _connect_once(self) -> None:
        """Try to establish one TCP connection to the bridge."""
        try:
            sock = socket.create_connection((self.host, self.port), timeout=2.0)
            sock.settimeout(0.2)
            self._socket = sock
            self._set_connected(True)
            self._log("INFO", f"connected to {self.host}:{self.port}")
        except OSError as exc:
            self._set_connected(False)
            self._log("WARN", f"connect failed to {self.host}:{self.port}: {exc}")
            self._socket = None

    def _flush_outbox(self) -> None:
        """Send all queued messages to the socket."""
        if self._socket is None:
            return
        while True:
            try:
                message = self._outbox.get_nowait()
            except queue.Empty:
                break
            payload = (json.dumps(message, ensure_ascii=False, separators=(",", ":")) + "\n").encode("utf-8")
            self._socket.sendall(payload)

    def _emit_message(self, message: Dict[str, Any]) -> None:
        """Invoke the registered message callback."""
        if self.on_message is not None:
            self.on_message(message)

    def _log(self, level: str, text: str) -> None:
        """Invoke the registered log callback."""
        if self.on_log is not None:
            self.on_log(level, text)

    def _set_connected(self, connected: bool) -> None:
        """Update and emit connection state if it changed."""
        if connected == self._connected:
            return
        self._connected = connected
        if self.on_connection_change is not None:
            self.on_connection_change(connected)

    def _close_socket(self) -> None:
        """Close the current socket if any."""
        if self._socket is not None:
            try:
                self._socket.close()
            except OSError:
                pass
            self._socket = None
        self._set_connected(False)


class QtGcsClientBridge(QObject):
    """Qt-facing wrapper exposing GcsTcpClient through signals and slots."""

    connectionChanged = Signal(bool)
    logLine = Signal(str, str)
    messageReceived = Signal(object)
    bridgeStatusReceived = Signal(object)
    robotMessageReceived = Signal(object)
    eventReceived = Signal(object)

    def __init__(self, host: str, port: int, heartbeat_interval: float = 1.0, reconnect_interval: float = 1.0) -> None:
        super().__init__()
        self.client = GcsTcpClient(host, port, heartbeat_interval, reconnect_interval)
        self.client.on_message = self._on_message
        self.client.on_log = self._on_log
        self.client.on_connection_change = self._on_connection_change

    @Slot()
    def start(self) -> None:
        """Start the underlying TCP client."""
        self.client.start()

    @Slot()
    def stop(self) -> None:
        """Stop the underlying TCP client."""
        self.client.stop()

    @Slot(str, int)
    def reconfigure(self, host: str, port: int) -> None:
        """Update host and port for the next reconnect."""
        self.client.reconfigure(host, port)

    @Slot(str)
    def send_mode(self, mode: str) -> None:
        """Send one mode command."""
        self.client.send_mode(mode)

    @Slot(float, float, float, int)
    def send_cmd_vel(self, vx: float, vy: float, wz: float, timeout_ms: int = 400) -> None:
        """Send one manual velocity command."""
        self.client.send_cmd_vel(vx, vy, wz, timeout_ms)

    @Slot(float, float, float)
    def send_pose(self, x: float, y: float, yaw: float) -> None:
        """Send one pose2d update."""
        self.client.send_pose(x, y, yaw)

    @Slot(object, float)
    def send_path(self, points: object, target_speed: float = 0.2) -> None:
        """Send one path message."""
        converted = [[float(p[0]), float(p[1])] for p in list(points)]
        self.client.send_path(converted, target_speed)

    @Slot()
    def request_status(self) -> None:
        """Request one bridge_status update."""
        self.client.request_status()

    def _on_message(self, message: Dict[str, Any]) -> None:
        """Dispatch raw incoming messages to more specific signals."""
        self.messageReceived.emit(message)
        mtype = message.get("type")
        if mtype == "bridge_status":
            self.bridgeStatusReceived.emit(message)
        elif mtype == "robot_message":
            self.robotMessageReceived.emit(message)
        elif mtype == "event":
            self.eventReceived.emit(message)

    def _on_log(self, level: str, text: str) -> None:
        """Forward a client log line to the GUI."""
        self.logLine.emit(level, text)

    def _on_connection_change(self, connected: bool) -> None:
        """Forward a connection state change to the GUI."""
        self.connectionChanged.emit(bool(connected))
