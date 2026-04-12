"""Ground station planning and protocol backend."""
from __future__ import annotations

import heapq
import json
import math
import queue
import socket
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from common.protocol import decode_message, encode_message, make_message, validate_telemetry


WorldPoint = Tuple[float, float]
LogCallback = Callable[[str, str], None]
TelemetryCallback = Callable[[Dict[str, Any]], None]
ConnectionCallback = Callable[[bool], None]


@dataclass
class ObstacleRect:
    """Axis-aligned obstacle rectangle in world coordinates."""

    x_min: float
    y_min: float
    x_max: float
    y_max: float

    def normalized(self) -> "ObstacleRect":
        return ObstacleRect(
            x_min=min(self.x_min, self.x_max),
            y_min=min(self.y_min, self.y_max),
            x_max=max(self.x_min, self.x_max),
            y_max=max(self.y_min, self.y_max),
        )


@dataclass
class PlannerOverlay:
    """Interactive overlays maintained by the frontend."""

    waypoints: List[WorldPoint] = field(default_factory=list)
    obstacles: List[ObstacleRect] = field(default_factory=list)
    path: List[WorldPoint] = field(default_factory=list)
    hovered_world: Optional[WorldPoint] = None


@dataclass
class PlannerConfig:
    """A* planner map and robot geometry settings."""

    x_min: float = -1.0
    x_max: float = 1.0
    y_min: float = -1.0
    y_max: float = 1.0
    resolution_m: float = 0.05
    robot_radius_m: float = 0.10


class AStarPlanner:
    """2D grid A* planner with obstacle inflation."""

    def __init__(self, config: PlannerConfig) -> None:
        self.config = config
        self.width = max(2, int(math.ceil((config.x_max - config.x_min) / config.resolution_m)))
        self.height = max(2, int(math.ceil((config.y_max - config.y_min) / config.resolution_m)))

    def plan(self, start: WorldPoint, goals: Sequence[WorldPoint], obstacles: Sequence[ObstacleRect]) -> List[WorldPoint]:
        if not goals:
            return [start]
        occupancy = self._build_occupancy(obstacles)
        total_path: List[WorldPoint] = [start]
        current = start
        for goal in goals:
            segment = self._plan_segment(current, goal, occupancy)
            if len(segment) <= 1:
                total_path.append(goal)
            else:
                total_path.extend(segment[1:])
            current = goal
        return self._simplify(total_path)

    def _plan_segment(self, start: WorldPoint, goal: WorldPoint, occupancy: List[List[bool]]) -> List[WorldPoint]:
        s = self._world_to_grid(*start)
        g = self._world_to_grid(*goal)
        if self._is_blocked(s, occupancy):
            raise ValueError("start cell is blocked")
        if self._is_blocked(g, occupancy):
            raise ValueError("goal cell is blocked")

        frontier: List[Tuple[float, Tuple[int, int]]] = [(0.0, s)]
        came_from: Dict[Tuple[int, int], Optional[Tuple[int, int]]] = {s: None}
        cost_so_far: Dict[Tuple[int, int], float] = {s: 0.0}

        while frontier:
            _, current = heapq.heappop(frontier)
            if current == g:
                break
            for nxt, step_cost in self._neighbors(current, occupancy):
                new_cost = cost_so_far[current] + step_cost
                if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                    cost_so_far[nxt] = new_cost
                    priority = new_cost + self._heuristic(nxt, g)
                    heapq.heappush(frontier, (priority, nxt))
                    came_from[nxt] = current

        if g not in came_from:
            raise ValueError("planner failed to find path")

        path_grid: List[Tuple[int, int]] = []
        cur: Optional[Tuple[int, int]] = g
        while cur is not None:
            path_grid.append(cur)
            cur = came_from[cur]
        path_grid.reverse()
        return [self._grid_to_world(ix, iy) for ix, iy in path_grid]

    def _build_occupancy(self, obstacles: Sequence[ObstacleRect]) -> List[List[bool]]:
        grid = [[False for _ in range(self.width)] for _ in range(self.height)]
        inflate = max(0, int(math.ceil(self.config.robot_radius_m / self.config.resolution_m)))
        for obs in obstacles:
            rect = obs.normalized()
            x0, y0 = self._world_to_grid(rect.x_min, rect.y_min)
            x1, y1 = self._world_to_grid(rect.x_max, rect.y_max)
            for iy in range(max(0, min(y0, y1) - inflate), min(self.height, max(y0, y1) + inflate + 1)):
                for ix in range(max(0, min(x0, x1) - inflate), min(self.width, max(x0, x1) + inflate + 1)):
                    grid[iy][ix] = True
        return grid

    def _neighbors(self, node: Tuple[int, int], occupancy: List[List[bool]]) -> List[Tuple[Tuple[int, int], float]]:
        x, y = node
        result: List[Tuple[Tuple[int, int], float]] = []
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.width and 0 <= ny < self.height and not occupancy[ny][nx]:
                    result.append(((nx, ny), math.hypot(dx, dy)))
        return result

    def _world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        ix = int((x - self.config.x_min) / self.config.resolution_m)
        iy = int((y - self.config.y_min) / self.config.resolution_m)
        ix = max(0, min(self.width - 1, ix))
        iy = max(0, min(self.height - 1, iy))
        return ix, iy

    def _grid_to_world(self, ix: int, iy: int) -> WorldPoint:
        x = self.config.x_min + (ix + 0.5) * self.config.resolution_m
        y = self.config.y_min + (iy + 0.5) * self.config.resolution_m
        return x, y

    @staticmethod
    def _heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def _is_blocked(node: Tuple[int, int], occupancy: List[List[bool]]) -> bool:
        return occupancy[node[1]][node[0]]

    @staticmethod
    def _simplify(points: Sequence[WorldPoint], epsilon: float = 0.03) -> List[WorldPoint]:
        pts = list(points)
        if len(pts) <= 2:
            return pts
        start = pts[0]
        end = pts[-1]
        max_dist = -1.0
        max_idx = -1
        for idx in range(1, len(pts) - 1):
            dist = _point_to_segment_distance(pts[idx], start, end)
            if dist > max_dist:
                max_dist = dist
                max_idx = idx
        if max_dist <= epsilon:
            return [start, end]
        left = AStarPlanner._simplify(pts[: max_idx + 1], epsilon)
        right = AStarPlanner._simplify(pts[max_idx:], epsilon)
        return left[:-1] + right


def _point_to_segment_distance(p: WorldPoint, a: WorldPoint, b: WorldPoint) -> float:
    ax, ay = a
    bx, by = b
    px, py = p
    dx = bx - ax
    dy = by - ay
    if dx == 0.0 and dy == 0.0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    cx = ax + t * dx
    cy = ay + t * dy
    return math.hypot(px - cx, py - cy)


class PlanningBackend:
    """Ground station backend for path planning and TCP protocol publishing."""

    def __init__(self, config: Dict[str, Any]) -> None:
        self.config = config
        planner_cfg = PlannerConfig(
            x_min=float(config["map"].get("x_min", -1.0)),
            x_max=float(config["map"].get("x_max", 1.0)),
            y_min=float(config["map"].get("y_min", -1.0)),
            y_max=float(config["map"].get("y_max", 1.0)),
            resolution_m=float(config["planning"].get("resolution_m", 0.05)),
            robot_radius_m=float(config["planning"].get("robot_radius_m", 0.10)),
        )
        self.planner = AStarPlanner(planner_cfg)

        self.sky_host = str(config.get("sky_host", "127.0.0.1"))
        self.sky_port = int(config.get("sky_port", 46001))
        self.reconnect_interval_sec = float(config.get("reconnect_interval_sec", 1.0))

        self.on_log: Optional[LogCallback] = None
        self.on_telemetry: Optional[TelemetryCallback] = None
        self.on_connection: Optional[ConnectionCallback] = None

        self._seq = 1
        self._outbox: "queue.Queue[Dict[str, Any]]" = queue.Queue()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._socket: Optional[socket.socket] = None
        self._connected = False

    def start(self) -> None:
        """Start networking thread."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop networking thread and close socket."""
        self._stop_event.set()
        self._close_socket()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def next_seq(self) -> int:
        value = self._seq
        self._seq += 1
        return value

    def plan(self, start: WorldPoint, goals: Sequence[WorldPoint], obstacles: Sequence[ObstacleRect]) -> List[WorldPoint]:
        """Run A* path planner."""
        return self.planner.plan(start, goals, obstacles)

    def send_path(self, points: Sequence[WorldPoint], target_speed: float) -> None:
        """Queue a path message to sky_uav."""
        payload = make_message(
            "path",
            seq=self.next_seq(),
            points=[[float(p[0]), float(p[1])] for p in points],
            target_speed=float(target_speed),
        )
        self._outbox.put(payload)

    def request_status(self) -> None:
        """Request one telemetry push."""
        self._outbox.put(make_message("request_status", seq=self.next_seq()))

    def _run(self) -> None:
        buffer = b""
        while not self._stop_event.is_set():
            if self._socket is None:
                self._connect_once()
                buffer = b""
                if self._socket is None:
                    time.sleep(self.reconnect_interval_sec)
                    continue
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
                    message = decode_message(line)
                    self._dispatch_message(message)
            except socket.timeout:
                continue
            except (OSError, ValueError, ConnectionError) as exc:
                self._log("WARN", f"sky connection dropped: {exc}")
                self._close_socket()
                time.sleep(self.reconnect_interval_sec)

    def _connect_once(self) -> None:
        try:
            sock = socket.create_connection((self.sky_host, self.sky_port), timeout=2.0)
            sock.settimeout(0.2)
            self._socket = sock
            self._set_connected(True)
            self._log("INFO", f"connected to sky_uav {self.sky_host}:{self.sky_port}")
        except OSError as exc:
            self._set_connected(False)
            self._log("WARN", f"connect failed {self.sky_host}:{self.sky_port}: {exc}")
            self._socket = None

    def _flush_outbox(self) -> None:
        if self._socket is None:
            return
        while True:
            try:
                message = self._outbox.get_nowait()
            except queue.Empty:
                return
            self._socket.sendall(encode_message(message))

    def _dispatch_message(self, message: Dict[str, Any]) -> None:
        mtype = message.get("type")
        if mtype == "telemetry":
            try:
                validate_telemetry(message)
            except Exception as exc:
                self._log("WARN", f"invalid telemetry: {exc}")
                return
            if self.on_telemetry is not None:
                self.on_telemetry(message)
        elif mtype == "ack":
            self._log("INFO" if message.get("ok", False) else "WARN", f"ack seq={message.get('ack_seq')} msg={message.get('msg', '')}")
        elif mtype == "event":
            self._log(str(message.get("level", "INFO")), str(message.get("text", "")))
        elif mtype == "robot_message":
            self._log("INFO", f"robot: {message.get('payload', {})}")

    def _set_connected(self, connected: bool) -> None:
        if self._connected == connected:
            return
        self._connected = connected
        if self.on_connection is not None:
            self.on_connection(connected)

    def _close_socket(self) -> None:
        if self._socket is not None:
            try:
                self._socket.close()
            except OSError:
                pass
            self._socket = None
        self._set_connected(False)

    def _log(self, level: str, text: str) -> None:
        if self.on_log is not None:
            self.on_log(level, text)


DEFAULT_STATION_CONFIG: Dict[str, Any] = {
    "sky_host": "127.0.0.1",
    "sky_port": 46001,
    "reconnect_interval_sec": 1.0,
    "target_speed": 0.2,
    "rtsp_url": "rtsp://127.0.0.1:8554/video",
    "rtsp_pipeline": "",
    "video_fps_limit": 20.0,
    "bev_process_every_n": 2,
    "map": {
        "x_min": -1.0,
        "x_max": 1.0,
        "y_min": -1.0,
        "y_max": 1.0,
        "ppm": 300,
    },
    "planning": {
        "resolution_m": 0.05,
        "robot_radius_m": 0.10,
    },
    "bev": {
        "enabled": True,
        "calibration_path": "config/calibration.yaml",
        "target_id": 0,
        "tag_size": 0.09,
        "x_min": -1.0,
        "x_max": 1.0,
        "y_min": -1.0,
        "y_max": 1.0,
        "ppm": 300,
    },
}


def load_station_config(path: str) -> Dict[str, Any]:
    """Load and merge ground-station JSON config."""
    raw: Dict[str, Any] = {}
    config_path = Path(path)
    if config_path.exists():
        raw = json.loads(config_path.read_text(encoding="utf-8"))
    cfg = json.loads(json.dumps(DEFAULT_STATION_CONFIG))
    for key in ("sky_host", "sky_port", "reconnect_interval_sec", "target_speed", "rtsp_url", "rtsp_pipeline", "video_fps_limit", "bev_process_every_n"):
        if key in raw:
            cfg[key] = raw[key]
    for key in ("map", "planning", "bev"):
        if key in raw and isinstance(raw[key], dict):
            cfg[key].update(raw[key])
    return cfg
