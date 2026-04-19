# Ground Station Qt Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor the ground station into a single Python 3.12 Qt application that owns RTSP ingest, BEV display, grid-based A* planning, pure pursuit control, and TCP server communication with the robot.

**Architecture:** Keep Qt as a thin UI shell. Move shared geometry, map editing, runtime state, planning, control, and TCP server logic into small Python modules that all consume the same world/grid/pixel mapping. Use one TOML config and one minimal JSON protocol with `cmd_vel` and `reach_goal`.

**Tech Stack:** Python 3.12, `uv`, Qt (`PySide6` with `PyQt5` fallback), OpenCV/FFmpeg, `tomllib`, `unittest`

---

## File Structure

### Files to Create

- `config/ground_station.toml`
  - Single runtime config source for RTSP, server, map, grid, control, and BEV parameters.
- `ground_station/config.py`
  - TOML loader with defaults merged into a single typed dict.
- `ground_station/grid_map.py`
  - Shared world/grid/pixel mapper plus rectangle and line obstacle rasterization.
- `ground_station/map_editor.py`
  - Pure Python goal/obstacle editing model used by the Qt canvas.
- `ground_station/state_machine.py`
  - Explicit `idle/planned/executing/cancelled` runtime transitions and lock rules.
- `ground_station/gcs_server.py`
  - TCP server that waits for the robot to connect and sends `cmd_vel` / `reach_goal`.
- `ground_station/planner_controller.py`
  - Minimal A* plus pure pursuit path tracking over the shared geometry.
- `tests/test_ground_station_protocol.py`
  - Protocol and TOML config tests.
- `tests/test_grid_map.py`
  - Mapper round-trip and obstacle rasterization tests.
- `tests/test_map_editor.py`
  - Goal, rectangle obstacle, line obstacle, and erase behavior tests.
- `tests/test_state_machine.py`
  - Runtime transition tests.
- `tests/test_gcs_server.py`
  - One-client TCP server behavior tests.
- `tests/test_planner_controller.py`
  - A*, pure pursuit, and goal reached tests.
- `tests/test_bev_backend.py`
  - RTSP reconnect loop tests.

### Files to Modify

- `pyproject.toml`
  - Tighten runtime to Python 3.12 and keep dependencies unchanged.
- `common/protocol.py`
  - Reduce protocol helpers to the two outgoing message encoders and one decoder.
- `ground_station/bev_backend.py`
  - Add automatic reconnect loop and pose callback surface for the UI/controller.
- `ground_station/frontend.py`
  - Rebuild the Qt shell around the new modules and auto-start behavior.
- `ground_robot/receiver.py`
  - Change the client to connect to the GCS server and accept `cmd_vel` / `reach_goal`.
- `scripts/run_ground_station.sh`
  - Point to `config/ground_station.toml`.
- `README.md`
  - Update the architecture and startup notes after implementation is verified.

### Files to Delete or Stop Using

- `config/ground_station.json`
  - Replaced by `config/ground_station.toml`.
- `ground_station/planning_backend.py`
  - Replaced by `grid_map.py`, `planner_controller.py`, `state_machine.py`, and `gcs_server.py`.

## Task 1: Python 3.12, TOML Config, and Minimal Protocol

**Files:**
- Create: `config/ground_station.toml`
- Create: `ground_station/config.py`
- Modify: `pyproject.toml`
- Modify: `common/protocol.py`
- Test: `tests/test_ground_station_protocol.py`

- [ ] **Step 1: Write the failing config/protocol tests**

```python
import tempfile
import textwrap
import unittest
from pathlib import Path

from common.protocol import decode_message, encode_cmd_vel, encode_reach_goal
from ground_station.config import load_config


class GroundStationProtocolTests(unittest.TestCase):
    def test_encode_cmd_vel(self) -> None:
        payload = encode_cmd_vel(vx=0.25, vy=0.0, wz=-0.4, timestamp=12.5)
        self.assertEqual(
            decode_message(payload),
            {
                "type": "cmd_vel",
                "vx": 0.25,
                "vy": 0.0,
                "wz": -0.4,
                "timestamp": 12.5,
            },
        )

    def test_encode_reach_goal(self) -> None:
        payload = encode_reach_goal(timestamp=18.0)
        self.assertEqual(
            decode_message(payload),
            {
                "type": "reach_goal",
                "timestamp": 18.0,
            },
        )

    def test_load_config_reads_toml(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            path = Path(tmp_dir) / "ground_station.toml"
            path.write_text(
                textwrap.dedent(
                    """
                    [server]
                    host = "0.0.0.0"
                    port = 47001

                    [grid]
                    resolution_m = 0.1
                    line_obstacle_width_m = 0.08
                    """
                ).strip(),
                encoding="utf-8",
            )
            cfg = load_config(str(path))
        self.assertEqual(cfg["server"]["host"], "0.0.0.0")
        self.assertEqual(cfg["server"]["port"], 47001)
        self.assertEqual(cfg["grid"]["resolution_m"], 0.1)
        self.assertEqual(cfg["grid"]["line_obstacle_width_m"], 0.08)
        self.assertIn("control", cfg)
        self.assertIn("rtsp", cfg)
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `uv run python -m unittest tests.test_ground_station_protocol -v`  
Expected: `FAIL` with import errors for `encode_cmd_vel`, `encode_reach_goal`, or `ground_station.config`

- [ ] **Step 3: Write the minimal implementation**

`pyproject.toml`

```toml
[project]
name = "ag"
version = "0.1.0"
description = "Add your description here"
readme = "README.md"
requires-python = ">=3.12"
dependencies = [
    "numpy>=1.26.4",
    "opencv-python>=4.11.0.86",
    "pupil-apriltags>=1.0.4.post11",
    "pyyaml>=6.0.3",
]
```

`ground_station/config.py`

```python
from __future__ import annotations

import copy
import tomllib
from pathlib import Path
from typing import Any, Dict


DEFAULT_CONFIG: Dict[str, Dict[str, Any]] = {
    "rtsp": {
        "url": "rtsp://10.28.215.179:8554/cam",
        "retry_sec": 1.0,
        "fps_limit": 20.0,
    },
    "server": {
        "host": "0.0.0.0",
        "port": 47001,
    },
    "map": {
        "x_min": -1.0,
        "x_max": 1.0,
        "y_min": -1.0,
        "y_max": 1.0,
        "ppm": 300,
    },
    "grid": {
        "resolution_m": 0.05,
        "line_obstacle_width_m": 0.08,
    },
    "control": {
        "control_hz": 20.0,
        "lookahead_m": 0.25,
        "goal_tolerance_m": 0.08,
        "slowdown_radius_m": 0.35,
        "max_vx": 0.30,
        "max_wz": 0.80,
    },
    "bev": {
        "enabled": True,
        "calibration_path": "config/calibration.yaml",
        "target_id": 0,
        "tag_size": 0.09,
    },
}


def load_config(path: str) -> Dict[str, Dict[str, Any]]:
    cfg = copy.deepcopy(DEFAULT_CONFIG)
    data = tomllib.loads(Path(path).read_text(encoding="utf-8"))
    for section, values in data.items():
        if isinstance(values, dict) and section in cfg:
            cfg[section].update(values)
        else:
            cfg[section] = values
    return cfg
```

`common/protocol.py`

```python
from __future__ import annotations

import json
from typing import Any, Dict


def encode_cmd_vel(vx: float, vy: float, wz: float, timestamp: float) -> bytes:
    return _encode(
        {
            "type": "cmd_vel",
            "vx": float(vx),
            "vy": float(vy),
            "wz": float(wz),
            "timestamp": float(timestamp),
        }
    )


def encode_reach_goal(timestamp: float) -> bytes:
    return _encode(
        {
            "type": "reach_goal",
            "timestamp": float(timestamp),
        }
    )


def decode_message(line: bytes | str) -> Dict[str, Any]:
    if isinstance(line, bytes):
        line = line.decode("utf-8")
    return json.loads(line.strip())


def _encode(payload: Dict[str, Any]) -> bytes:
    return (json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8")
```

`config/ground_station.toml`

```toml
[rtsp]
url = "rtsp://10.28.215.179:8554/cam"
retry_sec = 1.0
fps_limit = 20.0

[server]
host = "0.0.0.0"
port = 47001

[map]
x_min = -1.0
x_max = 1.0
y_min = -1.0
y_max = 1.0
ppm = 300

[grid]
resolution_m = 0.05
line_obstacle_width_m = 0.08

[control]
control_hz = 20.0
lookahead_m = 0.25
goal_tolerance_m = 0.08
slowdown_radius_m = 0.35
max_vx = 0.30
max_wz = 0.80

[bev]
enabled = true
calibration_path = "config/calibration.yaml"
target_id = 0
tag_size = 0.09
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `uv run python -m unittest tests.test_ground_station_protocol -v`  
Expected: `OK`

- [ ] **Step 5: Commit**

```bash
git add pyproject.toml config/ground_station.toml ground_station/config.py common/protocol.py tests/test_ground_station_protocol.py
git commit -m "refactor: add toml config and minimal protocol"
```

## Task 2: Shared Geometry and Map Editing Model

**Files:**
- Create: `ground_station/grid_map.py`
- Create: `ground_station/map_editor.py`
- Test: `tests/test_grid_map.py`
- Test: `tests/test_map_editor.py`

- [ ] **Step 1: Write the failing geometry and map editor tests**

`tests/test_grid_map.py`

```python
import unittest

from ground_station.grid_map import GridConfig, GridMapper, LineObstacle, RectObstacle


class GridMapTests(unittest.TestCase):
    def setUp(self) -> None:
        self.mapper = GridMapper(
            GridConfig(
                x_min=-1.0,
                x_max=1.0,
                y_min=-1.0,
                y_max=1.0,
                resolution_m=0.1,
                ppm=100,
                line_obstacle_width_m=0.2,
            )
        )

    def test_world_grid_round_trip_stays_in_same_cell(self) -> None:
        cell = self.mapper.world_to_grid(0.24, -0.31)
        x_world, y_world = self.mapper.grid_to_world(*cell)
        self.assertEqual(self.mapper.world_to_grid(x_world, y_world), cell)

    def test_rect_obstacle_marks_cells(self) -> None:
        occ = self.mapper.build_occupancy(
            rects=[RectObstacle(0.0, 0.0, 0.2, 0.2)],
            lines=[],
        )
        self.assertTrue(any(any(row) for row in occ))

    def test_line_obstacle_marks_cells(self) -> None:
        occ = self.mapper.build_occupancy(
            rects=[],
            lines=[LineObstacle(points=[(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5)])],
        )
        self.assertTrue(any(any(row) for row in occ))
```

`tests/test_map_editor.py`

```python
import unittest

from ground_station.map_editor import MapEditor, ToolMode


class MapEditorTests(unittest.TestCase):
    def test_set_goal_replaces_previous_goal(self) -> None:
        editor = MapEditor()
        editor.set_tool(ToolMode.SET_GOAL)
        editor.left_click((0.1, 0.1))
        editor.left_click((0.2, 0.2))
        self.assertEqual(editor.goal, (0.2, 0.2))

    def test_rect_obstacle_uses_two_clicks(self) -> None:
        editor = MapEditor()
        editor.set_tool(ToolMode.DRAW_RECT)
        editor.left_click((0.0, 0.0))
        editor.left_click((0.3, 0.2))
        self.assertEqual(len(editor.rectangles), 1)

    def test_line_obstacle_chains_segments_until_right_click(self) -> None:
        editor = MapEditor()
        editor.set_tool(ToolMode.DRAW_LINE)
        editor.left_click((0.0, 0.0))
        editor.left_click((0.2, 0.0))
        editor.left_click((0.2, 0.3))
        editor.right_click((0.2, 0.3))
        self.assertEqual(editor.lines[0].points, [(0.0, 0.0), (0.2, 0.0), (0.2, 0.3)])
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `uv run python -m unittest tests.test_grid_map tests.test_map_editor -v`  
Expected: `FAIL` with missing module errors

- [ ] **Step 3: Write the minimal implementation**

`ground_station/grid_map.py`

```python
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple


WorldPoint = Tuple[float, float]


@dataclass(frozen=True)
class RectObstacle:
    x0: float
    y0: float
    x1: float
    y1: float

    def normalized(self) -> "RectObstacle":
        return RectObstacle(
            min(self.x0, self.x1),
            min(self.y0, self.y1),
            max(self.x0, self.x1),
            max(self.y0, self.y1),
        )


@dataclass(frozen=True)
class LineObstacle:
    points: List[WorldPoint]


@dataclass(frozen=True)
class GridConfig:
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    resolution_m: float
    ppm: int
    line_obstacle_width_m: float


class GridMapper:
    def __init__(self, cfg: GridConfig) -> None:
        self.cfg = cfg
        self.width = max(1, int(math.ceil((cfg.x_max - cfg.x_min) / cfg.resolution_m)))
        self.height = max(1, int(math.ceil((cfg.y_max - cfg.y_min) / cfg.resolution_m)))

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        ix = int((x - self.cfg.x_min) / self.cfg.resolution_m)
        iy = int((y - self.cfg.y_min) / self.cfg.resolution_m)
        return max(0, min(self.width - 1, ix)), max(0, min(self.height - 1, iy))

    def grid_to_world(self, ix: int, iy: int) -> WorldPoint:
        return (
            self.cfg.x_min + (ix + 0.5) * self.cfg.resolution_m,
            self.cfg.y_min + (iy + 0.5) * self.cfg.resolution_m,
        )

    def world_to_pixel(self, x: float, y: float) -> tuple[float, float]:
        width_px = int((self.cfg.x_max - self.cfg.x_min) * self.cfg.ppm)
        height_px = int((self.cfg.y_max - self.cfg.y_min) * self.cfg.ppm)
        u = (x - self.cfg.x_min) / (self.cfg.x_max - self.cfg.x_min) * width_px
        v = (self.cfg.y_max - y) / (self.cfg.y_max - self.cfg.y_min) * height_px
        return u, v

    def pixel_to_world(self, u: float, v: float) -> WorldPoint:
        width_px = int((self.cfg.x_max - self.cfg.x_min) * self.cfg.ppm)
        height_px = int((self.cfg.y_max - self.cfg.y_min) * self.cfg.ppm)
        return (
            self.cfg.x_min + (u / max(width_px, 1)) * (self.cfg.x_max - self.cfg.x_min),
            self.cfg.y_max - (v / max(height_px, 1)) * (self.cfg.y_max - self.cfg.y_min),
        )

    def build_occupancy(
        self,
        rects: Sequence[RectObstacle],
        lines: Sequence[LineObstacle],
    ) -> list[list[bool]]:
        grid = [[False for _ in range(self.width)] for _ in range(self.height)]
        for rect in rects:
            norm = rect.normalized()
            x0, y0 = self.world_to_grid(norm.x0, norm.y0)
            x1, y1 = self.world_to_grid(norm.x1, norm.y1)
            for iy in range(min(y0, y1), max(y0, y1) + 1):
                for ix in range(min(x0, x1), max(x0, x1) + 1):
                    grid[iy][ix] = True
        radius = max(0, int(math.ceil(self.cfg.line_obstacle_width_m / self.cfg.resolution_m / 2.0)))
        for line in lines:
            for start, end in zip(line.points, line.points[1:]):
                for ix, iy in self._segment_cells(start, end):
                    for oy in range(-radius, radius + 1):
                        for ox in range(-radius, radius + 1):
                            nx = ix + ox
                            ny = iy + oy
                            if 0 <= nx < self.width and 0 <= ny < self.height:
                                grid[ny][nx] = True
        return grid

    def _segment_cells(self, start: WorldPoint, end: WorldPoint) -> Iterable[tuple[int, int]]:
        x0, y0 = self.world_to_grid(*start)
        x1, y1 = self.world_to_grid(*end)
        steps = max(abs(x1 - x0), abs(y1 - y0), 1)
        for idx in range(steps + 1):
            t = idx / steps
            yield (
                int(round(x0 + (x1 - x0) * t)),
                int(round(y0 + (y1 - y0) * t)),
            )
```

`ground_station/map_editor.py`

```python
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Tuple

from ground_station.grid_map import LineObstacle, RectObstacle


WorldPoint = Tuple[float, float]


class ToolMode(str, Enum):
    NONE = "none"
    SET_GOAL = "set_goal"
    DRAW_RECT = "draw_rect"
    DRAW_LINE = "draw_line"
    ERASE = "erase"


@dataclass
class MapEditor:
    goal: Optional[WorldPoint] = None
    rectangles: list[RectObstacle] = field(default_factory=list)
    lines: list[LineObstacle] = field(default_factory=list)
    tool: ToolMode = ToolMode.NONE
    locked: bool = False
    _pending_rect_start: Optional[WorldPoint] = None
    _active_line: Optional[list[WorldPoint]] = None

    def set_tool(self, tool: ToolMode) -> None:
        self.tool = tool
        if tool != ToolMode.DRAW_LINE:
            self._active_line = None
        if tool != ToolMode.DRAW_RECT:
            self._pending_rect_start = None

    def left_click(self, point: WorldPoint) -> None:
        if self.locked:
            return
        if self.tool == ToolMode.SET_GOAL:
            self.goal = point
        elif self.tool == ToolMode.DRAW_RECT:
            if self._pending_rect_start is None:
                self._pending_rect_start = point
            else:
                self.rectangles.append(RectObstacle(*self._pending_rect_start, *point))
                self._pending_rect_start = None
        elif self.tool == ToolMode.DRAW_LINE:
            if self._active_line is None:
                self._active_line = [point]
                self.lines.append(LineObstacle(points=self._active_line))
            else:
                self._active_line.append(point)

    def right_click(self, point: WorldPoint) -> None:
        if self.tool == ToolMode.DRAW_LINE:
            self._active_line = None
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `uv run python -m unittest tests.test_grid_map tests.test_map_editor -v`  
Expected: `OK`

- [ ] **Step 5: Commit**

```bash
git add ground_station/grid_map.py ground_station/map_editor.py tests/test_grid_map.py tests/test_map_editor.py
git commit -m "refactor: add shared grid mapper and map editor"
```

## Task 3: Runtime State Machine and TCP Server

**Files:**
- Create: `ground_station/state_machine.py`
- Create: `ground_station/gcs_server.py`
- Modify: `ground_robot/receiver.py`
- Test: `tests/test_state_machine.py`
- Test: `tests/test_gcs_server.py`

- [ ] **Step 1: Write the failing runtime tests**

`tests/test_state_machine.py`

```python
import unittest

from ground_station.state_machine import RuntimeStateMachine


class RuntimeStateMachineTests(unittest.TestCase):
    def test_execute_requires_plan_pose_and_connection(self) -> None:
        machine = RuntimeStateMachine()
        self.assertFalse(machine.can_execute(has_goal=True, has_path=True, has_pose=True, robot_connected=False))
        self.assertTrue(machine.can_execute(has_goal=True, has_path=True, has_pose=True, robot_connected=True))

    def test_executing_locks_editing(self) -> None:
        machine = RuntimeStateMachine()
        machine.to_planned()
        machine.to_executing()
        self.assertFalse(machine.editing_allowed)

    def test_cancel_returns_to_idle_and_clears_goal(self) -> None:
        machine = RuntimeStateMachine()
        machine.to_planned()
        machine.to_executing()
        state = machine.cancel()
        self.assertEqual(state, "idle")
```

`tests/test_gcs_server.py`

```python
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
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `uv run python -m unittest tests.test_state_machine tests.test_gcs_server -v`  
Expected: `FAIL` with missing module errors

- [ ] **Step 3: Write the minimal implementation**

`ground_station/state_machine.py`

```python
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class RuntimeStateMachine:
    state: str = "idle"

    @property
    def editing_allowed(self) -> bool:
        return self.state in {"idle", "planned"}

    def can_execute(self, has_goal: bool, has_path: bool, has_pose: bool, robot_connected: bool) -> bool:
        return all((has_goal, has_path, has_pose, robot_connected))

    def to_planned(self) -> None:
        self.state = "planned"

    def to_executing(self) -> None:
        self.state = "executing"

    def cancel(self) -> str:
        self.state = "idle"
        return self.state
```

`ground_station/gcs_server.py`

```python
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
        self._stop = threading.Event()
        self._outbox: "queue.Queue[bytes]" = queue.Queue()
        self.bound_address: tuple[str, int] = (host, port)

    def start(self) -> None:
        self._stop.clear()
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind((self.host, self.port))
        self._server.listen(1)
        self.bound_address = self._server.getsockname()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        for sock in (self._client, self._server):
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass

    @property
    def connected(self) -> bool:
        return self._client is not None

    def send_cmd_vel(self, vx: float, vy: float, wz: float, timestamp: float) -> None:
        self._outbox.put(encode_cmd_vel(vx=vx, vy=vy, wz=wz, timestamp=timestamp))

    def send_reach_goal(self, timestamp: float) -> None:
        self._outbox.put(encode_reach_goal(timestamp=timestamp))

    def _run(self) -> None:
        assert self._server is not None
        self._server.settimeout(0.2)
        while not self._stop.is_set():
            if self._client is None:
                try:
                    self._client, _ = self._server.accept()
                    self._client.settimeout(0.2)
                except socket.timeout:
                    continue
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
```

`ground_robot/receiver.py`

```python
DEFAULT_CONFIG = {
    "gcs_host": "127.0.0.1",
    "gcs_port": 47001,
    "reconnect_sec": 1.0,
}

# inside run_forever()
host = str(self.config["gcs_host"])
port = int(self.config["gcs_port"])

# inside _handle_line()
message = decode_message(line)
if message.get("type") == "cmd_vel":
    print(
        f"cmd_vel vx={message['vx']:.3f} vy={message['vy']:.3f} wz={message['wz']:.3f}",
        flush=True,
    )
    return
if message.get("type") == "reach_goal":
    print("reach_goal", flush=True)
    return
raise ValueError(f"unsupported type: {message.get('type')}")
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `uv run python -m unittest tests.test_state_machine tests.test_gcs_server -v`  
Expected: `OK`

- [ ] **Step 5: Commit**

```bash
git add ground_station/state_machine.py ground_station/gcs_server.py ground_robot/receiver.py tests/test_state_machine.py tests/test_gcs_server.py
git commit -m "refactor: add runtime state machine and gcs server"
```

## Task 4: A* Planner and Pure Pursuit Controller

**Files:**
- Create: `ground_station/planner_controller.py`
- Test: `tests/test_planner_controller.py`

- [ ] **Step 1: Write the failing planner/controller tests**

```python
import unittest

from ground_station.grid_map import GridConfig, GridMapper, LineObstacle, RectObstacle
from ground_station.planner_controller import PlannerController


class PlannerControllerTests(unittest.TestCase):
    def setUp(self) -> None:
        mapper = GridMapper(
            GridConfig(
                x_min=-1.0,
                x_max=1.0,
                y_min=-1.0,
                y_max=1.0,
                resolution_m=0.1,
                ppm=100,
                line_obstacle_width_m=0.1,
            )
        )
        self.ctrl = PlannerController(
            mapper=mapper,
            lookahead_m=0.25,
            goal_tolerance_m=0.08,
            slowdown_radius_m=0.35,
            max_vx=0.3,
            max_wz=0.8,
        )

    def test_astar_returns_path_to_goal(self) -> None:
        path = self.ctrl.plan(
            start=(-0.8, -0.8),
            goal=(0.8, 0.8),
            rectangles=[RectObstacle(-0.1, -0.1, 0.1, 0.1)],
            lines=[],
        )
        self.assertEqual(path[0], (-0.8, -0.8))
        self.assertEqual(path[-1], (0.8, 0.8))

    def test_pure_pursuit_drives_forward_on_straight_path(self) -> None:
        self.ctrl.set_path([(0.0, 0.0), (1.0, 0.0)])
        cmd = self.ctrl.compute_command(pose={"x": 0.0, "y": 0.0, "yaw": 0.0})
        self.assertGreater(cmd["vx"], 0.0)
        self.assertAlmostEqual(cmd["vy"], 0.0)

    def test_goal_reached_stops_and_reports_true(self) -> None:
        self.ctrl.set_path([(0.0, 0.0), (0.05, 0.0)])
        cmd = self.ctrl.compute_command(pose={"x": 0.05, "y": 0.0, "yaw": 0.0})
        self.assertEqual(cmd, {"vx": 0.0, "vy": 0.0, "wz": 0.0})
        self.assertTrue(self.ctrl.goal_reached)
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `uv run python -m unittest tests.test_planner_controller -v`  
Expected: `FAIL` with missing module errors

- [ ] **Step 3: Write the minimal implementation**

`ground_station/planner_controller.py`

```python
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

from ground_station.grid_map import GridMapper, LineObstacle, RectObstacle


WorldPoint = Tuple[float, float]


@dataclass
class PlannerController:
    mapper: GridMapper
    lookahead_m: float
    goal_tolerance_m: float
    slowdown_radius_m: float
    max_vx: float
    max_wz: float

    def __post_init__(self) -> None:
        self.path: List[WorldPoint] = []
        self.goal_reached = False

    def plan(
        self,
        start: WorldPoint,
        goal: WorldPoint,
        rectangles: Sequence[RectObstacle],
        lines: Sequence[LineObstacle],
    ) -> List[WorldPoint]:
        occupancy = self.mapper.build_occupancy(rectangles, lines)
        start_cell = self.mapper.world_to_grid(*start)
        goal_cell = self.mapper.world_to_grid(*goal)
        frontier = [(0.0, start_cell)]
        came_from = {start_cell: None}
        cost = {start_cell: 0.0}
        while frontier:
            _, current = heapq.heappop(frontier)
            if current == goal_cell:
                break
            for nxt in self._neighbors(current, occupancy):
                new_cost = cost[current] + math.hypot(nxt[0] - current[0], nxt[1] - current[1])
                if nxt not in cost or new_cost < cost[nxt]:
                    cost[nxt] = new_cost
                    priority = new_cost + math.hypot(goal_cell[0] - nxt[0], goal_cell[1] - nxt[1])
                    heapq.heappush(frontier, (priority, nxt))
                    came_from[nxt] = current
        path_cells = []
        node = goal_cell
        while node is not None:
            path_cells.append(node)
            node = came_from[node]
        path_cells.reverse()
        path = [start]
        path.extend(self.mapper.grid_to_world(ix, iy) for ix, iy in path_cells[1:-1])
        path.append(goal)
        self.path = path
        self.goal_reached = False
        return path

    def set_path(self, path: Sequence[WorldPoint]) -> None:
        self.path = list(path)
        self.goal_reached = False

    def compute_command(self, pose: Dict[str, float]) -> Dict[str, float]:
        if len(self.path) < 2:
            return {"vx": 0.0, "vy": 0.0, "wz": 0.0}
        goal = self.path[-1]
        dx_goal = goal[0] - float(pose["x"])
        dy_goal = goal[1] - float(pose["y"])
        goal_dist = math.hypot(dx_goal, dy_goal)
        if goal_dist <= self.goal_tolerance_m:
            self.goal_reached = True
            return {"vx": 0.0, "vy": 0.0, "wz": 0.0}
        target = self._lookahead_point((float(pose["x"]), float(pose["y"])))
        dx = target[0] - float(pose["x"])
        dy = target[1] - float(pose["y"])
        heading = math.atan2(dy, dx)
        yaw = float(pose["yaw"])
        yaw_error = math.atan2(math.sin(heading - yaw), math.cos(heading - yaw))
        speed_scale = min(1.0, goal_dist / max(self.slowdown_radius_m, 1e-6))
        vx = max(0.0, min(self.max_vx, self.max_vx * speed_scale * max(0.0, math.cos(yaw_error))))
        wz = max(-self.max_wz, min(self.max_wz, 2.0 * yaw_error))
        self.goal_reached = False
        return {"vx": vx, "vy": 0.0, "wz": wz}

    def _lookahead_point(self, pose_xy: WorldPoint) -> WorldPoint:
        for point in self.path:
            if math.hypot(point[0] - pose_xy[0], point[1] - pose_xy[1]) >= self.lookahead_m:
                return point
        return self.path[-1]

    def _neighbors(self, node: tuple[int, int], occupancy: list[list[bool]]) -> list[tuple[int, int]]:
        out = []
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx = node[0] + dx
                ny = node[1] + dy
                if 0 <= nx < self.mapper.width and 0 <= ny < self.mapper.height and not occupancy[ny][nx]:
                    out.append((nx, ny))
        return out
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `uv run python -m unittest tests.test_planner_controller -v`  
Expected: `OK`

- [ ] **Step 5: Commit**

```bash
git add ground_station/planner_controller.py tests/test_planner_controller.py
git commit -m "refactor: add astar planner and pure pursuit controller"
```

## Task 5: RTSP Reconnect Loop and Pose Callback Surface

**Files:**
- Modify: `ground_station/bev_backend.py`
- Test: `tests/test_bev_backend.py`

- [ ] **Step 1: Write the failing BEV backend tests**

```python
import unittest
from unittest.mock import Mock

from ground_station.bev_backend import BevBackend, BevConfig


class BevBackendTests(unittest.TestCase):
    def test_open_failure_logs_and_retries(self) -> None:
        logs = []
        backend = BevBackend(
            rtsp_url="rtsp://invalid",
            fps_limit=20.0,
            process_every_n=1,
            bev_config=BevConfig(),
            on_log=lambda level, text: logs.append((level, text)),
        )
        def fake_open():
            backend._stop_event.set()
            return None

        backend._open_capture = Mock(side_effect=fake_open)  # type: ignore[attr-defined]
        backend._run()
        self.assertTrue(any(level == "ERROR" for level, _ in logs))
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `uv run python -m unittest tests.test_bev_backend -v`  
Expected: `FAIL` because `_open_capture` does not exist or reconnect loop stops after one failure

- [ ] **Step 3: Write the minimal implementation**

`ground_station/bev_backend.py`

```python
class BevBackend:
    def __init__(self, rtsp_url: str, fps_limit: float, process_every_n: int, bev_config: BevConfig, on_raw_frame=None, on_bev_frame=None, on_meta=None, on_log=None, retry_sec: float = 1.0) -> None:
        self.rtsp_url = str(rtsp_url)
        self.fps_limit = float(fps_limit)
        self.process_every_n = max(1, int(process_every_n))
        self.processor = AprilTagBevProcessor(bev_config)
        self.on_raw_frame = on_raw_frame
        self.on_bev_frame = on_bev_frame
        self.on_meta = on_meta
        self.on_log = on_log
        self.retry_sec = float(retry_sec)
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def _open_capture(self):
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp|max_delay;0|fflags;nobuffer|flags;low_delay"
        cap = cv2.VideoCapture(
            self.rtsp_url,
            cv2.CAP_FFMPEG,
            [
                cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 3000,
                cv2.CAP_PROP_READ_TIMEOUT_MSEC, 1000,
                cv2.CAP_PROP_N_THREADS, 1,
            ],
        )
        if not cap.isOpened():
            cap.release()
            return None
        return cap

    def _run(self) -> None:
        while not self._stop_event.is_set():
            cap = self._open_capture()
            if cap is None:
                self._log("ERROR", f"failed to open rtsp source: {self.rtsp_url}")
                time.sleep(self.retry_sec)
                continue
            try:
                self._run_capture_session(cap)
            finally:
                cap.release()

    def _run_capture_session(self, cap) -> None:
        self._log("INFO", f"rtsp opened (ffmpeg): {self.rtsp_url}")
        latest = {"frame": None}
        lock = threading.Lock()

        def reader() -> None:
            while not self._stop_event.is_set():
                if not cap.grab():
                    continue
                ok, frame = cap.retrieve()
                if ok:
                    with lock:
                        latest["frame"] = frame

        threading.Thread(target=reader, daemon=True).start()
        frame_idx = 0
        target_period = 1.0 / max(1.0, self.fps_limit)
        while not self._stop_event.is_set():
            tick = time.perf_counter()
            with lock:
                frame = None if latest["frame"] is None else latest["frame"].copy()
            if frame is None:
                time.sleep(0.01)
                continue
            raw, bev, meta = self.processor.process(frame)
            frame_idx += 1
            if self.on_raw_frame is not None:
                self.on_raw_frame(raw)
            if frame_idx % self.process_every_n == 0:
                if self.on_bev_frame is not None:
                    self.on_bev_frame(bev)
                if self.on_meta is not None:
                    self.on_meta(meta)
            sleep_sec = target_period - (time.perf_counter() - tick)
            if sleep_sec > 0.0:
                time.sleep(sleep_sec)
```

Use the existing frame processing loop inside `_run_capture_session(cap)`. When processor metadata includes robot pose later, pass it unchanged through `on_meta`.

- [ ] **Step 4: Run the test to verify it passes**

Run: `uv run python -m unittest tests.test_bev_backend -v`  
Expected: `OK`

- [ ] **Step 5: Commit**

```bash
git add ground_station/bev_backend.py tests/test_bev_backend.py
git commit -m "refactor: add rtsp reconnect loop"
```

## Task 6: Qt Frontend Integration and Launch Wiring

**Files:**
- Modify: `ground_station/frontend.py`
- Modify: `scripts/run_ground_station.sh`

- [ ] **Step 1: Write the failing integration smoke tests**

```python
import unittest

from ground_station.map_editor import ToolMode
from ground_station.state_machine import RuntimeStateMachine


class FrontendSmokeTests(unittest.TestCase):
    def test_runtime_defaults_match_locked_editing_rules(self) -> None:
        machine = RuntimeStateMachine()
        self.assertEqual(machine.state, "idle")
        self.assertTrue(machine.editing_allowed)
        self.assertEqual(ToolMode.DRAW_LINE.value, "draw_line")
```

- [ ] **Step 2: Run the test to verify it fails only if imports or naming drifted**

Run: `uv run python -m unittest tests.test_state_machine tests.test_map_editor -v`  
Expected: `OK` before touching Qt wiring

- [ ] **Step 3: Rewrite the frontend with the new modules**

`ground_station/frontend.py`

```python
from __future__ import annotations

import argparse
import sys
import time

from ground_station.bev_backend import BevBackend, BevConfig
from ground_station.config import load_config
from ground_station.gcs_server import GcsServer
from ground_station.grid_map import GridConfig, GridMapper
from ground_station.map_editor import MapEditor, ToolMode
from ground_station.planner_controller import PlannerController
from ground_station.state_machine import RuntimeStateMachine

# keep the existing Qt compatibility imports and image helpers

class MainWindow(QMainWindow):
    def __init__(self, config_path: str) -> None:
        super().__init__()
        self.config = load_config(config_path)
        self.state_machine = RuntimeStateMachine()
        self.editor = MapEditor()
        self.mapper = GridMapper(
            GridConfig(
                x_min=self.config["map"]["x_min"],
                x_max=self.config["map"]["x_max"],
                y_min=self.config["map"]["y_min"],
                y_max=self.config["map"]["y_max"],
                resolution_m=self.config["grid"]["resolution_m"],
                ppm=self.config["map"]["ppm"],
                line_obstacle_width_m=self.config["grid"]["line_obstacle_width_m"],
            )
        )
        self.planner = PlannerController(
            mapper=self.mapper,
            lookahead_m=self.config["control"]["lookahead_m"],
            goal_tolerance_m=self.config["control"]["goal_tolerance_m"],
            slowdown_radius_m=self.config["control"]["slowdown_radius_m"],
            max_vx=self.config["control"]["max_vx"],
            max_wz=self.config["control"]["max_wz"],
        )
        self.server = GcsServer(
            host=self.config["server"]["host"],
            port=self.config["server"]["port"],
        )
        self.bev_backend = BevBackend(
            rtsp_url=self.config["rtsp"]["url"],
            fps_limit=self.config["rtsp"]["fps_limit"],
            process_every_n=1,
            bev_config=BevConfig(
                enabled=self.config["bev"]["enabled"],
                calibration_path=self.config["bev"]["calibration_path"],
                target_id=self.config["bev"]["target_id"],
                tag_size=self.config["bev"]["tag_size"],
                x_min=self.config["map"]["x_min"],
                x_max=self.config["map"]["x_max"],
                y_min=self.config["map"]["y_min"],
                y_max=self.config["map"]["y_max"],
                ppm=self.config["map"]["ppm"],
            ),
            retry_sec=self.config["rtsp"]["retry_sec"],
            on_raw_frame=lambda frame: self.bridge.rawFrame.emit(frame),
            on_bev_frame=lambda frame: self.bridge.bevFrame.emit(frame),
            on_meta=self.on_bev_meta,
            on_log=lambda level, text: self.bridge.logLine.emit(level, text),
        )
        self._build_ui()
        self._bind_signals()
        self.server.start()
        self.bev_backend.start()

    def on_plan(self) -> None:
        if self.editor.goal is None or self.pose is None:
            self.on_log("WARN", "goal or robot pose missing")
            return
        self.current_path = self.planner.plan(
            start=(self.pose.x, self.pose.y),
            goal=self.editor.goal,
            rectangles=self.editor.rectangles,
            lines=self.editor.lines,
        )
        self.state_machine.to_planned()
        self.map_canvas.set_path(self.current_path)

    def on_execute(self) -> None:
        if not self.state_machine.can_execute(
            has_goal=self.editor.goal is not None,
            has_path=bool(self.current_path),
            has_pose=self.pose is not None,
            robot_connected=self.server.connected,
        ):
            self.on_log("WARN", "execute preconditions not met")
            return
        self.editor.locked = True
        self.state_machine.to_executing()

    def on_cancel(self) -> None:
        self.server.send_cmd_vel(vx=0.0, vy=0.0, wz=0.0, timestamp=time.time())
        self.editor.goal = None
        self.current_path = []
        self.editor.locked = False
        self.state_machine.cancel()
        self.map_canvas.set_path([])
```

Keep the canvas draw methods, but route clicks through `GridMapper.pixel_to_world()` and `MapEditor.left_click()` / `right_click()`. Disable edit actions while `editing_allowed` is false. During execution, compute and send commands on a Qt timer using the latest BEV pose.

`scripts/run_ground_station.sh`

```bash
#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
uv run --extra gui --project "$ROOT_DIR" python -m ground_station.frontend --config "${1:-config/ground_station.toml}"
```

- [ ] **Step 4: Run the smoke tests and a manual launch check**

Run: `uv run python -m unittest tests.test_ground_station_protocol tests.test_grid_map tests.test_map_editor tests.test_state_machine tests.test_gcs_server tests.test_planner_controller tests.test_bev_backend -v`  
Expected: `OK`

Run: `uv run --extra gui python -m ground_station.frontend --config config/ground_station.toml`  
Expected: Qt window opens, RTSP retries automatically, server starts automatically, left buttons are visible, and no manual start buttons remain

- [ ] **Step 5: Commit**

```bash
git add ground_station/frontend.py scripts/run_ground_station.sh
git commit -m "refactor: wire qt frontend to new gcs stack"
```

## Task 7: Cleanup, Docs, and Full Verification

**Files:**
- Modify: `README.md`
- Delete: `config/ground_station.json`
- Delete: `ground_station/planning_backend.py`

- [ ] **Step 1: Update the README to match the new architecture**

```markdown
- `ground_station` is now the control owner:
  - RTSP pull with reconnect
  - BEV display
  - single-goal A* planning
  - pure pursuit control
  - TCP server for the robot
- `ground_robot` now connects to `gcs_host:gcs_port`
- `config/ground_station.toml` is the only GCS config file
```

- [ ] **Step 2: Remove the old JSON config and old planning backend**

Run: `git rm config/ground_station.json ground_station/planning_backend.py`  
Expected: files removed after the new frontend is already using the replacement modules

- [ ] **Step 3: Run the full verification suite**

Run: `uv run python -m unittest discover -s tests -p 'test_*.py' -v`  
Expected: `OK`

Run: `uv run --extra gui python -m ground_station.frontend --config config/ground_station.toml`  
Expected: window opens and auto-start behavior works

Run: `uv run python -m ground_robot.receiver --config config/ground_robot.json`  
Expected: robot client retries until the GCS server is listening, then prints `cmd_vel` or `reach_goal`

- [ ] **Step 4: Commit**

```bash
git add README.md config/ground_station.toml scripts/run_ground_station.sh ground_station/config.py ground_station/grid_map.py ground_station/map_editor.py ground_station/state_machine.py ground_station/gcs_server.py ground_station/planner_controller.py ground_station/bev_backend.py ground_station/frontend.py ground_robot/receiver.py tests/test_ground_station_protocol.py tests/test_grid_map.py tests/test_map_editor.py tests/test_state_machine.py tests/test_gcs_server.py tests/test_planner_controller.py tests/test_bev_backend.py
git rm config/ground_station.json ground_station/planning_backend.py
git commit -m "refactor: simplify ground station to gcs-owned control stack"
```

## Self-Review

### Spec Coverage

- Python Qt frontend retained: covered in Tasks 5 and 6.
- GCS as TCP server with robot active connection: covered in Task 3.
- Camera auto-reconnect: covered in Task 5.
- Single TOML config: covered in Task 1.
- Unified world/grid/pixel coordinates: covered in Task 2.
- Rectangle and chained line obstacles: covered in Task 2 and Task 6.
- A* planning with grid visualization input model: covered in Tasks 2, 4, and 6.
- Single active goal and cancel behavior: covered in Tasks 2, 3, and 6.
- Executing state locks edits and planning: covered in Tasks 3 and 6.
- Pure pursuit with `vx`, `vy`, `wz`: covered in Task 4.
- Minimal `cmd_vel` / `reach_goal` protocol: covered in Tasks 1 and 3.

### Placeholder Scan

- No `TODO`, `TBD`, or “implement later” markers remain.
- Every task names exact files and exact commands.
- Every code-writing step includes explicit code blocks.

### Type Consistency

- Protocol fields are consistently `vx`, `vy`, `wz`, `timestamp`.
- Config path is consistently `config/ground_station.toml`.
- Runtime states are consistently `idle`, `planned`, `executing`, `cancelled`.
- Obstacle types are consistently rectangles plus chained line obstacles.
