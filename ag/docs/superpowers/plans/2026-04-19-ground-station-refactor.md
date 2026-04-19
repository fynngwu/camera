# Ground Station Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor the AG ground station from 3-tier (GCS→Sky→Robot) to 2-tier (GCS→Robot), with GCS as TCP server, simplified protocol (cmd_vel + reach_goal), auto-reconnect RTSP, A* planning with line-segment obstacles, polynomial controller, and new Layout B UI.

**Architecture:** GCS becomes a TCP server on port 46001. The robot connects to it. All control logic (simulator + polynomial controller) runs on GCS at ~20Hz via QTimer. RTSP auto-connects on startup. Protocol is simplified to two unidirectional GCS→Robot messages: `cmd_vel` and `reach_goal`.

**Tech Stack:** Python 3.12, PySide6, OpenCV (FFmpeg), numpy, uv package manager.

---

## File Structure

| Action | Path | Responsibility |
|--------|------|---------------|
| Create | `ground_station/protocol.py` | JSON Lines encode/decode for cmd_vel and reach_goal |
| Create | `ground_station/rtsp_worker.py` | FFmpeg RTSP pull thread with auto-reconnect |
| Create | `ground_station/bev_processor.py` | AprilTag BEV processing (adapted from existing) |
| Create | `ground_station/robot_server.py` | TCP server: accept robot connection, send/receive |
| Create | `ground_station/simulator.py` | Dead-reckoning integrator with Gaussian noise |
| Create | `ground_station/planner.py` | A* planner + polynomial controller |
| Create | `ground_station/widgets/__init__.py` | Package init |
| Create | `ground_station/widgets/main_window.py` | Main window (Layout B) |
| Create | `ground_station/widgets/image_view.py` | Single frame display widget |
| Create | `ground_station/widgets/map_canvas.py` | BEV canvas with line-segment obstacles |
| Create | `ground_station/widgets/control_panel.py` | Left panel: buttons + speed + status |
| Create | `ground_station/widgets/status_bar.py` | Bottom bar: pose/vel/log |
| Create | `ground_station/__init__.py` | Package init |
| Create | `ground_station/main.py` | Entry point |
| Modify | `ground_robot/receiver.py` | Connect to GCS instead of sky, simplified protocol |
| Modify | `config/ground_station.json` | Updated config schema |
| Create | `config/ground_robot.json` | Robot config (GCS host/port) |
| Modify | `scripts/run_ground_station.sh` | Updated entry point |
| Modify | `scripts/run_ground_robot.sh` | Updated entry point |
| Modify | `tests/test_protocol.py` | Tests for new 2-message protocol |
| Create | `tests/test_simulator.py` | Tests for simulator |
| Create | `tests/test_planner.py` | Tests for planner + controller |

---

### Task 1: Protocol Library

**Files:**
- Create: `ground_station/protocol.py`
- Create: `tests/test_protocol.py`

- [ ] **Step 1: Write the failing test**

```python
"""Tests for the simplified GCS-Robot JSON Lines protocol."""
from __future__ import annotations

import unittest

from ground_station.protocol import (
    decode_message,
    encode_cmd_vel,
    encode_reach_goal,
    validate_cmd_vel,
    validate_reach_goal,
)


class TestEncodeCmdVel(unittest.TestCase):
    def test_produces_valid_jsonl(self):
        raw = encode_cmd_vel(0.2, 0.0, 0.1)
        self.assertIsInstance(raw, bytes)
        self.assertTrue(raw.endswith(b"\n"))
        msg = decode_message(raw)
        self.assertEqual(msg["type"], "cmd_vel")
        self.assertAlmostEqual(msg["vx"], 0.2)
        self.assertAlmostEqual(msg["vy"], 0.0)
        self.assertAlmostEqual(msg["wz"], 0.1)
        self.assertIn("ts", msg)

    def test_negative_values(self):
        raw = encode_cmd_vel(-0.5, 0.3, -1.0)
        msg = decode_message(raw)
        self.assertAlmostEqual(msg["vx"], -0.5)
        self.assertAlmostEqual(msg["vy"], 0.3)
        self.assertAlmostEqual(msg["wz"], -1.0)


class TestEncodeReachGoal(unittest.TestCase):
    def test_produces_valid_jsonl(self):
        raw = encode_reach_goal()
        self.assertIsInstance(raw, bytes)
        self.assertTrue(raw.endswith(b"\n"))
        msg = decode_message(raw)
        self.assertEqual(msg["type"], "reach_goal")
        self.assertIn("ts", msg)


class TestValidateCmdVel(unittest.TestCase):
    def test_valid_message(self):
        msg = decode_message(encode_cmd_vel(0.1, 0.2, 0.3))
        result = validate_cmd_vel(msg)
        self.assertAlmostEqual(result["vx"], 0.1)
        self.assertAlmostEqual(result["vy"], 0.2)
        self.assertAlmostEqual(result["wz"], 0.3)

    def test_missing_field_raises(self):
        with self.assertRaises(ValueError):
            validate_cmd_vel({"type": "cmd_vel", "vx": 0.1, "vy": 0.0})

    def test_wrong_type_raises(self):
        with self.assertRaises(ValueError):
            validate_cmd_vel({"type": "reach_goal", "ts": 1.0})

    def test_non_numeric_raises(self):
        with self.assertRaises(ValueError):
            validate_cmd_vel({"type": "cmd_vel", "vx": "fast", "vy": 0.0, "wz": 0.0})


class TestValidateReachGoal(unittest.TestCase):
    def test_valid_message(self):
        msg = decode_message(encode_reach_goal())
        validate_reach_goal(msg)

    def test_wrong_type_raises(self):
        with self.assertRaises(ValueError):
            validate_reach_goal({"type": "cmd_vel"})


class TestDecodeMessage(unittest.TestCase):
    def test_bytes_input(self):
        raw = encode_cmd_vel(0.0, 0.0, 0.0)
        msg = decode_message(raw)
        self.assertEqual(msg["type"], "cmd_vel")

    def test_string_input(self):
        raw = encode_cmd_vel(0.0, 0.0, 0.0)
        msg = decode_message(raw.decode("utf-8"))
        self.assertEqual(msg["type"], "cmd_vel")

    def test_missing_type_raises(self):
        with self.assertRaises(ValueError):
            decode_message(b'{"vx": 0.1}\n')

    def test_non_object_raises(self):
        with self.assertRaises(ValueError):
            decode_message(b'[1, 2, 3]\n')


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/wufy/projects/DGGRS-air/camera/ag && uv run python -m pytest tests/test_protocol.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'ground_station.protocol'`

- [ ] **Step 3: Write minimal implementation**

Create `ground_station/__init__.py`:
```python
```

Create `ground_station/protocol.py`:
```python
"""Simplified JSON Lines protocol for GCS -> Robot communication.

Two message types, both unidirectional GCS -> Robot:
- cmd_vel: {type, vx, vy, wz, ts}
- reach_goal: {type, ts}
"""
from __future__ import annotations

import json
import time


def encode_cmd_vel(vx: float, vy: float, wz: float) -> bytes:
    msg = {"type": "cmd_vel", "vx": float(vx), "vy": float(vy), "wz": float(wz), "ts": time.time()}
    return _encode(msg)


def encode_reach_goal() -> bytes:
    msg = {"type": "reach_goal", "ts": time.time()}
    return _encode(msg)


def decode_message(line: bytes | str) -> dict:
    if isinstance(line, bytes):
        text = line.decode("utf-8")
    else:
        text = line
    obj = json.loads(text.strip())
    if not isinstance(obj, dict):
        raise ValueError("decoded payload must be a JSON object")
    if "type" not in obj:
        raise ValueError("message missing required field 'type'")
    return obj


def validate_cmd_vel(msg: dict) -> dict:
    if msg.get("type") != "cmd_vel":
        raise ValueError("expected message type 'cmd_vel'")
    for key in ("vx", "vy", "wz"):
        if key not in msg:
            raise ValueError(f"cmd_vel missing field: {key}")
        if not isinstance(msg[key], (int, float)):
            raise ValueError(f"cmd_vel field {key} must be numeric")
    return {"vx": float(msg["vx"]), "vy": float(msg["vy"]), "wz": float(msg["wz"])}


def validate_reach_goal(msg: dict) -> None:
    if msg.get("type") != "reach_goal":
        raise ValueError("expected message type 'reach_goal'")


def _encode(msg: dict) -> bytes:
    return (json.dumps(msg, ensure_ascii=False, separators=(",", ":")) + "\n").encode("utf-8")
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/wufy/projects/DGGRS-air/camera/ag && uv run python -m pytest tests/test_protocol.py -v`
Expected: All tests PASS

- [ ] **Step 5: Commit**

```bash
cd /home/wufy/projects/DGGRS-air/camera/ag
git add ground_station/__init__.py ground_station/protocol.py tests/test_protocol.py
git commit -m "feat(protocol): add simplified cmd_vel + reach_goal JSON Lines protocol"
```

---

### Task 2: Simulator

**Files:**
- Create: `ground_station/simulator.py`
- Create: `tests/test_simulator.py`

- [ ] **Step 1: Write the failing test**

```python
"""Tests for the dead-reckoning simulator."""
from __future__ import annotations

import math
import unittest

from ground_station.simulator import Pose2D, Simulator


class TestSimulator(unittest.TestCase):
    def test_stationary(self):
        sim = Simulator(Pose2D(1.0, 2.0, 0.5))
        pose = sim.step(0.0, 0.0, 0.0, 0.1)
        self.assertAlmostEqual(pose.x, 1.0, places=5)
        self.assertAlmostEqual(pose.y, 2.0, places=5)
        self.assertAlmostEqual(pose.yaw, 0.5, places=5)

    def test_forward_motion(self):
        sim = Simulator(Pose2D(0.0, 0.0, 0.0))
        pose = sim.step(0.5, 0.0, 0.0, 1.0)
        self.assertAlmostEqual(pose.x, 0.5, places=5)
        self.assertAlmostEqual(pose.y, 0.0, places=5)

    def test_rotation(self):
        sim = Simulator(Pose2D(0.0, 0.0, 0.0))
        pose = sim.step(0.0, 0.0, 1.0, 1.0)
        self.assertAlmostEqual(pose.yaw, 1.0, places=5)
        self.assertAlmostEqual(pose.x, 0.0, places=5)
        self.assertAlmostEqual(pose.y, 0.0, places=5)

    def test_combined_motion(self):
        sim = Simulator(Pose2D(0.0, 0.0, 0.0))
        pose = sim.step(1.0, 0.0, 0.0, 1.0)
        self.assertAlmostEqual(pose.x, 1.0, places=5)
        pose = sim.step(0.0, 0.0, math.pi / 2, 1.0)
        self.assertAlmostEqual(pose.yaw, math.pi / 2, places=5)
        pose = sim.step(1.0, 0.0, 0.0, 1.0)
        self.assertAlmostEqual(pose.x, 1.0, places=5)
        self.assertAlmostEqual(pose.y, 1.0, places=5)

    def test_noise_added(self):
        sim = Simulator(Pose2D(0.0, 0.0, 0.0), noise_xy=1.0, noise_yaw=1.0)
        results = [sim.step(0.0, 0.0, 0.0, 1.0) for _ in range(20)]
        xs = [p.x for p in results]
        self.assertTrue(max(xs) > 0.001, "noise should cause position variation")


class TestPose2D(unittest.TestCase):
    def test_defaults(self):
        pose = Pose2D()
        self.assertAlmostEqual(pose.x, 0.0)
        self.assertAlmostEqual(pose.y, 0.0)
        self.assertAlmostEqual(pose.yaw, 0.0)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/wufy/projects/DGGRS-air/camera/ag && uv run python -m pytest tests/test_simulator.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'ground_station.simulator'`

- [ ] **Step 3: Write minimal implementation**

Create `ground_station/simulator.py`:
```python
"""Dead-reckoning simulator: integrate body-frame velocities into world-frame pose."""
from __future__ import annotations

import math
import random
from dataclasses import dataclass


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


class Simulator:
    def __init__(self, initial: Pose2D, noise_xy: float = 0.001, noise_yaw: float = 0.01) -> None:
        self.x = initial.x
        self.y = initial.y
        self.yaw = initial.yaw
        self.noise_xy = noise_xy
        self.noise_yaw = noise_yaw

    @property
    def pose(self) -> Pose2D:
        return Pose2D(self.x, self.y, self.yaw)

    def step(self, vx: float, vy: float, wz: float, dt: float) -> Pose2D:
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        self.x += (vx * cos_y - vy * sin_y) * dt + random.gauss(0, self.noise_xy)
        self.y += (vx * sin_y + vy * cos_y) * dt + random.gauss(0, self.noise_xy)
        self.yaw += wz * dt + random.gauss(0, self.noise_yaw)
        return Pose2D(self.x, self.y, self.yaw)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/wufy/projects/DGGRS-air/camera/ag && uv run python -m pytest tests/test_simulator.py -v`
Expected: All tests PASS

- [ ] **Step 5: Commit**

```bash
cd /home/wufy/projects/DGGRS-air/camera/ag
git add ground_station/simulator.py tests/test_simulator.py
git commit -m "feat(simulator): add dead-reckoning integrator with Gaussian noise"
```

---

### Task 3: A* Planner with Line-Segment Obstacles

**Files:**
- Create: `ground_station/planner.py`
- Create: `tests/test_planner.py`

- [ ] **Step 1: Write the failing test**

```python
"""Tests for A* planner and polynomial controller."""
from __future__ import annotations

import math
import unittest

from ground_station.planner import (
    AStarPlanner,
    LineObstacle,
    PlannerConfig,
    compute_command,
)
from ground_station.simulator import Pose2D


class TestAStarPlanner(unittest.TestCase):
    def setUp(self):
        self.config = PlannerConfig(
            x_min=-1.0, x_max=1.0, y_min=-1.0, y_max=1.0,
            resolution_m=0.05, robot_radius_m=0.1, goal_tolerance_m=0.05,
        )
        self.planner = AStarPlanner(self.config)

    def test_straight_line_no_obstacles(self):
        path = self.planner.plan((-0.5, -0.5), [(0.5, 0.5)], [])
        self.assertGreaterEqual(len(path), 2)
        self.assertAlmostEqual(path[0][0], -0.5, places=2)
        self.assertAlmostEqual(path[-1][0], 0.5, places=2)

    def test_single_goal_no_obstacles(self):
        path = self.planner.plan((0.0, 0.0), [(0.5, 0.0)], [])
        self.assertGreaterEqual(len(path), 2)

    def test_line_obstacle_forces_detour(self):
        wall = [LineObstacle(-0.05, -0.3, -0.05, 0.3)]
        path = self.planner.plan((-0.5, 0.0), [(0.5, 0.0)], wall)
        self.assertGreaterEqual(len(path), 2)
        for px, py in path:
            self.assertFalse(abs(px - (-0.05)) < 0.15 and abs(py) < 0.25)

    def test_start_inside_obstacle_raises(self):
        wall = [LineObstacle(-0.5, -0.5, 0.5, -0.5)]
        with self.assertRaises(ValueError):
            self.planner.plan((0.0, -0.5), [(0.5, 0.5)], wall)

    def test_no_goals_returns_start(self):
        path = self.planner.plan((0.3, 0.3), [], [])
        self.assertEqual(len(path), 1)

    def test_shortcut_reduces_points(self):
        path = self.planner.plan((-0.9, 0.0), [(0.9, 0.0)], [])
        self.assertLessEqual(len(path), 10)


class TestComputeCommand(unittest.TestCase):
    def test_goal_reached(self):
        pose = Pose2D(0.5, 0.5, 0.0)
        vx, vy, wz, reached = compute_command(pose, (0.5, 0.5), 0.2)
        self.assertTrue(reached)
        self.assertAlmostEqual(vx, 0.0)
        self.assertAlmostEqual(wz, 0.0)

    def test_heading_toward_goal(self):
        pose = Pose2D(0.0, 0.0, 0.0)
        vx, vy, wz, reached = compute_command(pose, (1.0, 0.0), 0.3)
        self.assertFalse(reached)
        self.assertGreater(vx, 0.0)
        self.assertAlmostEqual(wz, 0.0, places=2)

    def test_rotates_to_face_goal(self):
        pose = Pose2D(0.0, 0.0, math.pi)
        vx, vy, wz, reached = compute_command(pose, (1.0, 0.0), 0.3)
        self.assertFalse(reached)
        self.assertLess(wz, 0.0)

    def test_slows_near_goal(self):
        pose_far = Pose2D(0.0, 0.0, 0.0)
        vx_far, *_ = compute_command(pose_far, (1.0, 0.0), 0.3)
        pose_near = Pose2D(0.95, 0.0, 0.0)
        vx_near, *_ = compute_command(pose_near, (1.0, 0.0), 0.3)
        self.assertLess(vx_near, vx_far)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/wufy/projects/DGGRS-air/camera/ag && uv run python -m pytest tests/test_planner.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'ground_station.planner'`

- [ ] **Step 3: Write minimal implementation**

Create `ground_station/planner.py`:
```python
"""A* path planner with line-segment obstacles and polynomial body-frame controller."""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from ground_station.simulator import Pose2D


WorldPoint = Tuple[float, float]

TIE_BREAK_EPSILON = 1e-4


@dataclass
class LineObstacle:
    x0: float
    y0: float
    x1: float
    y1: float


@dataclass
class PlannerConfig:
    x_min: float = -1.0
    x_max: float = 1.0
    y_min: float = -1.0
    y_max: float = 1.0
    resolution_m: float = 0.05
    robot_radius_m: float = 0.10
    goal_tolerance_m: float = 0.05


class AStarPlanner:
    def __init__(self, config: PlannerConfig) -> None:
        self.config = config
        self.width = max(2, int(math.ceil((config.x_max - config.x_min) / config.resolution_m)))
        self.height = max(2, int(math.ceil((config.y_max - config.y_min) / config.resolution_m)))

    def plan(self, start: WorldPoint, goals: Sequence[WorldPoint], obstacles: Sequence[LineObstacle]) -> List[WorldPoint]:
        if not goals:
            return [start]
        occupancy = self._build_occupancy(obstacles)
        total_path: List[WorldPoint] = [start]
        current = start
        for goal in goals:
            segment = self._plan_segment(current, goal, occupancy)
            if total_path[-1] == segment[0]:
                total_path.extend(segment[1:])
            else:
                total_path.extend(segment)
            current = goal
        return total_path

    def _plan_segment(self, start: WorldPoint, goal: WorldPoint, occupancy: List[List[bool]]) -> List[WorldPoint]:
        s = self._world_to_grid(*start)
        g = self._world_to_grid(*goal)
        search_start = self._nearest_free_cell(s, occupancy)
        search_goal = self._nearest_free_cell(g, occupancy)
        if search_start is None:
            raise ValueError("start cell is blocked")
        if search_goal is None:
            raise ValueError("goal cell is blocked")
        if self._line_of_sight(search_start, search_goal, occupancy):
            return [start, goal]

        start_h = self._heuristic(search_start, search_goal)
        # Tie-break: f(n) = g(n) + h(n) * (1 + epsilon)
        frontier: List[Tuple[float, float, int, Tuple[int, int]]] = [(start_h * (1 + TIE_BREAK_EPSILON), 0.0, 0, search_start)]
        came_from: dict = {search_start: None}
        cost_so_far: dict = {search_start: 0.0}
        counter = 0

        while frontier:
            _, _, _, current = heapq.heappop(frontier)
            if current == search_goal:
                break
            for nxt, step_cost in self._neighbors(current, occupancy):
                new_cost = cost_so_far[current] + step_cost
                if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                    cost_so_far[nxt] = new_cost
                    h = self._heuristic(nxt, search_goal)
                    priority = new_cost + h * (1 + TIE_BREAK_EPSILON)
                    counter += 1
                    heapq.heappush(frontier, (priority, new_cost, counter, nxt))
                    came_from[nxt] = current

        if search_goal not in came_from:
            raise ValueError("planner failed to find path")

        path_grid: List[Tuple[int, int]] = []
        cur: Optional[Tuple[int, int]] = search_goal
        while cur is not None:
            path_grid.append(cur)
            cur = came_from[cur]
        path_grid.reverse()
        path_world: List[WorldPoint] = [start]
        for ix, iy in path_grid[1:-1]:
            path_world.append(self._grid_to_world(ix, iy))
        path_world.append(goal)
        return self._shortcut(path_world, occupancy)

    def _build_occupancy(self, obstacles: Sequence[LineObstacle]) -> List[List[bool]]:
        grid = [[False for _ in range(self.width)] for _ in range(self.height)]
        inflate = max(0, int(math.ceil(self.config.robot_radius_m / self.config.resolution_m)))
        for obs in obstacles:
            self._rasterize_line(obs, grid, inflate)
        return grid

    def _rasterize_line(self, obs: LineObstacle, grid: List[List[bool]], inflate: int) -> None:
        x0, y0 = self._world_to_grid(obs.x0, obs.y0)
        x1, y1 = self._world_to_grid(obs.x1, obs.y1)
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        steps = max(dx, dy)
        if steps == 0:
            for iy in range(max(0, y0 - inflate), min(self.height, y0 + inflate + 1)):
                for ix in range(max(0, x0 - inflate), min(self.width, x0 + inflate + 1)):
                    grid[iy][ix] = True
            return
        for i in range(steps + 1):
            t = i / steps
            ix = int(round(x0 + (x1 - x0) * t))
            iy = int(round(y0 + (y1 - y0) * t))
            for dy2 in range(-inflate, inflate + 1):
                for dx2 in range(-inflate, inflate + 1):
                    nx, ny = ix + dx2, iy + dy2
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        grid[ny][nx] = True

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
        return max(0, min(self.width - 1, ix)), max(0, min(self.height - 1, iy))

    def _grid_to_world(self, ix: int, iy: int) -> WorldPoint:
        x = self.config.x_min + (ix + 0.5) * self.config.resolution_m
        y = self.config.y_min + (iy + 0.5) * self.config.resolution_m
        return x, y

    @staticmethod
    def _heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        diagonal = min(dx, dy)
        straight = max(dx, dy) - diagonal
        return straight + math.sqrt(2.0) * diagonal

    def _line_of_sight(self, start: Tuple[int, int], goal: Tuple[int, int], occupancy: List[List[bool]]) -> bool:
        x0, y0 = start
        x1, y1 = goal
        steps = max(abs(x1 - x0), abs(y1 - y0))
        if steps == 0:
            return not occupancy[y0][x0]
        for i in range(steps + 1):
            t = i / steps
            ix = int(round(x0 + (x1 - x0) * t))
            iy = int(round(y0 + (y1 - y0) * t))
            ix = max(0, min(self.width - 1, ix))
            iy = max(0, min(self.height - 1, iy))
            if occupancy[iy][ix]:
                return False
        return True

    def _nearest_free_cell(self, node: Tuple[int, int], occupancy: List[List[bool]]) -> Optional[Tuple[int, int]]:
        if not occupancy[node[1]][node[0]]:
            return node
        for radius in range(1, max(self.width, self.height)):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if max(abs(dx), abs(dy)) != radius:
                        continue
                    nx, ny = node[0] + dx, node[1] + dy
                    if 0 <= nx < self.width and 0 <= ny < self.height and not occupancy[ny][nx]:
                        return (nx, ny)
        return None

    def _shortcut(self, points: Sequence[WorldPoint], occupancy: List[List[bool]]) -> List[WorldPoint]:
        path = list(points)
        if len(path) <= 2:
            return path
        compact = [path[0]]
        start_idx = 0
        while start_idx < len(path) - 1:
            end_idx = len(path) - 1
            while end_idx > start_idx + 1:
                if self._line_of_sight(
                    self._world_to_grid(*path[start_idx]),
                    self._world_to_grid(*path[end_idx]),
                    occupancy,
                ):
                    break
                end_idx -= 1
            compact.append(path[end_idx])
            start_idx = end_idx
        return compact


def compute_command(
    pose: Pose2D, goal: WorldPoint, target_speed: float,
    k_yaw: float = 3.0, k_speed: float = 1.5, goal_tolerance: float = 0.05,
) -> Tuple[float, float, float, bool]:
    dx = goal[0] - pose.x
    dy = goal[1] - pose.y
    dist = math.hypot(dx, dy)
    if dist < goal_tolerance:
        return 0.0, 0.0, 0.0, True

    target_heading = math.atan2(dy, dx)
    yaw_error = target_heading - pose.yaw
    while yaw_error > math.pi:
        yaw_error -= 2 * math.pi
    while yaw_error < -math.pi:
        yaw_error += 2 * math.pi

    wz = k_yaw * yaw_error
    vx = k_speed * dist * math.cos(yaw_error) * target_speed
    vx = max(0.0, min(vx, target_speed))
    return vx, 0.0, wz, False
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/wufy/projects/DGGRS-air/camera/ag && uv run python -m pytest tests/test_planner.py -v`
Expected: All tests PASS

- [ ] **Step 5: Commit**

```bash
cd /home/wufy/projects/DGGRS-air/camera/ag
git add ground_station/planner.py tests/test_planner.py
git commit -m "feat(planner): add A* planner with line-segment obstacles and polynomial controller"
```

---

### Task 4: RTSP Worker

**Files:**
- Create: `ground_station/rtsp_worker.py`

- [ ] **Step 1: Write the implementation**

Adapted from existing `bev_backend.py`. RTSP auto-connects on startup with auto-reconnect. Callbacks for raw frame, BEV frame, and log messages.

Create `ground_station/rtsp_worker.py`:
```python
"""FFmpeg RTSP pull thread with auto-reconnect."""
from __future__ import annotations

import os
import threading
import time
from typing import Callable, Optional

import cv2
import numpy as np

FrameCallback = Callable[[np.ndarray], None]
LogCallback = Callable[[str], None]


class RtspWorker:
    def __init__(
        self,
        rtsp_url: str,
        fps_limit: float = 20.0,
        reconnect_interval: float = 2.0,
        on_raw_frame: Optional[FrameCallback] = None,
        on_log: Optional[LogCallback] = None,
    ) -> None:
        self.rtsp_url = rtsp_url
        self.fps_limit = fps_limit
        self.reconnect_interval = reconnect_interval
        self.on_raw_frame = on_raw_frame
        self.on_log = on_log

        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _run(self) -> None:
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
            "rtsp_transport;udp|max_delay;0|fflags;nobuffer|flags;low_delay"
        )
        while not self._stop_event.is_set():
            self._log(f"RTSP: Connecting to {self.rtsp_url}...")
            cap = cv2.VideoCapture(
                self.rtsp_url, cv2.CAP_FFMPEG,
                [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 3000,
                 cv2.CAP_PROP_READ_TIMEOUT_MSEC, 1000,
                 cv2.CAP_PROP_N_THREADS, 1],
            )
            if not cap.isOpened():
                self._log(f"RTSP: Failed to connect, retrying in {self.reconnect_interval}s...")
                cap.release()
                if self._stop_event.wait(self.reconnect_interval):
                    return
                continue

            self._log("RTSP: Connected")
            self._reader_loop(cap)
            cap.release()
            if self._stop_event.is_set():
                return
            self._log(f"RTSP: Disconnected, reconnecting in {self.reconnect_interval}s...")
            self._stop_event.wait(self.reconnect_interval)

    def _reader_loop(self, cap: cv2.VideoCapture) -> None:
        latest: dict = {"frame": None}
        lock = threading.Lock()

        def _grab() -> None:
            while not self._stop_event.is_set():
                if not cap.grab():
                    continue
                ok, frame = cap.retrieve()
                if not ok:
                    continue
                with lock:
                    latest["frame"] = frame

        reader = threading.Thread(target=_grab, daemon=True)
        reader.start()

        target_period = 1.0 / max(1.0, self.fps_limit)
        while not self._stop_event.is_set():
            tick = time.perf_counter()
            with lock:
                frame = None if latest["frame"] is None else latest["frame"].copy()
            if frame is not None and self.on_raw_frame is not None:
                self.on_raw_frame(frame)
            dt = time.perf_counter() - tick
            sleep_sec = target_period - dt
            if sleep_sec > 0.0:
                if self._stop_event.wait(sleep_sec):
                    return

    def _log(self, text: str) -> None:
        if self.on_log is not None:
            self.on_log(text)
```

- [ ] **Step 2: Verify the module imports**

Run: `cd /home/wufy/projects/DGGRS-air/camera/ag && uv run python -c "from ground_station.rtsp_worker import RtspWorker; print('OK')"`
Expected: `OK`

- [ ] **Step 3: Commit**

```bash
cd /home/wufy/projects/DGGRS-air/camera/ag
git add ground_station/rtsp_worker.py
git commit -m "feat(rtsp): add FFmpeg RTSP worker with auto-reconnect"
```

---

### Task 5: BEV Processor

**Files:**
- Create: `ground_station/bev_processor.py`

- [ ] **Step 1: Write the implementation**

Adapted from existing `AprilTagBevProcessor`. Takes a raw frame and returns BEV image.

Create `ground_station/bev_processor.py`:
```python
"""AprilTag BEV processing: perspective warp from tag detection."""
from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import cv2
import numpy as np

try:
    from pupil_apriltags import Detector
except Exception:
    Detector = None


@dataclass
class BevConfig:
    enabled: bool = True
    calibration_path: str = "config/calibration.yaml"
    target_id: int = 0
    tag_size: float = 0.09
    x_min: float = -1.0
    x_max: float = 1.0
    y_min: float = -1.0
    y_max: float = 1.0
    ppm: int = 300


class BevProcessor:
    def __init__(self, config: BevConfig) -> None:
        self.config = config
        self.map1: Optional[np.ndarray] = None
        self.map2: Optional[np.ndarray] = None
        self.last_bev: Optional[np.ndarray] = None
        self.detector = None
        if config.enabled and Detector is not None:
            self.detector = Detector(
                families="tag36h11",
                nthreads=max(1, min(4, os.cpu_count() or 2)),
                quad_decimate=1.5, quad_sigma=0.0,
                refine_edges=1, decode_sharpening=0.5, debug=0,
            )

    @property
    def bev_size(self) -> Tuple[int, int]:
        width = int((self.config.x_max - self.config.x_min) * self.config.ppm)
        height = int((self.config.y_max - self.config.y_min) * self.config.ppm)
        return max(32, width), max(32, height)

    def process(self, frame: np.ndarray) -> Tuple[np.ndarray, Dict]:
        width, height = self.bev_size
        raw = frame.copy()

        if not self.config.enabled or self.detector is None:
            bev = cv2.resize(raw, (width, height))
            return bev, {"mode": "passthrough", "tag_detected": False}

        undistorted = self._undistort(raw)
        bev = np.zeros((height, width, 3), dtype=np.uint8)
        bev[:] = (24, 24, 24)
        self._draw_grid(bev)

        gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray)

        tag = None
        for det in detections:
            if int(det.tag_id) == int(self.config.target_id):
                tag = det
                break

        if tag is None:
            if self.last_bev is not None:
                bev = self.last_bev.copy()
            return bev, {"mode": "no_tag", "tag_detected": False}

        corners = tag.corners.astype(np.float32)
        cv2.polylines(undistorted, [corners.astype(np.int32)], True, (0, 255, 0), 2)

        half = self.config.tag_size * 0.5
        dst = np.array([
            self._world_to_pixel(+half, +half),
            self._world_to_pixel(-half, +half),
            self._world_to_pixel(-half, -half),
            self._world_to_pixel(+half, -half),
        ], dtype=np.float32)
        matrix = cv2.getPerspectiveTransform(corners, dst)
        warped = cv2.warpPerspective(undistorted, matrix, (width, height))
        bev = cv2.addWeighted(bev, 0.25, warped, 0.75, 0.0)
        self._draw_grid(bev)
        self.last_bev = bev
        return bev, {"mode": "apriltag", "tag_detected": True, "tag_id": int(tag.tag_id)}

    def _world_to_pixel(self, x: float, y: float) -> Tuple[float, float]:
        width, height = self.bev_size
        u = (float(x) - self.config.x_min) / max(self.config.x_max - self.config.x_min, 1e-6) * width
        v = (self.config.y_max - float(y)) / max(self.config.y_max - self.config.y_min, 1e-6) * height
        return float(u), float(v)

    def _undistort(self, frame: np.ndarray) -> np.ndarray:
        path = self.config.calibration_path
        if not path or not os.path.exists(path):
            return frame
        if self.map1 is None or self.map2 is None:
            import yaml
            try:
                with open(path, "r", encoding="utf-8") as fp:
                    data = yaml.safe_load(fp)
                camera_matrix = np.asarray(data["camera_matrix"], dtype=np.float32).reshape(3, 3)
                dist_coeffs = np.asarray(data["dist_coeffs"], dtype=np.float32).reshape(-1)
                h, w = frame.shape[:2]
                new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 0, (w, h))
                self.map1, self.map2 = cv2.initUndistortRectifyMap(
                    camera_matrix, dist_coeffs, None, new_camera_matrix, (w, h), cv2.CV_32FC1,
                )
            except Exception:
                return frame
        return cv2.remap(frame, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)

    def _draw_grid(self, image: np.ndarray) -> None:
        width, height = self.bev_size
        step = max(20, self.config.ppm // 5)
        for x in range(0, width, step):
            cv2.line(image, (x, 0), (x, height), (50, 50, 50), 1)
        for y in range(0, height, step):
            cv2.line(image, (0, y), (width, y), (50, 50, 50), 1)
```

- [ ] **Step 2: Verify the module imports**

Run: `cd /home/wufy/projects/DGGRS-air/camera/ag && uv run python -c "from ground_station.bev_processor import BevProcessor; print('OK')"`
Expected: `OK`

- [ ] **Step 3: Commit**

```bash
cd /home/wufy/projects/DGGRS-air/camera/ag
git add ground_station/bev_processor.py
git commit -m "feat(bev): add AprilTag BEV processor with perspective warp"
```

---

### Task 6: Robot TCP Server

**Files:**
- Create: `ground_station/robot_server.py`

- [ ] **Step 1: Write the implementation**

GCS TCP server that listens for robot connections. Thread-safe send with lock.

Create `ground_station/robot_server.py`:
```python
"""TCP server for GCS: accept robot connection and send cmd_vel/reach_goal."""
from __future__ import annotations

import logging
import socket
import threading
from typing import Callable, Optional

LOGGER = logging.getLogger(__name__)

ConnectionCallback = Callable[[], None]
MessageCallback = Callable[[dict], None]


class RobotServer:
    def __init__(
        self,
        bind_host: str = "0.0.0.0",
        bind_port: int = 46001,
        on_connected: Optional[ConnectionCallback] = None,
        on_disconnected: Optional[ConnectionCallback] = None,
    ) -> None:
        self.bind_host = bind_host
        self.bind_port = bind_port
        self.on_connected = on_connected
        self.on_disconnected = on_disconnected

        self._server_socket: Optional[socket.socket] = None
        self._client_socket: Optional[socket.socket] = None
        self._send_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._accept_thread: Optional[threading.Thread] = None
        self.is_connected = False

    def start(self) -> None:
        if self._accept_thread is not None and self._accept_thread.is_alive():
            return
        self._stop_event.clear()
        self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_socket.bind((self.bind_host, self.bind_port))
        self._server_socket.listen(1)
        self._server_socket.settimeout(1.0)
        self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._accept_thread.start()
        LOGGER.info("RobotServer listening on %s:%s", self.bind_host, self.bind_port)

    def stop(self) -> None:
        self._stop_event.set()
        self._close_client()
        if self._server_socket is not None:
            try:
                self._server_socket.close()
            except OSError:
                pass
            self._server_socket = None
        if self._accept_thread is not None:
            self._accept_thread.join(timeout=2.0)

    def send(self, data: bytes) -> None:
        with self._send_lock:
            if self._client_socket is not None:
                try:
                    self._client_socket.sendall(data)
                except OSError:
                    self._close_client()

    def _accept_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                client, addr = self._server_socket.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            LOGGER.info("Robot connected from %s", addr)
            self._client_socket = client
            self._client_socket.settimeout(1.0)
            self.is_connected = True
            if self.on_connected is not None:
                self.on_connected()
            self._read_loop()
            self.is_connected = False
            if self.on_disconnected is not None:
                self.on_disconnected()

    def _read_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                chunk = self._client_socket.recv(65536)
                if not chunk:
                    break
            except socket.timeout:
                continue
            except OSError:
                break
        self._close_client()

    def _close_client(self) -> None:
        if self._client_socket is not None:
            try:
                self._client_socket.close()
            except OSError:
                pass
            self._client_socket = None
```

- [ ] **Step 2: Verify the module imports**

Run: `cd /home/wufy/projects/DGGRS-air/camera/ag && uv run python -c "from ground_station.robot_server import RobotServer; print('OK')"`
Expected: `OK`

- [ ] **Step 3: Commit**

```bash
cd /home/wufy/projects/DGGRS-air/camera/ag
git add ground_station/robot_server.py
git commit -m "feat(server): add TCP server for robot connections"
```

---

### Task 7: Qt Widgets

**Files:**
- Create: `ground_station/widgets/__init__.py`
- Create: `ground_station/widgets/image_view.py`
- Create: `ground_station/widgets/map_canvas.py`
- Create: `ground_station/widgets/control_panel.py`
- Create: `ground_station/widgets/status_bar.py`

- [ ] **Step 1: Write widget implementations**

Create `ground_station/widgets/__init__.py`:
```python
```

Create `ground_station/widgets/image_view.py`:
```python
"""Single frame display widget."""
from __future__ import annotations

from typing import Optional

import numpy as np

try:
    from PySide6.QtCore import Qt
    from PySide6.QtGui import QColor, QImage, QPainter
    from PySide6.QtWidgets import QWidget
except ImportError:
    from PyQt5.QtCore import Qt
    from PyQt5.QtGui import QColor, QImage, QPainter
    from PyQt5.QtWidgets import QWidget


def ndarray_to_qimage(frame: Optional[np.ndarray]) -> Optional[QImage]:
    if frame is None:
        return None
    arr = np.ascontiguousarray(frame)
    if arr.ndim == 2:
        h, w = arr.shape
        return QImage(arr.data, w, h, w, QImage.Format_Grayscale8).copy()
    if arr.ndim != 3:
        return None
    h, w, c = arr.shape
    if c == 3:
        rgb = arr[:, :, ::-1].copy()
        return QImage(rgb.data, w, h, w * 3, QImage.Format_RGB888).copy()
    if c == 4:
        return QImage(arr.data, w, h, w * 4, QImage.Format_RGBA8888).copy()
    return None


class ImageView(QWidget):
    def __init__(self, title: str) -> None:
        super().__init__()
        self.title = title
        self._frame: Optional[QImage] = None
        self.setMinimumSize(420, 300)

    def set_frame(self, frame: np.ndarray) -> None:
        self._frame = ndarray_to_qimage(frame)
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(24, 24, 24))
        draw_rect = self.rect().adjusted(6, 6, -6, -6)
        if self._frame is not None:
            scaled = self._frame.scaled(draw_rect.size(), Qt.IgnoreAspectRatio, Qt.SmoothTransformation)
            painter.drawImage(draw_rect, scaled)
        painter.setPen(QColor(255, 255, 255))
        painter.drawText(14, 22, self.title)
```

Create `ground_station/widgets/map_canvas.py`:
```python
"""Interactive BEV canvas with line-segment obstacles, path, and robot pose."""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from ground_station.planner import LineObstacle

try:
    from PySide6.QtCore import QPointF, QRectF, Qt, Signal
    from PySide6.QtGui import QColor, QBrush, QImage, QPainter, QPen
    from PySide6.QtWidgets import QWidget
except ImportError:
    from PyQt5.QtCore import QPointF, QRectF, Qt, pyqtSignal as Signal
    from PyQt5.QtGui import QColor, QBrush, QImage, QPainter, QPen
    from PyQt5.QtWidgets import QWidget

WorldPoint = Tuple[float, float]


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass
class VelocityCommand:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


@dataclass
class MapOverlay:
    waypoints: List[WorldPoint] = field(default_factory=list)
    obstacles: List[LineObstacle] = field(default_factory=list)
    path: List[WorldPoint] = field(default_factory=list)


class MapCanvas(QWidget):
    overlayChanged = Signal(object)

    def __init__(self, x_min: float, x_max: float, y_min: float, y_max: float) -> None:
        super().__init__()
        self.x_min, self.x_max = float(x_min), float(x_max)
        self.y_min, self.y_max = float(y_min), float(y_max)
        self.overlay = MapOverlay()
        self.pose = Pose2D()
        self.cmd = VelocityCommand()
        self.mode = "waypoint"
        self._background: Optional[QImage] = None
        self._obstacle_start: Optional[WorldPoint] = None
        self._last_obstacle_end: Optional[WorldPoint] = None
        self._drawing_obstacle = False
        self.setMinimumSize(420, 320)
        self.setMouseTracking(True)

    def set_mode(self, mode: str) -> None:
        if self.mode == "obstacle" and mode != "obstacle":
            self._finish_obstacle_chain()
        self.mode = mode

    def set_background(self, frame) -> None:
        from ground_station.widgets.image_view import ndarray_to_qimage
        self._background = ndarray_to_qimage(frame)
        self.update()

    def set_pose_and_cmd(self, pose: Pose2D, cmd: VelocityCommand) -> None:
        self.pose = pose
        self.cmd = cmd
        self.update()

    def set_path(self, path: List[WorldPoint]) -> None:
        self.overlay.path = list(path)
        self.update()

    def clear_annotations(self) -> None:
        self.overlay.waypoints.clear()
        self.overlay.obstacles.clear()
        self.overlay.path.clear()
        self._obstacle_start = None
        self._last_obstacle_end = None
        self._drawing_obstacle = False
        self.update()

    def _finish_obstacle_chain(self) -> None:
        self._obstacle_start = None
        self._last_obstacle_end = None
        self._drawing_obstacle = False

    def paintEvent(self, event) -> None:  # noqa: N802
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(24, 24, 24))
        draw_rect = self.rect().adjusted(8, 8, -8, -8)
        if self._background is not None:
            scaled = self._background.scaled(draw_rect.size(), Qt.IgnoreAspectRatio, Qt.SmoothTransformation)
            painter.drawImage(draw_rect, scaled)
        self._draw_grid(painter, draw_rect)
        self._draw_obstacles(painter, draw_rect)
        self._draw_path(painter, draw_rect)
        self._draw_waypoints(painter, draw_rect)
        self._draw_pose(painter, draw_rect)
        self._draw_cmd_arrow(painter, draw_rect)
        painter.setPen(QColor(255, 255, 255))
        painter.drawText(16, 24, f"mode={self.mode}")

    def mousePressEvent(self, event) -> None:  # noqa: N802
        px, py = self._event_xy(event)
        world = self._widget_to_world(px, py)
        if world is None:
            return

        if event.button() == Qt.LeftButton:
            if self.mode == "waypoint":
                self.overlay.waypoints.append(world)
                self.overlayChanged.emit(self.overlay)
                self.update()
            elif self.mode == "obstacle":
                if not self._drawing_obstacle:
                    self._obstacle_start = world
                    self._drawing_obstacle = True
                else:
                    end = world
                    start = self._last_obstacle_end if self._last_obstacle_end is not None else self._obstacle_start
                    self.overlay.obstacles.append(LineObstacle(start[0], start[1], end[0], end[1]))
                    self._last_obstacle_end = end
                    self.overlayChanged.emit(self.overlay)
                self.update()
            elif self.mode == "erase":
                self._erase_nearest(world)
                self.overlayChanged.emit(self.overlay)
                self.update()
        elif event.button() == Qt.RightButton and self.mode == "obstacle":
            self._finish_obstacle_chain()
            self.update()

    def _draw_grid(self, painter: QPainter, draw_rect) -> None:
        painter.setPen(QPen(QColor(80, 80, 80), 1))
        for i in range(11):
            x = draw_rect.left() + i * draw_rect.width() / 10
            painter.drawLine(int(x), draw_rect.top(), int(x), draw_rect.bottom())
            y = draw_rect.top() + i * draw_rect.height() / 10
            painter.drawLine(draw_rect.left(), int(y), draw_rect.right(), int(y))

    def _draw_obstacles(self, painter: QPainter, draw_rect) -> None:
        painter.setPen(QPen(QColor(255, 120, 120), 3))
        for obs in self.overlay.obstacles:
            p0 = self._world_to_widget(obs.x0, obs.y0, draw_rect)
            p1 = self._world_to_widget(obs.x1, obs.y1, draw_rect)
            if p0 is not None and p1 is not None:
                painter.drawLine(p0, p1)

    def _draw_waypoints(self, painter: QPainter, draw_rect) -> None:
        painter.setPen(QPen(QColor(80, 220, 255), 2))
        painter.setBrush(QBrush(QColor(80, 220, 255)))
        for idx, point in enumerate(self.overlay.waypoints):
            p = self._world_to_widget(point[0], point[1], draw_rect)
            if p is not None:
                painter.drawEllipse(p, 5, 5)
                painter.drawText(p + QPointF(8, -6), str(idx))

    def _draw_path(self, painter: QPainter, draw_rect) -> None:
        if len(self.overlay.path) < 2:
            return
        painter.setPen(QPen(QColor(255, 220, 0), 3))
        prev = None
        for point in self.overlay.path:
            p = self._world_to_widget(point[0], point[1], draw_rect)
            if p is None:
                continue
            if prev is not None:
                painter.drawLine(prev, p)
            prev = p

    def _draw_pose(self, painter: QPainter, draw_rect) -> None:
        p = self._world_to_widget(self.pose.x, self.pose.y, draw_rect)
        if p is None:
            return
        painter.setPen(QPen(QColor(120, 255, 120), 3))
        painter.setBrush(QBrush(QColor(120, 255, 120)))
        painter.drawEllipse(p, 6, 6)
        tip = QPointF(p.x() + 26 * math.cos(self.pose.yaw), p.y() - 26 * math.sin(self.pose.yaw))
        painter.drawLine(p, tip)

    def _draw_cmd_arrow(self, painter: QPainter, draw_rect) -> None:
        p = self._world_to_widget(self.pose.x, self.pose.y, draw_rect)
        if p is None:
            return
        painter.setPen(QPen(QColor(255, 165, 0), 3))
        tip = QPointF(p.x() + 70 * self.cmd.vx, p.y() - 70 * self.cmd.vy)
        painter.drawLine(p, tip)

    def _world_to_widget(self, x: float, y: float, draw_rect) -> Optional[QPointF]:
        if self.x_max <= self.x_min or self.y_max <= self.y_min:
            return None
        u = (x - self.x_min) / (self.x_max - self.x_min)
        v = (self.y_max - y) / (self.y_max - self.y_min)
        return QPointF(draw_rect.left() + u * draw_rect.width(), draw_rect.top() + v * draw_rect.height())

    def _widget_to_world(self, px: float, py: float) -> Optional[WorldPoint]:
        draw_rect = self.rect().adjusted(8, 8, -8, -8)
        if not draw_rect.contains(int(px), int(py)):
            return None
        u = (px - draw_rect.left()) / max(draw_rect.width(), 1)
        v = (py - draw_rect.top()) / max(draw_rect.height(), 1)
        x = self.x_min + u * (self.x_max - self.x_min)
        y = self.y_max - v * (self.y_max - self.y_min)
        return float(x), float(y)

    def _erase_nearest(self, world: WorldPoint) -> None:
        wx, wy = world
        for idx, point in enumerate(self.overlay.waypoints):
            if math.hypot(point[0] - wx, point[1] - wy) < 0.08:
                del self.overlay.waypoints[idx]
                return
        for idx, obs in enumerate(self.overlay.obstacles):
            dist = self._point_to_segment_dist(wx, wy, obs.x0, obs.y0, obs.x1, obs.y1)
            if dist < 0.08:
                del self.overlay.obstacles[idx]
                return

    @staticmethod
    def _point_to_segment_dist(px: float, py: float, x0: float, y0: float, x1: float, y1: float) -> float:
        dx, dy = x1 - x0, y1 - y0
        if dx == 0.0 and dy == 0.0:
            return math.hypot(px - x0, py - y0)
        t = max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / (dx * dx + dy * dy)))
        return math.hypot(px - (x0 + t * dx), py - (y0 + t * dy))

    @staticmethod
    def _event_xy(event) -> Tuple[float, float]:
        if hasattr(event, "position"):
            pos = event.position()
            return float(pos.x()), float(pos.y())
        pos = event.pos()
        return float(pos.x()), float(pos.y())
```

Create `ground_station/widgets/control_panel.py`:
```python
"""Left control panel: buttons, speed control, connection status."""
from __future__ import annotations

try:
    from PySide6.QtCore import Signal
    from PySide6.QtWidgets import (
        QDoubleSpinBox, QLabel, QPushButton, QVBoxLayout, QWidget,
    )
except ImportError:
    from PyQt5.QtCore import pyqtSignal as Signal
    from PyQt5.QtWidgets import (
        QDoubleSpinBox, QLabel, QPushButton, QVBoxLayout, QWidget,
    )


class ControlPanel(QWidget):
    planClicked = Signal()
    startClicked = Signal()
    stopClicked = Signal()
    clearClicked = Signal()
    addObstacleClicked = Signal()
    eraseClicked = Signal()
    waypointClicked = Signal()
    targetSpeedChanged = Signal(float)

    def __init__(self, default_speed: float = 0.2) -> None:
        super().__init__()
        self.setFixedWidth(200)
        layout = QVBoxLayout(self)

        self.waypoint_btn = QPushButton("Waypoint Mode")
        self.obstacle_btn = QPushButton("Add Obstacle")
        self.erase_btn = QPushButton("Erase Mode")
        self.plan_btn = QPushButton("Plan Path")
        self.start_btn = QPushButton("Start Control")
        self.stop_btn = QPushButton("Stop")
        self.clear_btn = QPushButton("Clear")

        self.speed_spin = QDoubleSpinBox()
        self.speed_spin.setRange(0.05, 1.0)
        self.speed_spin.setSingleStep(0.05)
        self.speed_spin.setValue(default_speed)
        self.speed_spin.setPrefix("Speed: ")

        self.robot_status = QLabel("Robot: Disconnected")
        self.robot_status.setStyleSheet("color: red; font-weight: bold;")
        self.rtsp_status = QLabel("RTSP: Disconnected")
        self.rtsp_status.setStyleSheet("color: orange; font-weight: bold;")

        for btn in (self.waypoint_btn, self.obstacle_btn, self.erase_btn,
                    self.plan_btn, self.start_btn, self.stop_btn, self.clear_btn):
            layout.addWidget(btn)
        layout.addWidget(self.speed_spin)
        layout.addWidget(self.robot_status)
        layout.addWidget(self.rtsp_status)
        layout.addStretch(1)

        self.waypoint_btn.clicked.connect(self.waypointClicked.emit)
        self.obstacle_btn.clicked.connect(self.addObstacleClicked.emit)
        self.erase_btn.clicked.connect(self.eraseClicked.emit)
        self.plan_btn.clicked.connect(self.planClicked.emit)
        self.start_btn.clicked.connect(self.startClicked.emit)
        self.stop_btn.clicked.connect(self.stopClicked.emit)
        self.clear_btn.clicked.connect(self.clearClicked.emit)
        self.speed_spin.valueChanged.connect(self.targetSpeedChanged.emit)

    def set_robot_connected(self, connected: bool) -> None:
        if connected:
            self.robot_status.setText("Robot: Connected")
            self.robot_status.setStyleSheet("color: #00cc00; font-weight: bold;")
        else:
            self.robot_status.setText("Robot: Disconnected")
            self.robot_status.setStyleSheet("color: red; font-weight: bold;")

    def set_rtsp_status(self, text: str, connected: bool) -> None:
        self.rtsp_status.setText(f"RTSP: {text}")
        color = "#00cc00" if connected else "orange"
        self.rtsp_status.setStyleSheet(f"color: {color}; font-weight: bold;")
```

Create `ground_station/widgets/status_bar.py`:
```python
"""Bottom status bar: pose, velocity, and rolling log."""
from __future__ import annotations

try:
    from PySide6.QtWidgets import QHBoxLayout, QLabel, QWidget
except ImportError:
    from PyQt5.QtWidgets import QHBoxLayout, QLabel, QWidget


class StatusBar(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setFixedHeight(36)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(8, 2, 8, 2)

        self.pose_label = QLabel("Pose: (0.00, 0.00, 0.0°)")
        self.vel_label = QLabel("Vel: vx=0.00 wz=0.00")
        self.log_label = QLabel("")
        self.log_label.setStyleSheet("color: #888;")

        layout.addWidget(self.pose_label)
        layout.addWidget(self.vel_label)
        layout.addStretch(1)
        layout.addWidget(self.log_label)

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        deg = yaw * 180.0 / 3.141592653589793
        self.pose_label.setText(f"Pose: ({x:.2f}, {y:.2f}, {deg:.1f}°)")

    def set_velocity(self, vx: float, wz: float) -> None:
        self.vel_label.setText(f"Vel: vx={vx:.2f} wz={wz:.2f}")

    def set_log(self, text: str) -> None:
        self.log_label.setText(text)
```

- [ ] **Step 2: Verify all widgets import**

Run: `cd /home/wufy/projects/DGGRS-air/camera/ag && uv run --extra gui python -c "from ground_station.widgets.image_view import ImageView; from ground_station.widgets.map_canvas import MapCanvas; from ground_station.widgets.control_panel import ControlPanel; from ground_station.widgets.status_bar import StatusBar; print('OK')"`
Expected: `OK`

- [ ] **Step 3: Commit**

```bash
cd /home/wufy/projects/DGGRS-air/camera/ag
git add ground_station/widgets/__init__.py ground_station/widgets/image_view.py ground_station/widgets/map_canvas.py ground_station/widgets/control_panel.py ground_station/widgets/status_bar.py
git commit -m "feat(widgets): add ImageView, MapCanvas, ControlPanel, StatusBar"
```

---

### Task 8: Main Window

**Files:**
- Create: `ground_station/widgets/main_window.py`

- [ ] **Step 1: Write the implementation**

Layout B: left panel + side-by-side images + bottom status bar. Integrates all services (RTSP, BEV, planner, robot server, simulator) and runs control loop via QTimer.

Create `ground_station/widgets/main_window.py`:
```python
"""Main window: Layout B with left panel, dual image area, and bottom status bar."""
from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

from ground_station.bev_processor import BevConfig, BevProcessor
from ground_station.planner import AStarPlanner, LineObstacle, PlannerConfig, compute_command
from ground_station.protocol import encode_cmd_vel, encode_reach_goal
from ground_station.robot_server import RobotServer
from ground_station.rtsp_worker import RtspWorker
from ground_station.simulator import Pose2D, Simulator

from ground_station.widgets.control_panel import ControlPanel
from ground_station.widgets.image_view import ImageView
from ground_station.widgets.map_canvas import MapCanvas, Pose2D as MapPose2D, VelocityCommand
from ground_station.widgets.status_bar import StatusBar

try:
    from PySide6.QtCore import QObject, QTimer, Signal
    from PySide6.QtWidgets import QHBoxLayout, QMainWindow, QMessageBox, QSplitter, QVBoxLayout, QWidget
except ImportError:
    from PyQt5.QtCore import QObject, QTimer, pyqtSignal as Signal
    from PyQt5.QtWidgets import QHBoxLayout, QMainWindow, QMessageBox, QSplitter, QVBoxLayout, QWidget


WorldPoint = Tuple[float, float]


class UiBridge(QObject):
    logLine = Signal(str)
    rawFrame = Signal(object)
    bevFrame = Signal(object)
    robotConnected = Signal(bool)
    rtspStatus = Signal(str, bool)


def _load_config(path: str) -> Dict:
    config_path = Path(path)
    if config_path.exists():
        return json.loads(config_path.read_text(encoding="utf-8"))
    return {}


class MainWindow(QMainWindow):
    def __init__(self, config: Dict) -> None:
        super().__init__()
        self.config = config
        self.setWindowTitle("AG Ground Station")
        self.resize(1280, 720)

        self.bridge = UiBridge()
        self.sim_pose = Pose2D()
        self.current_path: List[WorldPoint] = []
        self.path_index = 0
        self.goal_reached = True
        self.controlling = False
        self._bev_counter = 0

        # Services
        map_cfg = config.get("map", {})
        plan_cfg = config.get("planning", {})
        sim_cfg = config.get("simulator", {})
        self.planner_config = PlannerConfig(
            x_min=map_cfg.get("x_min", -1.0), x_max=map_cfg.get("x_max", 1.0),
            y_min=map_cfg.get("y_min", -1.0), y_max=map_cfg.get("y_max", 1.0),
            resolution_m=plan_cfg.get("resolution_m", 0.05),
            robot_radius_m=plan_cfg.get("robot_radius_m", 0.1),
            goal_tolerance_m=plan_cfg.get("goal_tolerance_m", 0.05),
        )
        self.planner = AStarPlanner(self.planner_config)
        self.simulator = Simulator(
            Pose2D(*sim_cfg.get("initial_pose", [0.0, 0.0, 0.0])),
            noise_xy=sim_cfg.get("noise_xy", 0.001),
            noise_yaw=sim_cfg.get("noise_yaw", 0.01),
        )

        self.robot_server = RobotServer(
            bind_host=config.get("server_bind_host", "0.0.0.0"),
            bind_port=int(config.get("server_bind_port", 46001)),
            on_connected=lambda: self.bridge.robotConnected.emit(True),
            on_disconnected=lambda: self.bridge.robotConnected.emit(False),
        )

        bev_cfg_data = config.get("bev", {})
        bev_cfg = BevConfig(**bev_cfg_data)
        self.bev_processor = BevProcessor(bev_cfg) if bev_cfg.enabled else None
        process_every_n = int(config.get("bev_process_every_n", 2))

        self.rtsp_worker = RtspWorker(
            rtsp_url=config.get("rtsp_url", ""),
            fps_limit=float(config.get("video_fps_limit", 20.0)),
            reconnect_interval=float(config.get("rtsp_reconnect_interval_sec", 2.0)),
            on_raw_frame=self._on_raw_frame,
            on_log=lambda text: self.bridge.logLine.emit(text),
        )

        self._build_ui()
        self._bind_signals()

        # Auto-start services
        self.robot_server.start()
        self.rtsp_worker.start()

        # Control loop timer
        control_hz = int(config.get("control_hz", 20))
        self._control_timer = QTimer(self)
        self._control_timer.setInterval(int(1000 / control_hz))
        self._control_timer.timeout.connect(self._control_step)

    def _on_raw_frame(self, frame: np.ndarray) -> None:
        self.bridge.rawFrame.emit(frame)
        self._bev_counter += 1
        if self.bev_processor is not None and self._bev_counter % int(self.config.get("bev_process_every_n", 2)) == 0:
            bev, _ = self.bev_processor.process(frame)
            self.bridge.bevFrame.emit(bev)

    def _build_ui(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        root_layout = QVBoxLayout(root)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)

        # Top: left panel + dual images
        top_layout = QHBoxLayout()

        self.control_panel = ControlPanel(
            default_speed=float(self.config.get("target_speed", 0.2))
        )

        self.raw_view = ImageView("RTSP Raw")
        self.map_canvas = MapCanvas(
            self.planner_config.x_min, self.planner_config.x_max,
            self.planner_config.y_min, self.planner_config.y_max,
        )

        image_splitter = QSplitter()
        image_splitter.addWidget(self.raw_view)
        image_splitter.addWidget(self.map_canvas)

        top_layout.addWidget(self.control_panel)
        top_layout.addWidget(image_splitter, 1)

        # Bottom: status bar
        self.status_bar = StatusBar()

        root_layout.addLayout(top_layout, 1)
        root_layout.addWidget(self.status_bar)

    def _bind_signals(self) -> None:
        self.control_panel.planClicked.connect(self._on_plan)
        self.control_panel.startClicked.connect(self._on_start_control)
        self.control_panel.stopClicked.connect(self._on_stop)
        self.control_panel.clearClicked.connect(self._on_clear)
        self.control_panel.addObstacleClicked.connect(lambda: self.map_canvas.set_mode("obstacle"))
        self.control_panel.eraseClicked.connect(lambda: self.map_canvas.set_mode("erase"))
        self.control_panel.waypointClicked.connect(lambda: self.map_canvas.set_mode("waypoint"))

        self.bridge.rawFrame.connect(lambda frame: self.raw_view.set_frame(frame))
        self.bridge.bevFrame.connect(lambda frame: self.map_canvas.set_background(frame))
        self.bridge.logLine.connect(self._on_log)
        self.bridge.robotConnected.connect(self.control_panel.set_robot_connected)

    def _on_log(self, text: str) -> None:
        self.status_bar.set_log(text)

    def _on_plan(self) -> None:
        try:
            start = (self.sim_pose.x, self.sim_pose.y)
            goals = self.map_canvas.overlay.waypoints
            if not goals:
                return
            obstacles = self.map_canvas.overlay.obstacles
            self.current_path = self.planner.plan(start, goals, obstacles)
            self.path_index = 0
            self.goal_reached = False
            self.map_canvas.set_path(self.current_path)
            self._on_log(f"Path planned: {len(self.current_path)} points")
        except Exception as exc:
            QMessageBox.critical(self, "Plan Failed", str(exc))

    def _on_start_control(self) -> None:
        if not self.robot_server.is_connected:
            self._on_log("Cannot start: robot not connected")
            return
        if len(self.current_path) < 2:
            self._on_log("Cannot start: no path")
            return
        self.path_index = 0
        self.goal_reached = False
        self.controlling = True
        self._control_timer.start()
        self._on_log("Control started")

    def _on_stop(self) -> None:
        self.controlling = False
        self._control_timer.stop()
        if self.robot_server.is_connected:
            self.robot_server.send(encode_cmd_vel(0.0, 0.0, 0.0))
        self._on_log("Control stopped")

    def _on_clear(self) -> None:
        self.current_path = []
        self.path_index = 0
        self.goal_reached = True
        self.controlling = False
        self._control_timer.stop()
        self.map_canvas.clear_annotations()

    def _control_step(self) -> None:
        if not self.controlling or self.path_index >= len(self.current_path):
            self._on_stop()
            return

        dt = self._control_timer.interval() / 1000.0
        target_speed = self.control_panel.speed_spin.value()

        # Use current goal point from path
        goal = self.current_path[self.path_index]
        vx, vy, wz, reached = compute_command(self.sim_pose, goal, target_speed)

        # Update simulator
        self.sim_pose = self.simulator.step(vx, vy, wz, dt)

        # Send to robot
        self.robot_server.send(encode_cmd_vel(vx, vy, wz))

        # Update UI
        map_pose = MapPose2D(self.sim_pose.x, self.sim_pose.y, self.sim_pose.yaw)
        map_cmd = VelocityCommand(vx, vy, wz)
        self.map_canvas.set_pose_and_cmd(map_pose, map_cmd)
        self.status_bar.set_pose(self.sim_pose.x, self.sim_pose.y, self.sim_pose.yaw)
        self.status_bar.set_velocity(vx, wz)

        if reached:
            self.path_index += 1
            if self.path_index >= len(self.current_path):
                self.robot_server.send(encode_reach_goal())
                self._on_log("Goal reached!")
                self._on_stop()

    def closeEvent(self, event) -> None:  # noqa: N802
        self._control_timer.stop()
        self.robot_server.stop()
        self.rtsp_worker.stop()
        super().closeEvent(event)
```

- [ ] **Step 2: Verify the module imports**

Run: `cd /home/wufy/projects/DGGRS-air/camera/ag && uv run --extra gui python -c "from ground_station.widgets.main_window import MainWindow; print('OK')"`
Expected: `OK`

- [ ] **Step 3: Commit**

```bash
cd /home/wufy/projects/DGGRS-air/camera/ag
git add ground_station/widgets/main_window.py
git commit -m "feat(ui): add main window with Layout B and control loop"
```

---

### Task 9: Entry Point

**Files:**
- Create: `ground_station/main.py`
- Modify: `scripts/run_ground_station.sh`

- [ ] **Step 1: Write the entry point**

Create `ground_station/main.py`:
```python
"""Ground station entry point."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from ground_station.widgets.main_window import MainWindow, _load_config

try:
    from PySide6.QtWidgets import QApplication
except ImportError:
    from PyQt5.QtWidgets import QApplication


def main() -> None:
    parser = argparse.ArgumentParser(description="AG Ground Station")
    parser.add_argument("--config", default="config/ground_station.json")
    args = parser.parse_args()
    config = _load_config(args.config)
    app = QApplication(sys.argv)
    window = MainWindow(config)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Update the launch script**

Update `scripts/run_ground_station.sh` to use the new entry point:

```bash
#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
uv run --extra gui --project "$ROOT_DIR" python -m ground_station.main --config "${1:-config/ground_station.json}"
```

- [ ] **Step 3: Verify the app starts**

Run: `cd /home/wufy/projects/DGGRS-air/camera/ag && timeout 3 uv run --extra gui python -m ground_station.main --config config/ground_station.json || true`
Expected: Application window appears briefly (timeout kills it). No import errors.

- [ ] **Step 4: Commit**

```bash
cd /home/wufy/projects/DGGRS-air/camera/ag
git add ground_station/main.py scripts/run_ground_station.sh
git commit -m "feat: add ground station entry point and update launch script"
```

---

### Task 10: Robot Receiver (Modified)

**Files:**
- Modify: `ground_robot/receiver.py`
- Modify: `scripts/run_ground_robot.sh`
- Create: `config/ground_robot.json`

- [ ] **Step 1: Write the new receiver**

The robot now connects to GCS (not sky_uav). Simplified protocol: only receives `cmd_vel` and `reach_goal`. No sending acks back.

Rewrite `ground_robot/receiver.py`:
```python
"""Ground robot endpoint: connect to GCS and execute cmd_vel commands."""
from __future__ import annotations

import argparse
import json
import logging
import socket
import time
from pathlib import Path
from typing import Any, Dict

from ground_station.protocol import decode_message, validate_cmd_vel, validate_reach_goal

LOGGER = logging.getLogger(__name__)


class GroundRobotReceiver:
    def __init__(self, config: Dict[str, Any]) -> None:
        self.server_host = str(config.get("server_host", "127.0.0.1"))
        self.server_port = int(config.get("server_port", 46001))
        self.reconnect_sec = max(0.2, float(config.get("reconnect_sec", 1.0)))

    def run_forever(self) -> None:
        while True:
            try:
                LOGGER.info("Connecting to GCS %s:%s", self.server_host, self.server_port)
                with socket.create_connection((self.server_host, self.server_port), timeout=3.0) as sock:
                    sock.settimeout(1.0)
                    LOGGER.info("Connected to GCS")
                    self._read_loop(sock)
            except KeyboardInterrupt:
                raise
            except Exception as exc:
                LOGGER.warning("Connection failed/disconnected: %s", exc)
                time.sleep(self.reconnect_sec)

    def _read_loop(self, sock: socket.socket) -> None:
        buffer = b""
        while True:
            try:
                chunk = sock.recv(65536)
            except socket.timeout:
                continue
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
            msg = decode_message(line)
        except Exception as exc:
            LOGGER.warning("Invalid message: %s", exc)
            return

        if msg.get("type") == "cmd_vel":
            try:
                cmd = validate_cmd_vel(msg)
            except ValueError as exc:
                LOGGER.warning("Invalid cmd_vel: %s", exc)
                return
            print(
                f"[{time.strftime('%H:%M:%S')}] cmd_vel "
                f"vx={cmd['vx']:.3f} vy={cmd['vy']:.3f} wz={cmd['wz']:.3f}",
                flush=True,
            )
        elif msg.get("type") == "reach_goal":
            try:
                validate_reach_goal(msg)
            except ValueError as exc:
                LOGGER.warning("Invalid reach_goal: %s", exc)
                return
            print(f"[{time.strftime('%H:%M:%S')}] reach_goal", flush=True)
            LOGGER.info("Goal reached, stopping")
        else:
            LOGGER.warning("Unknown message type: %s", msg.get("type"))


def load_config(path: str) -> Dict[str, Any]:
    config_path = Path(path)
    if config_path.exists():
        return json.loads(config_path.read_text(encoding="utf-8"))
    return {}


def main() -> None:
    parser = argparse.ArgumentParser(description="Ground robot receiver")
    parser.add_argument("--config", default="config/ground_robot.json")
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args()
    logging.basicConfig(
        level=getattr(logging, str(args.log_level).upper(), logging.INFO),
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    )
    GroundRobotReceiver(load_config(args.config)).run_forever()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Create robot config**

Create `config/ground_robot.json`:
```json
{
  "server_host": "127.0.0.1",
  "server_port": 46001,
  "reconnect_sec": 1.0
}
```

- [ ] **Step 3: Update launch script**

Update `scripts/run_ground_robot.sh`:
```bash
#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
uv run --project "$ROOT_DIR" python -m ground_robot.receiver --config "${1:-config/ground_robot.json}"
```

- [ ] **Step 4: Verify receiver starts and connects**

Run: `cd /home/wufy/projects/DGGRS-air/camera/ag && timeout 3 uv run python -m ground_robot.receiver --config config/ground_robot.json || true`
Expected: Logs "Connecting to GCS 127.0.0.1:46001" then "Connection refused" (no GCS server running). No import errors.

- [ ] **Step 5: Commit**

```bash
cd /home/wufy/projects/DGGRS-air/camera/ag
git add ground_robot/receiver.py config/ground_robot.json scripts/run_ground_robot.sh
git commit -m "feat(robot): rewrite receiver to connect to GCS with simplified protocol"
```

---

### Task 11: Config Update

**Files:**
- Modify: `config/ground_station.json`

- [ ] **Step 1: Update the ground station config**

Replace `config/ground_station.json` with the new schema:
```json
{
  "server_bind_host": "0.0.0.0",
  "server_bind_port": 46001,
  "rtsp_url": "rtsp://10.28.215.179:8554/cam",
  "rtsp_reconnect_interval_sec": 2.0,
  "video_fps_limit": 20.0,
  "bev_process_every_n": 2,
  "target_speed": 0.2,
  "control_hz": 20,
  "simulator": {
    "noise_xy": 0.001,
    "noise_yaw": 0.01,
    "initial_pose": [0.0, 0.0, 0.0]
  },
  "planning": {
    "resolution_m": 0.05,
    "robot_radius_m": 0.1,
    "goal_tolerance_m": 0.05
  },
  "map": {
    "x_min": -1.0,
    "x_max": 1.0,
    "y_min": -1.0,
    "y_max": 1.0,
    "ppm": 300
  },
  "bev": {
    "enabled": true,
    "calibration_path": "config/calibration.yaml",
    "target_id": 0,
    "tag_size": 0.09,
    "x_min": -1.0,
    "x_max": 1.0,
    "y_min": -1.0,
    "y_max": 1.0,
    "ppm": 300
  }
}
```

- [ ] **Step 2: Commit**

```bash
cd /home/wufy/projects/DGGRS-air/camera/ag
git add config/ground_station.json
git commit -m "feat(config): update ground_station config for new 2-tier architecture"
```

---

### Task 12: Run All Tests

**Files:**
- No new files

- [ ] **Step 1: Run the full test suite**

Run: `cd /home/wufy/projects/DGGRS-air/camera/ag && uv run python -m pytest tests/test_protocol.py tests/test_simulator.py tests/test_planner.py -v`
Expected: All tests PASS

- [ ] **Step 2: Run a quick integration smoke test**

Start GCS and robot together to verify TCP communication:

Terminal 1 (GCS):
```bash
cd /home/wufy/projects/DGGRS-air/camera/ag && timeout 5 uv run --extra gui python -m ground_station.main --config config/ground_station.json || true
```

Terminal 2 (Robot, after GCS starts):
```bash
cd /home/wufy/projects/DGGRS-air/camera/ag && timeout 4 uv run python -m ground_robot.receiver --config config/ground_robot.json || true
```

Expected: Robot logs "Connected to GCS", GCS shows "Robot: Connected" in the control panel.

- [ ] **Step 3: Final commit if any fixes needed**

```bash
cd /home/wufy/projects/DGGRS-air/camera/ag
git add -A
git commit -m "fix: integration test fixes"
```
