"""Shared A* planning and omnidirectional P-controller execution backend for the GCS."""
from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass
from typing import Any, Optional, Sequence

from common.air_ground_bridge import Pose2D, VelocityCommand, normalize_angle, world_to_robot_frame
from ground_station.gcs_server import GcsServer
from ground_station.grid_map import GridConfig, GridMapper, LineObstacle, RectObstacle, WorldPoint
from ground_station.state_machine import RuntimeStateMachine


class Simulator:
    """Dead-reckoning integrator: cmd_vel -> x/y/yaw."""

    def __init__(
        self,
        initial_pose: tuple[float, float, float] = (0.0, 0.0, 0.0),
        noise_xy: float = 0.0,
        noise_yaw: float = 0.0,
    ) -> None:
        self.x, self.y, self.yaw = initial_pose
        self._noise_xy = noise_xy
        self._noise_yaw = noise_yaw
        self._last_tick: float | None = None

    @property
    def pose(self) -> Pose2D:
        return Pose2D(x=self.x, y=self.y, yaw=self.yaw)

    def step(self, vx: float, vy: float, wz: float, timestamp: float) -> Pose2D:
        if self._last_tick is not None:
            dt = max(0.0, min(timestamp - self._last_tick, 0.1))
            half_wz = wz * dt * 0.5
            cos_h = math.cos(self.yaw + half_wz)
            sin_h = math.sin(self.yaw + half_wz)
            self.x += (vx * cos_h - vy * sin_h) * dt
            self.y += (vx * sin_h + vy * cos_h) * dt
            self.yaw = normalize_angle(self.yaw + wz * dt)
        self._last_tick = timestamp
        return self.pose

    def reset(self, pose: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> None:
        self.x, self.y, self.yaw = pose
        self._last_tick = None


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(frozen=True)
class ControlConfig:
    control_hz: float = 20.0
    lookahead_m: float = 0.15
    goal_tolerance_m: float = 0.08
    kx: float = 1.5
    ky: float = 2.0
    kth: float = 3.0
    max_vx: float = 0.50
    max_vy: float = 0.50
    max_wz: float = 1.0
    ax_max: float = 1.0
    ay_max: float = 1.0
    aw_max: float = 2.0
    min_dist: float = 0.05
    min_angle_deg: float = 8.0


class AStarPlanner:
    """Very small 8-neighbor A* on the shared grid mapper."""

    def __init__(self, mapper: GridMapper) -> None:
        self.mapper = mapper

    def plan(
        self,
        start: WorldPoint,
        goal: WorldPoint,
        rects: Sequence[RectObstacle],
        lines: Sequence[LineObstacle],
        occupancy: list[list[bool]] | None = None,
    ) -> list[WorldPoint]:
        occupancy = occupancy or self.mapper.build_occupancy(rects, lines)
        start_cell = self._nearest_free(self.mapper.world_to_grid(*start), occupancy)
        goal_cell = self._nearest_free(self.mapper.world_to_grid(*goal), occupancy)
        if start_cell is None:
            raise ValueError("start cell is blocked")
        if goal_cell is None:
            raise ValueError("goal cell is blocked")
        if self._line_of_sight(start_cell, goal_cell, occupancy):
            return [start, goal]

        frontier: list[tuple[float, tuple[int, int]]] = [(0.0, start_cell)]
        came_from: dict[tuple[int, int], tuple[int, int] | None] = {start_cell: None}
        cost_so_far: dict[tuple[int, int], float] = {start_cell: 0.0}

        while frontier:
            _, current = heapq.heappop(frontier)
            if current == goal_cell:
                break
            for nxt, step_cost in self._neighbors(current, occupancy):
                new_cost = cost_so_far[current] + step_cost
                if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                    cost_so_far[nxt] = new_cost
                    priority = new_cost + self._heuristic(nxt, goal_cell)
                    heapq.heappush(frontier, (priority, nxt))
                    came_from[nxt] = current

        if goal_cell not in came_from:
            raise ValueError("planner failed to find path")

        path_cells: list[tuple[int, int]] = []
        current: tuple[int, int] | None = goal_cell
        while current is not None:
            path_cells.append(current)
            current = came_from[current]
        path_cells.reverse()

        path = [start]
        for cell in path_cells[1:-1]:
            path.append(self.mapper.grid_to_world(*cell))
        path.append(goal)
        return self._shortcut(path, occupancy)

    def _neighbors(
        self,
        cell: tuple[int, int],
        occupancy: list[list[bool]],
    ) -> list[tuple[tuple[int, int], float]]:
        x, y = cell
        result: list[tuple[tuple[int, int], float]] = []
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx = x + dx
                ny = y + dy
                if 0 <= nx < self.mapper.width and 0 <= ny < self.mapper.height and not occupancy[ny][nx]:
                    result.append(((nx, ny), math.hypot(dx, dy)))
        return result

    def _nearest_free(
        self,
        cell: tuple[int, int],
        occupancy: list[list[bool]],
    ) -> tuple[int, int] | None:
        x, y = cell
        if not occupancy[y][x]:
            return cell
        radius_limit = max(self.mapper.width, self.mapper.height)
        for radius in range(1, radius_limit):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if max(abs(dx), abs(dy)) != radius:
                        continue
                    nx = x + dx
                    ny = y + dy
                    if 0 <= nx < self.mapper.width and 0 <= ny < self.mapper.height and not occupancy[ny][nx]:
                        return (nx, ny)
        return None

    def _line_of_sight(
        self,
        start: tuple[int, int],
        goal: tuple[int, int],
        occupancy: list[list[bool]],
    ) -> bool:
        x0, y0 = start
        x1, y1 = goal
        steps = max(abs(x1 - x0), abs(y1 - y0), 1)
        for index in range(steps + 1):
            t = index / steps
            ix = int(round(x0 + (x1 - x0) * t))
            iy = int(round(y0 + (y1 - y0) * t))
            if occupancy[iy][ix]:
                return False
        return True

    def _shortcut(self, path: Sequence[WorldPoint], occupancy: list[list[bool]]) -> list[WorldPoint]:
        result = [path[0]]
        start_index = 0
        while start_index < len(path) - 1:
            end_index = len(path) - 1
            while end_index > start_index + 1:
                if self._line_of_sight(
                    self.mapper.world_to_grid(*path[start_index]),
                    self.mapper.world_to_grid(*path[end_index]),
                    occupancy,
                ):
                    break
                end_index -= 1
            result.append(path[end_index])
            start_index = end_index
        return result

    @staticmethod
    def _heuristic(a: tuple[int, int], b: tuple[int, int]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])


class OmniController:
    """Omnidirectional P-controller: projects onto path, looks ahead, drives in body frame."""

    def __init__(self, config: dict[str, Any]) -> None:
        self.cfg = ControlConfig(**{**ControlConfig().__dict__, **config})
        self.path: list[WorldPoint] = []
        self._prev_cmd: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._prev_time: float | None = None
        self.nearest_point: WorldPoint | None = None
        self.target_point: WorldPoint | None = None

    def set_path(self, path: Sequence[WorldPoint]) -> None:
        self.path = _simplify_path([(float(x), float(y)) for x, y in path], self.cfg.min_dist, self.cfg.min_angle_deg)
        self._prev_cmd = (0.0, 0.0, 0.0)
        self._prev_time = None

    def clear_path(self) -> None:
        self.path = []
        self._prev_cmd = (0.0, 0.0, 0.0)
        self._prev_time = None
        self.nearest_point = None
        self.target_point = None

    def compute_command(self, pose: Pose2D, *, timestamp: float) -> tuple[VelocityCommand, bool]:
        if len(self.path) < 2:
            return self._zero(timestamp), False

        # Goal check
        gx, gy = self.path[-1]
        if math.hypot(gx - pose.x, gy - pose.y) <= self.cfg.goal_tolerance_m:
            return self._zero(timestamp), True

        # Nearest projection on path segments
        nearest = _nearest_projection(pose.x, pose.y, self.path)
        self.nearest_point = (nearest["qx"], nearest["qy"])

        # Lookahead point + reference heading
        tx, ty, yaw_ref = _lookahead_point(self.path, nearest, self.cfg.lookahead_m)
        self.target_point = (tx, ty)

        # World-frame error → body-frame error
        ex, ey = world_to_robot_frame(pose, (tx, ty))
        etheta = normalize_angle(yaw_ref - pose.yaw)

        # P-control
        vx = self.cfg.kx * ex
        vy = self.cfg.ky * ey
        wz = self.cfg.kth * etheta

        # Clamp to max velocities
        vx = _clamp(vx, -self.cfg.max_vx, self.cfg.max_vx)
        vy = _clamp(vy, -self.cfg.max_vy, self.cfg.max_vy)
        wz = _clamp(wz, -self.cfg.max_wz, self.cfg.max_wz)

        # Rate-limit (acceleration clamp)
        dt = max(timestamp - self._prev_time, 1e-6) if self._prev_time is not None else 0.05
        px, py, pw = self._prev_cmd
        vx = _clamp(vx, px - self.cfg.ax_max * dt, px + self.cfg.ax_max * dt)
        vy = _clamp(vy, py - self.cfg.ay_max * dt, py + self.cfg.ay_max * dt)
        wz = _clamp(wz, pw - self.cfg.aw_max * dt, pw + self.cfg.aw_max * dt)

        self._prev_cmd = (vx, vy, wz)
        self._prev_time = timestamp
        return VelocityCommand(v_x=vx, v_y=vy, w_z=wz, timestamp=timestamp), False

    @staticmethod
    def _zero(timestamp: float) -> VelocityCommand:
        return VelocityCommand(v_x=0.0, v_y=0.0, w_z=0.0, timestamp=timestamp)


# ---------------------------------------------------------------------------
# Path helpers used by OmniController
# ---------------------------------------------------------------------------

def _simplify_path(
    points: list[WorldPoint],
    min_dist: float,
    min_angle_deg: float,
) -> list[WorldPoint]:
    """Remove close-together and nearly-collinear points."""
    if len(points) <= 2:
        return points[:]

    # Pass 1: drop points too close together
    filtered: list[WorldPoint] = [points[0]]
    for p in points[1:]:
        if math.hypot(p[0] - filtered[-1][0], p[1] - filtered[-1][1]) >= min_dist:
            filtered.append(p)
    if len(filtered) <= 2:
        return filtered

    # Pass 2: drop nearly-collinear interior points
    min_angle = math.radians(min_angle_deg)
    out: list[WorldPoint] = [filtered[0]]
    for i in range(1, len(filtered) - 1):
        p0 = out[-1]
        p1 = filtered[i]
        p2 = filtered[i + 1]
        v1x, v1y = p1[0] - p0[0], p1[1] - p0[1]
        v2x, v2y = p2[0] - p1[0], p2[1] - p1[1]
        n1 = math.hypot(v1x, v1y)
        n2 = math.hypot(v2x, v2y)
        if n1 < 1e-9 or n2 < 1e-9:
            continue
        cos_a = _clamp((v1x * v2x + v1y * v2y) / (n1 * n2), -1.0, 1.0)
        if math.acos(cos_a) >= min_angle:
            out.append(p1)
    out.append(filtered[-1])
    return out


def _nearest_projection(
    px: float, py: float, path: list[WorldPoint],
) -> dict:
    """Find the closest point on the polyline to (px, py)."""
    best_dist = 1e18
    best: dict = {"seg": 0, "qx": path[0][0], "qy": path[0][1], "t": 0.0}
    for i in range(len(path) - 1):
        ax, ay = path[i]
        bx, by = path[i + 1]
        dx, dy = bx - ax, by - ay
        l2 = dx * dx + dy * dy
        if l2 < 1e-12:
            t = 0.0
        else:
            t = _clamp(((px - ax) * dx + (py - ay) * dy) / l2, 0.0, 1.0)
        qx = ax + t * dx
        qy = ay + t * dy
        d = math.hypot(px - qx, py - qy)
        if d < best_dist:
            best_dist = d
            best = {"seg": i, "qx": qx, "qy": qy, "t": t}
    return best


def _lookahead_point(
    path: list[WorldPoint],
    nearest: dict,
    lookahead_dist: float,
) -> tuple[float, float, float]:
    """Walk forward along path from nearest projection by lookahead_dist.

    Returns (target_x, target_y, reference_yaw).
    """
    i = nearest["seg"]
    qx, qy = nearest["qx"], nearest["qy"]

    # Remaining distance on current segment
    bx, by = path[i + 1]
    remain = math.hypot(bx - qx, by - qy)

    if remain >= lookahead_dist:
        ratio = lookahead_dist / max(remain, 1e-12)
        tx = qx + ratio * (bx - qx)
        ty = qy + ratio * (by - qy)
        return tx, ty, math.atan2(by - qy, bx - qx)

    # Walk subsequent segments
    dist_left = lookahead_dist - remain
    for j in range(i + 1, len(path) - 1):
        ax, ay = path[j]
        bx, by = path[j + 1]
        seg_len = math.hypot(bx - ax, by - ay)
        if seg_len >= dist_left:
            ratio = dist_left / max(seg_len, 1e-12)
            tx = ax + ratio * (bx - ax)
            ty = ay + ratio * (by - ay)
            return tx, ty, math.atan2(by - ay, bx - ax)
        dist_left -= seg_len

    # Past the end — target is the last point, heading from second-to-last segment
    ax, ay = path[-2]
    bx, by = path[-1]
    return bx, by, math.atan2(by - ay, bx - ax)


class GroundStationRuntime:
    """Owns planning, execution state, and the robot TCP server."""

    def __init__(self, config: dict[str, dict[str, Any]], *, clock=time.time) -> None:
        self.clock = clock
        self.mapper = GridMapper(
            GridConfig(
                x_min=float(config["map"]["x_min"]),
                x_max=float(config["map"]["x_max"]),
                y_min=float(config["map"]["y_min"]),
                y_max=float(config["map"]["y_max"]),
                resolution_m=float(config["grid"]["resolution_m"]),
                ppm=int(config["map"]["ppm"]),
                line_obstacle_width_m=float(config["grid"]["line_obstacle_width_m"]),
            )
        )
        self.planner = AStarPlanner(self.mapper)
        self.controller = OmniController(config.get("control", {}))
        self.server = GcsServer(
            host=str(config["server"]["host"]),
            port=int(config["server"]["port"]),
        )
        self.state = RuntimeStateMachine()
        sim_cfg = config.get("simulator", {})
        self.simulator = Simulator(
            initial_pose=tuple(sim_cfg.get("initial_pose", [0.0, 0.0, 0.0])),
            noise_xy=float(sim_cfg.get("noise_xy", 0.0)),
            noise_yaw=float(sim_cfg.get("noise_yaw", 0.0)),
        )
        self.current_pose: Optional[Pose2D] = self.simulator.pose
        self.current_goal: Optional[WorldPoint] = None
        self.current_path: list[WorldPoint] = []
        self.occupancy: list[list[bool]] = self.mapper.build_occupancy([], [])

    @property
    def bound_address(self) -> tuple[str, int]:
        return self.server.bound_address

    @property
    def connected(self) -> bool:
        return self.server.connected

    def start(self) -> None:
        self.server.start()

    def stop(self) -> None:
        self.server.stop()

    def update_pose(self, pose: Pose2D | None) -> None:
        self.current_pose = pose

    def plan(
        self,
        goal: WorldPoint,
        rects: Sequence[RectObstacle],
        lines: Sequence[LineObstacle],
    ) -> list[WorldPoint]:
        if not self.state.editing_allowed:
            raise ValueError("cannot plan while executing")
        if self.current_pose is None:
            raise ValueError("missing robot pose")
        self.occupancy = self.mapper.build_occupancy(rects, lines)
        start = (self.current_pose.x, self.current_pose.y)
        self.current_goal = (float(goal[0]), float(goal[1]))
        raw_path = self.planner.plan(start, self.current_goal, rects, lines, occupancy=self.occupancy)
        self.controller.set_path(raw_path)
        self.current_path = list(self.controller.path)
        self.state.to_planned()
        return list(self.current_path)

    def start_execution(self) -> None:
        if not self.state.can_execute(
            has_goal=self.current_goal is not None,
            has_path=len(self.current_path) >= 2,
            has_pose=self.current_pose is not None,
            robot_connected=self.connected,
        ):
            raise ValueError("execution prerequisites are not satisfied")
        self.state.to_executing()

    def cancel(self) -> None:
        if self.connected:
            self.server.send_cmd_vel(vx=0.0, vy=0.0, wz=0.0, timestamp=self.clock())
        self.controller.clear_path()
        self.current_goal = None
        self.current_path = []
        self.state.cancel()
        self.state.finish_cancel()

    def tick(self) -> VelocityCommand:
        timestamp = float(self.clock())
        if self.state.state != "executing" or self.current_pose is None:
            self.simulator.step(0.0, 0.0, 0.0, timestamp)
            return VelocityCommand(v_x=0.0, v_y=0.0, w_z=0.0, timestamp=timestamp)
        if not self.connected:
            self.cancel()
            return VelocityCommand(v_x=0.0, v_y=0.0, w_z=0.0, timestamp=timestamp)

        command, goal_reached = self.controller.compute_command(self.current_pose, timestamp=timestamp)
        self.server.send_cmd_vel(vx=command.v_x, vy=command.v_y, wz=command.w_z, timestamp=command.timestamp)
        self.current_pose = self.simulator.step(command.v_x, command.v_y, command.w_z, timestamp)
        if goal_reached:
            self.server.send_reach_goal(timestamp=timestamp)
            self.controller.clear_path()
            self.current_goal = None
            self.current_path = []
            self.state.finish_execution()
        return command
