"""Local robot TCP client backed by `air_ground.simulator.core`."""
from __future__ import annotations

import socket
import threading
import time
from typing import Optional

import numpy as np

from common.air_ground_bridge import (
    CameraCalibration,
    PlannerConfig,
    Pose2D,
    RobotLightBarConfig,
    ScenarioTag,
    SimulatorCore,
    SimulatorRobotConfig,
    SimulatorScenario,
    VelocityCommand,
    WorldBounds,
)
from common.protocol import decode_message


def _default_scenario() -> SimulatorScenario:
    return SimulatorScenario(
        name="local_robot_client",
        image_size=(1, 1),
        fps=20.0,
        world_bounds=WorldBounds(width_m=4.0, height_m=4.0),
        camera_height_m=1.0,
        calibration=CameraCalibration(camera_matrix=np.eye(3, dtype=float), distortion_coeffs=np.zeros(5, dtype=float)),
        tags=(ScenarioTag(tag_id=0, size_m=0.1, world_x=0.0, world_y=0.0, yaw=0.0),),
        robot=SimulatorRobotConfig(
            initial_pose=Pose2D(x=0.0, y=0.0, yaw=0.0),
            light_bar=RobotLightBarConfig(length_m=0.28, width_m=0.14, bgr_color=(255, 0, 0)),
        ),
        planner=PlannerConfig(),
        obstacles=(),
    )


class LocalSimRobotClient:
    def __init__(self, host: str, port: int, *, reconnect_sec: float = 0.2) -> None:
        self.host = host
        self.port = port
        self.reconnect_sec = reconnect_sec
        self.core = SimulatorCore(_default_scenario())
        self.reach_goal_count = 0
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

    @property
    def pose(self) -> Pose2D:
        return self.core.state.robot_pose_world

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                with socket.create_connection((self.host, self.port), timeout=1.0) as sock:
                    sock.settimeout(0.05)
                    self._read_loop(sock)
            except OSError:
                time.sleep(self.reconnect_sec)

    def _read_loop(self, sock: socket.socket) -> None:
        last_tick = time.monotonic()
        buffer = b""
        while not self._stop_event.is_set():
            now = time.monotonic()
            self.core.step(max(0.0, min(now - last_tick, 0.1)))
            last_tick = now
            try:
                chunk = sock.recv(65536)
            except socket.timeout:
                continue
            if not chunk:
                return
            buffer += chunk
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                if line.strip():
                    self._handle_message(decode_message(line))

    def _handle_message(self, message: dict[str, object]) -> None:
        if message.get("type") == "cmd_vel":
            self.core.apply_command(
                VelocityCommand(
                    v_x=float(message["vx"]),
                    v_y=float(message["vy"]),
                    w_z=float(message["wz"]),
                    timestamp=float(message["timestamp"]),
                )
            )
            return
        if message.get("type") == "reach_goal":
            self.reach_goal_count += 1
            self.core.apply_command(VelocityCommand(v_x=0.0, v_y=0.0, w_z=0.0, timestamp=float(message["timestamp"])))
