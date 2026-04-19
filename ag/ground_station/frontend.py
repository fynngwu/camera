"""Ground station Qt frontend: RTSP/BEV view, planning, and control visualization.

Uses GroundStationRuntime for planning, path tracking, and robot communication.
TCP server and RTSP reader start automatically when the window opens.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
import traceback
from typing import Optional

import numpy as np

from common.air_ground_bridge import Pose2D, VelocityCommand
from ground_station.bev_backend import BevBackend, BevConfig
from ground_station.config import load_config
from ground_station.grid_map import GridMapper, LineObstacle, RectObstacle, WorldPoint
from ground_station.map_editor import MapEditor, ToolMode
from ground_station.navigation_backend import GroundStationRuntime

try:
    from PySide6.QtCore import QObject, QPointF, QRectF, Qt, Signal, QTimer
    from PySide6.QtGui import QColor, QBrush, QImage, QPainter, QPen
    from PySide6.QtWidgets import (
        QApplication,
        QFrame,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QPlainTextEdit,
        QSizePolicy,
        QSplitter,
        QVBoxLayout,
        QWidget,
    )
except ImportError:  # pragma: no cover
    from PyQt5.QtCore import QObject, QPointF, QRectF, Qt, pyqtSignal as Signal, QTimer
    from PyQt5.QtGui import QColor, QBrush, QImage, QPainter, QPen
    from PyQt5.QtWidgets import (
        QApplication,
        QFrame,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QPlainTextEdit,
        QSizePolicy,
        QSplitter,
        QVBoxLayout,
        QWidget,
    )


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def ndarray_to_qimage(frame: Optional[np.ndarray]) -> Optional[QImage]:
    """Convert a numpy BGR / gray frame into a detached QImage."""
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


# ---------------------------------------------------------------------------
# Signal bridge
# ---------------------------------------------------------------------------

class UiBridge(QObject):
    """Thread-safe signal bridge from background threads to Qt widgets."""
    logLine = Signal(str, str)
    rawFrame = Signal(object)
    bevFrame = Signal(object)
    bevMeta = Signal(object)


# ---------------------------------------------------------------------------
# Widgets
# ---------------------------------------------------------------------------

class ImageView(QWidget):
    """Simple image panel with title overlay."""

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


class MapCanvas(QWidget):
    """Interactive BEV map canvas using the shared GridMapper for coordinates.

    Delegates editing to MapEditor. Draws grid overlay, occupancy, obstacles,
    goal, planned path, robot pose, and command arrow.
    """

    # Right-click tool cycle: LINE -> GOAL -> (plan) -> (execute)
    _TOOL_CYCLE = [ToolMode.DRAW_LINE, ToolMode.SET_GOAL]

    def __init__(self, mapper: GridMapper) -> None:
        super().__init__()
        self.mapper = mapper
        self.editor = MapEditor()
        self.editor.set_tool(ToolMode.DRAW_LINE)
        self.pose: Optional[Pose2D] = None
        self.cmd = VelocityCommand(v_x=0.0, v_y=0.0, w_z=0.0, timestamp=0.0)
        self.current_path: list[WorldPoint] = []
        self.occupancy: list[list[bool]] | None = None
        self._background: Optional[QImage] = None
        self._mouse_world: Optional[WorldPoint] = None
        self._tool_cycle_idx: int = 0
        self.on_tool_cycle: Optional[callable] = None
        self.setMinimumSize(420, 320)
        self.setMouseTracking(True)

    def set_background(self, frame: np.ndarray) -> None:
        self._background = ndarray_to_qimage(frame)
        self.update()

    def set_pose_and_cmd(self, pose: Optional[Pose2D], cmd: VelocityCommand) -> None:
        self.pose = pose
        self.cmd = cmd
        self.update()

    def set_path(self, path: list[WorldPoint]) -> None:
        self.current_path = list(path)
        self.update()

    def set_occupancy(self, grid: list[list[bool]]) -> None:
        self.occupancy = grid
        self.update()

    def clear_overlay(self) -> None:
        self.current_path = []
        self.occupancy = None
        self.update()

    # --- Qt events --------------------------------------------------------

    def paintEvent(self, event) -> None:  # noqa: N802
        try:
            self._do_paint(event)
        except Exception as exc:
            print(f"[paintEvent ERROR] {exc}", flush=True)
            import traceback
            traceback.print_exc()

    def _do_paint(self, event) -> None:
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(24, 24, 24))
        draw_rect = QRectF(self.rect().adjusted(8, 8, -8, -8))
        if self._background is not None:
            scaled = self._background.scaled(draw_rect.size().toSize(), Qt.IgnoreAspectRatio, Qt.SmoothTransformation)
            painter.drawImage(draw_rect, scaled)
        self._draw_occupancy(painter, draw_rect)
        self._draw_obstacles(painter, draw_rect)
        self._draw_goal(painter, draw_rect)
        self._draw_path(painter, draw_rect)
        self._draw_pose(painter, draw_rect)
        self._draw_cmd_arrow(painter, draw_rect)
        self._draw_drag_preview(painter, draw_rect)
        painter.setPen(QColor(255, 255, 255))
        painter.drawText(16, 24, f"tool={self.editor.tool.value}")

    def mousePressEvent(self, event) -> None:  # noqa: N802
        world = self._event_to_world(event)
        if world is None:
            return
        if event.button() == Qt.LeftButton:
            self.editor.left_click(world)
        elif event.button() == Qt.RightButton:
            self.editor.right_click(world)
            self._cycle_tool()
        self.update()

    def mouseMoveEvent(self, event) -> None:  # noqa: N802
        self._mouse_world = self._event_to_world(event)
        self.update()

    # --- Drawing ----------------------------------------------------------

    def _draw_occupancy(self, painter: QPainter, draw_rect: QRectF) -> None:
        if self.occupancy is None:
            return
        h = len(self.occupancy)
        w = len(self.occupancy[0]) if h > 0 else 0
        if w == 0:
            return
        img = QImage(w, h, QImage.Format_ARGB32)
        img.fill(QColor(0, 0, 0, 0))
        for iy in range(h):
            for ix in range(w):
                if self.occupancy[iy][ix]:
                    img.setPixelColor(ix, h - 1 - iy, QColor(255, 80, 80, 50))
        scaled = img.scaled(draw_rect.size().toSize(), Qt.IgnoreAspectRatio, Qt.FastTransformation)
        painter.drawImage(draw_rect, scaled)

    def _draw_obstacles(self, painter: QPainter, draw_rect: QRectF) -> None:
        # Rectangles
        painter.setPen(QPen(QColor(255, 120, 120), 2))
        painter.setBrush(QBrush(QColor(255, 120, 120, 70)))
        for rect in self.editor.rectangles:
            norm = rect.normalized()
            p0 = self._world_to_widget(norm.x_min, norm.y_max, draw_rect)
            p1 = self._world_to_widget(norm.x_max, norm.y_min, draw_rect)
            if p0 and p1:
                painter.drawRect(QRectF(p0, p1).normalized())
        # Lines
        painter.setPen(QPen(QColor(255, 120, 120), 4))
        painter.setBrush(Qt.NoBrush)
        for line in self.editor.lines:
            prev = None
            for point in line.points:
                p = self._world_to_widget(point[0], point[1], draw_rect)
                if p and prev:
                    painter.drawLine(prev, p)
                prev = p

    def _draw_goal(self, painter: QPainter, draw_rect: QRectF) -> None:
        if self.editor.goal is None:
            return
        p = self._world_to_widget(self.editor.goal[0], self.editor.goal[1], draw_rect)
        if p is None:
            return
        painter.setPen(QPen(QColor(255, 50, 50), 2))
        painter.setBrush(QBrush(QColor(255, 50, 50)))
        painter.drawEllipse(p, 8, 8)
        painter.setPen(QColor(255, 50, 50))
        painter.drawText(p + QPointF(12, -8), "GOAL")

    def _draw_path(self, painter: QPainter, draw_rect: QRectF) -> None:
        if len(self.current_path) < 2:
            return
        painter.setPen(QPen(QColor(255, 220, 0), 3))
        painter.setBrush(Qt.NoBrush)
        prev = None
        for point in self.current_path:
            p = self._world_to_widget(point[0], point[1], draw_rect)
            if p is None:
                continue
            if prev is not None:
                painter.drawLine(prev, p)
            prev = p
        if prev is not None:
            painter.setPen(QColor(255, 220, 0))
            painter.drawText(prev + QPointF(6, -6), f"path({len(self.current_path)})")

    def _draw_pose(self, painter: QPainter, draw_rect: QRectF) -> None:
        if self.pose is None:
            return
        p = self._world_to_widget(self.pose.x, self.pose.y, draw_rect)
        if p is None:
            return
        painter.setPen(QPen(QColor(120, 255, 120), 3))
        painter.setBrush(QBrush(QColor(120, 255, 120)))
        painter.drawEllipse(p, 6, 6)
        tip = QPointF(
            p.x() + 26 * math.cos(self.pose.yaw),
            p.y() - 26 * math.sin(self.pose.yaw),
        )
        painter.drawLine(p, tip)

    def _draw_cmd_arrow(self, painter: QPainter, draw_rect: QRectF) -> None:
        if self.pose is None:
            return
        p = self._world_to_widget(self.pose.x, self.pose.y, draw_rect)
        if p is None:
            return
        painter.setPen(QPen(QColor(255, 165, 0), 3))
        painter.setBrush(Qt.NoBrush)
        tip = QPointF(p.x() + 70 * self.cmd.v_x, p.y() - 70 * self.cmd.v_y)
        painter.drawLine(p, tip)
        painter.drawText(
            tip + QPointF(6, -6),
            f"vx={self.cmd.v_x:.2f} wz={self.cmd.w_z:.2f}",
        )

    def _draw_drag_preview(self, painter: QPainter, draw_rect: QRectF) -> None:
        if self._mouse_world is None:
            return
        if self.editor.tool == ToolMode.DRAW_RECT and self.editor._pending_rect_start is not None:
            start = self.editor._pending_rect_start
            p0 = self._world_to_widget(start[0], start[1], draw_rect)
            p1 = self._world_to_widget(self._mouse_world[0], self._mouse_world[1], draw_rect)
            if p0 and p1:
                painter.setPen(QPen(QColor(255, 180, 180), 2, Qt.DashLine))
                painter.setBrush(QBrush(QColor(255, 180, 180, 40)))
                painter.drawRect(QRectF(p0, p1).normalized())
        elif self.editor.tool == ToolMode.DRAW_LINE:
            if self.editor._active_line is not None and self.editor._active_line.points:
                last = self.editor._active_line.points[-1]
                p0 = self._world_to_widget(last[0], last[1], draw_rect)
            elif self.editor._active_line is None and self.editor._pending_line_start is not None:
                p0 = self._world_to_widget(self.editor._pending_line_start[0], self.editor._pending_line_start[1], draw_rect)
            else:
                p0 = None
            if p0 is not None:
                p1 = self._world_to_widget(self._mouse_world[0], self._mouse_world[1], draw_rect)
                if p1:
                    painter.setPen(QPen(QColor(255, 180, 180), 2, Qt.DashLine))
                    painter.drawLine(p0, p1)

    def _cycle_tool(self) -> None:
        """Right-click advances: LINE -> SET_GOAL -> plan callback -> execute callback."""
        idx = self._tool_cycle_idx
        if idx < len(self._TOOL_CYCLE):
            next_tool = self._TOOL_CYCLE[idx]
            self.editor.set_tool(next_tool)
            self._tool_cycle_idx = idx + 1
            if self.on_tool_cycle:
                self.on_tool_cycle(next_tool)
        elif idx == len(self._TOOL_CYCLE):
            # Cycle index == len -> trigger plan
            self._tool_cycle_idx = idx + 1
            if self.on_tool_cycle:
                self.on_tool_cycle("plan")
        elif idx == len(self._TOOL_CYCLE) + 1:
            # Cycle index == len+1 -> trigger execute
            self._tool_cycle_idx = idx + 1
            if self.on_tool_cycle:
                self.on_tool_cycle("execute")

    # --- Coordinate conversion --------------------------------------------

    def _world_to_widget(self, x: float, y: float, draw_rect: QRectF) -> Optional[QPointF]:
        cfg = self.mapper.cfg
        dx = cfg.x_max - cfg.x_min
        dy = cfg.y_max - cfg.y_min
        if dx <= 0 or dy <= 0:
            return None
        u = (x - cfg.x_min) / dx
        v = (cfg.y_max - y) / dy
        return QPointF(draw_rect.left() + u * draw_rect.width(), draw_rect.top() + v * draw_rect.height())

    def _widget_to_world(self, px: float, py: float) -> Optional[WorldPoint]:
        draw_rect = self.rect().adjusted(8, 8, -8, -8)
        if not draw_rect.contains(int(px), int(py)):
            return None
        cfg = self.mapper.cfg
        dx = cfg.x_max - cfg.x_min
        dy = cfg.y_max - cfg.y_min
        if dx <= 0 or dy <= 0:
            return None
        u = (px - draw_rect.left()) / max(draw_rect.width(), 1)
        v = (py - draw_rect.top()) / max(draw_rect.height(), 1)
        return float(cfg.x_min + u * dx), float(cfg.y_max - v * dy)

    def _event_to_world(self, event) -> Optional[WorldPoint]:
        if hasattr(event, "position"):
            pos = event.position()
        else:
            pos = event.pos()
        return self._widget_to_world(float(pos.x()), float(pos.y()))


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------

class MainWindow(QMainWindow):
    """Main ground station window with auto-start TCP server and RTSP."""

    def __init__(self, config: dict) -> None:
        super().__init__()
        self.config = config
        self.setWindowTitle("AG Ground Station")
        self.resize(1680, 980)
        self._prev_state = "idle"
        self._bev_detect_samples: list[float] = []
        self._bev_last_report = time.monotonic()

        self.bridge = UiBridge()
        self.runtime = GroundStationRuntime(config)

        bev_kwargs = dict(config.get("bev", {}))
        bev_kwargs.update({
            "x_min": float(config["map"]["x_min"]),
            "x_max": float(config["map"]["x_max"]),
            "y_min": float(config["map"]["y_min"]),
            "y_max": float(config["map"]["y_max"]),
            "ppm": int(config["map"]["ppm"]),
        })
        bev_cfg = BevConfig(**bev_kwargs)
        self.bev = BevBackend(
            rtsp_url=str(config["rtsp"]["url"]),
            fps_limit=float(config["rtsp"].get("fps_limit", 20.0)),
            process_every_n=int(config["rtsp"].get("process_every_n", 2)),
            bev_config=bev_cfg,
            on_raw_frame=lambda frame: self.bridge.rawFrame.emit(frame),
            on_bev_frame=lambda frame: self.bridge.bevFrame.emit(frame),
            on_meta=lambda meta: self.bridge.bevMeta.emit(meta),
            on_log=lambda level, text: self.bridge.logLine.emit(level, text),
        )

        self._build_ui()
        self._bind_signals()
        self._auto_start()

    def _build_ui(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        root_layout = QHBoxLayout(root)

        # Left panel: buttons fill vertical space
        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(4, 4, 4, 4)
        left_layout.setSpacing(6)

        self.line_btn = QPushButton("Line Obstacle")
        self.goal_btn = QPushButton("Set Goal")
        self.rect_btn = QPushButton("Rect Obstacle")
        self.erase_btn = QPushButton("Erase")
        self.plan_btn = QPushButton("Plan")
        self.execute_btn = QPushButton("Execute")
        self.cancel_btn = QPushButton("Cancel")
        self.clear_btn = QPushButton("Clear Obstacles")

        for btn in (self.line_btn, self.goal_btn, self.rect_btn, self.erase_btn,
                    self.plan_btn, self.execute_btn, self.cancel_btn, self.clear_btn):
            btn.setMinimumHeight(44)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        left_layout.addWidget(self.line_btn, 1)
        left_layout.addWidget(self.goal_btn, 1)
        left_layout.addWidget(self.rect_btn, 1)
        left_layout.addWidget(self.erase_btn, 1)
        left_layout.addWidget(self.plan_btn, 1)
        left_layout.addWidget(self.execute_btn, 1)
        left_layout.addWidget(self.cancel_btn, 1)
        left_layout.addWidget(self.clear_btn, 1)

        # Status labels at the bottom
        self.conn_label = QLabel("Robot: waiting...")
        self.state_label = QLabel("State: idle")
        self.pose_label = QLabel("Pose: -")
        self.cmd_label = QLabel("cmd: vx=0.000 vy=0.000 wz=0.000")
        for lbl in (self.conn_label, self.state_label, self.pose_label, self.cmd_label):
            left_layout.addWidget(lbl)

        # Center: RTSP + BEV
        self.raw_view = ImageView("RTSP")
        self.map_canvas = MapCanvas(self.runtime.mapper)

        mid = QSplitter(Qt.Vertical)
        mid.addWidget(self.raw_view)
        mid.addWidget(self.map_canvas)

        # Right: log (fixed-size rolling window, no scrollbar)
        self.log_panel = QPlainTextEdit()
        self.log_panel.setReadOnly(True)
        self.log_panel.setMaximumBlockCount(200)
        self.log_panel.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left)
        splitter.addWidget(mid)
        splitter.addWidget(self.log_panel)
        splitter.setSizes([180, 1020, 280])
        root_layout.addWidget(splitter)

    def _bind_signals(self) -> None:
        self.goal_btn.clicked.connect(lambda: self.map_canvas.editor.set_tool(ToolMode.SET_GOAL))
        self.rect_btn.clicked.connect(lambda: self.map_canvas.editor.set_tool(ToolMode.DRAW_RECT))
        self.line_btn.clicked.connect(lambda: self.map_canvas.editor.set_tool(ToolMode.DRAW_LINE))
        self.erase_btn.clicked.connect(lambda: self.map_canvas.editor.set_tool(ToolMode.ERASE))

        self.plan_btn.clicked.connect(self._on_plan)
        self.execute_btn.clicked.connect(self._on_execute)
        self.cancel_btn.clicked.connect(self._on_cancel)
        self.clear_btn.clicked.connect(self._on_clear)

        self.map_canvas.on_tool_cycle = self._on_canvas_tool_cycle

        self.bridge.logLine.connect(lambda level, text: self.log_panel.appendPlainText(f"[{level}] {text}"))
        self.bridge.rawFrame.connect(lambda frame: self.raw_view.set_frame(frame))
        self.bridge.bevFrame.connect(lambda frame: self.map_canvas.set_background(frame))
        self.bridge.bevMeta.connect(self._on_bev_meta)

    def _auto_start(self) -> None:
        self.runtime.start()
        self._log("INFO", f"TCP server listening on {self.runtime.bound_address}")
        self.bev.start()
        self._log("INFO", "RTSP reader started")

        control_hz = float(self.config.get("control", {}).get("control_hz", 20.0))
        self._tick_timer = QTimer(self)
        self._tick_timer.timeout.connect(self._on_tick)
        self._tick_timer.start(int(1000.0 / control_hz))

        self._status_timer = QTimer(self)
        self._status_timer.timeout.connect(self._poll_status)
        self._status_timer.start(200)

    def _log(self, level: str, text: str) -> None:
        self.log_panel.appendPlainText(f"[{level}] {text}")

    # --- Timer callbacks --------------------------------------------------

    def _on_tick(self) -> None:
        cmd = self.runtime.tick()
        self.map_canvas.set_pose_and_cmd(self.runtime.current_pose, cmd)
        self.cmd_label.setText(f"cmd: vx={cmd.v_x:.3f} vy={cmd.v_y:.3f} wz={cmd.w_z:.3f}")
        if self.runtime.current_pose is not None:
            p = self.runtime.current_pose
            self.pose_label.setText(f"Pose: ({p.x:.2f}, {p.y:.2f}, {math.degrees(p.yaw):.1f}\u00b0)")

    def _poll_status(self) -> None:
        connected = self.runtime.connected
        self.conn_label.setText(f"Robot: {'connected' if connected else 'waiting...'}")

        state = self.runtime.state.state
        self.state_label.setText(f"State: {state}")

        # Detect goal-reached (executing -> idle) vs cancel (handled in _on_cancel)
        if state != self._prev_state:
            self._log("INFO", f"state: {self._prev_state} -> {state}")
            if self._prev_state == "executing" and state == "idle":
                self.map_canvas.set_path([])
                self._log("INFO", "goal reached")
            self._prev_state = state

        self.map_canvas.editor.locked = not self.runtime.state.editing_allowed
        self._update_button_states(state)

    def _update_button_states(self, state: str) -> None:
        editing = state in ("idle", "planned")
        self.goal_btn.setEnabled(editing)
        self.rect_btn.setEnabled(editing)
        self.line_btn.setEnabled(editing)
        self.erase_btn.setEnabled(editing)
        self.plan_btn.setEnabled(editing and self.map_canvas.editor.goal is not None)
        self.execute_btn.setEnabled(state == "planned" and self.runtime.connected)
        self.cancel_btn.setEnabled(state == "executing")
        self.clear_btn.setEnabled(editing)

    def _on_canvas_tool_cycle(self, action) -> None:
        """Handle right-click tool cycling from canvas."""
        if action == "plan":
            self._on_plan()
        elif action == "execute":
            self._on_execute()
        elif isinstance(action, ToolMode):
            self._log("INFO", f"tool -> {action.value}")

    # --- Button callbacks -------------------------------------------------

    def _on_plan(self) -> None:
        try:
            goal = self.map_canvas.editor.goal
            if goal is None:
                raise ValueError("no goal set")
            self._log("INFO", f"planning from {self.runtime.current_pose} to {goal}")
            path = self.runtime.plan(
                goal,
                self.map_canvas.editor.rectangles,
                self.map_canvas.editor.lines,
            )
            self.map_canvas.set_path(path)
            self.map_canvas.set_occupancy(self.runtime.occupancy)
            self._log("INFO", f"planned path with {len(path)} points")
        except Exception as exc:
            self._log("ERROR", f"plan failed: {exc}")
            self._show_error("plan failed", exc)

    def _on_execute(self) -> None:
        try:
            self.runtime.start_execution()
            self._log("INFO", "execution started")
        except Exception as exc:
            self._show_error("execute failed", exc)

    def _on_cancel(self) -> None:
        self.runtime.cancel()
        self.map_canvas.set_path([])
        self._log("INFO", "execution cancelled")

    def _on_clear(self) -> None:
        self.map_canvas.editor.rectangles.clear()
        self.map_canvas.editor.lines.clear()
        self.map_canvas.clear_overlay()
        self._log("INFO", "obstacles cleared")

    def _on_bev_meta(self, meta: dict) -> None:
        mode = str(meta.get("mode", "unknown"))
        detect_ms = float(meta.get("detect_ms", 0.0))
        self._bev_detect_samples.append(detect_ms)
        now = time.monotonic()
        if now - self._bev_last_report >= 3.0:
            avg = sum(self._bev_detect_samples) / len(self._bev_detect_samples)
            fps = len(self._bev_detect_samples) / (now - self._bev_last_report)
            self._log("INFO", f"bev mode={mode} detect={avg:.1f}ms avg ({fps:.1f} fps)")
            self._bev_detect_samples.clear()
            self._bev_last_report = now

    def _show_error(self, title: str, exc: Exception) -> None:
        self._log("ERROR", f"{title}: {exc}")
        QMessageBox.critical(self, title, f"{exc}\n\n{traceback.format_exc()}")

    def closeEvent(self, event) -> None:  # noqa: N802
        try:
            self.runtime.stop()
            self.bev.stop()
        finally:
            super().closeEvent(event)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="AG ground station frontend")
    parser.add_argument("--config", default="config/ground_station.toml", help="Path to config file (TOML)")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = load_config(args.config)
    app = QApplication(sys.argv)
    window = MainWindow(config)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
