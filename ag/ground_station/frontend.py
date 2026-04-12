"""Ground station Qt frontend: RTSP/BEV view, planning, and cmd visualization."""
from __future__ import annotations

import argparse
import math
import sys
import traceback
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from ground_station.bev_backend import BevBackend, BevConfig
from ground_station.planning_backend import ObstacleRect, PlanningBackend, PlannerOverlay, load_station_config

try:
    from PySide6.QtCore import QObject, QPointF, QRectF, Qt, Signal
    from PySide6.QtGui import QColor, QBrush, QImage, QPainter, QPen
    from PySide6.QtWidgets import (
        QApplication,
        QDoubleSpinBox,
        QFormLayout,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QPlainTextEdit,
        QSplitter,
        QVBoxLayout,
        QWidget,
    )
except ImportError:  # pragma: no cover
    from PyQt5.QtCore import QObject, QPointF, QRectF, Qt, pyqtSignal as Signal
    from PyQt5.QtGui import QColor, QBrush, QImage, QPainter, QPen
    from PyQt5.QtWidgets import (
        QApplication,
        QDoubleSpinBox,
        QFormLayout,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QPlainTextEdit,
        QSplitter,
        QVBoxLayout,
        QWidget,
    )


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


class UiBridge(QObject):
    """Thread-safe signal bridge from backends to Qt widgets."""

    logLine = Signal(str, str)
    telemetry = Signal(object)
    connectionChanged = Signal(bool)
    rawFrame = Signal(object)
    bevFrame = Signal(object)
    bevMeta = Signal(object)


def ndarray_to_qimage(frame: Optional[np.ndarray]) -> Optional[QImage]:
    """Convert numpy array image into detached QImage."""
    if frame is None:
        return None
    arr = np.ascontiguousarray(frame)
    if arr.ndim == 2:
        h, w = arr.shape
        image = QImage(arr.data, w, h, w, QImage.Format_Grayscale8)
        return image.copy()
    if arr.ndim != 3:
        return None
    h, w, c = arr.shape
    if c == 3:
        rgb = arr[:, :, ::-1].copy()
        image = QImage(rgb.data, w, h, w * 3, QImage.Format_RGB888)
        return image.copy()
    if c == 4:
        image = QImage(arr.data, w, h, w * 4, QImage.Format_RGBA8888)
        return image.copy()
    return None


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
    """Interactive BEV canvas for waypoints/obstacles/path and cmd arrow."""

    overlayChanged = Signal(object)

    def __init__(self, x_min: float, x_max: float, y_min: float, y_max: float) -> None:
        super().__init__()
        self.x_min = float(x_min)
        self.x_max = float(x_max)
        self.y_min = float(y_min)
        self.y_max = float(y_max)
        self.overlay = PlannerOverlay()
        self.pose = Pose2D()
        self.cmd = VelocityCommand()
        self.mode = "waypoint"
        self._background: Optional[QImage] = None
        self._drag_start: Optional[WorldPoint] = None
        self._drag_cur: Optional[WorldPoint] = None
        self.setMinimumSize(420, 320)
        self.setMouseTracking(True)

    def set_mode(self, mode: str) -> None:
        self.mode = mode

    def set_background(self, frame: np.ndarray) -> None:
        self._background = ndarray_to_qimage(frame)
        self.update()

    def set_pose_and_cmd(self, pose: Pose2D, cmd: VelocityCommand) -> None:
        self.pose = pose
        self.cmd = cmd
        self.update()

    def set_path(self, path: List[WorldPoint]) -> None:
        self.overlay.path = list(path)
        self.overlayChanged.emit(self.overlay)
        self.update()

    def clear_annotations(self) -> None:
        self.overlay.waypoints.clear()
        self.overlay.obstacles.clear()
        self.overlay.path.clear()
        self.overlayChanged.emit(self.overlay)
        self.update()

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
        self._draw_drag_preview(painter, draw_rect)
        painter.setPen(QColor(255, 255, 255))
        painter.drawText(16, 24, f"mode={self.mode}")

    def mousePressEvent(self, event) -> None:  # noqa: N802
        if event.button() != Qt.LeftButton:
            return
        px, py = self._event_xy(event)
        world = self._widget_to_world(px, py)
        if world is None:
            return
        if self.mode == "waypoint":
            self.overlay.waypoints.append(world)
            self.overlayChanged.emit(self.overlay)
            self.update()
        elif self.mode == "obstacle":
            self._drag_start = world
            self._drag_cur = world
        elif self.mode == "erase":
            self._erase_nearest(world)
            self.overlayChanged.emit(self.overlay)
            self.update()

    def mouseMoveEvent(self, event) -> None:  # noqa: N802
        if self.mode == "obstacle" and self._drag_start is not None:
            px, py = self._event_xy(event)
            world = self._widget_to_world(px, py)
            if world is not None:
                self._drag_cur = world
                self.update()

    def mouseReleaseEvent(self, event) -> None:  # noqa: N802
        if event.button() != Qt.LeftButton:
            return
        if self.mode == "obstacle" and self._drag_start is not None and self._drag_cur is not None:
            rect = ObstacleRect(self._drag_start[0], self._drag_start[1], self._drag_cur[0], self._drag_cur[1]).normalized()
            if abs(rect.x_max - rect.x_min) > 1e-3 and abs(rect.y_max - rect.y_min) > 1e-3:
                self.overlay.obstacles.append(rect)
                self.overlayChanged.emit(self.overlay)
        self._drag_start = None
        self._drag_cur = None
        self.update()

    def _draw_grid(self, painter: QPainter, draw_rect) -> None:
        painter.setPen(QPen(QColor(80, 80, 80), 1))
        for idx in range(11):
            x = draw_rect.left() + idx * draw_rect.width() / 10
            painter.drawLine(int(x), draw_rect.top(), int(x), draw_rect.bottom())
            y = draw_rect.top() + idx * draw_rect.height() / 10
            painter.drawLine(draw_rect.left(), int(y), draw_rect.right(), int(y))

    def _draw_obstacles(self, painter: QPainter, draw_rect) -> None:
        painter.setPen(QPen(QColor(255, 120, 120), 2))
        painter.setBrush(QBrush(QColor(255, 120, 120, 70)))
        for obs in self.overlay.obstacles:
            p0 = self._world_to_widget(obs.x_min, obs.y_max, draw_rect)
            p1 = self._world_to_widget(obs.x_max, obs.y_min, draw_rect)
            if p0 is None or p1 is None:
                continue
            painter.drawRect(QRectF(p0, p1).normalized())

    def _draw_waypoints(self, painter: QPainter, draw_rect) -> None:
        painter.setPen(QPen(QColor(80, 220, 255), 2))
        painter.setBrush(QBrush(QColor(80, 220, 255)))
        for idx, point in enumerate(self.overlay.waypoints):
            p = self._world_to_widget(point[0], point[1], draw_rect)
            if p is None:
                continue
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
        painter.drawText(tip + QPointF(6, -6), f"vx={self.cmd.vx:.2f} vy={self.cmd.vy:.2f} wz={self.cmd.wz:.2f}")

    def _draw_drag_preview(self, painter: QPainter, draw_rect) -> None:
        if self._drag_start is None or self._drag_cur is None:
            return
        p0 = self._world_to_widget(self._drag_start[0], self._drag_start[1], draw_rect)
        p1 = self._world_to_widget(self._drag_cur[0], self._drag_cur[1], draw_rect)
        if p0 is None or p1 is None:
            return
        painter.setPen(QPen(QColor(255, 180, 180), 2, Qt.DashLine))
        painter.setBrush(QBrush(QColor(255, 180, 180, 40)))
        painter.drawRect(QRectF(p0, p1).normalized())

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
            rect = obs.normalized()
            if rect.x_min <= wx <= rect.x_max and rect.y_min <= wy <= rect.y_max:
                del self.overlay.obstacles[idx]
                return

    @staticmethod
    def _event_xy(event) -> Tuple[float, float]:
        if hasattr(event, "position"):
            pos = event.position()
            return float(pos.x()), float(pos.y())
        pos = event.pos()
        return float(pos.x()), float(pos.y())


class MainWindow(QMainWindow):
    """Main UI window for the new ground_station frontend."""

    def __init__(self, config: Dict[str, object]) -> None:
        super().__init__()
        self.config = config
        self.setWindowTitle("AG Ground Station")
        self.resize(1680, 980)

        self.bridge = UiBridge()
        self.pose = Pose2D()
        self.cmd = VelocityCommand()
        self.current_path: List[WorldPoint] = []

        self.planning_backend = PlanningBackend(config)
        self.planning_backend.on_log = lambda level, text: self.bridge.logLine.emit(level, text)
        self.planning_backend.on_telemetry = lambda msg: self.bridge.telemetry.emit(msg)
        self.planning_backend.on_connection = lambda ok: self.bridge.connectionChanged.emit(ok)

        bev_cfg = BevConfig(**config.get("bev", {}))
        self.bev_backend = BevBackend(
            rtsp_url=str(config.get("rtsp_url")),
            rtsp_pipeline=str(config.get("rtsp_pipeline", "")),
            fps_limit=float(config.get("video_fps_limit", 20.0)),
            process_every_n=int(config.get("bev_process_every_n", 2)),
            bev_config=bev_cfg,
            on_raw_frame=lambda frame: self.bridge.rawFrame.emit(frame),
            on_bev_frame=lambda frame: self.bridge.bevFrame.emit(frame),
            on_meta=lambda meta: self.bridge.bevMeta.emit(meta),
            on_log=lambda level, text: self.bridge.logLine.emit(level, text),
        )

        self._build_ui()
        self._bind_signals()

    def _build_ui(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        root_layout = QHBoxLayout(root)

        left = QWidget()
        left_layout = QVBoxLayout(left)

        self.host_edit = QLineEdit(str(self.config.get("sky_host", "127.0.0.1")))
        self.port_edit = QLineEdit(str(self.config.get("sky_port", 46001)))
        self.speed_spin = QDoubleSpinBox()
        self.speed_spin.setRange(0.05, 1.0)
        self.speed_spin.setSingleStep(0.05)
        self.speed_spin.setValue(float(self.config.get("target_speed", 0.2)))

        form = QFormLayout()
        form.addRow("Sky Host", self.host_edit)
        form.addRow("Sky Port", self.port_edit)
        form.addRow("Target Speed", self.speed_spin)
        left_layout.addLayout(form)

        self.connect_btn = QPushButton("Connect Sky")
        self.disconnect_btn = QPushButton("Disconnect Sky")
        self.request_btn = QPushButton("Request Telemetry")
        self.rtsp_start_btn = QPushButton("Start RTSP")
        self.rtsp_stop_btn = QPushButton("Stop RTSP")
        self.mode_waypoint_btn = QPushButton("Waypoint Mode")
        self.mode_obstacle_btn = QPushButton("Obstacle Mode")
        self.mode_erase_btn = QPushButton("Erase Mode")
        self.plan_btn = QPushButton("Plan Path")
        self.send_btn = QPushButton("Send Path")
        self.clear_btn = QPushButton("Clear")

        for btn in (
            self.connect_btn,
            self.disconnect_btn,
            self.request_btn,
            self.rtsp_start_btn,
            self.rtsp_stop_btn,
            self.mode_waypoint_btn,
            self.mode_obstacle_btn,
            self.mode_erase_btn,
            self.plan_btn,
            self.send_btn,
            self.clear_btn,
        ):
            left_layout.addWidget(btn)

        self.conn_label = QLabel("TCP: disconnected")
        self.cmd_label = QLabel("cmd: vx=0.00 vy=0.00 wz=0.00")
        self.goal_label = QLabel("goal_reached: false")
        left_layout.addWidget(self.conn_label)
        left_layout.addWidget(self.cmd_label)
        left_layout.addWidget(self.goal_label)
        left_layout.addStretch(1)

        self.raw_view = ImageView("RTSP")
        self.map_canvas = MapCanvas(
            float(self.config["map"]["x_min"]),
            float(self.config["map"]["x_max"]),
            float(self.config["map"]["y_min"]),
            float(self.config["map"]["y_max"]),
        )

        mid = QSplitter(Qt.Vertical)
        mid.addWidget(self.raw_view)
        mid.addWidget(self.map_canvas)

        self.log_panel = QPlainTextEdit()
        self.log_panel.setReadOnly(True)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left)
        splitter.addWidget(mid)
        splitter.addWidget(self.log_panel)
        splitter.setSizes([320, 980, 360])
        root_layout.addWidget(splitter)

    def _bind_signals(self) -> None:
        self.connect_btn.clicked.connect(self.on_connect)
        self.disconnect_btn.clicked.connect(self.on_disconnect)
        self.request_btn.clicked.connect(lambda: self.planning_backend.request_status())
        self.rtsp_start_btn.clicked.connect(lambda: self.bev_backend.start())
        self.rtsp_stop_btn.clicked.connect(lambda: self.bev_backend.stop())
        self.mode_waypoint_btn.clicked.connect(lambda: self.map_canvas.set_mode("waypoint"))
        self.mode_obstacle_btn.clicked.connect(lambda: self.map_canvas.set_mode("obstacle"))
        self.mode_erase_btn.clicked.connect(lambda: self.map_canvas.set_mode("erase"))
        self.plan_btn.clicked.connect(self.on_plan)
        self.send_btn.clicked.connect(self.on_send)
        self.clear_btn.clicked.connect(self.on_clear)

        self.bridge.logLine.connect(self.on_log)
        self.bridge.telemetry.connect(self.on_telemetry)
        self.bridge.connectionChanged.connect(self.on_connection_changed)
        self.bridge.rawFrame.connect(lambda frame: self.raw_view.set_frame(frame))
        self.bridge.bevFrame.connect(lambda frame: self.map_canvas.set_background(frame))
        self.bridge.bevMeta.connect(self.on_bev_meta)

    def closeEvent(self, event) -> None:  # noqa: N802
        try:
            self.planning_backend.stop()
            self.bev_backend.stop()
        finally:
            super().closeEvent(event)

    def on_connect(self) -> None:
        try:
            self.planning_backend.sky_host = self.host_edit.text().strip()
            self.planning_backend.sky_port = int(self.port_edit.text().strip())
            self.planning_backend.start()
        except Exception as exc:
            self._show_error("connect failed", exc)

    def on_disconnect(self) -> None:
        self.planning_backend.stop()

    def on_plan(self) -> None:
        try:
            start = (self.pose.x, self.pose.y)
            self.current_path = self.planning_backend.plan(
                start=start,
                goals=self.map_canvas.overlay.waypoints,
                obstacles=self.map_canvas.overlay.obstacles,
            )
            self.map_canvas.set_path(self.current_path)
            self.on_log("INFO", f"path planned with {len(self.current_path)} points")
        except Exception as exc:
            self._show_error("plan failed", exc)

    def on_send(self) -> None:
        try:
            if len(self.current_path) < 2:
                raise ValueError("no valid path to send")
            self.planning_backend.send_path(self.current_path, float(self.speed_spin.value()))
            self.on_log("INFO", f"path sent ({len(self.current_path)} points)")
        except Exception as exc:
            self._show_error("send failed", exc)

    def on_clear(self) -> None:
        self.current_path = []
        self.map_canvas.clear_annotations()

    def on_log(self, level: str, text: str) -> None:
        self.log_panel.appendPlainText(f"[{level}] {text}")

    def on_connection_changed(self, connected: bool) -> None:
        self.conn_label.setText(f"TCP: {'connected' if connected else 'disconnected'}")

    def on_telemetry(self, message: Dict[str, object]) -> None:
        pose = message.get("pose", {})
        cmd = message.get("cmd", {})
        self.pose = Pose2D(float(pose.get("x", 0.0)), float(pose.get("y", 0.0)), float(pose.get("yaw", 0.0)))
        self.cmd = VelocityCommand(float(cmd.get("vx", 0.0)), float(cmd.get("vy", 0.0)), float(cmd.get("wz", 0.0)))
        self.map_canvas.set_pose_and_cmd(self.pose, self.cmd)
        self.cmd_label.setText(f"cmd: vx={self.cmd.vx:.3f} vy={self.cmd.vy:.3f} wz={self.cmd.wz:.3f}")
        self.goal_label.setText(f"goal_reached: {str(bool(message.get('goal_reached', False))).lower()}")

    def on_bev_meta(self, meta: Dict[str, object]) -> None:
        mode = str(meta.get("mode", "unknown"))
        detect_ms = float(meta.get("detect_ms", 0.0))
        self.on_log("INFO", f"bev mode={mode} detect={detect_ms:.1f}ms")

    def _show_error(self, title: str, exc: Exception) -> None:
        self.on_log("ERROR", f"{title}: {exc}")
        QMessageBox.critical(self, title, f"{exc}\n\n{traceback.format_exc()}")


def parse_args() -> argparse.Namespace:
    """CLI args for ground station frontend."""
    parser = argparse.ArgumentParser(description="AG ground station frontend")
    parser.add_argument("--config", default="config/ground_station.json", help="Path to ground_station JSON config")
    return parser.parse_args()


def main() -> None:
    """Program entry point."""
    args = parse_args()
    config = load_station_config(args.config)
    app = QApplication(sys.argv)
    window = MainWindow(config)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
