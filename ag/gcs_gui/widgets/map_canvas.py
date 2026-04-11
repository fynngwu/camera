"""Interactive BEV canvas with waypoint / obstacle drawing."""
from __future__ import annotations

import math
from typing import List, Optional, Tuple

from gcs_gui.app_state import ObstacleRect, PlannerOverlay, Pose2D, VelocityCommand
from gcs_gui.image_utils import ndarray_to_qimage
from gcs_gui.qt_compat import QColor, QPainter, QPen, QBrush, QRectF, QPointF, Qt, QWidget, Signal


class MapCanvas(QWidget):
    """Interactive top-down map widget used for obstacle drawing and path preview."""

    overlayChanged = Signal(object)
    hoverWorldChanged = Signal(object)

    def __init__(self, x_min: float, x_max: float, y_min: float, y_max: float) -> None:
        super().__init__()
        self.x_min = float(x_min)
        self.x_max = float(x_max)
        self.y_min = float(y_min)
        self.y_max = float(y_max)
        self.overlay = PlannerOverlay()
        self._background = None
        self._mode = "waypoint"
        self._drag_start_world: Optional[Tuple[float, float]] = None
        self._drag_current_world: Optional[Tuple[float, float]] = None
        self.setMinimumSize(420, 320)
        self.setMouseTracking(True)

    def set_mode(self, mode: str) -> None:
        """Set the current mouse interaction mode."""
        self._mode = mode

    def set_background(self, frame) -> None:
        """Update BEV background image."""
        self._background = ndarray_to_qimage(frame)
        self.update()

    def set_pose(self, pose: Pose2D) -> None:
        """Update displayed robot pose."""
        self.overlay.latest_pose = pose
        self.update()

    def set_cmd(self, cmd: VelocityCommand) -> None:
        """Update displayed command arrow."""
        self.overlay.latest_cmd = cmd
        self.update()

    def set_path(self, path: List[Tuple[float, float]]) -> None:
        """Replace the currently displayed planned path."""
        self.overlay.path = list(path)
        self._emit_overlay()
        self.update()

    def clear_annotations(self) -> None:
        """Clear user waypoints, obstacles, and the planned path."""
        self.overlay.waypoints.clear()
        self.overlay.obstacles.clear()
        self.overlay.path.clear()
        self._emit_overlay()
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802
        """Render background, metric grid, annotations, path, pose, and cmd arrow."""
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
        painter.drawText(16, 24, f"Map mode: {self._mode}")
        if self.overlay.hovered_world is not None:
            x, y = self.overlay.hovered_world
            painter.drawText(16, self.height() - 12, f"world=({x:.3f}, {y:.3f})")

    def mousePressEvent(self, event) -> None:  # noqa: N802
        """Handle click / drag start for map editing."""
        if event.button() != Qt.LeftButton:
            return
        world = self._widget_to_world(event.position().x(), event.position().y())
        if world is None:
            return
        if self._mode == "waypoint":
            self.overlay.waypoints.append(world)
            self._emit_overlay()
            self.update()
        elif self._mode == "obstacle":
            self._drag_start_world = world
            self._drag_current_world = world
        elif self._mode == "erase":
            self._erase_nearest(world)
            self._emit_overlay()
            self.update()

    def mouseMoveEvent(self, event) -> None:  # noqa: N802
        """Track hover coordinate and obstacle drag preview."""
        world = self._widget_to_world(event.position().x(), event.position().y())
        self.overlay.hovered_world = world
        self.hoverWorldChanged.emit(world)
        if self._mode == "obstacle" and self._drag_start_world is not None and world is not None:
            self._drag_current_world = world
        self.update()

    def mouseReleaseEvent(self, event) -> None:  # noqa: N802
        """Finish obstacle drag and commit the rectangle."""
        if event.button() != Qt.LeftButton:
            return
        if self._mode == "obstacle" and self._drag_start_world is not None and self._drag_current_world is not None:
            x0, y0 = self._drag_start_world
            x1, y1 = self._drag_current_world
            rect = ObstacleRect(x0, y0, x1, y1).normalized()
            if abs(rect.x_max - rect.x_min) > 1e-3 and abs(rect.y_max - rect.y_min) > 1e-3:
                self.overlay.obstacles.append(rect)
                self._emit_overlay()
        self._drag_start_world = None
        self._drag_current_world = None
        self.update()

    def _draw_grid(self, painter: QPainter, draw_rect) -> None:
        """Draw a lightweight helper grid."""
        painter.setPen(QPen(QColor(90, 90, 90), 1))
        cols = 10
        rows = 10
        for idx in range(cols + 1):
            x = draw_rect.left() + idx * draw_rect.width() / cols
            painter.drawLine(int(x), draw_rect.top(), int(x), draw_rect.bottom())
        for idx in range(rows + 1):
            y = draw_rect.top() + idx * draw_rect.height() / rows
            painter.drawLine(draw_rect.left(), int(y), draw_rect.right(), int(y))

    def _draw_obstacles(self, painter: QPainter, draw_rect) -> None:
        """Draw committed rectangular obstacles."""
        painter.setPen(QPen(QColor(255, 120, 120), 2))
        painter.setBrush(QBrush(QColor(255, 120, 120, 70)))
        for obs in self.overlay.obstacles:
            left_top = self._world_to_widget(obs.x_min, obs.y_max, draw_rect)
            right_bottom = self._world_to_widget(obs.x_max, obs.y_min, draw_rect)
            if left_top is None or right_bottom is None:
                continue
            rect = QRectF(left_top, right_bottom).normalized()
            painter.drawRect(rect)

    def _draw_waypoints(self, painter: QPainter, draw_rect) -> None:
        """Draw waypoint markers and indices."""
        painter.setPen(QPen(QColor(80, 220, 255), 2))
        painter.setBrush(QBrush(QColor(80, 220, 255)))
        for idx, point in enumerate(self.overlay.waypoints):
            pos = self._world_to_widget(point[0], point[1], draw_rect)
            if pos is None:
                continue
            painter.drawEllipse(pos, 5, 5)
            painter.drawText(pos + QPointF(8, -8), str(idx))

    def _draw_path(self, painter: QPainter, draw_rect) -> None:
        """Draw the current planned path polyline."""
        if len(self.overlay.path) < 2:
            return
        painter.setPen(QPen(QColor(255, 220, 0), 3))
        previous = None
        for point in self.overlay.path:
            pos = self._world_to_widget(point[0], point[1], draw_rect)
            if pos is None:
                continue
            if previous is not None:
                painter.drawLine(previous, pos)
            previous = pos

    def _draw_pose(self, painter: QPainter, draw_rect) -> None:
        """Draw the current robot pose arrow."""
        pose = self.overlay.latest_pose
        base = self._world_to_widget(pose.x, pose.y, draw_rect)
        if base is None:
            return
        painter.setPen(QPen(QColor(120, 255, 120), 3))
        painter.setBrush(QBrush(QColor(120, 255, 120)))
        painter.drawEllipse(base, 6, 6)
        tip = QPointF(base.x() + 25 * math.cos(pose.yaw), base.y() - 25 * math.sin(pose.yaw))
        painter.drawLine(base, tip)

    def _draw_cmd_arrow(self, painter: QPainter, draw_rect) -> None:
        """Draw current velocity command as an arrow at the robot pose."""
        pose = self.overlay.latest_pose
        cmd = self.overlay.latest_cmd
        base = self._world_to_widget(pose.x, pose.y, draw_rect)
        if base is None:
            return
        painter.setPen(QPen(QColor(255, 165, 0), 3))
        dx = 60 * cmd.vx
        dy = -60 * cmd.vy
        tip = QPointF(base.x() + dx, base.y() + dy)
        painter.drawLine(base, tip)
        painter.drawText(tip + QPointF(6, -6), f"wz={cmd.wz:.2f}")

    def _draw_drag_preview(self, painter: QPainter, draw_rect) -> None:
        """Draw a preview of the obstacle rectangle while dragging."""
        if self._drag_start_world is None or self._drag_current_world is None:
            return
        p0 = self._world_to_widget(self._drag_start_world[0], self._drag_start_world[1], draw_rect)
        p1 = self._world_to_widget(self._drag_current_world[0], self._drag_current_world[1], draw_rect)
        if p0 is None or p1 is None:
            return
        painter.setPen(QPen(QColor(255, 180, 180), 2, Qt.DashLine))
        painter.setBrush(QBrush(QColor(255, 180, 180, 40)))
        painter.drawRect(QRectF(p0, p1).normalized())

    def _world_to_widget(self, x: float, y: float, draw_rect) -> Optional[QPointF]:
        """Convert a world point to widget coordinates inside draw_rect."""
        if self.x_max <= self.x_min or self.y_max <= self.y_min:
            return None
        u = (x - self.x_min) / (self.x_max - self.x_min)
        v = (self.y_max - y) / (self.y_max - self.y_min)
        px = draw_rect.left() + u * draw_rect.width()
        py = draw_rect.top() + v * draw_rect.height()
        return QPointF(px, py)

    def _widget_to_world(self, px: float, py: float) -> Optional[Tuple[float, float]]:
        """Convert a widget position to world coordinates."""
        draw_rect = self.rect().adjusted(8, 8, -8, -8)
        if not draw_rect.contains(int(px), int(py)):
            return None
        u = (px - draw_rect.left()) / max(draw_rect.width(), 1)
        v = (py - draw_rect.top()) / max(draw_rect.height(), 1)
        x = self.x_min + u * (self.x_max - self.x_min)
        y = self.y_max - v * (self.y_max - self.y_min)
        return float(x), float(y)

    def _erase_nearest(self, world: Tuple[float, float]) -> None:
        """Erase the nearest waypoint or obstacle around a click."""
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

    def _emit_overlay(self) -> None:
        """Emit the current overlay object to listeners."""
        self.overlayChanged.emit(self.overlay)
