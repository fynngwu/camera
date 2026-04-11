"""Main Qt window for the air-ground GUI."""
from __future__ import annotations

import json
import traceback
from typing import Any, Dict, List, Tuple

from gcs_gui.app_config import AppConfig
from gcs_gui.app_state import Pose2D, VelocityCommand
from gcs_gui.qt_compat import QApplication, QMainWindow, QMessageBox, QSplitter, Qt, QToolBar, QVBoxLayout, QWidget
from gcs_gui.services.bev_processor import BevProcessor, BevProcessorConfig
from gcs_gui.services.gcs_client import QtGcsClientBridge
from gcs_gui.services.planner import AStarPlanner, PlannerConfig
from gcs_gui.services.rtsp_worker import RtspWorker
from gcs_gui.widgets.control_panel import ControlPanel
from gcs_gui.widgets.image_view import ImageView
from gcs_gui.widgets.log_panel import LogPanel
from gcs_gui.widgets.map_canvas import MapCanvas
from gcs_gui.widgets.status_panel import StatusPanel


class MainWindow(QMainWindow):
    """Top-level window integrating networking, video, planning, and operator controls."""

    def __init__(self, config: AppConfig) -> None:
        super().__init__()
        self.config = config
        self.setWindowTitle("Air-Ground Qt Suite")
        self.resize(1680, 980)
        self.current_path: List[Tuple[float, float]] = []
        self.latest_bridge_status: Dict[str, Any] = {}

        self.control_panel = ControlPanel(config.rpi_host, config.rpi_port, config.rtsp_url, config.command_timeout_ms)
        self.status_panel = StatusPanel()
        self.log_panel = LogPanel()
        self.raw_view = ImageView("RTSP / Raw")
        self.map_canvas = MapCanvas(config.map.x_min, config.map.x_max, config.map.y_min, config.map.y_max)

        self._build_layout()
        self._build_toolbar()
        self._connect_ui_signals()
        self._build_services()
        self._sync_initial_pose_to_canvas()

    def _build_layout(self) -> None:
        """Create the main three-column layout."""
        center = QWidget()
        self.setCentralWidget(center)
        layout = QVBoxLayout(center)
        splitter = QSplitter(Qt.Horizontal)
        layout.addWidget(splitter)

        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.addWidget(self.control_panel)
        splitter.addWidget(left)

        middle = QSplitter(Qt.Vertical)
        middle.addWidget(self.raw_view)
        middle.addWidget(self.map_canvas)
        splitter.addWidget(middle)

        right = QSplitter(Qt.Vertical)
        right.addWidget(self.status_panel)
        right.addWidget(self.log_panel)
        splitter.addWidget(right)
        splitter.setSizes([340, 860, 420])

    def _build_toolbar(self) -> None:
        """Create a small toolbar with helpful actions."""
        toolbar = QToolBar("Quick Actions")
        self.addToolBar(toolbar)
        toolbar.addAction("Connect", self.on_connect_clicked)
        toolbar.addAction("RTSP Start", self.on_start_rtsp_clicked)
        toolbar.addAction("Plan", self.on_plan_clicked)
        toolbar.addAction("Send Path", self.on_send_path_clicked)
        toolbar.addAction("E-STOP", lambda: self.gcs_client.send_mode("E_STOP"))

    def _connect_ui_signals(self) -> None:
        """Connect widget signals to window slots."""
        cp = self.control_panel
        cp.connect_btn.clicked.connect(self.on_connect_clicked)
        cp.disconnect_btn.clicked.connect(self.on_disconnect_clicked)
        cp.start_rtsp_btn.clicked.connect(self.on_start_rtsp_clicked)
        cp.stop_rtsp_btn.clicked.connect(self.on_stop_rtsp_clicked)
        cp.status_btn.clicked.connect(self.on_request_status_clicked)
        cp.mode_idle_btn.clicked.connect(lambda: self.gcs_client.send_mode("IDLE"))
        cp.mode_manual_btn.clicked.connect(lambda: self.gcs_client.send_mode("MANUAL_PASS"))
        cp.mode_track_btn.clicked.connect(lambda: self.gcs_client.send_mode("TRACK_PATH"))
        cp.mode_estop_btn.clicked.connect(lambda: self.gcs_client.send_mode("E_STOP"))
        cp.send_cmd_btn.clicked.connect(self.on_send_cmd_clicked)
        cp.stop_cmd_btn.clicked.connect(self.on_stop_cmd_clicked)
        cp.send_pose_btn.clicked.connect(self.on_send_pose_clicked)
        cp.mode_waypoint_btn.clicked.connect(lambda: self.map_canvas.set_mode("waypoint"))
        cp.mode_obstacle_btn.clicked.connect(lambda: self.map_canvas.set_mode("obstacle"))
        cp.mode_erase_btn.clicked.connect(lambda: self.map_canvas.set_mode("erase"))
        cp.plan_btn.clicked.connect(self.on_plan_clicked)
        cp.send_path_btn.clicked.connect(self.on_send_path_clicked)
        cp.clear_btn.clicked.connect(self.on_clear_clicked)
        self.map_canvas.overlayChanged.connect(self.on_overlay_changed)
        self.map_canvas.hoverWorldChanged.connect(self.on_hover_world_changed)

    def _build_services(self) -> None:
        """Create networking and RTSP services and connect their signals."""
        self.gcs_client = QtGcsClientBridge(
            self.config.rpi_host,
            self.config.rpi_port,
            self.config.heartbeat_interval_sec,
            self.config.reconnect_interval_sec,
        )
        self.gcs_client.connectionChanged.connect(self.status_panel.set_gcs_connected)
        self.gcs_client.logLine.connect(self.log_panel.append_line)
        self.gcs_client.bridgeStatusReceived.connect(self.on_bridge_status_received)
        self.gcs_client.robotMessageReceived.connect(self.on_robot_message_received)
        self.gcs_client.eventReceived.connect(self.on_event_received)
        self.gcs_client.messageReceived.connect(self.on_message_received)

        planner_cfg = PlannerConfig(
            x_min=self.config.map.x_min,
            x_max=self.config.map.x_max,
            y_min=self.config.map.y_min,
            y_max=self.config.map.y_max,
            resolution_m=self.config.planning.resolution_m,
            robot_radius_m=self.config.planning.robot_radius_m,
        )
        self.planner = AStarPlanner(planner_cfg)

        bev_cfg = BevProcessorConfig(**self.config.bev.__dict__)
        self.bev_processor = BevProcessor(bev_cfg)
        self.rtsp_worker = RtspWorker(
            self.config.rtsp_url,
            self.config.rtsp_pipeline,
            self.config.video_fps_limit,
            self.bev_processor,
            self.config.bev_process_every_n,
        )
        self.rtsp_worker.rawFrameReady.connect(self.on_raw_frame)
        self.rtsp_worker.bevFrameReady.connect(self.on_bev_frame)
        self.rtsp_worker.connectionChanged.connect(self.status_panel.set_rtsp_connected)
        self.rtsp_worker.logLine.connect(self.log_panel.append_line)
        self.rtsp_worker.statsUpdated.connect(self.on_rtsp_stats)
        self.rtsp_worker.bevMetaUpdated.connect(self.on_bev_meta)

    def _sync_initial_pose_to_canvas(self) -> None:
        """Initialize the map pose from the pose spin boxes."""
        self.map_canvas.set_pose(
            Pose2D(
                x=self.control_panel.pose_x_spin.value(),
                y=self.control_panel.pose_y_spin.value(),
                yaw=self.control_panel.pose_yaw_spin.value(),
            )
        )

    def closeEvent(self, event) -> None:  # noqa: N802
        """Gracefully stop background workers when the window closes."""
        try:
            self.gcs_client.stop()
            self.rtsp_worker.stop()
        finally:
            super().closeEvent(event)

    def on_connect_clicked(self) -> None:
        """Apply host / port edits and start the TCP client."""
        self.gcs_client.reconfigure(self.control_panel.host_edit.text().strip(), int(self.control_panel.port_spin.value()))
        self.gcs_client.start()

    def on_disconnect_clicked(self) -> None:
        """Stop the TCP client."""
        self.gcs_client.stop()

    def on_start_rtsp_clicked(self) -> None:
        """Apply RTSP URL edits and start the RTSP worker."""
        self.rtsp_worker.url = self.control_panel.rtsp_edit.text().strip()
        self.rtsp_worker.start()

    def on_stop_rtsp_clicked(self) -> None:
        """Stop the RTSP worker."""
        self.rtsp_worker.stop()

    def on_request_status_clicked(self) -> None:
        """Request one bridge_status message."""
        self.gcs_client.request_status()

    def on_send_cmd_clicked(self) -> None:
        """Send the current manual velocity command and update the arrow overlay."""
        vx = float(self.control_panel.vx_spin.value())
        vy = float(self.control_panel.vy_spin.value())
        wz = float(self.control_panel.wz_spin.value())
        timeout_ms = int(self.control_panel.timeout_spin.value())
        self.gcs_client.send_cmd_vel(vx, vy, wz, timeout_ms)
        self.map_canvas.set_cmd(VelocityCommand(vx=vx, vy=vy, wz=wz))

    def on_stop_cmd_clicked(self) -> None:
        """Send an immediate zero command."""
        self.control_panel.vx_spin.setValue(0.0)
        self.control_panel.vy_spin.setValue(0.0)
        self.control_panel.wz_spin.setValue(0.0)
        self.on_send_cmd_clicked()

    def on_send_pose_clicked(self) -> None:
        """Send pose2d and update pose overlay."""
        pose = Pose2D(
            x=float(self.control_panel.pose_x_spin.value()),
            y=float(self.control_panel.pose_y_spin.value()),
            yaw=float(self.control_panel.pose_yaw_spin.value()),
        )
        self.gcs_client.send_pose(pose.x, pose.y, pose.yaw)
        self.map_canvas.set_pose(pose)

    def on_plan_clicked(self) -> None:
        """Run A* planning from current pose through the clicked waypoints."""
        try:
            overlay = self.map_canvas.overlay
            start = (overlay.latest_pose.x, overlay.latest_pose.y)
            self.current_path = self.planner.plan(start, overlay.waypoints, overlay.obstacles)
            self.map_canvas.set_path(self.current_path)
            self.log_panel.append_line("INFO", f"planned path with {len(self.current_path)} points")
        except Exception as exc:
            self._show_error("Plan failed", exc)

    def on_send_path_clicked(self) -> None:
        """Send the last planned path to the RPi bridge."""
        if len(self.current_path) < 2:
            self._show_error("Send path failed", ValueError("no valid planned path to send"))
            return
        self.gcs_client.send_path(self.current_path, float(self.control_panel.target_speed_spin.value()))
        self.log_panel.append_line("INFO", f"path sent to RPi: {len(self.current_path)} points")

    def on_clear_clicked(self) -> None:
        """Clear all map annotations and current path."""
        self.current_path = []
        self.map_canvas.clear_annotations()

    def on_overlay_changed(self, overlay) -> None:
        """Keep the planned path consistent with user edits."""
        if self.current_path and overlay.path != self.current_path:
            self.current_path = list(overlay.path)

    def on_hover_world_changed(self, world) -> None:
        """React to hover updates on the map canvas."""
        _ = world

    def on_raw_frame(self, frame) -> None:
        """Display the latest raw / annotated frame."""
        self.raw_view.set_frame(frame)

    def on_bev_frame(self, frame) -> None:
        """Display the latest BEV background."""
        self.map_canvas.set_background(frame)

    def on_rtsp_stats(self, stats: Dict[str, Any]) -> None:
        """Log periodic RTSP performance statistics."""
        self.raw_view._overlay_text = f"read={stats.get('read_fps', 0):.1f}fps bev={stats.get('bev_fps', 0):.1f}fps"
        self.raw_view.update()

    def on_bev_meta(self, meta: Dict[str, Any]) -> None:
        """Update BEV status badge based on processor metadata."""
        if meta.get("tag_detected"):
            text = f"tag {meta.get('tag_id', '?')} | {meta.get('detect_ms', 0):.1f}ms"
            self.status_panel.set_bev_state(True, text)
        else:
            text = f"{meta.get('mode', 'no_tag')} | {meta.get('detect_ms', 0):.1f}ms"
            self.status_panel.set_bev_state(False, text)

    def on_message_received(self, message: Dict[str, Any]) -> None:
        """Log all incoming protocol messages at a compact level."""
        mtype = message.get("type", "unknown")
        if mtype == "ack":
            self.log_panel.append_line("INFO" if message.get("ok", False) else "WARN", f"ACK seq={message.get('ack_seq')} msg={message.get('msg', '')}")

    def on_bridge_status_received(self, message: Dict[str, Any]) -> None:
        """Update the status panel from bridge_status and sync pose / cmd overlays."""
        self.latest_bridge_status = message
        self.status_panel.update_bridge_status(message)
        pose = message.get("latest_pose") or {}
        self.map_canvas.set_pose(Pose2D(pose.get("x", 0.0), pose.get("y", 0.0), pose.get("yaw", 0.0)))
        cmd = message.get("last_cmd") or {}
        self.map_canvas.set_cmd(VelocityCommand(cmd.get("vx", 0.0), cmd.get("vy", 0.0), cmd.get("wz", 0.0)))

    def on_robot_message_received(self, message: Dict[str, Any]) -> None:
        """Update raw robot-message JSON and append important robot events to log."""
        self.status_panel.update_robot_message(message)
        payload = message.get("payload") or {}
        if payload.get("type") == "event":
            self.log_panel.append_line(payload.get("level", "INFO"), f"robot: {payload.get('text', '')}")

    def on_event_received(self, message: Dict[str, Any]) -> None:
        """Append forwarded bridge / robot events to the log panel."""
        self.log_panel.append_line(message.get("level", "INFO"), f"{message.get('source', 'event')}: {message.get('text', '')}")

    def _show_error(self, title: str, exc: Exception) -> None:
        """Show an exception in both the log and a message box."""
        self.log_panel.append_line("ERROR", f"{title}: {exc}")
        QMessageBox.critical(self, title, f"{exc}\n\n{traceback.format_exc()}")
