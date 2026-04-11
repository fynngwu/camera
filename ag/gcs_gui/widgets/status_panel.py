"""Status panel showing connection badges and JSON summaries."""
from __future__ import annotations

import json
from typing import Any, Dict

from gcs_gui.qt_compat import QFormLayout, QGroupBox, QLabel, QPlainTextEdit, QVBoxLayout, QWidget
from gcs_gui.widgets.status_badge import StatusBadge


class StatusPanel(QWidget):
    """Display connection and latest backend status."""

    def __init__(self) -> None:
        super().__init__()
        layout = QVBoxLayout(self)
        layout.addWidget(self._build_badges_group())
        layout.addWidget(self._build_summary_group())
        layout.addWidget(self._build_json_group())
        layout.addStretch(1)

    def _build_badges_group(self) -> QGroupBox:
        """Build connection state badges."""
        box = QGroupBox("Connections")
        form = QFormLayout(box)
        self.gcs_badge = StatusBadge("GCS↔RPi")
        self.robot_badge = StatusBadge("RPi↔Robot")
        self.rtsp_badge = StatusBadge("RTSP")
        self.bev_badge = StatusBadge("BEV")
        form.addRow("GCS↔RPi", self.gcs_badge)
        form.addRow("RPi↔Robot", self.robot_badge)
        form.addRow("RTSP", self.rtsp_badge)
        form.addRow("BEV", self.bev_badge)
        return box

    def _build_summary_group(self) -> QGroupBox:
        """Build human-readable status summary labels."""
        box = QGroupBox("Summary")
        form = QFormLayout(box)
        self.mode_label = QLabel("-")
        self.goal_label = QLabel("-")
        self.cmd_label = QLabel("-")
        self.robot_exec_label = QLabel("-")
        self.camera_label = QLabel("-")
        form.addRow("Mode", self.mode_label)
        form.addRow("Goal reached", self.goal_label)
        form.addRow("Last cmd", self.cmd_label)
        form.addRow("Robot exec", self.robot_exec_label)
        form.addRow("Camera", self.camera_label)
        return box

    def _build_json_group(self) -> QGroupBox:
        """Build raw JSON viewers for bridge and robot messages."""
        box = QGroupBox("Latest JSON")
        layout = QVBoxLayout(box)
        self.bridge_json = QPlainTextEdit()
        self.bridge_json.setReadOnly(True)
        self.robot_json = QPlainTextEdit()
        self.robot_json.setReadOnly(True)
        layout.addWidget(QLabel("bridge_status"))
        layout.addWidget(self.bridge_json)
        layout.addWidget(QLabel("robot_message"))
        layout.addWidget(self.robot_json)
        return box

    def set_gcs_connected(self, connected: bool) -> None:
        """Update GCS↔RPi badge."""
        self.gcs_badge.set_state("ok" if connected else "bad", "connected" if connected else "disconnected")

    def set_robot_connected(self, connected: bool) -> None:
        """Update RPi↔Robot badge."""
        self.robot_badge.set_state("ok" if connected else "bad", "connected" if connected else "disconnected")

    def set_rtsp_connected(self, connected: bool) -> None:
        """Update RTSP badge."""
        self.rtsp_badge.set_state("ok" if connected else "bad", "streaming" if connected else "stopped")

    def set_bev_state(self, enabled: bool, text: str) -> None:
        """Update BEV badge and summary line."""
        self.bev_badge.set_state("ok" if enabled else "warn", text)
        self.camera_label.setText(text)

    def update_bridge_status(self, message: Dict[str, Any]) -> None:
        """Populate bridge-related summary fields and raw JSON view."""
        self.bridge_json.setPlainText(json.dumps(message, ensure_ascii=False, indent=2))
        self.mode_label.setText(str(message.get("mode", "-")))
        self.goal_label.setText(str(message.get("goal_reached", "-")))
        last_cmd = message.get("last_cmd", {})
        self.cmd_label.setText(f"vx={last_cmd.get('vx', 0):.2f}, vy={last_cmd.get('vy', 0):.2f}, wz={last_cmd.get('wz', 0):.2f}")
        robot = message.get("latest_robot_status") or {}
        self.robot_exec_label.setText(
            f"vx={robot.get('vx_exec', 0):.2f}, vy={robot.get('vy_exec', 0):.2f}, wz={robot.get('wz_exec', 0):.2f}"
        )
        self.set_robot_connected(bool(message.get("robot_connected", False)))

    def update_robot_message(self, message: Dict[str, Any]) -> None:
        """Update raw robot-message JSON view."""
        self.robot_json.setPlainText(json.dumps(message, ensure_ascii=False, indent=2))
