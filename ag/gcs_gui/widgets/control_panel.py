"""Left-side control widget with connection, motion, and planning tools."""
from __future__ import annotations

from gcs_gui.qt_compat import (
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)


class ControlPanel(QWidget):
    """Composite widget containing all operator controls."""

    def __init__(self, host: str, port: int, rtsp_url: str, timeout_ms: int) -> None:
        super().__init__()
        layout = QVBoxLayout(self)
        layout.addWidget(self._build_connection_group(host, port, rtsp_url))
        layout.addWidget(self._build_mode_group())
        layout.addWidget(self._build_cmd_group(timeout_ms))
        layout.addWidget(self._build_pose_group())
        layout.addWidget(self._build_planner_group())
        layout.addStretch(1)

    def _build_connection_group(self, host: str, port: int, rtsp_url: str) -> QGroupBox:
        """Build connection controls."""
        box = QGroupBox("Connections")
        form = QFormLayout(box)
        self.host_edit = QLineEdit(host)
        self.port_spin = QSpinBox()
        self.port_spin.setRange(1, 65535)
        self.port_spin.setValue(int(port))
        self.rtsp_edit = QLineEdit(rtsp_url)
        self.connect_btn = QPushButton("Connect RPi")
        self.disconnect_btn = QPushButton("Disconnect")
        self.start_rtsp_btn = QPushButton("Start RTSP")
        self.stop_rtsp_btn = QPushButton("Stop RTSP")
        self.status_btn = QPushButton("Request Status")
        row1 = QHBoxLayout()
        row1.addWidget(self.connect_btn)
        row1.addWidget(self.disconnect_btn)
        row2 = QHBoxLayout()
        row2.addWidget(self.start_rtsp_btn)
        row2.addWidget(self.stop_rtsp_btn)
        form.addRow("RPi host", self.host_edit)
        form.addRow("RPi port", self.port_spin)
        form.addRow("RTSP url", self.rtsp_edit)
        form.addRow(row1)
        form.addRow(row2)
        form.addRow(self.status_btn)
        return box

    def _build_mode_group(self) -> QGroupBox:
        """Build mode buttons."""
        box = QGroupBox("Mode")
        grid = QGridLayout(box)
        self.mode_idle_btn = QPushButton("IDLE")
        self.mode_manual_btn = QPushButton("MANUAL_PASS")
        self.mode_track_btn = QPushButton("TRACK_PATH")
        self.mode_estop_btn = QPushButton("E_STOP")
        grid.addWidget(self.mode_idle_btn, 0, 0)
        grid.addWidget(self.mode_manual_btn, 0, 1)
        grid.addWidget(self.mode_track_btn, 1, 0)
        grid.addWidget(self.mode_estop_btn, 1, 1)
        return box

    def _build_cmd_group(self, timeout_ms: int) -> QGroupBox:
        """Build manual cmd_vel controls."""
        box = QGroupBox("Manual Command")
        form = QFormLayout(box)
        self.vx_spin = self._make_double_spin(-1.0, 1.0, 0.05)
        self.vy_spin = self._make_double_spin(-1.0, 1.0, 0.05)
        self.wz_spin = self._make_double_spin(-3.0, 3.0, 0.05)
        self.timeout_spin = QSpinBox()
        self.timeout_spin.setRange(50, 5000)
        self.timeout_spin.setValue(int(timeout_ms))
        self.send_cmd_btn = QPushButton("Send Cmd")
        self.stop_cmd_btn = QPushButton("Stop")
        row = QHBoxLayout()
        row.addWidget(self.send_cmd_btn)
        row.addWidget(self.stop_cmd_btn)
        form.addRow("vx", self.vx_spin)
        form.addRow("vy", self.vy_spin)
        form.addRow("wz", self.wz_spin)
        form.addRow("timeout(ms)", self.timeout_spin)
        form.addRow(row)
        return box

    def _build_pose_group(self) -> QGroupBox:
        """Build pose sending controls."""
        box = QGroupBox("Pose2D")
        form = QFormLayout(box)
        self.pose_x_spin = self._make_double_spin(-10.0, 10.0, 0.05)
        self.pose_y_spin = self._make_double_spin(-10.0, 10.0, 0.05)
        self.pose_yaw_spin = self._make_double_spin(-6.3, 6.3, 0.05)
        self.send_pose_btn = QPushButton("Send Pose")
        form.addRow("x", self.pose_x_spin)
        form.addRow("y", self.pose_y_spin)
        form.addRow("yaw", self.pose_yaw_spin)
        form.addRow(self.send_pose_btn)
        return box

    def _build_planner_group(self) -> QGroupBox:
        """Build planner / annotation controls."""
        box = QGroupBox("Planner / Annotation")
        layout = QVBoxLayout(box)
        self.target_speed_spin = self._make_double_spin(0.01, 2.0, 0.05)
        self.target_speed_spin.setValue(0.2)
        self.mode_waypoint_btn = QPushButton("Waypoint")
        self.mode_obstacle_btn = QPushButton("Obstacle")
        self.mode_erase_btn = QPushButton("Erase")
        self.plan_btn = QPushButton("Plan")
        self.send_path_btn = QPushButton("Send Path")
        self.clear_btn = QPushButton("Clear")
        layout.addWidget(QLabel("Target speed"))
        layout.addWidget(self.target_speed_spin)
        row1 = QHBoxLayout()
        row1.addWidget(self.mode_waypoint_btn)
        row1.addWidget(self.mode_obstacle_btn)
        row1.addWidget(self.mode_erase_btn)
        row2 = QHBoxLayout()
        row2.addWidget(self.plan_btn)
        row2.addWidget(self.send_path_btn)
        row2.addWidget(self.clear_btn)
        layout.addLayout(row1)
        layout.addLayout(row2)
        return box

    @staticmethod
    def _make_double_spin(minimum: float, maximum: float, step: float) -> QDoubleSpinBox:
        """Create a consistently configured QDoubleSpinBox."""
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setSingleStep(step)
        spin.setDecimals(3)
        return spin
