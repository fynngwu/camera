"""Main window for camera viewer application.

Provides the primary UI layout with camera controls,
display areas, and connection settings.
"""

import numpy as np
from PySide6.QtCore import Qt, QTimer, Slot
from PySide6.QtGui import QAction, QKeySequence
from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QSpinBox,
    QGroupBox, QGridLayout, QStatusBar, QToolBar,
    QMessageBox, QSplitter, QFrame
)

from .camera_widget import CameraWidget
from ..server.camera_worker import CameraWorker, DEFAULT_RTSP_PIPELINE


class MainWindow(QMainWindow):
    """Main application window for camera viewing.

    Features:
    - Multiple camera display support
    - RTSP and USB camera connection
    - Real-time FPS monitoring
    - Connection controls
    """

    def __init__(self, parent: QWidget | None = None):
        """Initialize main window."""
        super().__init__(parent)

        self.setWindowTitle("Camera Viewer - Qt")
        self.setMinimumSize(1200, 800)
        self.resize(1600, 900)

        # Camera worker (initialized when connecting)
        self._camera_worker: CameraWorker | None = None

        # Refresh timer for UI updates
        self._refresh_timer = QTimer(self)
        self._refresh_timer.timeout.connect(self._refresh_display)

        self._setup_ui()
        self._setup_toolbar()
        self._setup_statusbar()

    def _setup_ui(self) -> None:
        """Setup the main UI layout."""
        # Central widget
        central = QWidget()
        self.setCentralWidget(central)

        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(8)

        # Top panel - Connection controls
        control_panel = self._create_control_panel()
        main_layout.addWidget(control_panel)

        # Camera display area
        display_panel = self._create_display_panel()
        main_layout.addWidget(display_panel, 1)

        # Apply dark theme
        self.setStyleSheet("""
            QMainWindow {
                background-color: #2b2b2b;
            }
            QWidget {
                background-color: #2b2b2b;
                color: #ffffff;
            }
            QLabel {
                color: #ffffff;
            }
            QPushButton {
                background-color: #404040;
                border: 1px solid #555555;
                border-radius: 4px;
                padding: 6px 12px;
                color: #ffffff;
                min-width: 80px;
            }
            QPushButton:hover {
                background-color: #505050;
            }
            QPushButton:pressed {
                background-color: #606060;
            }
            QPushButton#connectButton {
                background-color: #006600;
            }
            QPushButton#connectButton:hover {
                background-color: #008800;
            }
            QPushButton#disconnectButton {
                background-color: #660000;
            }
            QPushButton#disconnectButton:hover {
                background-color: #880000;
            }
            QComboBox, QSpinBox {
                background-color: #404040;
                border: 1px solid #555555;
                border-radius: 4px;
                padding: 4px;
                color: #ffffff;
                min-width: 120px;
            }
            QComboBox::drop-down {
                border: none;
            }
            QGroupBox {
                border: 1px solid #444444;
                border-radius: 6px;
                margin-top: 12px;
                padding-top: 10px;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
            QStatusBar {
                background-color: #333333;
            }
            QToolBar {
                background-color: #333333;
                border: none;
                spacing: 8px;
            }
        """)

    def _create_control_panel(self) -> QWidget:
        """Create connection control panel.

        Returns:
            Control panel widget
        """
        group = QGroupBox("相机连接设置")
        layout = QGridLayout(group)
        layout.setSpacing(12)

        # Source type selection
        layout.addWidget(QLabel("相机类型:"), 0, 0)
        self._source_combo = QComboBox()
        self._source_combo.addItems(["RTSP 流", "USB 相机"])
        layout.addWidget(self._source_combo, 0, 1)

        # RTSP URL input
        layout.addWidget(QLabel("RTSP URL:"), 0, 2)
        self._rtsp_url_edit = QComboBox()
        self._rtsp_url_edit.setEditable(True)
        self._rtsp_url_edit.addItem("rtsp://100.103.8.66:8554/video")
        self._rtsp_url_edit.setCurrentText("rtsp://100.103.8.66:8554/video")
        layout.addWidget(self._rtsp_url_edit, 0, 3, 1, 2)

        # USB camera ID
        layout.addWidget(QLabel("相机 ID:"), 1, 0)
        self._camera_id_spin = QSpinBox()
        self._camera_id_spin.setRange(0, 10)
        self._camera_id_spin.setValue(0)
        layout.addWidget(self._camera_id_spin, 1, 1)

        # Resolution info
        self._resolution_label = QLabel("分辨率: --")
        layout.addWidget(self._resolution_label, 1, 2)

        # FPS info
        self._fps_label = QLabel("读取 FPS: --")
        layout.addWidget(self._fps_label, 1, 3)

        # Connection buttons
        button_layout = QHBoxLayout()

        self._connect_btn = QPushButton("连接")
        self._connect_btn.setObjectName("connectButton")
        self._connect_btn.clicked.connect(self._connect_camera)
        button_layout.addWidget(self._connect_btn)

        self._disconnect_btn = QPushButton("断开")
        self._disconnect_btn.setObjectName("disconnectButton")
        self._disconnect_btn.clicked.connect(self._disconnect_camera)
        self._disconnect_btn.setEnabled(False)
        button_layout.addWidget(self._disconnect_btn)

        layout.addLayout(button_layout, 1, 4)

        # Update UI based on source type
        self._source_combo.currentIndexChanged.connect(self._update_source_ui)
        self._update_source_ui(0)

        return group

    def _create_display_panel(self) -> QWidget:
        """Create camera display panel.

        Returns:
            Display panel widget
        """
        # Use splitter for potential multi-camera layout
        splitter = QSplitter(Qt.Horizontal)

        # Main camera display
        self._main_camera = CameraWidget("主相机视图")
        splitter.addWidget(self._main_camera)

        # Set splitter sizes
        splitter.setSizes([self.width()])

        return splitter

    def _setup_toolbar(self) -> None:
        """Setup application toolbar."""
        toolbar = QToolBar("Main Toolbar")
        toolbar.setMovable(False)
        self.addToolBar(toolbar)

        # Connect action
        connect_action = QAction("连接", self)
        connect_action.setShortcut(QKeySequence("Ctrl+O"))
        connect_action.triggered.connect(self._connect_camera)
        toolbar.addAction(connect_action)

        # Disconnect action
        disconnect_action = QAction("断开", self)
        disconnect_action.setShortcut(QKeySequence("Ctrl+W"))
        disconnect_action.triggered.connect(self._disconnect_camera)
        toolbar.addAction(disconnect_action)

        toolbar.addSeparator()

        # Refresh rate control
        toolbar.addWidget(QLabel("刷新率(ms):"))
        self._refresh_spin = QSpinBox()
        self._refresh_spin.setRange(10, 1000)
        self._refresh_spin.setValue(16)  # ~60fps
        self._refresh_spin.valueChanged.connect(self._update_refresh_rate)
        toolbar.addWidget(self._refresh_spin)

    def _setup_statusbar(self) -> None:
        """Setup status bar."""
        self._statusbar = QStatusBar()
        self.setStatusBar(self._statusbar)
        self._statusbar.showMessage("就绪 - 点击连接开始")

    def _update_source_ui(self, index: int) -> None:
        """Update UI based on camera source type.

        Args:
            index: Selected source index (0=RTSP, 1=USB)
        """
        is_rtsp = index == 0

        self._rtsp_url_edit.setEnabled(is_rtsp)
        self._camera_id_spin.setEnabled(not is_rtsp)

    def _connect_camera(self) -> None:
        """Connect to camera source."""
        if self._camera_worker and self._camera_worker.is_running():
            return

        # Get source settings
        source_type = self._source_combo.currentText()
        is_rtsp = "RTSP" in source_type

        if is_rtsp:
            rtsp_url = self._rtsp_url_edit.currentText()
            self._camera_worker = CameraWorker(
                source="rtsp",
                rtsp_url=rtsp_url,
            )
        else:
            camera_id = self._camera_id_spin.value()
            self._camera_worker = CameraWorker(
                source="usb",
                camera_id=camera_id,
            )

        # Connect signals
        self._camera_worker.frame_ready.connect(self._on_frame_ready)
        self._camera_worker.fps_updated.connect(self._on_fps_updated)
        self._camera_worker.error_occurred.connect(self._on_error)
        self._camera_worker.started_capture.connect(self._on_started)

        # Start worker
        self._camera_worker.start()

        # Update UI
        self._connect_btn.setEnabled(False)
        self._disconnect_btn.setEnabled(True)
        self._main_camera.set_status("连接中...", False)
        self._statusbar.showMessage(f"正在连接: {source_type}")

    def _disconnect_camera(self) -> None:
        """Disconnect from camera."""
        if self._camera_worker:
            self._camera_worker.stop()
            self._camera_worker = None

        # Stop refresh timer
        self._refresh_timer.stop()

        # Update UI
        self._connect_btn.setEnabled(True)
        self._disconnect_btn.setEnabled(False)
        self._main_camera.clear_frame()
        self._main_camera.set_status("已断开", False)
        self._main_camera.set_fps(0)
        self._resolution_label.setText("分辨率: --")
        self._fps_label.setText("读取 FPS: --")
        self._statusbar.showMessage("已断开连接")

    @Slot(np.ndarray)
    def _on_frame_ready(self, frame: np.ndarray) -> None:
        """Handle new frame from camera.

        Args:
            frame: New frame in BGR format
        """
        # This signal is emitted from worker thread
        # Qt handles the cross-thread signal delivery
        self._main_camera.set_frame(frame)

    @Slot(float)
    def _on_fps_updated(self, fps: float) -> None:
        """Handle FPS update from camera worker.

        Args:
            fps: Current FPS
        """
        self._main_camera.set_fps(fps)
        self._fps_label.setText(f"读取 FPS: {fps:.1f}")

    @Slot(str)
    def _on_error(self, message: str) -> None:
        """Handle error from camera worker.

        Args:
            message: Error message
        """
        self._disconnect_camera()
        QMessageBox.warning(self, "相机错误", message)
        self._statusbar.showMessage(f"错误: {message}")

    @Slot()
    def _on_started(self) -> None:
        """Handle successful camera connection."""
        if self._camera_worker:
            w, h = self._camera_worker.get_frame_dimensions()
            self._resolution_label.setText(f"分辨率: {w}x{h}")

        self._main_camera.set_status("已连接", True)
        self._statusbar.showMessage("相机已连接")

        # Start refresh timer
        self._refresh_timer.start(self._refresh_spin.value())

    def _refresh_display(self) -> None:
        """Refresh display (called by timer)."""
        # This is mostly for ensuring UI responsiveness
        # The actual frames come via frame_ready signal
        pass

    def _update_refresh_rate(self, value: int) -> None:
        """Update refresh timer rate.

        Args:
            value: Timer interval in milliseconds
        """
        if self._refresh_timer.isActive():
            self._refresh_timer.setInterval(value)

    def closeEvent(self, event) -> None:
        """Handle window close - cleanup camera worker."""
        self._disconnect_camera()
        event.accept()