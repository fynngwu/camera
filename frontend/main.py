"""Camera Viewer - Qt frontend application.

High-performance camera viewer using PySide6 with:
- Low-latency GStreamer RTSP streaming
- USB camera support
- Real-time frame display
- Background frame capture thread

Usage:
    python -m frontend.main

    Or with custom RTSP URL:
    python -m frontend.main --rtsp rtsp://localhost:8554/video

    Or with USB camera:
    python -m frontend.main --usb 0
"""

import argparse
import sys
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import Qt

from .view.main_window import MainWindow
from .server.camera_worker import DEFAULT_RTSP_PIPELINE


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Qt Camera Viewer - High-performance camera display",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s                           # 使用默认RTSP流
  %(prog)s --rtsp rtsp://ip:port/v   # 自定义RTSP URL
  %(prog)s --usb 0                   # 使用USB相机
  %(prog)s --fullscreen              # 全屏模式
        """
    )

    parser.add_argument(
        "--rtsp",
        default=None,
        help="RTSP stream URL (default: rtsp://100.103.8.66:8554/video)"
    )
    parser.add_argument(
        "--usb",
        type=int,
        default=None,
        metavar="ID",
        help="USB camera device ID (e.g., 0 for /dev/video0)"
    )
    parser.add_argument(
        "--fullscreen",
        action="store_true",
        help="Start in fullscreen mode"
    )
    parser.add_argument(
        "--width",
        type=int,
        default=1600,
        help="Window width (default: 1600)"
    )
    parser.add_argument(
        "--height",
        type=int,
        default=900,
        help="Window height (default: 900)"
    )

    return parser.parse_args()


def main():
    """Main application entry point."""
    args = parse_args()

    # Create Qt application
    # Use high DPI scaling for modern displays
    QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough
    )

    app = QApplication(sys.argv)
    app.setApplicationName("Camera Viewer")
    app.setApplicationVersion("1.0.0")

    # Create main window
    window = MainWindow()
    window.resize(args.width, args.height)

    if args.fullscreen:
        window.showFullScreen()
    else:
        window.show()

    # Auto-connect if source specified
    if args.rtsp or args.usb is not None:
        # Set source type
        if args.usb is not None:
            window._source_combo.setCurrentIndex(1)  # USB camera
            window._camera_id_spin.setValue(args.usb)
        else:
            window._source_combo.setCurrentIndex(0)  # RTSP
            if args.rtsp:
                window._rtsp_url_edit.setCurrentText(args.rtsp)

        # Auto-connect
        window._connect_camera()

    # Run application
    sys.exit(app.exec())


if __name__ == "__main__":
    main()