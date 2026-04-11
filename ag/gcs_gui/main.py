"""Qt GUI entry point."""
from __future__ import annotations

import argparse
import sys

from gcs_gui.app_config import load_app_config
from gcs_gui.qt_compat import QApplication
from gcs_gui.widgets.main_window import MainWindow


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments for the Qt GUI."""
    parser = argparse.ArgumentParser(description="Air-Ground Qt Suite GUI")
    parser.add_argument("--config", required=True, help="Path to GUI config JSON")
    return parser.parse_args()


def main() -> None:
    """Create the QApplication and show the main window."""
    args = parse_args()
    config = load_app_config(args.config)
    app = QApplication(sys.argv)
    window = MainWindow(config)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
