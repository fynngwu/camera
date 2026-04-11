"""Scrollable log display widget."""
from __future__ import annotations

import time

from gcs_gui.qt_compat import QPlainTextEdit, QVBoxLayout, QWidget


class LogPanel(QWidget):
    """Append-only text log panel."""

    def __init__(self) -> None:
        super().__init__()
        layout = QVBoxLayout(self)
        self.text_edit = QPlainTextEdit()
        self.text_edit.setReadOnly(True)
        layout.addWidget(self.text_edit)

    def append_line(self, level: str, text: str) -> None:
        """Append one timestamped line."""
        stamp = time.strftime("%H:%M:%S")
        self.text_edit.appendPlainText(f"[{stamp}] [{level}] {text}")
        scrollbar = self.text_edit.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
