"""Small colored status badge widget."""
from __future__ import annotations

from gcs_gui.qt_compat import QLabel


class StatusBadge(QLabel):
    """A rounded badge indicating an OK / BAD / WARN style state."""

    def __init__(self, label: str) -> None:
        super().__init__(label)
        self.set_state("idle", label)

    def set_state(self, state: str, text: str | None = None) -> None:
        """Update badge state and optional text."""
        palette = {
            "ok": "#1f9d55",
            "bad": "#d64545",
            "warn": "#d9a441",
            "idle": "#666666",
        }
        color = palette.get(state, palette["idle"])
        self.setText(text or self.text())
        self.setStyleSheet(
            f"QLabel {{ background: {color}; color: white; border-radius: 8px; padding: 4px 8px; font-weight: 600; }}"
        )
