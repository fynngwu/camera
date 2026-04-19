"""Compatibility wrapper for running the unified launcher from the camera repo."""
from __future__ import annotations

import runpy
from pathlib import Path


ROOT_LAUNCH_SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "launch_system.py"


if __name__ == "__main__":
    runpy.run_path(str(ROOT_LAUNCH_SCRIPT), run_name="__main__")
