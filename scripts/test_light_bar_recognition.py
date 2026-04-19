"""Compatibility wrapper for the root light-bar recognition test script."""

from __future__ import annotations

import runpy
import sys
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[2]
ROOT_SCRIPT = ROOT_DIR / "scripts" / "test_light_bar_recognition.py"

if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))


if __name__ == "__main__":
    runpy.run_path(str(ROOT_SCRIPT), run_name="__main__")
