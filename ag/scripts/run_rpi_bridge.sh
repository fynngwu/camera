#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
uv run --project "$ROOT_DIR" python -m backend.rpi_bridge.bridge_main --config "${1:-config/rpi_bridge.json}"
