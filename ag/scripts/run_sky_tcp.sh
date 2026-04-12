#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
uv run --project "$ROOT_DIR" python -m sky_uav.tcp_connector --config "${1:-config/sky_uav.json}"
