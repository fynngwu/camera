#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
uv run --extra gui --project "$ROOT_DIR" python -m ground_station.frontend --config "${1:-config/ground_station.json}"
