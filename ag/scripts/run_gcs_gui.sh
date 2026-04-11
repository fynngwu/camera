#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
CONFIG_PATH="${1:-config/gcs_gui.json}"
if [[ ! -f "$CONFIG_PATH" ]]; then
  if [[ -f config/gcs_gui.example.json ]]; then
    echo "[INFO] $CONFIG_PATH 不存在，先复制示例配置。"
    cp config/gcs_gui.example.json config/gcs_gui.json
    CONFIG_PATH="config/gcs_gui.json"
  fi
fi
uv run --extra gui --project "$ROOT_DIR" python -m gcs_gui.main --config "$CONFIG_PATH"
