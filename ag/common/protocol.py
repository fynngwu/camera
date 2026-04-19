"""Minimal JSON-lines protocol helpers for the ground-station refactor."""
from __future__ import annotations

import json
from typing import Any, Dict


def encode_cmd_vel(vx: float, vy: float, wz: float, timestamp: float) -> bytes:
    """Encode one `cmd_vel` JSON-lines message."""
    return _encode_message(
        {
            "type": "cmd_vel",
            "vx": float(vx),
            "vy": float(vy),
            "wz": float(wz),
            "timestamp": float(timestamp),
        }
    )


def encode_reach_goal(timestamp: float) -> bytes:
    """Encode one `reach_goal` JSON-lines message."""
    return _encode_message(
        {
            "type": "reach_goal",
            "timestamp": float(timestamp),
        }
    )


def decode_message(line: bytes | str) -> Dict[str, Any]:
    """Decode one JSON-lines payload."""
    if isinstance(line, bytes):
        line = line.decode("utf-8")
    payload = json.loads(line.strip())
    if not isinstance(payload, dict):
        raise ValueError("decoded payload must be a JSON object")
    if "type" not in payload:
        raise ValueError("message missing required field 'type'")
    return payload


def _encode_message(message: Dict[str, Any]) -> bytes:
    return (json.dumps(message, ensure_ascii=False, separators=(",", ":")) + "\n").encode("utf-8")
