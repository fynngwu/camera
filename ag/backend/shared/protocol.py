"""Protocol helpers for the air-ground communication suite."""
from __future__ import annotations

import json
import time
from typing import Any, Dict, Iterable

ALLOWED_MODES = {"IDLE", "MANUAL_PASS", "TRACK_PATH", "E_STOP"}


def now_ts() -> float:
    """Return current UNIX timestamp in seconds."""
    return time.time()


def make_message(message_type: str, **fields: Any) -> Dict[str, Any]:
    """Build a JSON-serializable protocol message."""
    msg = {"type": message_type}
    msg.update(fields)
    return msg


def encode_message(message: Dict[str, Any]) -> bytes:
    """Serialize one message as UTF-8 JSON Lines bytes."""
    return (json.dumps(message, ensure_ascii=False, separators=(",", ":")) + "\n").encode("utf-8")


def decode_message(line: bytes) -> Dict[str, Any]:
    """Decode one JSON Lines payload into a dict."""
    obj = json.loads(line.decode("utf-8").strip())
    if not isinstance(obj, dict):
        raise ValueError("Decoded message must be a JSON object")
    if "type" not in obj:
        raise ValueError("Decoded message missing required field 'type'")
    return obj


def validate_mode(mode: str) -> None:
    """Validate a control mode string."""
    if mode not in ALLOWED_MODES:
        raise ValueError(f"Unsupported mode: {mode!r}")


def validate_cmd_vel(message: Dict[str, Any]) -> None:
    """Validate a cmd_vel message."""
    for key in ("vx", "vy", "wz"):
        if key not in message:
            raise ValueError(f"cmd_vel missing field: {key}")
        if not isinstance(message[key], (int, float)):
            raise ValueError(f"cmd_vel field {key} must be numeric")


def validate_pose2d(message: Dict[str, Any]) -> None:
    """Validate a pose2d message."""
    for key in ("x", "y", "yaw"):
        if key not in message:
            raise ValueError(f"pose2d missing field: {key}")
        if not isinstance(message[key], (int, float)):
            raise ValueError(f"pose2d field {key} must be numeric")


def validate_path(message: Dict[str, Any]) -> None:
    """Validate a path message."""
    points = message.get("points")
    if not isinstance(points, list) or len(points) < 2:
        raise ValueError("path requires at least two points")
    for point in points:
        if not isinstance(point, (list, tuple)) or len(point) != 2 or not all(isinstance(v, (int, float)) for v in point):
            raise ValueError(f"Invalid path point: {point!r}")


def clamp(value: float, minimum: float, maximum: float) -> float:
    """Clamp a scalar to [minimum, maximum]."""
    return max(minimum, min(maximum, value))


def seq_or_zero(message: Dict[str, Any]) -> int:
    """Extract a numeric sequence number or return zero."""
    value = message.get("seq", 0)
    return int(value) if isinstance(value, (int, float)) else 0


def make_ack(ack_seq: int, ok: bool = True, msg: str = "") -> Dict[str, Any]:
    """Build an ACK message."""
    return make_message("ack", ack_seq=int(ack_seq), ok=bool(ok), msg=str(msg))


def load_path_file(path_text: str) -> Dict[str, Any]:
    """Load a path message from a JSON file."""
    with open(path_text, "r", encoding="utf-8") as fp:
        obj = json.load(fp)
    if not isinstance(obj, dict):
        raise ValueError("Path file must contain a JSON object")
    obj.setdefault("type", "path")
    validate_path(obj)
    return obj


def summarize_points(points: Iterable[Iterable[float]]) -> str:
    """Return a compact text summary for a point list."""
    pts = list(points)
    if not pts:
        return "0 points"
    return f"{len(pts)} points, start={pts[0]}, end={pts[-1]}"
