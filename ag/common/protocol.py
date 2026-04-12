"""Shared protocol helpers for AG three-end architecture."""
from __future__ import annotations

import json
import time
from typing import Any, Dict, Iterable, List, Sequence, Tuple


WorldPoint = Tuple[float, float]


def now_ts() -> float:
    """Return current UNIX timestamp."""
    return time.time()


def clamp(value: float, minimum: float, maximum: float) -> float:
    """Clamp value into [minimum, maximum]."""
    return max(minimum, min(maximum, value))


def wrap_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle > 3.141592653589793:
        angle -= 6.283185307179586
    while angle < -3.141592653589793:
        angle += 6.283185307179586
    return angle


def make_message(message_type: str, **fields: Any) -> Dict[str, Any]:
    """Build a message dictionary."""
    msg = {"type": str(message_type)}
    msg.update(fields)
    return msg


def encode_message(message: Dict[str, Any]) -> bytes:
    """Encode one message to JSON Lines bytes."""
    return (json.dumps(message, ensure_ascii=False, separators=(",", ":")) + "\n").encode("utf-8")


def decode_message(line: bytes | str) -> Dict[str, Any]:
    """Decode one JSON Lines payload."""
    if isinstance(line, bytes):
        text = line.decode("utf-8")
    else:
        text = line
    obj = json.loads(text.strip())
    if not isinstance(obj, dict):
        raise ValueError("decoded payload must be a JSON object")
    if "type" not in obj:
        raise ValueError("message missing required field 'type'")
    return obj


def _require_numeric(message: Dict[str, Any], fields: Sequence[str], message_type: str) -> None:
    for key in fields:
        if key not in message:
            raise ValueError(f"{message_type} missing field: {key}")
        if not isinstance(message[key], (int, float)):
            raise ValueError(f"{message_type} field {key} must be numeric")


def _normalize_points(points: Iterable[Iterable[float]]) -> List[WorldPoint]:
    out: List[WorldPoint] = []
    for idx, point in enumerate(points):
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            raise ValueError(f"invalid path point at {idx}: {point!r}")
        x, y = point
        if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
            raise ValueError(f"invalid path point at {idx}: {point!r}")
        out.append((float(x), float(y)))
    return out


def validate_path(message: Dict[str, Any]) -> List[WorldPoint]:
    """Validate and normalize a `path` message."""
    if message.get("type") != "path":
        raise ValueError("expected message type 'path'")
    points = message.get("points")
    if not isinstance(points, list):
        raise ValueError("path field 'points' must be a list")
    normalized = _normalize_points(points)
    if len(normalized) < 2:
        raise ValueError("path requires at least 2 points")
    target_speed = message.get("target_speed", 0.2)
    if not isinstance(target_speed, (int, float)):
        raise ValueError("path field 'target_speed' must be numeric")
    return normalized


def validate_cmd_vel(message: Dict[str, Any]) -> Dict[str, float]:
    """Validate and normalize a `cmd_vel` message."""
    if message.get("type") != "cmd_vel":
        raise ValueError("expected message type 'cmd_vel'")
    _require_numeric(message, ("vx", "vy", "wz"), "cmd_vel")
    return {
        "vx": float(message["vx"]),
        "vy": float(message["vy"]),
        "wz": float(message["wz"]),
    }


def validate_telemetry(message: Dict[str, Any]) -> None:
    """Validate required telemetry fields."""
    if message.get("type") != "telemetry":
        raise ValueError("expected message type 'telemetry'")
    pose = message.get("pose")
    cmd = message.get("cmd")
    if not isinstance(pose, dict):
        raise ValueError("telemetry field 'pose' must be an object")
    if not isinstance(cmd, dict):
        raise ValueError("telemetry field 'cmd' must be an object")
    _require_numeric(pose, ("x", "y", "yaw"), "telemetry.pose")
    _require_numeric(cmd, ("vx", "vy", "wz"), "telemetry.cmd")
    target_index = message.get("target_index")
    if not isinstance(target_index, int):
        raise ValueError("telemetry field 'target_index' must be int")
    goal_reached = message.get("goal_reached")
    if not isinstance(goal_reached, bool):
        raise ValueError("telemetry field 'goal_reached' must be bool")


def make_ack(ack_seq: int, ok: bool = True, msg: str = "") -> Dict[str, Any]:
    """Build an ACK message."""
    return make_message("ack", ack_seq=int(ack_seq), ok=bool(ok), msg=str(msg), ts=now_ts())


def make_event(level: str, text: str, source: str) -> Dict[str, Any]:
    """Build an event message."""
    return make_message("event", level=str(level), text=str(text), source=str(source), ts=now_ts())
