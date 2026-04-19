"""Ground-station TOML config loader."""
from __future__ import annotations

import copy
import tomllib
from pathlib import Path
from typing import Any, Dict


DEFAULT_CONFIG: Dict[str, Dict[str, Any]] = {
    "rtsp": {
        "url": "rtsp://10.28.215.179:8554/cam",
        "retry_sec": 1.0,
        "fps_limit": 20.0,
    },
    "server": {
        "host": "127.0.0.1",
        "port": 47001,
    },
    "map": {
        "x_min": -1.0,
        "x_max": 1.0,
        "y_min": -1.0,
        "y_max": 1.0,
        "ppm": 300,
    },
    "grid": {
        "resolution_m": 0.05,
        "line_obstacle_width_m": 0.08,
    },
    "control": {
        "control_hz": 20.0,
        "lookahead_m": 0.15,
        "goal_tolerance_m": 0.08,
        "kx": 1.5,
        "ky": 2.0,
        "kth": 3.0,
        "max_vx": 0.50,
        "max_vy": 0.50,
        "max_wz": 1.0,
        "ax_max": 1.0,
        "ay_max": 1.0,
        "aw_max": 2.0,
        "min_dist": 0.05,
        "min_angle_deg": 8.0,
    },
    "simulator": {
        "initial_pose": [0.0, 0.0, 0.0],
        "noise_xy": 0.0,
        "noise_yaw": 0.0,
    },
    "bev": {
        "enabled": True,
        "calibration_path": "config/calibration.yaml",
        "target_id": 0,
        "tag_size": 0.09,
        "aruco_dict": "DICT_4X4_50",
    },
}


def load_config(path: str | Path) -> Dict[str, Dict[str, Any]]:
    """Load one TOML file and merge it over the built-in defaults."""
    cfg = copy.deepcopy(DEFAULT_CONFIG)
    raw = tomllib.loads(Path(path).read_text(encoding="utf-8"))
    for section, values in raw.items():
        if section in cfg and isinstance(values, dict):
            cfg[section].update(values)
        else:
            cfg[section] = values
    return cfg
