"""Configuration loading for the Qt GUI."""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict


@dataclass
class MapConfig:
    """Map display and planning extents."""
    x_min: float = -1.0
    x_max: float = 1.0
    y_min: float = -1.0
    y_max: float = 1.0
    ppm: int = 300


@dataclass
class BevConfig:
    """BEV processor parameters."""
    enabled: bool = True
    calibration_path: str = "config/camera_assets/camera_calibration.npz"
    target_id: int = 0
    tag_size: float = 0.09
    x_min: float = -1.0
    x_max: float = 1.0
    y_min: float = -1.0
    y_max: float = 1.0
    ppm: int = 300


@dataclass
class PlanningConfig:
    """Planner parameters."""
    resolution_m: float = 0.05
    robot_radius_m: float = 0.10


@dataclass
class AppConfig:
    """Top-level GUI configuration."""
    rpi_host: str = "127.0.0.1"
    rpi_port: int = 46001
    heartbeat_interval_sec: float = 1.0
    reconnect_interval_sec: float = 1.0
    rtsp_url: str = "rtsp://127.0.0.1:8554/video"
    rtsp_pipeline: str = ""
    video_fps_limit: float = 20.0
    bev_process_every_n: int = 2
    command_timeout_ms: int = 400
    map: MapConfig = field(default_factory=MapConfig)
    bev: BevConfig = field(default_factory=BevConfig)
    planning: PlanningConfig = field(default_factory=PlanningConfig)


def load_app_config(path_text: str) -> AppConfig:
    """Load GUI configuration from JSON and merge with defaults."""
    raw: Dict[str, Any] = {}
    path = Path(path_text)
    if path.exists():
        raw = json.loads(path.read_text(encoding="utf-8"))
    cfg = AppConfig()
    for key in ("rpi_host", "rpi_port", "heartbeat_interval_sec", "reconnect_interval_sec", "rtsp_url", "rtsp_pipeline", "video_fps_limit", "bev_process_every_n", "command_timeout_ms"):
        if key in raw:
            setattr(cfg, key, raw[key])
    if "map" in raw:
        cfg.map = MapConfig(**{**cfg.map.__dict__, **raw["map"]})
    if "bev" in raw:
        cfg.bev = BevConfig(**{**cfg.bev.__dict__, **raw["bev"]})
    if "planning" in raw:
        cfg.planning = PlanningConfig(**{**cfg.planning.__dict__, **raw["planning"]})
    return cfg
