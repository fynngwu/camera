"""Typed state objects shared inside the GUI."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple


@dataclass
class Pose2D:
    """Simple planar pose."""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass
class VelocityCommand:
    """Simple planar velocity command."""
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


@dataclass
class ObstacleRect:
    """Axis-aligned obstacle rectangle in world coordinates."""
    x_min: float
    y_min: float
    x_max: float
    y_max: float

    def normalized(self) -> "ObstacleRect":
        """Return a rect with sorted min/max bounds."""
        return ObstacleRect(
            x_min=min(self.x_min, self.x_max),
            y_min=min(self.y_min, self.y_max),
            x_max=max(self.x_min, self.x_max),
            y_max=max(self.y_min, self.y_max),
        )


@dataclass
class PlannerOverlay:
    """User annotations shown on top of the BEV map."""
    waypoints: List[Tuple[float, float]] = field(default_factory=list)
    obstacles: List[ObstacleRect] = field(default_factory=list)
    path: List[Tuple[float, float]] = field(default_factory=list)
    latest_pose: Pose2D = field(default_factory=Pose2D)
    latest_cmd: VelocityCommand = field(default_factory=VelocityCommand)
    hovered_world: Optional[Tuple[float, float]] = None
