"""Import bridge for the sibling `air_ground` package in this monorepo."""
from __future__ import annotations

import sys
from pathlib import Path


def _ensure_repo_root_on_path() -> None:
    for parent in Path(__file__).resolve().parents:
        if (parent / "air_ground" / "__init__.py").exists():
            root = str(parent)
            if root not in sys.path:
                sys.path.insert(0, root)
            return
    raise ImportError("could not locate sibling air_ground package")


_ensure_repo_root_on_path()

from air_ground.geometry import normalize_angle, world_to_robot_frame  # noqa: E402
from air_ground.models import CameraCalibration, Pose2D, VelocityCommand  # noqa: E402
from air_ground.planning import PlannerConfig  # noqa: E402
from air_ground.simulator.core import SimulatorCore  # noqa: E402
from air_ground.simulator.models import (  # noqa: E402
    ObstacleRect as SimulatorObstacleRect,
    RobotLightBarConfig,
    ScenarioTag,
    SimulatorRobotConfig,
    SimulatorScenario,
    WorldBounds,
)

