"""Runtime state machine for the ground-station workflow."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class RuntimeStateMachine:
    state: str = "idle"

    @property
    def editing_allowed(self) -> bool:
        return self.state in {"idle", "planned"}

    def can_execute(
        self,
        has_goal: bool,
        has_path: bool,
        has_pose: bool,
        robot_connected: bool,
    ) -> bool:
        return all((has_goal, has_path, has_pose, robot_connected))

    def to_planned(self) -> None:
        self.state = "planned"

    def to_executing(self) -> None:
        self.state = "executing"

    def cancel(self) -> str:
        self.state = "idle"
        return self.state
