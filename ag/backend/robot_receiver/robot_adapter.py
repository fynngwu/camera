"""Robot adapter abstractions and a default dummy implementation."""
from __future__ import annotations

import logging
import time
from typing import Dict

LOGGER = logging.getLogger(__name__)


class BaseRobotAdapter:
    """Abstract adapter API between the receiver and the real robot backend."""

    def enable(self) -> None:
        """Enable the robot backend."""

    def disable(self) -> None:
        """Disable the robot backend."""

    def set_velocity(self, vx: float, vy: float, wz: float) -> None:
        """Apply a velocity command to the robot backend."""
        raise NotImplementedError

    def stop(self) -> None:
        """Stop the robot immediately."""
        raise NotImplementedError

    def get_status(self) -> Dict[str, float]:
        """Return JSON-serializable runtime status."""
        return {}


class DummyRobotAdapter(BaseRobotAdapter):
    """Dummy adapter used for dry-run testing before real integration."""

    def __init__(self) -> None:
        self.enabled = True
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.last_update_ts = 0.0

    def set_velocity(self, vx: float, vy: float, wz: float) -> None:
        """Store and log the latest velocity command."""
        self.vx = float(vx)
        self.vy = float(vy)
        self.wz = float(wz)
        self.last_update_ts = time.time()
        LOGGER.info("DummyRobotAdapter cmd_vel => vx=%.3f vy=%.3f wz=%.3f", self.vx, self.vy, self.wz)

    def stop(self) -> None:
        """Set all velocities to zero and log the stop action."""
        self.set_velocity(0.0, 0.0, 0.0)
        LOGGER.info("DummyRobotAdapter STOP")

    def get_status(self) -> Dict[str, float]:
        """Return the current dummy state."""
        return {
            "enabled": self.enabled,
            "vx_exec": self.vx,
            "vy_exec": self.vy,
            "wz_exec": self.wz,
            "last_update_ts": self.last_update_ts,
        }
