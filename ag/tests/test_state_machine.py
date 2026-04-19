"""Tests for the runtime state machine."""
from __future__ import annotations

import unittest

from ground_station.state_machine import RuntimeStateMachine


class RuntimeStateMachineTests(unittest.TestCase):
    def test_execute_requires_goal_path_pose_and_connection(self) -> None:
        machine = RuntimeStateMachine()
        self.assertFalse(machine.can_execute(True, True, True, False))
        self.assertTrue(machine.can_execute(True, True, True, True))

    def test_executing_locks_editing(self) -> None:
        machine = RuntimeStateMachine()
        machine.to_planned()
        machine.to_executing()
        self.assertFalse(machine.editing_allowed)

    def test_cancel_returns_to_idle(self) -> None:
        machine = RuntimeStateMachine()
        machine.to_planned()
        machine.to_executing()
        self.assertEqual(machine.cancel(), "idle")


if __name__ == "__main__":
    unittest.main()
