"""Tests for the goal and obstacle editing model."""
from __future__ import annotations

import unittest

from ground_station.grid_map import LineObstacle, RectObstacle
from ground_station.map_editor import MapEditor, ToolMode


class MapEditorTests(unittest.TestCase):
    def test_set_goal_replaces_previous_goal(self) -> None:
        editor = MapEditor()
        editor.set_tool(ToolMode.SET_GOAL)
        editor.left_click((0.1, 0.1))
        editor.left_click((0.2, 0.2))
        self.assertEqual(editor.goal, (0.2, 0.2))

    def test_rect_obstacle_uses_two_clicks(self) -> None:
        editor = MapEditor()
        editor.set_tool(ToolMode.DRAW_RECT)
        editor.left_click((0.0, 0.0))
        editor.left_click((0.3, 0.2))
        self.assertEqual(len(editor.rectangles), 1)

    def test_line_obstacle_chains_segments_until_right_click(self) -> None:
        editor = MapEditor()
        editor.set_tool(ToolMode.DRAW_LINE)
        editor.left_click((0.0, 0.0))
        self.assertEqual(editor.lines, [])
        editor.left_click((0.2, 0.0))
        editor.left_click((0.2, 0.3))
        editor.right_click((0.2, 0.3))
        self.assertEqual(editor.lines[0].points, [(0.0, 0.0), (0.2, 0.0), (0.2, 0.3)])

    def test_right_click_cancels_pending_line_without_persisting(self) -> None:
        editor = MapEditor()
        editor.set_tool(ToolMode.DRAW_LINE)
        editor.left_click((0.0, 0.0))
        editor.right_click((0.0, 0.0))
        self.assertEqual(editor.lines, [])

    def test_erase_removes_goal(self) -> None:
        editor = MapEditor(goal=(0.1, 0.1))
        editor.set_tool(ToolMode.ERASE)
        editor.left_click((0.11, 0.11))
        self.assertIsNone(editor.goal)

    def test_erase_removes_rectangle(self) -> None:
        editor = MapEditor(rectangles=[RectObstacle(0.0, 0.0, 0.2, 0.2)])
        editor.set_tool(ToolMode.ERASE)
        editor.left_click((0.1, 0.1))
        self.assertEqual(editor.rectangles, [])

    def test_erase_removes_line(self) -> None:
        editor = MapEditor(lines=[LineObstacle(points=[(0.0, 0.0), (0.2, 0.0)])])
        editor.set_tool(ToolMode.ERASE)
        editor.left_click((0.1, 0.02))
        self.assertEqual(editor.lines, [])

    def test_locked_editor_ignores_input(self) -> None:
        editor = MapEditor(locked=True)
        editor.set_tool(ToolMode.SET_GOAL)
        editor.left_click((0.2, 0.2))
        self.assertIsNone(editor.goal)

    def test_switching_tools_cancels_pending_line(self) -> None:
        editor = MapEditor()
        editor.set_tool(ToolMode.DRAW_LINE)
        editor.left_click((0.0, 0.0))
        editor.set_tool(ToolMode.SET_GOAL)
        editor.set_tool(ToolMode.DRAW_LINE)
        editor.left_click((0.2, 0.0))
        self.assertEqual(editor.lines, [])


if __name__ == "__main__":
    unittest.main()
