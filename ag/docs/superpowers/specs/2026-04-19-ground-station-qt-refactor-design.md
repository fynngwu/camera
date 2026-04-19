# Ground Station Qt Refactor Design

## Summary

Refactor the current ground station into a single, minimal Python Qt application that owns visualization, planning, control, and robot communication. The ground station becomes the TCP server. The robot actively connects to the GCS. The camera also actively connects to the GCS through RTSP pull, with automatic retry on failure.

This refactor removes the old `GCS -> Sky -> Robot` control path. The new control path is `GCS -> Robot`. The sky side is treated as a video source only.

## Goals

- Keep the frontend as Python Qt.
- Display two live views:
  - raw FFmpeg RTSP image
  - processed BEV image
- Show current robot pose, target speed, planned path, and planning grid.
- Keep the left-side UI simple: set goal, create rectangle obstacles, create line obstacles, erase, plan, execute, cancel, clear obstacles.
- Make the GCS a TCP server that waits for the robot to connect.
- Keep the protocol minimal:
  - `cmd_vel` with `vx`, `vy`, `wz`, `timestamp`
  - `reach_goal`
- Keep all planning, pose estimation, path tracking, and control inside the GCS.
- Use one shared world/grid/pixel mapping across visualization, planning, and control.
- Use a simple A* planner and a simple pure pursuit controller.
- Use one TOML file for runtime parameters.

## Non-Goals

- No Raspberry Pi connection logic.
- No ACK, heartbeat, telemetry reply, or defensive protocol layer.
- No multi-goal planning.
- No dynamic obstacle avoidance.
- No heavy architecture or generic framework abstractions.

## Current Problems

The current behavior mixes multiple coordinate and execution paths:

- The frontend draws one path in `ground_station`.
- The old execution logic lives in `sky_uav`.
- Planning uses a grid conversion path that can shift points by cell center.
- Execution uses a separate simulator pose instead of the image-derived robot pose.

This causes visible disagreement between what is displayed and what the robot is expected to follow. The refactor fixes this by collapsing planning, path tracking, and communication into one GCS-side stack.

## Architecture

The refactored ground station has five focused parts:

1. `ground_station/frontend.py`
   - Qt window, buttons, labels, and canvas drawing.
   - No socket logic and no planner internals.

2. `ground_station/bev_backend.py`
   - FFmpeg RTSP pull.
   - BEV processing and robot pose extraction.
   - Automatic reconnect loop with log output on failures.

3. `ground_station/grid_map.py`
   - Shared coordinate mapper for:
     - world to grid
     - grid to world
     - world to pixel
     - pixel to world
   - Shared obstacle rasterization for both drawing and A* occupancy.

4. `ground_station/planner_controller.py`
   - Minimal A* planner over the shared occupancy grid.
   - Path simplification only if it preserves exact start and goal semantics.
   - Pure pursuit path tracking using the image-derived robot pose.
   - Goal reached detection.

5. `ground_station/gcs_server.py`
   - TCP server.
   - Waits for one robot client connection.
   - Sends `cmd_vel` during execution.
   - Sends `reach_goal` once when the goal is reached.

The protocol helpers stay in `common/protocol.py`, but are reduced to the two outgoing message encoders and one decoder.

## Unified Coordinates

All planning and display use one shared world frame.

- The BEV processor exposes world extents from config.
- The canvas uses the same mapper as the planner.
- Obstacle clicks are converted from widget pixels to world coordinates through the shared mapper.
- A* grid resolution is configured once in TOML and used everywhere.
- Grid overlay drawing uses the same cell geometry as occupancy generation.
- The pure pursuit controller consumes the same world-frame path that the UI displays.

This removes the previous mismatch between display coordinates, planning grid coordinates, and execution pose.

## UI Layout

The window keeps a simple structure:

- Left panel:
  - `Set Goal`
  - `Rect Obstacle`
  - `Line Obstacle`
  - `Erase`
  - `Plan`
  - `Execute`
  - `Cancel`
  - `Clear Obstacles`
  - status labels for robot connection, current pose, current command, current state
- Center:
  - raw image panel
  - BEV / map panel
- Right:
  - plain text log panel

The RTSP reader and TCP server start automatically when the window starts. There are no manual start buttons.

## Interaction Model

There is exactly one active goal.

- `Set Goal`
  - the next left click sets or replaces the goal
- `Rect Obstacle`
  - first click stores one corner
  - second click stores the opposite corner and creates a rectangle
- `Line Obstacle`
  - first left click starts a line obstacle
  - every next left click appends one more segment from the previous endpoint
  - right click ends the current continuous line obstacle
- `Erase`
  - clicking near an object removes the nearest goal, rectangle obstacle, or line obstacle segment

Obstacle visualization rules:

- rectangle obstacles draw as semi-transparent filled boxes
- line obstacles draw as thick polylines
- planning grid is always visible
- occupied cells are tinted to show the rasterized obstacle result

## State Machine

The application uses an explicit runtime state machine:

- `idle`
  - editing is allowed
  - planning is allowed
  - execution is not active
- `planned`
  - a valid path exists
  - editing and replanning are still allowed
- `executing`
  - editing is locked
  - planning is locked
  - only `Cancel` is allowed
- `cancelled`
  - transient reset state that sends zero velocity if needed, clears the active goal and active path, and returns to `idle`

Rules:

- execution requires:
  - one active goal
  - one valid current robot pose
  - one planned path
  - one connected robot socket
- while `executing`, the user cannot add obstacles, remove obstacles, change the goal, or request replanning

## Planning

Planning stays as A*, but is intentionally small:

- single occupancy grid
- 8-neighbor expansion
- one start pose from current robot world position
- one active goal
- rectangle obstacles rasterized into occupied cells
- line obstacles rasterized into occupied cells using a configured obstacle width

The output path is a world-frame polyline. The same polyline is drawn in the UI and passed to the controller.

## Control

The controller is pure pursuit.

Inputs:

- current robot pose from the GCS image pipeline
- current planned world-frame path

Outputs:

- `vx`
- `vy`
- `wz`

Behavior:

- pure pursuit selects a lookahead point on the current path
- `vx` is the forward speed command
- `vy` remains in the protocol and is sent as `0.0` in the initial implementation
- `wz` comes from the pure pursuit curvature / heading correction
- speed is reduced near the goal with a small smooth polynomial profile
- once the goal tolerance is satisfied:
  - send one zero `cmd_vel`
  - send one `reach_goal`
  - move the state machine out of `executing`

## Communication

The GCS server listens on configured host and port and waits for the robot to connect.

- no active outbound robot connection
- no ACK handling
- no extra RPCs
- only two JSON message types:

`cmd_vel`

```json
{"type":"cmd_vel","vx":0.2,"vy":0.0,"wz":0.1,"timestamp":1710000000.0}
```

`reach_goal`

```json
{"type":"reach_goal","timestamp":1710000001.0}
```

If the robot disconnects, the UI logs it and execution stops.

## Configuration

All runtime configuration moves into one file:

- `config/ground_station.toml`

Sections:

- `[rtsp]`
  - `url`
  - `retry_sec`
- `[server]`
  - `host`
  - `port`
- `[map]`
  - `x_min`
  - `x_max`
  - `y_min`
  - `y_max`
- `[grid]`
  - `resolution_m`
  - `line_obstacle_width_m`
- `[control]`
  - `lookahead_m`
  - `goal_tolerance_m`
  - `slowdown_radius_m`
  - `max_vx`
  - `max_wz`
  - `control_hz`
- `[bev]`
  - current BEV and AprilTag related parameters

## Error Handling

Keep runtime behavior simple and observable:

- RTSP open failure:
  - log the error
  - sleep `retry_sec`
  - try again
- robot disconnect:
  - log the error
  - mark robot disconnected
  - leave execution state
- missing robot pose:
  - refuse planning or execution and log a clear message

No additional defensive protocol behavior is added.

## Testing

Tests should cover the new minimum behavior:

- protocol encoding for `cmd_vel` and `reach_goal`
- shared coordinate mapper consistency:
  - world to grid to world
  - world to pixel to world
- rectangle obstacle rasterization
- continuous line obstacle rasterization
- A* on shared occupancy grid
- state machine locking rules
- pure pursuit output on a straight path
- goal reached behavior

## Migration Notes

The old `ground_station/planning_backend.py` should be split or replaced. The old sky-side path tracking path should no longer be part of the control loop for the ground station workflow.

The implementation should prefer small files with one responsibility each. The key constraint is that planning, drawing, and control must all read from the same geometry and state, not parallel copies.
